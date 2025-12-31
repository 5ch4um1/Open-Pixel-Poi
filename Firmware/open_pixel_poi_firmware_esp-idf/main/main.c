#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <dirent.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_random.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
//debug

#include "esp_heap_caps.h"

//FS
#include "esp_spiffs.h"

//queue
#include "freertos/queue.h"

typedef struct {
    uint8_t pkt_data[512]; // Buffer large enough for MTU
    uint16_t len;
} flash_packet_t;

QueueHandle_t flash_queue = NULL;

// NimBLE Includes
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

uint16_t notify_handle;
uint16_t conn_hdl;

uint8_t tx_buffer[512];
uint16_t tx_len = 0;

//advertising forward declaration
void start_advertising(void);
void bleSendFWVersion(void);
static const char *TAG = "Pixel Poi";

// speeds
// Predefined speed presets (delay calculations will use these)
// 0 = slowest, 255 = fastest
uint8_t g_speed_presets[6] = {50, 100, 150, 200, 230, 255}; 

// Which preset are we currently using? (0 to 5)
uint8_t g_selected_speed_index = 2; // Default to the 3rd speed

//brightness
// 6 Brightness Presets (0 to 255)
uint8_t g_brightness_presets[6] = {10, 40, 80, 140, 200, 255};
uint8_t g_selected_brightness_index = 3; // Default to a middle-high value

// This is the actual value used in your LED loop (updated via the index)
uint8_t g_brightness = 140;

//shuffle stuff
// --- SHUFFLE GLOBALS ---
bool g_shuffle_all_banks = false;
bool g_shuffle_slots_only = false;
uint32_t g_shuffle_duration_ms = 5000; // Default 5 seconds
uint32_t g_last_shuffle_tick = 0;

//Button
#define BOOT_BUTTON_PIN 9
volatile bool g_btn_is_down = false;
volatile TickType_t g_btn_transition_tick = 0;
volatile TickType_t g_last_press_tick = 0; // Added this

static void IRAM_ATTR button_isr_handler(void* arg) {
    bool current_level = gpio_get_level(BOOT_BUTTON_PIN);
    g_btn_is_down = !current_level; 
    g_btn_transition_tick = xTaskGetTickCountFromISR();
    
    if (g_btn_is_down) {
        g_last_press_tick = g_btn_transition_tick;
    }
}

// Add the prototype for the flash animation if you don't have it
void run_flash_animation(uint32_t color);

// Simplified ISR to track both Press and Release


// --- Hardware ---
#define LED_GPIO            GPIO_NUM_4
#define BUTTON_GPIO         GPIO_NUM_9
#define REGULATOR_GPIO      GPIO_NUM_7 
#define MAX_LEDS            20

//LEDs stuff
#define READ_BUFFER_SIZE 1024 
 uint32_t frame_counter = 0;
volatile bool led_task_running = true;
volatile bool led_task_paused = false; // To tell us it's safely stopped

#define BYTES_PER_FRAME (MAX_LEDS * 3)
#define FRAME_COUNT 42 
#define RING_BUF_SIZE (BYTES_PER_FRAME * FRAME_COUNT)

//streaming
uint8_t g_ring_buffer[RING_BUF_SIZE];
uint16_t g_write_idx = 0; // Where BLE writes
uint16_t g_read_idx = 0;  // Where the LED task reads
bool g_is_streaming = false;
uint64_t g_last_stream_time = 0;
bool g_is_refreshing = false;
static uint8_t g_assembly_buf[425]; // Header(1) + CMD(1) + Len(2) + Payload(420) + Footer(1)
static uint16_t g_assembly_pos = 0;

//banks & slots
volatile uint8_t g_current_bank = 1;
volatile uint8_t g_current_slot = 1;

//battery
#include "esp_adc/adc_oneshot.h"

adc_oneshot_unit_handle_t adc1_handle;
float g_battery_voltage = 0.0;
uint64_t g_last_battery_check = 0;

//FS stuff
void init_spiffs() {
    ESP_LOGI("SPIFFS", "Initializing SPIFFS...");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true // This creates the "directory" if it's blank
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("SPIFFS", "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE("SPIFFS", "Failed to find SPIFFS partition (check partitions.csv)");
        } else {
            ESP_LOGE("SPIFFS", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret == ESP_OK) {
        ESP_LOGI("SPIFFS", "Partition size: total: %d, used: %d", total, used);
    }
}

// TX response buffer and responses
static uint8_t ble_resp_buf[20]; // Size it for your largest possible response
static uint16_t ble_resp_len = 0;

static const uint8_t resp_success[] = {0xD0, 0x00, 0x01}; // Standard Success
static const uint8_t resp_error[]   = {0xD0, 0x01, 0x01}; // Standard Error 
static const uint8_t resp_firmware[]= {0xD0, 0x00, 0x06, 0x09, 0x02, 0xD1};

void set_ble_reply(const uint8_t *data, uint16_t len) {
    if (len <= sizeof(ble_resp_buf)) {
        memcpy(ble_resp_buf, data, len);
        ble_resp_len = len;
    }
}


// Save to file logic

bool g_multipart_active = false;
int current_active_slot = 0;       // Controlled by CC_SET_PATTERN_SLOT
uint8_t g_strip_height = 0;   // Extracted from the pattern data
uint16_t g_pattern_width = 0; // Extracted from the pattern data



// Janitor task
void run_storage_janitor() {
    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info("storage", &total, &used);
    if (ret != ESP_OK) return;

    // If more than 90% is used, start clearing
    if (used > (total * 0.9)) {
        ESP_LOGW("SPIFFS", "Storage near limit (%zu/%zu). Cleaning slots...", used, total);
        
        // Delete slots 10 through 50 to make a big chunk of room
        for (int i = 50; i >= 10; i--) {
            char path[32];
            snprintf(path, sizeof(path), "/spiffs/p_%d.bin", i);
            
            struct stat st;
            if (stat(path, &st) == 0) { // Check if file exists
                unlink(path);
                ESP_LOGI("JANITOR", "Deleted %s", path);
            }
        }
    }
}

// LEDS POV
uint8_t g_poi_speed = 128; // Default speed 
led_strip_handle_t led_strip;
volatile bool g_reloading_pattern = false;

// Upload animation
void run_upload_animation() {                         // --- MODE C: UPLOAD IN PROGRESS (Scanner Animation) ---

                        static int scanner_pos = 0;

                        static int direction = 1;

                        const int tail_len = 6; // Length of the glow tail


                        led_strip_clear(led_strip);


                        // Draw a fading tail for a "comet" effect

                        for (int i = 0; i < tail_len; i++) {

                            int p = scanner_pos - (i * direction);

                            if (p >= 0 && p < MAX_LEDS) {

                                // Fades out as it gets further from the head

                                uint8_t br = 150 / (i + 1); 

                                // Bright Blue for the head, dimmer for the tail

                                led_strip_set_pixel(led_strip, p, 0, br / 2, br); 

                            }

                        }


                        led_strip_refresh(led_strip);


                        // Move the scanner

                        scanner_pos += direction;

                        if (scanner_pos >= MAX_LEDS - 1 || scanner_pos <= 0) {

                            direction *= -1; // Bounce off the ends

                        }


                        // Faster refresh for the animation, but still plenty of CPU for BLE

                        vTaskDelay(pdMS_TO_TICKS(30)); 

                         

                    } 

//default pattern
void default_rainbow_pattern() {
	for (int i = 0; i < 144; i++) {

                uint8_t r = (uint8_t)(127 + 127 * sin((i + frame_counter) * 0.1));

                uint8_t g = (uint8_t)(127 + 127 * sin((i + frame_counter) * 0.1 + 2));

                uint8_t b = (uint8_t)(127 + 127 * sin((i + frame_counter) * 0.1 + 4));

                

                float scale = g_brightness / 255.0;

                led_strip_set_pixel(led_strip, i, r * scale, g * scale, b * scale);

            }

            led_strip_refresh(led_strip);

            frame_counter++;

            vTaskDelay(pdMS_TO_TICKS(20));

        }  
 
 //flash animation
 void run_flash_animation(uint32_t color) {
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    for (int i = 0; i < MAX_LEDS; i++) {
        led_strip_set_pixel(led_strip, i, r, g, b);
    }
    led_strip_refresh(led_strip);
    // Note: We don't vTaskDelay here because we are inside the task loop
}
        
// LEDS POV render task
void pov_render_task(void *pvParameters) {
    // 1. Buffers: Use 'static' to keep them off the stack
    static uint8_t read_buf[1024]; 
    static FILE* f = NULL;
    
    uint16_t buf_pos = 0;
    uint16_t bytes_in_buf = 0;
    uint8_t img_h = 0;
    uint16_t img_w = 0;
   

    while (1) {
//button 
// Static variables to persist between loop iterations
static int g_short_press_count = 0;
static bool g_in_menu_mode = false;
static TickType_t g_last_release_tick = 0;

TickType_t current_tick = xTaskGetTickCount();

// 1. Detect Long Press to enter "Menu Mode"
if (g_btn_is_down) {
    // 1. Detect Long Press
    if (!g_in_menu_mode && (current_tick - g_btn_transition_tick) > pdMS_TO_TICKS(1000)) {
        g_in_menu_mode = true;
        g_short_press_count = 0;
        ESP_LOGI("BTN", "Menu Mode Active");
        run_flash_animation(0xFFFFFF); 
    }
} else {
    // 2. Detect Short Press on Release
    if (g_btn_transition_tick > g_last_release_tick) { 
        TickType_t duration = g_btn_transition_tick - g_last_press_tick;
        // If it was a quick tap (50ms to 500ms)
        if (duration < pdMS_TO_TICKS(500) && duration > pdMS_TO_TICKS(50)) {
            g_short_press_count++;
            ESP_LOGI("BTN", "Short Press #%d", g_short_press_count);
        }
        g_last_release_tick = g_btn_transition_tick;
    }

    // 3. Action Trigger (1s after last release)
    if (g_in_menu_mode && (current_tick - g_last_release_tick) > pdMS_TO_TICKS(1000)) {
        if (g_short_press_count == 1) {
            g_is_streaming = !g_is_streaming;
            run_flash_animation(0x00FF00); // Green for stream toggle
        } else if (g_short_press_count == 2) {
            g_current_bank = (g_current_bank + 1) % 3;
            g_reloading_pattern = true;
            run_flash_animation(0x0000FF); // Blue for bank change
        } else if (g_short_press_count == 3) {
            g_brightness = 100;
            run_flash_animation(0xFFFF00); // Yellow for reset
        }
        g_in_menu_mode = false;
        g_short_press_count = 0;
    }
}

//button end
		
       //check if we should do anything at all
       if (!led_task_running) {
            led_task_paused = true;
            led_strip_clear(led_strip);
            vTaskDelay(pdMS_TO_TICKS(100)); // Sleep until upload finishes
            continue;
        }
        
        led_task_paused = false;
        // ... your existing pattern rendering code ...
        // Ensure there is a vTaskDelay() here so the WDT stays happy!
        vTaskDelay(pdMS_TO_TICKS(10));
        // --- UPLOAD GUARD ---
        if (g_multipart_active) {
            if (f) { fclose(f); f = NULL; }
            bytes_in_buf = 0; buf_pos = 0;
            // ... scanner code ...
            run_upload_animation();
            vTaskDelay(pdMS_TO_TICKS(30));
            continue;
        }
          
          //streaming
if (g_is_streaming) {
            uint64_t now = esp_timer_get_time() / 1000;

            if (now - g_last_stream_time > 2000) {
                g_is_streaming = false;
                ESP_LOGW("WATCHDOG", "Stream Timeout");
            } 
            else if (g_read_idx != g_write_idx) {
                // If streaming, we close the file if it was open
                if (f) { fclose(f); f = NULL; }

                float scale = g_brightness / 255.0f;

    for (int j = 0; j < MAX_LEDS; j++) {
        int offset = g_read_idx + (j * 3);
        
        // Apply the scale to each color channel
        uint8_t r = (uint8_t)(g_ring_buffer[offset] * scale);
        uint8_t g = (uint8_t)(g_ring_buffer[offset + 1] * scale);
        uint8_t b = (uint8_t)(g_ring_buffer[offset + 2] * scale);
        
        // Use your preferred inversion logic: MAX_LEDS - 1 - j
        led_strip_set_pixel(led_strip, MAX_LEDS - 1 - j, r, g, b);
    }
    
    
                led_strip_refresh(led_strip);
                g_read_idx = (g_read_idx + BYTES_PER_FRAME) % RING_BUF_SIZE;
            }

            // IMPORTANT: Use a fixed high-speed delay for streaming
            vTaskDelay(pdMS_TO_TICKS(5)); 
            continue; // <--- THIS IS THE KEY: Skip all the file/pattern code below
        }
    
        
   		//check battery
		static TickType_t last_battery_tick = 0;
         current_tick = xTaskGetTickCount();

if ((current_tick - last_battery_tick) > pdMS_TO_TICKS(10000)){
    int adc_raw;
    if (adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw) == ESP_OK) {
        
        // Convert raw to voltage at the PIN (approx 0-3.1V)
        float pin_volt = (adc_raw * 3.1f) / 4095.0f;
        
        // Account for the divider: V_bat = V_pin * (R1 + R2) / R2
        // For 100k and 47k: (147 / 47) = ~3.127
        g_battery_voltage = pin_volt * 3.127f;

        // Safety: If battery is too low (< 3.3V), maybe dim the LEDs?
        if (g_battery_voltage < 3.3f) {
            
        }
    }
    last_battery_tick = current_tick;
} 

					
				// --- SHUFFLE TIMER CHECK ---
				if (g_shuffle_slots_only || g_shuffle_all_banks) {
					uint32_t now = xTaskGetTickCount();
					if (pdTICKS_TO_MS(now - g_last_shuffle_tick) > g_shuffle_duration_ms) {
						
						if (g_shuffle_all_banks) {
							// Pick a random bank (assuming 0-3) and random slot (0-7)
							g_current_bank = esp_random() % 3; 
							g_current_slot = esp_random() % 4;
						} else {
							// Shuffle only slots in current bank
							g_current_slot = esp_random() % 4;
						}

						g_reloading_pattern = true; // Trigger the file-opener logic
						g_last_shuffle_tick = now;
						
					
					}
				}

        // --- FILE OPENING ---
        if (g_reloading_pattern || f == NULL) {
            if (f) { fclose(f); f = NULL; }
            char path[32];
            snprintf(path, sizeof(path), "/spiffs/b%d_s%d.bin", g_current_bank, g_current_slot);
            f = fopen(path, "rb");
            if (f) {
                uint8_t header[3];
                if (fread(header, 1, 3, f) == 3) {
                    img_h = (header[0] > MAX_LEDS) ? MAX_LEDS : header[0]; // Cap height
                    img_w = (header[1] << 8) | header[2];
                    // ADD THIS LINE to satisfy the compiler
                    (void)img_w;
                    bytes_in_buf = 0; buf_pos = 0;
                } else { fclose(f); f = NULL; }
            }
            g_reloading_pattern = false;
        }

        if (f == NULL) {
            default_rainbow_pattern(&frame_counter);
            continue;
        }

        // --- THE "SHIELDED" BUFFER MATH ---
        size_t slice_size = img_h * 3;
        if (slice_size == 0 || slice_size > 1024) {
            fseek(f, 3, SEEK_SET); bytes_in_buf = 0; buf_pos = 0;
            continue;
        }

        // Check if we need more data
        if (buf_pos + slice_size > bytes_in_buf) {
            // Safety: Ensure buf_pos hasn't gone crazy
            if (buf_pos < bytes_in_buf) {
                uint16_t remaining = bytes_in_buf - buf_pos;
                memmove(read_buf, &read_buf[buf_pos], remaining);
                bytes_in_buf = remaining;
            } else {
                bytes_in_buf = 0;
            }
            buf_pos = 0;

            // Calculate exactly how much space is left
            size_t space_left = 1024 - bytes_in_buf;
            if (space_left > 0) {
                size_t n = fread(&read_buf[bytes_in_buf], 1, space_left, f);
                if (n == 0) { // EOF
                    fseek(f, 3, SEEK_SET);
                    // Don't reset bytes_in_buf here yet, let it finish the current buffer
                } else {
                    bytes_in_buf += n;
                }
            }
        }

// --- FINAL BOUNDARY CHECK BEFORE DISPLAY ---
        if (buf_pos + slice_size <= bytes_in_buf) {
            uint8_t *slice = &read_buf[buf_pos];
            float scale = g_brightness / 255.0;

            // Use MAX_LEDS (your physical strip length) to drive the loop
            for (int j = 0; j < MAX_LEDS; j++) {
                // The modulo operator (%) tiles the image if physical LEDs > image height
                // If img_h is 1, j % img_h is always 0 (perfect for 1-pixel height)
                int pixel_index = (j % img_h) * 3;

                uint8_t r = (uint8_t)(slice[pixel_index + 0] * scale);
                uint8_t g = (uint8_t)(slice[pixel_index + 1] * scale);
                uint8_t b = (uint8_t)(slice[pixel_index + 2] * scale);

                // Setting pixels from the end (MAX_LEDS - 1 - j) to invert the display
                // as per your Arduino logic requirement.
                led_strip_set_pixel(led_strip, MAX_LEDS - 1 - j, r, g, b);
            }

            led_strip_refresh(led_strip);
            buf_pos += slice_size;
        }
        uint8_t current_raw_speed = g_speed_presets[g_selected_speed_index];
        int delay_ms = (255 - current_raw_speed) / 5;
        
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}


// advertising cleanup
static int ble_gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI("BLE", "Connected! Status: %d", event->connect.status);
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGW("BLE", "Disconnected! Reason: %d. Restarting...", event->disconnect.reason);
            
            // Safety: Reset multipart flag so we don't stay in "Loading" state
            g_multipart_active = false;
            
            // AUTO-RECOVERY: Start advertising again
            start_advertising(); 
            return 0;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI("BLE", "MTU update: %d", event->mtu.value);
            return 0;

        case BLE_GAP_EVENT_CONN_UPDATE:
            ESP_LOGI("BLE", "Connection updated");
            return 0;
    }
    return 0;
}


// --- Advertising Logic ---
void start_advertising(void) {
    struct ble_hs_adv_fields fields;
    int rc;

    // --- 1. Main Advertising Packet ---
    memset(&fields, 0, sizeof(fields));
    const char *name = "Pixel Poi"; 
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE("BLE", "Error setting main adv fields; rc=%d", rc);
        return;
    }

    // --- 2. Scan Response Packet ---
    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    rsp_fields.uuids128 = (ble_uuid128_t[]){
        BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E)
    };
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE("BLE", "Error setting scan response fields; rc=%d", rc);
        return;
    }

    // --- 3. Advertising Parameters ---
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    // --- FIXED: REGISTER THE HANDLER HERE ---
    // We replace NULL with ble_gap_event_handler
    rc = ble_gap_adv_start(BLE_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, 
                           &adv_params, ble_gap_event_handler, NULL);
    
    if (rc != 0) {
        ESP_LOGE("BLE", "Error starting advertisement; rc=%d", rc);
    } else {
        ESP_LOGI("BLE", "Advertising successfully started with Event Handler!");
    }
}

// --- Sync Callback: Triggers when BLE is actually ready ---
static void on_sync(void) {
    uint8_t addr_type;
    // Determine address type automatically
    ESP_ERROR_CHECK(ble_hs_id_infer_auto(0, &addr_type));
    start_advertising();
}

// Define the success response: [Start, SuccessCode, End]
// CC_SUCCESS is usually 0 in most enums, check your CommCode.dart

typedef enum {
    CC_SUCCESS = 0,
    CC_ERROR = 1,
    CC_SET_BRIGHTNESS = 2,
    CC_SET_SPEED = 3,
    CC_SET_PATTERN = 4,
    CC_SET_PATTERN_SLOT = 5,
    CC_SET_PATTERN_ALL = 6,
    CC_SET_BANK = 7,
    CC_SET_BANK_ALL = 8,
    CC_GET_FW_VERSION = 9,
    CC_SET_HARDWARE_VERSION = 10,
    CC_SET_LED_TYPE = 11,
    CC_SET_LED_COUNT = 12,
    CC_SET_DEVICE_NAME = 13,
    CC_SET_SEQUENCER = 14,
    CC_START_SEQUENCER = 15,
    CC_SET_BRIGHTNESS_OPTION = 16,
    CC_SET_BRIGHTNESS_OPTIONS = 17,
    CC_SET_SPEED_OPTION = 18,
    CC_SET_SPEED_OPTIONS = 19,
    CC_SET_PATTERN_SHUFFLE_DURATION = 20,
    CC_START_STREAM = 21,
    CC_STOP_STREAM = 22
} poi_comm_code_t;

// multi part upload
// --- HELPER FUNCTIONS (Place these above gatt_svr_cb) ---
void storage_worker_task(void *pvParameters) {
    flash_packet_t pkt;
    FILE* f = NULL;

    while (1) {
        if (xQueueReceive(flash_queue, &pkt, portMAX_DELAY)) {
            char path[32];
            snprintf(path, sizeof(path), "/spiffs/b%d_s%d.bin", g_current_bank, g_current_slot);

            // If f is NULL, this is the very first packet of a new upload
            if (f == NULL) {
                unlink(path); // Clear space
                f = fopen(path, "wb");
                ESP_LOGI("STORAGE", "Opening new file: %s", path);
            }

            if (f) {
                // Check if this packet contains the terminator 0xD1
                bool is_last = (pkt.pkt_data[pkt.len - 1] == 0xD1);
                size_t write_len = is_last ? pkt.len - 1 : pkt.len;

                fwrite(pkt.pkt_data, 1, write_len, f);

                if (is_last) {
					
                    fflush(f);
                    fsync(fileno(f));
                    fclose(f);
                    f = NULL; // Reset for next time
                    
                    vTaskDelay(pdMS_TO_TICKS(100)); // Flash cool-down
                    
                    g_multipart_active = false;
                    led_task_running = true;
					ESP_LOGI("BLE", "LED Task Resumed");
                    
                    
                    g_reloading_pattern = true;     // Tell LED task to load new file
                    ESP_LOGI("STORAGE", "Upload finished and saved.");
                }
            }
        }
    }
}

//helper function
void setTxCharacteristicValue(uint8_t* data, uint16_t len) {
    if (len > sizeof(tx_buffer)) len = sizeof(tx_buffer);
    
    // Copy the actual bytes
    memcpy(tx_buffer, data, len);
    tx_len = len;
    
    // Log the actual hex bytes to the console so you can see them 
    // BEFORE the phone reads them.
    ESP_LOG_BUFFER_HEX("BLE_TX_DEBUG", tx_buffer, tx_len);
}


// --- GATT Callback ---
static int gatt_svr_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    conn_hdl = conn_handle;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    if (ble_resp_len > 0) {
        return os_mbuf_append(ctxt->om, ble_resp_buf, ble_resp_len);
    }
    return 0; // Or return an error code if you prefer
}

//some debug
if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC) {
    uint16_t dsc_val = 0;
    if (OS_MBUF_PKTLEN(ctxt->om) >= 2) {
        os_mbuf_copydata(ctxt->om, 0, 2, &dsc_val);
        if (dsc_val == 0x0001) {
            ESP_LOGW("BLE_DEBUG", ">>> Phone just ENABLED notifications!");
        } else if (dsc_val == 0x0000) {
            ESP_LOGW("BLE_DEBUG", ">>> Phone just DISABLED notifications!");
        }
    }
}
//debug end



    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t data[512]; 
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len > sizeof(data)) len = sizeof(data);
        os_mbuf_copydata(ctxt->om, 0, len, data);

        uint8_t *payload = &data[2];
        int payload_len = (len > 2) ? (len - 2) : 0;



						// --- COMMAND HANDLING (0xD0) ---
						if (len >= 2 && data[0] == 0xD0) {
							poi_comm_code_t code = (poi_comm_code_t)data[1];

							// Handle standard commands
							switch (code) {
								case CC_SET_BRIGHTNESS:
									ESP_LOG_BUFFER_HEX("BLE_SPEED_RAW", data, len);
									if (payload_len >= 1) g_brightness = payload[0];
									ESP_LOGI("BLE", "Brightness request");
									break;
									
								case CC_SET_BRIGHTNESS_OPTION:
									ESP_LOGI("BLE_BRIGHT", "Brightness option request");
									ESP_LOG_BUFFER_HEX("BLE_BRIGHT_RAW", data, len);

									// Using the same 4-byte structure: [Start][Cmd][Index][End]
									if (len >= 3) {
										uint8_t index = data[2]; 

										if (index < 6) {
											g_selected_brightness_index = index;
											// Update the actual brightness variable used in the LED loop
											g_brightness = g_brightness_presets[g_selected_brightness_index];

											ESP_LOGW("BLE_BRIGHT", ">>> Gear %d selected. Real Brightness: %d/255", 
													 index, g_brightness);
										} else {
											ESP_LOGE("BLE_BRIGHT", ">>> ERROR: Index %d out of bounds", index);
										}
									}
									break;

								case CC_START_STREAM: 
    // 1. Detect Frame Start
    if (len > 0 && data[0] == 0xD0) {
        g_assembly_pos = 0; 
    }

    // 2. Accumulate Fragments
    if (g_assembly_pos + len <= sizeof(g_assembly_buf)) {
        memcpy(&g_assembly_buf[g_assembly_pos], data, len);
        g_assembly_pos += len;
    }

    // 3. Process Only When Full Frame is Reassembled
    // We expect 425 bytes: [D0][0D][LenH][LenL][420 Bytes][D1]
    if (g_assembly_pos >= 425) {
        // Validate Footer to ensure memory alignment is correct
        if (g_assembly_buf[424] == 0xD1) {
            
            g_is_streaming = true;
            g_last_stream_time = esp_timer_get_time() / 1000;

            // Copy only the 420 payload bytes (skipping D0, 0D, LenH, LenL)
            memcpy(&g_ring_buffer[g_write_idx], &g_assembly_buf[4], 420);

            // Move the write pointer for the render task
            g_write_idx = (g_write_idx + 420) % RING_BUF_SIZE;
        } else {
            ESP_LOGE("STREAM", "Frame Corrupted: Missing D1 Footer");
        }
        
        g_assembly_pos = 0; // Reset for next frame
    }
    break;
										
										case CC_STOP_STREAM:
										g_is_streaming = false;
										
										// Reset Ring Buffer pointers so the next stream starts fresh
										g_read_idx = 0;
										g_write_idx = 0;

										// Inline blackout: Immediate visual feedback that stream has ended
										for(int i = 0; i < MAX_LEDS; i++) {
											led_strip_set_pixel(led_strip, i, 0, 0, 0);
										}
										led_strip_refresh(led_strip);

										ESP_LOGI("BLE", "Stream Stopped & Buffer Reset");
										break;
								
								case CC_SET_SPEED_OPTION:
									ESP_LOGI("BLE", "Speed option request");
									ESP_LOG_BUFFER_HEX("BLE_SPEED_RAW", data, len);
									if (len >= 2) {
									uint8_t index = data[2]; 
									if (index < 6) {
										g_selected_speed_index = index;
										ESP_LOGI("BLE", "Switched to Speed Gear: %d (Value: %d)", 
												  g_selected_speed_index, g_speed_presets[g_selected_speed_index]);
									    }
								    }
									break;								
																
								case CC_SET_SPEED_OPTIONS:
									ESP_LOGI("BLE", "Speed bank request");
									// If the phone sends 6 bytes, we update the whole gearbox
											if (len >= 7) { 
												for(int i = 0; i < 6; i++) {
													g_speed_presets[i] = data[i+1];
												}
												ESP_LOGI("BLE", "Speed Gearbox Reprogrammed!");
										
 									}
									break;
								case CC_SET_SPEED:

												ESP_LOGI("BLE", "Set speed reqwuest received");
				
									break;

									
								case CC_SET_BANK:
									if (len >= 3) g_current_bank = data[2];
									break;
									
								case CC_SET_SEQUENCER:
									ESP_LOG_BUFFER_HEX("BLE_SPEED_RAW", data, len);
									break;
									
								case CC_START_SEQUENCER:
									ESP_LOG_BUFFER_HEX("BLE_SPEED_RAW", data, len);
									break;
									
										
								case CC_SET_PATTERN_SLOT:
									if (len >= 3) {
										g_current_slot = data[2];
										g_reloading_pattern = true; // Trigger LED reload
									}
									break;

								case CC_SET_PATTERN:
									ESP_LOGI("BLE", "Pattern Upload Started - Sending to Queue");
									g_multipart_active = true;
									led_task_running = false;
									while(!led_task_paused) { vTaskDelay(1); } 
									ESP_LOGI("BLE", "LED Task safely parked for upload");
									// Optional: Turn off LEDs so they don't stay stuck on one color
									led_strip_clear(led_strip);
									
									// SHIP TO QUEUE
									flash_packet_t pkt;
									pkt.len = payload_len;
									memcpy(pkt.pkt_data, payload, payload_len);
									xQueueSend(flash_queue, &pkt, portMAX_DELAY);
									break;
								case CC_GET_FW_VERSION:
									ESP_LOGI("BLE", "Firmware version requested, sending direct response...");
									//uint8_t response[] = {0xD0, 0x00, 0x06, 0x09, 0x02, 0xD1};

									set_ble_reply(resp_firmware, sizeof(resp_firmware));
									//setTxCharacteristicValue(response, 6);
									
									
										break;

								case CC_SET_PATTERN_ALL: // Shuffle all slots in CURRENT bank (d0 06 d1)
									ESP_LOGI("SHUFFLE", "Toggle Bank Shuffle Requested");
									
									// Toggle the state: If it was true, it becomes false. If false, true.
									g_shuffle_slots_only = !g_shuffle_slots_only; 
									
									// Safety: If we turn this on, make sure the other shuffle mode is off
									if (g_shuffle_slots_only) {
										g_shuffle_all_banks = false;
										g_last_shuffle_tick = xTaskGetTickCount(); // Reset timer on start
									}
									
									ESP_LOGW("SHUFFLE", "Bank Shuffle is now: %s", g_shuffle_slots_only ? "ON" : "OFF");
									set_ble_reply(resp_success, sizeof(resp_success));
									break;

								case CC_SET_BANK_ALL: // Shuffle across ALL banks (d0 08 d1)
									ESP_LOGI("SHUFFLE", "Toggle Global Shuffle Requested");
									
									// Toggle the state
									g_shuffle_all_banks = !g_shuffle_all_banks;
									
									if (g_shuffle_all_banks) {
										g_shuffle_slots_only = false;
										g_last_shuffle_tick = xTaskGetTickCount(); // Reset timer on start
									}
									
									ESP_LOGW("SHUFFLE", "Global Shuffle is now: %s", g_shuffle_all_banks ? "ON" : "OFF");
									set_ble_reply(resp_success, sizeof(resp_success));
									break;


									case CC_SET_PATTERN_SHUFFLE_DURATION: // Set the timer (e.g., 0-255 seconds)
										ESP_LOG_BUFFER_HEX("BLE_SPEED_RAW", data, len);
										// If your app sends the value in seconds at data[2]
										g_shuffle_duration_ms = data[2] * 1000; 
										if (g_shuffle_duration_ms < 1000) {
										 g_shuffle_duration_ms = 1000; // Min 1s
										 set_ble_reply(resp_success, sizeof(resp_success)); // Set 0x00 Success
									} else {
										set_ble_reply(resp_error, sizeof(resp_error));     // Set 0x01 Error
									}
									break;
										

								default: break;
							}
							return 0;
						}

						// --- DATA CHUNK HANDLING (Multipart) ---
						if (g_multipart_active) {
							flash_packet_t pkt;
							pkt.len = len;
							memcpy(pkt.pkt_data, data, len);
							
							if (xQueueSend(flash_queue, &pkt, pdMS_TO_TICKS(10)) != pdPASS) {
									ESP_LOGW("BLE", "Queue full! Dropping packet or returning error.");
									return BLE_ATT_ERR_PREPARE_QUEUE_FULL; 
								}
							return 0;
						}
					}
					return 0;
				}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
{
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        // SERVICE_UUID: 6e400001-b5a3-f393-e0a9-e50e24dcca9e
        .uuid = BLE_UUID128_DECLARE(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // RX_UUID: 6e400002...
                .uuid = BLE_UUID128_DECLARE(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E),
                .access_cb = gatt_svr_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .descriptors = (struct ble_gatt_dsc_def[]) { {
                    .uuid = BLE_UUID16_DECLARE(0x2901), // User Description Descriptor
                    .att_flags = BLE_ATT_F_READ,
                    .access_cb = gatt_svr_cb,
                    .arg = "RX Characteristic",
                }, {0} }
            },
            {
                // TX_UUID: 6e400003...
                .uuid = BLE_UUID128_DECLARE(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E),
                .access_cb = gatt_svr_cb,
                .flags = BLE_GATT_CHR_F_READ,
                .descriptors = (struct ble_gatt_dsc_def[]) { {
                    .uuid = BLE_UUID16_DECLARE(0x2901),
                    .att_flags = BLE_ATT_F_READ,
                    .access_cb = gatt_svr_cb,
                    .arg = "TX Characteristic",
                }, {0} }
            },
            {
                // NOTIFY_UUID: 6e400004...
                .uuid = BLE_UUID128_DECLARE(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x04, 0x00, 0x40, 0x6E),
                .access_cb = gatt_svr_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &notify_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) { {
                    .uuid = BLE_UUID16_DECLARE(0x2901),
                    .att_flags = BLE_ATT_F_READ,
                    .access_cb = gatt_svr_cb,
                    .arg = "Notify Characteristic",
                },
                
                 {0} }
            },
            {0}
        },
    },
    {0},
};

void host_task(void *param) {
    ESP_LOGI("BLE", "NimBLE Host Task Started");

    /* This function will not return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up when the loop finishes */
    nimble_port_freertos_deinit();
}
//BLE functions
void writeToPixelPoi(uint8_t* data) {
    uint16_t len = (data[1] << 8) | data[2];
    
    // 1. Check if we even have a connection
    if (conn_hdl == 65535) {
        ESP_LOGE("BLE_DEBUG", "Notify failed: No active connection handle!");
        return;
    }

    // 2. Check if the Notify Handle was actually captured during boot
    if (notify_handle == 0) {
        ESP_LOGE("BLE_DEBUG", "Notify failed: notify_handle is 0. Check GATT registration.");
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) {
        ESP_LOGE("BLE_DEBUG", "Notify failed: Could not allocate mbuf (Memory full?)");
        return;
    }

    // 3. Perform the notify and capture the specific NimBLE error
    int rc = ble_gatts_notify_custom(conn_hdl, notify_handle, om);
    
    if (rc != 0) {
        switch (rc) {
            case 6: // BLE_HS_EDISABLED
                ESP_LOGE("BLE_DEBUG", "RC 6: Phone has NOT subscribed to notifications (CCCD is 0)!");
                break;
            case 2: // BLE_HS_ENOTCONN
                ESP_LOGE("BLE_DEBUG", "RC 2: Stack thinks we are disconnected.");
                break;
            case 13: // BLE_HS_EINVAL
                ESP_LOGE("BLE_DEBUG", "RC 13: Invalid handle (%d) or length (%d).", notify_handle, len);
                break;
            default:
                ESP_LOGE("BLE_DEBUG", "Notify failed with RC: %d", rc);
        }
    } else {
        ESP_LOGI("BLE_DEBUG", "Notification sent successfully to handle %d", notify_handle);
    }
}

void bleSendFWVersion() {
    // [Start][LenH][LenL][Major][Minor][Patch][End] -> Wait, this is 7 bytes?
    // Let's adjust to match your exact protocol requirements:
    uint8_t response[] = {0xD0, 0x00, 0x06, 0x09, 0x02, 0xD1}; 
    writeToPixelPoi(response);
}

// MAIN
void app_main(void) {
// 1. Minimum OS setup
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


 //init spiffs
init_spiffs();

    // 2. NIMBLE FIRST (Synchronous)
    void check_memory(const char* label) {
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_8BIT);
    printf("--- Memory Check: %s ---\n", label);
    printf("Total Free: %zu bytes\n", info.total_free_bytes);
    printf("Largest Block: %zu bytes\n", info.largest_free_block); // <--- This is the key!
    printf("Minimum Ever Free: %zu bytes\n", info.minimum_free_bytes);
    printf("---------------------------\n");
}
    check_memory("BEFORE_NIMBLE_INIT");
    // Do NOT initialize LED strip or SPIFFS before this line
     // 5. Start the NimBLE Host Task
  ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE("BLE", "nimble_port_init failed: %d", ret);
        return;
    }
    // 3. Configure Host
    ble_hs_cfg.sync_cb = on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);
	// 2. Create the task manually with HIGH priority (e.g., 10 or 15)
	// This ensures Bluetooth always wins over the LED math and Flash writes
	xTaskCreate(host_task, "nimble_host", 8192, NULL, 15, NULL);

	// 3. Start the port (this triggers the stack)
	



 

    // 6. Hardware Init (Keep this AFTER BLE init to ensure heap availability)
    gpio_set_direction(REGULATOR_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(REGULATOR_GPIO, 1);
    
    // init queue
    flash_queue = xQueueCreate(20, sizeof(flash_packet_t));
    xTaskCreate(storage_worker_task, "storage_task", 6144, NULL, 4, NULL);
   // --- NEW: Initialize LED Strip ---
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO, // Ensure this matches your wiring (e.g., GPIO 8)
        .max_leds = 144,            // The physical max of your strip
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz for clean timing
    };
    
    // We use RMT because it's the most stable way to drive LEDs on C3
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    
    ESP_LOGI(TAG, "LED Strip Initialized.");
// --- Button setup ---
gpio_config_t btn_conf = {
    .pin_bit_mask = (1ULL << BOOT_BUTTON_PIN),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .intr_type = GPIO_INTR_ANYEDGE, 
};
gpio_config(&btn_conf);

// 1. Install the service
gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

// 2. Use the NEW function name:
gpio_isr_handler_add(BOOT_BUTTON_PIN, button_isr_handler, NULL);
// --- Button setup end ---


    //battery check
    adc_oneshot_unit_init_cfg_t init_config1 = {
							.unit_id = ADC_UNIT_1,
						};
						ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

						adc_oneshot_chan_cfg_t config = {
							.bitwidth = ADC_BITWIDTH_DEFAULT,
							.atten = ADC_ATTEN_DB_12, // Allows measuring up to ~3.1V on the pin
						};
						// Using GPIO 0 (ADC1 Channel 0)
						ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
ESP_LOGI(TAG, "Battery Check Initialized");
    // 7. Start the POV Rendering Task
    // We give it a priority of 5 (Higher than the idle task, but balanced for BLE)
    xTaskCreate(pov_render_task, "led_task", 10240, NULL, 5, NULL);

    ESP_LOGI(TAG, "System Ready. Waiting for BLE Sync...");
    
    // The main task can now just sit and watch the system or be deleted
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
