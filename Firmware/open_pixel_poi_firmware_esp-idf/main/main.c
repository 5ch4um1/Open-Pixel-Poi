#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <dirent.h>
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

/* Define UUIDs as actual static variables first */
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

static const ble_uuid128_t rx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

static const ble_uuid128_t tx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

static const ble_uuid128_t notify_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x04, 0x00, 0x40, 0x6E);

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
#define LED_GPIO            GPIO_NUM_8
#define BUTTON_GPIO         GPIO_NUM_3
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
#define PIXEL_COUNT 20
#define MAX_FRAMES 928
#define FRAME_SIZE (PIXEL_COUNT * 3)
#define FRAME_SIZE (PIXEL_COUNT * 3)   // 60 bytes
#define FRAMES_PER_PACKET 8              // How many frames to pack
#define TOTAL_DATA_LEN (FRAME_SIZE * FRAMES_PER_PACKET) // 480 bytes
static uint8_t packed_buffer[TOTAL_DATA_LEN];
static uint16_t current_offset = 0;
static uint8_t frame_queue[TOTAL_DATA_LEN * 2];
volatile int frames_available = 0;
static int current_frame_idx = 0;

typedef struct {
    uint8_t frames[MAX_FRAMES][FRAME_SIZE]; // 928 * 60 bytes
    volatile uint16_t head;
    volatile uint16_t tail;
    uint32_t total_played; // <--- Add this for global tracking
    bool is_streaming;
} pov_stream_buf_t;

static pov_stream_buf_t *ring_buf = NULL;
static TaskHandle_t pov_task_handle = NULL;
static esp_timer_handle_t pov_timer;
uint32_t current_period_us = 5000; // Default 200Hz (1/200s = 5000us)

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
    for (int i = 0; i < MAX_LEDS; i++) {
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

        //check battery
        static TickType_t last_battery_tick = 0;
        current_tick = xTaskGetTickCount();

        if ((current_tick - last_battery_tick) > pdMS_TO_TICKS(10000)) {
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
            default_rainbow_pattern();
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
        conn_hdl = (event->connect.status == 0) ? event->connect.conn_handle : 0xFFFF;
        ESP_LOGI("BLE", "Connected! Status: %d", event->connect.status);
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW("BLE", "Disconnected! Reason: %d. Restarting...", event->disconnect.reason);
        conn_hdl = 0xFFFF;
        // Safety: Reset multipart flag so we don't stay in "Loading" state
        g_multipart_active = false;

        // AUTO-RECOVERY: Start advertising, stop the pov_timer and enter the render loop again
        led_task_running = true;
        esp_timer_stop(pov_timer);
        frames_available = 0;
        start_advertising();

        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI("BLE", "MTU update: %d", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI("BLE", "Connection updated");
        return 0;

    default:
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
    CC_STOP_STREAM = 22,
    CC_GET_CONFIG = 23,
    CC_STREAM_DATA = 24
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

            case CC_GET_CONFIG:
            {
                ESP_LOGI("BLE", "Config request received");

                // 1. Prepare the response structure (Big Endian)
                // Packet: [Header, Code, PixelCount, Flags, Buf_H, Buf_L, FPS_H, FPS_L, Ver_H, Ver_L]
                uint8_t config_resp[10];
                config_resp[0] = 0xD0;
                config_resp[1] = CC_GET_CONFIG;
                config_resp[2] = (uint8_t)MAX_LEDS; // Ensure this is defined (e.g., 20)
                config_resp[3] = 0x01;                 // Flag 0x01: GRB color order

                // Buffer size (e.g., 928 frames)
                config_resp[4] = (uint8_t)((928 >> 8) & 0xFF);
                config_resp[5] = (uint8_t)(928 & 0xFF);

                // Continue...
