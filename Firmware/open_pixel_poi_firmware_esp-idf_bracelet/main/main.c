#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2c_master.h"

/* NimBLE - Order is important here */
#include "host/ble_hs.h"
#include "host/ble_gatt.h" 
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "driver/gpio.h"
#include <math.h> // Required for atan2

// button stuff
#include "led_strip.h"
static led_strip_handle_t led_strip = NULL; 
int current_mode = 0;
const int total_modes = 5;
#define STATUS_LED_GPIO GPIO_NUM_10
#define BUTTON_GPIO     GPIO_NUM_9
#define LONG_PRESS_MS   800

/* Force declaration to stop "inside parameter list" warning */
struct ble_gatt_chr;

static const char *TAG = "POI_CENTRAL";
static const char *TARGET_NAME = "Pixel Poi";

// --- Configuration Variables ---
uint16_t pixel_count = 12; 
uint16_t frame_count = 4;  
#define BYTES_PER_PIXEL 3 
float brightness_factor = 0.14;

// Gyro
uint8_t raw_accel[6];
uint8_t reg = 0x3B; // Accel X High register

// --- Global Handles for the New Driver ---
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t mpu_dev_handle;

// --- Defines ---
#define I2C_MASTER_SCL_IO   GPIO_NUM_2      
#define I2C_MASTER_SDA_IO   GPIO_NUM_4
#define MPU6050_ADDR        0x68


// Command Definitions
#define CC_START_STREAM 21
#define CC_STOP_STREAM  22
#define CC_STREAM_DATA  24
#define START_BYTE      0xD0
#define END_BYTE        0xD1

static const ble_uuid128_t rx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

typedef struct {
    uint16_t conn_handle;
    uint16_t rx_char_handle;
    bool discovered;
} poi_device_t;

poi_device_t devices[2];
int device_count = 0;
bool is_streaming = false;
static uint8_t own_addr_type;

// --- 1. Forward Declarations ---
static int ble_central_event(struct ble_gap_event *event, void *arg);

// --- 2. Button & Helpers ---
static QueueHandle_t gpio_evt_queue = NULL;

// 2. Define the ISR Handler
static void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    // Now the compiler knows what gpio_evt_queue is
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}



void set_status_led(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}
void update_status_led() {
    if (is_streaming) return;

    int ready = 0;
    for (int i = 0; i < 2; i++) {
        if (devices[i].rx_char_handle != 0) ready++;
    }

    if (ready == 2) set_status_led(50, 0, 0);      // GREEN
    else if (ready == 1) set_status_led(20, 50, 0); // ORANGE
    else set_status_led(0, 0, 50);                 // BLUE (Static instead of blinking to save CPU)
}

static void button_event_task(void* arg) {
    uint32_t io_num;
    while (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
        // Debounce: Wait a moment and check if still pressed
        vTaskDelay(pdMS_TO_TICKS(50)); 
        if (gpio_get_level(io_num) == 0) {
            
            uint32_t start = xTaskGetTickCount();
            while (gpio_get_level(io_num) == 0) { vTaskDelay(20); } // Wait for release
            uint32_t duration = pdTICKS_TO_MS(xTaskGetTickCount() - start);

            if (duration > 800) { // Long Press
                current_mode = (current_mode + 1) % total_modes;
                set_status_led(50, 0, 00); // Yellow flash
                vTaskDelay(pdMS_TO_TICKS(420));
                set_status_led(0,0,0); // Return to connection color
            } else { // Short Press
                is_streaming = !is_streaming;
                if (!is_streaming) update_status_led();
                else set_status_led(0,0,0); // Dark during stream
            }
        }
    }
}



// status led stuff
// --- Status LED Helper ---


void led_startup_animation() {
    uint8_t colors[3][3] = {
        {20, 0, 0},   // Green
        {0, 20, 0},   // Red
        {0, 0, 20}    // Blue
    };

    for (int i = 0; i < 3; i++) {
        led_strip_set_pixel(led_strip, 0, colors[i][0], colors[i][1], colors[i][2]);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(200));
        
        led_strip_clear(led_strip);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int get_ready_device_count() {
    int ready = 0;
    for (int i = 0; i < 2; i++) {
        if (devices[i].discovered && devices[i].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ready++;
        }
    }
    return ready;
}

// --- Initialize LED Strip ---
void init_led() {
    led_strip_config_t strip_config = {
        .strip_gpio_num = 10, 
        .max_leds = 1,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false, // Critical: Ensure signal isn't inverted
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .mem_block_symbols = 64,           // Give it enough memory for the pulse
        .flags.with_dma = false,           // C3 RMT doesn't need DMA for 1 LED
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
    
    // Run the animation immediately
    led_startup_animation();
}

// --- Enhanced Button Task ---


// --- 3. GATT Callbacks ---

static int on_mtu_exchange(uint16_t conn_handle, const struct ble_gatt_error *error,
                           uint16_t mtu, void *arg) {
    if (error->status == 0) {
        ESP_LOGI(TAG, "MTU exchanged; handle=%d mtu=%d", conn_handle, mtu);
    }
    return 0;
}

static int on_disc_char(uint16_t conn_handle, const struct ble_gatt_error *error,
                         const struct ble_gatt_chr *chr, void *arg) {
    if (error->status == 0 && chr != NULL) {
        if (ble_uuid_cmp(&rx_uuid.u, &chr->uuid.u) == 0) {
            for(int i = 0; i < device_count; i++) {
                if(devices[i].conn_handle == conn_handle) {
                    devices[i].rx_char_handle = chr->val_handle;
                    devices[i].discovered = true;
                    ESP_LOGI(TAG, "RX handle found: %d for device index %d", chr->val_handle, i);
                    update_status_led();
                    break;
                }
            }
        }
    }
    return (error->status == BLE_HS_EDONE) ? 0 : error->status;
}

// --- 4. GAP Event Handler ---

static int ble_central_event(struct ble_gap_event *event, void *arg) {
    struct ble_hs_adv_fields fields;
    int rc;

    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            if (rc != 0) return 0;
            if (fields.name_len > 0 && strncmp((char *)fields.name, TARGET_NAME, fields.name_len) == 0) {
                ble_gap_disc_cancel();
                ble_gap_connect(own_addr_type, &event->disc.addr, 30000, NULL, ble_central_event, NULL);
            }
            return 0;

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                uint16_t h = event->connect.conn_handle;
                ESP_LOGI(TAG, "Connected to handle %d", h);
                
                devices[device_count].conn_handle = h;
                devices[device_count].discovered = false;
                devices[device_count].rx_char_handle = 0; // Ensure it's 0 until discovery
                device_count++;

                // 1. UPDATE LED TO ORANGE (We have at least one radio connection)
                update_status_led();

                ble_gattc_exchange_mtu(h, on_mtu_exchange, NULL);
                ble_gattc_disc_all_chrs(h, 1, 0xffff, on_disc_char, NULL);

                struct ble_gap_upd_params params = {
                    .itvl_min = 12, // 15ms
                    .itvl_max = 24, // 30ms
                    .latency = 0,
                    .supervision_timeout = 100,
                };
                ble_gap_update_params(event->connect.conn_handle, &params);

                if (device_count < 2) {
                    struct ble_gap_disc_params disc_params = {0};
                    // Use a slight delay before re-scanning to let the current connection stabilize
                    vTaskDelay(pdMS_TO_TICKS(100)); 
                    ble_gap_disc(own_addr_type, 10000, &disc_params, ble_central_event, NULL);
                }
            }
            return 0;

        // 2. ADD THIS: Handle Disconnections
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnect; reason=%d", event->disconnect.reason);
            
            // Clean up our device tracking
            for (int i = 0; i < 2; i++) {
                if (devices[i].conn_handle == event->disconnect.conn.conn_handle) {
                    devices[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;
                    devices[i].discovered = false;
                    devices[i].rx_char_handle = 0;
                    if (device_count > 0) device_count--;
                }
            }

            // If we aren't streaming, show the new status (Orange or Blue)
            update_status_led();

            // Restart scanning if we lost a device
            if (device_count < 2 && !is_streaming) {
                struct ble_gap_disc_params disc_params = {0};
                ble_gap_disc(own_addr_type, 10000, &disc_params, ble_central_event, NULL);
            }
            return 0;

        default:
            return 0;
    }
}

static void on_stack_sync(void) {
    ble_hs_id_infer_auto(0, &own_addr_type);
    struct ble_gap_disc_params disc_params = {0};
    ble_gap_disc(own_addr_type, 10000, &disc_params, ble_central_event, NULL);
}



// Gyro stuff
void init_mpu6050() {
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &mpu_dev_handle));

    // Wake up MPU6050 (Reg 0x6B = 0)
    uint8_t wake_cmd[] = {0x6B, 0x00};
    i2c_master_transmit(mpu_dev_handle, wake_cmd, sizeof(wake_cmd), -1);
    ESP_LOGI(TAG, "I2C Master Bus initialized and MPU6050 woken up.");
}



// colour stuff
void hsv_to_rgb(uint8_t h_in, uint8_t *r, uint8_t *g, uint8_t *b) {
    // We use a 16-bit integer for the calculation to prevent overflow
    uint16_t h_scaled = h_in * 3; 
    
    if (h_scaled < 255) {
        *r = 255 - h_scaled; 
        *g = h_scaled; 
        *b = 0;
    } else if (h_scaled < 510) {
        h_scaled -= 255;
        *r = 0; 
        *g = 255 - h_scaled; 
        *b = h_scaled;
    } else {
        h_scaled -= 510;
        *r = h_scaled; 
        *g = 0; 
        *b = 255 - h_scaled;
    }
}

// --- 5. Tasks ---

void stream_task(void *param) {
    float smoothed_hue = 0; // For buttery smooth transitions
   // Pre-allocate the packet once to save CPU and prevent memory leaks
    size_t payload_size = pixel_count * frame_count * BYTES_PER_PIXEL; 
    size_t total_len = 2 + payload_size;
    uint8_t packet[total_len]; // Static allocation is faster than malloc
    /*
    float accel_mag = sqrtf((float)acc_x * acc_x + (float)acc_y * acc_y); //for future use.
    */

    
    while (1) {
        if (is_streaming && device_count > 0 && mpu_dev_handle != NULL) {
            
            // 1. Robust Start Sequence
            uint8_t start_cmd[] = {START_BYTE, CC_START_STREAM, END_BYTE};
            for (int i = 0; i < device_count; i++) {
                if (devices[i].discovered) {
                    ble_gattc_write_flat(devices[i].conn_handle, devices[i].rx_char_handle, 
                                         start_cmd, 3, NULL, NULL);
                    vTaskDelay(pdMS_TO_TICKS(50)); // Delay between starting each device
                }
            }


            

                packet[0] = START_BYTE;
                packet[1] = CC_STREAM_DATA;

                while (is_streaming) {
                    uint8_t raw[14], reg = 0x3B;
                    int16_t acc_x = 0, acc_y = 0, acc_z = 0, gyro_x = 0 , gyro_y = 0, gyro_z = 0;
                    
                    uint8_t r = 0, g = 0, b = 0;

                    if (i2c_master_transmit_receive(mpu_dev_handle, &reg, 1, raw, 14, 50) == ESP_OK) {
                        acc_x = (int16_t)((raw[0] << 8) | raw[1]);
                        acc_y = (int16_t)((raw[2] << 8) | raw[3]);
                        acc_z = (int16_t)((raw[4] << 8) | raw[5]);
                        gyro_x = (int16_t)((raw[8] << 8) | raw[9]);
						gyro_y = (int16_t)((raw[10] << 8) | raw[11]);
						gyro_z = (int16_t)((raw[12] << 8) | raw[13]);
                    }

			// Kinteic modes
			switch (current_mode) {
				case 0: { // MODE 0: Smooth Tilt Rainbow (kept your favorite)
					float angle = atan2f((float)acc_y, (float)acc_x);
					float target_hue = ((angle + M_PI) / (2.0f * M_PI)) * 255.0f;
					smoothed_hue = (target_hue * 0.1f) + (smoothed_hue * 0.9f);
					hsv_to_rgb((uint8_t)smoothed_hue, &r, &g, &b);
					break;
				}

				case 1: { // MODE 1: Gyro RGB Mixer (New!)
					// Directly map X, Y, Z rotation speeds to R, G, B
					// Shift by 7 (divide by 128) to fit 0-32768 into 0-255
					r = (uint8_t)(abs(gyro_x) >> 7);
					g = (uint8_t)(abs(gyro_y) >> 7);
					b = (uint8_t)(abs(gyro_z) >> 7);
					break;
				}

				case 2: { // MODE 2: Fire Pulse
					uint8_t intensity = (uint8_t)(abs(acc_y) >> 7);
					r = intensity; 
					g = intensity / 4; 
					b = 0;
					break;
				}

case 3: { // IMPROVED MODE 3: 3D Kinetic Pulse
    // Calculate total 3D force
    // Using sum of absolute values is a fast CPU-friendly way to approximate magnitude
    int32_t total_force = abs(acc_x) + abs(acc_y) + abs(acc_z); 
    
    uint8_t pulse = (uint8_t)(fminf(total_force >> 7, 255));

    if (gyro_z > 0) {
        r = pulse / 2; g = 0; b = pulse; // Magenta
    } else {
        r = 0; g = pulse / 2; b = pulse; // Cyan
    }
    break;
}

case 4: { // IMPROVED MODE 4: 3D Gravity Compass
            // Map the absolute tilt of each axis to a color
            // 16384 is roughly 1G on the MPU6050 default scale
            r = (uint8_t)(fminf(abs(acc_x) >> 6, 255));
            g = (uint8_t)(fminf(abs(acc_y) >> 6, 255));
            b = (uint8_t)(fminf(abs(acc_z) >> 6, 255));
            break;
        }

				default:
					current_mode = 0;
					break;
			}

			//end kinetic modes




                    // 4. Fill packet
                    for (int p = 0; p < payload_size; p += 3) {
                        packet[2 + p]     = r * brightness_factor;
                        packet[2 + p + 1] = g * brightness_factor;
                        packet[2 + p + 2] = b * brightness_factor;
                    }

                    // 5. Interleaved sending
                    for (int i = 0; i < device_count; i++) {
                        if (devices[i].discovered) {
                            ble_gattc_write_no_rsp_flat(devices[i].conn_handle, 
                                                       devices[i].rx_char_handle, 
                                                       packet, total_len);
                            vTaskDelay(pdMS_TO_TICKS(20)); 
                        }
                    }
                    vTaskDelay(pdMS_TO_TICKS(20));
                }

                // 2. Robust Stop Sequence
                uint8_t stop_cmd[] = {START_BYTE, CC_STOP_STREAM, END_BYTE};
                for (int i = 0; i < device_count; i++) {
                    if (devices[i].discovered) {
                        ble_gattc_write_flat(devices[i].conn_handle, devices[i].rx_char_handle, 
                                            stop_cmd, 3, NULL, NULL);
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                }
                
            
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void init_button_interrupt() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Ensure this is explicitly enabled
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(button_event_task, "button_event_task", 4096, NULL, 1, NULL);

    gpio_install_isr_service(0);
    // Use the name 'button_isr_handler' here
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, (void*) BUTTON_GPIO);
}

// --- 6. Main ---

void app_main(void) {
    nvs_flash_init();
    

    init_button_interrupt();
    init_led();
    init_mpu6050();
    nimble_port_init();
    ble_hs_cfg.sync_cb = on_stack_sync;
    
    xTaskCreate(stream_task, "stream_task", 4096, NULL, 20, NULL);
    nimble_port_freertos_init(ble_host_task);
}
