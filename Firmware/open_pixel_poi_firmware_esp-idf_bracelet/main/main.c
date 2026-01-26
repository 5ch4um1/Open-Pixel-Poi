#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_random.h"

/* NimBLE */
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

static const char *TAG = "POI_CENTRAL";
static const char *TARGET_NAME = "Pixel Poi";

// --- Configuration ---
#define STATUS_LED_GPIO 10
#define BUTTON_GPIO     9
#define LONG_PRESS_MS   800
#define BYTES_PER_PIXEL 3
#define MPU6050_ADDR    0x68
#define I2C_MASTER_SCL_IO 2     
#define I2C_MASTER_SDA_IO 4

uint16_t pixel_count = 12; 
uint16_t frame_count = 4;  
float brightness_factor = 0.1;

// Command Definitions
#define CC_START_STREAM 21
#define CC_STOP_STREAM  22
#define CC_STREAM_DATA  24
#define START_BYTE      0xD0
#define END_BYTE        0xD1

static const ble_uuid128_t rx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

// --- Data Structures ---

typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float temp;
    float total_accel; // Combined G-force
} mpu_data_t;

typedef struct {
    uint16_t conn_handle;
    uint16_t rx_char_handle;
    bool discovered;
    bool stream_started;
} poi_device_t;

poi_device_t devices[2];
// Removed volatile device_count to prevent drift errors

bool is_streaming = true;
static uint8_t own_addr_type;
static led_strip_handle_t led_strip = NULL;
int current_mode = 0;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t mpu_dev_handle;
static QueueHandle_t gpio_evt_queue = NULL;

// Forward declaration needed for the scanner
static int ble_central_event(struct ble_gap_event *event, void *arg);

// --- Helper Functions ---

// Robustly count how many devices are actually connected
int get_connected_count() {
    int count = 0;
    for(int i=0; i<2; i++) {
        if(devices[i].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            count++;
        }
    }
    return count;
}

// Helper to start scanning if we have room
void poi_scan_start(void) {
    if (get_connected_count() >= 2) return;
    
    if (!ble_gap_disc_active()) {
        struct ble_gap_disc_params dp = { .passive = 0, .itvl = 100, .window = 50, .filter_duplicates = 1 };
        ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &dp, ble_central_event, NULL);
        ESP_LOGI(TAG, "Scanning started...");
    }
}

// --- Mode Logic Setup ---

typedef void (*poi_mode_fn)(mpu_data_t *s, uint8_t *pixel_data, size_t len);

void hsv_to_rgb(uint8_t h_in, uint8_t *r, uint8_t *g, uint8_t *b) {
    uint16_t h_scaled = h_in * 3; 
    if (h_scaled < 255) { *r = 255 - h_scaled; *g = h_scaled; *b = 0; }
    else if (h_scaled < 510) { h_scaled -= 255; *r = 0; *g = 255 - h_scaled; *b = h_scaled; }
    else { h_scaled -= 510; *r = h_scaled; *g = 0; *b = 255 - h_scaled; }
}

// Mode 0: Gravity Rainbow
void mode_gravity_rainbow(mpu_data_t *s, uint8_t *pixel_data, size_t len) {
    float angle = atan2f((float)s->acc_y, (float)s->acc_x);
    uint8_t r, g, b;
    hsv_to_rgb((uint8_t)(((angle + M_PI) / (2.0f * M_PI)) * 255.0f), &r, &g, &b);
    for (int i = 0; i < len; i += 3) {
        pixel_data[i] = r; pixel_data[i+1] = g; pixel_data[i+2] = b;
    }
}

// Mode 1: Centrifugal Fire
void mode_spin_fire(mpu_data_t *s, uint8_t *pixel_data, size_t len) {
    uint8_t intensity = (uint8_t)(fminf(abs(s->gyro_z) / 64.0f, 255.0f));
    for (int i = 0; i < len; i += 3) {
        pixel_data[i] = intensity; pixel_data[i+1] = intensity/4; pixel_data[i+2] = 0;
    }
}

// MODE 2: Thermal Fire
void mode_thermal_fire(mpu_data_t *s, uint8_t *pixel_data, size_t len) {
    float temp_clamped = fmaxf(20.0f, fminf(40.0f, s->temp));
    uint8_t base_hue = (uint8_t)((1.0f - (temp_clamped - 20.0f) / 20.0f) * 170.0f);
    
    float flicker = (s->total_accel / 16384.0f) * 255.0f;
    uint8_t r, g, b;
    hsv_to_rgb(base_hue, &r, &g, &b);

    for (int i = 0; i < len; i += 3) {
        float random_flicker = (float)(esp_random() % 50) / 100.0f + 0.5f;
        pixel_data[i]   = (uint8_t)(fminf(r * random_flicker * (flicker/128.0f), 255.0f));
        pixel_data[i+1] = (uint8_t)(fminf(g * random_flicker * (flicker/128.0f), 255.0f));
        pixel_data[i+2] = (uint8_t)(fminf(b * random_flicker * (flicker/128.0f), 255.0f));
    }
}

// MODE 3: Centrifugal Rainbow
void mode_centrifugal_rainbow(mpu_data_t *s, uint8_t *pixel_data, size_t len) {
    static float hue_tracker = 0;
    hue_tracker += abs(s->gyro_z) / 2000.0f;
    if (hue_tracker >= 255) hue_tracker = 0;

    for (int i = 0; i < len; i += 3) {
        uint8_t r, g, b;
        hsv_to_rgb((uint8_t)hue_tracker + (i * 2), &r, &g, &b);
        pixel_data[i] = r;
        pixel_data[i+1] = g;
        pixel_data[i+2] = b;
    }
}

// MODE 4: Flow-Trail
void mode_flow_trail(mpu_data_t *s, uint8_t *pixel_data, size_t len) {
    float swing_force = fmaxf(0.0f, s->total_accel - 16384.0f);
    uint8_t intensity = (uint8_t)fminf(swing_force / 40.0f, 255.0f);
    
    for (int i = 0; i < len; i += 3) {
        pixel_data[i] = intensity / 4; 
        pixel_data[i+1] = intensity;   
        pixel_data[i+2] = intensity;   
    }
}

// MODE 5: Gravity Compass
void mode_gravity_compass(mpu_data_t *s, uint8_t *pixel_data, size_t len) {
    float angle = atan2f((float)s->acc_y, (float)s->acc_x); 
    uint8_t base_hue = (uint8_t)(((angle + M_PI) / (2.0f * M_PI)) * 255.0f);

    for (int i = 0; i < len; i += 3) {
        uint8_t r, g, b;
        uint8_t pixel_offset = i / 3; 
        uint8_t final_hue = base_hue + (pixel_offset * 2); 
        
        hsv_to_rgb(final_hue, &r, &g, &b);
        pixel_data[i]     = r;
        pixel_data[i + 1] = g;
        pixel_data[i + 2] = b;
    }
}

// MODE 6: Velocity Prism
void mode_velocity_prism(mpu_data_t *s, uint8_t *pixel_data, size_t len) {
    float total_velocity = sqrtf(s->gyro_x*s->gyro_x + s->gyro_y*s->gyro_y + s->gyro_z*s->gyro_z);
    uint8_t hue = (uint8_t)fminf(total_velocity / 180.0f, 170.0f);
    uint8_t r, g, b;
    hsv_to_rgb(hue, &r, &g, &b);
    for (int i = 0; i < len; i += 3) {
        pixel_data[i] = r; pixel_data[i+1] = g; pixel_data[i+2] = b;
    }
}

// MODE 7: Warp Speed Vortex
void mode_warp_speed(mpu_data_t *s, uint8_t *pixel_data, size_t len) {
    float spin_speed = abs(s->gyro_z) / 1000.0f;
    static float travel = 0;
    travel += spin_speed * 0.5f;

    for (int i = 0; i < len; i += 3) {
        uint8_t pixel_idx = i / 3;
        float wave = sinf((pixel_idx * 0.5f) - travel) * 127.0f + 128.0f;
        uint8_t r, g, b;
        hsv_to_rgb(140 + (wave / 4), &r, &g, &b);
        pixel_data[i] = (uint8_t)(r * (wave / 255.0f));
        pixel_data[i+1] = (uint8_t)(g * (wave / 255.0f));
        pixel_data[i+2] = (uint8_t)(b * (wave / 255.0f));
    }
}

// MODE 8: Plasma Ghost
void mode_plasma_ghost(mpu_data_t *s, uint8_t *pixel_data, size_t len) {
    static float phase = 0;
    phase += 0.1f;
    float activity = (abs(s->gyro_x) + abs(s->gyro_y)) / 5000.0f;
    
    for (int i = 0; i < len; i += 3) {
        float hue = (sinf(phase + (i * 0.3f)) * 30.0f) + 160.0f;
        uint8_t r, g, b;
        hsv_to_rgb((uint8_t)hue, &r, &g, &b);
        float brightness = 0.3f + fminf(activity, 0.7f);
        pixel_data[i] = (uint8_t)(r * brightness);
        pixel_data[i+1] = (uint8_t)(g * brightness);
        pixel_data[i+2] = (uint8_t)(b * brightness);
    }
}

// MODE 9: Fire & Ice Split
void mode_fire_ice_split(mpu_data_t *s, uint8_t *pixel_data, size_t len) {
    float height = (float)s->acc_y / 16384.0f;
    uint8_t r_ice, g_ice, b_ice;
    uint8_t r_fire, g_fire, b_fire;
    hsv_to_rgb(160, &r_ice, &g_ice, &b_ice);
    hsv_to_rgb(15, &r_fire, &g_fire, &b_fire);
    
    float fire_weight = (height + 1.0f) / 2.0f;
    float ice_weight = 1.0f - fire_weight;

    for (int i = 0; i < len; i += 3) {
        pixel_data[i]   = (r_ice * ice_weight) + (r_fire * fire_weight);
        pixel_data[i+1] = (g_ice * ice_weight) + (g_fire * fire_weight);
        pixel_data[i+2] = (b_ice * ice_weight) + (b_fire * fire_weight);
    }
}

// MODE 10: Shifting Horizon
void mode_shifting_horizon(mpu_data_t *s, uint8_t *pixel_data, size_t len) {
    float tilt = (float)s->acc_x / 16384.0f;
    int border = (int)((tilt + 1.0f) * 6.0f); 

    for (int i = 0; i < 12; i++) {
        uint8_t r, g, b;
        if (i < border) {
            hsv_to_rgb(15, &r, &g, &b);
        } else {
            hsv_to_rgb(160, &r, &g, &b);
        }
        pixel_data[i*3] = r;
        pixel_data[i*3+1] = g;
        pixel_data[i*3+2] = b;
    }
}

// MODE 11: Gravity Ball
void mode_gravity_ball(mpu_data_t *s, uint8_t *pixel_data, size_t len) {
    static float ball_pos = 0;
    float force = (s->total_accel - 16384.0f) / 10000.0f; 
    ball_pos += force;
    
    if (ball_pos > 11) ball_pos = 11;
    if (ball_pos < 0) ball_pos = 0;

    memset(pixel_data, 10, len);
    int active_pixel = (int)ball_pos;
    pixel_data[active_pixel*3] = 255; 
    pixel_data[active_pixel*3+1] = 255; 
    pixel_data[active_pixel*3+2] = 255;
}

// MODE 12: Compass Navigator
void mode_compass_navigator(mpu_data_t *s, uint8_t *pixel_data, size_t len) {
    static float yaw = 0;
    yaw += (s->gyro_z / 131.0f) * 0.03f;
    
    uint8_t base_hue = (uint8_t)yaw;
    uint8_t r, g, b;
    hsv_to_rgb(base_hue, &r, &g, &b);

    for (int i = 0; i < len; i += 3) {
        if (i >= len - 3) {
            pixel_data[i] = 100; pixel_data[i+1] = 0; pixel_data[i+2] = 0;
        } else {
            pixel_data[i] = r/2; pixel_data[i+1] = g/2; pixel_data[i+2] = b/2;
        }
    }
}

poi_mode_fn mode_table[] = { 
    mode_gravity_rainbow, 
    mode_spin_fire, 
    mode_thermal_fire, 
    mode_centrifugal_rainbow, 
    mode_flow_trail,
    mode_gravity_compass,
    mode_velocity_prism,
    mode_warp_speed,
    mode_plasma_ghost,
    mode_fire_ice_split,
    mode_shifting_horizon,
    mode_gravity_ball,
    mode_compass_navigator
     
};
#define MODE_COUNT (sizeof(mode_table) / sizeof(poi_mode_fn))

// --- Helpers ---

void set_status_led(uint8_t r, uint8_t g, uint8_t b) {
    if (!led_strip) return;
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

void update_status_led() {
    int c = get_connected_count();
    if (is_streaming && c > 0) set_status_led(0, 0, 0);
    else if (c == 2) set_status_led(0, 50, 0);
    else if (c == 1) set_status_led(30, 20, 0);
    else set_status_led(0, 0, 50);
}

// --- BLE Handlers ---

static int on_mtu_exchange(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t mtu, void *arg) {
    if (error->status == 0) {
        // Tighter intervals for faster streaming (15ms - 30ms)
        struct ble_gap_upd_params params = { .itvl_min = 24, .itvl_max = 36, .latency = 0, .supervision_timeout = 400 };
        ble_gap_update_params(conn_handle, &params);
    }
    return 0;
}

static int on_disc_char(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg) {
    if (error->status == 0 && chr != NULL && ble_uuid_cmp(&rx_uuid.u, &chr->uuid.u) == 0) {
        for(int i = 0; i < 2; i++) {
            if(devices[i].conn_handle == conn_handle) {
                devices[i].rx_char_handle = chr->val_handle;
                devices[i].discovered = true;
                update_status_led();
                break;
            }
        }
    }
    return (error->status == BLE_HS_EDONE) ? 0 : error->status;
}

static int ble_central_event(struct ble_gap_event *event, void *arg) {
    struct ble_hs_adv_fields fields;
    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            if (fields.name_len > 0 && strncmp((char *)fields.name, TARGET_NAME, fields.name_len) == 0) {
                // Cancel scan and connect
                ble_gap_disc_cancel();
                ble_gap_connect(own_addr_type, &event->disc.addr, 30000, NULL, ble_central_event, NULL);
            }
            break;

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                bool slot_found = false;
                for (int i = 0; i < 2; i++) {
                    if (devices[i].conn_handle == BLE_HS_CONN_HANDLE_NONE) {
                        devices[i].conn_handle = event->connect.conn_handle;
                        slot_found = true;
                        break;
                    }
                }
                if (slot_found) {
                    ble_gattc_exchange_mtu(event->connect.conn_handle, on_mtu_exchange, NULL);
                    ble_gattc_disc_all_chrs(event->connect.conn_handle, 1, 0xffff, on_disc_char, NULL);
                } else {
                    // No room, disconnect this extra device
                    ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                }
            } else {
                // Connection failed, resume scan
                poi_scan_start();
            }
            update_status_led();
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGW(TAG, "Device Disconnected: handle=%d", event->disconnect.conn.conn_handle);
            for (int i = 0; i < 2; i++) {
                if (devices[i].conn_handle == event->disconnect.conn.conn_handle) {
                    // Full reset of the slot
                    devices[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;
                    devices[i].discovered = false;
                    devices[i].stream_started = false;
                    devices[i].rx_char_handle = 0;
                }
            }
            update_status_led();
            // IMMEDIATE RE-SCAN
            poi_scan_start();
            break;
    }
    return 0;
}

static void ble_watchdog_task(void *param) {
    while (1) {
        // Only run watchdog check if we are missing devices
        if (get_connected_count() < 2) {
            poi_scan_start();
            vTaskDelay(pdMS_TO_TICKS(5000));
        } else {
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }
}

// --- Sensor & Streaming ---

void stream_task(void *param) {
    mpu_data_t sensor;
    size_t payload_size = pixel_count * frame_count * BYTES_PER_PIXEL;
    uint8_t packet[2 + payload_size];
    packet[0] = START_BYTE; packet[1] = CC_STREAM_DATA;

    while (1) {
        int connected_cnt = get_connected_count();

        if (is_streaming && connected_cnt > 0 && mpu_dev_handle != NULL) {
            uint8_t raw[14], reg = 0x3B;
            // Shorter timeout for I2C to not block
            if (i2c_master_transmit_receive(mpu_dev_handle, &reg, 1, raw, 14, 20) == ESP_OK) {
                sensor.acc_x = (int16_t)((raw[0]<<8)|raw[1]); 
                sensor.acc_y = (int16_t)((raw[2]<<8)|raw[3]); 
                sensor.acc_z = (int16_t)((raw[4]<<8)|raw[5]);
                sensor.total_accel = sqrtf(sensor.acc_x*sensor.acc_x + sensor.acc_y*sensor.acc_y + sensor.acc_z*sensor.acc_z);
                
                sensor.temp = ((int16_t)((raw[6]<<8)|raw[7]) / 340.0) + 36.53;
                sensor.gyro_x = (int16_t)((raw[8]<<8)|raw[9]); 
                sensor.gyro_y = (int16_t)((raw[10]<<8)|raw[11]); 
                sensor.gyro_z = (int16_t)((raw[12]<<8)|raw[13]);
            }

            mode_table[current_mode % MODE_COUNT](&sensor, &packet[2], payload_size);
            
            // Apply Brightness
            for(int i=2; i<sizeof(packet); i++) {
                packet[i] = (uint8_t)(packet[i] * brightness_factor);
            }

for (int i = 0; i < 2; i++) {
    if (devices[i].conn_handle != BLE_HS_CONN_HANDLE_NONE && devices[i].discovered) {
        
        // 1. Initial Start Stream Command (Reliable)
        if (!devices[i].stream_started) {
            uint8_t sc[] = {START_BYTE, CC_START_STREAM, END_BYTE};
            int rc = ble_gattc_write_flat(devices[i].conn_handle, devices[i].rx_char_handle, sc, 3, NULL, NULL);
            if (rc == 0) devices[i].stream_started = true;
            continue; 
        }

        // 2. Stream Data (Unreliable/Fast)
        // We use NO_RSP. If the return code is non-zero, the buffer is full.
        int rc = ble_gattc_write_no_rsp_flat(devices[i].conn_handle, devices[i].rx_char_handle, packet, sizeof(packet));
        
        if (rc == BLE_HS_ENOMEM || rc == BLE_HS_EAGAIN) {
            // Stack is full! Instead of waiting and causing lag, we just skip this frame
            // for this specific device to keep the stream "live".
            ESP_LOGD(TAG, "Buffer full, dropping frame for device %d", i);
        }
    }
}
        }
        
        // Fluidity Logic:
        // If we are missing a device, run slower (120ms) to give radio time to scan.
        // If we have both, run fast (33ms = ~30fps) for smooth LED visuals.
        vTaskDelay(pdMS_TO_TICKS(connected_cnt < 2 ? 120 : 48));
    }
}

// --- Initialization Functions ---

static void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void button_event_task(void* arg) {
    uint32_t io_num;
    while (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (gpio_get_level(io_num) == 0) {
            uint32_t start = xTaskGetTickCount();
            while (gpio_get_level(io_num) == 0) vTaskDelay(20);
            if (pdTICKS_TO_MS(xTaskGetTickCount() - start) > LONG_PRESS_MS) {
                is_streaming = !is_streaming;
            } else {
                current_mode++;
                ESP_LOGI(TAG, "Mode changed to %d", current_mode % MODE_COUNT);
            }
            update_status_led();
        }
    }
}

static void on_sync(void) {
    ble_hs_id_infer_auto(0, &own_addr_type);
    // Kick off initial scan
    poi_scan_start();
}

void host_task_fn(void *param) {
    nimble_port_run();
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    for(int i=0; i<2; i++) devices[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;

    // LED Strip
    led_strip_config_t sc = { .strip_gpio_num = STATUS_LED_GPIO, .max_leds = 1, .led_model = LED_MODEL_WS2812, .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB };
    led_strip_rmt_config_t rc = { .clk_src = RMT_CLK_SRC_DEFAULT, .resolution_hz = 10000000 };
    led_strip_new_rmt_device(&sc, &rc, &led_strip);

    // MPU6050 - LOWERED SPEED FOR STABILITY
    i2c_master_bus_config_t bc = { .clk_source = I2C_CLK_SRC_DEFAULT, .i2c_port = I2C_NUM_0, .scl_io_num = I2C_MASTER_SCL_IO, .sda_io_num = I2C_MASTER_SDA_IO, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true };
    i2c_new_master_bus(&bc, &bus_handle);
    // 100kHz for robust connection over wires
    i2c_device_config_t dc = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = MPU6050_ADDR, .scl_speed_hz = 100000 };
    i2c_master_bus_add_device(bus_handle, &dc, &mpu_dev_handle);
    uint8_t wake[] = {0x6B, 0x00};
    i2c_master_transmit(mpu_dev_handle, wake, 2, -1);

    // Button
    gpio_config_t ic = { .intr_type = GPIO_INTR_NEGEDGE, .pin_bit_mask = (1ULL << BUTTON_GPIO), .mode = GPIO_MODE_INPUT, .pull_up_en = 1 };
    gpio_config(&ic);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(button_event_task, "btn", 4096, NULL, 1, NULL);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, (void*)BUTTON_GPIO);

    // BLE NimBLE
    nimble_port_init();
    ble_hs_cfg.sync_cb = on_sync;
    xTaskCreate(stream_task, "stream", 4096, NULL, 20, NULL);
    xTaskCreate(ble_watchdog_task, "wd", 2048, NULL, 1, NULL);
    nimble_port_freertos_init(host_task_fn);
}
