
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

    // NEW: keep peer address so we can reconnect instantly
    ble_addr_t peer_addr;
    bool       have_addr;
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
    else set_status_led(0, 0, 50);                 // BLUE
}

static void button_event_task(void* arg) {
    uint32_t io_num;
    while (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
        // Debounce
        vTaskDelay(pdMS_TO_TICKS(50));
        if (gpio_get_level(io_num) == 0) {
            uint32_t start = xTaskGetTickCount();
            while (gpio_get_level(io_num) == 0) { vTaskDelay(20); } // wait for release
            uint32_t duration = pdTICKS_TO_MS(xTaskGetTickCount() - start);

            if (duration > 800) { // Long Press = change mode
                current_mode = (current_mode + 1) % total_modes;
                set_status_led(50, 0, 00); // Yellow flash
                vTaskDelay(pdMS_TO_TICKS(420));
                set_status_led(0,0,0);
            } else { // Short Press = toggle streaming
                is_streaming = !is_streaming;
                if (!is_streaming) update_status_led();
                else set_status_led(0,0,0); // Dark during stream
            }
        }
    }
}

// status led stuff
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
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
    led_startup_animation();
}

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
            for(int i = 0; i < 2; i++) {
                if(devices[i].conn_handle == conn_handle) {
                    devices[i].rx_char_handle = chr->val_handle;
                    devices[i].discovered = true;
                    ESP_LOGI(TAG, "RX handle found: %d for device index %d", chr->val_handle, i);
                    update_status_led();

                    // If streaming is already active, immediately start stream on this device.
                    if (is_streaming) {
                        uint8_t start_cmd[] = {START_BYTE, CC_START_STREAM, END_BYTE};
                        ble_gattc_write_flat(devices[i].conn_handle,
                                             devices[i].rx_char_handle,
                                             start_cmd, sizeof(start_cmd),
                                             NULL, NULL);
                    }
                    break;
                }
            }
        }
    }
    return (error->status == BLE_HS_EDONE) ? 0 : error->status;
}

// --- 4. Robust Scan/Reconnect Machinery (NEW) ---
#include "freertos/timers.h"

static bool          scanning = false;
static TimerHandle_t scan_retry_tmr = NULL;
static uint32_t      scan_backoff_ms = 500;
static const uint32_t SCAN_BACKOFF_MAX_MS = 15000;
static const uint32_t SCAN_OK_RESET_MS   = 500;

static void scan_retry_cb(TimerHandle_t xTimer); // fwd

static void schedule_scan_soon(uint32_t delay_ms)
{
    if (scan_retry_tmr) xTimerStop(scan_retry_tmr, 0);
    xTimerChangePeriod(scan_retry_tmr, pdMS_TO_TICKS(delay_ms), 0);
    xTimerStart(scan_retry_tmr, 0);
}

static int start_scan(uint32_t duration_ms)
{
    if (scanning) return 0;

    struct ble_gap_disc_params p = {0}; // defaults are fine (active scan)

    int rc = ble_gap_disc(own_addr_type, duration_ms, &p, ble_central_event, NULL);
    if (rc == 0) {
        scanning = true;
        ESP_LOGI(TAG, "Scanning started (%ums)", (unsigned)duration_ms);
        return 0;
    }

    ESP_LOGW(TAG, "ble_gap_disc rc=%d; scheduling retry in %u ms", rc, (unsigned)scan_backoff_ms);
    if (scan_retry_tmr) xTimerStop(scan_retry_tmr, 0);
    xTimerChangePeriod(scan_retry_tmr, pdMS_TO_TICKS(scan_backoff_ms), 0);
    xTimerStart(scan_retry_tmr, 0);

    if (scan_backoff_ms < SCAN_BACKOFF_MAX_MS) {
        scan_backoff_ms <<= 1u;
        if (scan_backoff_ms > SCAN_BACKOFF_MAX_MS) scan_backoff_ms = SCAN_BACKOFF_MAX_MS;
    }
    return rc;
}

static void scan_retry_cb(TimerHandle_t xTimer)
{
    scanning = false;
    (void)start_scan(10000); // keep trying
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "nimble reset reason=%d; rescheduling scan", reason);
    schedule_scan_soon(250);
}

// --- 5. GAP Event Handler (UPDATED) ---

static int ble_central_event(struct ble_gap_event *event, void *arg) {
    struct ble_hs_adv_fields fields;
    int rc;

    switch (event->type) {
        case BLE_GAP_EVENT_DISC: {
            rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
            if (rc != 0) return 0;

            if (fields.name_len > 0 &&
                strncmp((char *)fields.name, TARGET_NAME, fields.name_len) == 0) {

                // Stop scanning and try to connect
                ble_gap_disc_cancel(); // will trigger DISC_COMPLETE event
                rc = ble_gap_connect(own_addr_type, &event->disc.addr, 30000, NULL, ble_central_event, NULL);
                if (rc != 0) {
                    ESP_LOGW(TAG, "ble_gap_connect rc=%d; rescan soon", rc);
                    schedule_scan_soon(250);
                }
            }
            return 0;
        }

        case BLE_GAP_EVENT_DISC_COMPLETE: {
            scanning = false;
            if (device_count < 2) {
                ESP_LOGI(TAG, "scan complete (reason=%d), rescan in %ums",
                         event->disc_complete.reason, (unsigned)scan_backoff_ms);
                schedule_scan_soon(scan_backoff_ms);
            } else {
                scan_backoff_ms = SCAN_OK_RESET_MS;
            }
            return 0;
        }

        case BLE_GAP_EVENT_CONNECT: {
            if (event->connect.status == 0) {
                uint16_t h = event->connect.conn_handle;
                ESP_LOGI(TAG, "Connected to handle %d", h);

                if (device_count < 2) {
                    devices[device_count].conn_handle = h;
                    devices[device_count].discovered = false;
                    devices[device_count].rx_char_handle = 0;
                    devices[device_count].peer_addr = event->connect.peer_addr;
                    devices[device_count].have_addr = true;
                    device_count++;
                } else {
                    ESP_LOGW(TAG, "More than two connections?");
                }

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

                // Keep scanning if we still need the second device
                if (device_count < 2) {
                    schedule_scan_soon(150);
                } else {
                    scan_backoff_ms = SCAN_OK_RESET_MS;
                }
            } else {
                ESP_LOGW(TAG, "Connect failed; status=%d", event->connect.status);
                scanning = false;
                schedule_scan_soon(250);
            }
            return 0;
        }

        case BLE_GAP_EVENT_DISCONNECT: {
            ESP_LOGI(TAG, "Disconnect; reason=%d", event->disconnect.reason);

            // Clean up our device tracking and attempt instant reconnect
            for (int i = 0; i < 2; i++) {
                if (devices[i].conn_handle == event->disconnect.conn.conn_handle) {
                    devices[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;
                    devices[i].discovered = false;
                    devices[i].rx_char_handle = 0;
                    if (device_count > 0) device_count--;

                    // Try direct reconnect if we know the address
                    if (devices[i].have_addr) {
                        int rc2 = ble_gap_connect(own_addr_type, &devices[i].peer_addr,
                                                  30000, NULL, ble_central_event, NULL);
                        if (rc2 == 0) {
                            ESP_LOGI(TAG, "Reconnecting to saved peer...");
                            update_status_led();
                            return 0;
                        }
                        ESP_LOGW(TAG, "Direct reconnect rc=%d; will rescan", rc2);
                    }
                }
            }

            // Update LED
            update_status_led();

            // Always try to regain connections
            if (device_count < 2) {
                schedule_scan_soon(250);
            }
            return 0;
        }

        default:
            return 0;
    }
}

static void on_stack_sync(void) {
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto rc=%d", rc);
        return;
    }
    scan_backoff_ms = SCAN_OK_RESET_MS;
    (void)start_scan(10000);
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

// --- 6. Stream Task ---

void stream_task(void *param) {
    float smoothed_hue = 0;

    size_t payload_size = pixel_count * frame_count * BYTES_PER_PIXEL;
    size_t total_len = 2 + payload_size;
    uint8_t packet[2 + 3 * 12 * 4]; // upper-bound static allocation for safety (12*4*3 + 2 = 146)
    // If your pixel/frame counts change dynamically, switch back to VLA or malloc.

    while (1) {
        if (is_streaming && mpu_dev_handle != NULL) {
            // Robust start for any currently discovered device(s)
            uint8_t start_cmd[] = {START_BYTE, CC_START_STREAM, END_BYTE};
            for (int i = 0; i < 2; i++) {
                if (devices[i].discovered && devices[i].rx_char_handle != 0) {
                    ble_gattc_write_flat(devices[i].conn_handle, devices[i].rx_char_handle,
                                         start_cmd, sizeof(start_cmd), NULL, NULL);
                    vTaskDelay(pdMS_TO_TICKS(50));
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

                // Kinetic modes
                switch (current_mode) {
                    case 0: { // Smooth Tilt Rainbow
                        float angle = atan2f((float)acc_y, (float)acc_x);
                        float target_hue = ((angle + M_PI) / (2.0f * M_PI)) * 255.0f;
                        smoothed_hue = (target_hue * 0.1f) + (smoothed_hue * 0.9f);
                        hsv_to_rgb((uint8_t)smoothed_hue, &r, &g, &b);
                        break;
                    }
                    case 1: { // Gyro RGB Mixer
                        r = (uint8_t)(abs(gyro_x) >> 7);
                        g = (uint8_t)(abs(gyro_y) >> 7);
                        b = (uint8_t)(abs(gyro_z) >> 7);
                        break;
                    }
                    case 2: { // Fire Pulse
                        uint8_t intensity = (uint8_t)(abs(acc_y) >> 7);
                        r = intensity; g = intensity / 4; b = 0;
                        break;
                    }
                    case 3: { // 3D Kinetic Pulse
                        int32_t total_force = abs(acc_x) + abs(acc_y) + abs(acc_z);
                        uint8_t pulse = (uint8_t)(fminf(total_force >> 7, 255));
                        if (gyro_z > 0) { r = pulse / 2; g = 0; b = pulse; }
                        else            { r = 0; g = pulse / 2; b = pulse; }
                        break;
                    }
                    case 4: { // 3D Gravity Compass
                        r = (uint8_t)(fminf(abs(acc_x) >> 6, 255));
                        g = (uint8_t)(fminf(abs(acc_y) >> 6, 255));
                        b = (uint8_t)(fminf(abs(acc_z) >> 6, 255));
                        break;
                    }
                    default:
                        current_mode = 0;
                        break;
                }

                // Fill packet
                for (int p = 0; p < payload_size; p += 3) {
                    packet[2 + p]     = (uint8_t)((float)r * brightness_factor);
                    packet[2 + p + 1] = (uint8_t)((float)g * brightness_factor);
                    packet[2 + p + 2] = (uint8_t)((float)b * brightness_factor);
                }

                // Interleaved sending to all currently discovered devices
                for (int i = 0; i < 2; i++) {
                    if (devices[i].discovered && devices[i].rx_char_handle != 0 &&
                        devices[i].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                        ble_gattc_write_no_rsp_flat(devices[i].conn_handle,
                                                    devices[i].rx_char_handle,
                                                    packet, 2 + payload_size);
                        vTaskDelay(pdMS_TO_TICKS(20));
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            }

            // Robust stop for any device still connected and discovered
            uint8_t stop_cmd[] = {START_BYTE, CC_STOP_STREAM, END_BYTE};
            for (int i = 0; i < 2; i++) {
                if (devices[i].discovered && devices[i].rx_char_handle != 0 &&
                    devices[i].conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                    ble_gattc_write_flat(devices[i].conn_handle, devices[i].rx_char_handle,
                                         stop_cmd, sizeof(stop_cmd), NULL, NULL);
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
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(button_event_task, "button_event_task", 4096, NULL, 1, NULL);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, (void*) BUTTON_GPIO);
}

// --- 7. Main ---

void app_main(void) {
    nvs_flash_init();

    // Init devices tracking
    for (int i = 0; i < 2; i++) {
        devices[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;
        devices[i].rx_char_handle = 0;
        devices[i].discovered = false;
        devices[i].have_addr = false;
    }

    init_button_interrupt();
    init_led();
    init_mpu6050();

    nimble_port_init();
    ble_hs_cfg.sync_cb  = on_stack_sync;
    ble_hs_cfg.reset_cb = on_reset;

    // Scan retry timer
    scan_retry_tmr = xTimerCreate("scan_retry",
                                  pdMS_TO_TICKS(500),
                                  pdFALSE, NULL, scan_retry_cb);
    configASSERT(scan_retry_tmr != NULL);

    xTaskCreate(stream_task, "stream_task", 4096, NULL, 20, NULL);
    nimble_port_freertos_init(ble_host_task);
}
