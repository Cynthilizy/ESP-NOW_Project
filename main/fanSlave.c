#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "aht.h"
#include "string.h"

// Tags and configurations
static const char *TAG = "Fan_Control_ESPNOW";

#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_PORT I2C_NUM_0
#define RELAY_PIN GPIO_NUM_7
#define TEMP_THRESHOLD 30.0f
#define AHT20_ADDR 0x38
#define RESET_TIMEOUT_MS (30 * 60 * 1000) // 30-minute timeout in ms
#define PING "PING"
#define PONG "PONG"
#define FAN_CONTROL_MSG "Fan Control Pressed"
#define FAN_ON_MSG "Fan On"
#define FAN_OFF_MSG "Fan Off"

static const uint8_t master_mac[6] = {0x34, 0x85, 0x18, 0xA1, 0x5D, 0xC0};

// Global Variables
aht_t aht20_dev;
bool manual_override = false;
bool relay_status = false; // Tracks the relay state
TimerHandle_t manual_override_timer;

// Function to reset manual override
void reset_manual_override(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Manual override reset. Resuming sensor control.");
    manual_override = false; // Reset manual override
    {
        if (xTimerReset(manual_override_timer, 0) != pdPASS)
        {
            ESP_LOGE(TAG, "Failed to reset manual override timer");
        }
        else
        {
            ESP_LOGI(TAG, "Manual override timer reset");
        }
    }
}

void send_callback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    ESP_LOGI(TAG, "Message send status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Initialize WiFi in STA mode for ESPNOW
void wifi_init()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// Initialize ESPNOW
void espnow_init()
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(receive_callback));
    ESP_ERROR_CHECK(esp_now_register_send_cb(send_callback));

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, master_mac, 6);
    peer_info.channel = 0;
    peer_info.ifidx = ESP_IF_WIFI_STA;
    peer_info.encrypt = false;

    if (esp_now_add_peer(&peer_info) == ESP_OK)
    {
        ESP_LOGI(TAG, "Peer added successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to add peer");
    }
}
void fan_control_task(void *pvParameter)
{
    float temperature, humidity;

    ESP_LOGI(TAG, "Starting fan control task...");
    while (true)
    {
        if (!manual_override)
        { // Only control automatically if not overridden
            if (aht_get_data(&aht20_dev, &temperature, &humidity) == ESP_OK)
            {
                ESP_LOGI(TAG, "Temperature: %.2f °C, Humidity: %.2f %%", temperature,
                         humidity);

                if (temperature > TEMP_THRESHOLD && !relay_status)
                {
                    relay_status = true;
                    gpio_set_level(RELAY_PIN, 1); // Turn fan ON
                    ESP_LOGI(TAG, "Fan turned ON (Temperature: %.2f °C)", temperature);
                }
                else if (temperature <= TEMP_THRESHOLD && relay_status)
                {
                    relay_status = false;
                    gpio_set_level(RELAY_PIN, 0); // Turn fan OFF
                    ESP_LOGI(TAG, "Fan turned OFF (Temperature: %.2f °C)", temperature);
                }
            }
            else
            {
                ESP_LOGE(TAG, "Failed to read data from AHT20");
            }
        }
        else
        {
            ESP_LOGI(TAG, "Manual override active. Sensor control disabled.");
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds
    }
}

void i2c_scan()
{
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "I2C device found at address 0x%02X", addr);
        }
    }
}

// Initialize the sensor and GPIO
void sensor_init()
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &i2c_config));

    esp_err_t ret = i2c_driver_install(I2C_PORT, i2c_config.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return;
    }

    i2c_scan(); // Optional: Scan I2C bus for devices

    vTaskDelay(pdMS_TO_TICKS(100)); // Allow I2C bus to stabilize

    ret = aht_init_desc(&aht20_dev, AHT20_ADDR, I2C_PORT, I2C_MASTER_SDA_IO, I2C_MAS TER_SCL_IO);
    if (ret != ESP_OK)
    {

        ESP_LOGE(TAG, "Failed to initialize AHT20 descriptor: %s",
                 esp_err_to_name(ret));
        return;
    }

    ret = aht_init(&aht20_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize AHT20 sensor: %s", esp_err_to_name(ret));
        return;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    gpio_set_level(RELAY_PIN, 0); // Ensure fan starts OFF
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    wifi_init();
    espnow_init();
    sensor_init();

    manual_override_timer = xTimerCreate("ManualOverrideTimer", pdMS_TO_TICKS(RE SET_TIMEOUT_MS), pdFALSE, NULL, reset_manual_override);
    if (!manual_override_timer)
    {
        ESP_LOGE(TAG, "Failed to create manual override timer");
    }

    xTaskCreate(fan_control_task, "Fan Control Task", 4096, NULL, 3, NULL);
}
