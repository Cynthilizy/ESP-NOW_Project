#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define TAG "ESP32_ESPNOW"
#define PING "PING"
#define PONG "PONG"
#define LIGHT_CONTROL_MSG "Light Control Pressed"
#define LIGHT_ON_MSG "Light On"
#define LIGHT_OFF_MSG "Light Off"

#define SENSOR_D0_PIN GPIO_NUM_1          // D0 connected to GPIO1
#define SENSOR_ADC_CHANNEL ADC1_CHANNEL_0 // GPIO1 for the sensor
#define POT_ADC_CHANNEL ADC1_CHANNEL_3    // GPIO4 for the potentiometer

static const uint8_t master_mac[6] = {0x34, 0x85, 0x18, 0xA1, 0x5D, 0xC0};
bool ON_Off_Flag = false;
static bool light_on = false;

bool manual_override = false;        // Flag to indicate manual control
bool manual_light_state = false;     // Current state of lights during manual control
TimerHandle_t manual_override_timer; // Timer to reset manual_override

#define SENSOR_PIN GPIO_NUM_36 // ADC1 channel 0 on ESP32
// #define POTENTIOMETER_PIN ADC1_CHANNEL_4  // ADC1 channel 4 for potentiometer
#define POTENTIOMETER_PIN GPIO_NUM_2 // GPIO2 for the potentiometer (ADC1_CHANNEL_4)

#define NUM_LEDS 5
const gpio_num_t LED_PINS[NUM_LEDS] = {GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_15, GPIO_NUM_16,
                                       GPIO_NUM_17};

void reset_manual_override(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Manual override reset. Resuming sensor control.");
    manual_override = false; // Reset manual override

    // Check if no motion is detected and turn off LEDs
    if (gpio_get_level(SENSOR_D0_PIN))
    {
        ESP_LOGI(TAG, "No motion detected. Turning LEDs OFF.");
        for (int i = NUM_LEDS - 1; i >= 0; i--)
        {
            gpio_set_level(LED_PINS[i], 0); // Turn OFF LED
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        light_on = false; // Update the global state

        const char *response = LIGHT_OFF_MSG;
        esp_err_t result = esp_now_send(master_mac, (uint8_t *)response, strlen(response) + 1);
        if (result == ESP_OK)
        {
            ESP_LOGI(TAG, "Sent to master: %s", response);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to send to master: %d", result);
        }
    }
    else
    {
        ESP_LOGI(TAG, "Motion detected during reset. LEDs remain under sensor control."); 
    }
}

void sensor_gpio_init()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SENSOR_D0_PIN), // Pin for D0
        .mode = GPIO_MODE_INPUT,                 // Set as input
        .pull_up_en = GPIO_PULLUP_DISABLE,       // No pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,   // No pull-down
        .intr_type = GPIO_INTR_DISABLE           // No interrupts
    };
    gpio_config(&io_conf);
}

void led_gpio_init()
{
    gpio_config_t io_conf;

    for (int i = 0; i < NUM_LEDS; i++)
    {
        io_conf.pin_bit_mask = (1ULL << LED_PINS[i]); // Configure each pin in the array
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;

        // Configure the GPIO pin
        esp_err_t err = gpio_config(&io_conf);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Configured GPIO %d as output", LED_PINS[i]);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", LED_PINS[i],
                     esp_err_to_name(err));
        }

        // Set the initial state of the LED to OFF
        gpio_set_level(LED_PINS[i], 0);
        ESP_LOGI(TAG, "Set GPIO %d to initial level 0 (OFF)", LED_PINS[i]);
    }

    ESP_LOGI(TAG, "All LEDs initialized.");
}

void receive_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info == NULL || data == NULL || len <= 0)
    {
        ESP_LOGE(TAG, "Invalid data received");
        return;
    }
    ESP_LOGI(TAG, "Received message: %.*s from MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             len, (char *)data,
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);

    if (strncmp((char *)data, PING, len) == 0)
    {
        ESP_LOGI(TAG, "PING received. Sending PONG...");

        esp_err_t result = esp_now_send(recv_info->src_addr, (uint8_t *)PONG,
                                        strlen(PONG) + 1);
        if (result == ESP_OK)
        {
            ESP_LOGI(TAG, "PONG sent successfully");
        }
        else
        {
            ESP_LOGE(TAG, "Error sending PONG: %d", result);
        }
        return; // Skip further processing for PING
    }

    if (strncmp((char *)data, LIGHT_CONTROL_MSG, len) == 0)
    {
        manual_override = true;                   // Enable manual override
        manual_light_state = !manual_light_state; // Toggle manual state
        light_on = manual_light_state;            // Ensure consistency with global state

        // Turn LEDs ON or OFF sequentially based on the new state
        if (manual_light_state)
        {
            ESP_LOGI(TAG, "Manual control: Turning LEDs ON sequentially");
            for (int i = 0; i < NUM_LEDS; i++)
            {
                gpio_set_level(LED_PINS[i], 1); // Turn ON LED
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }
        else
        {
            ESP_LOGI(TAG, "Manual control: Turning LEDs OFF sequentially");
            for (int i = NUM_LEDS - 1; i >= 0; i--)
            {
                gpio_set_level(LED_PINS[i], 0); // Turn OFF LED
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }

        // Respond to the master MAC with the new light state
        const char *response = manual_light_state ? LIGHT_ON_MSG : LIGHT_OFF_MSG;
        esp_err_t result = esp_now_send(recv_info->src_addr, (uint8_t *)response,
                                        strlen(response) + 1);
        if (result == ESP_OK)
        {
            ESP_LOGI(TAG, "Responded to master: %s", response);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to respond to master: %d", result);
        }

        // Reset the timer to extend manual override period
        if (manual_override_timer != NULL)
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

// Task to control LEDs based on sensor input
void led_control_task(void *pvParameter)
{
    bool previous_motion_detected = false; // Track the previous motion state

    while (1)
    {
        if (manual_override)
        {
            // Skip sensor-based control if manual override is active
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Read the D0 digital output
        bool motion_detected = !gpio_get_level(SENSOR_D0_PIN);

        if (motion_detected)
        {
            // Turn LEDs ON sequentially
            if (!previous_motion_detected)
            {

                ESP_LOGI(TAG, "Motion detected: Turning LEDs ON sequentially");
                for (int i = 0; i < NUM_LEDS; i++)
                {
                    gpio_set_level(LED_PINS[i], 1); // Turn ON LED
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                light_on = true;

                // Send update to master MAC
                const char *response = LIGHT_ON_MSG;
                esp_err_t result = esp_now_send(master_mac, (uint8_t *)response,
                                                strlen(response) + 1);
                if (result == ESP_OK)
                {
                    ESP_LOGI(TAG, "Sent to master: %s", response);
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to send to master: %d", result);
                }
            }
        }
        else if (!motion_detected && previous_motion_detected)
        {
            // Turn LEDs OFF sequentially
            ESP_LOGI(TAG, "No motion detected: Turning LEDs OFF sequentially");
            for (int i = NUM_LEDS - 1; i >= 0; i--)
            {
                gpio_set_level(LED_PINS[i], 0); // Turn OFF LED
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            light_on = false;

            // Send update to master MAC
            const char *response = LIGHT_OFF_MSG;
            esp_err_t result = esp_now_send(master_mac, (uint8_t *)response, strlen(response) + 1);
            if (result == ESP_OK)
            {
                ESP_LOGI(TAG, "Sent to master: %s", response);
            }
            else
            {
                ESP_LOGE(TAG, "Failed to send to master: %d", result);
            }
        }

        // Update the previous motion state
        previous_motion_detected = motion_detected;

        // Polling interval
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void adc_init()
{
    adc1_config_width(ADC_WIDTH_BIT_12);                            // 12-bit resolution
    adc1_config_channel_atten(SENSOR_ADC_CHANNEL, ADC_ATTEN_DB_11); // GPIO1
    adc1_config_channel_atten(POT_ADC_CHANNEL, ADC_ATTEN_DB_11);    // GPIO4
}

// Main application
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
    adc_init();
    sensor_gpio_init();
    led_gpio_init();

    const char *response = LIGHT_OFF_MSG;
    esp_err_t result = esp_now_send(master_mac, (uint8_t *)response, strlen(response) + 1);
    if (result == ESP_OK)
    {
        ESP_LOGI(TAG, "Initial state sent to master: %s", response);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to send initial state to master: %d", result);
    }

    manual_override_timer = xTimerCreate("ManualOverrideTimer", pdMS_TO_TICKS(15000),
                                         pdFALSE, NULL, reset_manual_override);
    if (manual_override_timer == NULL)
    {
        ESP_LOGE(TAG, "Failed to create manual override timer");
    }
    else
    {
        ESP_LOGI(TAG, "Manual override timer initialized");
    }

    // Create LED control task
    xTaskCreate(led_control_task, "LED Control Task", 4096, NULL, 5, NULL);
}