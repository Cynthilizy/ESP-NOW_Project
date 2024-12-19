#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "time.h"

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microseconds
#define SERVO_MIN_DEGREE -90         // Minimum angle
#define SERVO_MAX_DEGREE 90          // Maximum angle
#define SERVO_PULSE_GPIO 12          // GPIO pin for PWM signal
#define STEP_DELAY_MS 10             // Delay between each step in milliseconds
#define STEP_DEGREE 5                // Degree to move per step
#define TAG "ESP32_ESPNOW"
#define PING "PING"
#define PONG "PONG"
#define CURTAIN_CONTROL_MSG "Curtain Control Pressed"
#define CURTAIN_UP_MSG "Curtain Up"
#define CURTAIN_DOWN_MSG "Curtain Down"

bool isTime = false;
static const uint8_t master_mac[6] = {0x34, 0x85, 0x18, 0xA1, 0x5D, 0xC0};
bool ON_Off_Flag = false;
static bool curtain_on = false;

bool manual_override = false;        // Flag to indicate manual control
bool manual_curtain_state = false;   // Current state of curtain during manual control
TimerHandle_t manual_override_timer; // Timer to reset manual_override

// Function to convert angle to pulse width
static inline uint32_t angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSE WIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

// Function to initialize MCPWM for servo control
void setupServo(mcpwm_timer_handle_t *timer, mcpwm_oper_handle_t *oper, mcpwm_cmpr_han dle_t *comparator, mcpwm_gen_handle_t *generator)
{
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, // 1 MHz resolution for 1 µs per tick
        .period_ticks = 20000,    // 20 ms period (50 Hz frequency)
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    esp_err_t err = mcpwm_new_timer(&timer_config, timer);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MCPWM timer: %d", err);
        return;
    }

    mcpwm_operator_config_t operator_config = {.group_id = 0};
    err = mcpwm_new_operator(&operator_config, oper);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MCPWM operator: %d", err);
        return;
    }
    mcpwm_operator_connect_timer(*oper, *timer);

    mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez = true};
    err = mcpwm_new_comparator(*oper, &comparator_config, comparator);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MCPWM comparator: %d", err);
        return;
    }

    mcpwm_generator_config_t generator_config = {.gen_gpio_num = SERVO_PULSE_GPIO};
    err = mcpwm_new_generator(*oper, &generator_config, generator);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MCPWM generator: %d", err);
        return;
    }

    mcpwm_generator_set_action_on_timer_event(*generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(*generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, *comparator, MCPWM_GEN_ACTION_LOW));
    mcpwm_timer_enable(*timer);
    mcpwm_timer_start_stop(*timer, MCPWM_TIMER_START_NO_STOP);
}

// Function to move the servo to the specified angle
void moveServo(mcpwm_cmpr_handle_t *comparator, int angle)
{

    uint32_t pulse_width = angle_to_compare(angle);
    mcpwm_comparator_set_compare_value(*comparator, pulse_width);
}

// Check if the current system time matches the target hour and minute
bool isTimeToAct(int hour, int minute)
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    return (timeinfo.tm_hour == hour && timeinfo.tm_min == minute);
}

// Callback function for received data
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
        return;
    }

    if (strncmp((char *)data, CURTAIN_CONTROL_MSG, len) == 0)
    {
        manual_override = true;
        manual_curtain_state = !manual_curtain_state;
        curtain_on = manual_curtain_state;

        if (manual_curtain_state)
        {
            ESP_LOGI(TAG, "Manual control: Turning Curtain UP sequentially");
            for (int i = 0; i < 10; i++)
            {
                if (isTimeToAct(6, 0))
                {
                    ESP_LOGI(TAG, "Curtain moving UP...");
                }
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
        else
        {
            ESP_LOGI(TAG, "Manual control: Turning Curtain DOWN sequentially");
            for (int i = 0; i < 10; i++)
            {

                if (isTimeToAct(18, 0))
                {
                    ESP_LOGI(TAG, "Curtain moving DOWN...");
                }
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }

        const char *response = manual_curtain_state ? CURTAIN_UP_MSG : CURTAIN_DOWN_MSG;
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

    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;

    setupServo(&timer, &oper, &comparator, &generator);

    manual_override_timer = xTimerCreate("ManualOverride", pdMS_TO_TICKS(60000),
                                         pdFALSE, NULL, NULL);
    if (manual_override_timer == NULL)
    {
        ESP_LOGE(TAG, "Failed to create manual override timer");
    }

    int angle = -90;
    int step = STEP_DEGREE;

    while (1)
    {
        moveServo(&comparator, angle);
        vTaskDelay(STEP_DELAY_MS / portTICK_PERIOD_MS);

        if (angle >= 90 || angle <= -90)
        {
            step = -step;
        }

        angle += step;
    }
}