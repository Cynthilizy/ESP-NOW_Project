#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

// GPIO pins for LCD
#define LCD_RS GPIO_NUM_9
#define LCD_EN GPIO_NUM_10
#define LCD_D4 GPIO_NUM_11
#define LCD_D5 GPIO_NUM_12
#define LCD_D6 GPIO_NUM_13
#define LCD_D7 GPIO_NUM_14

// GPIO pins for Buttons
#define BUTTON_GPIO_1 GPIO_NUM_4 // Cycle status
#define BUTTON_GPIO_2 GPIO_NUM_5 // Fan control
#define BUTTON_GPIO_3 GPIO_NUM_6 // Light control
#define BUTTON_GPIO_4 GPIO_NUM_7 // Curtain control

// LCD Commands
#define LCD_CMD_CLEAR 0x01
#define LCD_CMD_HOME 0x02
#define LCD_CMD_FUNCTION_SET_4BIT 0x28
#define LCD_CMD_SET_CURSOR_2 0xC0

#define PING_MSG "PING"
#define PING_TIMEOUT_MS 2000
#define TAG "ESP-NOW MASTER"

// Slave MAC addresses (Replace with actual MAC addresses)
uint8_t slave_mac_addresses[3][6] = {
    {0x34, 0x85, 0x18, 0xA1, 0xF6, 0x58}, // Fan MAC address
    {0x34, 0x85, 0x18, 0xA1, 0x5A, 0xA8}, // Light MAC address
    {0x34, 0x85, 0x18, 0xA1, 0x58, 0x84}  // Curtain MAC address
};

char fan_status[20] = "Fan Off";
char light_status[20] = "Light Off";
char curtain_status[20] = "Curtain Down";

int current_status_index = 0;
char display_buffer[32];
bool manual_override = false;

typedef enum
{
    FAN,
    LIGHT,
    CURTAIN
} slave_t;
char slave_status[3][20] = {"Fan", "Light", "Curtain"};
static bool slave_responses[3] = {false, false, false};
QueueHandle_t lcd_queue;

// LCD Functions
void lcd_enable_pulse()
{
    gpio_set_level(LCD_EN, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(LCD_EN, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void lcd_send_nibble(uint8_t nibble)
{
    gpio_set_level(LCD_D4, (nibble >> 0) & 0x01);
    gpio_set_level(LCD_D5, (nibble >> 1) & 0x01);
    gpio_set_level(LCD_D6, (nibble >> 2) & 0x01);
    gpio_set_level(LCD_D7, (nibble >> 3) & 0x01);
    lcd_enable_pulse();
}

void lcd_send_byte(uint8_t byte, int is_data)
{
    gpio_set_level(LCD_RS, is_data);
    lcd_send_nibble(byte >> 4);   // Send higher nibble
    lcd_send_nibble(byte & 0x0F); // Send lower nibble
}

void lcd_command(uint8_t cmd)
{
    lcd_send_byte(cmd, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void lcd_write_char(char data)
{
    lcd_send_byte(data, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void lcd_clear()
{
    lcd_command(LCD_CMD_CLEAR);
    vTaskDelay(25 / portTICK_PERIOD_MS);
}

void lcd_init()
{
    gpio_set_direction(LCD_RS, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_EN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_D4, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_D5, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_D6, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_D7, GPIO_MODE_OUTPUT);

    vTaskDelay(100 / portTICK_PERIOD_MS);
    lcd_send_nibble(0x03);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    lcd_send_nibble(0x03);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    lcd_send_nibble(0x03);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    lcd_send_nibble(0x02);

    lcd_command(LCD_CMD_FUNCTION_SET_4BIT);
    lcd_command(LCD_CMD_CLEAR);
    lcd_command(LCD_CMD_HOME);
}

void lcd_print(const char *str)
{
    int len = strlen(str);

    // First 16 characters on the first row
    for (int i = 0; i < len; i++)
    {
        if (i == 16)
        {
            lcd_command(LCD_CMD_SET_CURSOR_2); // Move to second row
        }
        lcd_write_char(str[i]);
    }
}

void init_wifi()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi initialized in STA mode");
}

/// ESP-NOW Receive Callback
void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len)
{
    // Log the received data and MAC address
    ESP_LOGI("ESPNOW", "Received data from MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);

    if (strncmp((char *)data, "PONG", data_len) == 0)
    {
        for (int i = 0; i < 3; i++)
        {
            if (memcmp(recv_info->src_addr, slave_mac_addresses[i], ESP_NOW_ETH_ALEN) == 0)
            {
                slave_responses[i] = true;
                ESP_LOGI("ESPNOW", "Received PONG from %s", slave_status[i]);
                break;
            }
        }
        return;
    }

    // Update slave statuses based on MAC address
    if (memcmp(recv_info->src_addr, slave_mac_addresses[0], ESP_NOW_ETH_ALEN) == 0)
    {
        // Copy data into fan_status
        strncpy(fan_status, (char *)data, sizeof(fan_status) - 1);
        fan_status[sizeof(fan_status) - 1] = '\0'; // Ensure null termination
    }
    else if (memcmp(recv_info->src_addr, slave_mac_addresses[1], ESP_NOW_ETH_ALEN) == 0)
    {
        // Copy data into light_status
        strncpy(light_status, (char *)data, sizeof(light_status) - 1);
        light_status[sizeof(light_status) - 1] = '\0'; // Ensure null termination
    }
    else if (memcmp(recv_info->src_addr, slave_mac_addresses[2], ESP_NOW_ETH_ALEN) == 0)
    {
        // Copy data into curtain_status
        strncpy(curtain_status, (char *)data, sizeof(curtain_status) - 1);
        curtain_status[sizeof(curtain_status) - 1] = '\0'; // Ensure null termination
    }

    // Optionally log the received data (this is just for debugging)
    ESP_LOGI("ESPNOW", "Received data: %.*s", data_len, (char *)data);
}

// LCD Update Task
void lcd_task(void *pvParameters)
{
    char buffer[64];
    while (1)
    {
        if (xQueueReceive(lcd_queue, buffer, portMAX_DELAY))
        {
            ESP_LOGI("LCD_TASK", "Received data for LCD: %s", buffer); // Log the data received from the queue
            lcd_clear();
            lcd_print(buffer);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // Optional: Add delay if necessary
    }
}

// Function to send data to a specific slave
esp_err_t send_data_to_slave(const uint8_t *peer_addr, const char *data, size_t len)
{
    esp_err_t err = esp_now_send(peer_addr, (const uint8_t *)data, len);
    if (err != ESP_OK)
    {
        printf("Error sending data to slave: %d\n", err);
    }
    return err;
}

void automatic_cycle_task(void *pvParameters)
{
    while (1)
    {
        // Wait for 5 seconds before updating the cycle
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5000 ms = 5 seconds

        // Automatically cycle through the statuses
        if (manual_override)
        {
            continue; // Skip the cycle if a button press was detected
        }

        current_status_index = (current_status_index + 1) % 3;

        switch (current_status_index)
        {
        case 0:
            snprintf(display_buffer, sizeof(display_buffer), "Fan: %s", fan_status);
            break;
        case 1:
            snprintf(display_buffer, sizeof(display_buffer), "Light: %s", light_status);
            break;
        case 2:
            snprintf(display_buffer, sizeof(display_buffer), "Curtain: %s", curtain_status);
            break;
        }
        // Send updated display info to the LCD queue
        if (xQueueSend(lcd_queue, display_buffer, portMAX_DELAY) != pdPASS)
        {
            ESP_LOGW("QUEUE", "Queue send failed");
        }
    }
}

// Button Task (Manages Button Presses)
void button_task(void *pvParameters)
{
    gpio_set_direction(BUTTON_GPIO_1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO_1, GPIO_PULLUP_ONLY);

    gpio_set_direction(BUTTON_GPIO_2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO_2, GPIO_PULLUP_ONLY);

    gpio_set_direction(BUTTON_GPIO_3, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO_3, GPIO_PULLUP_ONLY);

    gpio_set_direction(BUTTON_GPIO_4, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO_4, GPIO_PULLUP_ONLY);

    while (1)
    {
        if (!gpio_get_level(BUTTON_GPIO_1))
        {
            vTaskDelay(100 / portTICK_PERIOD_MS); // Debounce delay
            if (!gpio_get_level(BUTTON_GPIO_1))   // Confirm press
            {
                manual_override = true;
                current_status_index = (current_status_index + 1) % 3;

                switch (current_status_index)
                {
                case 0:
                    snprintf(display_buffer, sizeof(display_buffer), "Fan: %s", fan_status);
                    break;
                case 1:
                    snprintf(display_buffer, sizeof(display_buffer), "Light: %s", light_status);
                    break;
                case 2:
                    snprintf(display_buffer, sizeof(display_buffer), "Curtain: %s", curtain_status);
                    break;
                }
                ESP_LOGI("BUTTON_TASK", "Button 1 pressed: %s", display_buffer);
                if (xQueueSend(lcd_queue, display_buffer, portMAX_DELAY) != pdPASS)
                {
                    ESP_LOGW("QUEUE", "Queue send failed");
                }
            }
            while (!gpio_get_level(BUTTON_GPIO_1))
            {
                vTaskDelay(50 / portTICK_PERIOD_MS); // Wait for release
            }
            manual_override = false;
        }
        // Button 2: Fan Control
        if (!gpio_get_level(BUTTON_GPIO_2))
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (!gpio_get_level(BUTTON_GPIO_2))
            {
                ESP_LOGI("BUTTON_TASK", "Button 2 pressed: Fan Control");
                if (xQueueSend(lcd_queue, "Fan Control Pressed", portMAX_DELAY) != pdPASS)
                {
                    ESP_LOGW("QUEUE", "Queue send failed");
                }
                send_data_to_slave(slave_mac_addresses[0], "Fan Control Pressed", 19);
            }
            while (!gpio_get_level(BUTTON_GPIO_2))
            {
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
        }
        // Button 3: Light Control
        if (!gpio_get_level(BUTTON_GPIO_3))
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (!gpio_get_level(BUTTON_GPIO_3))
            {
                ESP_LOGI("BUTTON_TASK", "Button 3 pressed: Light Control");
                if (xQueueSend(lcd_queue, "Light Control Pressed", portMAX_DELAY) != pdPASS)
                {
                    ESP_LOGW("QUEUE", "Queue send failed");
                }
                send_data_to_slave(slave_mac_addresses[1], "Light Control Pressed", 22);
            }
            while (!gpio_get_level(BUTTON_GPIO_3))
            {
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
        }
        // Button 4: Curtain Control
        if (!gpio_get_level(BUTTON_GPIO_4))
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (!gpio_get_level(BUTTON_GPIO_4))
            {
                ESP_LOGI("BUTTON_TASK", "Button 4 pressed: Curtain Control");
                if (xQueueSend(lcd_queue, "Curtain Control Pressed", portMAX_DELAY) != pdPASS)
                {
                    ESP_LOGW("QUEUE", "Queue send failed");
                }
                send_data_to_slave(slave_mac_addresses[2], "Curtain Control Pressed", 23);
            }
            while (!gpio_get_level(BUTTON_GPIO_4))
            {
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to reduce task CPU usage
    }
}

esp_err_t init_espnow()
{
    // Initialize ESPNOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_LOGI("ESPNOW", "ESP-NOW initialized successfully");

    // Register receive callback
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));

    return ESP_OK;
}

void initialize_connections()
{
    char buffer[64];

    for (int i = 0; i < 3; i++)
    {
        snprintf(buffer, sizeof(buffer), "Connecting to %s...", slave_status[i]);
        xQueueSend(lcd_queue, buffer, portMAX_DELAY);
        ESP_LOGI("INIT", "%s", buffer);

        // Configure the peer
        esp_now_peer_info_t peer_info = {};
        peer_info.channel = 0; // Default ESP-NOW channel
        peer_info.encrypt = false;
        memcpy(peer_info.peer_addr, slave_mac_addresses[i], 6);

        if (esp_now_add_peer(&peer_info) == ESP_OK)
        {
            // Send a PING message to the slave
            if (send_data_to_slave(slave_mac_addresses[i], PING_MSG, strlen(PING_MSG)) == ESP_OK)
            {
                // Wait for a response within the timeout period
                TickType_t start_time = xTaskGetTickCount();
                while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(PING_TIMEOUT_MS))
                {
                    if (slave_responses[i]) // Check if the response was received
                    {
                        snprintf(buffer, sizeof(buffer), "%s connected", slave_status[i]);
                        xQueueSend(lcd_queue, buffer, portMAX_DELAY);
                        ESP_LOGI("INIT", "%s connected", slave_status[i]);
                        break;
                    }
                    vTaskDelay(100 / portTICK_PERIOD_MS); // Short delay to avoid busy waiting
                }

                if (!slave_responses[i])
                {
                    snprintf(buffer, sizeof(buffer), "%s connection failed", slave_status[i]);
                    xQueueSend(lcd_queue, buffer, portMAX_DELAY);
                    ESP_LOGW("INIT", "%s connection failed", slave_status[i]);
                }
            }
            else
            {
                ESP_LOGE("INIT", "Failed to send PING to %s", slave_status[i]);
            }
        }
        else
        {
            ESP_LOGE("INIT", "Failed to add peer for %s", slave_status[i]);
        }
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
    ESP_ERROR_CHECK(ret);

    // Initialize Wi-Fi and ESPNOW
    init_wifi();
    lcd_init();
    lcd_queue = xQueueCreate(10, sizeof(char[64]));
    if (lcd_queue == NULL)
    {
        ESP_LOGE("MAIN", "Queue creation failed");
        return;
    }

    xQueueSend(lcd_queue, "ESP-NOW MASTER", portMAX_DELAY);
    ESP_LOGI("MAIN", "LCD initialized, displaying 'ESP-NOW MASTER'");

    // Initialize ESP-NOW after Wi-Fi is properly initialized
    ESP_LOGI("MAIN", "Initializing ESPNOW...");
    init_espnow();

    // Initialize connections with peers
    initialize_connections();

    // Start the tasks for handling button presses and LCD updates
    xTaskCreate(lcd_task, "LCD Task", 4096, NULL, 1, NULL);
    xTaskCreate(button_task, "Button Task", 4096, NULL, 2, NULL);
    xTaskCreate(automatic_cycle_task, "Automatic Cycle Task", 4096, NULL, 3, NULL);
}