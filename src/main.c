#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" // Added for queue support
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <string.h>
#include <math.h> // Added for sqrt and other math functions
#include "../include/routes/route_common.h"
#include "../include/sensor.h"

#define TAG "SERIAL_COMM"
#define FREEFALL_TAG "FREEFALL"

// UART configuration
#define UART_NUM UART_NUM_2
#define TXD_PIN 17
#define RXD_PIN 16
#define UART_BAUD_RATE 115200
#define BUF_SIZE 1024

// Packet constants
#define REQUEST_BYTE 0x01
#define RESPONSE_BYTE 0xAA
#define PACKET_SIZE 32

// Freefall detection constants
#define FREEFALL_THRESHOLD 0.3f  // Acceleration threshold for freefall in g (typically 0.2-0.4g)
#define FREEFALL_DURATION_MS 200 // Duration needed to confirm freefall (in ms)
#define LED_PIN 2                // Onboard LED pin for ESP32 devkit

// Global queue for sensor data
QueueHandle_t sensor_data_queue;

// Global variables for sharing sensor data with HTTP server
sensor_packet_t g_latest_sensor_data;
bool g_sensor_data_ready = false;

// WiFi credentials - change these to your network credentials
#define WIFI_SSID "ATTBsQU53C"
#define WIFI_PASS "rjf2y#2dp+h+"
#define MAXIMUM_RETRY 5

static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

// Union for float conversion
typedef union
{
      float f;
      uint8_t bytes[4];
} float_bytes_t;

// Convert from q4_11 to float
float convert_q4_11_to_float(int16_t value)
{
      return (float)value / 2048.0f; // 2^11 = 2048
}

// Initialize UART
void init_uart()
{
      const uart_config_t uart_config = {
          .baud_rate = UART_BAUD_RATE,
          .data_bits = UART_DATA_8_BITS,
          .parity = UART_PARITY_DISABLE,
          .stop_bits = UART_STOP_BITS_1,
          .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
          .source_clk = UART_SCLK_APB,
      };

      ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
      ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
      ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// Calculate checksum for the request packet
uint8_t calculate_request_checksum(uint8_t data)
{
      return data % 256;
}

// Calculate checksum for the received packet
uint8_t calculate_packet_checksum(uint8_t *packet, size_t len)
{
      uint16_t sum = 0;
      for (size_t i = 0; i < len - 1; i++)
      {
            sum += packet[i];
      }
      return sum % 256;
}

// Verify checksum of received packet
bool verify_checksum(uint8_t *packet, size_t len)
{
      uint8_t calculated_checksum = calculate_packet_checksum(packet, len);

      return calculated_checksum == packet[len - 1];
}

// Parse received packet into sensor data structure (little endian format)
void parse_sensor_packet(uint8_t *packet, sensor_packet_t *sensor_data)
{
      if (packet[0] != RESPONSE_BYTE)
      {
            ESP_LOGE(TAG, "Invalid start byte: 0x%02X", packet[0]);
            return;
      }

      // Parse int16_t values (low byte then high byte)
      sensor_data->accel_x = (int16_t)((packet[2] << 8) | packet[1]);
      sensor_data->accel_y = (int16_t)((packet[4] << 8) | packet[3]);
      sensor_data->accel_z = (int16_t)((packet[6] << 8) | packet[5]);

      // Parse temperature (float)
      float_bytes_t temp;
      temp.bytes[0] = packet[7];
      temp.bytes[1] = packet[8];
      temp.bytes[2] = packet[9];
      temp.bytes[3] = packet[10];
      sensor_data->temperature = temp.f;

      // Parse more int16_t values
      sensor_data->mag_x = (int16_t)((packet[12] << 8) | packet[11]);
      sensor_data->mag_y = (int16_t)((packet[14] << 8) | packet[13]);
      sensor_data->mag_z = (int16_t)((packet[16] << 8) | packet[15]);

      sensor_data->mag_angle_heading = (int16_t)((packet[18] << 8) | packet[17]);
      sensor_data->mag_angle_pitch = (int16_t)((packet[20] << 8) | packet[19]);
      sensor_data->mag_angle_roll = (int16_t)((packet[22] << 8) | packet[21]);

      sensor_data->light_ch0 = (uint16_t)((packet[24] << 8) | packet[23]);
      sensor_data->light_ch1 = (uint16_t)((packet[26] << 8) | packet[25]);

      // Parse light_lux (float)
      float_bytes_t lux;
      lux.bytes[0] = packet[27];
      lux.bytes[1] = packet[28];
      lux.bytes[2] = packet[29];
      lux.bytes[3] = packet[30];
      sensor_data->light_lux = lux.f;

      sensor_data->checksum = packet[31];
}

// Print sensor data
void print_sensor_data(sensor_packet_t *data)
{
      // Convert accelerometer values from q4_11 format to g units
      float accel_x = convert_q4_11_to_float(data->accel_x);
      float accel_y = convert_q4_11_to_float(data->accel_y);
      float accel_z = convert_q4_11_to_float(data->accel_z);

      ESP_LOGI(TAG, "Accelerometer: X=%.4f g, Y=%.4f g, Z=%.4f g (raw: %d, %d, %d)",
               accel_x, accel_y, accel_z,
               data->accel_x, data->accel_y, data->accel_z);
      ESP_LOGI(TAG, "Temperature: %.2f°C", data->temperature);
      ESP_LOGI(TAG, "Magnetometer: X=%d, Y=%d, Z=%d",
               data->mag_x, data->mag_y, data->mag_z);
      ESP_LOGI(TAG, "Mag Angles: Heading=%d°, Pitch=%d°, Roll=%d°",
               data->mag_angle_heading, data->mag_angle_pitch, data->mag_angle_roll);
      ESP_LOGI(TAG, "Light: CH0=%u, CH1=%u, Lux=%.2f",
               data->light_ch0, data->light_ch1, data->light_lux);
}

// Process the received packet
void process_packet(uint8_t *buffer, sensor_packet_t *sensor_data)
{
      // Verify checksum
      if (verify_checksum(buffer, PACKET_SIZE))
      {
            // Parse and print sensor data
            parse_sensor_packet(buffer, sensor_data);
            // print_sensor_data(sensor_data);

            // Update global sensor data for HTTP server
            memcpy(&g_latest_sensor_data, sensor_data, sizeof(sensor_packet_t));
            g_sensor_data_ready = true;

            // Send the sensor data to the freefall detection task
            if (sensor_data_queue != NULL)
            {
                  // Use xQueueSendToBack with a short timeout to avoid blocking if queue is full
                  if (xQueueSendToBack(sensor_data_queue, sensor_data, pdMS_TO_TICKS(10)) != pdPASS)
                  {
                        ESP_LOGW(TAG, "Failed to send data to freefall detection task - queue full");
                  }
            }
      }
      else
      {
            ESP_LOGE(TAG, "Checksum verification failed!");
      }
}

// Get expected packet length based on start byte
uint8_t expected_packet_length(uint8_t start_byte)
{
      if (start_byte == RESPONSE_BYTE)
      {
            return PACKET_SIZE;
      }
      return 0; // Unknown start byte
}

// Serial communication task
void serial_comm_task(void *pvParameters)
{
      // Request message: 0xAA + checksum
      uint8_t request[2] = {REQUEST_BYTE, calculate_request_checksum(REQUEST_BYTE)};
      uint8_t rx_buffer[BUF_SIZE];
      sensor_packet_t sensor_data = {0};

      while (1)
      {
            // Send request packet
            uart_write_bytes(UART_NUM, (const char *)request, sizeof(request));

            // Clear buffer
            memset(rx_buffer, 0, BUF_SIZE);

            // Read and process incoming bytes
            uint8_t idx = 0;
            int c;
            uint8_t byte_received = 0;
            uint8_t expected_length = 0;

            // Set a timeout for reading all bytes
            TickType_t start_time = xTaskGetTickCount();

            while (xTaskGetTickCount() - start_time < pdMS_TO_TICKS(500))
            {
                  // Read one byte with a short timeout
                  byte_received = 0;

                  if (uart_read_bytes(UART_NUM, &c, 1, pdMS_TO_TICKS(100)) == 1)
                  {
                        rx_buffer[idx++] = (uint8_t)c;
                        byte_received = 1;

                        // Check if we've received the expected packet length
                        expected_length = expected_packet_length(rx_buffer[0]);
                        if (expected_length > 0 && idx == expected_length)
                        {
                              process_packet(rx_buffer, &sensor_data);
                              break; // Exit the read loop after processing
                        }
                  }

                  // If we've started receiving a packet but then got no data for a while, abort
                  if (idx > 0 && !byte_received)
                  {
                        // Small delay before checking again to avoid hammering the CPU
                        vTaskDelay(1 / portTICK_PERIOD_MS);
                  }
            }

            // Check if we didn't complete a packet
            if (idx > 0 && idx < expected_length)
            {
                  ESP_LOGW(TAG, "Incomplete packet received (%d bytes)", idx);
                  printf("\n");
            }
            else if (idx == 0)
            {
                  ESP_LOGW(TAG, "No response received within timeout period");
            }

            // Wait for 0.1 second before sending next request
            vTaskDelay(100 / portTICK_PERIOD_MS);
      }
}

// Function declarations for freefall detection
void freefall_detection_task(void *pvParameters);
bool is_in_freefall(sensor_packet_t *data);
void init_freefall_detection(void);

// Freefall detection task
void freefall_detection_task(void *pvParameters)
{
      sensor_packet_t sensor_data;

      while (1)
      {
            // Wait for sensor data from the queue
            if (xQueueReceive(sensor_data_queue, &sensor_data, portMAX_DELAY))
            {
                  // Check if the device is in freefall
                  if (is_in_freefall(&sensor_data))
                  {
                        // Blink the LED to indicate freefall
                        ESP_LOGI(FREEFALL_TAG, "Freefall detected! Blinking LED...");
                        gpio_set_level(LED_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        gpio_set_level(LED_PIN, 0);
                  }
            }
      }
}

// Initialize freefall detection
void init_freefall_detection(void)
{
      // Create the queue for sensor data
      sensor_data_queue = xQueueCreate(10, sizeof(sensor_packet_t));

      // Create the freefall detection task
      xTaskCreate(freefall_detection_task, "freefall_detection_task", 2048, NULL, 5, NULL);

      // Initialize LED GPIO
      gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

// Check if the device is in freefall
bool is_in_freefall(sensor_packet_t *data)
{
      // Calculate the acceleration in g's
      float accel_x = convert_q4_11_to_float(data->accel_x);
      float accel_y = convert_q4_11_to_float(data->accel_y);
      float accel_z = convert_q4_11_to_float(data->accel_z);

      // Calculate the total acceleration
      float total_accel = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

      // Check if the total acceleration is below the freefall threshold
      return (total_accel < FREEFALL_THRESHOLD);
}

// HTTP server handle
httpd_handle_t server = NULL;

// Forward declaration for HTTP server start function (defined in server.c)
httpd_handle_t start_webserver(void);

// Event handler for WiFi events
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
      if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
      {
            esp_wifi_connect();
      }
      else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
      {
            if (s_retry_num < MAXIMUM_RETRY)
            {
                  esp_wifi_connect();
                  s_retry_num++;
                  ESP_LOGI(TAG, "Retry to connect to the AP");
            }
            else
            {
                  xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
            ESP_LOGI(TAG, "Connect to the AP failed");
      }
      else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
      {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
            s_retry_num = 0;
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
      }
}

// Initialize WiFi as a station
void wifi_init_sta(void)
{
      s_wifi_event_group = xEventGroupCreate();

      // Initialize the network interface (TCP/IP stack)
      ESP_ERROR_CHECK(esp_netif_init());
      ESP_ERROR_CHECK(esp_event_loop_create_default());
      esp_netif_create_default_wifi_sta();

      // Initialize Wi-Fi with default configuration
      wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
      ESP_ERROR_CHECK(esp_wifi_init(&cfg));

      // Register event handlers
      ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
      ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

      // Configure WiFi station with SSID and password
      wifi_config_t wifi_config = {
          .sta = {
              .ssid = WIFI_SSID,
              .password = WIFI_PASS,
              .threshold.authmode = WIFI_AUTH_WPA2_PSK,
              .pmf_cfg = {
                  .capable = true,
                  .required = false},
          },
      };

      // Set WiFi mode as station and configure
      ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
      ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

      // Start WiFi
      ESP_ERROR_CHECK(esp_wifi_start());

      ESP_LOGI(TAG, "wifi_init_sta finished.");

      // Wait until either the connection is established or connection failed
      EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                             WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                             pdFALSE,
                                             pdFALSE,
                                             portMAX_DELAY);

      // Handle the connection result
      if (bits & WIFI_CONNECTED_BIT)
      {
            ESP_LOGI(TAG, "Connected to AP SSID:%s", WIFI_SSID);

            // Start the HTTP server after WiFi is connected
            server = start_webserver();
      }
      else if (bits & WIFI_FAIL_BIT)
      {
            ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
      }
      else
      {
            ESP_LOGE(TAG, "Unexpected event");
      }
}

void app_main(void)
{
      // Initialize NVS
      esp_err_t ret = nvs_flash_init();
      if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
      {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
      }
      ESP_ERROR_CHECK(ret);
      ESP_LOGI(TAG, "NVS initialized");

      // Initialize UART
      init_uart();
      ESP_LOGI(TAG, "UART initialized");

      // Initialize freefall detection
      // init_freefall_detection();
      // ESP_LOGI(TAG, "Freefall detection initialized");

      // Initialize WiFi
      wifi_init_sta();
      ESP_LOGI(TAG, "WiFi initialization started");

      // Create task for serial communication
      xTaskCreate(serial_comm_task, "serial_comm_task", 4096, NULL, 5, NULL);
      ESP_LOGI(TAG, "Serial communication task started");
}
