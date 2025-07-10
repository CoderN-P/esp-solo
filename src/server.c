#include <string.h>
#include <stdlib.h>
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_log.h"
#include "cJSON.h"
#include "../include/routes/route_common.h"
#include "../include/sensor.h"

#define SERVER_TAG "HTTP_SERVER"

// External sensor data structure declaration
extern sensor_packet_t g_latest_sensor_data;
extern bool g_sensor_data_ready;

// Function to create a JSON response with sensor data
static char *create_sensor_json(sensor_packet_t *sensor_data)
{
      cJSON *root = cJSON_CreateObject();

      // Convert accelerometer values from q4_11 format to float
      float accel_x = (float)sensor_data->accel_x / 2048.0f;
      float accel_y = (float)sensor_data->accel_y / 2048.0f;
      float accel_z = (float)sensor_data->accel_z / 2048.0f;

      // Add accelerometer data
      cJSON *accel = cJSON_CreateObject();
      cJSON_AddNumberToObject(accel, "x", accel_x);
      cJSON_AddNumberToObject(accel, "y", accel_y);
      cJSON_AddNumberToObject(accel, "z", accel_z);
      cJSON_AddItemToObject(root, "accelerometer", accel);

      // Add temperature
      cJSON_AddNumberToObject(root, "temperature", sensor_data->temperature);

      // Add magnetometer data
      cJSON *mag = cJSON_CreateObject();
      // Raw mag data in q8_7 format
      cJSON_AddNumberToObject(mag, "x", sensor_data->mag_x/128.0f); 
      cJSON_AddNumberToObject(mag, "y", sensor_data->mag_y/128.0f);
      cJSON_AddNumberToObject(mag, "z", sensor_data->mag_z/128.0f);
      cJSON_AddItemToObject(root, "magnetometer", mag);

      // Add magnetometer angles
      cJSON *mag_angles = cJSON_CreateObject();
      cJSON_AddNumberToObject(mag_angles, "heading", sensor_data->mag_angle_heading);
      cJSON_AddNumberToObject(mag_angles, "pitch", sensor_data->mag_angle_pitch);
      cJSON_AddNumberToObject(mag_angles, "roll", sensor_data->mag_angle_roll);
      cJSON_AddItemToObject(root, "mag_angles", mag_angles);

      // Add light sensor data
      cJSON *light = cJSON_CreateObject();
      cJSON_AddNumberToObject(light, "ch0", sensor_data->light_ch0);
      cJSON_AddNumberToObject(light, "ch1", sensor_data->light_ch1);
      cJSON_AddNumberToObject(light, "lux", sensor_data->light_lux);
      cJSON_AddItemToObject(root, "light", light);

      // Convert to string
      char *json_str = cJSON_Print(root);
      cJSON_Delete(root);

      return json_str;
}

// Handler for the /sensors endpoint
static esp_err_t get_sensors_handler(httpd_req_t *req)
{
      esp_err_t res = ESP_OK;

      if (!g_sensor_data_ready)
      {
            // If no sensor data is available yet
            const char *no_data_msg = "{\"error\":\"No sensor data available yet\"}";
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, no_data_msg, strlen(no_data_msg));
            return ESP_OK;
      }

      // Create JSON response
      char *json_response = create_sensor_json(&g_latest_sensor_data);

      if (json_response)
      {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, json_response, strlen(json_response));
            free(json_response);
      }
      else
      {
            // Handle error creating JSON
            const char *error_msg = "{\"error\":\"Failed to create sensor data JSON\"}";
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, error_msg, strlen(error_msg));
      }

      return res;
}

// Register the sensor route with the HTTP server
void register_sensor_route(httpd_handle_t server)
{
      httpd_uri_t sensors_uri = {
          .uri = "/sensors",
          .method = HTTP_GET,
          .handler = get_sensors_handler,
          .user_ctx = NULL};

      ESP_LOGI(SERVER_TAG, "Registering URI handler for /sensors");
      httpd_register_uri_handler(server, &sensors_uri);
}

// Default handler for URI not found
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
      const char *error_msg = "Resource not found";
      httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, error_msg);
      return ESP_OK;
}

// Start the HTTP server
httpd_handle_t start_webserver(void)
{
      httpd_handle_t server = NULL;
      httpd_config_t config = HTTPD_DEFAULT_CONFIG();
      config.lru_purge_enable = true;

      ESP_LOGI(SERVER_TAG, "Starting server on port %d", config.server_port);

      if (httpd_start(&server, &config) == ESP_OK)
      {
            // Register URI handlers
            register_sensor_route(server);

            // Register error handler
            httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);

            ESP_LOGI(SERVER_TAG, "Server started successfully");
            return server;
      }

      ESP_LOGI(SERVER_TAG, "Error starting server");
      return NULL;
}

// Stop the HTTP server
void stop_webserver(httpd_handle_t server)
{
      if (server)
      {
            httpd_stop(server);
            ESP_LOGI(SERVER_TAG, "Server stopped");
      }
}
