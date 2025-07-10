#ifndef ROUTE_COMMON_H
#define ROUTE_COMMON_H

#include "esp_http_server.h"

void register_sensor_route(httpd_handle_t server);
void register_status_route(httpd_handle_t server);
void register_control_route(httpd_handle_t server);

#endif