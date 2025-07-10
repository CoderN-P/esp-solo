#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

typedef struct sensor_packet_t
{
      int16_t accel_x;
      int16_t accel_y;
      int16_t accel_z;
      float temperature;
      int16_t mag_x;
      int16_t mag_y;
      int16_t mag_z;
      int16_t mag_angle_heading;
      int16_t mag_angle_pitch;
      int16_t mag_angle_roll;
      uint16_t light_ch0;
      uint16_t light_ch1;
      float light_lux;
      uint8_t checksum;
} sensor_packet_t;

#endif // SENSOR_H
