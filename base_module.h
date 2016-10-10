#ifndef _BASE_MODULE_H
#define _BASE_MODULE_H

#include <pthread.h>


typedef struct astruct {
  long counterA, counterB;
  short velocityA, velocityB; 
  short dist1, dist2, dist3, cube, heading, ax, ay, az, gx, gy, gz;
} base_data_type;

void init_base_module();
void get_base_data(base_data_type *buffer);

void set_motor_speeds(int left_motor, int right_motor);
void stop_now();
void follow_azimuth(int azimuth);
void reset_counters();

void regulated_speed(int left_motor, int right_motor);

short angle_difference(short alpha, short beta);

#endif
