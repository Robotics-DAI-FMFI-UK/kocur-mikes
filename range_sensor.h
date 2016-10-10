#ifndef _RANGE_SENSOR_H
#define _RANGE_SENSOR_H

#include <pthread.h>

#define RANGE_DATA_COUNT 1081
#define HOKUYO_PORT 10940
#define HOKUYO_ADDR "169.254.0.10"

extern pthread_mutex_t range_sensor_lock;
extern int *range_data;

void init_range_sensor();
void get_range_data(int* buffer);

#endif
