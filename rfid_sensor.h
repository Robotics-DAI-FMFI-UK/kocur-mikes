#ifndef _RFID_SENSOR_H
#define _RFID_SENSOR_H

#include <pthread.h>

#define MAX_NUMBER_OF_TAGS 200

#define SICK_PORT 2112
#define SICK_ADDR "169.254.0.9"

typedef struct {
  short ntags;
  short x[MAX_NUMBER_OF_TAGS];
  short y[MAX_NUMBER_OF_TAGS];
  short a[MAX_NUMBER_OF_TAGS]; // 0 = free, 1 = player1, 2 = player2, 3 = field
} rfid_data_type;

void init_rfid_sensor();
void get_rfid_data(rfid_data_type *buffer);

#endif
