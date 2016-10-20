#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "mikes.h"
#include "mikes_logs.h"
#include "range_sensor.h"
#include "base_module.h"
#include "util.h"

#define CRITICAL_AVOIDING_DISTANCE     700
#define CRITICAL_OBSTACLE_MINIMUM_SIZE  30
#define CHANGE_PERIOD 7000000L

static short original_heading;

unsigned char obstacle_in_range_data(int *ranges)
{
  int starting = 1080 * 75 / 135 / 2;
  int ending = 1081 - starting;
  short segment = 0;
  short missing = 0;

  for (int i = starting; i < ending; i++)
  {
    unsigned char ray_hit = 0;
    if (ranges[i] < CRITICAL_AVOIDING_DISTANCE) ray_hit = 1;

    if (ray_hit) segment++;
    else if (segment) missing++;

    if (segment > CRITICAL_OBSTACLE_MINIMUM_SIZE) return 1;
    if ((missing > segment / 5) && (segment > 10)) { segment = 0; missing = 0; }
    if (missing > segment / 4) { segment = 0; missing = 0; }
  }
  return 0;
}

void turn_away_from_obstacle()
{
   base_data_type base_data;

   mikes_log(ML_INFO, "avoid");
   get_base_data(&base_data);
   short old_heading = base_data.heading;
   set_motor_speeds(20, -20);
   do {
     get_base_data(&base_data);
     //printf("%d %d -> diff %d\n", old_heading, base_data.heading, angle_difference(old_heading, base_data.heading));
     usleep(10000);
   } while (program_runs && (abs(angle_difference(old_heading, base_data.heading)) < 90));
  set_motor_speeds(20, 20);
}

void avoid_range_obstacle(int *ranges)
{
    if (obstacle_in_range_data(ranges))
      turn_away_from_obstacle();
}

void debug_navigation()
{
    while(1)
    {
        set_motor_speeds(30, 30);
        sleep(5);
        set_motor_speeds(-30, -30);
        sleep(5);
        stop_now();
        sleep(2);
    }
}

void *navigation_thread(void *arg)
{
    int ranges[RANGE_DATA_COUNT];
    base_data_type base_data;
    get_base_data(&base_data);
    original_heading = base_data.heading;

    mikes_log_val(ML_INFO, "original heading: ", original_heading);

    sleep(5);

    //debug_navigation();

    int attack = 1;
    long next_change = usec() + CHANGE_PERIOD;
    set_motor_speeds(20, 20);
    follow_azimuth(original_heading);
    mikes_log(ML_INFO, "navigate: put");

    while (program_runs)
    {
        get_range_data(ranges);
        avoid_range_obstacle(ranges);

        long tm = usec();
        if (tm > next_change)
        {
          attack = 1 - attack;
          follow_azimuth(attack?original_heading:((original_heading + 180) % 360));
          next_change = CHANGE_PERIOD + tm;
          mikes_log(ML_INFO, attack?"navigate: put":"navigate: fetch");
        }
    }
    mikes_log(ML_INFO, "navigation quits.");
    threads_running_add(-1);
    return 0;
}

void init_navigation()
{
    pthread_t t;
    if (pthread_create(&t, 0, navigation_thread, 0) != 0)
    {
        perror("mikes:navigation");
        mikes_log(ML_ERR, "creating navigation thread");
    }
    else threads_running_add(1);
}
