#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "mikes.h"
#include "mikes_logs.h"
#include "range_sensor.h"
#include "base_module.h"
#include "util.h"

#define CRITICAL_AVOIDING_DISTANCE       700
#define CRITICAL_OBSTACLE_MINIMUM_SIZE   30
#define AVOID_PERPENDICULAR_DISTANCE     MM2COUNTER(700)
#define NORMAL_NAVIGATION_SPEED          12
#define CHANGE_DISTANCE                  MM2COUNTER(1700)

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
   short old_heading = get_current_azimuth();
   get_base_data(&base_data);
   short old_dist = (base_data.counterA + base_data.counterB) / 2;
   follow_azimuth((old_heading + 90) % 360);
   do {
     get_base_data(&base_data);
     usleep(10000);
   } while (program_runs && (((base_data.counterA + base_data.counterB) / 2 - old_dist) < AVOID_PERPENDICULAR_DISTANCE));
  follow_azimuth(old_heading);
}

void avoid_range_obstacle(int *ranges)
{
    if (obstacle_in_range_data(ranges))
      turn_away_from_obstacle();
}

static int ranges[RANGE_DATA_COUNT];

void *navigation_thread(void *arg)
{
    base_data_type base_data;
    sleep(3);
    get_base_data(&base_data);
    original_heading = base_data.heading;
    mikes_log_val(ML_INFO, "original heading: ", original_heading);

    int attack = 1;
    reset_counters();
    set_motor_speeds(NORMAL_NAVIGATION_SPEED, NORMAL_NAVIGATION_SPEED);
    follow_azimuth(original_heading);
    mikes_log(ML_INFO, "navigate: put");

    while (program_runs)
    {
        get_range_data(ranges);
        avoid_range_obstacle(ranges);

        get_base_data(&base_data);
        long dist = (base_data.counterA + base_data.counterB) / 2;

        if (dist > CHANGE_DISTANCE)
        {
          attack = 1 - attack;
          follow_azimuth(attack?original_heading:((original_heading + 180) % 360));
          mikes_log(ML_INFO, attack?"navigate: put":"navigate: fetch");
          reset_counters();
          wait_for_new_base_data();
          wait_for_new_base_data();
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
