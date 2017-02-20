#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

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

#define STATE_INITIAL       1
#define STATE_SEARCHING     2
#define STATE_CATCHING      3
#define STATE_GOHOME        4
#define STATE_REPOSITIONING 5
#define STATE_USER_CONTROL  6

#define CUBE_THRESHOLD     310
#define CUBE_INERTIA       10

static short original_heading;
static unsigned char user_moving;

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

int cube_filter(base_data_type *base_data)
{
    static long discovered = 0;
    static long having_cube = 0;

    if ((base_data->cube < CUBE_THRESHOLD) == having_cube) 
    {
      long ms = msec();
      if ((ms - discovered) > CUBE_INERTIA * (20 - 19 * having_cube)) 
      {
        having_cube = 1 - having_cube;
        discovered = msec();
      }
    }
    else discovered = msec();
    return having_cube;
}

//static int ranges[RANGE_DATA_COUNT];

void *navigation_thread(void *arg)
{
    base_data_type base_data;
    sleep(3);
    get_base_data(&base_data);
    original_heading = base_data.heading;
    mikes_log_val(ML_INFO, "original heading: ", original_heading);

    static segments_type segments;
    int collect_state = STATE_INITIAL;
    reset_counters();
/*
    set_motor_speeds(NORMAL_NAVIGATION_SPEED, NORMAL_NAVIGATION_SPEED);
    follow_azimuth(original_heading);
    mikes_log(ML_INFO, "navigate: put");
*/
    long dist;
    int laststatus = -1;
    int current_dist = 8000;
    int had_cube = 0;
    time_t azimuth_noticed;
    time(&azimuth_noticed);

    while (!start_automatically) 
      usleep(10000);

    while (program_runs)
    {
        if (user_control && (collect_state != STATE_USER_CONTROL)) 
        { 
          stop_now();
          mikes_log(ML_INFO, "user in charge");
          collect_state = STATE_USER_CONTROL; 
        }
        else if ((!user_control) && (collect_state == STATE_USER_CONTROL))
        {
          collect_state = STATE_INITIAL;
          mikes_log(ML_INFO, "autonomous");
        }

        if (laststatus != collect_state)
        {
            mikes_log_val(ML_INFO, "status changed to:", collect_state);
            laststatus = collect_state;
        }
        get_base_data(&base_data);
        get_range_segments(&segments, 180*4, 145, 350);

        int have_cube = cube_filter(&base_data);
        if (have_cube != had_cube)
        {
          mikes_log_val(ML_INFO, "holding cube:", have_cube);
          had_cube = have_cube;
        }

        switch (collect_state)
        {
            case STATE_INITIAL: 
                reset_counters();
                set_motor_speeds(0,0);
                collect_state = STATE_SEARCHING;
                dist = 0;
                break;

            case STATE_SEARCHING: 
                if (segments.nsegs_found > 0) 
                { 
                    collect_state = STATE_CATCHING; 
                    current_dist = 8000;
                    time(&azimuth_noticed);
                    set_motor_speeds(NORMAL_NAVIGATION_SPEED, NORMAL_NAVIGATION_SPEED);
                    follow_azimuth(original_heading);
                }
                break;

            case STATE_CATCHING: 
                if (have_cube)
                {
                    dist = (base_data.counterA + base_data.counterB) / 2;
                    reset_counters();
                    follow_azimuth((original_heading + 180) % 360);
                    collect_state = STATE_GOHOME; 
                    break;
                }
                if (segments.nsegs_found == 0) 
                {
                  time_t tm;
                  time(&tm);
                  if (tm - azimuth_noticed > 2) collect_state = STATE_SEARCHING;
                  break;
                }
                int mindistID = 0;
                for (int i = 1; i < segments.nsegs_found; i++)
                    if (segments.dist[mindistID] > segments.dist[i])
                        mindistID = i;
                int suggested_azimuth = (base_data.heading + segments.alpha[mindistID] + 360) % 360;
                int suggested_dist = segments.dist[mindistID];
                if (abs(angle_difference(get_current_azimuth(), suggested_azimuth)) > 2)
                {
                  time_t tm;
                  time(&tm);
                  if ((suggested_dist < current_dist) || (tm - azimuth_noticed > 1))
                  {
                    current_dist = suggested_dist;
                    if (suggested_dist < 580) suggested_azimuth += 17;
                    follow_azimuth(suggested_azimuth);
                    time(&azimuth_noticed);
                    mikes_log_val2(ML_INFO, "new azimuth & dist: ", suggested_azimuth, suggested_dist);
                  }
                  /*
                  mikes_log_val(ML_INFO, "nsegs: ", segments.nsegs_found);
                  char segstr[500];
                  segstr[0] = 0;
                  for (int i = 0; i < segments.nsegs_found; i++)
                     sprintf(segstr + strlen(segstr), " %d(%d)", (base_data.heading + segments.alpha[i] + 360) % 360, 
                                                                 segments.dist[i]);
                  mikes_log(ML_INFO, segstr);
                  */
                }
                else 
                {
                  time(&azimuth_noticed);
                  current_dist = segments.dist[mindistID];
                }
                break;

            case STATE_GOHOME: 
                if (((base_data.counterA + base_data.counterB) / 2) >= dist + 110)
                {
                    reset_counters();
                    mikes_log(ML_INFO, "delivered, backing up");
                    regulated_speed(-27, -27);
                    do {
                      get_base_data(&base_data);
                      usleep(10000);
                    } while (program_runs && (((base_data.counterA + base_data.counterB) / 2) > -75));
                    mikes_log(ML_INFO, "turning towards more cubes");
                    follow_azimuth(original_heading);
                    collect_state = STATE_REPOSITIONING;
                }
                break;

            case STATE_REPOSITIONING:
                if (abs(angle_difference(base_data.heading, original_heading)) < 5)
                  collect_state = STATE_SEARCHING;
                break;

            case STATE_USER_CONTROL: 
                switch (user_dir)
                {
                    case USER_DIR_SPINRIGHT:
                             set_motor_speeds(NORMAL_NAVIGATION_SPEED, -NORMAL_NAVIGATION_SPEED);
			     user_moving = 1;
                             break; 
                    case USER_DIR_SPINLEFT:
                             set_motor_speeds(-NORMAL_NAVIGATION_SPEED, NORMAL_NAVIGATION_SPEED);
			     user_moving = 1;
                             break; 
                    case USER_DIR_RIGHT:  
                             set_motor_speeds(NORMAL_NAVIGATION_SPEED, NORMAL_NAVIGATION_SPEED);
                             follow_azimuth((base_data.heading + 30) % 360);
			     user_moving = 1;
			     //printf("follow %d\n", (base_data.heading + 30) % 360);
                             break;
                    case USER_DIR_LEFT: 
                             set_motor_speeds(NORMAL_NAVIGATION_SPEED, NORMAL_NAVIGATION_SPEED);
                             follow_azimuth((base_data.heading + 330) % 360);
			     user_moving = 1;
			     //printf("follow %d\n", (base_data.heading + 330) % 360);
                             break;
		    case USER_DIR_BACK: 
                             set_motor_speeds(NORMAL_NAVIGATION_SPEED, NORMAL_NAVIGATION_SPEED);
                             follow_azimuth((base_data.heading + 180) % 360);
			     //printf("follow %d\n", (base_data.heading + 180) % 360);
			     user_moving = 1;
                             break;
                    case USER_DIR_ONOFF:  
                             if (user_moving) 
                             {
                               stop_now();
                               user_moving = 0;
                             }
                             else {
                               set_motor_speeds(NORMAL_NAVIGATION_SPEED, NORMAL_NAVIGATION_SPEED);
                               follow_azimuth(base_data.heading);
			       //printf("follow %d\n", base_data.heading);
			       user_moving = 1;
                             }
                             break;
                    case USER_DIR_BACKUP:  
                             stop_now();
                             usleep(500000);
                             regulated_speed(-20, -20);
                             user_moving = 1;
                             break;
                }
                user_dir = 0;
                break;
        }
    /*        get_range_data(ranges);
            avoid_range_obstacle(ranges);

            get_base_data(&base_data);
            long dist = (base_data.counterA + base_data.counterB) / 2;

            if (dist > CHANGE_DISTANCE)
            {
              attack = 1 - attack;
              follow_azimuth(attack?original_heading:((original_heading + 180) % 360));
              mikes_log(ML_INFO, attack?"navigate: put":"navigate: fetch");
              reset_counters();
            }
    */
        usleep(1);
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
