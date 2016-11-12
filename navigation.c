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

//static int ranges[RANGE_DATA_COUNT];

void *navigation_thread(void *arg)
{
    base_data_type base_data;
    sleep(3);
    get_base_data(&base_data);
    original_heading = base_data.heading;
    mikes_log_val(ML_INFO, "original heading: ", original_heading);

    static segments_type segments;
    int status = 0;
    reset_counters();
/*
    set_motor_speeds(NORMAL_NAVIGATION_SPEED, NORMAL_NAVIGATION_SPEED);
    follow_azimuth(original_heading);
    mikes_log(ML_INFO, "navigate: put");
*/
    long dist;
    int lastdir;
    int laststatus = -1;

    while (!start_automatically) usleep(100000);

    while (program_runs)
    {
        if (user_control && (status != 6)) 
        { 
          stop_now();
          mikes_log(ML_INFO, "user in charge\n");
          status = 6; // USER CONTROL
        }
        else if ((!user_control) && (status == 6))
        {
          status = 0;
          mikes_log(ML_INFO, "autonomous\n");
        }

        if(laststatus != status){
            mikes_log_val(ML_INFO, "status changed to:",status);
            laststatus = status;
        }
        get_base_data(&base_data);
        get_range_segments(&segments, 180*4, 155, 350);
        switch(status){
            case 0: // STOP
                reset_counters();
                set_motor_speeds(0,0);
                status = 1;
                dist = 0;
                break;
            case 1: // searching
                if(segments.nsegs_found > 0){ // have something
                    status = 2; // catching
                    set_motor_speeds(NORMAL_NAVIGATION_SPEED, NORMAL_NAVIGATION_SPEED);
                }
                break;
            case 2: // catching
                if(!((base_data.cube < 210)&&(base_data.cube > 360)) || (segments.nsegs_found == 0)){ //HAVE IT? no SEE IT?
                    mikes_log_val(ML_INFO,"have it with value",base_data.cube);
                    dist = (base_data.counterA + base_data.counterB) / 2;
                    status = 3; // turn to home
                    break;
                }
                int mindistID = 0;
                for(int i=1; i<segments.nsegs_found; i++)
                    if(segments.dist[mindistID] > segments.dist[i])
                        mindistID = i;
                follow_azimuth(segments.alpha[mindistID]);
                lastdir=base_data.heading;
                break;
            case 3: // turn to home
                if(0){ //TODO see home
                    reset_counters();
                    status = 4;
                }
                break;
            case 4: // going home
                follow_azimuth((lastdir+720)%360);
                if(((base_data.counterA + base_data.counterB) / 2) >= dist){// at home?
                    status = 5;// turn to base direction
                }
                break;
            case 5: // turing to base direction
                follow_azimuth(original_heading);
                if(0){ //TODO is this base direction?
                    status = 0;
                }
                break;
            case 6: // user control
                switch (user_dir)
                {
                    case 1:  // right
                             set_motor_speeds(NORMAL_NAVIGATION_SPEED, NORMAL_NAVIGATION_SPEED);
                             follow_azimuth((base_data.heading + 30) % 360);
			     user_moving = 1;
			     //printf("follow %d\n", (base_data.heading + 30) % 360);
                             break;
                    case 2:  // left
                             set_motor_speeds(NORMAL_NAVIGATION_SPEED, NORMAL_NAVIGATION_SPEED);
                             follow_azimuth((base_data.heading + 330) % 360);
			     user_moving = 1;
			     //printf("follow %d\n", (base_data.heading + 330) % 360);
                             break;
		    case 3:  // back
                             set_motor_speeds(NORMAL_NAVIGATION_SPEED, NORMAL_NAVIGATION_SPEED);
                             follow_azimuth((base_data.heading + 180) % 360);
			     //printf("follow %d\n", (base_data.heading + 180) % 360);
			     user_moving = 1;
                             break;
                    case 4:  // on/off
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
                    case 5:  // backup now
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
              wait_for_new_base_data();
              wait_for_new_base_data();
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
