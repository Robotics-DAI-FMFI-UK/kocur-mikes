#include <stdio.h>
#include <unistd.h>

#include "mikes.h"
#include "mikes_logs.h"
#include "range_sensor.h"

void *navigation_thread(void *arg)
{
    int ranges[RANGE_DATA_COUNT];
    while (program_runs)
    {
/*        get_range_data(ranges);
    
        int portion = RANGE_DATA_COUNT / 10;
        for (int i = 0; i < 10; i++)
        {
          long avg = 0;
          for (int j = i * portion; j < (i + 1) * portion; j++)
             avg += ranges[j];
          printf("%4d\t", avg / portion);
        }
        printf("\n"); */
	sleep(2);
    } 
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
    else threads_running++;
}
