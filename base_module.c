#include "mikes.h"
#include "base_module.h"
#include "mikes_logs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <ctype.h>

#define MAX_PACKET_LENGTH 200

static pthread_mutex_t base_module_lock;

static int fdR[2];
static int fdW[2];


void connect_base_module() 
{
	int child;

        if (pipe(fdR) < 0) {
            perror("pipe2()");
            exit(-1);
        }
        if (!pipe(fdW) < 0) {
            perror("pipe2()");
            exit(-1);
        }

        if ((child = fork()) == 0) {
            /* child */

            close(0);
            close(1);
            dup2(fdR[0], 0);
            dup2(fdW[1], 1);
            close(fdR[0]);
            close(fdR[1]);
            close(fdW[0]);
            close(fdW[1]);

            if (execl("/usr/bin/plink", "/usr/bin/plink", "/dev/ttyUSB0",
             "-serial", "-sercfg", "115200,N,n,8,1", NULL) < 0) {
                perror("child execl()");
                exit(-1);
            }
        }

    if (child < 0) {
            perror("fork()");
            exit(-1);
    }

    close(fdR[0]);
    close(fdW[1]);
    mikes_log(ML_INFO, "base module connected");
}

base_data_type local_data;

void read_base_packet()
{
    unsigned char ch;
    int numRead;

    do {
        if ((numRead = read(fdW[0], &ch, 1)) < 0) {
            perror("read()");
            exit(-1);
        }
    } while (program_runs && (ch != '$'));

    char line[1024];
    int lnptr = 0;
    do {
      if ((numRead = read(fdW[0], line + lnptr, 1)) < 0) {
         perror("read()");
         exit(-1);
      }
      lnptr += numRead;
      if (lnptr > 1023) break;
    } while (program_runs && (line[lnptr - 1] != '\n'));
    pthread_mutex_lock(&base_module_lock);
    sscanf(line, "%ld%ld%d%d%d%d%d%d%d%d%d%d%d%d%d", 
                                  &(local_data.counterA), &(local_data.counterB), &(local_data.velocityA),
                                  &(local_data.velocityB), &(local_data.dist1), &(local_data.dist2),
                                  &(local_data.dist3), &(local_data.cube), &(local_data.heading), 
                                  &(local_data.ax), &(local_data.ay), &(local_data.az),
                                  &(local_data.gx), &(local_data.gy), &(local_data.gz));
    pthread_mutex_unlock(&base_module_lock);
    //mikes_log(ML_INFO, line);
}

void set_motor_speeds(int left_motor, int right_motor)
{
    char cmd[40];
    int lm = abs(left_motor);
    int rm = abs(right_motor);
    sprintf(cmd, "@M%c%1d%1d%c%1d%1d", ((left_motor > 0)?' ':'-'), 
                                     ((lm / 10) % 10),
                                     (lm % 10),
                                     ((right_motor > 0)?' ':'-'),
                                     (rm / 10) % 10,
                                     (rm % 10));
    printf("cmd:%s\n", cmd);
    if (write(fdR[1], cmd, strlen(cmd)) < strlen(cmd))
    {
       perror("mikes:base");
       mikes_log(ML_ERR, "base: could not send command");
    }
}

void stop_now()
{
    if (write(fdR[1], "@S", 2) < 2)
    {
       perror("mikes:base");
       mikes_log(ML_ERR, "base: could not send command");
    }
}

void follow_azimuth(int azimuth)
{
    char cmd[20];
    sprintf(cmd, "@A%d%d%d", azimuth / 100, (azimuth % 100) / 10, azimuth % 10);
    if (write(fdR[1], cmd, strlen(cmd)) < strlen(cmd))
    {
      perror("mikes:base");
      mikes_log(ML_ERR, "base: could not send azimuth");
    }
}

void reset_counters()
{
    if (write(fdR[1], "@R", 2) < 2)
    {
      perror("mikes:base");
      mikes_log(ML_ERR, "base: could not reset counters");
    }
}

void regulated_speed(int left_motor, int right_motor)
{
   char cmd[40];
    int lm = left_motor;
    int rm = right_motor;

    sprintf(cmd, "@V%c%d%d%c%d%d", ((left_motor > 0)?' ':'-'), 
                                     (lm / 10) % 10,
                                     (lm % 10),
                                     (right_motor > 0)?' ':'-',
                                     (rm / 10) % 10,
                                     (rm % 10));
    if (write(fdR[1], cmd, strlen(cmd)) < strlen(cmd))
    {
      perror("mikes:base");
      mikes_log(ML_ERR, "base: could not send regulated speed");
    }
}
                                                           
void *base_module_thread(void *args) 
{
    while (program_runs) 
    {
        read_base_packet();
        //parse_base_packet();
        usleep(10000);
    }

    mikes_log(ML_INFO, "base quits.");
    threads_running_add(-1);
}

void init_base_module()
{
    pthread_t t;
    connect_base_module();
    pthread_mutex_init(&base_module_lock, 0);
    if (pthread_create(&t, 0, base_module_thread, 0) != 0)
    {
      perror("mikes:base");
      mikes_log(ML_ERR, "creating thread for base module");
    }
    else threads_running_add(1);
}


void get_base_data(base_data_type* buffer) 
{
    pthread_mutex_lock(&base_module_lock);
    memcpy(buffer, &local_data, sizeof(base_data_type));
    pthread_mutex_unlock(&base_module_lock);
}

short angle_difference(short alpha, short beta)
{
  short diff = beta - alpha;
  if (diff > 180) return diff - 360;
  else if (diff < -180) return diff + 360;
  return diff;
}

