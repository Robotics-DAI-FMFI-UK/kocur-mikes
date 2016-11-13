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
#include <signal.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>

#define MAX_PACKET_LENGTH 200

static int current_azimuth;

static pthread_mutex_t base_module_lock;

static int fdR[2];
static int fdW[2];

static pid_t plink_child;

static volatile unsigned char new_base_data_arrived;
static unsigned char base_initialized;

void connect_base_module()
{
    if (pipe(fdR) < 0)
    {
        mikes_log(ML_ERR, "base: pipe2()");
        base_initialized = 0;
        return;
    }
    if (!pipe(fdW) < 0)
    {
        mikes_log(ML_ERR, "base: pipe2()");
        base_initialized = 0;
        return;
    }

    if ((plink_child = fork()) == 0)
    {
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
                  "-serial", "-sercfg", "115200,N,n,8,1", NULL) < 0)
        {
            mikes_log(ML_ERR, "base: child execl()");
            base_initialized = 0;
            return;
        }
    }

    if (plink_child < 0)
    {
        mikes_log(ML_ERR, "base: child execl()");
        base_initialized = 0;
        return;
    }

    close(fdR[0]);
    close(fdW[1]);
    if (fcntl( fdW[0], F_SETFL, fcntl(fdW[0], F_GETFL) | O_NONBLOCK) < 0)
    {
        mikes_log(ML_ERR, "base: setting nonblock on read pipe end");
        base_initialized = 0;
    }

    mikes_log(ML_INFO, "base module connected");
    current_azimuth = NO_AZIMUTH;
    base_initialized = 1;
}

base_data_type local_data;

void read_base_packet()
{
    unsigned char ch;
    int numRead;

    do {
        if ((numRead = read(fdW[0], &ch, 1)) < 0)
        {
            if (errno != EAGAIN)
            {
                perror("read()");
                exit(-1);
            }
            else usleep(1);
        }
    } while (program_runs && (ch != '$'));

    unsigned char more_packets_in_queue = 0;
    char line[1024];
    do {
        int lnptr = 0;
        do {
          if ((numRead = read(fdW[0], line + lnptr, 1)) < 0)
          {
              if (errno != EAGAIN)
              {
                  perror("read()");
                  exit(-1);
              }
              else { usleep(1); continue; }
          }
          lnptr += numRead;
          if (lnptr > 1023) break;
        } while (program_runs && (line[lnptr - 1] != '\n'));

        if (lnptr > 0) line[lnptr - 1] = 0;

        more_packets_in_queue = 0;
        while (program_runs)
        {
            if (read(fdW[0], &ch, 1) < 0)
            {
                if (errno == EAGAIN) break;
                else
                {
                    perror("read()");
                    exit(-1);
                }
            }
            if (ch == '$')
            {
                more_packets_in_queue = 1;
                break;
            }
        }
    } while (program_runs && more_packets_in_queue);

    pthread_mutex_lock(&base_module_lock);
    sscanf(line, "%ld%ld%hd%hd%hd%hd%hd%hd%hd%hd%hd%hd%hd%hd%hd",
                                  &(local_data.counterA), &(local_data.counterB), &(local_data.velocityA),
                                  &(local_data.velocityB), &(local_data.dist1), &(local_data.dist2),
                                  &(local_data.dist3), &(local_data.cube), &(local_data.heading),
                                  &(local_data.ax), &(local_data.ay), &(local_data.az),
                                  &(local_data.gx), &(local_data.gy), &(local_data.gz));
    new_base_data_arrived = 1;
    //printf("CA: %ld,  CB: %ld\n", local_data.counterA, local_data.counterB);
    pthread_mutex_unlock(&base_module_lock);
    //mikes_log(ML_INFO, line);
}

void wait_for_new_base_data()
{
    new_base_data_arrived = 0;
    while (!new_base_data_arrived) usleep(1000);
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
    current_azimuth = NO_AZIMUTH;
}

void follow_azimuth(int azimuth)
{
    char cmd[20];
    current_azimuth = azimuth;
    sprintf(cmd, "@A%d%d%d", azimuth / 100, (azimuth % 100) / 10, azimuth % 10);
    if (write(fdR[1], cmd, strlen(cmd)) < strlen(cmd))
    {
      perror("mikes:base");
      mikes_log(ML_ERR, "base: could not send azimuth");
    }
}

int get_current_azimuth()
{
    return current_azimuth;
}

void reset_counters()
{
    if (write(fdR[1], "@R", 2) < 2)
    {
      perror("mikes:base");
      mikes_log(ML_ERR, "base: could not reset counters");
    }
    wait_for_new_base_data();
    wait_for_new_base_data();
}

void regulated_speed(int left_motor, int right_motor)
{
    char cmd[40];
    int lm = abs(left_motor);
    int rm = abs(right_motor);

    sprintf(cmd, "@V%c%d%d%d%c%d%d%d", ((left_motor > 0)?' ':'-'),
                                     (lm / 100) % 10,
                                     (lm / 10) % 10,
                                     (lm % 10),
                                     (right_motor > 0)?' ':'-',
                                     (rm / 100) % 10,
                                     (rm / 10) % 10,
                                     (rm % 10));
    if (write(fdR[1], cmd, strlen(cmd)) < strlen(cmd))
    {
      perror("mikes:base");
      mikes_log(ML_ERR, "base: could not send regulated speed");
    }
    current_azimuth = NO_AZIMUTH;
}

void *base_module_thread(void *args)
{
    while (program_runs)
    {
        read_base_packet();
        usleep(10000);
    }

    mikes_log(ML_INFO, "base quits.");
    stop_now();
    usleep(100000);
    kill(plink_child, SIGTERM);
    threads_running_add(-1);
    return 0;
}

void init_base_module()
{
    pthread_t t;
    base_initialized = 0;
    connect_base_module();
    if (!base_initialized) return;
    pthread_mutex_init(&base_module_lock, 0);
    if (pthread_create(&t, 0, base_module_thread, 0) != 0)
    {
      perror("mikes:base");
      mikes_log(ML_ERR, "creating thread for base module");
      base_initialized = 0;
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

void cancel_azimuth_mode()
{
    if (write(fdR[1], "@X", 2) < 2)
    {
      perror("mikes:base");
      mikes_log(ML_ERR, "base: could not cancel azimuth mode");
    }
    current_azimuth = NO_AZIMUTH;
}

void pause_status_reporting()
{
    if (write(fdR[1], "@-", 2) < 2)
    {
      perror("mikes:base");
      mikes_log(ML_ERR, "base: could not pause reporting");
    }
}

void resume_status_reporting()
{
    if (write(fdR[1], "@+", 2) < 2)
    {
      perror("mikes:base");
      mikes_log(ML_ERR, "base: could not resume reporting");
    }
}

void set_laziness(unsigned char laziness)
{
    char cmd[10];
    sprintf(cmd, "@L %d%d", laziness / 10, laziness % 10);
    if (write(fdR[1], cmd, strlen(cmd)) < strlen(cmd))
    {
      perror("mikes:base");
      mikes_log(ML_ERR, "base: could not set laziness");
    }
}
