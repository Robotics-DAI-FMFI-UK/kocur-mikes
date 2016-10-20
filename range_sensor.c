#include "mikes.h"
#include "range_sensor.h"
#include "mikes_logs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#define BUFFER_SIZE 5000

pthread_mutex_t range_sensor_lock;
int *range_data;

static int *local_data;
static int sockfd;

void connect_range_sensor()
{
    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0) {
    mikes_log(ML_ERR, "cannot open range sensor socket");
    perror("mikes:range");
        return;
    }

    struct sockaddr_in remoteaddr;
    remoteaddr.sin_family = AF_INET;
    remoteaddr.sin_addr.s_addr = inet_addr(HOKUYO_ADDR);
    remoteaddr.sin_port = htons(HOKUYO_PORT);

    if (connect(sockfd, (struct sockaddr*)&remoteaddr, sizeof(remoteaddr)) < 0)
    {
    mikes_log(ML_ERR, "connecting range sensor socket");
    perror("mikes:range");
        return;
    }

    mikes_log(ML_INFO, "range sensor connected");
}

void *range_sensor_thread(void *args)
{
    char *start_measurement = "BM\n";
    char *request_measurement = "GD0000108000\n";
    unsigned char readbuf[BUFFER_SIZE];

    if (write(sockfd, start_measurement, strlen(start_measurement)) < 0)
    {
        perror("mikes:range");
        mikes_log(ML_ERR, "writing start measurement packet to range sensor");
    }

    usleep(250000);

    unsigned char x[2];
    int cnt = 0;
    do {
        if (read(sockfd, x + ((cnt++) % 2), 1) < 0)
        {
            perror("mikes:range");
            mikes_log(ML_ERR, "reading response from range sensor");
            break;
        }
    } while ((x[0] != 10) || (x[1] != 10));

    while (program_runs)
    {
        if (write(sockfd, request_measurement, strlen(request_measurement)) < 0)
        {
            perror("mikes:range");
            mikes_log(ML_ERR, "writing request to range sensor");
            break;
        }

        int readptr = 0;
        do {
            int nread = read(sockfd, readbuf + readptr, BUFFER_SIZE - readptr);
            if (nread < 0)
            {
                perror("mikes:range");
                mikes_log(ML_ERR, "reading response from range sensor");
                break;
            }
            readptr += nread;
            if (readptr < 2) continue;
        } while ((readbuf[readptr - 1] != 10) || (readbuf[readptr - 2] != 10));

        int searchptr = 0;
        for (int i = 0; i < 3; i++)
        {
            while ((readbuf[searchptr] != 10) && (searchptr < readptr))
                searchptr++;
            searchptr++;
        }

        if (readptr - searchptr != 103 + RANGE_DATA_COUNT * 3)
        {
            static char *logmsg1 = "Hokuyo returned packet of unexpected size, I will ignore it size=%d";
            char msg[strlen(logmsg1) + 20];
            sprintf(msg, logmsg1, readptr - searchptr);
            mikes_log(ML_WARN, msg);
            continue;
        }

        int beam_index = RANGE_DATA_COUNT - 1;
        readptr = searchptr;
        while (beam_index >= 0)
        {
            int pos = (searchptr - readptr) % 66;
            if (pos == 62)
            {
                local_data[beam_index] = ((readbuf[searchptr] - 0x30) << 12) |
                                         ((readbuf[searchptr + 1] - 0x30) << 6) |
                                         (readbuf[searchptr + 4] - 0x30);
                searchptr += 5;
            } else if (pos == 63)
            {
                local_data[beam_index] = ((readbuf[searchptr] - 0x30) << 12) |
                                         ((readbuf[searchptr + 3] - 0x30) << 6) |
                                         (readbuf[searchptr + 4] - 0x30);
                searchptr += 5;
            } else
            {
                if (pos == 64) searchptr += 2;
                local_data[beam_index] = ((((int)readbuf[searchptr]) - 0x30) << 12) |
                                         ((((int)readbuf[searchptr + 1]) - 0x30) << 6) |
                                         (((int)readbuf[searchptr + 2]) - 0x30);
                searchptr += 3;
            }
            beam_index--;
        }

        pthread_mutex_lock(&range_sensor_lock);
        memcpy(range_data, local_data, sizeof(int) * RANGE_DATA_COUNT);
        pthread_mutex_unlock(&range_sensor_lock);
        usleep(25000);
    }

    mikes_log(ML_INFO, "range quits.");
    threads_running_add(-1);
    return 0;
}

void init_range_sensor()
{
    pthread_t t;
    range_data = (int *) malloc(sizeof(int) * RANGE_DATA_COUNT);
    local_data = (int *) malloc(sizeof(int) * RANGE_DATA_COUNT);
    if ((range_data == 0) || (local_data == 0))
    {
      perror("mikes:range");
      mikes_log(ML_ERR, "insufficient memory");
      exit(1);
    }
    connect_range_sensor();
    pthread_mutex_init(&range_sensor_lock, 0);
    if (pthread_create(&t, 0, range_sensor_thread, 0) != 0)
    {
      perror("mikes:range");
      mikes_log(ML_ERR, "creating thread for range sensor");
    }
    else threads_running_add(1);
}

void get_range_data(int* buffer)
{
    pthread_mutex_lock(&range_sensor_lock);
    memcpy(buffer, range_data, sizeof(int) * RANGE_DATA_COUNT);
    pthread_mutex_unlock(&range_sensor_lock);
}

