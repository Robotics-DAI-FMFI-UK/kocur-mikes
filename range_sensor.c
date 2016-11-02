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
#define MAX_ERROR_RAYS 5
#define MAX_NEIGHBOR_DIFF 60

pthread_mutex_t range_sensor_lock;
int *range_data;

static int *local_data;
static int *detection_data;
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
    detection_data = (int *) malloc(sizeof(int) * RANGE_DATA_COUNT);
    if ((range_data == 0) || (local_data == 0) || (detection_data == 0))
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

short size_of_object(short a, short b, double gama){ // a=start, b=end - using Law of cosines
    return round(sqrt( a*a + b*b - 2*a*b*cos(gama) ));
}

void get_range_segments(segments_type *segments, int angular_detecting_range, int min_seg_size)
{
    pthread_mutex_lock(&range_sensor_lock);
    memcpy(detection_data, range_data, sizeof(int) * RANGE_DATA_COUNT);
    pthread_mutex_unlock(&range_sensor_lock);

    short starting = RANGE_DATA_COUNT/2 - angular_detecting_range/2;
    short ending = RANGE_DATA_COUNT/2 + angular_detecting_range/2;

    segments->nsegs_found = 0;

    short missing = 0;
    short error_rate = 0;
    unsigned long error_sum = 0;

    short firstsi = starting; // first seen index
    short lastsi = starting; // last seen index
    for(int i=starting+1; i < ending; i++){
        if ( (abs( detection_data[i] - detection_data[lastsi]) < MAX_NEIGHBOR_DIFF) && ( detection_data[i] < MAX_DISTANCE)){
            // same object
            lastsi = i;
            missing = 0;
        } else {
            error_sum += abs( detection_data[i] - detection_data[lastsi]);
            error_rate += 1;
            missing +=1;
            if( missing >= MAX_ERROR_RAYS ){ // CONST
                // new object
                if( (detection_data[firstsi] < MAX_DISTANCE) && (lastsi-firstsi > MAX_ERROR_RAYS) ){
                    short size = size_of_object(detection_data[firstsi], detection_data[lastsi], (lastsi-firstsi+1)*SIZE_OF_ONE_STEP );
                    if( size >= min_seg_size){
                        segments->dist[segments->nsegs_found] = (detection_data[firstsi]+detection_data[lastsi])/2; // count from first and last point. It maybe count like avg from all non-error values.
                        segments->width[segments->nsegs_found] = size;
                        segments->alpha[segments->nsegs_found] = ((firstsi+lastsi)/2 - RANGE_DATA_COUNT/2) / SIZE_OF_ONE_DEG;
                        segments->firstray[segments->nsegs_found] = firstsi;
                        segments->lastray[segments->nsegs_found] = lastsi;
                        segments->nsegs_found += 1;
/*
                        if(error_sum>=MAX_ERROR_RAYS*65533+detection_data[lastsi]) // CONST
                            error_sum -= MAX_ERROR_RAYS*65533;// TODO - nefunguje
                        error_rate -= MAX_ERROR_RAYS; // CONST
                        char tagstr[200];
                        sprintf(tagstr, "new object!!! first:%5d last:%5d Fval:%5d Lval:%5d size:%5d dist:%5d alpha:%5d error_rate:%3d error_sum:%8lu sizeInDeg:%5.2f sizeInRad:%7.5f",
                            firstsi,
                            lastsi,
                            detection_data[firstsi],
                            detection_data[lastsi],
                            size,
                            (detection_data[firstsi]+detection_data[lastsi])/2,
                            (firstsi+lastsi)/2,
                            error_rate,
                            error_sum,
                            ((double)(lastsi-firstsi+1))/4,
                            (lastsi-firstsi+1)*SIZE_OF_ONE_STEP);
                        mikes_log(ML_DEBUG, tagstr);
*/
                    }
                }
                missing = 0;
                error_rate = 0;
                error_sum = 0;

                i -= MAX_ERROR_RAYS-1;
                firstsi = lastsi = i;
            }
        }
    }

    return;
}
