#include "mikes.h"
#include "rfid_sensor.h"
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
#include <ctype.h>

#define MAX_PACKET_LENGTH 200

#define STX 2
#define ETX 3

static pthread_mutex_t rfid_sensor_lock;
static int rfid_sockfd;
static char rfid_connected;

static char input_packet[MAX_PACKET_LENGTH];

void read_input_packet()
{
    unsigned char ch = 0;

    while (program_runs && (ch != STX))
        if (read(rfid_sockfd, &ch, 1) < 0)
        {
            perror("mikes:rfid");
            mikes_log(ML_ERR, "reading response start from rfid sensor");
            return;
        }

    int ptr = 0;
    do {
        int nread;
        if ((nread = read(rfid_sockfd, &ch, 1)) < 0)
        {
            perror("mikes:rfid");
            mikes_log(ML_ERR, "reading response from rfid sensor");
            return;
        }
        if ((nread > 0) && (ptr < MAX_PACKET_LENGTH)) input_packet[ptr++] = ch;
    } while (program_runs && (ch != ETX));

    input_packet[--ptr] = 0;
    //mikes_log(ML_INFO, input_packet);
}

void send_output_packet(char *packet)
{
    unsigned char ch[2] = { STX, '\r' };

    if (write(rfid_sockfd, ch, 1) < 0)
    {
        perror("mikes:rfid");
        mikes_log(ML_ERR, "writing packet to rfid sensor");
    }

    if (write(rfid_sockfd, packet, strlen(packet)) < 0)
    {
        perror("mikes:rfid");
        mikes_log(ML_ERR, "writing  packet to rfid sensor");
    }

    ch[0] = ETX;
    if (write(rfid_sockfd, ch, 2) < 0)
    {
        perror("mikes:rfid");
        mikes_log(ML_ERR, "writing packet  to rfid sensor");
    }

    //mikes_log(ML_INFO, packet);
}

void connect_rfid_sensor()
{
    rfid_connected = 0;
    rfid_sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (rfid_sockfd < 0)
    {
        mikes_log(ML_ERR, "cannot open rfid sensor socket");
        perror("mikes:rfid");
           return;
    }

    struct sockaddr_in remoteaddr;
    remoteaddr.sin_family = AF_INET;
    remoteaddr.sin_addr.s_addr = inet_addr(SICK_ADDR);
    remoteaddr.sin_port = htons(SICK_PORT);

    if (connect(rfid_sockfd, (struct sockaddr*)&remoteaddr, sizeof(remoteaddr)) < 0)
    {
      mikes_log(ML_ERR, "connecting rfid sensor socket");
      perror("mikes:rfid");
        return;
    }
    mikes_log(ML_INFO, "rfid sensor connected");
    rfid_connected = 1;
}

void rfid_sensor_communication_prolog()
{
    char *init_packet = "sRI 0";
    char *firmware_ver_packet = "sRN FirmwareVersion";
    char *sopas_ver_packet = "sRN SOPASVersion";

    char *config0_packet = "sEN QSinv 0";
    char *config1_packet = "sEN QSinv 1";
    char *config2_packet = "sEN QSinv 2";
    char *config3_packet = "sEN QSinv 3";

    sleep(15); // sensor boots slowly
    send_output_packet(init_packet);
    read_input_packet();

    send_output_packet(firmware_ver_packet);
    read_input_packet();

    send_output_packet(sopas_ver_packet);
    read_input_packet();

    send_output_packet(config0_packet);
    read_input_packet();
    send_output_packet(config1_packet);
    read_input_packet();
    send_output_packet(config2_packet);
    read_input_packet();
    send_output_packet(config3_packet);
    read_input_packet();
}

static long tagid[MAX_NUMBER_OF_TAGS];
static rfid_data_type rfid_data;
static rfid_data_type local_data;

void localize_tags_found()
{
  char tagstr[80];

  for (int i = 0; i < local_data.ntags; i++)
  {
    local_data.x[i] = tagid[i]  / 1000000;
    local_data.y[i] = (tagid[i] / 10000) % 100;
    local_data.a[i] = tagid[i] % 100;
    sprintf(tagstr, " %d: [%d, %d, %d]", i, local_data.x[i], local_data.y[i], local_data.a[i]);
    mikes_log(ML_INFO, tagstr);
  }
}

void parse_input_packet()
{
    char tagstr[40];
    static unsigned char saw_anything = 0;
   
    if (isdigit(input_packet[0]))
    {
        local_data.ntags = strlen(input_packet) / 12;
        for (int i = 0; i < local_data.ntags; i++)
        {
            strncpy(tagstr, input_packet + i * 12 + 4, 8);
            tagstr[8] = 0;
            sscanf(tagstr, "%ld", &tagid[i]);
        }
        sprintf(tagstr, "#tags: %d", local_data.ntags);
        mikes_log(ML_INFO, tagstr);
        for (int i = 0; i < local_data.ntags; i++)
        {
            sprintf(tagstr, " -> %8ld", tagid[i]);
            mikes_log(ML_INFO, tagstr);
        }
        saw_anything = 1;
    }
    else if (saw_anything) saw_anything = 0;
    else local_data.ntags = 0;
}

void save_the_tags_found()
{
        pthread_mutex_lock(&rfid_sensor_lock);
        memcpy(&rfid_data, &local_data, sizeof(rfid_data_type));
        pthread_mutex_unlock(&rfid_sensor_lock);
}

void *rfid_sensor_thread(void *args)
{
    char *start_measuring_packet = "sMN MIStartIn";
    char *stop_measuring_packet = "sMN MIStopIn";

    if (!rfid_connected) return 0;
    rfid_sensor_communication_prolog();
    send_output_packet(start_measuring_packet);

    while (program_runs)
    {
        read_input_packet();
        parse_input_packet();
        localize_tags_found();
        save_the_tags_found();
        usleep(10000);
    }

    send_output_packet(stop_measuring_packet);
    mikes_log(ML_INFO, "rfid quits.");
    threads_running_add(-1);
    return 0;
}

void init_rfid_sensor()
{
    pthread_t t;
    connect_rfid_sensor();
    pthread_mutex_init(&rfid_sensor_lock, 0);
    if (pthread_create(&t, 0, rfid_sensor_thread, 0) != 0)
    {
      perror("mikes:rfid");
      mikes_log(ML_ERR, "creating thread for rfid sensor");
    }
    else threads_running_add(1);
}


void get_rfid_data(rfid_data_type* buffer)
{
    pthread_mutex_lock(&rfid_sensor_lock);
    memcpy(buffer, &rfid_data, sizeof(rfid_data_type));
    pthread_mutex_unlock(&rfid_sensor_lock);
}

