#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#include "public_relations.h"
#include "range_sensor.h"
#include "rfid_sensor.h"
#include "mikes_logs.h"
#include "navigation.h"
#include "gui.h"
#include "config_mikes.h"
#include "base_module.h"
#include "ncurses_control.h"

volatile unsigned char program_runs;
static pthread_mutex_t mikes_lock;
volatile unsigned short threads_running;
volatile unsigned char user_control;
volatile unsigned char user_dir;
volatile unsigned char start_automatically;

void threads_running_add(short x)
{
  pthread_mutex_lock(&mikes_lock);
  threads_running += x;
  pthread_mutex_unlock(&mikes_lock);
}

void signal_term_handler(int signum)
{
  program_runs = 0;
}

int main(int argc, char **argv)
{
  program_runs = 1;
  threads_running = 1;
  pthread_mutex_init(&mikes_lock, 0);
  signal(SIGTERM, signal_term_handler);

  load_config();

  if ((!mikes_config.autostart) && (argc > 1)) 
    if (strcmp(argv[1], "autostart") == 0) return 0;

  init_mikes_logs();
  init_public_relations();
  init_base_module();
  init_range_sensor();
  init_rfid_sensor();
  init_navigation();
  init_ncurses_control();

  init_gui();

  while (program_runs)
  {
     sleep(1);
  }

  int old_tr = threads_running + 1;
  while (threads_running > 1)
  {
    usleep(10000);
    if (threads_running < old_tr)
    {
      char tr[50];
      sprintf(tr, "%d threads running", threads_running);
      mikes_log(ML_INFO, tr);
      old_tr = threads_running;
    }
  }

  mikes_log(ML_INFO, "Kocur mikes quits.");
  usleep(100000);
  return 0;
}

