#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

#include "public_relations.h"
#include "range_sensor.h"
#include "mikes_logs.h"
#include "navigation.h"
#include "gui.h"
#include "config_mikes.h"

volatile unsigned char program_runs;
volatile unsigned short threads_running;

void signal_term_handler(int signum)
{
  program_runs = 0;
}

int main(int argc, char **argv)
{
  program_runs = 1;
  threads_running = 1;
  signal(SIGTERM, signal_term_handler);

  printf("Kocur mikes runs.\n");

  load_config();

  init_mikes_logs();
  init_public_relations();
  init_range_sensor();
  init_navigation();
  
  init_gui();

  while (program_runs)
  {
    sleep(60);
  }
 
  while (threads_running)
    usleep(10000);

  printf("Kocur mikes quits.\n");
  return 0;
}

