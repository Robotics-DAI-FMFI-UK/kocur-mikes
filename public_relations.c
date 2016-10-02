#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include "mikes.h"
#include "mikes_logs.h"

void report_status_to_server()
{
  system("ifconfig > /tmp/kocurmikes.ifconfig.txt");
  system("scp -q /tmp/kocurmikes.ifconfig.txt petrovic@kempelen.ii.fmph.uniba.sk:kocurmikes >/dev/null");
  system("rm -f /tmp/kocurmikes.ifconfig.txt");
}

void *public_relations_thread(void *args)
{
  sleep(10);
  while (program_runs)
  {
    report_status_to_server();
    sleep(60);
  }
  threads_running--;
  return 0;
}

void init_public_relations()
{
  pthread_t pr_thread;

  if (pthread_create(&pr_thread, 0, public_relations_thread, 0) != 0)
  {
    perror("mikes:pr");
    mikes_log(ML_ERR, "creating pr thread");
  }
  else threads_running++;
}

