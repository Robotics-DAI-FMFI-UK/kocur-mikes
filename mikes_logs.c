#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <malloc.h>
#include <stdlib.h>

#include "mikes_logs.h"

static char *log_filename;

void init_mikes_logs()
{
  char *filename_str = "/usr/local/logs/mikes/%ld_%s";
  char *lastlog = "/usr/local/logs/mikes/last";
  char *filename_base = "mikes.log";
  log_filename = (char *)malloc(strlen(filename_str) + 20 + strlen(filename_base));
  if (log_filename == 0)
  {
    perror("mikes:logs");
    exit(1);
  }
  time_t t;
  time(&t);
  sprintf(log_filename, filename_str, t, filename_base);
  FILE *f = fopen(log_filename, "w+");
  fclose(f);
  unlink(lastlog);
  symlink(log_filename, lastlog);
}

static char *log_type_str[3] = { "INFO", "WARN", " ERR" };

void mikes_log(unsigned int log_type, char *log_msg)
{

  FILE *f = fopen(log_filename, "w+");
  if ((log_type < 0) || (log_type > ML_MAX_TYPE))
  {
      fprintf(f, "WARN: unrecognized log type %d\n", log_type);
      log_type = ML_ERR;
  }
  fprintf(f, "%s: %s\n", log_type_str[log_type], log_msg); 
  fclose(f);
}

