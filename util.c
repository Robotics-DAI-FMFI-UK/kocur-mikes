#include <sys/time.h>

long usec()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  return 1000000L * tv.tv_sec + tv.tv_usec;
}
