#include <sys/time.h>

double distance(double x1, double y1, double x2, double y2)
{
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

long usec()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  return 1000000L * tv.tv_sec + tv.tv_usec;
}
