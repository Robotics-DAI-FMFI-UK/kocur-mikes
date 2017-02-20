#ifndef _UTIL_H_
#define _UTIL_H_

// square of distance of two points
double distance(double x1, double y1, double x2, double y2);

// return current time in milliseconds
long msec();

// return current time in usec (up to 71 minutes)
long usec();

// say the sentence
void say(char *sentence);

#endif
