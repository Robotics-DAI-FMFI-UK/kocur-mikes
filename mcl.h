#ifndef _MCL_MIKES_H_
#define _MCL_MIKES_H_

typedef struct {
  double x, y, alpha;
  double w;
}hypo_t ;

#define HYPO_COUNT 2500

int init_mcl();
void get_mcl_data(hypo_t *buffer);
int mcl_update(double traveled, int heading);

#endif
