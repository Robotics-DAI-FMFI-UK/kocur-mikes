#ifndef _MIKES_DAEMON_H_
#define _MIKES_DAEMON_H_

extern volatile unsigned char program_runs;
extern volatile unsigned char user_control;
extern volatile unsigned char user_dir;
extern volatile unsigned char start_automatically;

void threads_running_add(short x);

#endif
