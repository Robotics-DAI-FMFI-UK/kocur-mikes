#ifndef _MIKES_DAEMON_H_
#define _MIKES_DAEMON_H_

#define USER_DIR_LEFT   1
#define USER_DIR_RIGHT  2
#define USER_DIR_BACKUP 3
#define USER_DIR_ONOFF  4
#define USER_DIR_BACK   5
#define USER_DIR_SPINRIGHT 6
#define USER_DIR_SPINLEFT 7

extern volatile unsigned char program_runs;
extern volatile unsigned char user_control;
extern volatile unsigned char user_dir;
extern volatile unsigned char start_automatically;

void threads_running_add(short x);

#endif
