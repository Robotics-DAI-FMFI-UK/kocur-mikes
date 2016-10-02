#ifndef _MIKES_LOGS_H_
#define _MIKES_LOGS_H_

#define ML_MAX_TYPE 2
#define ML_ERR 2
#define ML_WARN 1
#define ML_INFO 0

void init_mikes_logs();
void mikes_log(unsigned int log_type, char *log_msg);

#endif
