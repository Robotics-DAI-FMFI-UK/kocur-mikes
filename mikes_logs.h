#ifndef _MIKES_LOGS_H_
#define _MIKES_LOGS_H_

#define ML_MAX_TYPE 2
#define ML_ERR 2
#define ML_WARN 1
#define ML_INFO 0

void init_mikes_logs();
void mikes_log(unsigned int log_type, char *log_msg);
void mikes_log_val(unsigned int log_type, char *log_msg, int val);
void mikes_log_val2(unsigned int log_type, char *log_msg, int val, int val2);

#endif
