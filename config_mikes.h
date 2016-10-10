#ifndef _CONFIG_MIKES_H_
#define _CONFIG_MIKES_H_

#include "config/config.h"

#define MIKES_CONFIG "/usr/local/etc/config_mikes.cfg"

typedef struct {
    int with_gui;
    int print_all_logs_to_console;
} mikes_config_t;

extern mikes_config_t mikes_config;

void load_config();

#endif
