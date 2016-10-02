#include "config/config.h"
#include "config_mikes.h"

mikes_config_t default_mikes_config = { 0 };
mikes_config_t mikes_config;

void load_config()
{
    config_data cfg = read_config(MIKES_CONFIG);
    mikes_config = default_mikes_config;
    mikes_config.with_gui = config_get_intval(cfg, "show_gui");
    config_dispose(cfg);
}
