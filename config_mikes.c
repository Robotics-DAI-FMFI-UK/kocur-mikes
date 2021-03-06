#include "config/config.h"
#include "config_mikes.h"

mikes_config_t default_mikes_config = { 0, 0, 0, 0, 0 };
mikes_config_t mikes_config;

void load_config()
{
    config_data cfg = read_config(MIKES_CONFIG);
    mikes_config = default_mikes_config;
    mikes_config.autostart = config_get_intval(cfg, "autostart", mikes_config.autostart);
    mikes_config.with_gui = config_get_intval(cfg, "show_gui", mikes_config.with_gui);
    mikes_config.print_all_logs_to_console = config_get_intval(cfg, "print_all_logs_to_console",
                                                               mikes_config.print_all_logs_to_console);
    mikes_config.print_debug_logs = config_get_intval(cfg, "print_debug_logs", mikes_config.print_debug_logs);
    mikes_config.use_ncurses_control_console = config_get_intval(cfg, "use_ncurses_control_console", mikes_config.use_ncurses_control_console);
    config_dispose(cfg);
}
