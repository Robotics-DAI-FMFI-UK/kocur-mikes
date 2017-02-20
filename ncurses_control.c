#include <ncurses.h>
#include <pthread.h>
#include <unistd.h>

#include "mikes.h"
#include "mikes_logs.h"
#include "config_mikes.h"
#include "rfid_sensor.h"
#include "util.h"
#include "mcl.h"
#include "base_module.h"

#define KEY_UP_ARROW 259
#define KEY_DOWN_ARROW 258
#define KEY_LEFT_ARROW 260
#define KEY_RIGHT_ARROW 261
#define KEY_ESC 27
#define KEY_A 97
#define KEY_D 100
#define KEY_I 105
#define KEY_L 108
#define KEY_M 109
#define KEY_O 111
#define KEY_P 112
#define KEY_Q 113
#define KEY_S 115
#define KEY_W 119
#define KEY_X 120
#define KEY_SPACE 32

int last_action, last_heading;

static WINDOW *mainwin;

void rfid_curses_update()
{
    if (!mikes_config.use_ncurses_control_console) return;
    char rfid_scan[100];
    deleteln();
    print_reading_of_rfid(rfid_scan, 100);
    //mvprintw(9, 10, rfid_scan);
    refresh();
    if (rfid_scan[2] == '0') say("no");
    else say("yes");
}

void *ncurses_control_thread(void *arg)
{
    int log_ID = 0;
    int ch;
    base_data_type base_data;
    if ((mainwin = initscr()) == 0)
    {
      mikes_log(ML_WARN, "Error initializing ncurses, not using ncurses");
      return 0;
    }

    noecho();                  /*  Turn off key echoing                 */
    keypad(mainwin, TRUE);     /*  Enable the keypad for non-char keys  */


    /*  Print a prompt and refresh() the screen  */

    mvaddstr(5, 10, "Control Kocur Mikes with arrow keys, space to backup, 'a' to toggle autopilot, ESC to quit");
    mvprintw(7, 10, "You pressed: ");
    refresh();

    while (program_runs)
    {
        ch = getch();
        if (!start_automatically) start_automatically = 1;
        switch (ch)
        {
            case KEY_W:
            case KEY_UP_ARROW: mvprintw(7, 10, "on/off");
                     user_dir = USER_DIR_ONOFF;
                     last_action = user_dir;
                     user_control = 1;
                      break;
            case KEY_LEFT_ARROW: mvprintw(7, 10, "left");
                     user_dir = USER_DIR_LEFT;
                     last_action = user_dir;
                     user_control = 1;
                     get_base_data(&base_data);
                     last_heading = base_data.heading;
                      break;
            case KEY_RIGHT_ARROW: mvprintw(7, 10, "right");
                     user_dir = USER_DIR_RIGHT;
                     last_action = user_dir;
                     user_control = 1;
                     get_base_data(&base_data);
                     last_heading = base_data.heading;
                       break;
            case KEY_DOWN_ARROW: mvprintw(7, 10, "back");
                     user_dir = USER_DIR_BACK;
                     last_action = user_dir;
                     user_control = 1;
                       break;
            case KEY_SPACE: mvprintw(7, 10, "backup");
                     user_dir = USER_DIR_BACKUP;
                     last_action = user_dir;
                     user_control = 1;
                     break;
            case KEY_A:
                     mvprintw(7, 10, "autonomous: %d", user_control);
                     user_control = 1 - user_control;
                     break;
            case KEY_L: mvprintw(7, 15, "log ID: %3d  ", log_ID);
                     append_reading_to_rfid_log(log_ID);
                     log_ID++;
                     mvprintw(7, 16, "ok");
                     break;
            case KEY_I: init_mcl();
                     mvprintw(7, 10, "MCL inited");
                     break;
            case KEY_O:
                     user_dir = USER_DIR_SPINLEFT;
                     last_action = user_dir;
                     user_control = 1;
                     get_base_data(&base_data);
                     last_heading = base_data.heading;
                     break;
            case KEY_P:
                     user_dir = USER_DIR_SPINRIGHT;
                     last_action = user_dir;
                     user_control = 1;
                     get_base_data(&base_data);
                     last_heading = base_data.heading;
                     break;
            case KEY_M:
                     user_dir = USER_DIR_ONOFF;
                     user_control = 1;
                     sleep(1.5);
                     get_base_data(&base_data);
                     double traveled = (base_data.counterA + base_data.counterB) / 2;
                     mvprintw(7, 11, "dist: %d", traveled);
                     if( last_action == USER_DIR_ONOFF)
                         mcl_update(COUNTER2MM(traveled)/10.0, 0);
                     else{
                         mikes_log_val2(ML_INFO, "499", last_heading, base_data.heading);	 
                         mikes_log_val(ML_INFO, "ang_diff", -angle_difference(last_heading, base_data.heading));	 
                         mcl_update(0, -angle_difference(last_heading, base_data.heading));
                     }
                     reset_counters();
                     break;
            case KEY_ESC: program_runs = 0;
                     mikes_log(ML_INFO, "quit by ESC");
                     break;
            default: mvprintw(7, 10, "key: %d", ch);
        }
        last_action = user_dir;
        refresh();
    }

    delwin(mainwin);
    endwin();
    refresh();

    mikes_log(ML_INFO, "ncurses quits.");
    threads_running_add(-1);

    return 0;
}

void init_ncurses_control()
{
    if (!mikes_config.use_ncurses_control_console) return;

    pthread_t t;
    if (pthread_create(&t, 0, ncurses_control_thread, 0) != 0)
    {
        perror("mikes:ncurses");
        mikes_log(ML_ERR, "creating ncurses thread");
    }
    else threads_running_add(1);
}


