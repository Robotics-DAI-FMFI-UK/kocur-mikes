#include <ncurses.h>
#include <pthread.h>

#include "mikes.h"
#include "mikes_logs.h"
#include "config_mikes.h"

#define KEY_UP_ARROW 259
#define KEY_DOWN_ARROW 258
#define KEY_LEFT_ARROW 260
#define KEY_RIGHT_ARROW 261
#define KEY_ESC 27
#define KEY_A 97
#define KEY_SPACE 32

static WINDOW *mainwin;

void *ncurses_control_thread(void *arg)
{
    int ch;
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
        deleteln();
        switch (ch)
        {
            case KEY_UP_ARROW: mvprintw(7, 10, "on/off");
                     user_dir = USER_DIR_ONOFF;
                     user_control = 1;
                      break;
            case KEY_LEFT_ARROW: mvprintw(7, 10, "left");
                     user_dir = USER_DIR_LEFT;
                     user_control = 1;
                      break;
            case KEY_RIGHT_ARROW: mvprintw(7, 10, "right");
                     user_dir = USER_DIR_RIGHT;
                     user_control = 1;
                       break;
            case KEY_DOWN_ARROW: mvprintw(7, 10, "back");
                     user_dir = USER_DIR_BACK;
                     user_control = 1;
                       break;
            case KEY_SPACE: mvprintw(7, 10, "backup");
                     user_dir = USER_DIR_BACKUP;
                     user_control = 1;
                     break;
            case KEY_A: user_control = 1 - user_control;
                     mvprintw(7, 10, "autonomous: %d", user_control);
                     break; 
            case KEY_ESC: program_runs = 0;
                     mikes_log(ML_INFO, "quit by ESC");
                     break;
            default: mvprintw(7, 10, "key: %d", ch);
        }
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


