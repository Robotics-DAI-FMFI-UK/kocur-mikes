#include <ncurses.h>
#include <pthread.h>

#include "mikes.h"
#include "mikes_logs.h"
#include "config_mikes.h"

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

    mvaddstr(5, 10, "Control Kocur Mikes with arrow keys, space to backup, ESC to quit");
    mvprintw(7, 10, "You pressed: ");
    refresh();

    /*  Loop until user presses 'q'  */

    while (program_runs)
    {
        ch = getch();
        if (!start_automatically) start_automatically = 1;
        deleteln();
        switch (ch)
        {
            case 259: mvprintw(7, 10, "on/off");
                     user_dir = 4;
                     user_control = 1;
                      break;
            case 260: mvprintw(7, 10, "left");
                     user_dir = 2;
                     user_control = 1;
                      break;
            case 261: mvprintw(7, 10, "right");
                     user_dir = 3;
                     user_control = 1;
                       break;
            case 258: mvprintw(7, 10, "back");
                     user_dir = 3;
                     user_control = 1;
                       break;
            case 32: mvprintw(7, 10, "backup");
                     user_dir = 5;
                     user_control = 1;
                     break;
            case 97: user_control = 1 - user_control;
                     mvprintw(7, 10, "autonomous: %d", user_control);
                     break; 
            case 27: program_runs = 0;
                     mikes_log(ML_INFO, "quit by ESC\n");
                     break;
            default: mvprintw(7, 10, "key: %d", ch);
        }
        refresh();
    }

    /*  Clean up after ourselves  */

    delwin(mainwin);
    endwin();
    refresh();

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


