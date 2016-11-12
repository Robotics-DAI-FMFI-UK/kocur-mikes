#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <cairo.h>
#include <cairo-xlib.h>

#include "mikes.h"
#include "mikes_logs.h"
#include "range_sensor.h"
#include "rfid_sensor.h"
#include "base_module.h"
#include "gui.h"

#define MAP_ZOOM_FACTOR 80
#define MAP_XOFFSET -120
#define MAP_YOFFSET -270
#define COMPASS_RADIUS 70

#define MAX_MAP_SIZE 32

#define DISPLAY_FREQUENCY 10
#define LOOP_DELAY 30000
#define DECAY 0.95
#define NOSEE_THRESHOLD 0.2

int gui_cairo_check_event(cairo_surface_t *sfc, int block);
void gui_shutdown();

const int minx = 2, miny = 4, maxx = 5, maxy = 6;

static rfid_data_type rfid_data;
static int ranges[RANGE_DATA_COUNT];
static int tagmap[MAX_MAP_SIZE][MAX_MAP_SIZE];
static double decaymap[MAX_MAP_SIZE][MAX_MAP_SIZE];
static double decay = DECAY;
static segments_type segments;


void *gui_thread(void *arg)
{
    double brkAngle = -135 / 180.0 * M_PI;
    double deltaAngle = 0.25 / 180.0 * M_PI;
    double guiWidth = 600;
    double guiHeight = 600;
    double cpWidth = 350;
    double cpHeight = 200;
    base_data_type base_data;
    int disp_counter = 0;

    int actual_seg = 0;

    for (int i = minx; i <= maxx; i++)
       for (int j = miny; j <= maxy; j++)
          decaymap[i][j] = 0;

    for (int i = minx; i <= maxx; i++)
        for (int j = miny; j <= maxy; j++)
            tagmap[i][j] = -1;

    while (program_runs)
    {
      disp_counter++;
      if (disp_counter == DISPLAY_FREQUENCY)
      {
        disp_counter = 0;
        actual_seg = 0;
        // LASER
        get_range_data(ranges);
        get_range_segments(&segments, 180*4, 145, 280);

        cairo_push_group(gui);
        cairo_set_source_rgb(gui, 1, 1, 1);
        cairo_paint(gui);
        cairo_set_line_width(gui, 2);

        for (int i = 0; i < RANGE_DATA_COUNT; i++)
        {

            if (ranges[i] > MAX_DISTANCE)
                ranges[i] = MAX_DISTANCE;

            int x = (int)(-ranges[i] / 8000.0 * guiWidth * 0.45 * sin(brkAngle + i * deltaAngle) +
                  guiWidth / 2);
            int y = (int)(ranges[i] / 8000.0 * guiWidth * 0.45 * cos(brkAngle + i * deltaAngle) +
                  guiHeight / 2);

            cairo_set_source_rgb(gui, 0.1, 0.1, 0.8);
            cairo_move_to(gui, x, guiHeight - y);
            cairo_line_to(gui, guiWidth / 2, guiHeight / 2);
            cairo_stroke(gui);

            if(actual_seg < (segments.nsegs_found)){
                if( i < segments.firstray[actual_seg]){ // before segment
                    cairo_set_source_rgb(gui, 1, 0.1, 0.2);
                }else if( i > segments.lastray[actual_seg]){ // after segment
                    actual_seg += 1;
                    cairo_set_source_rgb(gui, 1, 0.1, 0.2);
                }else{  // in segment
                    //if(segments.width[actual_seg] < 280)
                        cairo_set_source_rgb(gui, 0, 1, 0);
                    //else
                       // cairo_set_source_rgb(gui, (double)actual_seg/(double)segments.nsegs_found, (double)actual_seg/(double)segments.nsegs_found, (double)actual_seg/(double)segments.nsegs_found);
                }
            }
            else{
                cairo_set_source_rgb(gui, 1, 0.1, 0.2);
            }
//            cairo_set_source_rgb(gui, color, 0.1, 0.2);
            cairo_arc(gui, x, guiHeight - y, 3, 0, 2 * M_PI);
            cairo_stroke(gui);
        }
        //cairo_stroke(gui);
        cairo_pop_group_to_source(gui);
        cairo_paint(gui);
        cairo_surface_flush(gui_surface);
        //gui_cairo_check_event(gui_surface, 0);

        // MAP
        get_rfid_data(&rfid_data);
        cairo_push_group(map_gui);
        cairo_set_source_rgb(map_gui, 1, 1, 1);
        cairo_paint(map_gui);
        cairo_set_line_width(map_gui, 7);

        for (int i = 0; i < rfid_data.ntags; i++)
        {
            int x = rfid_data.x[i];
            int y = rfid_data.y[i];
            if ((x < minx) || (x > maxx) || (y < miny) || (y > maxy))
                mikes_log_val2(ML_WARN, "tag coordinates are out of map: ", x, y);
            else
            {
                int xcoor = rfid_data.x[i];
                int ycoor = rfid_data.y[i];
                tagmap[xcoor][ycoor] = rfid_data.a[i];
                decaymap[xcoor][ycoor] = 1;
            }
        }

        for (int i = minx; i <= maxx; i++)
            for (int j = miny; j <= maxy; j++)
            {
                switch(tagmap[i][j]){
                    case -1:
                        cairo_set_source_rgb(map_gui, 0, 0, 0);
                        break;
                    case 0:
                        cairo_set_source_rgb(map_gui, 0, 0, decaymap[i][j]);
                        break;
                    case 1:
                        cairo_set_source_rgb(map_gui, 0, decaymap[i][j], 0);
                        break;
                    case 2:
                        cairo_set_source_rgb(map_gui, decaymap[i][j], 0, 0);
                        break;
                    case 3:
                        cairo_set_source_rgb(map_gui, decaymap[i][j], 0, decaymap[i][j]);
                        break;
                    default:
                        cairo_set_source_rgb(map_gui, 1, 1, 1);
                        break;

                }
                cairo_arc(map_gui, i * MAP_ZOOM_FACTOR + MAP_XOFFSET, j * MAP_ZOOM_FACTOR + MAP_YOFFSET, 7, 0, 2 * M_PI);
                cairo_stroke(map_gui);
            }

        //cairo_stroke(gui);
        cairo_pop_group_to_source(map_gui);
        cairo_paint(map_gui);
        cairo_surface_flush(map_gui_surface);
        //gui_cairo_check_event(map_gui_surface, 0);

        cairo_push_group(cp_gui);
        cairo_set_source_rgb(cp_gui, 1, 1, 1);
        cairo_paint(cp_gui);
        cairo_set_line_width(cp_gui, 3);
        cairo_set_source_rgb(cp_gui, 0.1, 0.3, 1);
        cairo_arc(cp_gui, cpWidth / 2, cpHeight / 2, COMPASS_RADIUS + 5, 0, 2 * M_PI);
        cairo_stroke(cp_gui);
        get_base_data(&base_data);
        double compass_x = COMPASS_RADIUS * sin(M_PI * base_data.heading / 180.0);
        double compass_y = COMPASS_RADIUS * cos(M_PI * base_data.heading / 180.0);
        cairo_set_source_rgb(cp_gui, 1, 0, 0.2);
        cairo_move_to(cp_gui, cpWidth / 2 + compass_x, cpHeight / 2 - compass_y);
        cairo_line_to(cp_gui, cpWidth / 2, cpHeight / 2);
        cairo_stroke(cp_gui);
        cairo_pop_group_to_source(cp_gui);
        cairo_paint(cp_gui);
        cairo_surface_flush(cp_gui_surface);
      }

        int event = gui_cairo_check_event(cp_gui_surface, 0);
        if ((event != 0) && (!start_automatically)) 
          start_automatically = 1;
        switch (event)
        {
         case 0xff53:   // right arrow
            user_dir = 1;
            user_control = 1;
            //printf("----------------------RIGHT\n");
            break;

         case 0xff51:   // left arrow
            user_dir = 2;
            user_control = 1;
            //printf("----------------------LEFT\n");
            break;

         case 65364:    // down arrow
            user_dir = 3;
            user_control = 1;
            //printf("----------------------BACK\n");
            break;

         case 65362:    // up arrow
            user_dir = 4;
            user_control = 1;
            //printf("----------------------ON/OFF\n");
            break;

         case 0xff1b:   // Esc
            program_runs = 0;
            mikes_log(ML_INFO, "quit by ESC\n");
            break;

         case 32:  //space - backup now
            user_dir = 5;
            user_control = 1;
            break;

         case -1:       // left mouse button
            //printf("-----------------------USER CONTROL\n");
            user_control = 1 - user_control;
            break;

         //default:
         //   printf("event %d\n", event);
        }

        for (int i = minx; i <= maxx; i++)
           for (int j = miny; j <= maxy; j++)
           {
              decaymap[i][j] *= decay;
              if (decaymap[i][j] < NOSEE_THRESHOLD) tagmap[i][j] = -1;
           }

    usleep(LOOP_DELAY);
    }

    gui_shutdown();
    mikes_log(ML_INFO, "gui quits.");
    threads_running_add(-1);
    return 0;
}

