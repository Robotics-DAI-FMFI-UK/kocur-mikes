#include <stdio.h>
#include <stdlib.h>
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
#include "mcl.h"

#define MAP_ZOOM_FACTOR_HYPO 2
#define MAP_ZOOM_FACTOR (MAP_ZOOM_FACTOR_HYPO * 40)
#define MAP_XOFFSET 0 // -120
#define MAP_YOFFSET 0 // -270
#define COMPASS_RADIUS 70

#define MAX_MAP_SIZE 32

#define DISPLAY_FREQUENCY 10
#define LOOP_DELAY 30000
#define DECAY 0.95
#define NOSEE_THRESHOLD 0.2

#define RIGHT_ARROW 0xff53
#define LEFT_ARROW 0xff51
#define UP_ARROW 0xff52
#define DOWN_ARROW 0xff54
#define ESC_KEY 0xff1b
#define LEFT_MOUSE_BUTTON -1

#define RAY_USUAL_TYPE 1
#define RAY_AZIMUTH_TYPE 2

int gui_cairo_check_event(cairo_surface_t *sfc, int block);
void gui_shutdown();

const int minx = 1, miny = 1, maxx = 8, maxy = 8;

static rfid_data_type rfid_data;
static hypo_t hypo[HYPO_COUNT];
static int ranges[RANGE_DATA_COUNT];
static int tagmap[MAX_MAP_SIZE][MAX_MAP_SIZE];
static double decaymap[MAX_MAP_SIZE][MAX_MAP_SIZE];
static double decay = DECAY;
static segments_type segments;

static double guiWidth = 600;
static double guiHeight = 600;

void draw_ray(int i, int ray_type)
{
    int x = (int)(-ranges[i] / 8000.0 * guiWidth * 0.45 * sin(i * SIZE_OF_ONE_STEP - TOTAL_ANGLE / 2) + guiWidth / 2);
    int y = (int)(ranges[i] / 8000.0 * guiWidth * 0.45 * cos(i * SIZE_OF_ONE_STEP - TOTAL_ANGLE / 2) + guiHeight / 2);

    if (ray_type == RAY_USUAL_TYPE)
      cairo_set_source_rgb(gui, 0.1, 0.1, 0.8);
    else if (ray_type == RAY_AZIMUTH_TYPE)
      cairo_set_source_rgb(gui, 0.8, 0.8, 0.3);

    cairo_move_to(gui, x, guiHeight - y);
    cairo_line_to(gui, guiWidth / 2, guiHeight / 2);
    cairo_stroke(gui);
    cairo_set_source_rgb(gui, 1, 0.3, 0.3);
    cairo_arc(gui, x, guiHeight - y, 2, 0, 2 * M_PI);
    cairo_stroke(gui);
}

void draw_segment(int ray1, int ray2)
{
    int x1 = (int)(-ranges[ray1] / 8000.0 * guiWidth * 0.45 * sin(ray1 * SIZE_OF_ONE_STEP - TOTAL_ANGLE / 2) + guiWidth / 2);
    int y1 = (int)(ranges[ray1] / 8000.0 * guiWidth * 0.45 * cos(ray1 * SIZE_OF_ONE_STEP - TOTAL_ANGLE / 2) + guiHeight / 2);
    int x2 = (int)(-ranges[ray2] / 8000.0 * guiWidth * 0.45 * sin(ray2 * SIZE_OF_ONE_STEP - TOTAL_ANGLE / 2) + guiWidth / 2);
    int y2 = (int)(ranges[ray2] / 8000.0 * guiWidth * 0.45 * cos(ray2 * SIZE_OF_ONE_STEP - TOTAL_ANGLE / 2) + guiHeight / 2);

    cairo_set_source_rgb(gui, 1, 1, 0.5);
    cairo_rectangle(gui, x2, guiHeight - y2, abs(x2 - x1), abs(y2 - y1));
    cairo_stroke(gui);
}

void *gui_thread(void *arg)
{
    double cpWidth = 350;
    double cpHeight = 200;
    base_data_type base_data;
    int disp_counter = 0;

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
        get_base_data(&base_data);
        disp_counter = 0;
        // LASER
        get_range_data(ranges);
        get_range_segments(&segments, 180*4, 145, 280);

        //mikes_log_val(ML_INFO, "cubes found: ", segments.nsegs_found);

        cairo_push_group(gui);
        cairo_set_source_rgb(gui, 1, 1, 1);
        cairo_paint(gui);
        cairo_set_line_width(gui, 2);

        for (int i = 0; i < RANGE_DATA_COUNT; i++)
        {
          if (ranges[i] > MAX_DISTANCE) ranges[i] = MAX_DISTANCE;
          draw_ray(i, RAY_USUAL_TYPE);
        }

        for (int i = 0; i < segments.nsegs_found; i++)
          draw_segment(segments.firstray[i], segments.lastray[i]);

        if (get_current_azimuth() != NO_AZIMUTH)
          draw_ray(azimuth2ray(get_current_azimuth() - base_data.heading), RAY_AZIMUTH_TYPE);

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
        get_mcl_data(hypo);

        for (int i = 0; i < HYPO_COUNT; i++)
        {
            double x = hypo[i].x;
            double y = hypo[i].y;
            double alpha = hypo[i].alpha;
            double w = hypo[i].w;

            cairo_set_source_rgb(map_gui, 1 - w*0.8 - 0.1, 1 - w*0.8 - 0.1, 1 - w*0.8 - 0.1);
            cairo_set_line_width(map_gui, 3);

            cairo_set_line_width(map_gui, 1);
            double hypoax = 5*cos(M_PI * alpha / 180);
            double hypoay = -5*sin(M_PI * alpha / 180);

            cairo_move_to(map_gui, (int) (x * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_XOFFSET),        (int) (y * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_YOFFSET));
            cairo_line_to(map_gui, (int) (x * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_XOFFSET+hypoax), (int) (y * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_YOFFSET + hypoay));
            cairo_stroke(map_gui);

            hypoax = 3*cos(M_PI * alpha / 180 + M_PI/2);
            hypoay = -3*sin(M_PI * alpha / 180 + M_PI/2);
            cairo_move_to(map_gui, (int) (x * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_XOFFSET),        (int) (y * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_YOFFSET));
            cairo_line_to(map_gui, (int) (x * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_XOFFSET+hypoax), (int) (y * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_YOFFSET + hypoay));
            cairo_stroke(map_gui);

            hypoax = 3*cos(M_PI * alpha / 180-M_PI/2);
            hypoay = -3*sin(M_PI * alpha / 180- M_PI/2);
            cairo_move_to(map_gui, (int) (x * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_XOFFSET),        (int) (y * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_YOFFSET));
            cairo_line_to(map_gui, (int) (x * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_XOFFSET+hypoax), (int) (y * MAP_ZOOM_FACTOR_HYPO + MAP_ZOOM_FACTOR + MAP_YOFFSET + hypoay));
            cairo_stroke(map_gui);

        }


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
         case RIGHT_ARROW:
            user_dir = USER_DIR_RIGHT;
            user_control = 1;
            break;

         case LEFT_ARROW:
            user_dir = USER_DIR_LEFT;
            user_control = 1;
            break;

         case DOWN_ARROW:
            user_dir = USER_DIR_BACK;
            user_control = 1;
            break;

         case UP_ARROW:
            user_dir = USER_DIR_ONOFF;
            user_control = 1;
            break;

         case ESC_KEY:
            program_runs = 0;
            mikes_log(ML_INFO, "quit by ESC");
            break;

         case 32:
            user_dir = USER_DIR_BACKUP;
            user_control = 1;
            break;

         case LEFT_MOUSE_BUTTON:
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

