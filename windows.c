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

extern cairo_surface_t *gui_surface;
extern cairo_t *gui;
extern cairo_surface_t *map_gui_surface;
extern cairo_t *map_gui;

int gui_cairo_check_event(cairo_surface_t *sfc, int block);
void gui_shutdown();

const int minx = 2, miny = 4, maxx = 5, maxy = 6;

void *gui_thread(void *arg)
{
    int ranges[RANGE_DATA_COUNT];
    int tagmap[maxx-minx+1][maxy-miny+1];
    double brkAngle = -135 / 180.0 * M_PI;
    double deltaAngle = 0.25 / 180.0 * M_PI;
    double guiWidth = 600;
    double guiHeight = 600;

    static rfid_data_type rfid_data;

    while (program_runs)
    {
        // LASER
        get_range_data(ranges);

        cairo_push_group(gui);
        cairo_set_source_rgb(gui, 1, 1, 1);
        cairo_paint(gui);
        cairo_set_line_width(gui, 2);

        for (int i = 0; i < RANGE_DATA_COUNT; i++)
        {
            int x = (int)(-ranges[i] / 8000.0 * guiWidth * 0.45 * sin(brkAngle + i * deltaAngle) +
                  guiWidth / 2);
            int y = (int)(ranges[i] / 8000.0 * guiWidth * 0.45 * cos(brkAngle + i * deltaAngle) +
                  guiHeight / 2);
            cairo_set_source_rgb(gui, 0.1, 0.1, 0.8);
            cairo_move_to(gui, x, guiHeight - y);
        cairo_line_to(gui, guiWidth / 2, guiHeight / 2);
        cairo_stroke(gui);
            cairo_set_source_rgb(gui, 1, 0.1, 0.2);
        cairo_arc(gui, x, guiHeight - y, 3, 0, 2 * M_PI);
        cairo_stroke(gui);
        }
        //cairo_stroke(gui);
        cairo_pop_group_to_source(gui);
        cairo_paint(gui);
        cairo_surface_flush(gui_surface);
        gui_cairo_check_event(gui_surface, 0);

        // MAP
        get_rfid_data(&rfid_data);
        cairo_push_group(map_gui);
        cairo_set_source_rgb(map_gui, 1, 1, 1);
        cairo_paint(map_gui);
        cairo_set_line_width(map_gui, 7);

        for (int i = minx; i <= maxx; i++)
            for (int j = miny; j <= maxy; j++)
                tagmap[i][j] = -1;

        for (int i = 0; i < rfid_data.ntags; i++)
        {
            tagmap[rfid_data.x[i]][rfid_data.y[i]] = rfid_data.a[i];
        }

        for (int i = minx; i <= maxx; i++)
            for (int j = miny; j <= maxy; j++)
            {
                switch(tagmap[i][j]){
                    case -1:
                        cairo_set_source_rgb(map_gui, 0, 0, 0);
                        break;
                    case 0:
                        cairo_set_source_rgb(map_gui, 0, 0, 1);
                        break;
                    case 1:
                        cairo_set_source_rgb(map_gui, 0, 1, 0);
                        break;
                    case 2:
                        cairo_set_source_rgb(map_gui, 1, 0, 0);
                        break;
                    case 3:
                        cairo_set_source_rgb(map_gui, 1, 0, 1);
                        break;
                    default:
                        cairo_set_source_rgb(map_gui, 1, 1, 1);
                        break;

                }
                cairo_arc(map_gui, i*30+50, j*30+50, 7, 0, 2 * M_PI);
                cairo_stroke(map_gui);
            }


        //cairo_stroke(gui);
        cairo_pop_group_to_source(map_gui);
        cairo_paint(map_gui);
        cairo_surface_flush(map_gui_surface);
        gui_cairo_check_event(map_gui_surface, 0);

    usleep(300000);
    }

    gui_shutdown();
    mikes_log(ML_INFO, "gui quits.");
    threads_running_add(-1);
    return 0;
}

