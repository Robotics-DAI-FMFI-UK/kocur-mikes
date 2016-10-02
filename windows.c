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

extern cairo_surface_t *gui_surface;
extern cairo_t *gui;

int gui_cairo_check_event(cairo_surface_t *sfc, int block);
void gui_shutdown();

void *gui_thread(void *arg)
{
    int ranges[RANGE_DATA_COUNT];
    double brkAngle = -135 / 180.0 * M_PI;
    double deltaAngle = 0.25 / 180.0 * M_PI;
    double guiWidth = 600;
    double guiHeight = 600;

    while (program_runs)
    {
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
	usleep(300000);
    } 

    gui_shutdown();
    return 0;
}

