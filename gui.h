#ifndef _GUI_H_
#define _GUI_H_

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <cairo.h>
#include <cairo-xlib.h>

void init_gui();

extern cairo_surface_t *gui_surface;
extern cairo_t *gui;
extern cairo_surface_t *map_gui_surface;
extern cairo_t *map_gui;
extern cairo_surface_t *cp_gui_surface;
extern cairo_t *cp_gui;

int gui_cairo_check_event(cairo_surface_t *sfc, int block);

#endif
