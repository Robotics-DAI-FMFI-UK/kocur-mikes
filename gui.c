#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>
#include <cairo.h>
#include <cairo-xlib.h>

#include "mikes.h"
#include "mikes_logs.h"
#include "config_mikes.h"

void gui_fullscreen(Display* dpy, Window win)
{
  Atom atoms[2] = { XInternAtom(dpy, "_NET_WM_STATE_FULLSCREEN", False), None };
  XChangeProperty(dpy, win, XInternAtom(dpy, "_NET_WM_STATE", False),
                  XA_ATOM, 32, PropModeReplace, (unsigned char*) atoms, 1);
}

int gui_cairo_check_event(cairo_surface_t *sfc, int block)
{
   char keybuf[8];
   KeySym key;
   XEvent e;

   for (;;)
   {
      if (block || XPending(cairo_xlib_surface_get_display(sfc)))
         XNextEvent(cairo_xlib_surface_get_display(sfc), &e);
      else
         return 0;

      switch (e.type)
      {
         case ButtonPress:
            return -e.xbutton.button;
         case KeyPress:
            XLookupString(&e.xkey, keybuf, sizeof(keybuf), &key, NULL);
            return key;
         //default:
            //fprintf(stderr, "Dropping unhandled XEevent.type = %d.\n", e.type);
      }
   }
}

static Display *dsp;
static Screen *scr;
static int screen;
static unsigned char x_opened;

cairo_surface_t *gui_cairo_create_x11_surface(int *x, int *y)
{
    Drawable da;
    cairo_surface_t *sfc;

    if (!x_opened)
    {
        if ((dsp = XOpenDisplay(0)) == 0)
        {
            mikes_log(ML_WARN, "could not open X display, will not use graphics");
            return 0;
        }
        screen = DefaultScreen(dsp);
        scr = DefaultScreenOfDisplay(dsp);
    }
    if (!*x || !*y)
    {
         *x = WidthOfScreen(scr), *y = HeightOfScreen(scr);
         da = XCreateSimpleWindow(dsp, DefaultRootWindow(dsp), 0, 0, *x, *y, 0, 0, 0);
         gui_fullscreen (dsp, da);
    }
   else
      da = XCreateSimpleWindow(dsp, DefaultRootWindow(dsp), 0, 0, *x, *y, 0, 0, 0);
   XSelectInput(dsp, da, ButtonPressMask | KeyPressMask);
   XMapWindow(dsp, da);

   sfc = cairo_xlib_surface_create(dsp, da, DefaultVisual(dsp, screen), *x, *y);
   cairo_xlib_surface_set_size(sfc, *x, *y);

   x_opened = 1;
   return sfc;
}

cairo_surface_t *gui_surface;
cairo_t *gui;
cairo_surface_t *map_gui_surface;
cairo_t *map_gui;

extern void *gui_thread(void *arg);

void init_gui()
{
    x_opened = 0;

    if (!mikes_config.with_gui)
    {
        mikes_log(ML_INFO, "gui supressed by config.");
        return;
    }
    int width = 600;
    int height = 600;
    gui_surface = gui_cairo_create_x11_surface(&width, &height);
    gui = cairo_create(gui_surface);


    int mapwidth = 350;
    int mapheight = 250;
    map_gui_surface = gui_cairo_create_x11_surface(&mapwidth, &mapheight);
    map_gui = cairo_create(map_gui_surface);

    pthread_t t;
    if (pthread_create(&t, 0, gui_thread, 0) != 0)
    {
        perror("mikes:gui");
        mikes_log(ML_ERR, "creating gui thread");
    }
    else threads_running_add(1);
}

void gui_shutdown()
{
    if (!mikes_config.with_gui) return;
    cairo_destroy(gui);
    cairo_destroy(map_gui);
    Display *dsp = cairo_xlib_surface_get_display(gui_surface);
    cairo_surface_destroy(gui_surface);
    cairo_surface_destroy(map_gui_surface);
    XCloseDisplay(dsp);
}

