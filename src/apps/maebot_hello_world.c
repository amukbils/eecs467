#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <signal.h>

//lcm
#include <lcm/lcm.h>
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_targeting_laser_command_t.h"
#include "lcmtypes/maebot_leds_command_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_laser_scan_t.h"

// core api
//#include "vx/vx.h"
//#include "vx/vx_util.h"
//#include "vx/vx_remote_display_source.h"
//#include "vx/gtk/vx_gtk_display_source.h"

// drawables
//#include "vx/vxo_drawables.h"

// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"

// imagesource
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"
#include "eecs467_util.h" // This is where a lot of the internals live

#define MAX_REVERSE_SPEED -1.0f
#define MAX_FORWARD_SPEED 1.0f

typedef struct state state_t;
struct state {
    //bool running;
    getopt_t *gopt;
//    parameter_gui_t *pg;

    // LCM
    lcm_t *lcm;
    const char *laser_channel;
    const char *corner_laser_channel;
    // image stuff
//    char *img_url;
//    int img_height;
//    int img_width;

    // vx stuff
    //vx_application_t vxapp;
    //vx_world_t *vxworld; // where vx objects are live
    //vx_event_handler_t *vxeh; // for getting mouse, key, and touch events
    //vx_mouse_event_t last_mouse_event;

    // threads
    //pthread_t animate_thread;
    pthread_t laser_thread;

    pthread_mutex_t laser_mutex;
//    maebot_laser_scan_t *laser_info;
    bool need_scan;
};

static void 
change_need_scan(bool value, state_t *state)
{
    pthread_mutex_lock(&(state->laser_mutex));
    state->need_scan = value;
    pthread_mutex_unlock(&(state->laser_mutex));
}

// save laser information to a different channel
static void
laser_handler (const lcm_recv_buf_t *rbuf,
                const char *channel,
                const maebot_laser_scan_t *msg,
                void *state_p)
{
    state_t *state = state_p;

    if (!state->need_scan)
        return;

    maebot_laser_scan_t_publish(state->lcm, state->corner_laser_channel, msg);
    printf("republished laser scan\n");

    printf("num_ranges: %d", msg->num_ranges);
    int i;
    for (i = 0; i < msg->num_ranges; ++i){
        printf("%6.3f %6.3f\n", msg->thetas[i], msg->ranges[i]);
    }

    change_need_scan(false, state);
}

void *
laser_thread (void *data)
{
    state_t *state = data;
    maebot_laser_scan_t_subscribe (state->lcm,
                                   state->laser_channel,
                                   laser_handler,
                                   state);

    while (1) 
        lcm_handle_timeout (state->lcm, 1000/5);

    return NULL;
}

state_t *
state_create (void)
{
    state_t *state = calloc (1, sizeof(*state));

    state->lcm = lcm_create (NULL);

    //state->vxworld = vx_world_create ();
    //state->vxeh = calloc (1, sizeof(*state->vxeh));
    //state->vxeh->key_event = key_event;
    //state->vxeh->mouse_event = mouse_event;
    //state->vxeh->touch_event = touch_event;
    //state->vxeh->dispatch_order = 100;
    //state->vxeh->impl = state; // this gets passed to events, so store useful struct here!

    //state->vxapp.display_started = eecs467_default_display_started;
    //state->vxapp.display_finished = eecs467_default_display_finished;
    //state->vxapp.impl = eecs467_default_implementation_create (state->vxworld, state->vxeh);

    return state;
}

void
state_destroy (state_t *state)
{
    if (!state)
        return;

    if (state->lcm)
        lcm_destroy(state->lcm);

    //if (state->vxeh)
    //    free (state->vxeh);

    if (state->gopt)
        getopt_destroy (state->gopt);

    //if (state->pg)
    //    pg_destroy (state->pg);

    free (state);
}

int
main (int argc, char *argv[])
{
    //eecs467_init (argc, argv);
    state_t *state = state_create ();

    state->gopt = getopt_create ();
    getopt_add_bool (state->gopt, 'h', "help", 0, "Show help");
    getopt_add_string (state->gopt, '\0', "laser-channel", "MAEBOT_LASER_SCAN", "LCM laser channel");
    getopt_add_string (state->gopt, '\0', "corner-laser-channel", "CORNER_LASER_SCAN", "LCM corner laser channel");

    state->laser_channel = getopt_get_string (state->gopt, "laser-channel");
    state->corner_laser_channel = getopt_get_string (state->gopt, "corner-laser-channel");

    if (!getopt_parse(state->gopt, argc, argv, 1) || getopt_get_bool(state->gopt, "help")) {
        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(state->gopt);
        return 0;
    }

    //vx_remote_display_source_t *cxn = vx_remote_display_source_create (&state->vxapp);

    //state->pg = pg_create ();

    //parameter_listener_t *my_listener = calloc (1, sizeof(*my_listener));
    //my_listener->impl = state;
    //my_listener->param_changed = my_param_changed;
    //pg_add_listener (state->pg, my_listener);

    pthread_create (&state->laser_thread, NULL, laser_thread, state);
    //pthread_create (&state->status_thread, NULL, status_loop, state);

    //eecs467_gui_run (&state->vxapp, state->pg, 768, 768);

    state->need_scan = 0;

    change_need_scan(true, state);

    pthread_join (state->laser_thread, NULL);

    //free (my_listener);
    state_destroy (state);
    //vx_remote_display_source_destroy (cxn);
    //vx_global_destroy ();

	return 0;
}

