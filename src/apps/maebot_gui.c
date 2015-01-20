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
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/vx_remote_display_source.h"
#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

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
    parameter_gui_t *pg;

    // LCM
    lcm_t *lcm;
    const char *laser_channel;

    // image stuff
    char *img_url;
    int img_height;
    int img_width;

    // vx stuff
    vx_application_t vxapp;
    vx_world_t *vxworld; // where vx objects are live
    vx_event_handler_t *vxeh; // for getting mouse, key, and touch events
    vx_mouse_event_t last_mouse_event;

    // threads
    pthread_t animate_thread;
    pthread_t lcm_thread;

    float x_pos;
    float y_pos;
    float a_pos;

    // for accessing the arrays
    pthread_mutex_t laser_mutex;
    pthread_mutex_t gui_mutex;
    maebot_laser_scan_t laser_info;
    bool add_laser;

    //float *lasers[4];
    //int32_t rays[4];
    int new_laser;
};

// save laser information
static void
laser_handler (const lcm_recv_buf_t *rbuf,
                const char *channel,
                const maebot_laser_scan_t *msg,
                void *state_p)
{
    state_t *state = state_p;

    //pthread_mutex_lock(&state->laser_mutex);
    //state->laser_info = *msg;
    //state->add_laser = true;
    //pthread_mutex_unlock(&state->laser_mutex);

    float *points = calloc(6 * msg->num_ranges, sizeof(float));

    int i;
    for (i = 0; i < msg->num_ranges; ++i){
        points[6*i+0] = state->x_pos;
        points[6*i+1] = state->y_pos;
        points[6*i+2] = 0.0f;
        points[6*i+3] = cos(state->a_pos + msg->thetas[i]) * msg->ranges[i] + state->x_pos;
        points[6*i+4] = sin(state->a_pos + msg->thetas[i]) * msg->ranges[i] + state->y_pos;
        points[6*i+5] = 0.0f;
    }

    
    state->new_laser = (state->new_laser > 2) ? 0 : state->new_laser + 1;
    char buff_name[20];
    sprintf(buff_name, "laser-buff-%d", state->new_laser);

    pthread_mutex_lock(&state->gui_mutex);
    vx_buffer_t *buff = vx_world_get_buffer (state->vxworld, buff_name);
    vx_resc_t *verts = vx_resc_copyf(points, 6*msg->num_ranges);
    vx_buffer_add_back(buff, vxo_lines(verts, 2*msg->num_ranges, GL_LINES, vxo_lines_style(vx_olive, 1.0f)));
    vx_buffer_swap (buff);
    pthread_mutex_lock(&state->gui_mutex);

    //int i;
    //for (i = 0; i < 4; ++i){
    //    int idx = state->new_laser + i;

    //    if (idx < 0)
    //        continue;

    //    for ()
    //    vx_buffer_add_back (buff, vxo_rect(vxo_mesh_style(vx_white)));
    //}
}

// receive lcm messages
void *
receive_lcm_msg (void *data)
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

void *
animate_thread (void *data)
{
    //const int fps = 60;
    state_t *state = data;

    int i;
    pthread_mutex_lock(&state->laser_mutex);
    for (i = 0; i < state->laser_info.num_ranges; ++i){
        printf("angle: %6.3f - range: %6.3f\n", 
               state->laser_info.thetas[i], 
               state->laser_info.ranges[i]);
    }
    pthread_mutex_unlock(&state->laser_mutex);

    return NULL;
}

static void
my_param_changed (parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
    if (0==strcmp ("sl1", name))
        printf ("sl1 = %f\n", pg_gd (pg, name));
    else if (0==strcmp ("sl2", name))
        printf ("sl2 = %d\n", pg_gi (pg, name));
    else if (0==strcmp ("cb1", name) || 0==strcmp ("cb2", name))
        printf ("%s = %d\n", name, pg_gb (pg, name));
    else
        printf ("%s changed\n", name);
}

static int
mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    return 0;
}

static int
key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key)
{
    return 0;
}

static int
touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse)
{
    return 0; // Does nothing
}


state_t *
state_create (void)
{
    state_t *state = calloc (1, sizeof(*state));

    state->lcm = lcm_create (NULL);

    state->vxworld = vx_world_create ();
    state->vxeh = calloc (1, sizeof(*state->vxeh));
    state->vxeh->key_event = key_event;
    state->vxeh->mouse_event = mouse_event;
    state->vxeh->touch_event = touch_event;
    state->vxeh->dispatch_order = 100;
    state->vxeh->impl = state; // this gets passed to events, so store useful struct here!

    state->vxapp.display_started = eecs467_default_display_started;
    state->vxapp.display_finished = eecs467_default_display_finished;
    state->vxapp.impl = eecs467_default_implementation_create (state->vxworld, state->vxeh);

    state->new_laser = -4;

    return state;
}

void
state_destroy (state_t *state)
{
    if (!state)
        return;

    if (state->lcm)
        lcm_destroy(state->lcm);

    if (state->vxeh)
        free (state->vxeh);

    if (state->gopt)
        getopt_destroy (state->gopt);

    if (state->pg)
        pg_destroy (state->pg);

    free (state);
}

int
main (int argc, char *argv[])
{
    eecs467_init (argc, argv);
    state_t *state = state_create ();

    state->gopt = getopt_create ();
    getopt_add_bool (state->gopt, 'h', "help", 0, "Show help");
    getopt_add_string (state->gopt, '\0', "laser-channel", "CORNER_LASER_SCAN", "LCM laser channel");

    state->laser_channel = getopt_get_string (state->gopt, "laser-channel");

    if (!getopt_parse(state->gopt, argc, argv, 1) || getopt_get_bool(state->gopt, "help")) {
        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(state->gopt);
        return 0;
    }

    vx_remote_display_source_t *cxn = vx_remote_display_source_create (&state->vxapp);

    state->pg = pg_create ();

    parameter_listener_t *my_listener = calloc (1, sizeof(*my_listener));
    my_listener->impl = state;
    my_listener->param_changed = my_param_changed;
    pg_add_listener (state->pg, my_listener);

    pthread_create (&state->animate_thread, NULL, animate_thread, state);
    pthread_create (&state->lcm_thread, NULL, receive_lcm_msg, state);

    eecs467_gui_run (&state->vxapp, state->pg, 768, 768);

    //state->running = 0;
    pthread_join (state->animate_thread, NULL);
    pthread_join (state->lcm_thread, NULL);

    free (my_listener);
    state_destroy (state);
    vx_remote_display_source_destroy (cxn);
    vx_global_destroy ();

	return 0;
}

