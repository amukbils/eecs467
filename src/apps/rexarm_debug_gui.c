#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>

//lcm
#include <lcm/lcm.h>
#include "lcmtypes/dynamixel_command_list_t.h"
#include "lcmtypes/dynamixel_command_t.h"
#include "lcmtypes/dynamixel_status_list_t.h"
#include "lcmtypes/dynamixel_status_t.h"

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

#include "eecs467_util.h"    // This is where a lot of the internals live

#define NUM_SERVOS 6

// It's good form for every application to keep its state in a struct.
typedef struct state state_t;
struct state {
    bool running;

    getopt_t        *gopt;
    parameter_gui_t *pg;

    // LCM
    lcm_t *lcm;
    const char *status_channel;

    // image stuff
    char *img_url;
    int   img_height;
    int   img_width;

    // vx stuff
    vx_application_t    vxapp;
    vx_world_t         *vxworld;      // where vx objects are live
    vx_event_handler_t *vxeh; // for getting mouse, key, and touch events
    vx_mouse_event_t    last_mouse_event;

    // threads
    pthread_t animate_thread;
    pthread_t status_thread;

    // for accessing the arrays
    pthread_mutex_t mutex;
    double servo_pos[NUM_SERVOS];
};


// Save the positions
static void
status_handler (const lcm_recv_buf_t *rbuf,
                const char *channel,
                const dynamixel_status_list_t *msg,
                void *user)
{
    state_t *state = user;
    for (int id = 0; id < msg->len; id++) {
        dynamixel_status_t stat = msg->statuses[id];
        state->servo_pos[id] = stat.position_radians;
    }
}

void *
status_loop (void *data)
{
    state_t *state = data;
    dynamixel_status_list_t_subscribe (state->lcm,
                                       state->status_channel,
                                       status_handler,
                                       state);
    const int hz = 15;
    while (1) {
        int status = lcm_handle_timeout (state->lcm, 1000/hz);
        if (status <= 0)
            continue;
    }

    return NULL;
}

// === Parameter listener =================================================
// This function is handed to the parameter gui (via a parameter listener)
// and handles events coming from the parameter gui. The parameter listener
// also holds a void* pointer to "impl", which can point to a struct holding
// state, etc if need be.
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

// draw the image
void *
animate_thread (void *data)
{
    const int fps = 60;
    state_t *state = data;

    const char *servo_name[NUM_SERVOS];
    servo_name[0] = "Base";
    servo_name[1] = "Sholder";
    servo_name[2] = "Elbow";
    servo_name[3] = "Wrist1";
    servo_name[4] = "Wrist2";
    servo_name[5] = "Finger";

    // Continue running until we are signaled otherwise. This happens
    // when the window is closed/Ctrl+C is received.
    while (state->running) {
        vx_buffer_t *buff = vx_world_get_buffer (state->vxworld, "osc-rec");
        vx_buffer_add_back (buff, vxo_rect(vxo_mesh_style(vx_white)));

	for (int id = 0; id < NUM_SERVOS; ++id){
            float *color;
	    char str[40];
	    double offset = 0.03;

            if (state->servo_pos[id] < 0){
                color = vx_red;
                offset *= -1;
            }else{
                color = vx_blue;
            }
            
            vx_object_t *vxo_rec = vxo_chain (vxo_mat_translate3(0.035*state->servo_pos[id], id*0.13-0.36, 0),
                                              vxo_mat_scale3(0.07*state->servo_pos[id], 0.07, 0.07),
                                              vxo_rect(vxo_mesh_style (color)));

            sprintf(str, "<<right,#000000,serif>>%s", servo_name[id]);
            vx_buffer_add_back(buff, vxo_chain(vxo_mat_translate3(0, id*0.13-0.3, 0),
					       vxo_mat_scale3(0.001, 0.001, 0.001),
					       vxo_text_create(VXO_TEXT_ANCHOR_LEFT, str)));

            sprintf(str, "<<right,#000000,serif>>%0.3f", state->servo_pos[id]);
            vx_buffer_add_back(buff, vxo_chain(vxo_mat_translate3(0.07*state->servo_pos[id]+offset, id*0.13-0.36, 0),
					       vxo_mat_scale3(0.001, 0.001, 0.001),
					       vxo_text_create(VXO_TEXT_ANCHOR_CENTER, str)));

            vx_buffer_add_back(buff, vxo_rec);
	}

        vx_buffer_swap (vx_world_get_buffer (state->vxworld, "osc-rec"));

        usleep (1000000/fps);
    }

    return NULL;
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

    state->running = 1;

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

    // Parse arguments from the command line, showing the help
    // screen if required
    state->gopt = getopt_create ();
    getopt_add_bool   (state->gopt,  'h', "help", 0, "Show help");
    getopt_add_string (state->gopt, '\0', "status-channel", "ARM_STATUS", "LCM status channel");

    state->status_channel = getopt_get_string (state->gopt, "status-channel");

    if (!getopt_parse (state->gopt, argc, argv, 1) || getopt_get_bool (state->gopt, "help")) {
        printf ("Usage: %s [--url=CAMERAURL] [other options]\n\n", argv[0]);
        getopt_do_usage (state->gopt);
        exit (EXIT_FAILURE);
    }

    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your display.
    vx_remote_display_source_t *cxn = vx_remote_display_source_create (&state->vxapp);

    // Initialize a parameter gui
    state->pg = pg_create ();

    parameter_listener_t *my_listener = calloc (1, sizeof(*my_listener));
    my_listener->impl = state;
    my_listener->param_changed = my_param_changed;
    pg_add_listener (state->pg, my_listener);

    // Launch our worker threads
    pthread_create (&state->animate_thread, NULL, animate_thread, state);
    pthread_create (&state->status_thread, NULL, status_loop, state);

    // This is the main loop
    eecs467_gui_run (&state->vxapp, state->pg, 768, 768);

    // Quit when GTK closes
    state->running = 0;
    pthread_join (state->animate_thread, NULL);
    pthread_join (state->status_thread, NULL);

    // Cleanup
    free (my_listener);
    state_destroy (state);
    vx_remote_display_source_destroy (cxn);
    vx_global_destroy ();
}
