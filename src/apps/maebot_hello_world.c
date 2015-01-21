#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <inttypes.h>

//lcm
#include <lcm/lcm.h>
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_targeting_laser_command_t.h"
#include "lcmtypes/maebot_leds_command_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_laser_scan_t.h"
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"

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
#include "common/timestamp.h"

// imagesource
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"
#include "eecs467_util.h" // This is where a lot of the internals live

#define MAX_REVERSE_SPEED -1.0f
#define MAX_FORWARD_SPEED 1.0f
#define CMD_PRD 50000 //us -> 20Hz
#define MTR_SPD 0.25f
#define MTR_STOP 0.0f





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
    pthread_t lcm_thread;
    pthread_t drive_square_thread;

    pthread_mutex_t laser_mutex;
    pthread_mutex_t move_mutex;
//    maebot_laser_scan_t *laser_info;
    bool need_scan;
    bool need_init;

    // bot info
    double right_prev_dist;
    double left_prev_dist;

    double theta_0;
    double x_0;
    double y_0;

    double dist_edge; //total distance traveled alnog one edge

    int turn; //bool
    int short_edge; //bool
    int eCount; //edge count, 12 edges = 3 rotations around square
    char move;

    double stop_turn_theta; //during turn compare to this
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

static void
motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                        const maebot_motor_feedback_t *msg, void *user)
{
    state_t *state = user;

    int res = system ("clear");
    if (res)
        printf ("system clear failed\n");

    char cur_move = 's';

    if (state->need_init){
        state->right_prev_dist = (msg->encoder_right_ticks/480.0) * 0.032 * 3.14;
        state->left_prev_dist = (msg->encoder_left_ticks/480.0) * 0.032 * 3.14;
        state->need_init = false;
    }

    //update the distance traveled since last timestep
    double cur_dist = (msg->encoder_right_ticks/480.0) * 0.032 * 3.14;
    double right_step = cur_dist - state->right_prev_dist;
    printf("cur_dist: prev_dist: right_step:\t%f\t%f\t%f\n", cur_dist, state->right_prev_dist, right_step);
    state->right_prev_dist = cur_dist;

    cur_dist = (msg->encoder_left_ticks/480.0) * 0.032 *3.14;
    double left_step = cur_dist - state->left_prev_dist;
    printf("cur_dist: prev_dist: left_step:\t%f\t%f\t%f\n", cur_dist, state->left_prev_dist, left_step);
    state->left_prev_dist = cur_dist;

    //odometry
    double s_ = (right_step + left_step)/2;
    double delta_theta = (right_step - left_step)/0.08;
    double alpha = delta_theta/2;
    double x = cos(state->theta_0 + alpha) * s_ + state->x_0; //tot dist traveled
    double y = sin(state->theta_0 + alpha) * s_ + state->y_0; //tot dist traveled
    double theta = delta_theta + state->theta_0;

    if (state->turn){ //making the turn
        if(theta >= state->stop_turn_theta){ //stop turining
            state->turn = 0; 
            cur_move = 'f'; 
        } else { //keep turning
            state->turn = 1; 
            cur_move = 't'; 
        }
    } else { //traveling forward along an edge
        double ave_step = (right_step + left_step)/2; //ave dist of motors
        state->dist_edge = state->dist_edge + ave_step;

        printf("ave step:\t%f", ave_step);

        if( (state->dist_edge >= 0.6096) && state->short_edge) //short edge, 2ft
        {
            state->dist_edge = 0.0; 
            state->turn = 1;
            state->short_edge = 0; 
            state->eCount++;
            state->stop_turn_theta = theta + 1.57; 
            cur_move = 't';
        } else if (state->dist_edge >= 0.9144 ){ //long edge, 3ft
            state->dist_edge = 0.0;
            state->turn = 1;
            state->short_edge = 1;
            state->eCount++;
            state->stop_turn_theta = theta + 1.57; 
            cur_move = 't';
        } else { //continue along the edge
            state->turn = 0;
            cur_move = 'f';
        }
    }

    //update Bot position
    state->theta_0 = theta;
    state->x_0 = x;
    state->y_0 = y;

    //3 loops, stop maebot
    if(state->eCount >= 12) 
        cur_move = 's';

    pthread_mutex_lock(&state->move_mutex);
    state->move = cur_move;
    pthread_mutex_unlock(&state->move_mutex);

    //print debug/info to screen
    printf ("utime: %"PRId64"\n", msg->utime);
    printf ("encoder_[left, right]_ticks:\t\t%d,\t%d\n",
            msg->encoder_left_ticks, msg->encoder_right_ticks);
    printf ("motor_current[left, right]:\t\t%d,\t%d\n",
            msg->motor_current_left, msg->motor_current_right);
    printf ("motor_[left, right]_commanded_speed:\t%f,\t%f\n",
            msg->motor_left_commanded_speed, msg->motor_right_commanded_speed);
    printf ("motor_[left, right]_actual_speed:\t%f,\t%f\n",
            msg->motor_left_commanded_speed, msg->motor_right_commanded_speed);

    printf("right step, left step\t\t%f\t%f\n", right_step, left_step);
    printf("theta, x, y\t(%f,%f,%f)\n", state->theta_0, state->x_0, state->y_0);

    printf("state_machine:\t\t%c\n", cur_move);
    printf("distance_along_edge:\t\t%f\n", state->dist_edge);
    printf("Edges:\t\t%d\n", state->eCount);
}

ensor_data_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                     const maebot_sensor_data_t *msg, void *user)
{
    int res = system ("clear");
    if (res)
        printf ("system clear failed\n");

    printf ("Subscribed to channel: MAEBOT_SENSOR_DATA\n");
    printf ("utime: %"PRId64"\n", msg->utime);
    printf ("accel[0, 1, 2]: %d,\t%d,\t%d\n",
            msg->accel[0], msg->accel[1], msg->accel[2]);
    printf ("gyro[0, 1, 2]: %d,\t%d,\t%d\n\n\n",
            msg->gyro[0], msg->gyro[1], msg->gyro[2]);
    printf ("gyro_int[0, 1, 2]: %"PRId64",\t%"PRId64",\t%"PRId64"\n",
    msg->gyro_int[0], msg->gyro_int[1], msg->gyro_int[2]);
    printf ("line_sensors[0, 1, 2]: %d,\t%d,\t%d\n",
    msg->line_sensors[0], msg->line_sensors[1], msg->line_sensors[2]);
    printf ("range: %d\n", msg->range);
    printf ("user_button_pressed: %s\n", msg->user_button_pressed ? "true" : "false");
    printf ("power_button_pressed: %s\n", msg->power_button_pressed ? "true" : "false");
}


void *
receive_lcm_msg (void *data)
{
    state_t *state = data;

    maebot_laser_scan_t_subscribe (state->lcm,
                                   state->laser_channel,
                                   laser_handler,
                                   state);
    maebot_motor_feedback_t_subscribe(state->lcm, "MAEBOT_MOTOR_FEEDBACK",
                                      motor_feedback_handler, state);

    while (1) 
        lcm_handle_timeout (state->lcm, 1000/5);

    return NULL;
}

void * drive_square_loop (void *arg) {
    state_t *state = arg;
    uint64_t utime_start;

    while(1) {
        utime_start = utime_now ();
        maebot_motor_command_t msg;

        pthread_mutex_lock(&state->move_mutex);
        char cur_move = state->move;
        pthread_mutex_unlock(&state->move_mutex);

        //send movment commands to maebot
        switch(cur_move) {
            case 'f': //forward
            {
                msg.motor_left_speed = MTR_SPD;
                msg.motor_right_speed = MTR_SPD;
                break;
            }
            case 't': // left turn
            {
                msg.motor_left_speed = MTR_SPD * -1;
                msg.motor_right_speed = MTR_SPD;
                break;
            }
            case 's': //stop
            {
                msg.motor_left_speed = MTR_STOP;
                msg.motor_right_speed = MTR_STOP;
                break;
            }
            default:
                printf("error: undefined move type\n");
        }

        maebot_motor_command_t_publish (state->lcm, "MAEBOT_MOTOR_COMMAND", &msg);

        usleep (CMD_PRD - (utime_now() - utime_start));
    }

    return NULL;
}

state_t *
state_create (void)
{
    state_t *state = calloc (1, sizeof(*state));

    if (pthread_mutex_init (&state->laser_mutex, NULL)) {
        printf ("mutex init failed\n");
        exit(1);
    }

    if (pthread_mutex_init (&state->move_mutex, NULL)) {
        printf ("mutex init failed\n");
        exit(1);
    }

    state->lcm = lcm_create (NULL);

    printf ("utime,\t\tleft_ticks,\tright_ticks\n");

    //initilize bot
    state->theta_0 = 0.0;
    state->x_0 = 0.0;
    state->y_0 = 0.0;
    state->dist_edge = 0.0;
    state->turn = 0; //false
    state->short_edge = 0; //false
    state->eCount = 0;
    state->stop_turn_theta = 0.0;
    state->move = 's';
    state->need_init = true;

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

    change_need_scan(false, state);

    pthread_create (&state->lcm_thread, NULL, receive_lcm_msg, state);
    pthread_create (&state->drive_square_thread, NULL, drive_square_loop, state);

    //eecs467_gui_run (&state->vxapp, state->pg, 768, 768);

    pthread_join (state->lcm_thread, NULL);
    pthread_join (state->drive_square_thread, NULL);

    //free (my_listener);
    state_destroy (state);
    //vx_remote_display_source_destroy (cxn);
    //vx_global_destroy ();

	return 0;
}

