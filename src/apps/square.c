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

// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"
#include "common/timestamp.h"

// imagesource
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"
#include "eecs467_util.h" // This is where a lot of the internals livemovement

//camera
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

#include "imagesource/image_u8x3.h"
//#include "image_source.h"
//#include "image_convert.h"
#include "imagesource/pnm.h"

#define MAX_REVERSE_SPEED -1.0f
#define MAX_FORWARD_SPEED 1.0f
#define CMD_PRD 50000 //us -> 20Hz
#define MTR_SPD 0.25f
#define MTR_STOP 0.0f


typedef struct state state_t;
struct state {

	getopt_t *gopt;

	// LCM
	lcm_t *lcm;
	const char *laser_channel;
	const char *corner_laser_channel;

	//camera
	char *url; //image_source url
	image_source_t *isrc;
	int fidx;

	// threads
	pthread_t lcm_thread;
	pthread_t drive_square_thread;

	pthread_mutex_t laser_mutex;
	pthread_mutex_t move_mutex;

	//logic
	char move;
	int eCount; //edge count, 12 edges = 3 rotations around square
	bool need_scan; 
};

	static void 
change_need_scan(bool value, state_t *state)
{
	pthread_mutex_lock(&(state->laser_mutex));
	state->need_scan = value;
	pthread_mutex_unlock(&(state->laser_mutex));
}

static void take_picture(state_t *state)
{
	printf("taking picture ... ");

	// take picture code (copy-past, don't know how it works)
	image_source_data_t *frmd;
	frmd = malloc(sizeof(image_source_data_t));
	image_u32_t * save_image = (image_u32_t*)state->isrc->get_frame(state->isrc, frmd);// might need to change casting
	save_image = image_convert_u32(frmd);

	char path[100];
	snprintf(path, 100, "/home/team2/eecs467/corner_shot_%d.ppm", state->eCount);


	image_u32_write_pnm(save_image, path); //this path can be changed
	state->isrc->release_frame(state->isrc,frmd);
	printf("image saved\n");
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
{}

void * drive_square_loop(void *data){
	state_t *state = data;

	int res = system ("clear");
	if (res)
		printf ("system clear failed\n");

	char cur_move = 'f';

	int i;
	int eCount = 0;
	maebot_motor_command_t msg;
	while(eCount < 12){
		if(cur_move == 'f') {
			msg.motor_left_speed = MTR_SPD;
			msg.motor_right_speed = MTR_SPD;
			for (i = 0; i < 15; i++){ // for 15/5 dec
				msg.utime = utime_now ();
				maebot_motor_command_t_publish (state->lcm, "MAEBOT_MOTOR_COMMAND", &msg);
				usleep(200000);
			}
			cur_move = 't';
			usleep(250000);
			take_picture(state);
			change_need_scan(true, state);
			usleep(250000);
			eCount++;
		} else {//if cur_move == 't'
			msg.motor_left_speed = -1.0 * MTR_SPD;
			msg.motor_right_speed = MTR_SPD;
			for (i = 0; i < 5; i++){ // for 5/5 sec
				msg.utime = utime_now ();
				maebot_motor_command_t_publish (state->lcm, "MAEBOT_MOTOR_COMMAND", &msg);
				usleep(200000);
			}
			cur_move = 'f';
		}

		//print debug/info to screen
		printf ("utime: %"PRId64"\n", msg.utime);    
		printf ("motor_[left, right]_commanded_speed:\t%f,\t%f\n",
				msg.motor_left_speed, msg.motor_right_speed);
		printf ("motor_[left, right]_actual_speed:\t%f,\t%f\n",
				msg.motor_left_speed, msg.motor_right_speed);
		printf("state:\t\t%c\n", cur_move);
		printf("Edges:\t\t%d\n", eCount);
	}

	msg.motor_left_speed = MTR_STOP;
	msg.motor_right_speed = MTR_STOP;
	msg.utime = utime_now ();
	maebot_motor_command_t_publish (state->lcm, "MAEBOT_MOTOR_COMMAND", &msg);

	return NULL;
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
	state->eCount = 0;
	state->move = 'f';
	state->need_scan = false;

	return state;
}

void state_destroy (state_t *state)
{
	if (!state)
		return;

	if (state->lcm)
		lcm_destroy(state->lcm);

	if (state->gopt)
		getopt_destroy (state->gopt);

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

	const zarray_t *args = getopt_get_extra_args(state->gopt);
	if (zarray_size(args) > 0) {
		zarray_get(args, 0, &state->url);
	} else {
		zarray_t *urls = image_source_enumerate();

		printf("Cameras:\n");
		for (int i = 0; i < zarray_size(urls); i++) {
			char *url;
			zarray_get(urls, i, &url);
			printf("  %3d: %s\n", i, url);
		}

		if (zarray_size(urls) == 0) {
			printf("No cameras found.\n");
			exit(0);
		}
		zarray_get(urls, 0, &state->url);
	}

	state->isrc = image_source_open(state->url);
	if (state->isrc == NULL) {
		printf("Unable to open device %s\n", state->url);
		exit(-1);
	}

	image_source_t *isrc = state->isrc;

	if (isrc->start(isrc))
		exit(-1);

	state->fidx = isrc->get_current_format(isrc);

	change_need_scan(false, state);

	pthread_create (&state->lcm_thread, NULL, receive_lcm_msg, state);
	pthread_create (&state->drive_square_thread, NULL, drive_square_loop, state);


	//gtk_main();

	pthread_join (state->lcm_thread, NULL);
	pthread_join (state->drive_square_thread, NULL);

	state_destroy (state);

	return 0;
}

