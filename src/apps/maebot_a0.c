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
    //getopt_t *gopt;
    //parameter_gui_t *pg;

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
    //pthread_t laser_thread;

    // for accessing the arrays
    pthread_mutex_t mutex_laser;
    maebot_laser_scan_t *laser_info;
};

// save laser information
static void
laser_handler (const lcm_recv_buf_t *rbuf,
                const char *channel,
                const maebot_laser_scan_t *msg,
                void *state_p)
{
    state_t *state = state_p;
    state->laser_info = msg;
}

// do a scan
void *
laser_scan (void *data)
{
    state_t *state = data;
    maebot_laser_scan_t_subscribe (state->lcm,
                                   state->laser_channel,
                                   laser_handler,
                                   state);
    int status = 0;
    while (status <= 0) {
        status = lcm_handle_timeout (state->lcm, 1000/hz);
    }
    return NULL;
}

void *
animate_thread (void *data)
{
    const int fps = 60;
    state_t *state = data;

    pthread_mutex_lock(&state->mutex_laser);
    for (int ){

    }


}



int main()
{
	// just a signal!
	printf("Program running ...\n\n");

	
	// LCM protocol (like udp and tcp --sorta)
	lcm_t *lcm = lcm_create (NULL);
   	maebot_leds_command_t msg;

   	msg.bottom_led_left = 0;
   	msg.bottom_led_middle = 0;
   	msg.bottom_led_right = 0;
   	msg.line_sensor_leds = 1;

	// green top  and bottom 2 leds off
    	msg.top_rgb_led_left = 0x1000;
    	msg.top_rgb_led_right = 0x1000;
    	msg.bottom_led_left = 0;
	msg.bottom_led_right = 0;	

	// publish the commands
	maebot_leds_command_t_publish (lcm, "MAEBOT_LEDS_COMMAND", &msg);
	printf("change led colors ... \n");
	printf("wait ...\n\n"	);

	usleep(1000000);
	
	// change top and bottom led colors
	msg.bottom_led_left = 1;
	msg.bottom_led_right = 1;
	msg.bottom_led_middle = 1;
	
	msg.top_rgb_led_left = 0x100000;
	msg.top_rgb_led_right = 0x100000;
	
	maebot_leds_command_t_publish(lcm, "MAEBOT_LEDS_COMMAND", &msg);
	printf("Change led colors ...\n");

	// motor commands
	maebot_motor_command_t command;
	
	int i = 0;
	int j = 0;
	for (i = 0; i < 4; i++)
	{
		
		// move forward
		for (j = 0; j < 10; j++)
		{
			command.motor_right_speed = MAX_FORWARD_SPEED;
			command.motor_left_speed = MAX_FORWARD_SPEED;

			// publish commands
			maebot_motor_command_t_publish(lcm, "MAEBOT_MOTOR_COMMAND", &command);
		}
		usleep(3000000);//3 seconds forward (I think)
	
		// stop
		//command.motor_right_speed = 0.0f;
		//command.motor_left_speed = 0.0f;
	
		// publish motor commands
		//maebot_motor_command_t_publish(lcm, "MAEBOT_MOTOR_COMMAND", &command);
	
	
		// turn right
		command.motor_right_speed = 0.0f;
		command.motor_left_speed = MAX_FORWARD_SPEED/4;
	
		// publish command
		maebot_motor_command_t_publish(lcm, "MAEBOT_MOTOR_COMMAND", &command);
	

		// wait for the turn to happen
		usleep(100000); // 1 sec? maybe
		
	}


	// loop forever
	//int x = 0;
	//while(1){
	//	x++;
	//}
	
	return 0;
}
