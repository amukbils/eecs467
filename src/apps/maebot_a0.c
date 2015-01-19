
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm.h>
#include <signal.h>

#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_targeting_laser_command_t.h"
#include "lcmtypes/maebot_leds_command_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"


#define MAX_REVERSE_SPEED -1.0f
#define MAX_FORWARD_SPEED 1.0f

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
