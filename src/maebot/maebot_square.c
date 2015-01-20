#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

#include "common/timestamp.h"
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"

#define CMD_PRD 50000 //us  -> 20Hz
#define MTR_SPD 0.25f
#define MTR_STOP 0.0f

typedef struct state state_t;
struct state
{

	lcm_t *lcm;
	pthread_t motor_feedback_thread;
	pthread_t drive_square_thread;

};

//typedef struct BotState BotState_t;
struct BotState
{

	double right_prev_dist;
	double left_prev_dist;

	double theta_0;
	double x_0;
	double y_0; 

	double dist_edge; //total distance traveled alnog one edge
	
	int turn; //bool
	int short_edge; //bool
	int eCount; //edge count, 12 edges = 3 rotations around square

	double stop_turn_theta; //during turn compare to this
};

//initlize bot state
struct BotState Bot;
char Move = 's';
pthread_mutex_t move_mutex;

static void
motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel,
						const maebot_motor_feedback_t *msg, void *user) 
{	
	int res = system ("clear");
	if (res)
		printf ("system clear failed\n");
	
	char cur_move = 's';

	//update the distance traveled since last timestep
	double cur_dist = (msg->encoder_right_ticks/480.0) * 0.032 * 3.14; 
	double right_step = cur_dist - Bot.right_prev_dist;
	printf("cur_dist: prev_dist: right_step:\t%f\t%f\t%f\n", cur_dist, Bot.right_prev_dist, right_step);
	Bot.right_prev_dist = cur_dist;

	cur_dist = (msg->encoder_left_ticks/480.0) * 0.032 *3.14; 
	double left_step = cur_dist - Bot.left_prev_dist;
	printf("cur_dist: prev_dist: left_step:\t%f\t%f\t%f\n", cur_dist, Bot.left_prev_dist, left_step);	
	Bot.left_prev_dist = cur_dist; 

	//odometry
	double s_ = (right_step + left_step)/2;
	double delta_theta = (right_step - left_step)/0.04;	
	double alpha = delta_theta/2;
	double x = cos(Bot.theta_0 + alpha) * s_ + Bot.x_0; //tot dist traveled
	double y = sin(Bot.theta_0 + alpha) * s_ + Bot.y_0; //tot dist traveled
	double theta = delta_theta + Bot.theta_0;
	
	if(Bot.turn){ //making the turn
		
		if(theta >= Bot.stop_turn_theta) //stop turining 
			{ Bot.turn = 0; cur_move = 'f'; }
		else //keep turning
			{ Bot.turn = 1; cur_move = 't'; }
	}

	else { //traveling forward along an edge	
			
		double ave_step = (right_step + left_step)/2; //ave dist of motors
		Bot.dist_edge = Bot.dist_edge + ave_step;
				
		printf("ave step:\t%f", ave_step);

		if( (Bot.dist_edge >= 0.6096) && Bot.short_edge) //short edge, 2ft
		{ 
			Bot.dist_edge = 0.0; Bot.turn = 1; 
			Bot.short_edge = 0; Bot.eCount++; 
			Bot.stop_turn_theta = theta + 1.0; cur_move = 't';
		}
		else if( Bot.dist_edge >= 0.9144 ) //long edge, 3ft
		{ 
			Bot.dist_edge = 0.0; Bot.turn = 1; 
			Bot.short_edge = 1; Bot.eCount++; 
			Bot.stop_turn_theta = theta + 1.0; cur_move = 't';
		}
		else //continue along the edge
		{
			Bot.turn = 0;
			cur_move = 'f';
		}
	}

	//update Bot position
	Bot.theta_0 = theta;
	Bot.x_0 = x;
	Bot.y_0 = y;

	//3 loops, stop maebot
	if(Bot.eCount >= 12) cur_move = 's';

	pthread_mutex_lock(&move_mutex);
	Move = cur_move;
	pthread_mutex_unlock(&move_mutex);

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
	printf("theta, x, y\t(%f,%f,%f)\n", Bot.theta_0, Bot.x_0, Bot.y_0);

	printf("state_machine:\t\t%c\n", cur_move);
	printf("distance_along_edge:\t\t%f\n", Bot.dist_edge);
	printf("Edges:\t\t%d\n", Bot.eCount);	
}

void * motor_feedback_loop(void *arg)
{
	state_t *state = arg;
	maebot_motor_feedback_t_subscribe(state->lcm, "MAEBOT_MOTOR_FEEDBACK", 
    								  motor_feedback_handler, NULL);
	
	const int hz = 15;
	while(1) {
		int status = lcm_handle_timeout (state->lcm, 1000/hz);
		if(status <= 0)
			continue;
	}								  
	
	return NULL;
}

void * drive_square_loop (void *arg)
{
	state_t *state = arg;
	uint64_t utime_start;

	while(1) {
		utime_start = utime_now ();
		maebot_motor_command_t msg;

		pthread_mutex_lock(&move_mutex);
		char cur_move = Move;
		pthread_mutex_unlock(&move_mutex);

		//send movment commands to maebot
		switch(cur_move) { 	
			case 'f': //forward
			{
    			msg.motor_left_speed  = MTR_SPD;
    			msg.motor_right_speed = MTR_SPD;
				break;
			}
    
			case 't': // left turn
			{
    			msg.motor_left_speed  = MTR_STOP;
    			msg.motor_right_speed = MTR_SPD;
				break;
			}
	
			case 's': //stop
			{
    			msg.motor_left_speed  = MTR_STOP;
    			msg.motor_right_speed = MTR_STOP;
				break;
			}
		}		

		maebot_motor_command_t_publish (state->lcm, "MAEBOT_MOTOR_COMMAND", &msg);

		usleep (CMD_PRD - (utime_now() - utime_start));
	}

	return NULL;
}

int
main (int argc, char *argv[])
{
	//A0: drive maebot in a 2 x 3 square

    if (pthread_mutex_init (&move_mutex, NULL)) {
        printf ("mutex init failed\n");
        return 1;
    }
    
    printf ("utime,\t\tleft_ticks,\tright_ticks\n");

	//initilize bot
	Bot.right_prev_dist = 0.0;
	Bot.left_prev_dist = 0.0;
	Bot.theta_0 = 0.0;
	Bot.x_0 = 0.0;
	Bot.y_0 = 0.0;
	Bot.dist_edge = 0.0;
	Bot.turn = 0; //false
	Bot.short_edge = 0; //false
	Bot.eCount = 0;
	Bot.stop_turn_theta = 0.0;

    //Init msg, no need for mutex here, as command thread hasn't started yet.
    //msg.motor_left_speed = MTR_STOP;
    //msg.motor_right_speed = MTR_STOP;

	state_t *state = calloc (1, sizeof(*state));
    
    state->lcm = lcm_create(NULL);

    pthread_create (&state->motor_feedback_thread, NULL, motor_feedback_loop, state);
    pthread_create (&state->drive_square_thread, NULL, drive_square_loop, state);
    
    pthread_join(state->motor_feedback_thread, NULL);
    pthread_join(state->drive_square_thread, NULL);
    
	lcm_destroy(state->lcm);
	free (state);

    return 0;
}
