#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

#include "common/timestamp.h"
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtyoes/maebot_motor_feedback_t.h"

#define CMD_PRD 50000 //us  -> 20Hz
#define MTR_SPDL 0.25f
#define MTR_SPDR 0.25f
#define MTR_STOP 0.0f

//A0: drive maebot in a 2 x 3 square

maebot_motor_command_t msg;
pthread_mutex_t msg_mutex;
lcm_t *lcm;
pthreaad_t motor_feedback_thread;
pthread_t drive_square_thread;

char Move = 's'; // controls case state machine, s-stop, t-turn, f-forward
double Bot_Dist_Tot = 0.0; //total distance bot has traveled, m
double Bot_Dist_Step = 0.0;  //total distance of current movement, m
double Bot_Dist_Edge = 0.0; //total distance traveled alnog one edge
double Bot_Rad = 0.0; //turn in radians
char Square = 'L'; //'L' for long side, 'S' for short side
bool Turn = false;
int eCount = 0; //edge count, 12 edges = 3 rotations around square

static void
motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel,
						const maebot_motor_feedback_t *msg, void *user) 
{	
	int res = system ("clear");
	if (res)
		printf ("system clear failed\n");

	//print info to screen
	printf ("utime: %"PRId64"\n", msg->utime);
	printf ("encoder_[left, right]_ticks:\t\t%d,\t%d\n",
	msg->encoder_left_ticks, msg->encoder_right_ticks);
	printf ("motor_current[left, right]:\t\t%d,\t%d\n",
	msg->motor_current_left, msg->motor_current_right);
	printf ("motor_[left, right]_commanded_speed:\t%f,\t%f\n",
	msg->motor_left_commanded_speed, msg->motor_right_commanded_speed);
	printf ("motor_[left, right]_actual_speed:\t%f,\t%f\n",
	msg->motor_left_commanded_speed, msg->motor_right_commanded_speed);
	
	//how many ticks translated to distance
	int ticks = (msg->motor_current_left + msf->motor_current_right)/2;
	int dist_tmp = (ticks/480) * 0.032 * 2 * M_PI; //tot dist so far
	Bot_Dist_Step = dist_tmp - Bot_Dist_Tot;
	Bot_Dist_Tot = dist_tmp; //update total distance 
	
	//making the turn
	if(Turn){
		Move = 't';
		//have to finish doing the turn calculation and equations
			
	}
	
	//traveling an edge of square update
	else { 
		Move = 'f';	
			
		Bot_Dist_Edge = Bot_Dist_Edge + Bot_Dist_Step;
	
		//have we traveled a complete edge
		if( (Bot_Dist_Edge >= 0.9096) && (Square == 'S')) //short edge, 2ft
			{ Bot_Dist_Edge = 0.0; Turn = true; Square = 'L'; eCount++; }
		else if( (Bot_Dist_Edge >= 0.9144) && (Square == 'L') ) //long edge, 3ft
			{ Bot_Dist_Edge = 0.0; Turn = true; Square = 'S'; eCount++; }
		else Turn = false;
	}

	//3 loops, stop maebot
	if(eCount >= 12) Move = 's';
		
}
void * motor_feedback_loop(void *arg)
{
	maebot_motor_feedback_t_subscribe(lcm, "MAEBOT_MOTOR_FEEDBACK", 
    								  motor_feedback_handler, NULL);
	
	const int hz = 15;
	while(1) {
		int status = lcm_handle_timeout (lcm, 1000/hz);
		if(status <= 0)
			continue;
	}								  
	
	return NULL;
}

void * drive_square_thread (void *arg)
{
	uint64_t utime_start;

	while(1) {
		utime_start = utime_now ();

		//send movment commands to maebot
		switch(Move) { 	
			case f: {
    			// forward
    			pthread_mutex_lock (&msg_mutex);
    			msg.motor_left_speed  = MTR_SPDL;
    			msg.motor_right_speed = MTR_SPDR;
    			pthread_mutex_unlock (&msg_mutex);
    		
    			usleep (500000);
				break;
			}
    
			case t: {
    			// left turn
    			pthread_mutex_lock (&msg_mutex);
    			msg.motor_left_speed  = -MTR_SPD;
    			msg.motor_right_speed = MTR_SPD;
    			pthread_mutex_unlock (&msg_mutex);
    		
				usleep (500000);
				break;
			}
	
			case s: {
    			// stop
    			pthread_mutex_lock (&msg_mutex);
    			msg.motor_left_speed  = MTR_STOP;
    			msg.motor_right_speed = MTR_STOP;
    			pthread_mutex_unlock (&msg_mutex);
    		
				usleep (100000);
				break;
			}
		}		

		pthread_mutex_lock (&msg_mutex);
		maebot_motor_command_t_publish (lcm, "MAEBOT_MOTOR_COMMAND", &msg);
		pthread_mutex_unlock (&msg_mutex);

		usleep (CMD_PRD - (utime_now() - utime_start));
	}

	return NULL;
}

int
main (int argc, char *argv[])
{

    if (pthread_mutex_init (&msg_mutex, NULL)) {
        printf ("mutex init failed\n");
        return 1;
    }
    
    printf ("utime,\t\tleft_ticks,\tright_ticks\n");

    //Init msg, no need for mutex here, as command thread hasn't started yet.
    msg.motor_left_speed = MTR_STOP;
    msg.motor_right_speed = MTR_STOP;
    
    lcm = lcm_create(NULL);

    pthread_create (motor_feeback_thread, NULL, motor_feedback_loop, NULL);
    pthread_create (drive_square_thread, NULL, drive_square_loop, NULL);
    
    pthread_join(motor_feeback_thread, NULL);
    pthread_join(command_thread, NULL);
    
	lcm_destroy(lcm);

    return 0;
}
