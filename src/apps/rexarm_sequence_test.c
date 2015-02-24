#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <sys/select.h>
#include <sys/time.h>
#include <pthread.h>

#include <lcm/lcm.h>
#include "lcmtypes/dynamixel_command_list_t.h"
#include "lcmtypes/dynamixel_command_t.h"
#include "lcmtypes/dynamixel_status_list_t.h"
#include "lcmtypes/dynamixel_status_t.h"

#include "common/getopt.h"
#include "common/timestamp.h"
#include "math/math_util.h"

#define NUM_SERVOS 6

typedef struct state state_t;
struct state
{
    getopt_t *gopt;

    // LCM
    lcm_t *lcm;
    const char *command_channel;

    pthread_t command_thread;
};

void *
command_loop (void *user)
{
    state_t *state = user;
    const int hz = 0.5;

    dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = calloc (NUM_SERVOS, sizeof(dynamixel_command_t));

    int moving_id = 0;
    while (1) {
        // Send LCM commands to arm.
        for (int id = 0; id < NUM_SERVOS; id++) {
            cmds.commands[id].utime = utime_now ();
            cmds.commands[id].position_radians = (moving_id == id) ? 0.5 : 0.0;
            cmds.commands[id].speed = 0.1;
            cmds.commands[id].max_torque = 0.5;
        }
        dynamixel_command_list_t_publish (state->lcm, state->command_channel, &cmds);
        
        moving_id = ((moving_id == 5) ? 0 : moving_id + 1);

        usleep (2000000);
    }

    free (cmds.commands);

    return NULL;
}

int
main (int argc, char *argv[])
{
    getopt_t *gopt = getopt_create ();
    getopt_add_bool (gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_string (gopt, '\0', "command-channel", "ARM_COMMAND", "LCM command channel");

    if (!getopt_parse (gopt, argc, argv, 1) || getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt);
        exit (EXIT_FAILURE);
    }

    state_t *state = calloc (1, sizeof(*state));
    state->gopt = gopt;
    state->lcm = lcm_create (NULL);
    state->command_channel = getopt_get_string (gopt, "command-channel");

    pthread_create (&state->command_thread, NULL, command_loop, state);
    pthread_join (state->command_thread, NULL);

    lcm_destroy (state->lcm);
    free (state);
    getopt_destroy (gopt);
}
