#ifndef VIMAN_SA
#define VIMAN_SA

#include "viman.h"
#include "vm_pid.h"

#include <geometry_msgs/PointStamped.h>

// Libraries to read from keyboard
#include <termios.h>
#include <thread>

// Maximum velocities in m/s^2 and rad/s
#define MAX_LINEAR_X 2
#define MAX_LINEAR_Y 2
#define MAX_LINEAR_Z 2
#define MAX_YAW 2.5

// Store setpoints
float set_points[4];				// z, x, y, yaw

// Store velocity commands
float cmd_values[4];

// Sensing and related variables
float sensor_values[4];

// VIMAN instance
Viman vm;

// Controller and related
VmPID height_controller_;
bool isPidRunning;

// Vars to read user cmds
static struct termios old, current;
bool do_not_quit = true;

// Functions to read user input
void display_help(void);
void initTermios(void);
void resetTermios(void);
char getch(void);
void read_input(void);

// Functions to assist semi-autonomous movement
void HeightCallbck(const geometry_msgs::PointStamped&);

#endif // VIMAN_SA
