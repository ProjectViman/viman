#ifndef VM_Z_LINEAR_NAV
#define VM_Z_LINEAR_NAV

#include "viman.h"
#include "vm_pid_linear.h"
#include "vm_pid_rotate.h"
#include "imumath_brief.h"

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include "viman_utility/CamZ.h"

// Libraries to read from keyboard
#include <termios.h>
#include <thread>

// Uncomment this if you want to use angle range (-180,180]
#define USE_NEG_ANGLE

// Store setpoints
double set_points[3];				// x,y,z
Cardan set_orient;					// set orientation

// Store velocity commands
float cmd_values[4];				// x,y,z,yaw

// Sensing and related variables
float position[3];					// x,y,z
Cardan cur_orient;					// current orientation 

// VIMAN instance
Viman vm;

// Controller and related
VmPidLinear height_controller_;
VmPidRotate yaw_controller_;
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
void ImuCallbck(const sensor_msgs::Imu&);
void CamCallbck(const viman_utility::CamZ&);

/**********************************************************************/
bool isMapping = false;

#endif // VM_Z_LINEAR_NAV
