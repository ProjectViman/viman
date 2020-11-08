#ifndef VM_Z_LINEAR_NAV
#define VM_Z_LINEAR_NAV

// Libraries: Viman
#include "viman.h"
#include "vm_pid_linear.h"
#include "vm_pid_rotate.h"
#include "imumath_brief.h"

// Libraries: Sensor data
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include "viman_utility/CamZ.h"

// Libraries: Mapping
#include "Map.h"

// Libraries: User control
#include <termios.h>
#include <thread>

// Uncomment this if you want to use angle range (-180,180]
#define USE_NEG_ANGLE

// Map properties
// Map m(mapLoc, lndmrkNodeName)
Map m("/home/mate/viman_ws/src/viman/viman_control/maps/senseMap", "LNDMRK");

// Mapping constants

// SET_POINT_RANGE: new set point is created if current value is in set_point +- range
#define SET_POINT_RANGE 0.05

// SET_POINT_STEP: increase in set point to traverse the environment.
#define SET_POINT_STEP 0.1

/* DO NOT MODIFY BEYOND THIS LINE */

// Store setpoints
double set_points[3];				// x,y,z
Cardan set_orient;					// set orientation

// Store velocity commands
float cmd_values[4];				// x,y,z,yaw

// Sensing and related variables
float position[3];					// x,y,z
Cardan cur_orient;					// current orientation
std::string cur_color;				// current detected color
std::string prev_color;				// previously mapped color

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

// Mapping vars
bool isMapping;

// Functions for mapping the environment
void addDataPoint();
void setNextSetPoint();
void showStoredMap();
void saveStoredMap();

#endif // VM_Z_LINEAR_NAV
