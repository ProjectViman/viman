#ifndef VIMAN_KEY_CTRL_H
#define VIMAN_KEY_CTRL_H

// ROS Libraries
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

// Libraries to read from keyboard
#include <termios.h>
#include <thread>

// Maximum velocities in m/s^2 and rad/s
#define MAX_LINEAR_X 2
#define MAX_LINEAR_Y 2
#define MAX_LINEAR_Z 2
#define MAX_YAW 2.5

// Vars to control and read viman state
bool isFlying;
geometry_msgs::Twist twist_msg;
float cur_values[9];

ros::Publisher pubTakeOff;
ros::Publisher pubLand;
ros::Publisher pubReset;
ros::Publisher pubCmd;

// Vars to read user cmds
static struct termios old, current;
bool do_not_quit = true;

// Functions to control viman state
void toggle_takeOff(void);
void hover(void);
void move(float, float, float);

// Functions to read user input
void display_help(void);
void initTermios(void);
void resetTermios(void);
char getch(void);
void read_input(void);

// Utility functions
int sgn(float);

#endif // VIMAN_KEY_CTRL_H
