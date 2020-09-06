#include "viman_key_ctrl.h"

// Viman control functions

void toggle_takeOff(void){
	if(isFlying){
		pubLand.publish(std_msgs::Empty());
		ROS_INFO("Landing...");
    
		isFlying = false;
	}
	else{
		pubTakeOff.publish(std_msgs::Empty());
		ROS_INFO("Taking Off...");
    
		isFlying = true;
	}
}

void hover(void){
    if(!isFlying)
        return;
    
    twist_msg.linear.x=0;
    twist_msg.linear.y=0;
    twist_msg.linear.z=0;
    twist_msg.angular.x=0.0;
    twist_msg.angular.y=0.0;
    twist_msg.angular.z= 0.0;
    
    pubCmd.publish(twist_msg);
    ROS_INFO("Hovering...");
}

void move(float speed_x, float speed_y, float speed_z){
    if (!isFlying)
		return;
    
    twist_msg.linear.x = speed_x;
    twist_msg.linear.y = speed_y;
    twist_msg.linear.z = speed_z;
    twist_msg.angular.x = sgn(speed_y)*0.05;
    twist_msg.angular.y = sgn(speed_x)*0.05;
    twist_msg.angular.z = 0.0;
    pubCmd.publish(twist_msg);
}

void yaw(float speed){
	if (!isFlying)
		return;
    
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = speed;
    pubCmd.publish(twist_msg);
}

int sgn(float x){
	if (x > 0) return 1;
	if (x < 0) return -1;
	return 0;
}

/* Receiving set point from the user */
void initTermios(void){
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  current = old; /* make new settings same as old settings */
  current.c_lflag &= ~ICANON; /* disable buffered i/o */
  current.c_lflag &= ~ECHO; /* set no echo mode */
  tcsetattr(0, TCSANOW, &current); /* use these new terminal i/o settings now */
}
void resetTermios(void){
  tcsetattr(0, TCSANOW, &old);
}
char getch(void){
  char ch;
  initTermios();
  ch = getchar();
  resetTermios();
  return ch;
}

void read_input(void){
	char input;
	
	while(do_not_quit){
		input = getch();
		
		switch(input){
			case 'x': do_not_quit = false;
					  break;
			case 'h': display_help();
					  break;
			
			// Take off and Land
			case 't': toggle_takeOff();
					  break;
			
			// Hover
			case 's': hover();
					  for(int i=0;i<9;i++) cur_values[i]=0;
					  break;
					  
			// Up and down
			case 'q': if(cur_values[0]<0) cur_values[0]=0;
					  cur_values[0] += 0.05;
					  if(cur_values[0] > MAX_LINEAR_Z) cur_values[0]=MAX_LINEAR_Z;
					  move(0,0,cur_values[0]);
					  break;
			case 'w': if(cur_values[0]>0) cur_values[0]=0;
					  cur_values[0] -= 0.05;
					  if(cur_values[0] < -MAX_LINEAR_Z) cur_values[0]=-MAX_LINEAR_Z;
					  move(0,0,cur_values[0]);
					  break;
			
			// Left and Right
			case 'j': if(cur_values[1]<0) cur_values[1]=0;
					  cur_values[1] += 0.05;
					  if(cur_values[1] > MAX_LINEAR_X) cur_values[1]=MAX_LINEAR_X;
					  move(cur_values[1],0,0);
					  break;
			case 'l': if(cur_values[0]>0) cur_values[1]=0;
					  cur_values[1] -= 0.05;
					  if(cur_values[1] < -MAX_LINEAR_X) cur_values[1]=-MAX_LINEAR_X;
					  move(cur_values[1],0,0);
					  break;
			
			// Fwd and Bck
			case 'k': if(cur_values[2]<0) cur_values[2]=0;
					  cur_values[2] += 0.05;
					  if(cur_values[2] > MAX_LINEAR_Y) cur_values[2]=MAX_LINEAR_Y;
					  move(0,cur_values[2],0);
					  break;
			case 'i': if(cur_values[2]>0) cur_values[2]=0;
					  cur_values[2] -= 0.05;
					  if(cur_values[2] < -MAX_LINEAR_Y) cur_values[2]=-MAX_LINEAR_Y;
					  move(0,cur_values[2],0);
					  break;
			
			// Yaw
			case 'u': if(cur_values[3]<0) cur_values[3]=0;
					  cur_values[3] += 0.05;
					  if(cur_values[3] > MAX_YAW) cur_values[3]=MAX_YAW;
					  yaw(cur_values[3]);
					  break;
			case 'o': if(cur_values[3]>0) cur_values[3]=0;
					  cur_values[3] -= 0.05;
					  if(cur_values[3] < -MAX_YAW) cur_values[3]=-MAX_YAW;
					  yaw(cur_values[3]);
					  break;
			
		}		
	}
}

void display_help(void){
	std::cout << "\nUse the following commands: "<< std::endl
			<< "t: Take off/Land (cmds work iff VIMAN has taken off)" << std::endl
			<< "q: Up" << std::endl
			<< "w: Down" << std::endl
			<< "i: Forward" << std::endl
			<< "k: Backward" << std::endl
			<< "j: Left" << std::endl
			<< "l: Right" << std::endl
			<< "u: Yaw+" << std::endl
			<< "o: Yaw-" << std::endl
			<< "y: Roll+"<<std::endl 
			<< "p: Roll-"<<std::endl
			<< "n: Pitch+"<<std::endl
			<< "m: Pitch-"<<std::endl
			<< "s: Hover"<< std::endl
			<< "h: Display help" << std::endl
			<< "x: Quit\n" << std::endl;
}

int main(int argc, char **argv){
	
	ros::init(argc,argv,"viman_key_ctrl");
	ros::NodeHandle node;
	ros::Rate rate(2);
	
	isFlying = false;

    pubTakeOff = node.advertise<std_msgs::Empty>("/viman/takeoff",1024);
    pubLand = node.advertise<std_msgs::Empty>("/viman/land",1024);
    pubReset = node.advertise<std_msgs::Empty>("/viman/reset",1024);
    pubCmd = node.advertise<geometry_msgs::Twist>("/cmd_vel",1024); 
	
	display_help();
	
	// begin separate thread to read from the keyboard		
	std::thread reading_input_thread(read_input);
	reading_input_thread.detach();

	while(do_not_quit && ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
}
