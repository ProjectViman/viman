#include "vm_z_linear_nav.h"

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
			case 'z': display_help();
					  break;
			
			// Take off and Land
			case 't': vm.toggle_ready();
					  isPidRunning = true;
					  break; 
			
			// Stop
			case 'S': vm.allStop();
					  for(int i=0;i<3;i++){
						 set_points[i] = 0;
					  }
					  if(isMapping) isMapping = false;
					  ROS_INFO("Set height: %f",set_points[2]);
					  break;
					  
			// Height (z) +-
			case 'q': set_points[2] += 0.1;
					  ROS_INFO("Set height: %f",set_points[2]);
					  break;
			case 'w': set_points[2] -= 0.1;
					  if(set_points[2] < 0) set_points[2] = 0;
					  ROS_INFO("Set height: %f",set_points[2]);
					  break;
			
			// Setting height
			case 'h': std::cout << "\nEnter height to hover (m): ";
					  std::cin >> set_points[2];
					  break;
			
			// Set height PID gains
			case 'H': 
					  isPidRunning = false;
					  std::cout << "\nEnter height P gain: ";
					  std::cin >> height_controller_.gain_p;
					  std::cout << "Enter height I gain: ";
					  std::cin >> height_controller_.gain_i;
					  std::cout << "Enter height D gain: ";
					  std::cin >> height_controller_.gain_d;
					  height_controller_.reset();
					  isPidRunning = true;
					  break;
			
			// reset height PID
			case 'P': height_controller_.reset();
					  break;
			
			// start/stop mapping
			case 'm': if(isMapping){
						isMapping = false;
					  }
					  else{
						isMapping = true;
					  }
					  break;
		}		
	}
}

void display_help(void){
	std::cout << "\nUse the following commands: "<< std::endl
			<< "t: Take off/Land (cmds work iff VIMAN has taken off)" << std::endl
			<< "s: Start mapping" << std::endl
			<< "S: Stop and set height = 0 m" << std::endl
			<< "q: Increase height by 0.1 m" << std::endl
			<< "w: Decrease height by 0.1 m" << std::endl
			<< "H: Set height PID gains" << std::endl
			<< "P: Reset height PID" << std::endl
			<< "m: Start/Stop mapping" << std::endl
			<< "z: Display help" << std::endl
			<< "x: Quit\n" << std::endl;
}

int main(int argc, char **argv){
	
	ros::init(argc,argv,"viman_semiAuto");
	ros::NodeHandle node;
	
	ros::Subscriber height_subs_ = node.subscribe("/viman/height",500,HeightCallbck);
	ros::Subscriber imu_subs_ = node.subscribe("/viman/imu",500,ImuCallbck);
	ros::Subscriber cam_subs_ = node.subscribe("/viman/color_id",500,CamCallbck);
	
	vm = Viman(node);
	height_controller_ = VmPidLinear();
	yaw_controller_ = VmPidRotate();
	
	height_controller_.gain_p = 2;
	height_controller_.gain_i = 0.1;
	height_controller_.gain_d = 0.03;
	
	yaw_controller_.gain_p = 0.6;
	yaw_controller_.gain_i = 0.1;
	yaw_controller_.gain_d = 0.4;
	yaw_controller_.sp_range = 0.3;
	
	display_help();
	// check if topics are present
    if( height_subs_.getTopic() != "")
        ROS_INFO("found altimeter height topic");
    else
        ROS_INFO("cannot find altimeter height topic!");
    if( imu_subs_.getTopic() != "")
        ROS_INFO("found imu topic");
    else
        ROS_INFO("cannot find imu topic!");
        
    isPidRunning = false;
    
	//ros::Rate rate(2);
	
	// begin separate thread to read from the keyboard		
	std::thread reading_input_thread(read_input);
	reading_input_thread.detach();
	
	double prev_time = ros::Time::now().toSec();
	double cur_time;
	double dt;
		
	set_points[2] = 0;
	set_orient.yaw = 0;
		
	while(do_not_quit && ros::ok()){
		if(isPidRunning){
			cur_time = ros::Time::now().toSec();
			
			dt = cur_time - prev_time;
			
			if(dt <= 0 ) continue;
			
			cmd_values[2] = height_controller_.update(set_points[2], position[2], 
									  dt);
									  
			cmd_values[3] = yaw_controller_.update(set_orient.yaw, cur_orient.yaw, dt);
			
			vm.move(cmd_values[0],cmd_values[1],cmd_values[2],cmd_values[3]);
			
			prev_time = cur_time;			
		}
		else{
			prev_time = ros::Time::now().toSec();
		}
		ros::spinOnce();
	}
}

void HeightCallbck(const geometry_msgs::PointStamped& height_){
	position[2] = height_.point.z;
}

void ImuCallbck(const sensor_msgs::Imu& imu_){
	cur_orient = get_cardan_angles(imu_.orientation.x, imu_.orientation.y,
								   imu_.orientation.z, imu_.orientation.w);
	
	#ifndef USE_NEG_ANGLE
		// converting to positive angle [0,360)
		if(cur_orient.pitch < 0) cur_orient.pitch += 360;
		if(cur_orient.roll < 0) cur_orient.roll += 360;
		if(cur_orient.yaw < 0) cur_orient.yaw += 360;
	#endif
}

void CamCallbck(const viman_utility::CamZ& data){
	
}

