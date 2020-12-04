#include "vm_z.h"

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
			case 's': vm.allStop();
					  for(int i=0;i<3;i++){
						 set_points[i] = 0;
					  }
					  set_orient.yaw = 0;
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
			
			// Yaw +-
			case 'u': set_orient.yaw += 10;
					  #ifdef USE_NEG_ANGLE
						if(set_orient.yaw >= 180.01){
							set_orient.yaw = -(360 - set_orient.yaw);
						}
					  #else
						if(set_orient.yaw >= 360){
							set_orient.yaw = 360 - set_orient.yaw;
						}
					  #endif
					  ROS_INFO("Set yaw: %f",set_orient.yaw);
					  break;
			case 'o': set_orient.yaw -= 10;
					  #ifdef USE_NEG_ANGLE
						  if(set_orient.yaw <= -179.9){
							set_orient.yaw = 180;
						  }
					  #else
						if(set_orient.yaw < 0){
							set_orient.yaw = 360 + set_orient.yaw;
						}
					  #endif
					  ROS_INFO("Set yaw: %f",set_orient.yaw);
					  break;

			// reset PIDs
			case 'R': height_controller_.reset();
					  yaw_controller_.reset();
					  break;
			
			// start/stop mapping
			case 'm':if(isMapping){
						isMapping = false;
						ROS_INFO("Stopped mapping");
					  }
					  else{
						isMapping = true;
						ROS_INFO("Started mapping");
						m.lndmrks.resize(0);
					  }
					  break;

			// show stored map
			case 'M':if(isMapping){
						ROS_INFO("Please wait, currently mapping.");
					  }
					  else{
						showStoredMap();
					  }
					  break;
			
			// optimize map
			case 'O':if(isMapping){
						ROS_INFO("Please wait, currently mapping.");
					  }
					  else{
						optimizeMap();
					  }
					  break;

			// save stored map as a file
			case 'S':if(isMapping){
						ROS_INFO("Please wait, currently mapping.");
					  }
					  else{
						saveStoredMap();
					  }
					  break;
		}		
	}
}

void display_help(void){
	std::cout << "\nUse the following commands: "<< std::endl
			<< " --- Motion commands ---" << std::endl
			<< "t: Take off/Land (cmds work iff VIMAN has taken off)" << std::endl
			<< "s: Stop and set height = 0 m" << std::endl
			<< "q: Increase height by 0.1 m" << std::endl
			<< "w: Decrease height by 0.1 m" << std::endl
			<< "u: Counter-clockwise yaw by 10 deg" << std::endl
			<< "o: Clockwise yaw by 10 deg" << std::endl
			<< "R: Reset PIDs" << std::endl
			<< "\n --- Mapping commands ---" << std::endl
			<< "m: Start/Stop mapping" << std::endl
			<< "M: Show stored map" << std::endl
			<< "S: Save stored map" << std::endl
			<< "O: Optimize map and store optimized map" << std::endl
			<< "\n --- Misc commands ---" << std::endl
			<< "z: Display help" << std::endl
			<< "x: Quit\n" << std::endl;
}

int main(int argc, char **argv){
	
	ros::init(argc,argv,"vm_z");
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
	
	yaw_controller_.gain_p = 0.2;
	yaw_controller_.gain_i = 0.01;
	yaw_controller_.gain_d = 0.05;
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
	isMapping = false;
    
	//ros::Rate rate(2);
	
	// begin separate thread to read from the keyboard		
	std::thread reading_input_thread(read_input);
	reading_input_thread.detach();
	
	double prev_time = ros::Time::now().toSec();
	double cur_time;
	double dt;
		
	set_points[2] = 0.1;
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

			if(isMapping){
				setNextSetPoint();
				addDataPoint();
			}

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

void CamCallbck(const viman_utility::CamZ& camData_){
	cur_color = camData_.name;
}

/**
 * Implements the landmark searching algorithm:
 * 
 * Viman moves linearly in fixed step sizes. Once it reaches a height, it
 * rotates and searches for landmarks. 
 */
void setNextSetPoint(){
	// once viman reaches a set height
	if((position[2] >= set_points[2] - SET_POINT_RANGE) && 
	(position[2] <= set_points[2] + SET_POINT_RANGE)){
		// rotate 360 degrees and search
		// update the set point for continuos rotation
		if((cur_orient.yaw >= set_orient.yaw - SEARCH_CONST*SET_POINT_RANGE) &&
			(cur_orient.yaw <= set_orient.yaw + SEARCH_CONST*SET_POINT_RANGE)){
				if(set_orient.yaw == 180){
					set_orient.yaw = 0;
					// increment the height set point once viman comes back to
					// original position
					set_points[2] += SET_POINT_STEP_LIN;
					ROS_INFO("Increased height set point");
				}
				else{
					set_orient.yaw = 180;
				}	
		}

		// stop searching once upper limit is reached
		if(set_points[2] >= MAX_HEIGHT){
			height_controller_.reset();
			yaw_controller_.reset();
			set_orient.yaw = 0;
			set_points[2] = 0;
			isMapping = false;
			ROS_INFO("Finished mapping. Returning to ground.");
		}
	}
}

void addDataPoint(){
	if(prev_color.compare(cur_color)){
		if(cur_color.compare("")){
			m.addLndmrkZ(cur_color, position[2], cur_orient.yaw);
		}
		prev_color = cur_color;
	}
}

void showStoredMap(){
	if(m.lndmrks.size()>0){
		std::cout << "STORED MAP: " << std::endl;
		for(int i=0;i<m.lndmrks.size();i++){
		std::cout << "Feature color: " << m.lndmrks[i].colorid + " --> @Z: " + std::to_string(m.lndmrks[i].z)
			 <<  " | @Yaw: " + std::to_string(m.lndmrks[i].yaw) << std::endl;
		}
	}
	else{
		ROS_INFO("No stored map. Please map first.");
	}
}

void saveStoredMap(){
	if(m.lndmrks.size()>0){
		if(m.writeToFile()){
			ROS_INFO("Successfully written the map to a file.");
		}
	}
	else{
		ROS_INFO("No stored map. Please map first.");
	}
	
}

void optimizeMap(){
	if(m.lndmrks.size()>0){
		std::vector<std::string> colors(5);
		colors.at(0) = "yellow";
		colors.at(1) = "red"; 
		colors.at(2) = "blue"; 
		colors.at(3) = "green"; 
		colors.at(4) = "purple";
		
		m.optimize_map(colors, 8.6, 2);
		std::string temp = m._fileName;
		m._fileName = temp + "_opt";
		if(m.writeToFile()){
			ROS_INFO("Successfully written the optimized map to a file.");
		}
		m._fileName = temp;
	}
	else{
		ROS_INFO("No stored map. Please map first.");
	}
}