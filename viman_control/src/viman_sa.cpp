#include "viman_sa.h"

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
			case 't': vm.toggle_ready();
					  isPidRunning = true;
					  break; 
			
			// Stop all
			case 's': vm.allStop();
					  for(int i=0;i<4;i++){
						 set_points[i] = 0;
					  }
					  ROS_INFO("Set height: %f",set_points[0]);
					  break;
					  
			// Up and down
			case 'q': set_points[0] += 0.1;
					  ROS_INFO("Set height: %f",set_points[0]);
					  break;
			case 'w': set_points[0] -= 0.1;
					  ROS_INFO("Set height: %f",set_points[0]);
					  break;
			
			// Setting height
			case 'S': std::cout << "\nEnter height to hover (m): ";
					  std::cin >> set_points[0];
					  break;
			
			// Set PID gains
			case 'p': 
					  isPidRunning = false;
					  std::cout << "\nEnter P gain: ";
					  std::cin >> height_controller_.gain_p;
					  std::cout << "Enter I gain: ";
					  std::cin >> height_controller_.gain_i;
					  std::cout << "Enter D gain: ";
					  std::cin >> height_controller_.gain_d;
					  height_controller_.reset();
					  isPidRunning = true;
		}		
	}
}

void display_help(void){
	std::cout << "\nUse the following commands: "<< std::endl
			<< "t: Take off/Land (cmds work iff VIMAN has taken off)" << std::endl
			<< "s: Stop all" << std::endl
			<< "S: Set VIMAN at height" << std::endl
			<< "q: Increase height by 0.1 m" << std::endl
			<< "w: Decrease height by 0.1 m" << std::endl
			<< "p: Set PID gains" << std::endl
			<< "h: Display help" << std::endl
			<< "x: Quit\n" << std::endl;
}

int main(int argc, char **argv){
	
	ros::init(argc,argv,"viman_semiAuto");
	ros::NodeHandle node;
	
	ros::Subscriber height_subs_ = node.subscribe("/viman/height",100,HeightCallbck);
	
	vm = Viman(node);
	height_controller_ = VmPID();
	
	height_controller_.gain_p = 1.5;
	height_controller_.gain_i = 0.02;
	height_controller_.gain_d = 0.01;
	
	display_help();
    if( height_subs_.getTopic() != "")
        ROS_INFO("found altimeter height topic");
    else
        ROS_INFO("cannot find altimeter height topic!");
        
    isPidRunning = false;
    
	//ros::Rate rate(2);
	
	// begin separate thread to read from the keyboard		
	std::thread reading_input_thread(read_input);
	reading_input_thread.detach();
	
	double prev_time = ros::Time::now().toSec();
	double cur_time;
	double dt;
		
	while(do_not_quit && ros::ok()){
		if(isPidRunning){
			cur_time = ros::Time::now().toSec();
			
			dt = cur_time - prev_time;
			
			if(dt <= 0 ) continue;
			
			cmd_values[0] = height_controller_.update(set_points[0], sensor_values[0], 
									  dt);
			
			vm.move(cmd_values[1],cmd_values[2],cmd_values[0]);
			
			prev_time = cur_time;			
		}
		else{
			prev_time = ros::Time::now().toSec();
		}
		ros::spinOnce();
	}
}

void HeightCallbck(const geometry_msgs::PointStamped& height_){
	sensor_values[0] = height_.point.z;
}
