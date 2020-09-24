#include "viman_key_ctrl.h"


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
			case 't': vi.toggle_ready();
					  break; 
			
			// Hover
			case 's': vi.allStop();
					  for(int i=0;i<4;i++) cur_values[i]=0;
					  break;
					  
			// Up and down
			case 'q': if(cur_values[0]<0) cur_values[0]=0;
					  cur_values[0] += 0.05;
					  if(cur_values[0] > MAX_LINEAR_Z) cur_values[0]=MAX_LINEAR_Z;
					  vi.move(0,0,cur_values[0]);
					  break;
			case 'w': if(cur_values[0]>0) cur_values[0]=0;
					  cur_values[0] -= 0.05;
					  if(cur_values[0] < -MAX_LINEAR_Z) cur_values[0]=-MAX_LINEAR_Z;
					  vi.move(0,0,cur_values[0]);
					  break;
			
			// Left and Right
			case 'j': if(cur_values[1]<0) cur_values[1]=0;
					  cur_values[1] += 0.05;
					  if(cur_values[1] > MAX_LINEAR_X) cur_values[1]=MAX_LINEAR_X;
					  vi.move(cur_values[1],0,0);
					  break;
			case 'l': if(cur_values[0]>0) cur_values[1]=0;
					  cur_values[1] -= 0.05;
					  if(cur_values[1] < -MAX_LINEAR_X) cur_values[1]=-MAX_LINEAR_X;
					  vi.move(cur_values[1],0,0);
					  break;
			
			// Fwd and Bck
			case 'k': if(cur_values[2]<0) cur_values[2]=0;
					  cur_values[2] += 0.05;
					  if(cur_values[2] > MAX_LINEAR_Y) cur_values[2]=MAX_LINEAR_Y;
					  vi.move(0,cur_values[2],0);
					  break;
			case 'i': if(cur_values[2]>0) cur_values[2]=0;
					  cur_values[2] -= 0.05;
					  if(cur_values[2] < -MAX_LINEAR_Y) cur_values[2]=-MAX_LINEAR_Y;
					  vi.move(0,cur_values[2],0);
					  break;
			
			// Yaw
			case 'u': if(cur_values[3]<0) cur_values[3]=0;
					  cur_values[3] += 0.05;
					  if(cur_values[3] > MAX_YAW) cur_values[3]=MAX_YAW;
					  vi.yaw(cur_values[3]);
					  break;
			case 'o': if(cur_values[3]>0) cur_values[3]=0;
					  cur_values[3] -= 0.05;
					  if(cur_values[3] < -MAX_YAW) cur_values[3]=-MAX_YAW;
					  vi.yaw(cur_values[3]);
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
	
	vi = Viman(node); 
	
	display_help();
	
	// begin separate thread to read from the keyboard		
	std::thread reading_input_thread(read_input);
	reading_input_thread.detach();

	while(do_not_quit && ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
}
