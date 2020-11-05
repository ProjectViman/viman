#include "Map.h"

int main(){
	Map m("/home/mate/viman_ws/src/viman/viman_control/maps/test","LNDMRK");

	// m.addLndmrkZ("red", 0.1, 0);
	// m.addLndmrkZ("blue", 0.2, 20);
	// m.addLndmrkZ("random", 0.3, 50);

	m.readFromFile();

	for(int i=0;i<m.lndmrks.size();i++){
		std::cout << m.lndmrks[i].colorid + " " + std::to_string(m.lndmrks[i].z) + " "
			 << std::to_string(m.lndmrks[i].yaw) << std::endl;
	}


	// m.writeToFile();

	return 0;
}