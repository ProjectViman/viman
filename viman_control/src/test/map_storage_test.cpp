#include "Map.h"

int main(){
	Map m("/home/mate/viman_ws/src/viman/viman_control/maps/test","LNDMRK");

	m.toggleMapping();

	m.write("red", 0.1, 0);
	m.write("yellow", 0.2, 20);
	m.write("random", 0.3, 50);

	m.toggleMapping();

	m.extractLndmrks();

	for(int i=0;i<m.lndmrks.size();i++){
		std::cout << m.lndmrks[i].colorid + " " + std::to_string(m.lndmrks[i].z) + " "
			 << std::to_string(m.lndmrks[i].yaw) << std::endl;
	}

	return 0;
}