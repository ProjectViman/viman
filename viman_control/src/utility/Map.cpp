#include "Map.h"

/**
 * @brief Constructor
 * @param std::string fileName - name of the file in which you wish to store your map. (Without extension)
 * @param std::string lndmrkNodeName - identifier in the map for a landmark node
*/
Map::Map(std::string fileName, std::string lndmrkNodeName){
	_fileName = fileName+".g2o";
	_lndmrkNodeName = lndmrkNodeName;
}

/**
 * @brief Adds landmark to the map
 * Input parameters:
 * @param std::string colorid - color observed
 * @param double z - Z coordinate (wrt local frame) in m
 * @param double yaw - yaw angle (wrt local frame) in deg
*/
void Map::addLndmrkZ(std::string colorid, double z, double yaw){
	LndmrkLocs l;
	l.colorid = colorid;
	l.z = z;
	l.yaw = yaw;

	lndmrks.push_back(l);
}

/**
 * @brief Returns true if writing the map to file is successful, else returns false.
*/
bool Map::writeToFile(){
	std::fstream fio;
	fio.open(_fileName, std::ios::out);
	if(fio.is_open()){
		for(int i = 0; i < lndmrks.size(); i++){
			fio << _lndmrkNodeName + " " + lndmrks[i].colorid + " "
				<< std::to_string(lndmrks[i].z) + " "
				<< std::to_string(lndmrks[i].yaw) << std::endl;
		}
		fio.close();
		return true;
	}
	else{
		return false;
	}
}

/**
 * @brief  Extracts landmarks from file into a vector data member : lndmrks
 * type(lndmrks) : std::vector<LndmrkLocs>
*/
bool Map::readFromFile(){
	std::fstream fio;

	std::string line;
	std::vector<std::string> splitStr;
	LndmrkLocs l;

	lndmrks.resize(0);

	// open the file
	fio.open(_fileName, std::ios::in);

	// Execute a loop untill EOF (End of File) 
    // point read pointer at beginning of file 
    fio.seekg(0, std::ios::beg);

	while (std::getline(fio, line)) {
		// extract data and place it in lndmrks 
		std::istringstream iss(line); 
		for(std::string s; iss >> s; ){
			// std::cout << s << std::endl;
			splitStr.push_back(s);
		}
		if(splitStr[0].compare(_lndmrkNodeName)==0){
			l.colorid = splitStr[1]; 
			l.z = std::stod(splitStr[2]);
			l.yaw = std::stod(splitStr[3]);
			lndmrks.push_back(l);
		}
			splitStr.resize(0);
    } 
    // Close the file 
    fio.close(); 
}
