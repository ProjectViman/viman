#include "Map.h"

/**
 * Input parameters:
 * std::string fileName - name of the file in which you wish to store your map. (Without extension)
 * std::string lndmrkNodeName - identifier in the map for a landmark node
*/
Map::Map(std::string fileName, std::string lndmrkNodeName){
	_fileName = fileName+".g2o";
	_lndmrkNodeName = lndmrkNodeName;
}

/**
 * Closes the map file if it is open.
*/
Map::~Map(){
	if(_fio.is_open()){
		_fio.close();
	}
}

/**
 * Returns true if writing to the map is successful, else returns false.
 * Input parameters:
 * std::string colorid - color observed
 * double z - Z coordinate (wrt local frame) in m
 * double yaw - yaw angle (wrt local frame) in deg
*/
bool Map::write(std::string colorid, double z, double yaw){
	if(_fio.is_open()){
		_fio << _lndmrkNodeName + " " + colorid + " " + std::to_string(z) + " "
			 << std::to_string(yaw) << std::endl;
		return true;
	}
	else{
		return false;
	}
}

/**
 * @brief  Extracts the landmarks into a vector data member : lndmrks
 * type(lndmrks) : std::vector<LndmrkLocs>
*/
void Map::extractLndmrks(){
	if(_fio.is_open()){
		return;
	}
	else{
		std::string line;
		std::vector<std::string> splitStr;
		LndmrkLocs l;

		// open the file
		_fio.open(_fileName, std::ios::in);

		// Execute a loop untill EOF (End of File) 
    	// point read pointer at beginning of file 
    	_fio.seekg(0, std::ios::beg);

		while (std::getline(_fio, line)) {
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
    	_fio.close(); 
	}
}

/**
 * Opens and closes stored map.
*/
void Map::toggleMapping(){
	if(_fio.is_open()){
		_fio.close();
	}
	else{
		_fio.open(_fileName, std::ios::out);
	}
}

/**
 * Returns true is mapping is on going, else false
*/
bool Map::mappingStatus(){
	return _fio.is_open();
}