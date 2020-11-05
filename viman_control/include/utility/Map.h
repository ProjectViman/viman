#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

struct LndmrkLocs{
	std::string colorid;
	double z;
	double yaw;
};

class Map{

	private:
		std::string _fileName;
		std::string _lndmrkNodeName;

	public:
		std::vector<LndmrkLocs> lndmrks;

		Map(std::string fileName, std::string _lndmrkNodeName);
		void addLndmrkZ(std::string colorid, double z, double yaw);
		bool writeToFile();
		bool readFromFile();
};

#endif