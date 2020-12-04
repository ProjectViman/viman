#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <math.h>

struct LndmrkLocs{
	std::string colorid;
	double z;
	double yaw;
};

class Map{

	private:
		std::string _lndmrkNodeName;

	public:
		std::string _fileName;
		std::vector<LndmrkLocs> lndmrks;

		Map(std::string fileName, std::string _lndmrkNodeName);
		void addLndmrkZ(std::string colorid, double z, double yaw);
		bool writeToFile();
		bool readFromFile();
		std::vector<LndmrkLocs> extractLndmrks(std::string, std::vector<LndmrkLocs> &);
		void optimize_map(std::vector<std::string>&, float, int);
};

class AdjacencyMatrix{
private:
    int n;
    int **adj;
    bool *visited;
    
public:
    AdjacencyMatrix(int n);
    void add_edge(int origin, int destin);
    void display();
    std::vector<int> BFS(int s);
    std::vector<int> get_outliers();
};

#endif