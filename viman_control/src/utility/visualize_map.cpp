#include <cmath>
#include <vector>
#include "matplotlibcpp.h"
#include "Map.h"

using namespace std;
namespace plt = matplotlibcpp;

void plot_loc(float, std::string);

std::vector<std::string> colors(5), color_code(5);
Map m("/home/mate/viman_ws/src/viman/viman_control/maps/zMap1", "LNDMRKZ");

int main() {
	colors.at(0) = "yellow"; color_code.at(0) = "#F1EA00";
	colors.at(1) = "red"; color_code.at(1) = "#FF3A0F";
	colors.at(2) = "blue"; color_code.at(2) = "#1C3BDD";
	colors.at(3) = "green"; color_code.at(3) = "#34F100";
	colors.at(4) = "purple"; color_code.at(4) = "#A706FF";

	m.readFromFile();
	plot_loc(10.0, "Un-optimized map");

	plt::close();

	m._fileName = m._fileName + "_opt";
	m.readFromFile();
	plot_loc(80.0, "Optimized map");

}

void plot_loc(float size, std::string name){
	// variables to store data
	vector<double> x, y;
	std::map<string, string> keywords;

	for(int i=0; i < colors.size(); i++){
		x.clear();
		y.clear();
		// take data of a particular color
		for(int j=0; j < m.lndmrks.size(); j++){
			if(m.lndmrks[j].colorid.compare(colors[i])==0){
				x.push_back(m.lndmrks[j].yaw);
				y.push_back(m.lndmrks[j].z);
			}
		}
		// plot the data
		keywords["c"] = color_code[i];
		plt::scatter(x, y, size, keywords);
	}
	plt::title(name);
	plt::xlabel("Yaw (deg)");
	plt::ylabel("Height (m)");

    plt::show();
}