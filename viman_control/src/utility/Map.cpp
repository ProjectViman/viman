#include "Map.h"


AdjacencyMatrix::AdjacencyMatrix(int n){
        this->n = n;
        visited = new bool [n];
        // Mark all the vertices as not visited
        for (int i=0; i<n; i++) {
            visited[i] = false;
        }
        
        // initialize matrix
        adj = new int* [n];
        for (int i = 0; i < n; i++){
            adj[i] = new int [n];
            for(int j = 0; j < n; j++)
            {
                adj[i][j] = 0;
            }
        }
}

/**
 * Adding Edge to Graph
*/
void AdjacencyMatrix::add_edge(int origin, int destin){
    if( origin > n || destin > n || origin < 0 || destin < 0){
        std::cout<<"Invalid edge!\n";
    }
    else{
        adj[origin][destin] = 1;
    }
}

/** 
 * Print the graph
*/
void AdjacencyMatrix::display(){
    int i,j;
    for(i = 0;i < n;i++){
	    for(j = 0; j < n; j++)
            std::cout<<adj[i][j]<<"  ";
        std::cout<<std::endl;
    }
}

/**
 * Breadth First Search
*/
std::vector<int> AdjacencyMatrix::BFS(int s){
    std::vector<int> cluster;
    if (!visited[s]){
        // Create a queue for BFS
        std::list<int> queue;
            
        // Mark the current node as visited and enqueue it
        visited[s] = true;
        queue.push_back(s);
            
        while (!queue.empty()){
            // Dequeue a vertex from queue and print it
            s = queue.front();
            cluster.push_back(s);
            queue.pop_front();
                
            for (int i=0; i<n; i++){
                // Connect to other vertex and that vertex "i" is not visited
                if (adj[s][i] != 0 && !visited[i]){
                    visited[i] = true;
                    queue.push_back(i);
                }
            }
        }
    }
	return cluster;
}
    
/**
 * Get the outliers
*/
std::vector<int> AdjacencyMatrix::get_outliers(){
    std::vector<int> outLiers;
    for(int i=0; i<n; i++){
        if(!visited[i])
            outLiers.push_back(i);
    }
    return outLiers;
}


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

/**
 * @brief  Extracts the landmarks of given "colorid" from vector : lmrks
 * @param string colorid
 * @param std::vector<LndmrkLocs> lmrks
*/
std::vector<LndmrkLocs> Map::extractLndmrks(std::string colorid, std::vector<LndmrkLocs> &lmrks){
	std::vector<LndmrkLocs> exMrks;
	if(lmrks.size() > 0){
		for(int i=0; i < lmrks.size(); i++){
			if(lmrks[i].colorid.compare(colorid)==0){
				exMrks.push_back(lmrks[i]);
			}
		}
		std::cout << colorid <<" | initial number of points : " << exMrks.size();
	}
	else{
		std::cout << "Some prob";
	}
	return exMrks;
}

/**
 * @brief Optimizes the map made by the bot by using DBSCAN algorithm. The map must be present in this->lndmarks
 * @param std::vector<std::string> colors: vector of colors to optimize for
 * @param float radius: radius of cluster from core point
 * @param int thresh: threshold number of points in a cluster. Default: 2
*/
void Map::optimize_map(std::vector<std::string> &colors, float radius, int thresh=2){
	if(lndmrks.size() > 0){
		LndmrkLocs l;
		// save a copy map
		std::vector<LndmrkLocs> lmrks = lndmrks;
		lndmrks.clear();
		// extract data points of each color
		std::vector<LndmrkLocs> exMrks;
		for(int color_id = 0; color_id < colors.size(); color_id++){
			exMrks = extractLndmrks(colors[color_id], lmrks);

			// Read points
			std::vector< std::pair<double, double> > pts;
			for(int i = 0; i < exMrks.size(); i++){
				std::pair<double, double> tmp = std::make_pair(exMrks[i].yaw, exMrks[i].z);
				pts.push_back(tmp);
			}
			
			// Find Core Points
			int mat_size = (int)pts.size();
			std::vector<int> corePts;
			
			for (int i=0; i<mat_size; i++){
				int neighborPtCount = 0;
				
				// Find neighbor points
				for (int j=0; j<mat_size; j++){
					// Check sqrt( (x1-x2)^2 + (y1-y2)^2 ) <= r
					bool isInside = sqrt(pow(pts[i].first-pts[j].first,2)+pow(pts[i].second-pts[j].second,2)) <= radius;
					if (isInside)
						neighborPtCount++;
				}
				
				// Check if i is core point
				if (neighborPtCount >= thresh){
					corePts.push_back(i);
				}
			}
			
			// Generate directed graph
			AdjacencyMatrix dbscan_graph(mat_size);
			for (int i=0; i<corePts.size(); i++){
				int corePt = corePts[i];
				for (int j=0; j<mat_size; j++){
					if (i == j)
						continue;
					bool isInside = sqrt(pow(pts[corePt].first-pts[j].first,2)+pow(pts[corePt].second-pts[j].second,2)) <= radius;
					if (isInside)
						dbscan_graph.add_edge(corePt, j);
				}
			}
			
			// Form clusters (BFS)
			std::vector< std::vector<int> > clusters;   // Store the clusters
			for (int i=0; i<corePts.size(); i++){
				int corePt = corePts[i];
				std::vector<int> cluster = dbscan_graph.BFS(corePt);
				if (!cluster.empty())
					clusters.push_back(cluster);
			}
			
			std::cout << " optimized points: " << clusters.size() << std::endl;

			// Condense all points in a cluster into cluster average
			for(int i=0; i < clusters.size(); i++){
				int s = clusters[i].size();
				double avg_yaw = 0;
				double avg_z = 0;
				// take average of all points in a cluster
				for(int j=0;j < s; j++){
					avg_yaw += pts[clusters[i][j]].first; 
					avg_z += pts[clusters[i][j]].second;
				}
				avg_yaw /= s;
				avg_z /= s;
				l.colorid = exMrks[0].colorid;
				l.yaw = avg_yaw;
				l.z = avg_z;
				lndmrks.push_back(l);
			}
		}
		std::cout << "Successfully optimized the map." << std::endl;
	}
	else{
		std::cout << "Please make sure the 'extractLndmrks' function is called before." << std::endl;
		return;
	}
}