/*
 * rebalancingOffline.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: katarzyna
 */

#include <iostream>
#include <sstream>
#include "gurobi_c++.h"
using namespace std;

//void readFiles(const string stationsFile, std::vector<std::vector<double> >& stations);
void readFiles(const string costMatrixFile, std::vector<std::vector<double> >& cost);
// readFiles function reads from the file station ids, x and y coordinates
// @param stationsFile -> type const string, input file with 3 columns (id, x, y) and n rows describing n stations
// @param &stations -> type std::vector<std::vector<double> >, output variable, vector of stations where each station is described as a vector (id, x, y)

int main(int argc, char *argv[]) {

	GRBEnv* env = 0;
	GRBVar** nEmptyVhs = 0;
	std::vector<std::vector<double> > stations;
	const string stationsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/inputDemand/ecbd_stations21.txt";
	readFiles(stationsFile, stations);

	// distances or cost of traveling between the stations
	std::vector<std::vector<double> > cost;
	const string costMatrixFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/costMatrixForRebalancingBetween21Stations.txt";
	readFiles(costMatrixFile, cost);

	// origin counts
	std::vector<std::vector<double> > origin_counts;
	const string originCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/origCounts_reb1800_stations21_updated.txt";
	readFiles(originCountsFile, origin_counts);

	// destination counts
	std::vector<std::vector<double> > dest_counts;
	const string destinationCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/destCounts_reb1800_stations21_updated.txt";
	readFiles(destinationCountsFile, dest_counts);

	try {

		// number of stations in the network
		const int nStations = stations.size();

		// distances or cost of traveling between the stations
//		int distances[nStations][nStations];
//		for (int i = 0; i < nStations; i++) {
//			for (int j = 0; j < nStations; j++) {
//				distances[i][j] = cost[i][j];
//			}
//
//		}

		// number of vehicles at each station
		int nVhs [nStations];

		// desired number of vehicles at each station
		int nVhsDesired [nStations];

		// surplus of vehicles at each station (positive value if we have extra vehicles, negative value otherwise)
		int excessVhs [nStations];

		for(int k = 0; k< nStations; k++){
			excessVhs[k] = nVhs[k] - nVhsDesired[k];
		}

		// Model
		env = new GRBEnv();
		GRBModel model = GRBModel(*env);
		model.set(GRB_StringAttr_ModelName, "rebalancing");

		// Create decision variables -> how many vehicles to move from station i to station j
		nEmptyVhs = new GRBVar* [nStations];
		int s;
		int t;
		for ( s = 0; s < nStations; ++s)
		{
			nEmptyVhs[s] = model.addVars(nStations); // nStation is the number of variables to add to the model
			model.update();

			for(t = 0; t < nStations; ++t){
				ostringstream vname;
				vname << "nEmptyVhs" << t << "." << s;
				nEmptyVhs[s][t].set(GRB_DoubleAttr_Obj, cost[s][t]);
				nEmptyVhs[s][t].set(GRB_StringAttr_VarName, vname.str());
			}
		}

		// The objective is to minimize the number of empty vehicles traveling on the network (and cost associated with it)
		model.set(GRB_IntAttr_ModelSense, 1);

		// Update model to integrate new variables
		model.update();

		// Constraint 1: Vehicle conservation, we cannot send more vehicles than the station owns

		for (s = 0; s < nStations; ++s)
		{
			GRBLinExpr vToBeSend = 0;
			for (t = 0; t < nStations; ++t){
				if(t != s){
					vToBeSend += nEmptyVhs[s][t];
				}
			}
			ostringstream cname;
			cname << "Capacity" << s;
			model.addConstr(vToBeSend <= nVhs[s], cname.str());
		}


		// Constraint 2: Flow conservation for each station -> to satisfy the demand
		for (s = 0; s < nStations; ++s){

			GRBLinExpr nVhsToSend = 0; // number of vehicles to send from station j to station i, i != j
			GRBLinExpr nVhsToCome = 0;
			for(t = 0; t < nStations; ++t){
				if(t != s){
					nVhsToSend += nEmptyVhs[t][s];
					nVhsToCome += nEmptyVhs[s][t];
				}
			}

			model.addConstr(nVhsToSend - nVhsToCome + excessVhs[s] >= 0, "station" + s);
		}

		// Solve
		model.optimize();
		model.write("/Users/katarzyna/Dropbox/matlab/output_rebalancing.lp");

		cout << "\nTOTAL EMPTY VH DISTANCE: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
		cout << "SOLUTION:" << endl;

		for (s = 0; s < nStations; ++s){

			for (t = 0; t < nStations; ++t){
				if(t != s){
					cout << "From station: " << s <<
							" to station: " << t << " send "  <<
							nEmptyVhs[s][t].get(GRB_DoubleAttr_X) << " vehicles." << endl;
				}
			}
		}

	} catch(GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	} catch(...) {
		cout << "Exception during optimization" << endl;
	}
	return 0;
}


//void readFiles(const string stationsFile, std::vector<std::vector<double> >& stations) {
//
//	// Open the file:
//	std::ifstream in(stationsFile.c_str());
//	if (!in.good()) {
//		std::cout << "Cannot read: " << stationsFile << std::endl;
//	}
//
//	while (in.good()) {
//
//		double st_id;
//		double st_x;
//		double st_y;
//		std::stringstream ss;
//		std:: string element1, element2, element3;
//		char buffer[256000];
//		in.getline(buffer, 256000);
//		if (strlen(buffer) == 0) break;
//
//		ss << buffer;
//		//		std::cout << "Content of buffer: " << buffer << std::endl;
//
//		vector<double> v_temp;
//		while (!ss.eof()){
//			ss >> element1 >> element2 >> element3;
//			//			std::cout << "Content of element2: " << element2 << ", element3: " << element3 << std::endl;
//			st_id = atoi(element1.c_str());
//			st_x = atof(element2.c_str());
//			st_y = atof(element3.c_str());
//			v_temp.push_back(st_id);
//			v_temp.push_back(st_x);
//			v_temp.push_back(st_y);
//
//			//			std::cout << "Content of vector: ";
//			//			std::cout.precision(9); // I am loosing the digits when printing
//			//			for (std::vector<double>::const_iterator i = v_temp.begin(); i != v_temp.end(); ++i) {
//			//				std::cout << *i << ' ';
//			//			}
//			//			std::cout << std::endl;
//		}
//		stations.push_back(v_temp);
//
//	}
//}

void readFiles(const string filename, std::vector<std::vector<double> >& cost) {
	// Open the file:
	std::ifstream in(filename.c_str());
	if (!in.good()) {
		std::cout << "Cannot read: " << filename << std::endl;
	}
	std::cout << "Filename: " << filename << std::endl;

	while (in.good()) {

		double cij_temp;
		std::stringstream ss;
		std:: string line_;
		char buffer[256000];
		in.getline(buffer, 256000);
		if (strlen(buffer) == 0) break;

		ss << buffer;
		std::cout << "Content of buffer: " << buffer << std::endl;

		vector<double> v_temp;
		while (!ss.eof()){
			ss >> line_;
			cij_temp = atof(line_.c_str());
			v_temp.push_back(cij_temp);
		}

		std::cout << "Content of vector: ";
		std::cout.precision(9); // I am loosing the digits when printing
		for (std::vector<double>::const_iterator i = v_temp.begin(); i != v_temp.end(); ++i) {
			std::cout << *i << ' ';
		}
		std::cout << std::endl;

		cost.push_back(v_temp);
	}
}
