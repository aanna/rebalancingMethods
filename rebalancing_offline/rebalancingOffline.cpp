/*
 * rebalancingOffline.cpp
 *
 * Offline, static method of rebalancing: here is no queueing and each passenger is picked up upon arrival
 *
 *
 *  Created on: Nov 24, 2015
 *      Author: katarzyna
 */

#include <iostream>
#include <sstream>
#include <math.h>
#include <stdlib.h>
#include "gurobi_c++.h"
using namespace std;

void readFiles(const string filename, std::vector<std::vector<double> >& cost);
// readFiles function reads from the input file and outputs file content into vector of vector doubles
// i.e., given a file of cost/distances between a number of stations stored as a table, the function
// generates "cost" table in vector<vector> type.
// the function is used to load stations location, counts of origins and destinations, and distances between stations
// @param fileToRead -> type const string, input file, i.e., 3 columns (id, x, y) and n rows describing n stations
// @param &cost -> type std::vector<std::vector<double> >, output variable, i.e., vector of rows of stations
// where each station is described as a vector (id, x, y)
// the internal vector stores number of counts for each station at the same time interval while
// the external vector stores different periods of time

int main(int argc, char *argv[]) {

	GRBEnv* env = 0;
	GRBVar** nEmptyVhs = 0; // number of empty vehicles traveling between stations
	GRBVar** nVhsIdle = 0; // number of vehicles at each station is unknown

	std::vector<std::vector<double> > stations;
	//const string stationsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/inputDemand/ecbd_stations21.txt";
	const string stationsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/stationsXY.txt";
	readFiles(stationsFile, stations);

	// distances or cost of traveling between the stations
	// internal vector stores "to where" and external vector stores "from where"
	std::vector<std::vector<double> > cost;
	//const string costMatrixFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/costMatrixForRebalancingBetween21Stations.txt";
	const string costMatrixFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/costM3x3.txt";
	readFiles(costMatrixFile, cost);

	// origin counts
	std::vector<std::vector<double> > origin_counts;
	//const string originCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/origCounts_reb1800_stations21_updated.txt";
	const string originCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/origCounts.txt";
	readFiles(originCountsFile, origin_counts);

	// destination counts
	std::vector<std::vector<double> > dest_counts;
	//const string destinationCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/destCounts_reb1800_stations21_updated.txt";
	const string destinationCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/destCounts.txt";
	readFiles(destinationCountsFile, dest_counts);

	try {

		// number of stations in the network
		const int nStations = stations.size();
		//number of rebalancing periods = number of rows in the origin and destination counts, size of the first vector
		const int nRebPeriods = origin_counts.size();

		// Model
		env = new GRBEnv();
		GRBModel model = GRBModel(*env);
		model.set(GRB_StringAttr_ModelName, "rebalancing");

		// for each station and each rebalancing period --> and decision variables
		// Create decision variables -> how many vehicles should be idling at each period of time and each station
		nVhsIdle = new GRBVar* [nRebPeriods];
		int station;
		int time_;
		div_t divresult;
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			nVhsIdle[time_] = model.addVars(nStations);
			model.update();
		}
		// Create decision variables -> how many vehicles to move at each period of time from station i to station j
		nEmptyVhs = new GRBVar* [nRebPeriods];

		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			nEmptyVhs[time_] = model.addVars(nStations * nStations);
			model.update();

			for(station = 0; station < (nStations * nStations); ++station){
				ostringstream vname;
				divresult = div (station, nStations);
				vname << "nEmptyVhsTime" << time_ << "." << floor(station/nStations) << "." << divresult.rem;
				nEmptyVhs[time_][station].set(GRB_DoubleAttr_Obj, cost[floor(station/nStations)][divresult.rem]);
				nEmptyVhs[time_][station].set(GRB_StringAttr_VarName, vname.str());
			}
		}

		// The objective is to minimize the number of empty vehicles traveling on the network (and cost associated with it)
		model.set(GRB_IntAttr_ModelSense, 1);
		model.update();

		// Constraint 1: Satisfy the demand at each node at every time interval

		// first fill in the station matrix so that later you can access the elements of it
		// matrix is in a form i.e., for 3 stations [[0,1,2],[3,4,5],[6,7,8]] it stores the indices
		int stationMatrix [nStations][nStations];
		int indxCounter = 0;
		for(int i = 0; i < nStations; ++i) {
			for (int k = 0; k < nStations; ++k) {
				stationMatrix[i][k] = indxCounter;
				//std::cout << "stationMatrix["<< i<< "][" << k << "] = " << indxCounter << endl;
				indxCounter++;
			}
		}

		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			// calculate (demand leaving - demand coming) for all stations at period time_
			int dem[nStations];
			std::cout << "demand at time " << time_ << "= ";
			for (int i = 0; i < nStations; i++) {
				dem[i] = origin_counts[time_][i] - dest_counts[time_][i];
				std::cout << origin_counts[time_][i] << " - " << dest_counts[time_][i] << " = " << dem[i] << std::endl;
			}

			GRBLinExpr reb_dep = 0;
			GRBLinExpr reb_arr = 0;

			for(int depSt = 0; depSt < nStations; depSt++){
				for (int arrSt = 0; arrSt < nStations; arrSt++) {

					if (depSt != arrSt) {
						std::cout << "depSt: " << depSt << "arrSt: " << arrSt << std::endl;
						int idx = stationMatrix[depSt][arrSt];
						int idx2 = stationMatrix[arrSt][depSt];
						reb_dep += nEmptyVhs[time_][idx];
						std::cout << "reb_dep from = " << idx << " ,";
						reb_arr += nEmptyVhs[time_][idx2];
						std::cout << "reb_arr to = " << idx2 << std::endl;
					}
				}
				ostringstream cname;
				cname << "Demand" << time_ << "." << depSt;
				model.addConstr(reb_arr - reb_dep >= dem[depSt], cname.str());
			}

			// Solve
			model.optimize();
			model.write("/Users/katarzyna/Dropbox/matlab/output_rebalancing.lp");

			cout << "\nTOTAL EMPTY VH DISTANCE: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
			cout << "SOLUTION:" << endl;

			//		for (time_ = 0; time_ < nRebPeriods; ++time_){
			//
			//			for (station = 0; station < nStations; ++station){
			//				divresult = div (station, nStations);
			//				if((int)floor(station/nStations) != divresult.rem ) {
			//					fromToV[station] = stationMatrix[(int)floor(station/nStations)][divresult.rem];
			//
			//					cout << "From station: " << (int)floor(station/nStations) <<
			//							" to station: " << divresult.rem << " send "  <<
			//							nEmptyVhs[time_][fromToV[station]].get(GRB_DoubleAttr_X) << " vehicles." << endl;
			//				}
			//			}
		}

	} catch(GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	} catch(...) {
		cout << "Exception during optimization" << endl;
	}
	return 0;
}

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
		//		std::cout << "Content of buffer: " << buffer << std::endl;

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
