/*
 * rebalancingOffline.cpp
 *
 * Offline, static method of rebalancing: no queuing and each passenger is picked up upon arrival
 * Algorithm minimizes number of vehicles which is needed to satisfy the entire demand
 * Rebalancing between stations is taken into account
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

// void outputFile(const string filename, std::vector<std::vector<int> >& cost);

int main(int argc, char *argv[]) {

	GRBEnv* env = 0;
	GRBVar** nEmptyVhs = 0; // number of empty vehicles traveling between stations
	GRBVar** nVhs_idle = 0; // number of vehicles at each station is unknown

	// stations coordinates
	std::vector<std::vector<double> > stations;
	// distances or cost of traveling between the stations
	// internal vector stores "to where" and external vector stores "from where"
	std::vector<std::vector<double> > cost;
	// origin counts, size of n_rebalancing_periods x n_stations
	std::vector<std::vector<double> > origin_counts;
	// destination counts, size of n_rebalancing_periods x n_stations
	std::vector<std::vector<double> > dest_counts;
	// cost of one idle vehicle in the system, when objective minimizes # vehicles, then set cost to 1,
	// when objective minimizes number of rebalancing vehicles then set vost to a huge number
	// (to make sure that this is always more expensive that rebalancing as it adds more vehicles to the system)
	double cost_of_idle_veh = 1;

	// input and output files declaration
	// simple_model
	const string stationsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/stationsXY.txt";
	const string costMatrixFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/costM3x3.txt";
	const string originCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/origCounts.txt";
	const string destinationCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/destCounts.txt";
	const string modelOutput = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/output_rebalancing.lp";
	const string solutionOutput = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/solution.sol";

	// simmobility files
	//	const string stationsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/inputDemand/ecbd_stations21.txt";
	//	const string costMatrixFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/costMatrixForRebalancingBetween21Stations.txt";
	//	const string originCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/origCounts_reb1800_stations21_updated.txt";
	//	const string destinationCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/destCounts_reb1800_stations21_updated.txt";
	//	const string modelOutput = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/optimizationOut/output_rebalancing.lp";
	//	const string solutionOutput = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/optimizationOut/solution.sol";

	readFiles(stationsFile, stations);
	readFiles(costMatrixFile, cost);
	readFiles(originCountsFile, origin_counts);
	readFiles(destinationCountsFile, dest_counts);

	try {

		// number of stations in the network
		const int nStations = stations.size();
		const int nStSquare = pow (nStations, 2);
		//number of rebalancing periods = number of rows in the origin and destination counts, size of the first vector
		const int nRebPeriods = origin_counts.size();

		// Model
		env = new GRBEnv();
		GRBModel model = GRBModel(*env);
		model.set(GRB_StringAttr_ModelName, "rebalancing");

		// for each station and each rebalancing period --> and decision variables
		// Create decision variables -> how many vehicles should be idling at each period of time and each station
		nVhs_idle = new GRBVar* [nRebPeriods];
		int station;
		int time_;
		div_t divresult;
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			nVhs_idle[time_] = model.addVars(nStations);
			model.update();
			for (station = 0; station < nStations; ++station) {
				ostringstream cname;
				cname << "IdleVhs." << time_ << "." << station;
				if (time_ == 0) {
					nVhs_idle[time_][station].set(GRB_DoubleAttr_Obj, cost_of_idle_veh);
					nVhs_idle[time_][station].set(GRB_StringAttr_VarName, cname.str());
				} else {
					nVhs_idle[time_][station].set(GRB_DoubleAttr_Obj, 0.0);
					nVhs_idle[time_][station].set(GRB_StringAttr_VarName, cname.str());
				}
			}
		}
		// Create decision variables -> how many vehicles to move at each period of time from station i to station j
		nEmptyVhs = new GRBVar* [nRebPeriods];

		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			nEmptyVhs[time_] = model.addVars(nStSquare);
			model.update();

			for(station = 0; station < nStSquare; ++station){
				ostringstream vname;
				divresult = div (station, nStations);
				vname << "nEmptyVhsTime." << time_ << "." << station << "."<< divresult.quot << "." << divresult.rem;
				// std::cout << "nEmptyVhsTime." << time_ << ".indx." << station << ".from."<< divresult.quot << ".to." << divresult.rem << std:: endl;
				nEmptyVhs[time_][station].set(GRB_DoubleAttr_Obj, 0); // cost[divresult.quot][divresult.rem]
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
			// std::cout << "demand at time " << time_ << "= ";
			for (int i = 0; i < nStations; ++i) {
				dem[i] = origin_counts[time_][i] - dest_counts[time_][i];
				// std::cout << origin_counts[time_][i] << " - " << dest_counts[time_][i] << " = " << dem[i] << std::endl;
			}

			GRBLinExpr reb_dep = 0;
			GRBLinExpr reb_arr = 0;

			for(int depSt = 0; depSt < nStations; ++depSt){
				// std::cout << "departure station: " << depSt << std::endl;
				for (int arrSt = 0; arrSt < nStations; ++arrSt) {

					if (depSt != arrSt) {
						int idx = stationMatrix[depSt][arrSt];
						// std:: cout << "indx[" << depSt << "][" << arrSt << "] = " << idx << std::endl;
						int idx2 = stationMatrix[arrSt][depSt];
						// std:: cout << "indx2[" << arrSt << "][" << depSt << "] = " << idx2 << std::endl;
						reb_dep += nEmptyVhs[time_][idx];
						// std::cout << "reb_dep from = " << idx << " ,";
						reb_arr += nEmptyVhs[time_][idx2];
						// std::cout << "reb_arr to = " << idx2 << std::endl;
					}
				}
				//				if (reb_arr.size() > 0) {
				//					reb_arr += nVhs_idle[time_][depSt];
				//				}

				ostringstream cname;
				cname << "Demand" << time_ << "." << depSt;
				model.addConstr(reb_arr - reb_dep >= dem[depSt] - nVhs_idle[time_][depSt], cname.str());
				std::cout << "Constraint 1: " << depSt << " reb_dep.size() = " << reb_dep.size() <<
						" reb_arr.size() = " << reb_dep.size()  << std::endl;
				reb_arr.clear();
				reb_dep.clear();
			}
		}

		// Constraint 2: # of vehicles at time t == # of vehicles at time (t-1)
		int total_origin = 0;
		int total_origin_prev = 0;
		int total_origin_first = 0;
		GRBLinExpr total_idle_vh = 0;
		GRBLinExpr total_idle_vh_prev = 0;
		GRBLinExpr total_idle_vh_first = 0;
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			std::cout << "time_ = " << time_ << std::endl;

			for(int depSt = 0; depSt < nStations; ++depSt){
				total_idle_vh += nVhs_idle[time_][depSt];
				total_origin += origin_counts[time_][depSt];
				std::cout << "origin_counts[time_][depSt] = " << origin_counts[time_][depSt] << std::endl;
			}
//			std::cout << "total_idle_vh = " << total_idle_vh <<
//					"and total_origin = " << total_origin << std:: endl;

			if (time_ == 0) {
				total_idle_vh_prev = total_idle_vh;
				total_origin_prev = total_origin;
				total_idle_vh_first = total_idle_vh;
				total_origin_first = total_origin;
				std::cout << "within time == 0: total_idle_vh_prev = " << total_idle_vh_prev <<
						"and total_origin_prev = " << total_origin_prev << std:: endl;
				total_idle_vh.clear();
				total_origin = 0;

			} else if(time_ == (nRebPeriods - 1)) {
				std::cout << "time_ == (nStations - 1), time = " << time_ << std::endl;
				// last constraint: should be (veh in last period == veh in first period)
				ostringstream cname;
				cname << "VehiclesTime." << time_;
				model.addConstr(total_idle_vh + total_origin == total_origin_first + total_idle_vh_first, cname.str());
				std::cout << "Constraint 2 last: " << total_idle_vh << " + " << total_origin << " = " <<
						total_origin_first << " + " << total_idle_vh_first << std::endl;

			} else {
				ostringstream cname;
				cname << "VehiclesTime." << time_;
				model.addConstr(total_idle_vh + total_origin == total_origin_prev + total_idle_vh_prev, cname.str());
				std::cout << "Constraint 2: " << total_idle_vh << " + " << total_origin << " = " <<
						total_origin_prev << " + " << total_idle_vh_prev << std::endl;

				total_idle_vh_prev.clear();
				total_origin_prev = 0;
				total_idle_vh_prev = total_idle_vh;
				total_origin_prev = total_origin;
//				std::cout << "in else: total_idle_vh_prev = " << total_idle_vh_prev <<
//						"and total_origin_prev = " << total_origin_prev << " total_idle_vh = " << total_idle_vh <<
//						"and total_origin = " << total_origin_prev << std:: endl;

				total_idle_vh.clear();
				total_origin = 0;
			}
		}

		// Solve
		model.optimize();
		model.write(modelOutput);

		cout << "\nTOTAL EMPTY VH DISTANCE: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
		cout << "SOLUTION:" << endl;

		for (time_ = 0; time_ < nRebPeriods; ++time_){

			for(int depSt = 0; depSt < nStations; ++depSt){
				for (int arrSt = 0; arrSt < nStations; ++arrSt) {
					if(depSt != arrSt) {
						int idx = stationMatrix[depSt][arrSt];
						cout << "At time " << time_ << ", rebalancing from station " << depSt <<
								" to station " << arrSt << " send "  <<
								nEmptyVhs[time_][idx].get(GRB_DoubleAttr_X) << " vehicles." << endl;
					}
				}
			}
		}

		for (time_ = 0; time_ < nRebPeriods; ++time_){
			for(int depSt = 0; depSt < nStations; ++depSt){
				cout << "At time " << time_ << ", # of idling vhs at station " << depSt <<
						" = " <<  nVhs_idle[time_][depSt].get(GRB_DoubleAttr_X) <<
						" and total # of vehicles " << nVhs_idle[time_][depSt].get(GRB_DoubleAttr_X) +
						origin_counts[time_][depSt]<< endl;
			}
		}

		model.write(solutionOutput);

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

// write solution to file
//void outputFile(const string filename, std::vector<std::vector<int> >& cost) {
//	std::ofstream out(filename.c_str());
//
//	if (!out.good()) {
//		std::cout << "Cannot write: " << filename << std::endl;
//	}
//
//	for(int i = 0; i < cost.size(); ++i) {
//		std::cout << "time " << i << '\n';
//
//		   for (vector<int>::const_iterator iter = cost[i].begin(); iter != cost[i].end(); ++iter) {
//		        std::cout << *iter << ", ";
//		    }
//		   std::cout << std::endl;
//	}
//}
