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

int roundUp(int numToRound, int multiple);
// a function designed to round up the exact travel time up to the multiple of the rebalancing interval

// void outputFile(const string filename, std::vector<std::vector<int> >& cost);

int main(int argc, char *argv[]) {

	GRBEnv* env = 0; //< gurobi env
	GRBVar** empty_veh = 0; // number of empty vehicles traveling between stations
	GRBVar** vhs_st_i = 0; // number of vehicles available at station i
	GRBVar* in_transfer = 0; // number of vehicles which departed i and have not arrived to j yet
	GRBVar total_veh = 0; // total number of vehicles in the system

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
	double cost_of_adding_veh = 1;
	double rebalancing_cost = 0;

	// input and output files declaration
	// simple_model
	const string stationsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/stationsXY.txt";
	const string costMatrixFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/costM3x3.txt";
	const string originCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/origCounts.txt";
	const string destinationCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/destCounts.txt";
	const string modelOutput = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/rebalancing_formulation.lp";
	const string solutionOutput = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/rebalancing_solution.sol";

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

	//round cost of traveling between stations to multiply of the rebalancing period
	int reb_period = 2; // in minutes
	std::vector<std::vector<int> > rounded_cost;
	for (int i = 0; i < cost.size(); i++){
		for (int j = 0; j < cost[0].size(); j++){
			rounded_cost[i][j] = roundUp((int)cost[i][j], reb_period);
		}
	}

	try {

		// number of stations in the network
		const int nStations = stations.size();
		const int nStSquare = pow (nStations, 2);
		//number of rebalancing periods = number of rows in the origin and destination counts, size of the first vector
		const int nRebPeriods = origin_counts.size();
		const int rebPeriod = 30; //< discretization period, in minutes

		// Model
		env = new GRBEnv();
		GRBModel model = GRBModel(*env);
		model.set(GRB_StringAttr_ModelName, "rebalancing");

		/***********************************************************************************
		 * Decision variables
		 ***********************************************************************************/
		// Number of vehicles available at each period of time and each station
		vhs_st_i = new GRBVar* [nRebPeriods];
		int station;
		int time_;
		div_t divresult;
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			vhs_st_i[time_] = model.addVars(nStations);
			model.update();
			for (station = 0; station < nStations; ++station) {
				ostringstream cname;
				cname << "vhs_st_i." << time_ << "." << station;
				if (time_ == 0) {
					vhs_st_i[time_][station].set(GRB_DoubleAttr_Obj, cost_of_adding_veh);
					vhs_st_i[time_][station].set(GRB_StringAttr_VarName, cname.str());
				} else {
					vhs_st_i[time_][station].set(GRB_DoubleAttr_Obj, 0.0);
					vhs_st_i[time_][station].set(GRB_StringAttr_VarName, cname.str());
				}
			}
		}

		// number of empty vehicles traveling between stations
		empty_veh = new GRBVar* [nRebPeriods];
		// number of empty trips in not directly taken into account in the objective function
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			empty_veh[time_] = model.addVars(nStSquare);
			model.update();

			for(station = 0; station < nStSquare; ++station){
				ostringstream vname;
				divresult = div (station, nStations);
				vname << "nEmptyVhsTime." << time_ << "." << station << "."<< divresult.quot << "." << divresult.rem;
				// std::cout << "nEmptyVhsTime." << time_ << ".indx." << station << ".from."<< divresult.quot << ".to." << divresult.rem << std:: endl;
				// in the current implementation the rebalancing cost is equal to zero
				// rebalancing_cost = cost[divresult.quot][divresult.rem];
				empty_veh[time_][station].set(GRB_DoubleAttr_Obj, rebalancing_cost);
				empty_veh[time_][station].set(GRB_StringAttr_VarName, vname.str());
			}
		}

		// number of vehicles in transfer at each time slot
		in_transfer = model.addVars(nRebPeriods);
		model.update();
		int p;
		for (p = 0; p < nRebPeriods; ++p)
		{
			ostringstream vname;
			vname << "in_transfer" << p;
			in_transfer[p].set(GRB_DoubleAttr_Obj, 0.0);
			in_transfer[p].set(GRB_StringAttr_VarName, vname.str());
		}

		/***********************************************************************************
		 * Objective function
		 ***********************************************************************************/
		// The objective is to minimize the number of vehicles traveling in the network (and cost associated with it)
		model.set(GRB_IntAttr_ModelSense, 1);
		model.update();

		/***********************************************************************************
		 * Station matrix
		 ***********************************************************************************/
		// station matrix to access the elements of cost/rebalancing vectors
		// matrix form stores the indices of the cost vector i.e., for 3 stations [[0,1,2],[3,4,5],[6,7,8]]
		int stationMatrix [nStations][nStations];
		int indxCounter = 0;
		for(int i = 0; i < nStations; ++i) {
			for (int k = 0; k < nStations; ++k) {
				stationMatrix[i][k] = indxCounter;
				//std::cout << "stationMatrix["<< i<< "][" << k << "] = " << indxCounter << endl;
				indxCounter++;
			}
		}

		/***********************************************************************************
		 * Constraint 1: Satisfy the demand at each node at every time interval
		 ***********************************************************************************/
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			// Current demand
			int dem_curr[nStations] = 0;
			// calculate (trips departing - trips arriving) for all stations during the next time interval (time_ + 1)
			// store the demand counts in dem_antic[nStations]

			for (int i = 0; i < nStations; ++i) {
				dem_curr[i] = origin_counts[time_][i] - dest_counts[time_][i];
				// std::cout << origin_counts[time_][i] << " - " << dest_counts[time_][i] << " = " << dem_curr[i] << std::endl;
			}

			// Anticipated demand (demand in the next rebalancing period)
			int dem_next[nStations] = 0;
			// calculate (trips departing - trips arriving) for all stations during the next time interval (time_ + 1)
			// store the demand counts in dem_next[nStations]

			for (int i = 0; i < nStations; ++i) {
				// if we are not in the last interval then we just search for the anticipated demand in the next interval
				// otherwise, we compare against the first interval to close the loop
				if (time_ + 1 != nRebPeriods) {
					dem_next[i] = origin_counts[time_ + 1][i] - dest_counts[time_ + 1][i];
					// std::cout << origin_counts[time_ + 1][i] << " - " << dest_counts[time_ + 1][i] << " = " << dem_next[i] << std::endl;
				} else {
					dem_next[i] = origin_counts[0][i] - dest_counts[0][i];
				}
			}
			for(int depSt = 0; depSt < nStations; ++depSt){
				ostringstream cname;
				cname << "SatisfyFutureDemand" << time_ << "." << depSt;
				// for all stations i: supply must be equal or greater than the demand
				// supply: vehicles arriving - departing + idle > demand departing - arriving
				model.addConstr(vhs_st_i[time_][depSt] >= dem_next[depSt], cname.str());
				std::cout << "Constraint 1: vhs_st_i[" << time_ << "][" << depSt << "] = " <<
						vhs_st_i[time_][depSt] << " >= " << dem_next[depSt] << std::endl;
			}
		}

		/***********************************************************************************
		 * Constraint 2: Evolution of vehicles at station i
		 ***********************************************************************************/
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			// Current demand
			int dem_curr[nStations] = 0;
			// calculate (trips departing - trips arriving) for all stations during the next time interval (time_ + 1)
			// store the demand counts in dem_antic[nStations]

			for (int i = 0; i < nStations; ++i) {
				// current demand = arriving vehicles - departing vehicles
				dem_curr[i] = dest_counts[time_][i] - origin_counts[time_][i];
				// std::cout << origin_counts[time_][i] << " - " << dest_counts[time_][i] << " = " << dem_curr[i] << std::endl;
			}

			GRBLinExpr reb_dep = 0;
			GRBLinExpr reb_arr = 0;

			for(int depSt = 0; depSt < nStations; ++depSt){
				// std::cout << "departure station: " << depSt << std::endl;
				for (int arrSt = 0; arrSt < nStations; ++arrSt) {

					if (depSt != arrSt) {
						int idx = stationMatrix[depSt][arrSt];
						int idx2 = stationMatrix[arrSt][depSt];
						reb_dep += empty_veh[time_][idx];
						int t_veh_departed = rounded_cost[depSt][arrSt];
						reb_arr += empty_veh[t_veh_departed][idx2];
					}
				}
				ostringstream cname;
				cname << "available_veh" << time_ << "." << depSt;
				if (time_ != 0) {
					model.addConstr(vhs_st_i[time_][depSt] ==
							vhs_st_i[time_ - 1][depSt] + reb_arr - reb_dep + dem_curr[depSt], cname.str());
				} else {
					// if we are in the first interval then we compare against the last interval of the previous day
					// in that case I am comparing against the last interval of the same day
					model.addConstr(vhs_st_i[time_][depSt] ==
							vhs_st_i[nRebPeriods - 1][depSt] + reb_arr - reb_dep + dem_curr[depSt], cname.str());
				}
				std::cout << "Constraint 1: " << time_  << "." << depSt << " reb_dep.size() = " << reb_dep.size() <<
						" reb_arr.size() = " << reb_arr.size()  << std::endl;
				reb_arr.clear();
				reb_dep.clear();
			}
		}

		/***********************************************************************************
		 * Constraint 3:
		 ***********************************************************************************/
		// Constraint 2: # of vehicles at time t == # of vehicles at time (t+ delta_t)

		int total_origin = 0; //< total number of departing trips from all nodes at current time step
		int total_origin_prev = 0; //< total number of departing trips from all nodes at previous time step
		int total_origin_first = 0; //< total number of departing trips from all nodes during the first time step
		int total_dest = 0; //< total number of arriving trips to all nodes at current time step
		int total_dest_prev = 0; //< total number of arriving trips to all nodes at previous time step
		int total_dest_first = 0; //< total number of arriving trips to all nodes during the first time step
		GRBLinExpr total_idle_vh = 0; //< Gurobi Linear Expression, total number of idling vehicles at current time step
		GRBLinExpr total_idle_vh_prev = 0; //< Gurobi Linear Expression, total number of idling vehicles at previous time step
		GRBLinExpr total_idle_vh_first = 0; //< Gurobi Linear Expression, total number of idling vehicles during the first time step
		GRBLinExpr total_reb = 0; //< Gurobi Linear Expression, total number of rebalancing vehicles at current time step
		GRBLinExpr total_reb_prev = 0; //< Gurobi Linear Expression, total number of rebalancing vehicles at previous time step
		GRBLinExpr total_reb_first = 0; //< Gurobi Linear Expression, total number of rebalancing vehicles during the first time step

		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			std::cout << "time_ = " << time_ << std::endl;

			for(int depSt = 0; depSt < nStations; ++depSt){
				total_idle_vh += vhs_parked_i[time_][depSt];
				total_origin += origin_counts[time_][depSt];
				std::cout << "origin_counts[" << time_ << "][" << depSt << "] = "
						<< origin_counts[time_][depSt] << std::endl;

				total_dest += dest_counts[time_][depSt];
				std::cout << "dest_counts[" << time_ << "][" << depSt << "] = "
						<< dest_counts[time_][depSt] << std::endl;

				for (int arrSt = 0; arrSt < nStations; ++arrSt) {
					std::cout << "arrSt: " << arrSt << std::endl;
					if (depSt != arrSt) {
						int idx = stationMatrix[depSt][arrSt];
						std::cout << "idx: " << idx << std::endl;
						int idx2 = stationMatrix[arrSt][depSt];
						std::cout << "idx2: " << idx2 << std::endl;
						total_reb += nEmptyVhs[time_][idx];
						std::cout << "total_reb: " << total_reb << std::endl;
						//total_reb -= nEmptyVhs[time_][idx2];
						std::cout << "total_reb = " << total_reb << std::endl;
					}
				}
			}
			//			std::cout << "total_idle_vh = " << total_idle_vh <<
			//					"and total_origin = " << total_origin << std:: endl;

			if (time_ == 0) {
				total_idle_vh_prev = total_idle_vh;
				total_origin_prev = total_origin;
				total_dest_prev = total_dest;
				total_reb_prev = total_reb;
				total_idle_vh_first = total_idle_vh;
				total_origin_first = total_origin;
				total_dest_first = total_dest;
				total_reb_first = total_reb;

				std::cout << "within time == 0: total_idle_vh_prev = " << total_idle_vh_prev <<
						", total_origin_prev = " << total_origin_prev << ", total_dest_prev = "
						<< total_dest_prev << ", total_reb_prev = " << total_reb_prev << std:: endl;

				total_idle_vh.clear();
				total_reb.clear();
				total_origin = 0;
				total_dest = 0;

			} else if(time_ == (nRebPeriods - 1)) {
				std::cout << "time_ == (nStations - 1), time = " << time_ << std::endl;
				// last constraint: should be (veh in last period == veh in first period)
				ostringstream cname;
				cname << "VehiclesAtTime." << time_;
				std::cout << "total_reb.size() = " << total_reb.size() << std::endl;
				model.addConstr(total_idle_vh + total_origin + total_dest + total_reb ==
						total_origin_first + total_idle_vh_first + total_dest_first + total_reb_first, cname.str());

				std::cout << "Constraint 2 last rebalancing period: " << total_idle_vh << " + " << total_origin << " + "
						<< total_dest <<  " + " << total_reb <<" = " << total_origin_first << " + "
						<< total_idle_vh_first << " + " << total_dest_first << " + " << total_reb_first << std::endl;

			} else {
				ostringstream cname;
				cname << "VehiclesAtTime." << time_;
				std::cout << "total_reb.size() = " << total_reb.size() << std::endl;
				model.addConstr(total_idle_vh + total_origin + total_dest + total_reb ==
						total_origin_prev + total_idle_vh_prev + total_dest_prev + total_reb_prev, cname.str());

				std::cout << "Constraint 2: " << total_idle_vh << " + " << total_origin << " + "
						<< total_dest <<  " + " << total_reb <<" = " << total_origin_prev << " + " <<
						total_idle_vh_prev << " + " << total_dest_prev << " + " << total_reb_prev << std::endl;

				total_idle_vh_prev.clear();
				total_origin_prev = 0;
				total_idle_vh_prev = total_idle_vh;
				total_origin_prev = total_origin;
				//				std::cout << "in else: total_idle_vh_prev = " << total_idle_vh_prev <<
				//						"and total_origin_prev = " << total_origin_prev << " total_idle_vh = " << total_idle_vh <<
				//						"and total_origin = " << total_origin_prev << std:: endl;

				total_idle_vh.clear();
				total_reb.clear();
				total_origin = 0;
				total_dest = 0;
			}
		}

		// Solve
		model.optimize();
		model.write(modelOutput);

		cout << "\nTOTAL NUMBER OF VEHICLES: " << model.get(GRB_DoubleAttr_ObjVal) +
				total_dest_first + total_origin_first + total_reb_first << endl;

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
						" = " <<  vhs_parked_i[time_][depSt].get(GRB_DoubleAttr_X) <<
						" and total # of vehicles " << vhs_parked_i[time_][depSt].get(GRB_DoubleAttr_X) +
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

int roundUp(int numToRound, int multiple)
{
	if (multiple == 0)
		return numToRound;

	int remainder = numToRound % multiple;
	if (remainder == 0)
		return numToRound;

	return numToRound + multiple - remainder;
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
