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
	GRBVar** passenger_dep = 0; // number of vehicles with passenger departing at each station
	GRBVar** passenger_arr = 0; // number of vehicles with passenger arriving at each station
	GRBVar** vhs_st_i = 0; // number of vehicles available at station i
	GRBVar* in_transit = 0; // number of vehicles which departed i and have not arrived to j yet

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
	double cost_of_veh = 1;
	double rebalancing_cost = 0;

	// input and output files declaration
	// simple_model
	const string costMatrixFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/costM3x3.txt";
	const string originCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/origCounts.txt";
	const string destinationCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/destCounts.txt";
	const string modelOutput = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/rebalancing_formulation.lp";
	const string solutionOutput = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/rebalancing_solution.sol";
	int reb_period = 2; // in minutes


	// simmobility files
	//		const string costMatrixFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/costMatrixForRebalancingBetween21Stations.txt";
	//		const string originCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/origCounts_reb1800_stations21_updated.txt";
	//		const string destinationCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/destCounts_reb1800_stations21_updated.txt";
	//		const string modelOutput = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/optimizationOut/output_rebalancing.lp";
	//		const string solutionOutput = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/optimizationOut/solution.sol";
	//		int reb_period = 30; // in minutes

	//readFiles(stationsFile, stations);
	readFiles(costMatrixFile, cost);
	readFiles(originCountsFile, origin_counts);
	readFiles(destinationCountsFile, dest_counts);


	/***********************************************************************************
	 * Round cost of traveling between stations to be a multiply of the rebalancing period
	 ***********************************************************************************/

	int rounded_cost[cost.size()][cost.size()];
	for (int i = 0; i < cost.size(); i++){
		for (int j = 0; j < cost[0].size(); j++){
			rounded_cost[i][j] = roundUp((int)cost[i][j], reb_period);
			std::cout << "Cost[" << i<< "][" << j << "] = " << cost[i][j] <<
					", Rounded cost[" << i<< "][" << j << "] = " << rounded_cost[i][j] << std::endl;
		}
	}

	try {

		// number of stations in the network
		const int nStations = cost.size();
		const int nStSquare = pow (nStations, 2);
		//number of rebalancing periods = number of rows in the origin and destination counts, size of the first vector
		const int nRebPeriods = origin_counts.size();

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
		 * Model
		 ***********************************************************************************/
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
		// div_t divresult;
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			vhs_st_i[time_] = model.addVars(nStations); // GRB_INTEGER
			model.update();
			for (station = 0; station < nStations; ++station) {
				ostringstream cname;
				cname << "vhs_st_i." << time_ << "." << station;
				if (time_ == 0) {
					vhs_st_i[time_][station].set(GRB_DoubleAttr_Obj, cost_of_veh);
					vhs_st_i[time_][station].set(GRB_StringAttr_VarName, cname.str());
				} else {
					vhs_st_i[time_][station].set(GRB_DoubleAttr_Obj, 0.0);
					vhs_st_i[time_][station].set(GRB_StringAttr_VarName, cname.str());
				}
			}
		}

		std::cout << "Decision variable vhs_st_i added." << std::endl;

		// number of empty vehicles traveling between stations
		empty_veh = new GRBVar* [nRebPeriods];
		// number of empty trips in not directly taken into account in the objective function
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			empty_veh[time_] = model.addVars(nStSquare);
			model.update();

			for(int depSt = 0; depSt < nStations; ++depSt){
				// std::cout << "departure station: " << depSt << std::endl;
				for (int arrSt = 0; arrSt < nStations; ++arrSt) {

					int idx = stationMatrix[depSt][arrSt];
					ostringstream vname;
					vname << "nEmptyVhsTime." << time_ << "." << depSt << "."<< arrSt;
					//std::cout << "nEmptyVhsTime." << time_ << "." << depSt << "."<< arrSt << "."<< idx << std::endl;

					if (depSt != arrSt) {
						// std::cout << "nEmptyVhsTime." << time_ << ".indx." << station << ".from."<< divresult.quot << ".to." << divresult.rem << std:: endl;
						// in the current implementation the rebalancing cost is equal to zero
						// rebalancing_cost = cost[divresult.quot][divresult.rem];
						// or rebalancing_cost = cost[depSt][arrSt];
						empty_veh[time_][idx].set(GRB_DoubleAttr_Obj, rebalancing_cost);
						empty_veh[time_][idx].set(GRB_StringAttr_VarName, vname.str());
					} else {
						empty_veh[time_][idx].set(GRB_DoubleAttr_Obj, rebalancing_cost);
						empty_veh[time_][idx].set(GRB_StringAttr_VarName, vname.str());
					}
				}
			}
		}
		std::cout << "Decision variable empty_veh added." << std::endl;

		// number of vehicles in transfer at each time slot
		in_transit = model.addVars(nRebPeriods);
		model.update();
		for (time_ = 0; time_ < nRebPeriods; ++time_)
		{
			ostringstream vname;
			vname << "in_transit" << time_;
			if (time_ == 0) {
				in_transit[time_].set(GRB_DoubleAttr_Obj, cost_of_veh);
				in_transit[time_].set(GRB_StringAttr_VarName, vname.str());
			} else {
				in_transit[time_].set(GRB_DoubleAttr_Obj, 0);
				in_transit[time_].set(GRB_StringAttr_VarName, vname.str());
			}
		}
		std::cout << "Decision variable in_transit added." << std::endl;

		// number of customers to be serviced
		// passenger_dep
		passenger_dep = new GRBVar* [nRebPeriods];
		// div_t divresult;
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			passenger_dep[time_] = model.addVars(nStations); // GRB_INTEGER
			model.update();
			for (station = 0; station < nStations; ++station) {
				ostringstream cname;
				cname << "passenger_dep_st_i." << time_ << "." << station;
				passenger_dep[time_][station].set(GRB_DoubleAttr_Obj, 0.0);
				passenger_dep[time_][station].set(GRB_StringAttr_VarName, cname.str());
			}
		}
		std::cout << "Decision variable passenger_dep added." << std::endl;

		// passenger_arr
		passenger_arr = new GRBVar* [nRebPeriods];
		// div_t divresult;
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			passenger_arr[time_] = model.addVars(nStations); // GRB_INTEGER
			model.update();
			for (station = 0; station < nStations; ++station) {
				ostringstream cname;
				cname << "passenger_arr_st_i." << time_ << "." << station;
				passenger_arr[time_][station].set(GRB_DoubleAttr_Obj, 0.0);
				passenger_arr[time_][station].set(GRB_StringAttr_VarName, cname.str());
			}
		}
		std::cout << "Decision variable passenger_arr added." << std::endl;

		/***********************************************************************************
		 * Objective function
		 ***********************************************************************************/
		// The objective is to minimize the number of vehicles traveling in the network (and cost associated with it)
		model.set(GRB_IntAttr_ModelSense, 1);
		model.update();

		/***********************************************************************************
		 * Constraint 1: Flow conservation at station i
		 ***********************************************************************************/
		div_t divresult;
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {

			GRBLinExpr reb_dep = 0;
			GRBLinExpr reb_arr = 0;
			GRBLinExpr pas_dep = 0;
			GRBLinExpr pas_arr = 0;

			for(int depSt = 0; depSt < nStations; ++depSt){
				// std::cout << "departure station: " << depSt << std::endl;
				pas_dep += passenger_dep[time_][depSt];
				pas_arr += passenger_arr[time_][depSt];

				for (int arrSt = 0; arrSt < nStations; ++arrSt) {

					if (depSt != arrSt) {
						int idx = stationMatrix[depSt][arrSt];
						int idx2 = stationMatrix[arrSt][depSt];
						reb_dep += empty_veh[time_][idx];
						// std::cout << "rounded_cost = " << (int)rounded_cost[depSt][arrSt]/reb_period -1 << std::endl;
						int travel_cost = (int) (rounded_cost[depSt][arrSt]/reb_period);
						if (travel_cost > time_) {
							//we have to add the vehicle in the counts for the previous day
							int dep_time = nRebPeriods + time_ - travel_cost;
							reb_arr += empty_veh[dep_time][idx2];
						} else {
							int dep_time = time_ - travel_cost;
							reb_arr += empty_veh[dep_time][idx2];
						}
					}
				}
				ostringstream cname;
				cname << "available_veh" << time_ << "." << depSt;
				if (time_ != 0) {
					model.addConstr(vhs_st_i[time_][depSt] ==
							vhs_st_i[time_ - 1][depSt] + reb_arr - reb_dep - pas_dep + pas_arr, cname.str());
				} else {
					// if we are in the first interval then we compare against the last interval of the previous day
					// in that case I am comparing against the last interval of the same day
					model.addConstr(vhs_st_i[0][depSt] ==
							vhs_st_i[nRebPeriods - 1][depSt] + reb_arr - reb_dep - pas_dep + pas_arr, cname.str());
				}
				std::cout << "Constraint 1: " << time_  << "." << depSt << std::endl;
				reb_arr.clear();
				reb_dep.clear();
				pas_dep.clear();
				pas_arr.clear();
			}
		}

		std::cout << "Constraint set 1 added." << std::endl;

		/***********************************************************************************
		 * Constraint 2: Conservation of the number of vehicles in transit
		 ***********************************************************************************/
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {

			GRBLinExpr reb_dep = 0;
			GRBLinExpr reb_arr = 0;
			GRBLinExpr pas_dep = 0;
			GRBLinExpr pas_arr = 0;

			for(int depSt = 0; depSt < nStations; ++depSt){
				pas_dep += passenger_dep[time_][depSt];
				pas_arr += passenger_arr[time_][depSt];

				for (int arrSt = 0; arrSt < nStations; ++arrSt) {

					if (depSt != arrSt) {
						int idx = stationMatrix[depSt][arrSt];
						int idx2 = stationMatrix[arrSt][depSt];
						reb_dep += empty_veh[time_][idx];
						int travel_cost = (int) (rounded_cost[depSt][arrSt]/reb_period);
						if (travel_cost > time_) {
							//we have to add the vehicle in the counts for the previous day
							int dep_time = nRebPeriods + time_ - travel_cost;
							reb_arr += empty_veh[dep_time][idx2];
						}else {
							int dep_time = time_ - travel_cost;
							reb_arr += empty_veh[dep_time][idx2];
						}
					}
				}
			}

			ostringstream cname;
			cname << "veh_in_transit" << time_;
			if (time_ != 0) {
				model.addConstr(in_transit[time_] ==
						in_transit[time_ - 1] + (reb_dep - reb_arr) - pas_arr + pas_dep , cname.str());
			} else {
				// if we are in the first interval then we compare against the last interval of the previous day
				// in my case I am comparing against the last interval of the same day
				model.addConstr(in_transit[time_] ==
						in_transit[nRebPeriods - 1] + (reb_dep - reb_arr) + pas_dep - pas_arr, cname.str());
			}
			//std::cout << "Constraint 2: " << time_  << " in_transit = " << in_transit << std::endl;
			reb_arr.clear();
			reb_dep.clear();
			pas_dep.clear();
			pas_arr.clear();
		}

		std::cout << "Constraint set 2 added." << std::endl;

		/***********************************************************************************
		 * Constraint 3: Number of departing/arriving passengers should be less or equal
		 * to the demand in current and previous step
		 ***********************************************************************************/
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			for(int depSt = 0; depSt < nStations; ++depSt) {

				GRBLinExpr pas_dep = 0;
				GRBLinExpr pas_arr = 0;

				pas_dep += passenger_dep[time_][depSt];
				pas_arr += passenger_arr[time_][depSt];

				ostringstream cname, aname;
				cname << "dep_passengers." << time_ << depSt;
				aname << "arr_passengers." << time_ << depSt;
				if (time_ != 0) {
					model.addConstr(pas_dep <=
							origin_counts[time_][depSt] + origin_counts[time_ - 1][depSt], cname.str());
					model.addConstr(pas_arr <=
							dest_counts[time_][depSt] + dest_counts[time_ - 1][depSt], aname.str());
				} else {
					model.addConstr(pas_dep <=
							origin_counts[0][depSt] + origin_counts[nRebPeriods - 1][depSt], cname.str());
					model.addConstr(pas_arr <=
							dest_counts[0][depSt] + dest_counts[nRebPeriods - 1][depSt], aname.str());
				}
				pas_dep.clear();
				pas_arr.clear();
			}
		}
		std::cout << "Constraint set 3 added." << std::endl;
		/***********************************************************************************
		 * Constraint 4: Sum of sent vehicles over the whole day must be equal to sum of
		 * demand for each station
		 ***********************************************************************************/
		for(int depSt = 0; depSt < nStations; ++depSt) {
			// Total demand
			int total_dep = 0;
			int total_arr = 0;
			GRBLinExpr pas_dep = 0;
			GRBLinExpr pas_arr = 0;

			for ( time_ = 0; time_ < nRebPeriods; ++time_) {
				pas_dep += passenger_dep[time_][depSt];
				pas_arr += passenger_arr[time_][depSt];
				total_arr += origin_counts[time_][depSt];
				total_dep += dest_counts[time_][depSt];
			}

			ostringstream cname, aname;
			cname << "dep_passengers." << time_ << "." << depSt;
			aname << "arr_passengers." << time_ << "." << depSt;

			model.addConstr(pas_dep == total_dep, cname.str());
			model.addConstr(pas_arr == total_arr, aname.str());

			pas_dep.clear();
			pas_arr.clear();
		}

		std::cout << "Constraint set 4 added." << std::endl;
		/***********************************************************************************
		 * Solve the problem and print solution to the console and to the file
		 ***********************************************************************************/
		model.optimize();
		model.write(modelOutput);

		int total_demand = 0;
		int dem_curr[nStations];
		for (int i = 0; i < nStations; ++i) {
			// current demand = arriving vehicles - departing vehicles
			dem_curr[i] = dest_counts[0][i] + origin_counts[0][i];
			total_demand += dem_curr[i];
		}

		double numOfVeh = model.get(GRB_DoubleAttr_ObjVal) + total_demand;

		std::cout << "\nTOTAL NUMBER OF VEHICLES: " << numOfVeh << std::endl;
		std::cout << "\nTOTAL NUMBER OF AVAILABLE VEH AT TIME 0: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
		std::cout << "SOLUTION:" << endl;

		for (time_ = 0; time_ < nRebPeriods; ++time_){

			for(int depSt = 0; depSt < nStations; ++depSt){
				for (int arrSt = 0; arrSt < nStations; ++arrSt) {
					if(depSt != arrSt) {
						int idx = stationMatrix[depSt][arrSt];
						std::cout << "At time " << time_ << ", rebalancing from station " << depSt <<
								" to station " << arrSt << " send "  <<
								empty_veh[time_][idx].get(GRB_DoubleAttr_X) << " vehicles." << std::endl;
					}
				}
			}
		}

		for (time_ = 0; time_ < nRebPeriods; ++time_){
			for(int arrSt = 0; arrSt < nStations; ++arrSt){
				std::cout << "At time " << time_ << " at station " << arrSt <<
						" number of available vehicles: " << vhs_st_i[time_][arrSt].get(GRB_DoubleAttr_X) << std::endl;
			}
			std::cout << "At time " << time_ << " number of vehicles in transit: "
					<< in_transit[time_].get(GRB_DoubleAttr_X) << std::endl;
		}
		model.write(solutionOutput);

		std::cout << "PROGRAM COMPLETED WITH SUCCESS." << std::endl;

	} catch(GRBException e) {
		cout << "Error code = " << e.getErrorCode() << std::endl;
		cout << e.getMessage() << std::endl;
	} catch(...) {
		cout << "Exception during optimization" << std::endl;
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
