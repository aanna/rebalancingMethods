/*
 * rebalancingOffline.cpp
 *
 * Offline rebalancing: no queuing is allowed (each passenger is picked up upon arrival in the system)
 * Algorithm minimizes number of vehicles which is needed to satisfy the entire demand
 * Rebalancing only between stations is taken into account
 *
 * We input the booking counts between zones/stations aggregated into intervals
 * of the same size as the rebalancing interval.
 * We assume the travel cost (time or distance) is known and constant over time.
 * Station is understood as the centroid of the zone.
 *
 * Based on the demand counts the algorithm solves for the minimum number of vehicles
 * required to carry the entire demand. Rebalancing trips at time t are to satisfy customers
 * at time (t+1)
 *
 * REMEMBER TO SET THE CORRECT REBALANCING INTERVAL (In the future it has to be done in a smarter way than manually)
 * (line 103: int reb_period = 900;)
 *
 *  Created on: Nov 24, 2015
 *      Author: katarzyna
 */

#include <iostream>
#include <sstream>
#include <math.h>
#include <stdlib.h>
#include <cstring>
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

	// do we want to force integer solution?
	// if false, we solve a linear program without the integrity constraint
	bool integer_solution = false;
	bool objective_min_N = false;

	GRBEnv* env = 0; //< gurobi env
	GRBVar** rij = 0; // number of empty vehicles traveling between stations
	GRBVar** vi = 0; // number of vehicles available at station i

	// stations coordinates
	std::vector<std::vector<double> > stations;
	// distances or cost of traveling between the stations
	// internal vector stores "to where" and external vector stores "from where"
	std::vector<std::vector<double> > cost;
	// origin counts, size of n_rebalancing_periods x n_stations
	std::vector<std::vector<double> > origin_counts;
	// destination counts, size of n_rebalancing_periods x n_stations
	std::vector<std::vector<double> > dest_counts;
	// counts of vehicles in transit to each station, size of n_rebalancing_periods x n_stations
	std::vector<std::vector<double> > in_transit_counts;
	// cost of one idle vehicle in the system, when objective minimizes # vehicles, then set cost to 1,
	// when objective minimizes number of rebalancing vehicles then set cost to a huge number
	// (to make sure that this is always more expensive that rebalancing as it adds more vehicles to the system)
	const double cost_of_veh = 1.0;
	const double rebalancing_cost = 1.0;
	// what constraints do you want to take into account

	// input and output files declaration
	// simple_model
	bool simple_model = true;
	const string stationsFile = "/home/kasia/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/stationsXY.txt";
	const string costMatrixFile = "/home/kasia/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/costM3x3TT.txt";
	const string originCountsFile = "/home/kasia/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/origCounts3x3TT.txt";
	const string destinationCountsFile = "/home/kasia/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/destCounts3x3TT.txt";
	const string inTransitCountsFile = "/home/kasia/Dropbox/matlab/2015-09_FleetSizeEstimation/sampleFiles/inTransit3x3TT.txt";
	const string modelOutput = "rebalancing_formulation_simple.lp";
	const string solutionOutput = "rebalancing_solution_simple.sol";

	//	// simmobility files
	//		// ubuntu
//		bool simple_model = false;
//		const string stationsFile = "/home/kasia/Dropbox/matlab/2016-03-Demand_generation/facility_location/stations_ecbd34.txt";
//		const string costMatrixFile = "/home/kasia/Dropbox/matlab/2015-09_FleetSizeEstimation/RebTimeInSecs34Stations.txt";
//		const string originCountsFile = "/home/kasia/Dropbox/matlab/2015-09_FleetSizeEstimation/origCounts_rebEvery900_stations34.txt";
//		const string destinationCountsFile = "/home/kasia/Dropbox/matlab/2015-09_FleetSizeEstimation/destCounts_rebEvery900_stations34.txt";
//		const string inTransitCountsFile = "/home/kasia/Dropbox/matlab/2015-09_FleetSizeEstimation/inTransitReb900Stations34";
//		const string modelOutput = "formulation_rebalancing.lp";
//		const string solutionOutput = "rebalancing_solution.sol";

	// mac
	//		stationsFile = "/Users/katarzyna/Dropbox/matlab/2016-03-Demand_generation/facility_location/stations_ecbd34.txt";
	//		costMatrixFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/RebTime34Stations.txt";
	//		originCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/origCounts_rebEvery900_stations34.txt";
	//		destinationCountsFile = "/Users/katarzyna/Dropbox/matlab/2015-09_FleetSizeEstimation/destCounts_rebEvery900_stations34.txt";


	//readFiles(stationsFile, stations);
	readFiles(costMatrixFile, cost);
	readFiles(originCountsFile, origin_counts);
	readFiles(destinationCountsFile, dest_counts);
	readFiles(inTransitCountsFile, in_transit_counts);

	//round cost of traveling between stations to be a multiple of the rebalancing period
	int reb_period = 1;
	if(!simple_model) {
		reb_period = 900; // in minutes, output from costOfRebalancing.m is in secs
	}

	int rounded_cost[cost.size()][cost.size()];
	for (unsigned int i = 0; i < cost.size(); i++){
		for (unsigned int j = 0; j < cost[0].size(); j++){
			if (i != j) {
				// cost in seconds from the beginning of the simulation
				rounded_cost[i][j] = roundUp((int)cost[i][j], reb_period);
				//std::cout << "roundUp((int)cost[" << i <<"][" << j << "] = " << rounded_cost[i][j] << std::endl;
			} else {
				rounded_cost[i][j] = 99999999; // 0
			}
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
		// Number of vehicles available (idle) at each period of time and each station
				vi = new GRBVar* [nRebPeriods];
				int station;
				int time_;
				// div_t divresult;
				for ( time_ = 0; time_ < nRebPeriods; ++time_) {

					if (integer_solution) {
						vi[time_] = model.addVars(nStations, GRB_INTEGER); // GRB_INTEGER
					} else {
						vi[time_] = model.addVars(nStations);
					}
					model.update();
					for (station = 0; station < nStations; ++station) {
						ostringstream cname;
						cname << "v_ti," << time_ << "," << station << ",0";
						if (time_ == 0) {
							// add in the objective
							vi[time_][station].set(GRB_DoubleAttr_Obj, 0.0); // cost_of_veh
							vi[time_][station].set(GRB_StringAttr_VarName, cname.str());
						} else {
							// not minimized in the objective
							vi[time_][station].set(GRB_DoubleAttr_Obj, 0.0);
							vi[time_][station].set(GRB_StringAttr_VarName, cname.str());
						}
					}
				}
				model.update();

				// number of empty vehicles traveling between stations
				rij = new GRBVar* [nRebPeriods];
				// number of empty trips is not directly taken into account in the objective function
				for ( time_ = 0; time_ < nRebPeriods; ++time_) {
					if (integer_solution) {
						rij[time_] = model.addVars(nStSquare, GRB_INTEGER);
					} else {
						rij[time_] = model.addVars(nStSquare);
					}
					model.update();

					for(int depSt = 0; depSt < nStations; ++depSt){
						// std::cout << "departure station: " << depSt << std::endl;
						for (int arrSt = 0; arrSt < nStations; ++arrSt) {

							int idx = stationMatrix[depSt][arrSt];
							ostringstream vname;
							vname << "r_tij," << time_ << "," << depSt << ","<< arrSt;
							//std::cout << "nEmptyVhsTime." << time_ << "." << depSt << "."<< arrSt << "."<< idx << std::endl;

							if (depSt != arrSt) {
								// std::cout << "nEmptyVhsTime." << time_ << ".indx." << station << ".from."<< divresult.quot << ".to." << divresult.rem << std:: endl;
								// in the current implementation the rebalancing cost is equal to zero
								// rebalancing_cost = cost[divresult.quot][divresult.rem];
								// or rebalancing_cost = cost[depSt][arrSt];
								if (time_ == 0) {
									// add in the objective
									rij[time_][idx].set(GRB_DoubleAttr_Obj, rebalancing_cost * rounded_cost[depSt][arrSt]); // rebalancing_cost
									rij[time_][idx].set(GRB_StringAttr_VarName, vname.str());
								} else {
									// we do not count it in the objective
									rij[time_][idx].set(GRB_DoubleAttr_Obj, rebalancing_cost * rounded_cost[depSt][arrSt]); // rebalancing_cost
									rij[time_][idx].set(GRB_StringAttr_VarName, vname.str());
								}
							} else { // origin == destination
								// this variable should not exist because we do not send vehicles within the same station
								rij[time_][idx].set(GRB_DoubleAttr_Obj, 0.0);
								rij[time_][idx].set(GRB_StringAttr_VarName, vname.str());
							}
						}
					}
				}
				model.update();

		/***********************************************************************************
		 * Objective function
		 ***********************************************************************************/
		// The objective is to minimize the number of vehicles traveling in the network (and cost associated with it)
		// Optimization sense:  The default +1.0 value indicates that the objective is to minimize the
		// objective. Setting this attribute to -1 changes the sense to maximization
		model.set(GRB_IntAttr_ModelSense, 1);
		model.update();

		/***********************************************************************************
		 * Constraint 1: The number of vehicles must be sufficient to serve the demand
		 ***********************************************************************************/
		// This constraint is set to ensure that the number of vehicles available at each station i
		// is sufficient to serve the demand within this station.
		// If there is not enough vehicles, then we do rebalancing to make sure we can serve the trips
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			// Current demand
			int booking_requests[nStations];
			for (int i = 0; i < nStations; ++i) {
				// booking_requests = departing_vehicles - arriving_vehicles (how many more vehicles do we need)
				booking_requests[i] = origin_counts[time_][i] - dest_counts[time_][i];
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
						reb_dep += rij[time_][idx];
						// std::cout << "rounded_cost = " << (int)rounded_cost[depSt][arrSt]/reb_period -1 << std::endl;
						// cost is expressed in the number of reb periods, i.e., 1,2,3,...nRebPeriods
						int travel_cost = (int) (rounded_cost[depSt][arrSt]/reb_period); // maybe ceil would be better than just int
						// ceil version of the above line
						// int travel_cost = (int) ceil((rounded_cost[depSt][arrSt]/reb_period));
						if (travel_cost > time_) {
							int dep_time = nRebPeriods + time_ - travel_cost;
							reb_arr += rij[dep_time][idx2];
						} else {
							int dep_time = time_ - travel_cost;
							reb_arr += rij[dep_time][idx2];
						}
					}
				}
				ostringstream cname;
				cname << "demand_ti" << time_ << "." << depSt;
				model.addConstr(vi[time_][depSt] + reb_arr - reb_dep >= booking_requests[depSt], cname.str());

				//std::cout << "Constraint 1 demand: " << time_  << "." << depSt << std::endl;
				reb_arr.clear();
				reb_dep.clear();
			}
		}
		model.update();
		std::cout << "Constraint set 1 added." << std::endl;

		/***********************************************************************************
		 * Constraint 2: Constant number of vehicles over time
		 ***********************************************************************************/
		// This constraint is set to ensure that the total number of vehicles in the system does not change
		// over time
		// the total number of vehicles in the system is equal to the sum of the vehicles owned by each station
		// vehicles owned by station i: available_veh + cust_arriving + cust_in_transit_to_station + empty_arr + empty_in_transit

		/* --------------------------------------------------------------
		HAS TO BE CORRECTED !!
		----------------------------------------------------------------*/
		int total_vhs_owned[nStations];
		int total_vhs_at_t[time_];
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			total_vhs_at_t[time_] = 0;
			for (int i = 0; i < nStations; ++i) {
				// total_vhs_owned is the number of vehicles owned by station i
				// this number does not account for rebalancing vehicles
				// total_vhs_owned = arriving_costomers - departing_customers + in_transit_to_station
				total_vhs_owned[i] = dest_counts[time_][i] + in_transit_counts[time_][i];
				// std::cout << "total_vhs_owned[" << i << "] = " << total_vhs_owned[i] << std::endl;
				total_vhs_at_t[time_] += total_vhs_owned[i];
			}
			//std::cout << "total_vhs_at_t[" << time_ << "] = " << total_vhs_at_t[time_] << std::endl;
		}

		GRBLinExpr veh_total_i = 0; // Gurobi Linear Expression, total number of available vehicles now
		GRBLinExpr veh_total_i_first = 0; // Gurobi Linear Expression, total number of available vehicles in first time step
		GRBLinExpr veh_total_i_previous = 0; // Gurobi Linear Expression, total number of available vehicles in previous time

		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			std::cout << "time_ = " << time_ << std::endl;

			// adding variables
			for(int depSt = 0; depSt < nStations; ++depSt) {
				// for station depSt:

				if(time_ == 0) {
					// variables at the last time interval veh_total_i_last
					// veh_total_at_depSt = idling_vehicles
					// previous time step is the last interval
					veh_total_i_previous += vi[nRebPeriods - 1][depSt];
					for(int arrSt = 0; arrSt < nStations; ++arrSt) {
						if (depSt != arrSt) {
							//std::cout << "time_ = " << time_ << ", depSt = " << depSt << ", arrSt = " << arrSt <<  std::endl;
							// vehicles arriving in the station
							int idx_arr = stationMatrix[arrSt][depSt];
							// get the travel time and the departure time for trips arriving now to the station i
							// cost is expressed in the number of reb periods, i.e., 1,2,3,...nRebPeriods
							int travel_cost = (int) (rounded_cost[depSt][arrSt]/reb_period);

							// is should never happen that (nRebPeriods - 1 - travel_cost < 0), but for the sake of correctness
							if (nRebPeriods - 1 - travel_cost >= 0) {
								int dep_time = nRebPeriods - 1 - travel_cost;
								// veh_total_at_depSt = idling_vehicles + reb_arriving
								veh_total_i_previous += rij[dep_time][idx_arr];
								// std::cout << "veh_total_i_last.size() = " << veh_total_i_previous.size() << std::endl;
							} else {
								// (nRebPeriods - 1 - travel_cost < 0)
								// do some reminder stuff
								std::cout << "ERROR: (nRebPeriods - 1 - travel_cost < 0)" << std::endl;
							}
						}
					}
				}
			}

			for (int depSt = 0; depSt < nStations; ++depSt) {
				// add variables for all intervals
				// veh_total_at_depSt = idling_vehicles
				veh_total_i += vi[time_][depSt];
				for(int arrSt = 0; arrSt < nStations; ++arrSt) {
					if (depSt != arrSt) {
						// vehicles arriving in the station
						int idx_arr = stationMatrix[arrSt][depSt];
						// veh_total_at_depSt = idling_vehicles - rebalancing_departing
						//veh_total_i -= rij[time_][idx_dep];
						//veh_total_i_previous -= rij[time_ - 1][idx_dep];
						// get the travel time and the departure time for trips arriving now to the station i
						// cost is expressed in the number of reb periods, i.e., 1,2,3,...nRebPeriods
						int travel_cost = (int) (rounded_cost[depSt][arrSt]/reb_period);

						// current time step time_
						if (travel_cost > time_) {
							int dep_time = nRebPeriods + time_ - travel_cost;
							// veh_total_at_depSt = idling_vehicles - rebalancing_departing + reb_arriving
							veh_total_i += rij[dep_time][idx_arr];
							// std::cout << "veh_total_i.size() = " << veh_total_i.size() << std::endl;
						} else {
							int dep_time = time_ - travel_cost;
							// veh_total_at_depSt = idling_vehicles - rebalancing_departing + reb_arriving
							veh_total_i += rij[dep_time][idx_arr];
							// std::cout << "veh_total_i.size() = " << veh_total_i.size() << std::endl;
						}
					}
				}
			}

			// add constraints
			if (time_ == 0) {
				// we skip time_= zero; and for all other intervals except the last one
				ostringstream cname;
				cname << "supply_t" << time_ ;

				// total number of vehicles at time t == total number of vehicles at time (t-1)
				model.addConstr(veh_total_i + total_vhs_at_t[time_] == veh_total_i_previous + total_vhs_at_t[nRebPeriods - 1], cname.str());
				model.update();

				//std::cout << "Constraint 2 supply at time : " << time_  <<  std::endl;
				veh_total_i_previous.clear();
				veh_total_i_previous = veh_total_i;
				veh_total_i.clear();
			} else {
				// add constraints
				// we skip time_= zero; and for all other intervals except the last one
				ostringstream cname;
				cname << "supply_t" << time_ ;

				// total number of vehicles at time t == total number of vehicles at time (t-1)
				model.addConstr(veh_total_i + total_vhs_at_t[time_] == veh_total_i_previous + total_vhs_at_t[time_ - 1], cname.str());
				model.update();

				//	std::cout << "Constraint 2 supply at time : " << time_  <<  std::endl;
				veh_total_i_previous.clear();
				veh_total_i_previous = veh_total_i;
				veh_total_i.clear();
			}

		} // end constraints loop: for ( time_ = 0; time_ < nRebPeriods; ++time_)
		std::cout << "Constraint set 2 added." << std::endl;

		/***********************************************************************************
		 * Constraint 3: Flow conservation at station i
		 ***********************************************************************************/
		// This constraint is set to ensure that the number of vehicles available at station i at time t
		// is equal to the number of vehicles available at this station in previous time interval
		// plus whatever has arrived minus whatever has departed
		for ( time_ = 0; time_ < nRebPeriods; ++time_) {
			// Current demand
			int dem_curr[nStations];
			for (int i = 0; i < nStations; ++i) {
				// current demand = arriving_vehicles - departing_vehicles
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
						reb_dep += rij[time_][idx];
						// std::cout << "rounded_cost = " << (int)rounded_cost[depSt][arrSt]/reb_period -1 << std::endl;
						int travel_cost = (int) (rounded_cost[depSt][arrSt]/reb_period);
						if (travel_cost > time_) {
							int dep_time = nRebPeriods + time_ - travel_cost;
							reb_arr += rij[dep_time][idx2];
						} else {
							int dep_time = time_ - travel_cost;
							reb_arr += rij[dep_time][idx2];
						}
					}
				}
				ostringstream cname;
				cname << "flow_ti" << time_ << "." << depSt;
				if (time_ != nRebPeriods - 1) {
					model.addConstr(vi[time_ + 1][depSt] ==
							vi[time_][depSt] + reb_arr - reb_dep + dem_curr[depSt], cname.str());
				} else {
					// if we are in the last interval then we compare against the first interval of the next day
					// here: vi(t=0) == vi(t=Tp)
					model.addConstr(vi[0][depSt] ==
							vi[time_][depSt] + reb_arr - reb_dep + dem_curr[depSt], cname.str());
				}
				//std::cout << "Constraint 1 vi @: " << time_  << "." << depSt << std::endl;
				reb_arr.clear();
				reb_dep.clear();
			}
		}
		model.update();
		std::cout << "Constraint set 3 added." << std::endl;
		/***********************************************************************************
		 * Solve the problem and print solution to the console and to the file
		 ***********************************************************************************/
		model.optimize();
		model.write(modelOutput);

		int total_demand = 0;
		int dem_curr[nStations];
		for (int i = 0; i < nStations; ++i) {
			dem_curr[i] = dest_counts[0][i] + in_transit_counts[0][i];
			total_demand += dem_curr[i];
		}

		double numOfVeh = model.get(GRB_DoubleAttr_ObjVal) + total_demand; //plus rebalancing counts

		cout << "\nTOTAL NUMBER OF VEHICLES: " << numOfVeh << std::endl;
		cout << "\nTOTAL NUMBER OF AVAILABLE VEH AT TIME 0: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
		//		cout << "SOLUTION:" << endl;
		//
		//		for (time_ = 0; time_ < nRebPeriods; ++time_){
		//
		//			for(int depSt = 0; depSt < nStations; ++depSt){
		//				for (int arrSt = 0; arrSt < nStations; ++arrSt) {
		//					if(depSt != arrSt) {
		//						int idx = stationMatrix[depSt][arrSt];
		//						cout << "At time " << time_ << ", rebalancing from station " << depSt <<
		//								" to station " << arrSt << " send "  <<
		//								empty_veh[time_][idx].get(GRB_DoubleAttr_X) << " vehicles." << std::endl;
		//					}
		//				}
		//			}
		//		}
		//
		//		for (time_ = 0; time_ < nRebPeriods; ++time_){
		//			for(int arrSt = 0; arrSt < nStations; ++arrSt){
		//				cout << "At time " << time_ << " at station " << arrSt <<
		//						" number of available vehicles: " << vhs_st_i[time_][arrSt].get(GRB_DoubleAttr_X) << std::endl;
		//			}
		//			cout << "At time " << time_ << " number of vehicles in transit: "
		//					<< in_transit[time_].get(GRB_DoubleAttr_X) << std::endl;
		//		}

		model.write(solutionOutput);

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

