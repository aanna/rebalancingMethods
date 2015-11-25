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

void readStations(const string stationsFile, std::vector<std::vector<double> >& stations);
// readStation function reads from the file station ids, x and y coordinates
// @param stationsFile -> type const string, input file with 3 columns (id, x, y) and n rows describing n stations
// @param &stations -> type std::vector<std::vector<double> >, output variable, vector of stations where each station is described as a vector (id, x, y)
void readCounts (const string countsFile, std::vector<std::vector<int>& counts);
void readCostMatrix(const string costMatrixFile, std::vector<std::vector<double> >& stations);

int main(int argc, char *argv[]) {

	GRBEnv* env = 0;
	GRBVar** nEmptyVhs = 0;
	//int emptyVhsDistance = 0;

	try {

		// number of stations in the network
		const int nStations = 3;

		// distances between the stations

		// 3 stations
		int distances[nStations][nStations];
		distances[0][0] = 0;
		distances[1][1] = 0;
		distances[2][2] = 0;
		distances[0][1] = 100;
		distances[0][2] = 1;
		distances[1][2] = 1;
		distances[1][0] = 100;
		distances[2][0] = 1;
		distances[2][1] = 1;

		// random distances
		//		int distances[nStations][nStations];
		//		for (int i = 0; i < nStations; ++i){
		//			for (int j = 0; j < nStations; ++j){
		//				distances[i][j] = rand() % 10000;
		//			}
		//		}

		// number of vehicles at each station

		// 3 stations
		int nVhs [nStations];
		nVhs[0] = 0;
		nVhs[1] = 100;
		nVhs[2] = 100;
	//	int nVhsTotal = 200;

		// random values

		//		int nVhs [nStations];
		//		int sumOfVeh = 0;
		//		for (int i = 0; i < nStations; ++i){
		//			nVhs[i] = rand() % 10;
		//			sumOfVeh += nVhs[i];
		//		}

		// desired number of vehicles at each station

		// 3 stations
		int nVhsDesired [nStations];
		nVhsDesired[0] = 200;
		nVhsDesired[1] = 0;
		nVhsDesired[2] = 0;

		// random values
		//		int nVhsDesired [nStations];
		//		int sumOfVehDesired = 0;
		//		for (int i = 0; i < nStations; ++i){
		//			nVhsDesired[i] = rand() % 10;
		//			sumOfVehDesired += nVhs[i];
		//		}

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
				nEmptyVhs[s][t].set(GRB_DoubleAttr_Obj, distances[s][t]);
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


