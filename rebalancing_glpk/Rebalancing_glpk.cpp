/*
 * Rebalancing_glpk.cpp
 *
 *  Created on: Jun 8, 2015
 *      Author: katarzyna
 */

#include <iostream>
#include <sstream>
#include <unordered_map>
#include "glpk.h"
using namespace std;


int
main(int   argc,
		char *argv[])
{

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

	// number of vehicles at each station
	// 3 stations
	int nVhs [nStations];
	nVhs[0] = 10;
	nVhs[1] = 100;
	nVhs[2] = 90;

	// desired number of vehicles at each station
	// 3 stations
	int nVhsDesired [nStations];
	nVhsDesired[0] = 100;
	nVhsDesired[1] = 0;
	nVhsDesired[2] = 100;

	//	std::unordered_map<int,int> index_to_ids;
	//
	//	for (int s = 0; s < nStations; ++s){
	//		for (int t = 0; t < nStations; ++t){
	//			index_to_ids.insert(std::make_pair<int,int>(s+1, t+1));
	//		}
	//	}

	int nRows = 2*nStations;

	//set up the problem
	glp_prob *lp;
	lp = glp_create_prob();
	glp_set_prob_name(lp, "rebalancing");
	glp_set_obj_dir(lp, GLP_MIN);

	// add columns, column bounds and objective coefficients
	int nelems = nStations*nStations;
	glp_add_cols(lp, nelems);

	int i = 1;
	for(int s = 0; s < nStations; ++s){

		for(int t = 0; t < nStations; ++t){

			std::stringstream ss;
			ss << "from station " << s + 1 << "to station " << t + 1;
			const std::string& tmp = ss.str();
			const char* cstr = tmp.c_str();

			glp_set_col_name(lp, i, cstr);
			glp_set_col_bnds(lp, i, GLP_LO, 0.0, 0.0);
			glp_set_obj_coef(lp, i, distances[s][t]);
			++i;
		}
	}

	// indexing the matrix
	int *ia; // format [ia, ja] = ar;
	int  *ja;
	double *ar;
	ia = new int[1000 + 1]; // +1 because glpk starts indexing at 1
	ja = new int[1000 + 1];
	ar = new double[1000 + 1];

	// add row bounds
	i = 1;
	int k = 1; // k-element in the matrix
	int rows = 1;
	int columns = 1;
	glp_add_rows(lp, nRows);
	// add row bounds and constraints: stations cannot send more vehicles than they have CORRECT
	for(int s = 0; s < nStations; ++s){

		std::stringstream ss_;
		ss_ << "eq " << " " << i;
		const std::string& tmp = ss_.str();
		const char* cstr = tmp.c_str();
		glp_set_row_name(lp, i, cstr);
		glp_set_row_bnds(lp, i, GLP_LO, nVhs[s], 1.0);
		++i;

		for(int t = 0; t < nStations; ++t) {
			ia[k] = rows;
			ja[k] = columns;
			ar[k] = 1.0;
			++k;
			++columns;
		}
		++rows;
	} // after this we are at nStations' row and the last column so we have to continue with adding rows and have to come back to the first column

	// add row bounds and constraints: (vehicles coming xji - vehicles departing xij) >= -(excess vehicles)
	columns = 1;
	for(int from = 0; from < nStations; ++from){

		std::stringstream ss_;
		ss_ << "eq " << " " << i;
		const std::string& tmp = ss_.str();
		const char* cstr = tmp.c_str();
		glp_set_row_name(lp, i, cstr);
		double excessVh = (double) nVhs[from] - (double) nVhsDesired[from];
		glp_set_row_bnds(lp, i, GLP_UP, 0.0, -excessVh);
		++i;

		for (int to = 0; to < nStations; ++to){
			//if (from == to) continue;
			ia[k] = rows;
			ja[k] = columns;
			ar[k] = -1.0; // we should not allow for the elem xii to take any value
			++k;
			++columns;
		}
		++rows;
	} // now we are at the last column and the last row, have to come back to the first column and nStations+1 row

	rows = rows - nStations + 1; // rows = nStations + 1;
	columns = columns - nelems + 1; // columns = 1;
	// experimental -> diagonal +1.0
	for(int s = 0; s < nStations; ++s){
		for(int t = 0; t < nStations; ++t) {
			//if (s == t) continue;
			ia[k] = rows;
			ja[k] = columns;
			ar[k] = 1.0; // we should not allow for the elem xii to take any value, because it is being overwritten and gives an error
			++k;
			++columns;
			++rows;
		}
		rows = 1;
	}

	// ------------------------------------------------------------------

	glp_load_matrix(lp, nelems, ia, ja, ar);
	glp_simplex(lp, NULL);

	/* recover and display results */
	double z;
	z = glp_get_obj_val(lp);
	printf("z = %g; \n", z);

	// housekeeping
	glp_delete_prob(lp);
	glp_free_env();

	delete [] ia;
	delete [] ja;
	delete [] ar;

	cout << "Done\n";

}
