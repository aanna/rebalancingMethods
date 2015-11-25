/*
 * MinNumberOfFacilities.cpp
 *
 *  Created on: May 12, 2015
 *      Author: katarzyna
 */

#include <sstream>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include "gurobi_c++.h"
using namespace std;

void readMatrix_aij(const string filename, long int& size, std::vector<std::vector<long int> >& matrix);
void writeAnswerToFile(const string filename_out, std::vector<int>& list_of_facilites);

int main(int argc, char *argv[])
{
	GRBEnv* env = 0;
	GRBVar* open = 0; // we want to minimize number of opened facilities

	try
	{
		const string filename = "/Users/katarzyna/Dropbox/matlab/2015-05 PKITS/demand/aij_matrix/aij_matrix_ExtCBD-2015-05-20-EveningPeak2500m.txt";
		const string filename_out = "/Users/katarzyna/Dropbox/matlab/2015-05 PKITS/out_minNumberOFFacilities-ExtCBD-2015-05-20-EveningPeak2500m.txt";
		long int nFacilties;
		std::vector<int> list_of_facilites;

		std::vector<std::vector<long int> > aij_matrix;
		readMatrix_aij(filename, nFacilties, aij_matrix);

		std::cout << "matrix size "<< aij_matrix.size() <<" x "<< aij_matrix[0].size() << std::endl;

		// Model
		env = new GRBEnv();
		GRBModel model = GRBModel(*env);
		model.set(GRB_StringAttr_ModelName, "minimize number of facilities");

		// Facility open decision variables: open[j] == 1 if facility j is open.
		open = model.addVars(nFacilties, GRB_BINARY);
		model.update();

		for (int i = 0; i < nFacilties; ++i)
		{
			ostringstream vname;
			vname << "Open" << i;
			open[i].set(GRB_DoubleAttr_Obj, 1);
			open[i].set(GRB_StringAttr_VarName, vname.str());
		}

		// The objective is to minimize the total number of facilities
		model.set(GRB_IntAttr_ModelSense, 1);

		// Update model
		model.update();

		// Constraints

		// Demand i is covered by facility j
		for (int i = 0; i < nFacilties; ++i)
		{
			GRBLinExpr demand_is_covered = 0;
			for (int j = 0; j < nFacilties; ++j)
			{
				demand_is_covered += aij_matrix[i][j] * open[j];
			}
			ostringstream cname;
			cname << "Demand_i_is_covered " << i;
			model.addConstr(demand_is_covered >= 1, cname.str());
		}

		// Solve
		model.optimize();

		// Print solution
		cout << "\nTOTAL COSTS: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
		cout << "SOLUTION:" << endl;
		for (int i = 0; i < nFacilties; ++i)
		{
			if (open[i].get(GRB_DoubleAttr_X) == 1.0)
			{
				list_of_facilites.push_back(i);
				cout << "Facility " << i << " open." << endl;
			}
		}

		// write answer to the file (x,y coordinates of chosen facilities)
		writeAnswerToFile(filename_out, list_of_facilites);

	} catch (GRBException e)
	{
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	} catch (...)
	{
		cout << "Exception during optimization" << endl;
	}
}
void readMatrix_aij(const string filename, long int& size, std::vector<std::vector<long int> >& aij_){

	// Open the file:
	std::ifstream in(filename.c_str());
	if (!in.good()) {
		std::cout << "Cannot read: " << filename << std::endl;
	}

	while (in.good()) {

		int aij_temp;
		std::stringstream ss;
		std:: string line_;
		char buffer[256000];
		in.getline(buffer, 256000);
		if (strlen(buffer) == 0) break;

		ss << buffer;

		vector<long int> v_temp;
		while (!ss.eof()){
			ss >> line_;
			aij_temp = atoi(line_.c_str());
			v_temp.push_back(aij_temp);
		}
		aij_.push_back(v_temp);

	}
	size = aij_.size();
}

void writeAnswerToFile(const string filename_out, std::vector<int>& list_of_facilites){

	std::ofstream out(filename_out.c_str());

	if (!out.good()) {
		std::cout << "Cannot write: " << filename_out << std::endl;
	}

	for(vector<int>::const_iterator i = list_of_facilites.begin(); i != list_of_facilites.end(); ++i) {
		out << *i << '\n';
	}

}
