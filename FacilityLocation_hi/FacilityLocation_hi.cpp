/*
 * FacilityLocation_hi.cpp
 *
 *  Created on: May 18, 2015
 *      Author: katarzyna
 */

#include <sstream>
#include <iostream>
#include "gurobi_c++.h"
using namespace std;

void readMatrix_aij(const string filename, std::vector<std::vector<long int> >& matrix);
void readVector_hi (const string filename_hi, std::vector<int>& occurences);
void writeAnswerToFile(const string filename_out, std::vector<int>& list_of_facilites);

int main(int argc, char *argv[])
{
	GRBEnv* env = 0;
	GRBVar* covered = 0; // we want to maximize demand covered
	GRBVar* facility = 0;
	int max_n_Facilities = 60;

	try
	{
		const string filename = "/Users/katarzyna/Dropbox/matlab/2015-07_FMWorkshop/facility_location/outputs/aij_matrix_ecbd-2015-06-12-S1000m.txt";
		const string filename_hi = "/Users/katarzyna/Dropbox/matlab/2015-07_FMWorkshop/facility_location/outputs/hi_of_bins-2015-06-12.txt";
		const string filename_out = "/Users/katarzyna/Dropbox/matlab/2015-07_FMWorkshop/facility_location/outputs/out_eclipse/out_facilitiesIDs_ecbd_S1000_2015-06-12.txt";
		std::vector<int> list_of_facilites;

		std::vector<std::vector<long int> > aij_matrix;
		readMatrix_aij(filename, aij_matrix);
		std::cout << "matrix size "<< aij_matrix.size() <<" x "<< aij_matrix[0].size() << std::endl;

		vector<int> occurences;
		readVector_hi(filename_hi, occurences);
		std::cout << "vector size "<< occurences.size() << std::endl;

		// Model
		env = new GRBEnv();
		GRBModel model = GRBModel(*env);
		model.set(GRB_StringAttr_ModelName, "minimize number of facilities");

		// Demand coverage decision variable, covered[i] == 1 if demand is covered
		covered = model.addVars(occurences.size(), GRB_BINARY);
		model.update();

		// Facility open variables: facility[j] == 1 if facility j is open.
		facility = model.addVars(occurences.size(), GRB_BINARY);
		model.update();


		// Set the objective to maximize the demand covered
		for (unsigned int i = 0; i < occurences.size(); ++i)
		{
			ostringstream vname;
			vname << "Demand_" << i;
			covered[i].set(GRB_DoubleAttr_Obj, occurences[i]);
			covered[i].set(GRB_StringAttr_VarName, vname.str());
		}

		for (unsigned int i = 0; i < aij_matrix.size(); ++i)
		{
			ostringstream vname;
			vname << "Open_" << i;
			facility[i].set(GRB_DoubleAttr_Obj, 0);
			facility[i].set(GRB_StringAttr_VarName, vname.str());
		}

		// The objective is to maximize the total demand covered
		model.set(GRB_IntAttr_ModelSense, -1);

		// Update model
		model.update();

		// Constraints

		// If demand i is covered by at least one station, then covered[i] == 1
		for (unsigned int i = 0; i < aij_matrix.size(); ++i)
		{
			GRBLinExpr demand_is_covered = 0;
			for (unsigned int j = 0; j < aij_matrix.size(); ++j)
			{
				demand_is_covered += aij_matrix[i][j] * facility[j];
			}
			ostringstream cname;
			cname << "Demand_i_is_covered_" << i;
			model.addConstr(demand_is_covered >= covered[i], cname.str());
		}

		// We allow no more than n facilities
		GRBLinExpr facilities_to_open = 0;
		for (unsigned int j = 0; j < aij_matrix.size(); ++j) {
			facilities_to_open += facility[j];
		}

		ostringstream vname;
		vname << "Max facility... ";
		model.addConstr(facilities_to_open <= max_n_Facilities);

		model.update();

		// Solve
		model.optimize();

		//model.write("/Users/katarzyna/Dropbox/matlab/2015-05 PKITS/output.lp");

		// Print solution
		cout << "\nTOTAL COSTS: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
		cout << "SOLUTION:" << endl;
		int sum_d = 0;
		for (unsigned int i = 0; i < aij_matrix.size(); ++i)
		{
			if (covered[i].get(GRB_DoubleAttr_X) == 1.0)
			{
				//cout << "Demand " << i << " covered." << endl;

				sum_d += occurences[i];
			}
		}

		int sum_f = 0;
		for (unsigned int i = 0; i < aij_matrix.size(); ++i)
		{
			if (facility[i].get(GRB_DoubleAttr_X) == 1.0)
			{
				cout << "Facility " << i << " is opened." << endl;
				list_of_facilites.push_back(i);
				sum_f += 1;
			}
		}
		cout << sum_f << " facilities is open." << endl;
		cout << sum_d << " customers is covered." << endl;

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
void readMatrix_aij(const string filename, std::vector<std::vector<long int> >& aij_){

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
}

void readVector_hi (const string filename_hi, std::vector<int>& occurences){

	// Open the file:
	std::ifstream in(filename_hi.c_str());
	if (!in.good()) {
		std::cout << "Cannot read: " << filename_hi << std::endl;
	}

	while (in.good()) {

		int hi_temp;
		std::stringstream ss;
		std:: string line_;
		char buffer[256];
		in.getline(buffer, 256);
		if (strlen(buffer) == 0) break;

		ss << buffer;

		while (!ss.eof()){
			ss >> line_;
			hi_temp = atoi(line_.c_str());
			occurences.push_back(hi_temp);
		}
	}
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
