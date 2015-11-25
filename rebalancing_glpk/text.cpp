// /*
// * text.cpp
// *
// *  Created on: Jun 9, 2015
// *      Author: katarzyna
// */
//
// amod::ReturnCode ManagerMatchRebalance::solveRebalancing(amod::World *world_state) {
//    	if (!world_state) {
//    		throw std::runtime_error("solveMatching: world_state is nullptr!");
//    	}
//
//    	if (available_vehs_.size() == 0) return amod::SUCCESS; // no vehicles to rebalance
//
//        if (stations_.size() == 0) return amod::SUCCESS; // nothing to rebalance
//
//    	// create variables for solving lp
//    	int nvehs = available_vehs_.size();
//    	int nstations = stations_.size();
//        int nstations_underserved = 0;
//    	int nvars = nstations*nstations; // how many to send from one station to another
//
//        int vi_total = available_vehs_.size();
//        int cex_total = 0;
//        std::unordered_map<int, int> cex;
//        std::unordered_map<int, std::set<int>> vi; // free vehicles at this station
//
//    	// set up the problem
//        glp_prob *lp;
//        lp = glp_create_prob();
//        glp_set_prob_name(lp, "rebalancing");
//        glp_set_obj_dir(lp, GLP_MIN);
//        glp_add_cols(lp, nvars);
//
//
//        // add the structural variables (decision variables)
//		std::unordered_map<int, std::pair<int,int>> index_to_ids;
//        std::unordered_map<std::pair<int,int>, int> ids_to_index;
//
//		int k = 1;
//		for (auto sitr = stations_.begin(); sitr != stations_.end(); ++sitr){
//			for (auto sitr2 = stations_.begin(); sitr2 != stations_.end(); ++sitr2) {
//
//				// store the indices to make lookups easier (can be optimized in future iterations)
//				index_to_ids[k] = {sitr->first, sitr2->first};
//                ids_to_index[std::make_pair(sitr->first, sitr2->first)] = k;
//
//				// get cost
//				double cost = sim_->getDrivingDistance(sitr->second.getPosition(),
//						sitr2->second.getPosition() );
//
//				// add this variable to the solver
//				std::stringstream ss;
//				ss << "x " << sitr->first << " " << sitr2->first;
//				const std::string& tmp = ss.str();
//				const char* cstr = tmp.c_str();
//				glp_set_col_name(lp, k, cstr);
//
//				glp_set_col_bnds(lp, k, GLP_LO, 0.0, 0.0); // set lower bound of zero, no upperbound
//				glp_set_obj_coef(lp, k, cost);
//
//				// increment index
//				++k;
//			}
//
//            // compute variables for the lp
//
//			// use current demand
//			// int cexi = sitr->second.getNumCustomers() - sitr->second.getNumVehicles();
//
//			// use predicted demand
//			int mean_pred = ceil(dem_est_->predict(sitr->second.getId(), *world_state, world_state->getCurrentTime()).first);
//			/*int mean_pred = ceil(std::max(
//					(double) dem_est_->predict(sitr->second.getId(), *world_state, world_state->getCurrentTime()).first,
//					(double) sitr->second.getNumCustomers()));
//			*/
//            std::cout << "Mean prediction: " << mean_pred;
//			int cexi = mean_pred - sitr->second.getNumVehicles();
//            std::cout << "cexi: " << cexi;
//            std::cout << "vehs: " << sitr->second.getNumVehicles();
//
//			cex[sitr->first] = cexi; // excess customers at this station
//            cex_total += cexi; // total number of excess customers
//
//            if (cexi > 0) {
//                nstations_underserved++;
//            }
//
//            std::cout << "cex[" << sitr->first << "]: " << cex[sitr->first] << std::endl;
//
//		}
//
//
//        // set up available vehicles at each station
//        for (auto vitr = available_vehs_.begin(); vitr != available_vehs_.end(); ++vitr) {
//            // get which station this vehicle belongs
//            int sid = veh_id_to_station_id_[*vitr];
//            vi[sid].insert(*vitr);
//        }
//
//
//		// set up constraints
//        int *ia;
//        int  *ja; // +1 because glpk starts indexing at 1 (why? I don't know)
//        double *ar;
//        if (cex_total <= 0) {
//            // should be possible to satisfy all customers by rebalancing
//            int ncons = nstations*2;
//            int nelems = nstations*((nstations - 1)*2) + nstations*(nstations-1);
//            ia = new int[nelems + 1];
//            ja = new int[nelems + 1]; // +1 because glpk starts indexing at 1 (why? I don't know)
//            ar = new double[nelems + 1];
//
//            glp_add_rows(lp, ncons);
//            int k = 1;
//            int i = 1;
//
//            // constraint for net flow to match (or exceed) excess customers
//            for (auto sitr = stations_.begin(); sitr!= stations_.end(); ++sitr) {
//                std::stringstream ss;
//                ss << "st " << sitr->second.getId();
//                const std::string& tmp = ss.str();
//                const char* cstr = tmp.c_str();
//                glp_set_row_name(lp, i, cstr);
//                glp_set_row_bnds(lp, i, GLP_LO, cex[sitr->second.getId()], 0.0);
//
//
//                for (auto sitr2 = stations_.begin(); sitr2 != stations_.end(); ++sitr2) {
//                    if (sitr2->first == sitr->first) continue;
//                    // from i to j
//                    ia[k] = i;
//                    int st_source = sitr->second.getId();
//                    int st_dest   = sitr2->second.getId();
//                    ja[k] = ids_to_index[{st_source, st_dest}];
//                    ar[k] = -1.0;
//                    ++k;
//
//                    // from j to i
//                    ia[k] = i;
//                    ja[k] = ids_to_index[{st_dest, st_source}];
//                    ar[k] = 1.0;
//                    ++k;
//                }
//                ++i; // increment i
//            }
//
//            // constraint to make sure stations don't send more vehicles than they have
//            for (auto sitr = stations_.begin(); sitr!= stations_.end(); ++sitr) {
//                std::stringstream ss;
//                ss << "st " << sitr->second.getId() << " veh constraint";
//                const std::string& tmp = ss.str();
//                const char* cstr = tmp.c_str();
//                glp_set_row_name(lp, i, cstr);
//                //std::cout << "vi[" << sitr->first << "]: " <<  vi[sitr->second.getId()].size() << std::endl;
//                glp_set_row_bnds(lp, i, GLP_UP, 0.0, vi[sitr->second.getId()].size());
//
//                for (auto sitr2 = stations_.begin(); sitr2 != stations_.end(); ++sitr2) {
//                    if (sitr2->first == sitr->first) continue;
//
//                    // from i to j
//                    ia[k] = i;
//                    int st_source = sitr->second.getId();
//                    int st_dest   = sitr2->second.getId();
//                    ja[k] = ids_to_index[{st_source, st_dest}];
//                    ar[k] = 1.0;
//                    ++k;
//                }
//                ++i; // increment i
//            }
//
//            glp_load_matrix(lp, nelems, ia, ja, ar);
//
//        } else {
//            // cannot satisfy all customers, rebalance to obtain even distribution
//
//            // should be possible to satisfy all customers by rebalancing
//            int ncons = nstations*3;
//            int nelems = nstations*((nstations-1)*2) + 2*nstations*(nstations-1) ;
//            ia = new int[nelems + 1];
//            ja = new int[nelems + 1]; // +1 because glpk starts indexing at 1 (why? I don't know)
//            ar = new double[nelems + 1];
//
//            glp_add_rows(lp, ncons);
//            int k = 1;
//            int i = 1;
//
//            // std::cout << "Even distribution: " <<  floor(vi_total/nstations_underserved) << std::endl;
//            // constraint for net flow to match (or exceed) excess customers
//            for (auto sitr = stations_.begin(); sitr!= stations_.end(); ++sitr) {
//                std::stringstream ss;
//                ss << "st " << sitr->second.getId();
//                const std::string& tmp = ss.str();
//                const char* cstr = tmp.c_str();
//                glp_set_row_name(lp, i, cstr);
//                glp_set_row_bnds(lp, i, GLP_LO,
//                                 std::min((double) cex[sitr->second.getId()] ,
//                                          (double) floor(vi_total/nstations_underserved)), 0.0);
//
//
//                for (auto sitr2 = stations_.begin(); sitr2 != stations_.end(); ++sitr2) {
//                    if (sitr2->first == sitr->first) continue;
//
//                    // from i to j
//                    ia[k] = i;
//                    int st_source = sitr->second.getId();
//                    int st_dest   = sitr2->second.getId();
//                    ja[k] = ids_to_index[{st_source, st_dest}];
//                    ar[k] =  -1.0;
//                    ++k;
//
//                    // from j to i
//                    ia[k] = i;
//                    ja[k] = ids_to_index[{st_dest, st_source}];
//                    ar[k] =  1.0;
//                    ++k;
//                }
//                ++i; // increment i
//            }
//
//            // constraint to make sure stations don't send more vehicles than they have
//            for (auto sitr = stations_.begin(); sitr!= stations_.end(); ++sitr) {
//                std::stringstream ss;
//                ss << "st " << sitr->second.getId() << " veh constraint";
//                const std::string& tmp = ss.str();
//                const char* cstr = tmp.c_str();
//                glp_set_row_name(lp, i, cstr);
//                glp_set_row_bnds(lp, i, GLP_UP, 0.0, vi[sitr->second.getId()].size());
//
//                for (auto sitr2 = stations_.begin(); sitr2 != stations_.end(); ++sitr2) {
//                    // from i to j
//                    if (sitr2->first == sitr->first) continue;
//
//                    ia[k] = i;
//                    int st_source = sitr->second.getId();
//                    int st_dest   = sitr2->second.getId();
//                    ja[k] = ids_to_index[{st_source, st_dest}];
//                    ar[k] =  1.0;
//                    ++k;
//                }
//                ++i; // increment i
//            }
//
//            // constraint for stations to send as many vehicles as possible
//            for (auto sitr = stations_.begin(); sitr!= stations_.end(); ++sitr) {
//                std::stringstream ss;
//                ss << "st " << sitr->second.getId() << " send all constraint";
//                const std::string& tmp = ss.str();
//                const char* cstr = tmp.c_str();
//                glp_set_row_name(lp, i, cstr);
//                double constr = std::min( (double) vi[sitr->first].size(), (double) std::max(0, -cex[sitr->first] ));
//                glp_set_row_bnds(lp, i, GLP_LO, constr, 0.0);
//
//                for (auto sitr2 = stations_.begin(); sitr2 != stations_.end(); ++sitr2) {
//                    if (sitr2->first == sitr->first) continue;
//
//                    // from i to j
//                    ia[k] = i;
//                    int st_source = sitr->second.getId();
//                    int st_dest   = sitr2->second.getId();
//                    ja[k] = ids_to_index[{st_source, st_dest}];
//                    ar[k] =  1.0;
//                    ++k;
//                }
//                ++i; // increment i
//            }
//
//            glp_load_matrix(lp, nelems, ia, ja, ar);
//        }
//
//        // solve the lp
//        // glp_term_out(GLP_OFF); // suppress terminal output
//        glp_simplex(lp, nullptr);
//
//
//        // redispatch based on lp solution
//        for (int k=1; k<=nvars; k++) {
//            // get the value
//            int to_dispatch = floor(glp_get_col_prim(lp,k));
//            //std::cout << k << ": " << to_dispatch << std::endl;
//            if (to_dispatch > 0) {
//                int st_source = index_to_ids[k].first;
//                int st_dest = index_to_ids[k].second;
//
//                // dispatch to_dispatch vehicles form station st_source to st_dest
//                amod::ReturnCode rc = interStationDispatch(st_source, st_dest, to_dispatch, world_state, vi);
//                if (rc != amod::SUCCESS) {
//                    std::cout << amod::kErrorStrings[rc] << std::endl;
//
//                    // be stringent and throw an exception: this shouldn't happen
//                    throw std::runtime_error("solveRebalancing: interStationDispatch failed.");
//                }
//            }
//        }
//
//        // housekeeping
//        glp_delete_prob(lp);
//        glp_free_env();
//
//        delete [] ia;
//        delete [] ja;
//        delete [] ar;
//
//    	return amod::SUCCESS;
//    }


