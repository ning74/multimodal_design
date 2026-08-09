import pandas as pd
import networkx as nx
# import osmnx as ox
import dill as pickle
# import folium
# import geopandas as gpd
import matplotlib.pyplot as plt
# import seaborn as sns
import pandas as pd
from datetime import datetime
# import swifter 
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import time
import random
from itertools import permutations
import copy
import re
import math
import ast

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "root"))

import maxRideLP
import maxRideLPRelaxed


import instance

import maxRide_generate_lines_CG_exact
import maxRide_generate_lines_CG_relaxed

from paths import output_path, save_pickle, data_path, load_pickle, output_dir


def savePickle(city, lines_mixed_relaxed, pair_lineInd_dict_mixed_relaxed, dict_stlInd_uInd_mixed_relaxed,\
            lines_mixed_exact, pair_lineInd_dict_mixed_exact, dict_stlInd_uInd_mixed_exact,\
                saved_folder, list_whichCG, list_numSol_relaxed, list_numSol_exact):

    
	save_pickle(list_whichCG, output_path(city, saved_folder, 'list_whichCG.pkl'))
	save_pickle(list_numSol_relaxed, output_path(city, saved_folder, 'list_numSol_relaxed.pkl'))
	save_pickle(list_numSol_exact, output_path(city, saved_folder, 'list_numSol_exact.pkl'))
    
    

	save_pickle(lines_mixed_relaxed, output_path(city, saved_folder, 'lines_mixed_relaxed.pkl'))
	save_pickle(pair_lineInd_dict_mixed_relaxed, output_path(city, saved_folder, 'pair_lineInd_dict_mixed_relaxed.pkl'))
	save_pickle(dict_stlInd_uInd_mixed_relaxed, output_path(city, saved_folder, 'dict_stlInd_uInd_mixed_relaxed.pkl'))


	save_pickle(lines_mixed_exact, output_path(city, saved_folder, 'lines_mixed_exact.pkl'))
	save_pickle(pair_lineInd_dict_mixed_exact, output_path(city, saved_folder, 'pair_lineInd_dict_mixed_exact.pkl'))
	save_pickle(dict_stlInd_uInd_mixed_exact, output_path(city, saved_folder, 'dict_stlInd_uInd_mixed_exact.pkl'))


def main(city, which_method, gamma, cap, budget, unit_dist, num_rounds, perc_bus_edges_selected, flex_dist, mip_gap, time_limit, num_sol, max_travel, saved_folder,\
	detour_coeff, alpha, include_naive, len_naive, cost_factor_naive, transit):
	

	output_dir(city, saved_folder)
 
 
	node_list, od_pairs, od_demands, lines, pair_lineInd_dict, dict_stlInd_uInd, ori_nodes, dest_nodes, demand_dic,\
	 cost_edges, cost_lines, cap_lines, bus_nodes, bus_edges = instance.main(city, which_method, gamma, cap, unit_dist, saved_folder, include_naive, len_naive, cost_factor_naive)

	num_bus_edges_selected = math.ceil(perc_bus_edges_selected * len(bus_edges))
		
 
	lines_mixed_relaxed = copy.deepcopy(lines)
	cost_lines_mixed_relaxed = copy.deepcopy(cost_lines)
	cap_lines_mixed_relaxed = copy.deepcopy(cap_lines)
	pair_lineInd_dict_mixed_relaxed = copy.deepcopy(pair_lineInd_dict)
	dict_stlInd_uInd_mixed_relaxed = copy.deepcopy(dict_stlInd_uInd)
	
	lines_mixed_exact = copy.deepcopy(lines)
	cost_lines_mixed_exact = copy.deepcopy(cost_lines)
	cap_lines_mixed_exact = copy.deepcopy(cap_lines)
	pair_lineInd_dict_mixed_exact = copy.deepcopy(pair_lineInd_dict)
	dict_stlInd_uInd_mixed_exact = copy.deepcopy(dict_stlInd_uInd)
	
	list_whichCG = []  
	list_numSol_relaxed = []
	list_numSol_exact = []
 
 
	# relevant quantities for running LP
	lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = maxRideLP.LPPrep(lines_mixed_exact, node_list, ori_nodes, bus_nodes, od_pairs, dict_stlInd_uInd_mixed_exact)

	# run LP only once
	m, obj, C4_toRemove, C5_toRemove, C8_toRemove, C6_toFix, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP = maxRideLP.getResult(node_list, od_pairs, ori_nodes, dest_nodes, lines_mixed_exact, lines_edge, cost_edges, cost_lines_mixed_exact, demand_dic, cap_lines_mixed_exact, budget, list_f_sluv,\
		   list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, alpha, transit)


	m.write(output_path(city, saved_folder, 'LP.mps'))
	m.write(output_path(city, saved_folder, 'LP.lp'))
 
  
	p_current = maxRideLP.getShadowPrice(m, "p")
	k_current = maxRideLP.getShadowPrice(m, "k")
	bud_current_exact = maxRideLP.getShadowPrice(m, "bud")[0]
 
 
	# relevant quantities for running LPRelaxed
	lines_edge_rela, list_f_sluv_rela, list_y_slv_rela, list_w_slv_rela, list_z_stv_rela, list_z_rela, st_dict_rela, ls_dict_rela, svl_dict_rela, z_uvs_dict_rela, y_uvl_dict_rela, all_lines_edge, dict_edge_lines\
		  = maxRideLPRelaxed.LPPrep(lines_mixed_relaxed, node_list, ori_nodes, bus_nodes, od_pairs, dict_stlInd_uInd_mixed_relaxed)
   
	# run LPRelaxed only once 
	m_rela, obj_rela, C4_toRemove_rela, C5_toRemove_rela, C6_toRemove_rela, C8_toRemove_rela, f_rela, y_rela, w_rela, z_rela, q_rela, x_rela, Y_rela =\
		  maxRideLPRelaxed.getResult(node_list, od_pairs, ori_nodes, dest_nodes, lines_mixed_relaxed, lines_edge_rela, cost_edges, cost_lines_mixed_relaxed, demand_dic, cap_lines_mixed_relaxed, budget, list_f_sluv_rela,\
		   list_y_slv_rela, list_w_slv_rela, list_z_stv_rela, list_z_rela, bus_nodes, st_dict_rela, ls_dict_rela, svl_dict_rela, z_uvs_dict_rela, y_uvl_dict_rela, all_lines_edge, dict_edge_lines, alpha, transit)

 
 
	m_rela.write(output_path(city, saved_folder, 'LP_rela.mps'))
	m_rela.write(output_path(city, saved_folder, 'LP_rela.lp'))
 
 
	r_current = maxRideLPRelaxed.getShadowPrice(m_rela, "r_relaxed")
	bud_current_relaxed = maxRideLPRelaxed.getShadowPrice(m_rela, "bud")[0]


	# for RELAXED subproblemLP
	bus_nodes_x_relaxed, bus_edges_x_relaxed = maxRide_generate_lines_CG_relaxed.subPrep(bus_nodes, bus_edges)

	# trim graph based on the current dual values
	bus_edges_selected_relaxed = set()
	bus_nodes_selected_relaxed = set()
	
	bus_edges_selected_relaxed, bus_nodes_selected_relaxed = maxRide_generate_lines_CG_relaxed.trim(r_current, bud_current_relaxed, node_list, cost_edges, cap, gamma, bus_edges, bus_nodes_x_relaxed, bus_edges_x_relaxed, mip_gap, time_limit, num_sol, max_travel, num_bus_edges_selected)
 

	# for RELAXED subproblem 
	bus_nodes_selected_x_relaxed, bus_edges_selected_x_relaxed = maxRide_generate_lines_CG_relaxed.subPrep(bus_nodes_selected_relaxed, bus_edges_selected_relaxed)

	# run RELAXED subproblem
	log_directory = output_path(city, saved_folder, 'generate_lines_CG_relaxed_log.txt')
	mSubprob_relaxed, subObj_relaxed, x_relaxed, numVars_relaxed = maxRide_generate_lines_CG_relaxed.subproblem(r_current, bud_current_relaxed, bus_nodes_selected_relaxed, cost_edges, cap, gamma, bus_edges_selected_relaxed, bus_nodes_selected_x_relaxed, bus_edges_selected_x_relaxed, mip_gap, time_limit, num_sol, max_travel, detour_coeff, log_directory)
	

 
	# for EXACT subproblemLP
	bus_nodes_x_exact, bus_edges_x_exact, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, sBs_list, list_p_sv = maxRide_generate_lines_CG_exact.subPrep(bus_nodes, bus_edges, ori_nodes, flex_dist, cost_edges, od_pairs)
	
	# trim graph based on the current dual values 
	bus_edges_selected_exact = set()
	bus_nodes_selected_exact = set()
 

	bus_edges_selected_exact, bus_nodes_selected_exact =  maxRide_generate_lines_CG_exact.trim(k_current, p_current, bud_current_exact, cap, gamma, bus_nodes, bus_nodes_x_exact, ori_nodes, cost_edges, bus_edges, bus_edges_x_exact,\
							   sBs_list, list_p_sv, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, num_bus_edges_selected, max_travel)


 
	# for subproblem
	bus_nodes_selected_x_exact, bus_edges_selected_x_exact, dict_bus_to_selected, dict_bus_from_selected,\
	 dict_bus_to_selected_x, dict_bus_from_selected_x, sBs_list_selected, list_p_sv_selected\
		  = maxRide_generate_lines_CG_exact.subPrep(bus_nodes_selected_exact, bus_edges_selected_exact, ori_nodes, flex_dist, cost_edges, od_pairs)
	
 
	 # run subproblem based on the trimmed graph
	log_directory = output_path(city, saved_folder, 'generate_lines_CG_exact_log.txt')
	mSubprob_exact, subObj_exact, h_exact, numVars_exact = maxRide_generate_lines_CG_exact.subproblem(k_current, p_current, bud_current_exact, cap, gamma, bus_nodes_selected_exact, bus_nodes_selected_x_exact, ori_nodes, cost_edges, bus_edges_selected_exact, bus_edges_selected_x_exact,\
					   sBs_list_selected, list_p_sv_selected, dict_bus_to_selected, dict_bus_from_selected, dict_bus_to_selected_x, dict_bus_from_selected_x, mip_gap, time_limit, num_sol, max_travel, detour_coeff, log_directory)
	
	# haven't add new lines to lines_mixed yet
	savePickle(city, lines_mixed_relaxed, pair_lineInd_dict_mixed_relaxed, dict_stlInd_uInd_mixed_relaxed,\
     			lines_mixed_exact, pair_lineInd_dict_mixed_exact, dict_stlInd_uInd_mixed_exact,\
     			saved_folder, list_whichCG, list_numSol_relaxed, list_numSol_exact)
	
	counter_outer = 0
	continue_flag = True
 
	if len(bus_edges_selected_relaxed) == 0 and len(bus_edges_selected_exact) == 0:
		continue_flag = False

	while continue_flag == True and counter_outer < num_rounds:
  
		if counter_outer < num_rounds and subObj_relaxed > 0 and len(bus_edges_selected_relaxed) > 0:
			counter_outer += 1
			list_whichCG.append('relaxed')
   
			nSolutions = mSubprob_relaxed.SolCount
   
			list_numSol_relaxed.append(min(num_sol, nSolutions))

			for i in range(num_sol):
				if nSolutions > i:
					x_edge = []
					mSubprob_relaxed.setParam(GRB.Param.SolutionNumber, i)
	 
					if mSubprob_relaxed.PoolObjVal <= 0:
						break
  
					for item in bus_edges_selected_x_relaxed:
						if x_relaxed[item].Xn > 0.999:
							x_edge.append(item)
	   
					tour, cycles_list, irrelevant_edges = maxRide_generate_lines_CG_relaxed.oneSol(x_edge)
					detour_flag = maxRide_generate_lines_CG_relaxed.check_detour(tour, cost_edges, detour_coeff)
	 
					if tour not in lines_mixed_relaxed and len(cycles_list) <= 0 and len(irrelevant_edges) <= 0 and detour_flag == True:
						# create variables needed for running LPRelaxed after generating the new line
						lines_mixed_relaxed, cost_lines_mixed_relaxed, cap_lines_mixed_relaxed, pair_lineInd_dict_mixed_relaxed, dict_stlInd_uInd_mixed_relaxed =\
								  maxRideLPRelaxed.newLineVars(tour, lines_mixed_relaxed, cost_lines_mixed_relaxed, cap_lines_mixed_relaxed, pair_lineInd_dict_mixed_relaxed, dict_stlInd_uInd_mixed_relaxed, gamma, cap, cost_edges, node_list, flex_dist, od_pairs, unit_dist)
				

      
      
						# re-generate LPRelaxed (add additional vars and constraints)
						m_rela, C4_toRemove_rela, C5_toRemove_rela, C6_toRemove_rela, C8_toRemove_rela, f_rela, y_rela, w_rela, z_rela, q_rela, x_rela, Y_rela, lines_edge_rela, list_f_sluv_rela, list_y_slv_rela, list_w_slv_rela, list_z_stv_rela, list_z_rela\
						  , st_dict_rela, ls_dict_rela, svl_dict_rela, z_uvs_dict_rela, y_uvl_dict_rela = maxRideLPRelaxed.newLineLP(m_rela, C4_toRemove_rela, C5_toRemove_rela, C6_toRemove_rela, C8_toRemove_rela, cap, budget, od_pairs, node_list, lines_mixed_relaxed,\
									  lines_edge_rela, ori_nodes, cost_edges, tour, cost_lines_mixed_relaxed, cap_lines_mixed_relaxed, dict_stlInd_uInd_mixed_relaxed, list_f_sluv_rela, list_y_slv_rela, list_w_slv_rela, list_z_stv_rela, list_z_rela\
									  , bus_nodes, st_dict_rela, ls_dict_rela, svl_dict_rela, z_uvs_dict_rela, y_uvl_dict_rela, f_rela, y_rela, w_rela, z_rela, q_rela, x_rela, Y_rela, alpha)

	  
						# create variables needed for running LP after generating the new line
						lines_mixed_exact, cost_lines_mixed_exact, cap_lines_mixed_exact, pair_lineInd_dict_mixed_exact, dict_stlInd_uInd_mixed_exact = maxRideLP.newLineVars(tour, lines_mixed_exact, cost_lines_mixed_exact, cap_lines_mixed_exact, pair_lineInd_dict_mixed_exact, dict_stlInd_uInd_mixed_exact, gamma, cap, cost_edges, node_list, flex_dist, od_pairs, unit_dist)

					
						# re-generate LP (add additional vars and constraints)
						m, C4_toRemove, C5_toRemove, C8_toRemove, C6_toFix, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP, lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z\
						  , st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = maxRideLP.newLineLP(m, C4_toRemove, C5_toRemove, C8_toRemove, C6_toFix, cap, budget, od_pairs, node_list, lines_mixed_exact, lines_edge, ori_nodes, cost_edges, tour, cost_lines_mixed_exact, cap_lines_mixed_exact, dict_stlInd_uInd_mixed_exact, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z\
						  , bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP, alpha)
		
					

			# resolve LPRelaxed
			m_rela.optimize()
			r_current = maxRideLPRelaxed.getShadowPrice(m_rela, 'r_relaxed')
   
			bus_nodes = load_pickle(data_path(city, 'bus_nodes_dense.pkl'))
  
			bus_edges = load_pickle(data_path(city, 'bus_edges_1.25mile_dense.pkl'))

	
			# for RELAXED subproblemLP
			bus_nodes_x_relaxed, bus_edges_x_relaxed = maxRide_generate_lines_CG_relaxed.subPrep(bus_nodes, bus_edges)
   
			# trim graph based on the current dual values
			bus_edges_selected_relaxed = set()
			bus_nodes_selected_relaxed = set()
   
			bus_edges_selected_relaxed, bus_nodes_selected_relaxed = maxRide_generate_lines_CG_relaxed.trim(r_current, bud_current_relaxed, node_list, cost_edges, cap, gamma, bus_edges, bus_nodes_x_relaxed, bus_edges_x_relaxed, mip_gap, time_limit, num_sol, max_travel, num_bus_edges_selected)
   
   
			if len(bus_edges_selected_exact) > 0:
				# for subproblem
				bus_nodes_selected_x_relaxed, bus_edges_selected_x_relaxed = maxRide_generate_lines_CG_relaxed.subPrep(bus_nodes_selected_relaxed, bus_edges_selected_relaxed)
	
				# run subproblem based on the trimmed graph
				log_directory = output_path(city, saved_folder, 'generate_lines_CG_relaxed_log.txt')
				mSubprob_relaxed, subObj_relaxed, x_relaxed, numVars_relaxed = maxRide_generate_lines_CG_relaxed.subproblem(r_current, bud_current_relaxed, bus_nodes_selected_relaxed, cost_edges, cap, gamma, bus_edges_selected_relaxed, bus_nodes_selected_x_relaxed, bus_edges_selected_x_relaxed, mip_gap, time_limit, num_sol, max_travel, detour_coeff, log_directory)
		
				
	
		if counter_outer < num_rounds and subObj_exact > 0 and len(bus_edges_selected_exact) > 0:
			counter_outer += 1
			list_whichCG.append('exact')
   
			nSolutions = mSubprob_exact.SolCount
   
			list_numSol_exact.append(min(num_sol, nSolutions))
   
			for i in range(num_sol):
				if (nSolutions > i):
					h_edge = []
					mSubprob_exact.setParam(GRB.Param.SolutionNumber, i)
	 
					if mSubprob_exact.PoolObjVal <= 0:
						break
  
					for item in bus_edges_selected_x_exact:
						if h_exact[item].Xn > 0.999:
							h_edge.append(item)
	   
					tour, cycles_list, irrelevant_edges = maxRide_generate_lines_CG_exact.oneSol(h_edge)
					detour_flag = maxRide_generate_lines_CG_exact.check_detour(tour, cost_edges, detour_coeff)
					
	 
					if tour not in lines_mixed_exact and len(cycles_list) <= 0 and len(irrelevant_edges) <= 0 and detour_flag == True:
						# create variables needed for running LP after generating the new line 
						lines_mixed_exact, cost_lines_mixed_exact, cap_lines_mixed_exact, pair_lineInd_dict_mixed_exact, dict_stlInd_uInd_mixed_exact = maxRideLP.newLineVars(tour, lines_mixed_exact, cost_lines_mixed_exact, cap_lines_mixed_exact, pair_lineInd_dict_mixed_exact, dict_stlInd_uInd_mixed_exact, gamma, cap, cost_edges, node_list, flex_dist, od_pairs, unit_dist)

						# re-generate LP (add additional vars and constraints)
						m, C4_toRemove, C5_toRemove, C8_toRemove, C6_toFix, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP, lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z\
						  , st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = maxRideLP.newLineLP(m, C4_toRemove, C5_toRemove, C8_toRemove, C6_toFix, cap, budget, od_pairs, node_list, lines_mixed_exact, lines_edge, ori_nodes, cost_edges, tour, cost_lines_mixed_exact, cap_lines_mixed_exact, dict_stlInd_uInd_mixed_exact, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z\
						  , bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP, alpha)
		

						# create variables needed for running LPRelaxed after generating the new line
						lines_mixed_relaxed, cost_lines_mixed_relaxed, cap_lines_mixed_relaxed, pair_lineInd_dict_mixed_relaxed, dict_stlInd_uInd_mixed_relaxed =\
								  maxRideLPRelaxed.newLineVars(tour, lines_mixed_relaxed, cost_lines_mixed_relaxed, cap_lines_mixed_relaxed, pair_lineInd_dict_mixed_relaxed, dict_stlInd_uInd_mixed_relaxed, gamma, cap, cost_edges, node_list, flex_dist, od_pairs, unit_dist)
				
	  
						# re-generate LPRelaxed (add additional vars and constraints)
						m_rela, C4_toRemove_rela, C5_toRemove_rela, C6_toRemove_rela, C8_toRemove_rela, f_rela, y_rela, w_rela, z_rela, q_rela, x_rela, Y_rela, lines_edge_rela, list_f_sluv_rela, list_y_slv_rela, list_w_slv_rela, list_z_stv_rela, list_z_rela\
						  , st_dict_rela, ls_dict_rela, svl_dict_rela, z_uvs_dict_rela, y_uvl_dict_rela = maxRideLPRelaxed.newLineLP(m_rela, C4_toRemove_rela, C5_toRemove_rela, C6_toRemove_rela, C8_toRemove_rela, cap, budget, od_pairs, node_list, lines_mixed_relaxed,\
									  lines_edge_rela, ori_nodes, cost_edges, tour, cost_lines_mixed_relaxed, cap_lines_mixed_relaxed, dict_stlInd_uInd_mixed_relaxed, list_f_sluv_rela, list_y_slv_rela, list_w_slv_rela, list_z_stv_rela, list_z_rela\
									  , bus_nodes, st_dict_rela, ls_dict_rela, svl_dict_rela, z_uvs_dict_rela, y_uvl_dict_rela, f_rela, y_rela, w_rela, z_rela, q_rela, x_rela, Y_rela, alpha)

				
	  
	  
			# resolve LP
			m.optimize()
	
			p_current = maxRideLP.getShadowPrice(m, "p")
			k_current = maxRideLP.getShadowPrice(m, "k")
			bud_current_exact = maxRideLP.getShadowPrice(m, "bud")[0]
   
			bus_nodes = load_pickle(data_path(city, 'bus_nodes_dense.pkl'))
    
			bus_edges = load_pickle(data_path(city, 'bus_edges_1.25mile_dense.pkl'))
	
 
			# for EXACT subproblemLP
			bus_nodes_x_exact, bus_edges_x_exact, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, sBs_list, list_p_sv = maxRide_generate_lines_CG_exact.subPrep(bus_nodes, bus_edges, ori_nodes, flex_dist, cost_edges, od_pairs)
			
			# trim graph based on the current dual values 
			bus_edges_selected_exact = set()
			bus_nodes_selected_exact = set()
		
		
			bus_edges_selected_exact, bus_nodes_selected_exact =  maxRide_generate_lines_CG_exact.trim(k_current, p_current, bud_current_exact, cap, gamma, bus_nodes, bus_nodes_x_exact, ori_nodes, cost_edges, bus_edges, bus_edges_x_exact,\
									sBs_list, list_p_sv, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, num_bus_edges_selected, max_travel)

   
			if len(bus_edges_selected_exact) > 0:
				# for subproblem
				bus_nodes_selected_x_exact, bus_edges_selected_x_exact, dict_bus_to_selected, dict_bus_from_selected,\
				  dict_bus_to_selected_x, dict_bus_from_selected_x, sBs_list_selected, list_p_sv_selected\
					  = maxRide_generate_lines_CG_exact.subPrep(bus_nodes_selected_exact, bus_edges_selected_exact, ori_nodes, flex_dist, cost_edges, od_pairs)

				# run subproblem based on the trimmed graph
				log_directory = output_path(city, saved_folder, 'generate_lines_CG_exact_log.txt')
				mSubprob_exact, subObj_exact, h_exact, numVars_exact = maxRide_generate_lines_CG_exact.subproblem(k_current, p_current, bud_current_exact, cap, gamma, bus_nodes_selected_exact, bus_nodes_selected_x_exact, ori_nodes, cost_edges, bus_edges_selected_exact, bus_edges_selected_x_exact, sBs_list_selected,\
					 list_p_sv_selected, dict_bus_to_selected, dict_bus_from_selected, dict_bus_to_selected_x, dict_bus_from_selected_x, mip_gap, time_limit, num_sol, max_travel, detour_coeff, log_directory)
			

				
		savePickle(city, lines_mixed_relaxed, pair_lineInd_dict_mixed_relaxed, dict_stlInd_uInd_mixed_relaxed,\
     			lines_mixed_exact, pair_lineInd_dict_mixed_exact, dict_stlInd_uInd_mixed_exact,\
     			saved_folder, list_whichCG, list_numSol_relaxed, list_numSol_exact)
	
  
		if len(bus_edges_selected_relaxed) == 0 and len(bus_edges_selected_exact) == 0:
			continue_flag = False
		if subObj_relaxed <= 0 and subObj_exact <= 0:
			continue_flag = False
  
  
	return lines_mixed_relaxed, pair_lineInd_dict_mixed_relaxed, dict_stlInd_uInd_mixed_relaxed,\
	lines_mixed_exact, pair_lineInd_dict_mixed_exact, dict_stlInd_uInd_mixed_exact, \
	 list_whichCG
