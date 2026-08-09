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

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "root"))

import maxRideLP
import maxRideLPRelaxed
import instance

from paths import output_path, save_pickle, data_path, load_pickle, output_dir


def check_detour(line, cost_edges, detour_coeff):
	first_half_line = line[:int((len(line)+1)/2)]

	second_half_line = line[int((len(line)+1)/2)-1:]

	line = first_half_line
  
	# matches the index in lazy constraint
	line = [-1] + line + [-2]
	
	i = 1
	j = len(line)-3
		
	cost_i = 0
	for k in range(1,len(line)-2): 
		cost_i += cost_edges[line[k], line[k+1]]
	
	
	if cost_i > detour_coeff * cost_edges[line[i], line[j+1]]:
		return False
		
	return True


def lazyconstrs(model, where):
	if where == GRB.Callback.MIPSOL:
		# make a list of edges selected in the solution
		h_vals = model.cbGetSolution(model._h)
		selected = list(edge for edge in model._h.keys() if h_vals[edge] > 0.999)
		# find cycles in the selected edge list

		helper_G = nx.DiGraph(selected)
		path_generated = nx.bidirectional_shortest_path(helper_G, -1, -2)

		cycles_list = sorted(nx.simple_cycles(helper_G)) 

		costEdges_vals = model.cbGetSolution(model._costEdges)
		multiple_vals = model.cbGetSolution(model._multiple)

		# subtour elimination 
		if len(cycles_list) > 0:
			for item in cycles_list:
				model.cbLazy(gp.quicksum(model._h[i, j] for i in item for j in item if (i,j) in model._h.keys())\
										 <= len(item)-1)

		else:	
			i = 1
			j = len(path_generated)-3
	
			cost_i = 0
			cost_i_backward = 0
			for k in range(1,len(path_generated)-2): 
				cost_i += costEdges_vals[path_generated[k], path_generated[k+1]]
				cost_i_backward += costEdges_vals[path_generated[k+1], path_generated[k]]
    
			if cost_i > multiple_vals * costEdges_vals[path_generated[i], path_generated[j+1]] or cost_i_backward > multiple_vals * costEdges_vals[path_generated[j+1], path_generated[i]]:
				model.cbLazy(gp.quicksum(model._h[path_generated[index], path_generated[index+1]] for index in range(i, j+1)) <= j-i)
			


				# add backward path as well
				model.cbLazy(gp.quicksum(model._h[path_generated[index+1], path_generated[index]] for index in range(i, j+1)) <= j-i)
					





def subproblem(r_current, bud_current, node_list, cost_edges, cap, gamma, edge_list, node_list_x, edge_list_x, mip_gap, time_limit, num_sol, max_travel, detour_coeff, log_directory):
	mSubprob = gp.Model('modelSubprob')
	print('run subproblem exact~')
	mSubprob.Params.outputFlag = 0
	mSubprob.Params.lazyConstraints = 1
	mSubprob.Params.MIPGap = mip_gap
	mSubprob.Params.timeLimit = time_limit
	mSubprob.Params.poolSearchMode = 2
	mSubprob.Params.poolSolutions = num_sol
 
	mSubprob.Params.MIPFocus = 1
 
	mSubprob.Params.LogFile = log_directory
 
	
	h = mSubprob.addVars(edge_list_x, lb = 0, ub = 1, vtype = GRB.BINARY, name = 'h')

	mSubprob._h = h
 
	costEdges_var = mSubprob.addVars(list(cost_edges), lb = 0, vtype = GRB.CONTINUOUS)
	for key, value in cost_edges.items():
		costEdges_var[key].lb = value
		costEdges_var[key].ub = value
  
	multiple_var = mSubprob.addVar(lb = detour_coeff, ub = detour_coeff)
	
	mSubprob._costEdges = costEdges_var
	mSubprob._multiple = multiple_var


	obj_func = 0	
	for item in edge_list:
		obj_func += -(bud_current*gamma*cost_edges[item]-cap*r_current[item] + bud_current*gamma*cost_edges[item[1],item[0]]-cap*r_current[item[1],item[0]]) * h[item]




	mSubprob.setObjective(obj_func, GRB.MAXIMIZE)

	
	# constraints
	mSubprob.addConstr(gp.quicksum(h[-1,u] for u in node_list if (-1,u) in edge_list_x)\
		- gp.quicksum(h[u,-1] for u in node_list if (u,-1) in edge_list_x) == 1, name = 'Constr 1')

	mSubprob.addConstr(gp.quicksum(h[u,-2] for u in node_list if (u,-2) in edge_list_x)\
		- gp.quicksum(h[-2,u] for u in node_list if (-2,u) in edge_list_x) == 1, name = 'Constr 2')

	for uu in node_list:
		mSubprob.addConstr(gp.quicksum(h[uu,v] for v in node_list_x if (uu,v) in edge_list_x)\
			 - gp.quicksum(h[v,uu] for v in node_list_x if (v,uu) in edge_list_x) == 0, name = 'Constr 3 '+str(uu))
	
	

	# degree of node constraint
	mSubprob.addConstrs(gp.quicksum(h[uu,v] for v in node_list if (uu,v) in edge_list_x) <= 1 for uu in node_list)

	

	# budge/distance constraint
	mSubprob.addConstr(gp.quicksum((cost_edges[item]+cost_edges[(item[1], item[0])])*h[item] for item in edge_list) <= max_travel)

	# at least one edge in the line
	mSubprob.addConstr(gp.quicksum(h[item] for item in edge_list) >= 1) 

	
	

	mSubprob.optimize(lazyconstrs) 
	numVars = mSubprob.NumVars

	h_edge = []
	for item in edge_list_x:
			if h[item].X > 0.999:
				# print(item, x[item].X)
				h_edge.append(item)

	return mSubprob, mSubprob.objVal, h, numVars


def subproblemLP(r_current, bud_current, node_list, cost_edges, cap, gamma, edge_list, node_list_x, edge_list_x, mip_gap, time_limit, num_sol, max_travel):
	mSubprobLP = gp.Model('modelSubprobLP')
	print('run subproblemLP~')
	mSubprobLP.Params.outputFlag = 0
	
	h = mSubprobLP.addVars(edge_list_x, lb = 0, ub = 1, vtype = GRB.CONTINUOUS, name = 'h')
	
	mSubprobLP._h = h
 


	obj_func = 0	
	for item in edge_list:
		obj_func += -(bud_current*gamma*cost_edges[item]-cap*r_current[item] + bud_current*gamma*cost_edges[item[1],item[0]]-cap*r_current[item[1],item[0]]) * h[item]




	mSubprobLP.setObjective(obj_func, GRB.MAXIMIZE)

	
	# constraints
	mSubprobLP.addConstr(gp.quicksum(h[-1,u] for u in node_list if (-1,u) in edge_list_x)\
		- gp.quicksum(h[u,-1] for u in node_list if (u,-1) in edge_list_x) == 1, name = 'Constr 1')

	mSubprobLP.addConstr(gp.quicksum(h[u,-2] for u in node_list if (u,-2) in edge_list_x)\
		- gp.quicksum(h[-2,u] for u in node_list if (-2,u) in edge_list_x) == 1, name = 'Constr 2')

	for uu in node_list:
		mSubprobLP.addConstr(gp.quicksum(h[uu,v] for v in node_list_x if (uu,v) in edge_list_x)\
			 - gp.quicksum(h[v,uu] for v in node_list_x if (v,uu) in edge_list_x) == 0, name = 'Constr 3 '+str(uu))
	# print('created constraints')
	
	

	# degree of node constraint
	mSubprobLP.addConstrs(gp.quicksum(h[uu,v] for v in node_list if (uu,v) in edge_list_x) <= 1 for uu in node_list)

	

	# budge/distance constraint
	mSubprobLP.addConstr(gp.quicksum((cost_edges[item]+cost_edges[(item[1], item[0])])*h[item] for item in edge_list) <= max_travel)

	# at least one edge in the line
	mSubprobLP.addConstr(gp.quicksum(h[item] for item in edge_list) >= 1) 

	
	

	mSubprobLP.optimize() 

	h_edge = []
	for item in edge_list_x:
			if h[item].X > 0.00001:
				# print(item, x[item].X)
				h_edge.append(item)



	return mSubprobLP, mSubprobLP.objVal, h_edge





def subPrep(bus_nodes, bus_edges):
	bus_nodes_x = bus_nodes.copy()
	bus_nodes_x = bus_nodes_x + [-1, -2]

	bus_edges_x = bus_edges.copy()
	for v in bus_nodes:
		bus_edges_x.append((-1, v))
		bus_edges_x.append((v, -2))

	return bus_nodes_x, bus_edges_x


def oneSol(h_edge):
	path_G = nx.DiGraph(h_edge)
	path_generated = nx.bidirectional_shortest_path(path_G, -1, -2)
	path_generated_rev = path_generated.copy()
	path_generated_rev.reverse() 
	cycles_list = sorted(nx.simple_cycles(path_G)) 


	relevant_edges = []
	for i in range(len(path_generated)-1):
		relevant_edges.append((path_generated[i], path_generated[i+1]))
	irrelevant_edges = set(h_edge)-set(relevant_edges)
		


	path_generated.remove(-1)
	path_generated.remove(-2)
	path_generated_rev.remove(-1)
	path_generated_rev.remove(-2)

	tour = path_generated + path_generated_rev[1:]


	return tour, cycles_list, irrelevant_edges



def savePickle(city, lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current, saved_folder):

	save_pickle(lines_current, output_path(city, saved_folder, 'lines_relaxed.pkl'))
	save_pickle(pair_lineInd_dict_current, output_path(city, saved_folder, 'pair_lineInd_dict_relaxed.pkl'))
	save_pickle(dict_stlInd_uInd_current, output_path(city, saved_folder, 'dict_stlInd_uInd_relaxed.pkl'))



def trim(r_current, bud_current, node_list, cost_edges, cap, gamma, edge_list, node_list_x, edge_list_x, mip_gap, time_limit, num_sol, max_travel, num_bus_edges_selected):
	bus_edges_selected = set()
	bus_nodes_selected = set()
 
	start_time = time.time()
 
	mSubprobLP, subObjLP, x_edge = subproblemLP(r_current, bud_current, node_list, cost_edges, cap, gamma, edge_list, node_list_x, edge_list_x, mip_gap, time_limit, num_sol, max_travel)
 
	subLP_time = time.time() - start_time
 
	# get the reduced costs of h variables·
	x_RC = {}
	if mSubprobLP.status == gp.GRB.OPTIMAL:
		for var in mSubprobLP.getVars():
			if "h" in var.VarName:
				edge = re.findall("\[(.*?)\]", var.varName)[0]
				first, second = edge.split(',')
				first = int(first)
				second = int(second)

				if first != -1 and second != -2:
					x_RC[(first, second)] = var.RC
	 
	x_RC = dict(sorted(x_RC.items(), key = lambda item: item[1]))

	x_RC = list(x_RC.keys())

	x_RC_selected = set()

	while len(x_RC_selected) < num_bus_edges_selected:
		item = x_RC.pop(-1)
		if (item[1], item[0]) in x_RC:
			x_RC.remove((item[1], item[0]))

		x_RC_selected.add(item)
		x_RC_selected.add((item[1], item[0]))

		bus_nodes_selected.add(item[0])
		bus_nodes_selected.add(item[1])

	bus_edges_selected = list(x_RC_selected)
	bus_nodes_selected = list(bus_nodes_selected)

	return bus_edges_selected, bus_nodes_selected
	



def main(city, which_method, gamma, cap, budget, unit_dist, num_rounds, perc_bus_edges_selected, flex_dist, mip_gap, time_limit, num_sol, max_travel, saved_folder,\
	detour_coeff, alpha, include_naive, len_naive, cost_factor_naive, transit):

	output_dir(city, saved_folder)
 

	node_list, od_pairs, od_demands, lines, pair_lineInd_dict, dict_stlInd_uInd, ori_nodes, dest_nodes, demand_dic,\
	 cost_edges, cost_lines, cap_lines, bus_nodes, bus_edges = instance.main(city, which_method, gamma, cap, unit_dist, saved_folder, include_naive, len_naive, cost_factor_naive)


	num_bus_edges_selected = math.ceil(perc_bus_edges_selected * len(bus_edges))
	
	
	lines_current = copy.deepcopy(lines)
	cost_lines_current = copy.deepcopy(cost_lines)
	cap_lines_current = copy.deepcopy(cap_lines)
	pair_lineInd_dict_current = copy.deepcopy(pair_lineInd_dict)
	dict_stlInd_uInd_current = copy.deepcopy(dict_stlInd_uInd)
 


	lines_current_rela = copy.deepcopy(lines)
	cost_lines_current_rela = copy.deepcopy(cost_lines)
	cap_lines_current_rela = copy.deepcopy(cap_lines)
	pair_lineInd_dict_current_rela = copy.deepcopy(pair_lineInd_dict)
	dict_stlInd_uInd_current_rela = copy.deepcopy(dict_stlInd_uInd)
	
	


	# relevant quantities for running LP
	lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = maxRideLP.LPPrep(lines_current, node_list, ori_nodes, bus_nodes, od_pairs, dict_stlInd_uInd_current)

	# run LP only once
	m, obj, C4_toRemove, C5_toRemove, C8_toRemove, C6_toFix, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP = maxRideLP.getResult(node_list, od_pairs, ori_nodes, dest_nodes, lines_current, lines_edge, cost_edges, cost_lines_current, demand_dic, cap_lines, budget, list_f_sluv,\
		   list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, alpha, transit)


	# relevant quantities for running LPRelaxed
	lines_edge_rela, list_f_sluv_rela, list_y_slv_rela, list_w_slv_rela, list_z_stv_rela, list_z_rela, st_dict_rela, ls_dict_rela, svl_dict_rela, z_uvs_dict_rela, y_uvl_dict_rela, all_lines_edge, dict_edge_lines\
		  = maxRideLPRelaxed.LPPrep(lines_current_rela, node_list, ori_nodes, bus_nodes, od_pairs, dict_stlInd_uInd_current_rela)
   

	# run LPRelaxed only once
	m_rela, obj_rela, C4_toRemove_rela, C5_toRemove_rela, C6_toRemove_rela, C8_toRemove_rela, f_rela, y_rela, w_rela, z_rela, q_rela, x_rela, Y_rela =\
		  maxRideLPRelaxed.getResult(node_list, od_pairs, ori_nodes, dest_nodes, lines_current_rela, lines_edge_rela, cost_edges, cost_lines_current_rela, demand_dic, cap_lines, budget, list_f_sluv_rela,\
		   list_y_slv_rela, list_w_slv_rela, list_z_stv_rela, list_z_rela, bus_nodes, st_dict_rela, ls_dict_rela, svl_dict_rela, z_uvs_dict_rela, y_uvl_dict_rela, all_lines_edge, dict_edge_lines, alpha, transit)
	r_current = maxRideLPRelaxed.getShadowPrice(m_rela, "r_relaxed")
	bud_current = maxRideLPRelaxed.getShadowPrice(m_rela, "bud")[0]


	# for subproblemLP
	bus_nodes_x, bus_edges_x = subPrep(bus_nodes, bus_edges)
 
	# trim graph based on the current dual values
 
	
	bus_edges_selected, bus_nodes_selected = trim(r_current, bud_current, node_list, cost_edges, cap, gamma, bus_edges, bus_nodes_x, bus_edges_x, mip_gap, time_limit, num_sol, max_travel, num_bus_edges_selected)
 
	# for subproblem
	bus_nodes_selected_x, bus_edges_selected_x = subPrep(bus_nodes_selected, bus_edges_selected)

	# run subproblem 
	log_directory = output_path(city, saved_folder, 'generate_lines_CG_relaxed_log.txt')
	mSubprob, subObj, x, numVars = subproblem(r_current, bud_current, bus_nodes_selected, cost_edges, cap, gamma, bus_edges_selected, bus_nodes_selected_x, bus_edges_selected_x, mip_gap, time_limit, num_sol, max_travel, detour_coeff, log_directory)
	


	savePickle(city, lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current, saved_folder)




	counter = 0
	continue_flag = True # to flag whether trimmed graph has any edge at all

	
	while continue_flag == True and subObj > 0 and counter < num_rounds:
		counter += 1

		nSolutions = mSubprob.SolCount
  

		for i in range(num_sol):
			if nSolutions > i:
				x_edge = []
				mSubprob.setParam(GRB.Param.SolutionNumber, i)
	
				print('Selected elements in', i, 'best solution:')
				print('Objective of this solution', mSubprob.PoolObjVal)
	
	
				if mSubprob.PoolObjVal <= 0:
					break
				
				for item in bus_edges_selected_x:
					if x[item].Xn > 0.999:
						x_edge.append(item)
	  
	
				tour, cycles_list, irrelevant_edges = oneSol(x_edge)
				detour_flag = check_detour(tour, cost_edges, detour_coeff)
	
	
	
				if tour in lines_current:
					print('already has this line in the set')

				if len(cycles_list) > 0:
					print('contains cycles')
					break
 
				if len(irrelevant_edges) > 0:
					print('irrelevant edges')
					break 
 
				if detour_flag == False:
					print('violate detour')
					break

				# create variables needed for running LP after generating the new line 
				lines_current, cost_lines_current, cap_lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current =\
			  		maxRideLP.newLineVars(tour, lines_current, cost_lines_current, cap_lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current, gamma, cap, cost_edges, node_list, flex_dist, od_pairs, unit_dist)
				
				lines_current_rela, cost_lines_current_rela, cap_lines_current_rela, pair_lineInd_dict_current_rela, dict_stlInd_uInd_current_rela =\
			  		maxRideLPRelaxed.newLineVars(tour, lines_current_rela, cost_lines_current_rela, cap_lines_current_rela, pair_lineInd_dict_current_rela, dict_stlInd_uInd_current_rela, gamma, cap, cost_edges, node_list, flex_dist, od_pairs, unit_dist)
				
				# save important variables so far
				savePickle(city, lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current, saved_folder)

	
				#re-generate LP (add additional vars and constraints)
				m, C4_toRemove, C5_toRemove, C8_toRemove, C6_toFix, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP, lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z\
		  		, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = maxRideLP.newLineLP(m, C4_toRemove, C5_toRemove, C8_toRemove, C6_toFix, cap, budget, od_pairs, node_list, lines_current, lines_edge, ori_nodes, cost_edges, tour, cost_lines_current, cap_lines_current, dict_stlInd_uInd_current, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z\
		  		, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP, alpha)
      
				# re-generate LPRelaxed (add additional vars and constraints)
				m_rela, C4_toRemove_rela, C5_toRemove_rela, C6_toRemove_rela, C8_toRemove_rela, f_rela, y_rela, w_rela, z_rela, q_rela, x_rela, Y_rela, lines_edge_rela, list_f_sluv_rela, list_y_slv_rela, list_w_slv_rela, list_z_stv_rela, list_z_rela\
		  		, st_dict_rela, ls_dict_rela, svl_dict_rela, z_uvs_dict_rela, y_uvl_dict_rela = maxRideLPRelaxed.newLineLP(m_rela, C4_toRemove_rela, C5_toRemove_rela, C6_toRemove_rela, C8_toRemove_rela, cap, budget, od_pairs, node_list, lines_current_rela,\
	  					lines_edge_rela, ori_nodes, cost_edges, tour, cost_lines_current_rela, cap_lines_current_rela, dict_stlInd_uInd_current_rela, list_f_sluv_rela, list_y_slv_rela, list_w_slv_rela, list_z_stv_rela, list_z_rela\
		  				, bus_nodes, st_dict_rela, ls_dict_rela, svl_dict_rela, z_uvs_dict_rela, y_uvl_dict_rela, f_rela, y_rela, w_rela, z_rela, q_rela, x_rela, Y_rela, alpha)


		

		
		# resolve LP
		m.optimize()
		obj = m.ObjVal
  
	
		# resolve LPRelaxed
		m_rela.optimize()
		r_current = maxRideLPRelaxed.getShadowPrice(m_rela, "r_relaxed")
		bud_current = maxRideLPRelaxed.getShadowPrice(m_rela, "bud")[0]
  
  
		start_time_oneround = time.time()
		start_time_oneround_GPU = time.process_time()
  
  
		bus_nodes = load_pickle(data_path(city, 'bus_nodes_dense.pkl'))
  
		bus_edges = load_pickle(data_path(city, 'bus_edges_1.25mile_dense.pkl'))
	
		
		# for subproblemLP
		bus_nodes_x, bus_edges_x = subPrep(bus_nodes, bus_edges)
  
  
		# trim graph based on the current dual values
  
		bus_edges_selected, bus_nodes_selected = trim(r_current, bud_current, node_list, cost_edges, cap, gamma, bus_edges, bus_nodes_x, bus_edges_x, mip_gap, time_limit, num_sol, max_travel, num_bus_edges_selected)
 
	
		
		# unable to select trimmed graph anymore, stop the process
		if len(bus_edges_selected) == 0:
			continue_flag = False

		else: 

			# for subproblem
			bus_nodes_selected_x, bus_edges_selected_x = subPrep(bus_nodes_selected, bus_edges_selected)

			# run subproblem 
			log_directory = output_path(city, saved_folder, 'generate_lines_CG_relaxed_log.txt')
			mSubprob, subObj, x, numVars = subproblem(r_current, bud_current, bus_nodes_selected, cost_edges, cap, gamma, bus_edges_selected, bus_nodes_selected_x, bus_edges_selected_x, mip_gap, time_limit, num_sol, max_travel, detour_coeff, log_directory)
			

	return m, C4_toRemove, C5_toRemove, C8_toRemove, C6_toFix, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP, lines_current, cost_lines_current, cap_lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current

	
