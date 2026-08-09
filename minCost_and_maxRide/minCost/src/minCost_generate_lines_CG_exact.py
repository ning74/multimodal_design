import networkx as nx
import dill as pickle
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import time
import random
import math
import re
import copy


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "root"))

import minCostLP 
from paths import output_path, save_pickle, data_path, load_pickle, output_dir



def lazyconstrs(model, where):  
	
	if where == GRB.Callback.MIPSOL:
		# make a list of edges selected in the solution
		h_vals = model.cbGetSolution(model._h)
		selected = list(edge for edge in model._h.keys() if h_vals[edge] > 0.9999)
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
						


def subproblem(k_current, p_current, cap, gamma, node_list, node_list_x, ori_nodes, cost_edges, edge_list, edge_list_x,\
			sBs_list, list_p_sv, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, mip_gap, time_limit, num_sol, max_travel, detour_coeff, log_directory):
	mSubprob = gp.Model('modelSubprob')
	print('run subproblem exact~')
	mSubprob.Params.outputFlag = 1
	mSubprob.Params.lazyConstraints = 1
	mSubprob.Params.MIPGap = mip_gap
	mSubprob.Params.timeLimit = time_limit
	mSubprob.Params.poolSearchMode = 2
	mSubprob.Params.poolSolutions = num_sol
	mSubprob.Params.MIPFocus = 1
 
	mSubprob.Params.LogFile = log_directory
	


	h = mSubprob.addVars(edge_list_x, lb = 0, ub = 1, vtype = GRB.INTEGER, name = 'h')

	x = mSubprob.addVars(sBs_list, lb = 0, vtype = GRB.CONTINUOUS, name = 'x')

	y = mSubprob.addVars(list_p_sv, lb = 0, vtype = GRB.CONTINUOUS, name = 'y')

	f = mSubprob.addVars(ori_nodes, edge_list, lb = 0, vtype = GRB.CONTINUOUS, name = 'f')
	
	
	for v in x.values(): 
		v.PoolIgnore = 1
  
	for v in y.values(): 
		v.PoolIgnore = 1
   
	for v in f.values(): 
		v.PoolIgnore = 1
	
	mSubprob._h = h
 
	
	
	costEdges_var = mSubprob.addVars(list(cost_edges), lb = 0, vtype = GRB.CONTINUOUS)
	for key, value in cost_edges.items():
		costEdges_var[key].lb = value
		costEdges_var[key].ub = value
  
	multiple_var = mSubprob.addVar(lb = detour_coeff, ub = detour_coeff)
	
	mSubprob._costEdges = costEdges_var
	mSubprob._multiple = multiple_var


	

	obj_func = gp.quicksum(x[item]*k_current[item] for item in sBs_list) + gp.quicksum(y[item]*p_current[item] for item in list_p_sv)\
		  + gp.quicksum(((gamma / cap) * (cost_edges[item]+cost_edges[item[1],item[0]]) * h[item]) for item in edge_list)
	
	



	mSubprob.setObjective(obj_func, GRB.MINIMIZE)

	
	# source
	mSubprob.addConstr(gp.quicksum(h[-1,u] for u in dict_bus_to_x[-1])\
		- gp.quicksum(h[u,-1] for u in dict_bus_from_x[-1]) == 1)

	# sink
	mSubprob.addConstr(gp.quicksum(h[u,-2] for u in dict_bus_from_x[-2])\
		- gp.quicksum(h[-2,u] for u in dict_bus_to_x[-2]) == 1)


	# flow conservation
	mSubprob.addConstrs(gp.quicksum(h[uu,v] for v in dict_bus_to_x[uu])\
			 - gp.quicksum(h[v,uu] for v in dict_bus_from_x[uu]) == 0 for uu in node_list)
	
	

	# degree of node constraint
	mSubprob.addConstrs(gp.quicksum(h[uu,v] for v in dict_bus_to_x[uu]) <= 1 for uu in node_list)


	# budget/distance constraint 
	mSubprob.addConstr(gp.quicksum((cost_edges[item]+cost_edges[(item[1], item[0])])*h[item] for item in edge_list) <= max_travel) 

	# at least one edge in the line
	mSubprob.addConstr(gp.quicksum(h[item] for item in edge_list) >= 1)



	# capacity constraint
	mSubprob.addConstrs(gp.quicksum(f[s,item[0],item[1]] for s in ori_nodes) <= h[item] + h[item[1],item[0]] for item in edge_list)


	# flow conservation 
	mSubprob.addConstrs(gp.quicksum(-y[s,b] for (s,b) in [(ss,bb)] if (s,b) in list_p_sv) + gp.quicksum(x[s,b] for (s,b) in [(ss,bb)] if (s,b) in sBs_list)\
			+ gp.quicksum(f[ss,v,bb] for v in dict_bus_from[bb])\
				- gp.quicksum(f[ss,bb,v] for v in dict_bus_to[bb]) == 0 for ss in ori_nodes for bb in node_list)
	
	mSubprob.optimize(lazyconstrs) 


	numVars = mSubprob.NumVars


	h_edge = []
	for item in edge_list_x:
		if h[item].X > 0.999:
			h_edge.append(item)

	return mSubprob, mSubprob.ObjVal, h, numVars



def subproblemLP(k_current, p_current, cap, gamma, node_list, node_list_x, ori_nodes, cost_edges, edge_list, edge_list_x,\
		  sBs_list, list_p_sv, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, max_travel):
	mSubprobLP = gp.Model('modelSubprobLP')
	print('run subproblem LP~')
	mSubprobLP.Params.outputFlag = 0
	


	h = mSubprobLP.addVars(edge_list_x, lb = 0, ub = 1, vtype = GRB.CONTINUOUS, name = 'h')

	x = mSubprobLP.addVars(sBs_list, lb = 0, vtype = GRB.CONTINUOUS, name = 'x')

	y = mSubprobLP.addVars(list_p_sv, lb = 0, vtype = GRB.CONTINUOUS, name = 'y')


	f = mSubprobLP.addVars(ori_nodes, edge_list, lb = 0, vtype = GRB.CONTINUOUS, name = 'f')
	
	mSubprobLP._h = h

	
	obj_func = gp.quicksum(x[item]*k_current[item] for item in sBs_list) + gp.quicksum(y[item]*p_current[item] for item in list_p_sv)\
		  + gp.quicksum(((gamma / cap) * (cost_edges[item]+cost_edges[item[1],item[0]]) * h[item]) for item in edge_list)
	
	



	mSubprobLP.setObjective(obj_func, GRB.MINIMIZE)

	
	# source
	mSubprobLP.addConstr(gp.quicksum(h[-1,u] for u in dict_bus_to_x[-1])\
		- gp.quicksum(h[u,-1] for u in dict_bus_from_x[-1]) == 1)

	# sink
	mSubprobLP.addConstr(gp.quicksum(h[u,-2] for u in dict_bus_from_x[-2])\
		- gp.quicksum(h[-2,u] for u in dict_bus_to_x[-2]) == 1)


	# flow conservation
	mSubprobLP.addConstrs(gp.quicksum(h[uu,v] for v in dict_bus_to_x[uu])\
			 - gp.quicksum(h[v,uu] for v in dict_bus_from_x[uu]) == 0 for uu in node_list)
	
	

	# degree of node constraint
	mSubprobLP.addConstrs(gp.quicksum(h[uu,v] for v in dict_bus_to_x[uu]) <= 1 for uu in node_list)


	# budget/distance constraint 
	mSubprobLP.addConstr(gp.quicksum((cost_edges[item]+cost_edges[(item[1], item[0])])*h[item] for item in edge_list) <= max_travel) 

	# at least one edge in the line 
	mSubprobLP.addConstr(gp.quicksum(h[item] for item in edge_list) >= 1)



	# capacity constraint 
	mSubprobLP.addConstrs(gp.quicksum(f[s,item[0],item[1]] for s in ori_nodes) <= h[item] + h[item[1],item[0]] for item in edge_list)


	# flow conservation 
	mSubprobLP.addConstrs(gp.quicksum(-y[s,b] for (s,b) in [(ss,bb)] if (s,b) in list_p_sv) + gp.quicksum(x[s,b] for (s,b) in [(ss,bb)] if (s,b) in sBs_list)\
			+ gp.quicksum(f[ss,v,bb] for v in dict_bus_from[bb])\
				- gp.quicksum(f[ss,bb,v] for v in dict_bus_to[bb]) == 0 for ss in ori_nodes for bb in node_list)




	mSubprobLP.optimize()


	h_edge = []
	for item in edge_list_x:
		if h[item].X > 0.00001: 
			h_edge.append(item)

	return mSubprobLP, mSubprobLP.ObjVal, h_edge


def trimOneRound(h_edge, bus_edges, bus_edges_x, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, bus_edges_selected, bus_nodes_selected):
	h_edge_trim = []
	for item in h_edge:
		if item[0] != -1 and item[1] != -2 and (item[1], item[0]) not in h_edge_trim:
			h_edge_trim.append(item)
		
	for item in h_edge_trim:
		bus_edges.remove(item)
		bus_edges_x.remove(item)
		dict_bus_to[item[0]].remove(item[1])
		dict_bus_from[item[1]].remove(item[0])
		dict_bus_to_x[item[0]].remove(item[1])
		dict_bus_from_x[item[1]].remove(item[0])

		bus_edges.remove((item[1], item[0]))
		bus_edges_x.remove((item[1], item[0]))
		dict_bus_to[item[1]].remove(item[0])
		dict_bus_from[item[0]].remove(item[1])
		dict_bus_to_x[item[1]].remove(item[0])
		dict_bus_from_x[item[0]].remove(item[1])

		bus_edges_selected.add(item)
		bus_edges_selected.add((item[1], item[0]))
		bus_nodes_selected.add(item[0])
		bus_nodes_selected.add(item[1])

	return bus_edges, bus_edges_x, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, bus_edges_selected, bus_nodes_selected

	
def subPrep(bus_nodes, bus_edges, ori_nodes, flex_dist, cost_edges, od_pairs):

	bus_nodes_x = bus_nodes.copy()
	bus_nodes_x = bus_nodes_x + [-1, -2]

	bus_edges_x = bus_edges.copy()
	for v in bus_nodes:
		bus_edges_x.append((-1, v))
		bus_edges_x.append((v, -2))

	dict_bus_to = {}
	dict_bus_from = {}
	for v in bus_nodes:
		dict_bus_to[v] = set()
		dict_bus_from[v] = set()

	for item in bus_edges:
		dict_bus_to[item[0]].add(item[1])
		dict_bus_from[item[1]].add(item[0])

	dict_bus_to_x = {}
	dict_bus_from_x = {}
	for v in bus_nodes_x:
		dict_bus_to_x[v] = set()
		dict_bus_from_x[v] = set()
	for item in bus_edges_x:
		dict_bus_to_x[item[0]].add(item[1])
		dict_bus_from_x[item[1]].add(item[0])

	sBs_list = set()
	for s in ori_nodes:
		for v in bus_nodes:
			if cost_edges[s,v] <= flex_dist:
				sBs_list.add((s,v))
	sBs_list = list(sBs_list)

	list_p_sv = set()
	for (s,t) in od_pairs:
		for v in bus_nodes:
			if cost_edges[v,t] <= flex_dist:
				list_p_sv.add((s,v))

	return bus_nodes_x, bus_edges_x, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, sBs_list, list_p_sv




def oneSol(h_edge):
	path_G = nx.DiGraph(h_edge)
	path_generated = nx.bidirectional_shortest_path(path_G, -1, -2)
	path_generated_rev = path_generated.copy()
	path_generated_rev.reverse() 
	cycles_list = sorted(nx.simple_cycles(path_G)) 


	relevant_edges = []
	for i in range(len(path_generated)-1):
		relevant_edges.append((path_generated[i], path_generated[i+1]))
	irrevevant_edges = set(h_edge)-set(relevant_edges)
		


	path_generated.remove(-1)
	path_generated.remove(-2)
	path_generated_rev.remove(-1)
	path_generated_rev.remove(-2)

	tour = path_generated + path_generated_rev[1:]


	return tour, cycles_list, irrevevant_edges


def savePickle(city, lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current, saved_folder):

	save_pickle(lines_current, output_path(city, saved_folder, 'lines_exact.pkl'))
	save_pickle(pair_lineInd_dict_current, output_path(city, saved_folder, 'pair_lineInd_dict_exact.pkl'))
	save_pickle(dict_stlInd_uInd_current, output_path(city, saved_folder, 'dict_stlInd_uInd_exact.pkl'))

	
	


def trim(k_current, p_current, cap, gamma, bus_nodes, bus_nodes_x, ori_nodes, cost_edges, bus_edges, bus_edges_x,\
							   sBs_list, list_p_sv, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, num_bus_edges_selected, max_travel):
	
	start_time = time.time()
	mSubprobLP, subObjLP, h_edge = subproblemLP(k_current, p_current, cap, gamma, bus_nodes, bus_nodes_x, ori_nodes, cost_edges, bus_edges, bus_edges_x,\
							   sBs_list, list_p_sv, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, max_travel)
	subLP_time = time.time() - start_time

	h_RC = {}
	if mSubprobLP.status == gp.GRB.OPTIMAL:
		for var in mSubprobLP.getVars():
			if "h" in var.VarName:
				edge = re.findall("\[(.*?)\]", var.varName)[0]
				first, second = edge.split(',')
				first = int(first)
				second = int(second)

				if first != -1 and second != -2:
					h_RC[(first,second)] = var.RC

	h_RC = dict(sorted(h_RC.items(), key = lambda item: item[1]))
	h_RC = list(h_RC.keys())
	h_RC_selected = set()

	while len(h_RC_selected) < num_bus_edges_selected:
		item = h_RC.pop(0)
		if (item[1], item[0]) in h_RC:
			h_RC.remove((item[1], item[0]))

		h_RC_selected.add(item)
		h_RC_selected.add((item[1], item[0]))

		bus_nodes_selected.add(item[0])
		bus_nodes_selected.add(item[1])

	bus_edges_selected = list(h_RC_selected)
	bus_nodes_selected = list(bus_nodes_selected)

	return bus_edges_selected, bus_nodes_selected



def main(city, which_method, which_trim, gamma, cap, unit_dist, num_rounds, perc_bus_edges_selected, flex_dist, mip_gap, time_limit, num_sol, max_travel, saved_folder, detour_coeff, demand_perc, include_naive, len_naive, cost_factor_naive):

	output_dir(city, saved_folder)
 

	node_list, od_pairs, od_demands, lines, pair_lineInd_dict, dict_stlInd_uInd, ori_nodes, dest_nodes, demand_dic,\
	 cost_edges, cost_lines, cap_lines, bus_nodes, bus_edges = instance.main(city, which_method, gamma, cap, unit_dist, saved_folder, include_naive, len_naive, cost_factor_naive)

	
	num_bus_edges_selected = math.ceil(perc_bus_edges_selected * len(bus_edges))

	
	lines_current = copy.deepcopy(lines)
	cost_lines_current = copy.deepcopy(cost_lines)
	cap_lines_current = copy.deepcopy(cap_lines)
	pair_lineInd_dict_current = copy.deepcopy(pair_lineInd_dict)
	dict_stlInd_uInd_current = copy.deepcopy(dict_stlInd_uInd)
	



	# relevant quantities for running LP
	lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = minCostLP.LPPrep(lines_current, node_list, ori_nodes, bus_nodes, od_pairs, dict_stlInd_uInd_current)

	# run LP only once
	m, obj, C4_toRemove, C5_toRemove, C6_toFix, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP = minCostLP.getResult(node_list, od_pairs, ori_nodes, dest_nodes, lines_current, lines_edge, cost_edges, cost_lines_current, demand_dic, cap_lines,\
		  list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, demand_perc)
	

	


	
	p_current = minCostLP.getShadowPrice(m, "p")
	k_current = minCostLP.getShadowPrice(m, "k")
 
	
	
	# for subproblemLP 
	bus_nodes_x, bus_edges_x, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, sBs_list, list_p_sv = subPrep(bus_nodes, bus_edges, ori_nodes, flex_dist, cost_edges, od_pairs)
	

	# trim graph based on the current dual values
	bus_edges_selected = set()
	bus_nodes_selected = set()
	

	bus_edges_selected, bus_nodes_selected =  trim(k_current, p_current, cap, gamma, bus_nodes, bus_nodes_x, ori_nodes, cost_edges, bus_edges, bus_edges_x,\
			sBs_list, list_p_sv, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, num_bus_edges_selected, max_travel)




	# for subproblem
	bus_nodes_selected_x, bus_edges_selected_x, dict_bus_to_selected, dict_bus_from_selected,\
	 dict_bus_to_selected_x, dict_bus_from_selected_x, sBs_list_selected, list_p_sv_selected\
		  = subPrep(bus_nodes_selected, bus_edges_selected, ori_nodes, flex_dist, cost_edges, od_pairs)
	
	
	# run subprblem based on the trimmed graph
	log_directory = output_path(city, saved_folder, 'generate_lines_CG_exact_log.txt')
	mSubprob, subObj, h, numVars = subproblem(k_current, p_current, cap, gamma, bus_nodes_selected, bus_nodes_selected_x, ori_nodes, cost_edges, bus_edges_selected, bus_edges_selected_x,\
					   sBs_list_selected, list_p_sv_selected, dict_bus_to_selected, dict_bus_from_selected, dict_bus_to_selected_x, dict_bus_from_selected_x, mip_gap, time_limit, num_sol, max_travel, detour_coeff, log_directory)
	
	
	savePickle(city, lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current, saved_folder)



 
	

	counter_outer = 0
	continue_flag = True # to flag whether trimmed graph has any edge at all

	if len(bus_edges_selected) == 0:
		continue_flag = False

	while continue_flag == True and counter_outer < num_rounds and subObj < 0:
		counter_outer += 1


		nSolutions = mSubprob.SolCount

		for i in range(num_sol):
			if (nSolutions > i):
				h_edge = []

				mSubprob.setParam(GRB.Param.SolutionNumber, i)

				if mSubprob.PoolObjVal >= 0:
					break

				for item in bus_edges_selected_x:
					if h[item].Xn > 0.999: 
						h_edge.append(item)


				tour, cycles_list, irrelevant_edges = oneSol(h_edge)

				if tour in lines_current:
					print('already has this line in the set') 

				if len(cycles_list) > 0:
					print('contains cycles')
					break

				if len(irrelevant_edges) > 0:
					print('irrelevant edges')
					break

				# create variables needed for running LP after generating the new line
				lines_current, cost_lines_current, cap_lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current = minCostLP.newLineVars(tour, lines_current, cost_lines_current, cap_lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current, gamma, cap, cost_edges, node_list, flex_dist, od_pairs, unit_dist)

				# save important variables so far
				savePickle(city, lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current, saved_folder)


				# re-generate LP (add additional vars and constraints)
				m, C4_toRemove, C5_toRemove, C6_toFix, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP, lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z\
		  		, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = minCostLP.newLineLP(m, C4_toRemove, C5_toRemove, C6_toFix, cap, od_pairs, node_list, lines_current, lines_edge, ori_nodes, cost_edges, tour, cost_lines_current, cap_lines_current, dict_stlInd_uInd_current, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z\
		  		, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP)
		
				
		



		# resolve LP 

		m.optimize()
		obj = m.ObjVal


		p_current = minCostLP.getShadowPrice(m, "p")
		k_current = minCostLP.getShadowPrice(m, "k")

		bus_nodes = load_pickle(data_path(city, 'bus_nodes_dense.pkl'))
  
		bus_edges = load_pickle(data_path(city, 'bus_edges_1.25mile_dense.pkl'))
	


		
		# for subproblemLP 
		bus_nodes_x, bus_edges_x, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, sBs_list, list_p_sv = subPrep(bus_nodes, bus_edges, ori_nodes, flex_dist, cost_edges, od_pairs)


		# get a new trimmed graph from the dual 
		bus_edges_selected = set()
		bus_nodes_selected = set()

		bus_edges_selected, bus_nodes_selected =  trim(k_current, p_current, cap, gamma, bus_nodes, bus_nodes_x, ori_nodes, cost_edges, bus_edges, bus_edges_x,\
							   sBs_list, list_p_sv, dict_bus_to, dict_bus_from, dict_bus_to_x, dict_bus_from_x, num_bus_edges_selected, max_travel)


		


		# unable to select trimmed graph anymore, stop the process
		if len(bus_edges_selected) == 0:
			continue_flag = False # break the out while loop 

		else: 


			# for subproblem
			bus_nodes_selected_x, bus_edges_selected_x, dict_bus_to_selected, dict_bus_from_selected,\
				  dict_bus_to_selected_x, dict_bus_from_selected_x, sBs_list_selected, list_p_sv_selected\
					  = subPrep(bus_nodes_selected, bus_edges_selected, ori_nodes, flex_dist, cost_edges, od_pairs)

			# run subprblem based on the trimmed graph
			log_directory = output_path(city, saved_folder, 'generate_lines_CG_exact_log.txt')
			mSubprob, subObj, h, numVars = subproblem(k_current, p_current, cap, gamma, bus_nodes_selected, bus_nodes_selected_x, ori_nodes, cost_edges, bus_edges_selected, bus_edges_selected_x, sBs_list_selected,\
					 list_p_sv_selected, dict_bus_to_selected, dict_bus_from_selected, dict_bus_to_selected_x, dict_bus_from_selected_x, mip_gap, time_limit, num_sol, max_travel, detour_coeff, log_directory)


	return m, C4_toRemove, C5_toRemove, C6_toFix, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP, lines_current, cost_lines_current, cap_lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current

		

	
			

		
