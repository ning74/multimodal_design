import gurobipy as gp
from gurobipy import GRB
import time
import networkx as nx
import dill as pickle
import collections
import numpy as np
import re
import math
import argparse


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "root"))


import instance

import maxRideLP
import maxRideMIP
from paths import output_dir, result_path, heur_path






def modifyLP(Y, list_lines_to_zero):
	for i in list_lines_to_zero:
		Y[i].lb = 0
		Y[i].ub = 0

def main(city, gamma, cap, budget, flex_dist, mip_gap, unit_dist, top, bottom, saved_folder, firstSelect, secondSelectStep, num_finalLines, saved_folder_new, mip_focus, time_limit, alpha, transit):     

	output_dir(city, saved_folder)
	

	node_list, od_pairs, od_demands, lines, pair_lineInd_dict, dict_stlInd_uInd, ori_nodes, dest_nodes, demand_dic,\
	cost_edges, cost_lines, cap_lines, bus_nodes, bus_edges = instance.main(city, 'final', gamma, cap, unit_dist, saved_folder, include_naive=False, len_naive=-1, cost_factor_naive=-1)

	num_originalLines = len(lines)

	lines_current = [lines[i] for i in list(range(0, 0))]
	pair_lineInd_dict_current, dict_stlInd_uInd_current, cost_lines_current, cap_lines_current = maxRideLP.extract_lines(city, demand_dic ,gamma, cap, flex_dist, unit_dist, lines_current)
	 																						
	# relevant quantities for running 
	lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = maxRideLP.LPPrep(lines_current, node_list, ori_nodes, bus_nodes, od_pairs, dict_stlInd_uInd_current)
	# run once 
	m_int, obj_int, C4_toRemove_int, C5_toRemove_int, C8_toRemove_int, C6_toFix_int, f_int, y_int, w_int, z_int, q_int, x_int, Y_int = maxRideLP.getResult(node_list, od_pairs, ori_nodes, dest_nodes, lines_current, lines_edge, cost_edges, cost_lines_current, demand_dic, cap_lines_current,\
		  budget, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, alpha, transit)
	currentBest = obj_int
	
 
	# write gurobi optimal solutions
	path_name = result_path(city, 'heur_optSol_taxiOnly[city@{}|alpha@{}|budget@{}|mip_gap@{}|top@{}|bottom@{}|firstSelect@{}|secondSelectStep@{}|num_finalLines@{}|mip_focus@{}|time_limit@{}|saved_folder@{}].txt'\
		.format(city, alpha, budget, mip_gap, top, bottom, firstSelect, secondSelectStep, num_finalLines, mip_focus, time_limit, saved_folder))
		
	with open(path_name, "w") as file:
		file.write(f"Objective Value = {m_int.ObjVal}\n")
		for v in m_int.getVars():
				# Write each line to the file
			file.write(f"{v.VarName} = {v.X}\n") 
 
 

	# run lines LP 
	node_list, od_pairs, od_demands, lines, pair_lineInd_dict, dict_stlInd_uInd, ori_nodes, dest_nodes, demand_dic,\
	cost_edges, cost_lines, cap_lines, bus_nodes, bus_edges = instance.main(city, 'final', gamma, cap, unit_dist, saved_folder, include_naive=False, len_naive=-1, cost_factor_naive=-1)

	print('extract [{}, {}]'.format(top, min(bottom, len(lines))))
	lines_current = [lines[i] for i in list(range(top, min(bottom, len(lines))))]
 
	# first stage of selecting lines
	numSelect_real = min(len(lines_current), firstSelect)
 
	pair_lineInd_dict_current, dict_stlInd_uInd_current, cost_lines_current, cap_lines_current = maxRideLP.extract_lines(city, demand_dic, gamma, cap, flex_dist, unit_dist, lines_current)
 
	# relevant quantities for running LP
	lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = maxRideLP.LPPrep(lines_current, node_list, ori_nodes, bus_nodes, od_pairs, dict_stlInd_uInd_current)


	# run LP only once
	m_LP, obj_LP, C4_toRemove_LP, C5_toRemove_LP, C8_toRemove_LP, C6_toFix_LP, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP = maxRideLP.getResult(node_list, od_pairs, ori_nodes, dest_nodes, lines_current, lines_edge, cost_edges, cost_lines_current, demand_dic, cap_lines_current,\
		  budget, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, alpha, transit)
	
	m_LP.write(heur_path(city, saved_folder_new, 'LP_gradualSelect_fullModel.mps', tree='heur_output'))
	m_LP.write(heur_path(city, saved_folder_new, 'LP_gradualSelect_fullModel.lp', tree='heur_output'))
 
 
	path_name = heur_path(city, saved_folder_new, 'heur_gradualSelect_LPsol_budget@{}_fullModel.txt'.format(budget), tree='heur_output')
	with open(path_name, "w") as file:
		file.write(f"Objective Value = {m_LP.ObjVal}\n")
		for v in m_LP.getVars():
			# Write each line to the file
			file.write(f"{v.VarName} = {v.X}\n")
   


	# get value and reduced costs of Y variables
	Y_RC = {}
	Y_value = {}
	if m_LP.status == gp.GRB.OPTIMAL:
		for var in m_LP.getVars():
			if "var_Y" in var.VarName:
				# print('var.Name', var.VarName)
				# print('var.RC', var.RC)
				line = re.findall("\[(.*?)\]", var.varName)[0]
				Y_RC[int(line)] = var.RC
				Y_value[int(line)] = var.X
	

	# sort in increasing order in reduced cost
	Y_RC = dict(sorted(Y_RC.items(), key = lambda item: item[1]))


	# sort in increasing order in Y value
	Y_value = dict(sorted(Y_value.items(), key = lambda item: item[1]))
	# print('Y_value: ', Y_value)
	
	# normalize Y value based on lower bound
	Y_value_normalized = {}
	Y_RC_normalized = {}
	for key, value in Y_value.items():
			if value > 0.0001:
				Y_value_normalized[key] = value / math.ceil(cap/cap_lines_current[key])
	
	# sort in increasing order in normalized Y value
	Y_value_normalized = dict(sorted(Y_value_normalized.items(), key = lambda item: item[1]))
   
	# normalized reduced cost based on lower bound 
	for key, value in Y_RC.items():
		if key not in Y_value_normalized:
			Y_RC_normalized[key] = value * math.ceil(cap/cap_lines_current[key])
   
	Y_RC_normalized = dict(sorted(Y_RC_normalized.items(), key = lambda item: item[1]))
	
 
	
	# select lines by LP_thresh
	Y_value_normalized_ordered = list(Y_value_normalized.keys())
	Y_RC_normalized_ordered = list(Y_RC_normalized.keys())
	Y_normalized_ordered = Y_RC_normalized_ordered + Y_value_normalized_ordered
 
	line_select = []
	if len(lines_current) > numSelect_real:
		line_select = Y_normalized_ordered[-numSelect_real:]
	else:
		line_select = list(range(len(lines_current)))
	
 

	with open(heur_path(city, saved_folder_new, 'heur_gradualSelect_selectedLineIndex_budget@{}_numGradualSelected@{}_from{}.pkl'.format(budget, firstSelect, num_originalLines)), 'wb') as file:
		pickle.dump(line_select, file)
 
  
	
	lines_current = [lines_current[i] for i in line_select]
	
	numSelect_real = len(lines_current) - secondSelectStep # update numSelect_real
	
	pair_lineInd_dict_current, dict_stlInd_uInd_current, cost_lines_current, cap_lines_current = maxRideLP.extract_lines(city, demand_dic, gamma, cap, flex_dist, unit_dist, lines_current)
 
	lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = maxRideLP.LPPrep(lines_current, node_list, ori_nodes, bus_nodes, od_pairs, dict_stlInd_uInd_current)

	m_LP, obj_LP, C4_toRemove_LP, C5_toRemove_LP, C8_toRemove_LP, C6_toFix_LP, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP = maxRideLP.getResult(node_list, od_pairs, ori_nodes, dest_nodes, lines_current, lines_edge, cost_edges, cost_lines_current, demand_dic, cap_lines_current,\
		  budget, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, alpha, transit)
	
	

	m_LP.write(heur_path(city, saved_folder_new, 'LP_gradualSelect_{}lines.mps'.format(firstSelect), tree='heur_output'))
	m_LP.write(heur_path(city, saved_folder_new, 'LP_gradualSelect_{}lines.lp'.format(firstSelect), tree='heur_output'))
 
	path_name = heur_path(city, saved_folder_new, 'heur_gradualSelect_LPsol_budget@{}_firstSelect@{}.txt'.format(budget, firstSelect), tree='heur_output')
	with open(path_name, "w") as file:
		file.write(f"Objective Value = {m_LP.ObjVal}\n")
		for v in m_LP.getVars():
			# Write each line to the file
			file.write(f"{v.VarName} = {v.X}\n")
   
   
	# gradual selection to get down to 50 lines
 
	# get value and reduced costs of Y variables
	Y_RC = {}
	Y_value = {}
	if m_LP.status == gp.GRB.OPTIMAL:
		for var in m_LP.getVars():
			if "var_Y" in var.VarName:
				# print('var.Name', var.VarName)
				# print('var.RC', var.RC)
				line = re.findall("\[(.*?)\]", var.varName)[0]
				Y_RC[int(line)] = var.RC
				Y_value[int(line)] = var.X
    
    
	# sort in increasing order in reduced cost
	Y_RC = dict(sorted(Y_RC.items(), key = lambda item: item[1]))


	# sort in increasing order in Y value
	Y_value = dict(sorted(Y_value.items(), key = lambda item: item[1]))
	# print('Y_value: ', Y_value)
	
	# normalize Y value based on lower bound
	Y_value_normalized = {}
	Y_RC_normalized = {}
	for key, value in Y_value.items():
			if value > 0.0001:
				Y_value_normalized[key] = value / math.ceil(cap/cap_lines_current[key])
	
	# sort in increasing order in normalized Y value
	Y_value_normalized = dict(sorted(Y_value_normalized.items(), key = lambda item: item[1]))
   
	# normalized reduced cost based on lower bound 
	for key, value in Y_RC.items():
		if key not in Y_value_normalized:
			Y_RC_normalized[key] = value * math.ceil(cap/cap_lines_current[key])
   
	Y_RC_normalized = dict(sorted(Y_RC_normalized.items(), key = lambda item: item[1]))
	
 
	# select lines by LP_thresh
	Y_value_normalized_ordered = list(Y_value_normalized.keys())
	Y_RC_normalized_ordered = list(Y_RC_normalized.keys())
	Y_normalized_ordered = Y_RC_normalized_ordered + Y_value_normalized_ordered
 
	line_select = []
	if len(lines_current) > numSelect_real:
		line_select = Y_normalized_ordered[-numSelect_real:]
	else:
		line_select = list(range(len(lines_current)))
 
	# save pickle of selected line index
	with open(heur_path(city, saved_folder_new, 'heur_gradualSelect_selectedLineIndex_budget@{}_numGradualSelected@{}_from{}.pkl'.format(budget, numSelect_real, firstSelect)), 'wb') as file:
		pickle.dump(line_select, file)
	
  
	while numSelect_real > num_finalLines:
		numSelect_real = max(numSelect_real - secondSelectStep, num_finalLines)
  
		line_eliminate = set(range(len(lines_current))) - set(line_select)
  
		modifyLP(Y_LP, line_eliminate)
		m_LP.update()
		m_LP.optimize()
  
		path_name = heur_path(city, saved_folder_new, 'heur_gradualSelect_LPsol_budget@{}_nonzeroLines@{}_from{}.txt'.format(budget, numSelect_real + secondSelectStep, firstSelect), tree='heur_output')
		with open(path_name, "w") as file:
			file.write(f"Objective Value = {m_LP.ObjVal}\n")
			for v in m_LP.getVars():
				# Write each line to the file
				file.write(f"{v.VarName} = {v.X}\n")
    
		# get value and reduced costs of Y variables
		Y_RC = {}
		Y_value = {}
		if m_LP.status == gp.GRB.OPTIMAL:
			for var in m_LP.getVars():
				if "var_Y" in var.VarName:
					# print('var.Name', var.VarName)
					# print('var.RC', var.RC)
					line = re.findall("\[(.*?)\]", var.varName)[0]
					if int(line) not in line_eliminate:
						Y_RC[int(line)] = var.RC
						Y_value[int(line)] = var.X
      
		# sort in increasing order in reduced cost
		Y_RC = dict(sorted(Y_RC.items(), key = lambda item: item[1]))


		# sort in increasing order in Y value
		Y_value = dict(sorted(Y_value.items(), key = lambda item: item[1]))

		
		# normalize Y value based on lower bound
		Y_value_normalized = {}
		Y_RC_normalized = {}
		for key, value in Y_value.items():
				if value > 0.0001:
					Y_value_normalized[key] = value / math.ceil(cap/cap_lines_current[key])
		
		# sort in increasing order in normalized Y value
		Y_value_normalized = dict(sorted(Y_value_normalized.items(), key = lambda item: item[1]))
	
		# normalized reduced cost based on lower bound 
		for key, value in Y_RC.items():
			if key not in Y_value_normalized:
				Y_RC_normalized[key] = value * math.ceil(cap/cap_lines_current[key])
	
		Y_RC_normalized = dict(sorted(Y_RC_normalized.items(), key = lambda item: item[1]))
  

		# select lines by LP_thresh
		Y_value_normalized_ordered = list(Y_value_normalized.keys())
		Y_RC_normalized_ordered = list(Y_RC_normalized.keys())
		Y_normalized_ordered = Y_RC_normalized_ordered + Y_value_normalized_ordered
	
		line_select = []
		if len(lines_current) > len(line_select) - secondSelectStep:
			line_select = Y_normalized_ordered[-(len(line_select) - secondSelectStep):]
		else:
			line_select = list(range(len(lines_current)))
  
		#save pickle of selected line index
		with open(heur_path(city, saved_folder_new, 'heur_gradualSelect_selectedLineIndex_budget@{}_numGradualSelected@{}_from{}.pkl'.format(budget, numSelect_real, firstSelect)), 'wb') as file:
			pickle.dump(line_select, file)
		
  

	with open(heur_path(city, saved_folder_new, 'heur_gradualSelect_selectedLineIndex_budget@{}_numGradualSelected@{}_from{}.pkl'.format(budget, firstSelect, num_originalLines), create=False), 'rb') as file:
		lines_firstSelect_index = pickle.load(file) 
 
	with open(heur_path(city, saved_folder_new, 'heur_gradualSelect_selectedLineIndex_budget@{}_numGradualSelected@{}_from{}.pkl'.format(budget, num_finalLines, firstSelect), create=False), 'rb') as file:
		lines_finalLines_index_from_firstSelect = pickle.load(file) 
	
	
	lines_finalLines_index_from_original = [lines_firstSelect_index[i] for i in lines_finalLines_index_from_firstSelect]
 

	
 
	node_list, od_pairs, od_demands, lines, pair_lineInd_dict, dict_stlInd_uInd, ori_nodes, dest_nodes, demand_dic,\
	cost_edges, cost_lines, cap_lines, bus_nodes, bus_edges = instance.main(city, 'final', gamma, cap, unit_dist, saved_folder, include_naive=False, len_naive=-1, cost_factor_naive=-1)


	lines_current = [lines[i] for i in lines_finalLines_index_from_original]
 

	pair_lineInd_dict_current, dict_stlInd_uInd_current, cost_lines_current, cap_lines_current = maxRideLP.extract_lines(city, demand_dic, gamma, cap, flex_dist, unit_dist, lines_current)
 
	# relevant quantities for running 
	lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = maxRideLP.LPPrep(lines_current, node_list, ori_nodes, bus_nodes, od_pairs, dict_stlInd_uInd_current)

	# write gurobi log file
	log_name = result_path(city, 'heur_log[city@{}|alpha@{}|budget@{}|mip_gap@{}|top@{}|bottom@{}|firstSelect@{}|secondSelectStep@{}|num_finalLines@{}|mip_focus@{}|time_limit@{}|saved_folder@{}].txt'\
		.format(city, alpha, budget, mip_gap, top, bottom, firstSelect, secondSelectStep, num_finalLines, mip_focus, time_limit, saved_folder))
	
	# run once 
	m_MIP, obj_MIP, C4_toRemove_MIP, C5_toRemove_MIP, C8_toRemove_MIP, f_MIP, y_MIP, w_MIP, z_MIP, q_MIP, x_MIP, Y_MIP = maxRideMIP.getResult(node_list, od_pairs, ori_nodes, dest_nodes, lines_current, lines_edge, cost_edges, cost_lines_current, demand_dic, cap_lines_current,\
			budget, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, mip_gap, log_name, cap, mip_focus, time_limit, alpha, transit)
	
 
	
	# write gurobi optimal solutions
	path_name = result_path(city, 'heur_optSol[city@{}|alpha@{}|budget@{}|mip_gap@{}|top@{}|bottom@{}|firstSelect@{}|secondSelectStep@{}|num_finalLines@{}|mip_focus@{}|time_limit@{}|saved_folder@{}].txt'\
		.format(city, alpha, budget, mip_gap, top, bottom, firstSelect, secondSelectStep, num_finalLines, mip_focus, time_limit, saved_folder))
		
	with open(path_name, "w") as file:
		file.write(f"Objective Value = {m_MIP.ObjVal}\n")
		for v in m_MIP.getVars():
				# Write each line to the file
			file.write(f"{v.VarName} = {v.X}\n") 
   
	with open(path_name, 'a') as file:
		file.write('line index in full candidate set\n')
		for v in m_MIP.getVars():
			if 'var_Y' in v.VarName:
				item = re.findall("\[(.*?)\]", v.VarName)[0] 
				file.write(f"Y[{lines_finalLines_index_from_original[int(item)]}] = {v.X}\n")
    
 
 


if __name__ == "__main__":
    
    # create the parser
	parser = argparse.ArgumentParser()
 
 
	# add an argument
	parser.add_argument('--city', required=True) # Boston, Atlanta, Chicago 
	parser.add_argument('--gamma', required=True) # 5
	parser.add_argument('--alpha', required=True) # 1, 0.5, 0.3
	parser.add_argument('--cap', required=True) # 50
	parser.add_argument('--budget', required=True) 
	parser.add_argument('--flex_dist', required=True) # 1000 
	parser.add_argument('--mip_gap', required=True) # 0
	parser.add_argument('--unit_dist', required=True) # 4000  
	parser.add_argument('--top', required=True) # 0
	parser.add_argument('--bottom', required=True) # 600
	parser.add_argument('--saved_folder', required=True) # multimodal_gamma5_alpha1
	parser.add_argument('--firstSelect', required=True) # 200 
	parser.add_argument('--secondSelectStep', required=True) # 10
	parser.add_argument('--num_finalLines', required=True) # 50
	parser.add_argument('--saved_folder_new', required=True) # multimodal_gamma5_alpha1_MIP
	parser.add_argument('--mip_focus', required=True) # 0
	parser.add_argument('--time_limit', required=True) # 86400
	parser.add_argument("--transit", action="store_true") # (include for generating bus-only system, though this heuristic is defined for multi-modal systems)
	
	

	args = parser.parse_args()
	city = args.city
	gamma = float(args.gamma)
	alpha = float(args.alpha)
	cap = int(args.cap)
	budget = int(args.budget)
	flex_dist = int(args.flex_dist)
	mip_gap = float(args.mip_gap)
	unit_dist = int(args.unit_dist)
	top = int(args.top)
	bottom = int(args.bottom)
	saved_folder = args.saved_folder
	firstSelect = int(args.firstSelect)
	secondSelectStep = int(args.secondSelectStep)
	num_finalLines = int(args.num_finalLines)
	saved_folder_new = args.saved_folder_new
	mip_focus = int(args.mip_focus)
	time_limit = int(args.time_limit)
	transit = args.transit
 
	start = time.time()
	main(city=city, gamma=gamma, cap=cap, budget=budget, flex_dist=flex_dist, mip_gap=mip_gap, unit_dist=unit_dist, top=top, bottom=bottom, saved_folder=saved_folder,\
     firstSelect=firstSelect, secondSelectStep=secondSelectStep, num_finalLines=num_finalLines, saved_folder_new=saved_folder_new, mip_focus=mip_focus, time_limit=time_limit, alpha=alpha, transit=transit)
	total = time.time() - start

	print('total time: ', total)

