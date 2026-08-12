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
import time

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "root"))


import instance
import maxRideLP
from paths import result_path





def createVars(m, node_list, od_pairs, ori_nodes, dest_nodes, lines, lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes, cap_lines, cap, transit):



	f = m.addVars(list_f_sluv, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_f')
	y = m.addVars(list_y_slv, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_y')
	w = m.addVars(list_w_slv, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_w')


	z = m.addVars(list_z_stv, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_z')
	list_dummy = set(list_z) - set(list_z_stv)
	list_dummy = list(list_dummy)
	z_dummy = m.addVars(list_dummy, lb = 0, ub = 0, vtype = GRB.CONTINUOUS)
	z.update(z_dummy)


	q = m.addVars(od_pairs, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_q')
	x = m.addVars(node_list, node_list, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_x')
	
	Y = m.addVars(len(lines), lb = 1, ub = float('inf'), vtype = GRB.SEMIINT, name = 'var_Y')
 
	for i in range(len(lines)):
		Y[i].LB = math.ceil(cap/cap_lines[i])
  
  
	if transit == True:
		for u in node_list:
			for v in node_list:
				if u != v:
					x[u,v].lb = 0
					x[u,v].ub = 0

	return f, y, w, z, q, x, Y





def createConstrs(m, node_list, od_pairs, ori_nodes, dest_nodes, lines, lines_edge, demand_dic, cap_lines, budget, cost_edges, cost_lines, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z,\
				  f, y, w, z, q, x, Y, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, alpha):
	
	m.addConstrs((q[ss,tt] + gp.quicksum(z[ss,tt,v] for v in bus_nodes) <= demand_dic[(ss,tt)] for (ss,tt) in od_pairs), name = 'a')

	
	
	m.addConstrs(gp.quicksum(w[s,l,v] for (s,l,v) in [(ss,ll,vv)] if (s,l,v) in list_w_slv) + gp.quicksum(f[ss,ll,vv,edge[1]] for edge in lines_edge[ll] if edge[0] == vv) \
		   - gp.quicksum(y[s,l,v] for (s,l,v) in [(ss,ll,vv)] if (s,l,v) in list_y_slv) - gp.quicksum(f[ss,ll,edge[0],vv] for edge in lines_edge[ll] if edge[1] == vv) \
			== 0 for ll in range(len(lines)) for ss in ls_dict[ll] for vv in set(lines[ll]))

	
	C4_toRemove = m.addConstrs((gp.quicksum(z[ss,t,vv] for t in st_dict[ss]) -gp.quicksum(w[ss,l,vv] for l in svl_dict[ss,vv]) == 0 for ss in ori_nodes for vv in bus_nodes), name = 'p')
	

	
	C5_toRemove = m.addConstrs((-x[uu,vv] + gp.quicksum(q[u,v] for (u,v) in [(uu,vv)] if (u,v) in od_pairs) + gp.quicksum(z[s,vv,uu] for s in z_uvs_dict[uu,vv]) + gp.quicksum(y[uu,l,vv]\
												        for l in y_uvl_dict[uu,vv]) <=0 for uu in node_list for vv in node_list), name = 'k')
	
	m.addConstrs(-cap_lines[ll]*Y[ll] + gp.quicksum(f[s,ll,edge[0], edge[1]] for s in ls_dict[ll]) <= 0 for ll in range(len(lines)) for edge in lines_edge[ll])
	
	

	m.addConstrs((gp.quicksum(x[v,uu] for v in node_list if v != uu) - gp.quicksum(x[uu,v] for v in node_list if v != uu) == 0 for uu in node_list), name='w')



	C8_toRemove = m.addConstr(gp.quicksum(x[u,v]*cost_edges[(u,v)]*alpha for u in node_list for v in node_list if v != u)\
		+ gp.quicksum(cost_lines[l]*Y[l] for l in range(len(lines))) <= budget, name = 'bud')

	return C4_toRemove, C5_toRemove, C8_toRemove


	



	



def getResult(node_list, od_pairs, ori_nodes, dest_nodes, lines, lines_edge, cost_edges, cost_lines, demand_dic, cap_lines, budget, list_f_sluv,\
	       list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, mip_gap, log_name, cap, mip_focus, time_limit, alpha, transit):
	start_time = time.time()
	m = gp.Model('model')
	m.Params.outputFlag = 1
	m.Params.MIPGap = mip_gap
	m.Params.LogFile = log_name
	m.Params.MIPFocus = mip_focus 
	m.Params.timeLimit = time_limit 

	# create variables
	f, y, w, z, q, x, Y = createVars(m, node_list, od_pairs, ori_nodes, dest_nodes, lines, lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes, cap_lines, cap, transit)
	

	# create objective
	obj_func = gp.quicksum(q[s,t] + gp.quicksum(z[s,t,v] for v in bus_nodes) for (s,t) in od_pairs)

	m.setObjective(obj_func, GRB.MAXIMIZE)
	

	# create constraints 
	C4_toRemove, C5_toRemove, C8_toRemove = createConstrs(m, node_list, od_pairs, ori_nodes, dest_nodes, lines, lines_edge, demand_dic, cap_lines, budget, cost_edges, cost_lines, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z,\
				  f, y, w, z, q, x, Y, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, alpha)


	m.optimize()
	
		

	return m, m.ObjVal, C4_toRemove, C5_toRemove, C8_toRemove, f, y, w, z, q, x, Y



def main(city, gamma, alpha, cap, unit_dist, saved_folder, budget, mip_gap, mip_focus, time_limit, transit): 
    
    
	node_list, od_pairs, od_demands, lines, pair_lineInd_dict, dict_stlInd_uInd, ori_nodes, dest_nodes, demand_dic,\
	cost_edges, cost_lines, cap_lines, bus_nodes, bus_edges = instance.main(city, 'final', gamma, cap, unit_dist, saved_folder, include_naive=False, len_naive=-1, cost_factor_naive=-1)

	# relevant quantities for running 
	lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = maxRideLP.LPPrep(lines, node_list, ori_nodes, bus_nodes, od_pairs, dict_stlInd_uInd)
	
	# write gurobi log file
	log_name = result_path(city, 'heur_log[city@{}|alpha@{}|budget@{}|mip_gap@{}|mip_focus@{}|time_limit@{}].txt'\
		.format(city, alpha, budget, mip_gap, mip_focus, time_limit))
	

	# run once 
	m_MIP, obj_MIP, C4_toRemove_MIP, C5_toRemove_MIP, C8_toRemove_MIP, f_MIP, y_MIP, w_MIP, z_MIP, q_MIP, x_MIP, Y_MIP = getResult(node_list, od_pairs, ori_nodes, dest_nodes, lines, lines_edge, cost_edges, cost_lines, demand_dic, cap_lines,\
			budget, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, mip_gap, log_name, cap, mip_focus, time_limit, alpha, transit)
	



if __name__ == "__main__":
    
    # create the parser
	parser = argparse.ArgumentParser()

 
	# add an argument
	parser.add_argument('--city', required=True) # Boston, Chicago, Atlanta
	parser.add_argument('--gamma', required=True) # 5
	parser.add_argument('--alpha', required=True) # 1, 0.5, 0.3
	parser.add_argument('--cap', required=True) # 50
	parser.add_argument('--unit_dist', required=True) # 4000
	parser.add_argument('--saved_folder', required=True) # transit_gamma5_alpha1
	parser.add_argument('--budget', required=True)
	parser.add_argument('--mip_gap', required=True) # 0
	parser.add_argument('--mip_focus', required=True) # 0
	parser.add_argument('--time_limit', required=True) # 86400
	parser.add_argument("--transit", action="store_true") # (include for running bus-only system MIP)
		
	
	

	args = parser.parse_args()
	city = args.city
	gamma = float(args.gamma)
	alpha = float(args.alpha)
	cap = int(args.cap)
	unit_dist = int(args.unit_dist)
	saved_folder = args.saved_folder
	budget = int(args.budget)
	mip_gap = float(args.mip_gap)
	mip_focus = int(args.mip_focus)
	time_limit = int(args.time_limit)
	transit = args.transit
	 
 
 
 
	start = time.time()
	main(city, gamma, alpha, cap, unit_dist, saved_folder, budget, mip_gap, mip_focus, time_limit, transit)
	total = time.time() - start
	print('total time: ', total)
