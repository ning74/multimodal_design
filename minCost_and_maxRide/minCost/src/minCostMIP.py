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


def createVars(m, node_list, od_pairs, ori_nodes, dest_nodes, lines, lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, cap_lines, cap):



	f = m.addVars(list_f_sluv, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_f')
	y = m.addVars(list_y_slv, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_y')
	w = m.addVars(list_w_slv, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_w')
	
	z = m.addVars(list_z_stv, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_z')
	list_dummy = set(list_z) - set(list_z_stv)
	list_dummy = list(list_dummy)
	z_dummy = m.addVars(list_dummy, lb = 0, ub = 0, vtype = GRB.CONTINUOUS, name = 'var_z')
	z.update(z_dummy)


	q = m.addVars(od_pairs, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_q')
	x = m.addVars(node_list, node_list, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_x')
	# Y = m.addVars(len(lines), lb = 0, ub = float('inf'), vtype = GRB.INTEGER)
  
	Y = m.addVars(len(lines), lb = 1, ub = float('inf'), vtype = GRB.SEMIINT, name = 'var_Y')
 
	for i in range(len(lines)):
		Y[i].lb = math.ceil(cap/cap_lines[i])

	return f, y, w, z, q, x, Y





def createConstrs(m, node_list, od_pairs, ori_nodes, lines, lines_edge, demand_dic, cap_lines, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z,\
				  f, y, w, z, q, x, Y, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, demand_perc):
	
	constraint_time = time.process_time()
	
	# change constraint 1 to satisfy partial demand
	m.addConstrs((q[ss,tt] + gp.quicksum(z[ss,tt,v] for v in bus_nodes) <= demand_dic[(ss,tt)] for (ss,tt) in od_pairs), name = 'a')
	m.addConstr((gp.quicksum(q[ss,tt] + gp.quicksum(z[ss,tt,v] for v in bus_nodes) for (ss,tt) in od_pairs) >= demand_perc * gp.quicksum(demand_dic[(ss,tt)] for (ss,tt) in od_pairs)), name = 'a_star')
	
	
	

	m.addConstrs((gp.quicksum(w[s,l,v] for (s,l,v) in [(ss,ll,vv)] if (s,l,v) in list_w_slv) + gp.quicksum(f[ss,ll,vv,edge[1]] for edge in lines_edge[ll] if edge[0] == vv) \
		  - gp.quicksum(y[s,l,v] for (s,l,v) in [(ss,ll,vv)] if (s,l,v) in list_y_slv) - gp.quicksum(f[ss,ll,edge[0],vv] for edge in lines_edge[ll] if edge[1] == vv)\
	        == 0 for ll in range(len(lines)) for ss in ls_dict[ll] for vv in set(lines[ll])), name = 'constraint_3')

	
	C4_toRemove = m.addConstrs((gp.quicksum(z[ss,t,vv] for t in st_dict[ss]) -gp.quicksum(w[ss,l,vv] for l in svl_dict[ss,vv]) == 0 for ss in ori_nodes for vv in bus_nodes), name = 'p')
	
	
	

	
	C5_toRemove = m.addConstrs((x[uu,vv] - gp.quicksum(q[u,v] for (u,v) in [(uu,vv)] if (u,v) in od_pairs) - gp.quicksum(z[s,vv,uu] for s in z_uvs_dict[uu,vv]) - gp.quicksum(y[uu,l,vv]\
												        for l in y_uvl_dict[uu,vv]) >=0 for uu in node_list for vv in node_list), name = 'k')
	
	
	m.addConstrs((cap_lines[ll]*Y[ll] - gp.quicksum(f[s,ll,edge[0], edge[1]] for s in ls_dict[ll]) >= 0 for ll in range(len(lines)) for edge in lines_edge[ll]), name = 'constraint_6')

	m.addConstrs((gp.quicksum(x[v,uu] for v in node_list if v != uu) - gp.quicksum(x[uu,v] for v in node_list if v != uu) == 0 for uu in node_list), name = 'w')
 
 
	return C4_toRemove, C5_toRemove


	



def getResult(node_list, od_pairs, ori_nodes, dest_nodes, lines, lines_edge, cost_edges, cost_lines, demand_dic, cap_lines, list_f_sluv,\
	       list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, mip_gap, log_name, cap, mip_focus, time_limit, demand_perc):
	start_time = time.time()
	m = gp.Model('model')
	m.Params.outputFlag = 1
	m.Params.MIPGap = mip_gap
	m.Params.LogFile = log_name
	m.Params.MIPFocus = mip_focus
	m.Params.timeLimit = time_limit

	# create variables
	f, y, w, z, q, x, Y= createVars(m, node_list, od_pairs, ori_nodes, dest_nodes, lines, lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, cap_lines, cap)
	
	# create objective
	obj_func = gp.quicksum(x[u,v]*cost_edges[(u,v)] for u in node_list for v in node_list if v != u)\
		+gp.quicksum(cost_lines[l]*Y[l] for l in range(len(lines)))
	m.setObjective(obj_func, GRB.MINIMIZE)
	
	# create constraints 
	C4_toRemove, C5_toRemove = createConstrs(m, node_list, od_pairs, ori_nodes, lines, lines_edge, demand_dic, cap_lines, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z,\
				  f, y, w, z, q, x, Y, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, demand_perc)

	
	
	m.optimize()
	
	return m, m.ObjVal, C4_toRemove, C5_toRemove, f, y, w, z, q, x, Y


