import gurobipy as gp
from gurobipy import GRB
import time
import networkx as nx
import dill as pickle
import collections
import numpy as np
import re
import copy


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "root"))

from paths import data_path, load_pickle


def createVars(m, node_list, od_pairs, ori_nodes, dest_nodes, lines, lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes):



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
	Y = m.addVars(len(lines), lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_Y')
 


	return f, y, w, z, q, x, Y





def createConstrs(m, node_list, od_pairs, ori_nodes, dest_nodes, lines, lines_edge, demand_dic, cap_lines, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z,\
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
	
	C6_toFix = m.addConstrs((cap_lines[ll]*Y[ll] - gp.quicksum(f[s,ll,edge[0], edge[1]] for s in ls_dict[ll]) >= 0 for ll in range(len(lines)) for edge in lines_edge[ll]), name = 'constraint_6')
	
	
	

	m.addConstrs((gp.quicksum(x[v,uu] for v in node_list if v != uu) - gp.quicksum(x[uu,v] for v in node_list if v != uu) == 0 for uu in node_list), name = 'w')

	
	return C4_toRemove, C5_toRemove, C6_toFix


	



def getResult(node_list, od_pairs, ori_nodes, dest_nodes, lines, lines_edge, cost_edges, cost_lines, demand_dic, cap_lines, list_f_sluv,\
	       list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, demand_perc):
	start_time = time.time()
	m = gp.Model('model')
	m.Params.outputFlag = 1
	m.Params.SolutionTarget = 1
	m.Params.Method = 2

	# create variables
	f, y, w, z, q, x, Y = createVars(m, node_list, od_pairs, ori_nodes, dest_nodes, lines, lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes)
	
	# create objective
	obj_func = gp.quicksum(x[u,v]*cost_edges[(u,v)] for u in node_list for v in node_list if v != u)\
		+gp.quicksum(cost_lines[l]*Y[l] for l in range(len(lines_edge)))
	m.setObjective(obj_func, GRB.MINIMIZE)
	
	# create constraints 
	C4_toRemove, C5_toRemove, C6_toFix = createConstrs(m, node_list, od_pairs, ori_nodes, dest_nodes, lines, lines_edge, demand_dic, cap_lines, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z,\
				  f, y, w, z, q, x, Y, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, demand_perc)

	
	m.optimize()
	

	return m, m.ObjVal, C4_toRemove, C5_toRemove, C6_toFix, f, y, w, z, q, x, Y

def LPPrep(lines, node_list, ori_nodes, bus_nodes, od_pairs, dict_stlInd_uInd):
    
	lines_edge = []
	for line in lines:
		line_edge = []
		for i in range(len(line)-1):
			line_edge.append((line[i], line[i+1]))
		lines_edge.append(line_edge)




	list_y_slv = set() # create vars
	list_w_slv = set() # create vars
	list_z_stv = set() # actually useful z vars
	list_z = set() # create vars
	list_f_sluv = set() # create vars

	ls_dict = {} # create constrs
	for l in range(len(lines)):
		ls_dict[l] = set()


	svl_dict = {} # create constrs
	for s in ori_nodes:
		for v in bus_nodes:
			svl_dict[s,v] = set()
   
   


	z_uvs_dict = {} # create constrs
	y_uvl_dict = {} # create constrs
	for u in node_list:
		for v in node_list:
			z_uvs_dict[u,v] = set()
			y_uvl_dict[u,v] = set()

	for key, value in dict_stlInd_uInd.items():
		s = key[0]
		t = key[1]
		l = key[2]

		get_on_index = value[0]
		get_off_index = value[1]

		if len(get_on_index) > 0:
			for index in get_on_index:
				get_on_node = lines[l][index]
				list_y_slv.add((s,l,get_on_node))

				y_uv_set = y_uvl_dict[s,get_on_node]
				y_uv_set.add(l)
				y_uvl_dict[s,get_on_node] = y_uv_set



		if len(get_off_index) > 0:
			for index in get_off_index:
				get_off_node = lines[l][index]
				list_w_slv.add((s,l,get_off_node))
				list_z_stv.add((s,t,get_off_node))
     
				sv_set = svl_dict[s,get_off_node]
				sv_set.add(l)
				svl_dict[s,get_off_node] = sv_set

				z_uv_set = z_uvs_dict[get_off_node, t]
				z_uv_set.add(s)
				z_uvs_dict[get_off_node, t] = z_uv_set

		l_set = ls_dict[l]
		l_set.add(s)
		ls_dict[l] = l_set


	for (s,t) in od_pairs:
		for v in bus_nodes:
			list_z.add((s,t,v))

	for l in range(len(lines)):
		l_set = ls_dict[l]
		for s in l_set:
			for item in lines_edge[l]:
				list_f_sluv.add((s,l,item[0],item[1]))
	

	list_y_slv = list(list_y_slv)
	list_w_slv = list(list_w_slv)
	list_z_stv = list((list_z_stv))  
	list_z = list(list_z)  
	list_f_sluv = list(list_f_sluv)

	st_dict = {}
	for s in ori_nodes:
		dest = set()
		for item in od_pairs:
			if item[0] == s:
				dest.add(item[1])
		st_dict[s] = dest


	return lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict


def newLineLP(m, C4_toRemove, C5_toRemove, C6_toFix, cap, od_pairs, node_list, new_lines, lines_edge, ori_nodes, cost_edges, new_line, new_cost_lines, new_cap_lines, new_dict_stlInd_uInd, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z\
	      , bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, f, y, w, z, q, x, Y):
	
	ll = len(new_lines) - 1 # index of the newly added line

	line_edge = []
	for i in range(len(new_line)-1):
		line_edge.append((new_line[i], new_line[i+1]))
	lines_edge.append(line_edge)





	# redefine lists & dicts for loading variables & update constrs
	list_f_sluv = set(list_f_sluv)
	list_y_slv = set(list_y_slv)
	list_w_slv = set(list_w_slv)
	list_z_stv = set(list_z_stv)

	list_f_sluv_new = set()
	list_y_slv_new = set()
	list_w_slv_new = set()
	list_z_stv_new = set()


	# update dicts for constrs
	ls_dict[ll] = set()


	# fill in
	
	for key, value in new_dict_stlInd_uInd.items():
		s = key[0]
		t = key[1]
		l = key[2]

		if l == ll: 
			get_on_index = value[0]
			get_off_index = value[1]

			if len(get_on_index) > 0:
				for index in get_on_index:
					get_on_node = new_lines[l][index]
					list_y_slv.add((s,l,get_on_node))
					list_y_slv_new.add((s,l,get_on_node))

					y_uv_set = y_uvl_dict[s,get_on_node]
					y_uv_set.add(l)
					y_uvl_dict[s,get_on_node] = y_uv_set


			if len(get_off_index) > 0:
				for index in get_off_index:
					get_off_node = new_lines[l][index]
					list_w_slv.add((s,l,get_off_node))
					list_w_slv_new.add((s,l,get_off_node))
					list_z_stv.add((s,t,get_off_node))
					list_z_stv_new.add((s,t,get_off_node))

					sv_set = svl_dict[s,get_off_node]
					sv_set.add(l)
					svl_dict[s,get_off_node] = sv_set

					z_uv_set = z_uvs_dict[get_off_node, t]
					z_uv_set.add(s)
					z_uvs_dict[get_off_node, t] = z_uv_set

			l_set = ls_dict[l]
			l_set.add(s)
			ls_dict[l] = l_set

	l_set = ls_dict[ll]
	for s in l_set:
		for item in lines_edge[ll]:
			list_f_sluv.add((s,ll,item[0],item[1]))

			list_f_sluv_new.add((s,ll,item[0],item[1]))


	list_f_sluv = list(list_f_sluv)
	list_y_slv = list(list_y_slv)
	list_w_slv = list(list_w_slv)
	list_z_stv = list(list_z_stv)


	list_f_sluv_new = list(list_f_sluv_new)
	list_y_slv_new = list(list_y_slv_new)
	list_w_slv_new = list(list_w_slv_new)
	list_z_stv_new = list(list_z_stv_new)

	# reload variables 
	m.update()


	f_new = m.addVars(list_f_sluv_new, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_f')
	f.update(f_new)

	y_new = m.addVars(list_y_slv_new, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_y')
	y.update(y_new)

	w_new = m.addVars(list_w_slv_new, lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, name = 'var_w')
	w.update(w_new)

	Y.update({ll: m.addVar(lb = 0, ub = float('inf'), vtype = GRB.CONTINUOUS, obj = new_cost_lines[ll], name = 'var_Y[{}]'.format(ll))}) # add obj coefficient to update objective function



	for item in list_z_stv_new:
		z[item].ub = float('inf') # change upper bound of previously dummy vars


	
	# constraint 3
	m.addConstrs((gp.quicksum(w[s,l,v] for (s,l,v) in [(ss,ll,vv)] if (s,l,v) in list_w_slv) + gp.quicksum(f[ss,ll,vv,edge[1]] for edge in lines_edge[ll] if edge[0] == vv)\
		    - gp.quicksum(y[s,l,v] for (s,l,v) in [(ss,ll,vv)] if (s,l,v) in list_y_slv) - gp.quicksum(f[ss,ll,edge[0],vv] for edge in lines_edge[ll] if edge[1] == vv) \
			  == 0 for ll in [len(new_lines) - 1]for ss in ls_dict[ll] for vv in set(new_lines[ll])), name = 'constraint_3') 
	
	# constraint 4
	m.remove(C4_toRemove)
	C4_toRemove = m.addConstrs((gp.quicksum(z[ss,t,vv] for t in st_dict[ss])-gp.quicksum(w[ss,l,vv] for l in svl_dict[ss,vv]) == 0 for ss in ori_nodes for vv in bus_nodes), name ='p')
	

	# constraint 5
	m.remove(C5_toRemove)
	C5_toRemove = m.addConstrs((x[uu,vv] - gp.quicksum(q[u,v] for (u,v) in [(uu,vv)] if (u,v) in od_pairs) - gp.quicksum(z[s,vv,uu] for s in z_uvs_dict[uu,vv]) - gp.quicksum(y[uu,l,vv]\
												        for l in y_uvl_dict[uu,vv]) >= 0 for uu in node_list for vv in node_list), name ='k')
	
	# constraint 6
	C6_toFix = m.addConstrs((-new_cap_lines[ll]*Y[ll] + gp.quicksum(f[s, ll, edge[0], edge[1]] for ll in [len(new_lines) - 1] for s in ls_dict[ll]) <= 0 for edge in lines_edge[ll]), name='constraint_6')
	


	return m, C4_toRemove, C5_toRemove, C6_toFix, f, y, w, z, q, x, Y, lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z\
	      , st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict


def newLineVars(new_line, lines, cost_lines, cap_lines, pair_lineInd_dict, dict_stlInd_uInd, gamma, cap, cost_edges, node_list, flex_dist, od_pairs, unit_dist):

	# update lines
	new_lines = lines.copy()
	new_lines.append(new_line)

	# add cost and capacity to new lines 
	new_cost_lines = cost_lines.copy()
	new_cap_lines = cap_lines.copy()
	temp_cost = 0
	new_lineInd = new_lines.index(new_line)
	for j in range(len(new_lines[new_lineInd]) - 1):
		temp_cost = temp_cost + cost_edges[new_lines[new_lineInd][j], new_lines[new_lineInd][j+1]]
	new_cost_lines.append(gamma * unit_dist) 
	new_cap_lines.append(cap*float(unit_dist / temp_cost))
	


	# update pair_lineInd_dict_temp, pair_lineInd_dict_current
	new_pair_lineInd_dict = copy.deepcopy(pair_lineInd_dict)
	new_dict_stlInd_uInd = copy.deepcopy(dict_stlInd_uInd)

	dict_nbh = {}
	for u in node_list:
		dict_nbh[u] = []

	for u in node_list:
		for v in node_list:
			if cost_edges[u,v] <= flex_dist:
				value = dict_nbh[u]
				value.append(v)
				dict_nbh[u] = value

	for path in [new_lines[-1]]:
		for pair in od_pairs:
			get_on_index = []
			get_off_index = []

			for i in range(len(path)): # cannot do "for stop in path" because then "get_on_index.append(path.index(stop))" would mix the two indices 
				if path[i] in dict_nbh[pair[0]]:
					get_on_index.append(i)
				if pair[1] in dict_nbh[path[i]]:
					get_off_index.append(i)

			if len(get_on_index) != 0 or len(get_off_index) != 0:
				if pair in new_pair_lineInd_dict:
					value = new_pair_lineInd_dict[pair]	
					value.append(new_lines.index(path))
					new_pair_lineInd_dict[pair] = value
				else:
					new_pair_lineInd_dict[pair] = [new_lines.index(path)]

				new_dict_stlInd_uInd[(pair[0], pair[1], new_lines.index(path))] = (get_on_index, get_off_index)

	pair_lineInd_dict_temp = {k:list(set(v)) for (k,v) in new_pair_lineInd_dict.items() if len(v) > 0}
	new_pair_lineInd_dict = pair_lineInd_dict_temp



	return new_lines, new_cost_lines, new_cap_lines, new_pair_lineInd_dict, new_dict_stlInd_uInd


def getShadowPrice(m, consrtName_string):
	var_current = {}
	if m.status == gp.GRB.OPTIMAL:
		for constr in m.getConstrs():
			if constr.ConstrName.split('[')[0] == consrtName_string:
				if consrtName_string == "a" or consrtName_string == "k" or consrtName_string == "p": 
					item = re.findall("\[(.*?)\]", constr.ConstrName)[0] 
					first, second = item.split(',')
					var_current[int(first), int(second)] = constr.Pi
				elif consrtName_string == "bud":
					var_current[0] = constr.Pi 
				# elif consrtName_string == "w":
				# 	item = re.findall("\[(.*?)\]", constr.ConstrName)[0]
				# 	print(item) 
				# 	print(constr.Pi)

	return var_current





def extract_lines(city, demand_dic, gamma, cap, flex_dist, unit_dist, some_lines):

	taxi_nodes = load_pickle(data_path(city, 'taxi_nodes.pkl'))
		

	node_list = taxi_nodes.copy()


	cost_edges = load_pickle(data_path(city, 'cost_edges.pkl'))


	od_pairs = [] # in order to match indicies of od_demands
	for key, value in demand_dic.items():
		if key[0] != key[1]:
			od_pairs.append((key[0], key[1]))
	
 


	pair_lineInd_dict = {}
	dict_stlInd_uInd = {}

	dict_nbh = {}
	for u in node_list:
		dict_nbh[u] = [u]

	for u in node_list:
		for v in node_list:
			if cost_edges[u,v] <= flex_dist:
				value = dict_nbh[u]
				value.append(v)
				dict_nbh[u] = value

	for path in some_lines:
		for pair in od_pairs:
			get_on_index = []
			get_off_index = []

			for i in range(len(path)):  
				if path[i] in dict_nbh[pair[0]]:
					get_on_index.append(i)
				if pair[1] in dict_nbh[path[i]]:
					get_off_index.append(i)

			if len(get_on_index) != 0 or len(get_off_index) != 0:
				if pair in pair_lineInd_dict:
					value = pair_lineInd_dict[pair]	
					value.append(some_lines.index(path))
					pair_lineInd_dict[pair] = value
				else:
					pair_lineInd_dict[pair] = [some_lines.index(path)]

				dict_stlInd_uInd[(pair[0], pair[1], some_lines.index(path))] = (get_on_index, get_off_index)

	pair_lineInd_dict_temp = {k:v for (k,v) in pair_lineInd_dict.items() if len(v) > 0}
	pair_lineInd_dict = pair_lineInd_dict_temp
 
 
	# cost_lines and cap_lines
	cost_lines = [0]*len(some_lines)
	cap_lines = [0]*len(some_lines)
	for l in range(len(some_lines)):
		temp_cost = 0
		for j in range(len(some_lines[l]) - 1):
			temp_cost = temp_cost + cost_edges[(some_lines[l][j],some_lines[l][j+1])]
		# cost_lines[l] = gamma*temp_cost # list of cost for each line, adjust r value here
		cost_lines[l] = gamma * unit_dist
		cap_lines[l] = cap*float(unit_dist / temp_cost)
	

	return pair_lineInd_dict, dict_stlInd_uInd, cost_lines, cap_lines


