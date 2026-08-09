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
from networkx.algorithms import tournament
from itertools import permutations
import copy

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "root"))

import instance

import maxRideLP

from paths import output_path, save_pickle, data_path, load_pickle, output_dir



def savePickle(city, lines_current, dict_stlInd_uInd_current, pair_lineInd_dict_current, saved_folder):


	save_pickle(lines_current, output_path(city, saved_folder, 'lines_naive.pkl'))
	save_pickle(dict_stlInd_uInd_current, output_path(city, saved_folder, 'dict_stlInd_uInd_naive.pkl'))
	save_pickle(pair_lineInd_dict_current, output_path(city, saved_folder, 'pair_lineInd_dict_naive.pkl'))



def main(city, which_method, gamma, cap, budget, unit_dist, flex_dist, saved_folder, alpha, transit):

	output_dir(city, saved_folder)
 

	node_list, od_pairs, od_demands, lines, pair_lineInd_dict, dict_stlInd_uInd, ori_nodes, dest_nodes, demand_dic,\
	 cost_edges, cost_lines, cap_lines, bus_nodes, bus_edges = instance.main(city, which_method, gamma, cap, unit_dist, saved_folder, include_naive=False, len_naive=-1, cost_factor_naive=-1)
	


	lines_current = copy.deepcopy(lines)
	cost_lines_current = copy.deepcopy(cost_lines)
	cap_lines_current = copy.deepcopy(cap_lines)
	pair_lineInd_dict_current = copy.deepcopy(pair_lineInd_dict)
	dict_stlInd_uInd_current = copy.deepcopy(dict_stlInd_uInd)
	
	
	# relevant quantities for running LP
	lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = maxRideLP.LPPrep(lines_current, node_list, ori_nodes, bus_nodes, od_pairs, dict_stlInd_uInd_current)

	# run LP only once
	start_time = time.time()
	start_time_GPU = time.process_time()
	m, obj, C4_toRemove, C5_toRemove, C8_toRemove, C6_toFix, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP = maxRideLP.getResult(node_list, od_pairs, ori_nodes, dest_nodes, lines_current, lines_edge, cost_edges, cost_lines_current, demand_dic, cap_lines, budget, list_f_sluv,\
		   list_y_slv, list_w_slv, list_z_stv, list_z, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, alpha, transit)
	


	bus_network = nx.DiGraph()
	bus_network.add_nodes_from(bus_nodes)
	bus_network.add_edges_from(bus_edges)
	print('nx.is_strongly_connected(bus_network)', nx.is_strongly_connected(bus_network))


	edges_so_far = set()

	savePickle(city, lines_current, dict_stlInd_uInd_current, pair_lineInd_dict_current, saved_folder)

	counter = 0 
	bus_edges_temp = bus_edges.copy()

	while len(bus_edges_temp) > 2:

		counter += 1


		new_path = [] 

		first_edge = bus_edges_temp[0]


		new_path.append(first_edge[0])
		new_path.append(first_edge[1])
		bus_edges_temp.remove(first_edge)
		bus_edges_temp.remove((first_edge[1],first_edge[0]))

		path_nodes = set()
		path_nodes.add(first_edge[0])
		path_nodes.add(first_edge[1])

		edges_so_far.add(first_edge)
		edges_so_far.add((first_edge[1], first_edge[0]))


		flag = True 
		end = first_edge[1]
		while flag and len(bus_edges_temp) > 2:
			# find the next edge in the path
			found = False
			for item in list(bus_edges_temp):         
				# find a next edge to add
				if item[0] == end and item[1] not in path_nodes:
					new_path.append(item[1])
					bus_edges_temp.remove(item)
					bus_edges_temp.remove((item[1], item[0]))
					edges_so_far.add(item)
					edges_so_far.add((item[1], item[0]))
					path_nodes.add(item[1])
					end = item[1]
					found = True
					break

			# couldn't find a next edge this pass, stop growing the path
			if not found:
				flag = False

		
		new_path_rev = new_path.copy()
		new_path_rev.reverse()
		tour = new_path + new_path_rev[1:]



		if tour not in lines_current:

			# relevant quantities for the new line
			lines_current, cost_lines_current, cap_lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current = maxRideLP.newLineVars(tour, lines_current, cost_lines_current, cap_lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current, gamma, cap, cost_edges, node_list, flex_dist, od_pairs, unit_dist)


			# save variables
			savePickle(city, lines_current, dict_stlInd_uInd_current, pair_lineInd_dict_current, saved_folder)


			# re-generate LP (add additional vars and constraints)
			m, C4_toRemove, C5_toRemove, C8_toRemove, C6_toFix, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP, lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z\
	      		, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = maxRideLP.newLineLP(m, C4_toRemove, C5_toRemove, C8_toRemove, C6_toFix, cap, budget, od_pairs, node_list, lines_current, lines_edge, ori_nodes, cost_edges, tour, cost_lines_current, cap_lines_current, dict_stlInd_uInd_current, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z\
	      		, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP, alpha)


			

			# resolve LP 
			m.optimize()
			obj = m.ObjVal


	for item in bus_edges_temp:
		new_path = [item[0],item[1]]
		new_path_rev = [item[1],item[0]]
		tour = new_path + new_path_rev[1:]
		if tour not in lines_current:

			# relevant quantities for the new line
			lines_current, cost_lines_current, cap_lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current = maxRideLP.newLineVars(tour, lines_current, cost_lines_current, cap_lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current, gamma, cap, cost_edges, node_list, flex_dist, od_pairs, unit_dist)

			# save variables
			savePickle(city, lines_current, dict_stlInd_uInd_current, pair_lineInd_dict_current, saved_folder)


			# re-generate LP (add additional vars and constraints)
			m, C4_toRemove, C5_toRemove, C8_toRemove, C6_toFix, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP, lines_edge, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z\
	      		, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict = maxRideLP.newLineLP(m, C4_toRemove, C5_toRemove, C8_toRemove, C6_toFix, cap, budget, od_pairs, node_list, lines_current, lines_edge, ori_nodes, cost_edges, tour, cost_lines_current, cap_lines_current, dict_stlInd_uInd_current, list_f_sluv, list_y_slv, list_w_slv, list_z_stv, list_z\
	      		, bus_nodes, st_dict, ls_dict, svl_dict, z_uvs_dict, y_uvl_dict, f_LP, y_LP, w_LP, z_LP, q_LP, x_LP, Y_LP, alpha)



			# resolve LP 
			m.optimize()
			obj = m.ObjVal

		edges_so_far.add(item)
		edges_so_far.add((item[1],item[0]))

	




	return lines_current, cost_lines_current, cap_lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current


