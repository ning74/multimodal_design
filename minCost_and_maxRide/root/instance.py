import dill as pickle

from paths import data_path, output_path, load_pickle


def main(city, which_method, gamma, cap, unit_dist, saved_folder, include_naive, len_naive, cost_factor_naive):
	lines = []
	pair_lineInd_dict = {}
	dict_stlInd_uInd = {}

	if which_method != 'base':
		name = which_method
		lines = load_pickle(output_path(city, saved_folder, f'lines_{name}.pkl', create=False))
		pair_lineInd_dict = load_pickle(output_path( city, saved_folder, f'pair_lineInd_dict_{name}.pkl', create=False))
		dict_stlInd_uInd = load_pickle(output_path(city, saved_folder, f'dict_stlInd_uInd_{name}.pkl', create=False))


	# preprocessed network/demand data, produced by data/process_demand.py
	demand_dic = load_pickle(data_path(city, 'demand_taxi_dic_dense.pkl'))
	taxi_nodes = load_pickle(data_path(city, 'taxi_nodes.pkl'))
	bus_nodes = load_pickle(data_path(city, 'bus_nodes_dense.pkl'))
	bus_edges = load_pickle(data_path(city, 'bus_edges_1.25mile_dense.pkl'))
	cost_edges = load_pickle(data_path(city, 'cost_edges.pkl'))

	node_list = taxi_nodes.copy()

	od_pairs = []  # in order to match indices of od_demands
	od_demands = []  # upper bound for direct-taxi variable q
	ori_nodes = set()
	dest_nodes = set()
	for key, value in demand_dic.items():
		if key[0] != key[1]:
			ori_nodes.add(key[0])
			dest_nodes.add(key[1])
			od_pairs.append((key[0], key[1]))
			od_demands.append(value)

	ori_nodes = list(ori_nodes)
	dest_nodes = list(dest_nodes)

	cost_lines = [0] * len(lines)
	cap_lines = [0] * len(lines)
	for l in range(len(lines)):
		temp_cost = sum(cost_edges[(lines[l][j], lines[l][j + 1])] for j in range(len(lines[l]) - 1))
		cost_lines[l] = gamma * unit_dist
		cap_lines[l] = cap * float(unit_dist / temp_cost)

	if include_naive and len(lines) >= len_naive:
		for i in range(len_naive):
			cost_lines[i] = cost_factor_naive * cost_lines[i]

	return node_list, od_pairs, od_demands, lines, pair_lineInd_dict, dict_stlInd_uInd, ori_nodes, dest_nodes,\
			demand_dic, cost_edges, cost_lines, cap_lines, bus_nodes, bus_edges

