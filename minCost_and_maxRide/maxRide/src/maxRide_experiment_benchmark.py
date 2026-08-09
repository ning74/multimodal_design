import pandas as pd
import dill as pickle
import folium
import networkx as nx
import osmnx as ox
# from geopy import distance
import copy
import random
import numpy as np
import time
import argparse

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "root"))


from paths import data_path, output_path, load_pickle, save_pickle, output_dir
 


def savePickle(city, lines_current_random, pair_lineInd_dict_random, dict_stlInd_uInd_random, saved_folder, pair_lineInd_dict_random_transit, dict_stlInd_uInd_random_transit, saved_folder_transit):
    save_pickle(lines_current_random, output_path(city, saved_folder, 'lines_final.pkl'))
    save_pickle(pair_lineInd_dict_random, output_path(city, saved_folder, 'pair_lineInd_dict_final.pkl'))
    save_pickle(dict_stlInd_uInd_random, output_path(city, saved_folder, 'dict_stlInd_uInd_final.pkl'))

    save_pickle(lines_current_random, output_path(city, saved_folder_transit, 'lines_final.pkl'))
    save_pickle(pair_lineInd_dict_random_transit, output_path(city, saved_folder_transit, 'pair_lineInd_dict_final.pkl'))
    save_pickle(dict_stlInd_uInd_random_transit, output_path(city, saved_folder_transit, 'dict_stlInd_uInd_final.pkl'))



def line_auxiliary(demand_dic, node_list, cost_edges, lines_current_random, flex_dist):
    od_pairs = [] # in order to match indecies of od_demands
    for key, value in demand_dic.items():
        if key[0] != key[1]:
            od_pairs.append((key[0], key[1]))
        
    # define pair_lineInd_dict_random, pair_lineInd_dict_random

    pair_lineInd_dict_random = {}
    dict_stlInd_uInd_random = {}

    dict_nbh = {}
    for u in node_list:
        dict_nbh[u] = []

    for u in node_list:
        for v in node_list:
            if cost_edges[u,v] <= flex_dist:
                value = dict_nbh[u]
                value.append(v)
                dict_nbh[u] = value

    for path in lines_current_random:
        for pair in od_pairs:
            get_on_index = []
            get_off_index = []

            for i in range(len(path)): # cannot do "for stop in path" becuase then "get_on_index.append(path.index(stop))" would mix the two indices 
                if path[i] in dict_nbh[pair[0]]:
                    get_on_index.append(i)
                if pair[1] in dict_nbh[path[i]]:
                    get_off_index.append(i)

            if len(get_on_index) != 0 or len(get_off_index) != 0:
                if pair in pair_lineInd_dict_random:
                    value = pair_lineInd_dict_random[pair]	
                    value.append(lines_current_random.index(path))
                    pair_lineInd_dict_random[pair] = value
                else:
                    pair_lineInd_dict_random[pair] = [lines_current_random.index(path)]

                dict_stlInd_uInd_random[(pair[0], pair[1], lines_current_random.index(path))] = (get_on_index, get_off_index)

    pair_lineInd_dict_temp = {k:v for (k,v) in pair_lineInd_dict_random.items() if len(v) > 0}
    pair_lineInd_dict_random = pair_lineInd_dict_temp
    
    return pair_lineInd_dict_random, dict_stlInd_uInd_random
                       
                    

def generate_new_line(bus_nodes, cost_edges, min_start_end_distance, detour_skeleton, bus_network, max_travel, max_travel_actual, detour_coeff):
        
    remaining_stops = list(bus_nodes)
    n = len(remaining_stops)
    min_distance = min_start_end_distance


    start_index = random.randint(0,n-1)
    start = remaining_stops.pop(start_index)

    distance_start_end = 0
    i = 0
    while i <= 1000:
        i+=1
        end_index = random.randint(0,n-2)
        end = remaining_stops[end_index]
            
        if cost_edges[start,end] >= min_distance and cost_edges[start,end] <= max_travel:
            routeA = nx.shortest_path(bus_network, start, end, weight='length')
            if len(set(routeA)) == len(routeA):
                break
            
    end = remaining_stops.pop(end_index)

        
    i = 0
    while i <= 1000:
        i+=1
        inter_index = random.randint(0,n-3)
        inter = remaining_stops[inter_index]
        if cost_edges[start, inter]  + cost_edges[inter, end] <= cost_edges[start, end] * detour_skeleton:
            routeB = nx.shortest_path(bus_network, start, inter, weight='length')
            routeC = nx.shortest_path(bus_network, inter, end, weight='length')
            unionBC = set(routeB) | set(routeC)
            if len(set(unionBC)) == len(set(routeB)) + len(set(routeC)) - 1 and len(set(routeB)) == len(routeB) and len(set(routeC)) == len(routeC):
                break
            
    inter = remaining_stops.pop(inter_index)
    

    i = 0
    while i <= 1000:
        i+=1
        inter_index_2 = random.randint(0,n-4)
        inter_2 = remaining_stops[inter_index_2]
        if cost_edges[inter, inter_2]  + cost_edges[inter_2, end] <= cost_edges[inter, end] * detour_skeleton:
            routeD = nx.shortest_path(bus_network, inter, inter_2, weight='length')
            routeE = nx.shortest_path(bus_network, inter_2, end, weight='length')
            unionDE = set(routeD) | set(routeE)
            if len(set(unionDE)) == len(set(routeD)) + len(set(routeE)) - 1 and len(set(routeD)) == len(routeD) and len(set(routeE)) == len(routeE):
                break
    inter_2 = remaining_stops.pop(inter_index_2)


    # if we use g here we will remove extra nodes not in taxi network later
    route1 = nx.shortest_path(bus_network, start, inter, weight='length')
    route2 = nx.shortest_path(bus_network, inter, inter_2, weight='length')
    route3 = nx.shortest_path(bus_network, inter_2, end, weight='length')
    

    route = route1 + route2[1:] + route3[1:] 


    # we remove nodes not in taxi network, if needed
    route_temp = []
    for i in range(len(route)):
        if route[i] in bus_nodes:
            route_temp.append(route[i])
    route = copy.deepcopy(route_temp)
    


    # output part of the line if two-way line, not longer than the required max length
    route_temp = [route[0]]
    total_length = 0
    for i in range(len(route)-1):
        if total_length + cost_edges[route[i], route[i+1]] + cost_edges[route[i+1], route[i]] <= max_travel_actual:
            route_temp.append(route[i+1])
            total_length += cost_edges[route[i], route[i+1]] + cost_edges[route[i+1], route[i]]
        else:
            break
            
    route = route_temp
    
    
    # add detour and subtour constraint for the lines
    detour_violation = False
    subtour_violation = False
    
    route_length = 0
    route_rev_length = 0
    for i in range(len(route)-1):
        route_length += cost_edges[route[i], route[i+1]]
        route_rev_length += cost_edges[route[i+1], route[i]]
    if route_length > detour_coeff*cost_edges[route[0], route[-1]] or route_rev_length > detour_coeff*cost_edges[route[-1], route[0]]:
        detour_violation = True
    if len(set(route)) != len(route):
        subtour_violation = True
        
        
    
    route_rev = route.copy()
    route_rev.reverse() 
    route = route + route_rev[1:]


    return route, detour_violation, subtour_violation, start, inter, inter_2, end
    
    

def generate_random_set(num_lines, bus_nodes, cost_edges, min_start_end_distance, detour_skeleton, bus_network, max_travel, max_travel_actual, detour_coeff):
    lines_current_random = []

    num_new_lines = 0

    major_stops = []

    while num_new_lines < num_lines:
        route, detour_violation, subtour_violation, start, inter, inter_2, end = generate_new_line(bus_nodes, cost_edges, min_start_end_distance, detour_skeleton, bus_network, max_travel, max_travel_actual, detour_coeff)
        
        if route not in lines_current_random and detour_violation == False and subtour_violation == False:
            num_new_lines += 1
            major_stops.append((start, inter, inter_2, end))
            lines_current_random.append(route)

            # check subtour_violation
            existed = set()
            for i in range(int(len(route)/2)+1):
                node = route[i]
                if node in existed:
                    print((start, inter, inter_2, end))
                    print(route)
                    print(num_new_lines, node)
                existed.add(node)

    return lines_current_random


    
    
    


def main(city, detour_skeleton, min_start_end_distance, max_travel, max_travel_actual, num_lines, flex_dist, detour_coeff, saved_folder, saved_folder_transit):
    
    
    output_dir(city, saved_folder)
    output_dir(city, saved_folder_transit)
    

    taxi_nodes = load_pickle(data_path(city, 'taxi_nodes.pkl'))
    demand_dic = load_pickle(data_path(city, 'demand_taxi_dic_dense.pkl'))
    bus_nodes = load_pickle(data_path(city, 'bus_nodes_dense.pkl'))
    bus_edges = load_pickle(data_path(city, 'bus_edges_1.25mile_dense.pkl'))
    cost_edges = load_pickle(data_path(city, 'cost_edges.pkl'))
    
    
    bus_network = nx.DiGraph(bus_edges) 
    node_list = taxi_nodes.copy()


    lines_current_random = generate_random_set(num_lines, bus_nodes, cost_edges, min_start_end_distance, detour_skeleton, bus_network, max_travel, max_travel_actual, detour_coeff)


    pair_lineInd_dict_random, dict_stlInd_uInd_random = line_auxiliary(demand_dic, node_list, cost_edges, lines_current_random, flex_dist)
    
    
    pair_lineInd_dict_random_transit, dict_stlInd_uInd_random_transit = line_auxiliary(demand_dic, node_list, cost_edges, lines_current_random, 0)
    
        
    savePickle(city, lines_current_random, pair_lineInd_dict_random, dict_stlInd_uInd_random, saved_folder, pair_lineInd_dict_random_transit, dict_stlInd_uInd_random_transit, saved_folder_transit)


    



if __name__ == "__main__":
    
    # create the parser
    parser = argparse.ArgumentParser()
    
    # add an argument
    parser.add_argument('--city', required=True) # Boston, Chicago, Atlanta
    parser.add_argument('--num_lines', required=True) # 600 
    parser.add_argument('--detour_skeleton', required=True) # 2 
    parser.add_argument('--min_start_end_distance', required=True) # 200
    parser.add_argument('--max_travel_actual', required=True) # 20000
    parser.add_argument('--flex_dist', required=True) # 1000
    parser.add_argument('--detour_coeff', required=True) # 2
    parser.add_argument('--saved_folder', required=True) # benchmark_multimodal
    parser.add_argument('--saved_folder_transit', required=True) # benchmark_transit
    
    
    args = parser.parse_args()
    ct = args.city
    num_lines = int(args.num_lines)
    detour_skeleton = float(args.detour_skeleton)
    min_start_end_distance = int(args.min_start_end_distance)
    max_travel_actual = int(args.max_travel_actual)
    flex_dist = int(args.flex_dist)
    detour_coeff = float(args.detour_coeff)
    saved_folder = args.saved_folder
    saved_folder_transit = args.saved_folder_transit
 

    
    max_travel = float('inf')
 
    start = time.time()
    main(ct, detour_skeleton, min_start_end_distance, max_travel, max_travel_actual, num_lines, flex_dist, detour_coeff, saved_folder, saved_folder_transit)
    total = time.time() - start
 
    print('total: ', total)

