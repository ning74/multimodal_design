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
import math
import re
import argparse


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "root"))


import maxRideLP

import maxRide_generate_lines_naive

import maxRide_generate_lines_CG_mixed

from paths import output_path, save_pickle, data_path, load_pickle



def mixedRemoveSave(lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current, city, saved_folder):
    save_pickle(lines_current, output_path(city, saved_folder, 'lines_mixedRemove.pkl'))
    save_pickle(pair_lineInd_dict_current, output_path(city, saved_folder, 'pair_lineInd_dict_mixedRemove.pkl'))
    save_pickle(dict_stlInd_uInd_current, output_path(city, saved_folder, 'dict_stlInd_uInd_mixedRemove.pkl'))


def finalSave(lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current, city, saved_folder):
    save_pickle(lines_current, output_path(city, saved_folder, 'lines_final.pkl'))
    save_pickle(pair_lineInd_dict_current, output_path(city, saved_folder, 'pair_lineInd_dict_final.pkl'))
    save_pickle(dict_stlInd_uInd_current, output_path(city, saved_folder, 'dict_stlInd_uInd_final.pkl'))


def main(ct, g, c, b, u_dist, f_dist, m_travel, s_folder, d_coeff, num_rounds, mip_gap, num_sol, time_limit, a, cost_factor_naive, transit):

    # phase 0: generate "naive" lines (one round-trip per bus edge) to seed column generation
    lines_current, cost_lines_current, cap_lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current\
    = maxRide_generate_lines_naive.main(city=ct, which_method='base', gamma=g, cap=c, budget=b, unit_dist=u_dist, flex_dist=f_dist, saved_folder=s_folder, alpha=a, transit=transit)

    remove_top = len(lines_current)

    # phase 1: column generation, mixing relaxed and exact subproblems, to add useful lines on top of the naive ones
    lines_mixed_relaxed, pair_lineInd_dict_mixed_relaxed, dict_stlInd_uInd_mixed_relaxed, \
    lines_mixed_exact, pair_lineInd_dict_mixed_exact, dict_stlInd_uInd_mixed_exact, list_whichCG \
            = maxRide_generate_lines_CG_mixed.main(city=ct, which_method='naive', gamma=g, cap=c, budget=b, unit_dist=u_dist, num_rounds=num_rounds, perc_bus_edges_selected=0.3, flex_dist=f_dist, mip_gap=mip_gap, time_limit=time_limit, num_sol=num_sol, max_travel=m_travel, saved_folder=s_folder, \
          detour_coeff=d_coeff, alpha=a, include_naive=True, len_naive=remove_top, cost_factor_naive=cost_factor_naive, transit=transit)
 
 
 
    lines_current = lines_mixed_exact
    pair_lineInd_dict_current = pair_lineInd_dict_mixed_exact
    dict_stlInd_uInd_current = dict_stlInd_uInd_mixed_exact

 

    # remove the naive lines used to seed column generation, keeping only the lines CG actually added
    lines_current = lines_current[remove_top:]
    demand_dic = load_pickle(data_path(ct, 'demand_taxi_dic_dense.pkl'))
    pair_lineInd_dict_current, dict_stlInd_uInd_current, _, _ = maxRideLP.extract_lines(ct, demand_dic, g, c, f_dist, u_dist, lines_current)
 
    mixedRemoveSave(lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current, ct, s_folder)
    finalSave(lines_current, pair_lineInd_dict_current, dict_stlInd_uInd_current, ct, s_folder)

    

if __name__ == "__main__":
    
    # create the parser
    parser = argparse.ArgumentParser()
 
 
    # add an argument
    parser.add_argument('--city', required=True) # Boston, Chicago, Atlanta
    parser.add_argument('--gamma', required=True) # 5  
    parser.add_argument('--alpha', required=True) # 1, 0.5, 0.3
    parser.add_argument('--cap', required=True) # 50
    parser.add_argument('--budget', required=True) 
    parser.add_argument('--unit_dist', required=True)  # 4000
    parser.add_argument('--flex_dist', required=True)  # 1000
    parser.add_argument('--max_travel', required=True)  # 20000
    parser.add_argument('--saved_folder', required=True) # multimodal_gamma5_alpha1
    parser.add_argument('--detour_coeff', required=True) # 2
    parser.add_argument('--num_rounds', required=True)
    parser.add_argument('--mip_gap', required=True) # 0.05
    parser.add_argument('--num_sol', required=True) # 5
    parser.add_argument('--time_limit', required=True) # 600
    parser.add_argument('--cost_factor_naive', required=True) # 10000
    parser.add_argument("--transit", action="store_true") # (include for generating bus-only system)
    
    args = parser.parse_args()
    ct = args.city
    g = float(args.gamma)
    a = float(args.alpha)
    c = int(args.cap)
    b = int(args.budget)
    u_dist = int(args.unit_dist)
    f_dist = int(args.flex_dist)
    m_travel = int(args.max_travel)
    s_folder = args.saved_folder
    d_coeff = float(args.detour_coeff)
    num_rounds = int(args.num_rounds)
    mip_gap = float(args.mip_gap)
    num_sol = int(args.num_sol)
    time_limit = int(args.time_limit)
    cost_factor_naive = int(args.cost_factor_naive)
    transit = args.transit
    
    
 
    start = time.time()
    main(ct, g, c, b, u_dist, f_dist, m_travel, s_folder, d_coeff, num_rounds, mip_gap, num_sol, time_limit, a, cost_factor_naive, transit)
    total = time.time() - start
    print('total time: ', total)
