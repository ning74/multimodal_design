
import math
import pickle
import random

import folium
import geopandas as gpd
import geopy.distance
import gurobipy as gp
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import osmnx as ox
import pandas as pd
import scipy.sparse
from folium.plugins import HeatMap
from gurobipy import GRB, Model
from sklearn.neighbors import BallTree, NearestNeighbors


def check_demand(folder, csv_file, city_center, radius):
    city = ox.graph.graph_from_point(
        city_center,
        dist=radius,
        dist_type="bbox",
        network_type="drive",
        simplify=True,
        truncate_by_edge=True,
    )
    g = nx.DiGraph(city)

    # rename longitude/latitude node attribute
    for name, attr in g.nodes(data=True):
        attr["longitude"] = attr["x"]
        attr["latitude"] = attr["y"]

    # select proper drive roads
    g = nx.edge_subgraph(
        g,
        [
            (u, v)
            for u, v, a in g.edges(data=True)
            if a["highway"]
            not in [
                "unclassified",
                "living_street",
                "motorway",
                "motorway_link",
                "trunk",
                "trunk_link",
            ]
        ],
    )

    # remove self loops
    g = nx.DiGraph(g)  # unfrozen
    g.remove_edges_from(set(nx.selfloop_edges(g)))

    # largest SCC component
    g = g.subgraph(max(nx.strongly_connected_components(g), key=len))
    g = nx.DiGraph(g)  # unfrozen

    with open(f"{folder}/g.gpickle", "wb") as file:
        pickle.dump(g, file, protocol=pickle.HIGHEST_PROTOCOL)

    df = pd.read_csv(f"{folder}/{csv_file}")

    # calculate distance from pickup/dropoff to the center of city
    df["center_dist_ori_km"] = df.apply(
        lambda row: geopy.distance.distance(
            (row.pickup_latitude, row.pickup_longitude), city_center
        ).km,
        axis=1,
    )
    df["center_dist_dest_km"] = df.apply(
        lambda row: geopy.distance.distance(
            (row.dropoff_latitude, row.dropoff_longitude), city_center
        ).km,
        axis=1,
    )

    # filter demand outside of range
    df = df[
        (df["center_dist_ori_km"] < float(radius / 1000))
        & (df["center_dist_dest_km"] < float(radius / 1000))
    ]
    df = df.drop(["center_dist_ori_km", "center_dist_dest_km"], axis=1)
    df = df.reset_index(drop=True)

    print(len(df))

    df.to_csv(f"{folder}/df.csv", index=False)


def generate_clusters(clustering_distance, lats_deg, lons_deg):
    EARTH_R = 6_371_000.0
    clustering_distance = 100.0
    radius_rad = clustering_distance / EARTH_R

    # Build matrix of points in radians: [[lat, lon], ...]
    X = np.deg2rad(np.c_[lats_deg, lons_deg])

    # BallTree under the hood with metric='haversine' with data X
    nn = NearestNeighbors(algorithm="ball_tree", metric="haversine")
    nn.fit(X)

    # Sparse CSR matrix (n x n). Entry (i,j)=1 if j within 100 m of i (including i).
    A = nn.radius_neighbors_graph(X, radius=radius_rad, mode="connectivity")

    nbrs = {i: A.indices[A.indptr[i]: A.indptr[i + 1]].tolist() for i in range(A.shape[0])}

    return A, nbrs


def clustering_demand_random(nbrs):
    selected_key = set()
    not_selected_key = set(nbrs.keys())
    need_to_cover = set(nbrs.keys())

    # the remaining ones are very hard to cover, so we set a threshold of 20
    while len(need_to_cover) > 20:
        select_one = random.choice(list(not_selected_key))
        selected_key.add(select_one)
        not_selected_key.remove(select_one)

        covered_one = set(nbrs[select_one])

        need_to_cover = need_to_cover - covered_one

        for i in covered_one:
            if i in not_selected_key:
                not_selected_key.remove(i)

    for i in need_to_cover:
        if i not in selected_key:
            selected_key.add(i)

    return selected_key


def filter_demand(folder, walking_distance, clustering_distance):
    print("start filtering by distance")
    print(f"filtering distance: {walking_distance}")

    with open(f"{folder}/g.gpickle", "rb") as file:
        g = pickle.load(file)

    matched_num_demand = []

    df = pd.read_csv(f"{folder}/df.csv")
    print("len(df) before filtering: ", len(df))

    # filtering if closest nodes in NetworkX is not within walking_distance meters
    X = df["pickup_longitude"].to_numpy()
    Y = df["pickup_latitude"].to_numpy()

    nodes, dists = ox.distance.nearest_nodes(g, X, Y, return_dist=True)

    df["pickup_node"] = np.where(dists < walking_distance, nodes, -1)

    X = df["dropoff_longitude"].to_numpy()
    Y = df["dropoff_latitude"].to_numpy()

    nodes, dists = ox.distance.nearest_nodes(g, X, Y, return_dist=True)

    df["dropoff_node"] = np.where(dists < walking_distance, nodes, -1)

    df = df[(df["pickup_node"] != -1) & (df["dropoff_node"] != -1)]
    matched_num_demand.append(len(df))
    print("len(df) after filtering: ", len(df))

    df.to_csv(f"{folder}/df_filtered.csv", index=False)

    pickup_lats_deg = df["pickup_latitude"].to_numpy()
    pickup_lons_deg = df["pickup_longitude"].to_numpy()

    dropoff_lats_deg = df["dropoff_latitude"].to_numpy()
    dropoff_lons_deg = df["dropoff_longitude"].to_numpy()

    lats_deg = np.concatenate((pickup_lats_deg, dropoff_lats_deg))
    lons_deg = np.concatenate((pickup_lons_deg, dropoff_lons_deg))

    with open(f"{folder}/lats_deg_filtered.pkl", "wb") as file:
        pickle.dump(lats_deg, file)

    with open(f"{folder}/lons_deg_filtered.pkl", "wb") as file:
        pickle.dump(lons_deg, file)

    # cluster filtered demand by radius of clustering_distance meters
    A, nbrs = generate_clusters(clustering_distance, lats_deg, lons_deg)

    scipy.sparse.save_npz(f"{folder}/neighbors_filtered.npz", A)

    with open(f"{folder}/nbrs_filtered.pkl", "wb") as file:
        pickle.dump(nbrs, file)

    selected_key = clustering_demand_random(nbrs)

    print("number of clusters: ", len(selected_key))

    with open(f"{folder}/selected_key.pkl", "wb") as file:
        pickle.dump(selected_key, file)


def match_networkx(folder, walking_distance):
    with open(f"{folder}/g.gpickle", "rb") as file:
        g = pickle.load(file)

    latlon_by_all_nodes = {}
    for node, data in g.nodes(data=True):
        latlon_by_all_nodes[node] = (data["latitude"], data["longitude"])

    with open(f"{folder}/selected_key.pkl", "rb") as file:
        selected_key = pickle.load(file)

    with open(f"{folder}/lats_deg_filtered.pkl", "rb") as file:
        lats_deg = pickle.load(file)

    with open(f"{folder}/lons_deg_filtered.pkl", "rb") as file:
        lons_deg = pickle.load(file)

    latlon_by_key = {}
    for i in selected_key:
        latlon_by_key[i] = (lats_deg[i], lons_deg[i])

    print("len(selected_key):", len(selected_key))
    print("len(latlon_by_key):", len(latlon_by_key))

    print("hitting set radius: ", walking_distance)

    R = 6_371_000.0  # Earth radius (m)

    # Extract coordinate arrays and keys
    A_keys = list(g.nodes())
    B_keys = list(latlon_by_key.keys())

    print("check sets: ", set(A_keys) & set(B_keys))

    A_coords = np.deg2rad(np.array([latlon_by_all_nodes[k] for k in A_keys]))
    B_coords = np.deg2rad(np.array([latlon_by_key[k] for k in B_keys]))

    # Build BallTree for A
    tree_A = BallTree(A_coords, metric="haversine")

    # Convert radius from meters to radians
    radius_rad = walking_distance / R

    # Query neighbors
    indices = tree_A.query_radius(B_coords, r=radius_rad)

    # Build output dictionary with key mappings
    nbrs_dict = {B_keys[i]: [A_keys[j] for j in inds] for i, inds in enumerate(indices)}

    print("len(A_keys): ", len(A_keys))
    print("len(B_keys): ", len(B_keys))
    print("len(nbrs_dict): ", len(nbrs_dict))

    m = gp.Model("taxi_network")
    m.setParam("MIPGap", 0.1)

    # variable: x[v] = 1 if network node v is chosen
    x = m.addVars(list(A_keys), lb=0, ub=1, vtype=GRB.BINARY)

    # objective function: minimize number of chosen nodes
    obj_func = gp.quicksum(x[v] for v in list(A_keys))
    m.setObjective(obj_func, GRB.MINIMIZE)

    # constraint: every clustered demand point i must be covered by >=1 chosen node
    for i in B_keys:
        if i in nbrs_dict:
            m.addConstr(gp.quicksum(x[v] for v in nbrs_dict[i]) >= 1)

    m.optimize()

    selected = [i for i in list(A_keys) if x[i].X > 0]

    print("number of nodes: ", len(selected))

    with open(f"{folder}/selected_matched_network.pkl", "wb") as file:
        pickle.dump(selected, file)

    return selected


def reassign(folder, walking_distance):
    with open(f"{folder}/g.gpickle", "rb") as file:
        g = pickle.load(file)

    with open(f"{folder}/selected_matched_network.pkl", "rb") as file:
        selected_matched_network = pickle.load(file)

    g = g.subgraph(selected_matched_network).copy()

    df = pd.read_csv(f"{folder}/df.csv")

    print("original number of demand: ", len(df))

    X = df["pickup_longitude"].to_numpy()
    Y = df["pickup_latitude"].to_numpy()

    nodes, dists = ox.distance.nearest_nodes(g, X, Y, return_dist=True)

    df["pickup_nearest_node_reassign"] = nodes
    df["pickup_nearest_dist_reassign"] = dists

    X = df["dropoff_longitude"].to_numpy()
    Y = df["dropoff_latitude"].to_numpy()

    nodes, dists = ox.distance.nearest_nodes(g, X, Y, return_dist=True)

    df["dropoff_nearest_node_reassign"] = nodes
    df["dropoff_nearest_dist_reassign"] = dists

    df.to_csv(f"{folder}/df_reassign.csv", index=False)

    df_throw = df[
        (df["pickup_nearest_dist_reassign"] <= walking_distance)
        & (df["dropoff_nearest_dist_reassign"] <= walking_distance)
    ]
    df_throw.to_csv(f"{folder}/df_reassign_throw.csv", index=False)
    print("final number of demand: ", len(df_throw))

    plt.figure(figsize=(6, 4))
    plt.hist(df_throw["pickup_nearest_dist_reassign"], color="red", bins=50, edgecolor="black")
    plt.xlabel("Distance (meters)")
    plt.ylabel("Number of Passengers")
    plt.title(f"{folder}")
    plt.grid(True, alpha=0.3)
    plt.savefig(f"{folder}/pickup_nodes.pdf")

    plt.figure(figsize=(6, 4))
    plt.hist(df_throw["dropoff_nearest_dist_reassign"], color="green", bins=50, edgecolor="black")
    plt.xlabel("Distance (meters)")
    plt.ylabel("Number of Passengers")
    plt.title(f"{folder}")
    plt.grid(True, alpha=0.3)
    plt.savefig(f"{folder}/dropoff_nodes.pdf")


def _select_bus_nodes(taxi_nodes, nonemp_taxi_nodes, pass_taxi_dic, g_dists, num_bus_nodes):
    model = gp.Model("bus_network")

    M = 100000

    # variables
    x = model.addVars(list(taxi_nodes), lb=0, ub=1, vtype=GRB.BINARY)
    y = model.addVars(list(taxi_nodes), nonemp_taxi_nodes, lb=0, ub=1, vtype=GRB.BINARY)

    # objective function
    obj_func = gp.quicksum(
        g_dists[i][j] * pass_taxi_dic[j] * y[i, j]
        for i in list(taxi_nodes)
        for j in nonemp_taxi_nodes
    )

    model.setObjective(obj_func, GRB.MINIMIZE)

    # constraints
    for j in nonemp_taxi_nodes:
        model.addConstr(gp.quicksum(y[i, j] for i in list(taxi_nodes)) == 1, name="supply")

    for i in list(taxi_nodes):
        model.addConstr(gp.quicksum(y[i, j] for j in nonemp_taxi_nodes) <= M * x[i], name="capacity")

    num_bus_nodes = min(num_bus_nodes, len(taxi_nodes))
    model.addConstr(gp.quicksum(x[i] for i in list(taxi_nodes)) == num_bus_nodes, name="total number")

    model.optimize()

    bus_nodes = [i for i in list(taxi_nodes) if x[i].X > 0.999]

    return bus_nodes


def _build_bus_edges(bus_nodes, g_dists, bus_edge_threshold):
    bus_edges = []
    bus_long_edge_list = set()
    bus_long_edge_dict = {}

    for u in bus_nodes:
        for v in bus_nodes:
            if u != v:
                bus_edges.append((u, v))
            if u != v and g_dists[u][v] >= bus_edge_threshold:
                bus_long_edge_list.add((u, v))
                bus_long_edge_list.add((v, u))
                bus_long_edge_dict[(u, v)] = g_dists[u][v]
                bus_long_edge_dict[(v, u)] = g_dists[v][u]

    bus_long_edge_list = list(bus_long_edge_list)
    bus_long_edge_list = sorted(bus_long_edge_list, key=lambda item: bus_long_edge_dict[item])

    G = nx.DiGraph()
    G.add_edges_from(bus_edges)
    while len(bus_long_edge_list) > 0:
        one_edge = bus_long_edge_list[-1]
        bus_long_edge_list = bus_long_edge_list[:-1]

        reverse_one_edge = (one_edge[1], one_edge[0])
        bus_long_edge_list.remove(reverse_one_edge)

        G_temp = nx.DiGraph(G)
        G_temp.remove_edge(*one_edge)
        G_temp.remove_edge(*reverse_one_edge)

        if nx.is_strongly_connected(G_temp) and set(G_temp.nodes()) == set(bus_nodes):
            G.remove_edge(*one_edge)
            G.remove_edge(*reverse_one_edge)

    return list(G.edges())


def prepare_demand_and_networks(folder, num_bus_nodes, bus_edge_threshold):
    with open(f"{folder}/selected_matched_network.pkl", "rb") as file:
        selected_matched_network = pickle.load(file)

    df_reassign_throw = pd.read_csv(f"{folder}/df_reassign_throw.csv")

    taxi_nodes = selected_matched_network

    with open(f"{folder}/taxi_nodes.pkl", "wb") as file:
        pickle.dump(taxi_nodes, file)

    demand_taxi_dic = {}

    for i in taxi_nodes:
        for j in taxi_nodes:
            demand_taxi_dic[i, j] = 0

    # check the distances between pickup and dropoff locations if matched to the same virtual stop
    close_distances = []

    for index, row in df_reassign_throw.iterrows():
        if row.pickup_nearest_node_reassign != row.dropoff_nearest_node_reassign:
            demand_taxi_dic[row.pickup_nearest_node_reassign, row.dropoff_nearest_node_reassign] += 1
        else:
            close_distances.append(
                geopy.distance.distance(
                    (row.pickup_latitude, row.pickup_longitude),
                    (row.dropoff_latitude, row.dropoff_longitude),
                ).m
            )

    demand_taxi_dic = {key: value for key, value in demand_taxi_dic.items() if value > 0}

    print("demand with distinct virtual stops: ", sum(demand_taxi_dic.values()))

    # passengers that matched to the same virtual stops
    plt.figure(figsize=(6, 4))
    plt.hist(close_distances, color="orange", bins=50, edgecolor="black")
    plt.xlabel("Distance (meters)")
    plt.ylabel("Number of Passengers")
    plt.title(f"{folder}")
    plt.grid(True, alpha=0.3)
    plt.savefig(f"{folder}/OD_matched_same.pdf")

    # assume LODES data are for 3 hours, get 15 min demand
    demand_taxi_dic_temp = demand_taxi_dic.copy()

    demand_taxi_dic = {}
    demand_taxi_dic_dense = {}
    for key, value in demand_taxi_dic_temp.items():
        demand_taxi_dic_dense[key] = math.ceil(value / 12)
        demand_taxi_dic[key] = value / 12

    print("number of ODs: ", len(demand_taxi_dic))
    print("demand divided by 12: ", sum(demand_taxi_dic.values()))

    with open(f"{folder}/demand_taxi_dic_divided_by_12.pkl", "wb") as file:
        pickle.dump(demand_taxi_dic, file)

    demand_taxi_dic_dense = {key: value for key, value in demand_taxi_dic_dense.items() if value > 1}

    with open(f"{folder}/demand_taxi_dic_dense.pkl", "wb") as file:
        pickle.dump(demand_taxi_dic_dense, file)

    # treat the fractional part as probability
    demand_taxi_dic_base = {}
    demand_taxi_dic_prob = {}

    for key, value in demand_taxi_dic.items():
        demand_taxi_dic_base[key] = math.floor(value)

    for key, value in demand_taxi_dic.items():
        demand_taxi_dic_prob[key] = demand_taxi_dic[key] - demand_taxi_dic_base[key]

    # generate based on the probability
    demand_taxi_dic_prob_generate = {}

    for key, value in demand_taxi_dic_prob.items():
        v = np.random.choice(np.arange(0, 2), p=[1 - demand_taxi_dic_prob[key], demand_taxi_dic_prob[key]])
        demand_taxi_dic_prob_generate[key] = v

    demand_taxi_dic_temp = {}
    for key, value in demand_taxi_dic_base.items():
        demand_taxi_dic_temp[key] = demand_taxi_dic_base[key] + demand_taxi_dic_prob_generate[key]

    demand_taxi_dic_sparse = {key: value for key, value in demand_taxi_dic_temp.items() if value > 0}

    print("final number of ODs: ", len(demand_taxi_dic_sparse))
    print("final total demand: ", sum(demand_taxi_dic_sparse.values()))

    with open(f"{folder}/demand_taxi_dic_sparse.pkl", "wb") as file:
        pickle.dump(demand_taxi_dic_sparse, file)

    # get edge cost
    with open(f"{folder}/g.gpickle", "rb") as file:
        g = pickle.load(file)

    cost_edges = {}
    g_dists = dict(nx.all_pairs_dijkstra_path_length(g, weight="length"))
    for i in taxi_nodes:
        for j in taxi_nodes:
            if i != j:
                cost_edges[(i, j)] = round(g_dists[i][j], 2)
            else:
                cost_edges[(i, j)] = 0

    with open(f"{folder}/cost_edges.pkl", "wb") as file:
        pickle.dump(cost_edges, file)

    # sparse demand bus network
    pass_taxi_dic = {i: 0 for i in taxi_nodes}
    for key, value in demand_taxi_dic_sparse.items():
        pass_taxi_dic[key[0]] += value
        pass_taxi_dic[key[1]] += value

    nonemp_taxi_nodes = [key for key, value in pass_taxi_dic.items() if value > 0]

    bus_nodes_sparse = _select_bus_nodes(taxi_nodes, nonemp_taxi_nodes, pass_taxi_dic, g_dists, num_bus_nodes)
    print("len(bus_nodes_sparse): ", len(bus_nodes_sparse))

    with open(f"{folder}/bus_nodes_sparse.pkl", "wb") as file:
        pickle.dump(bus_nodes_sparse, file)

    bus_edges_sparse = _build_bus_edges(bus_nodes_sparse, g_dists, bus_edge_threshold)
    print("number of bus edges (sparse): ", len(bus_edges_sparse))

    with open(f"{folder}/bus_edges_1.25mile_sparse.pkl", "wb") as file:
        pickle.dump(bus_edges_sparse, file)

    # dense demand bus network
    pass_taxi_dic = {i: 0 for i in taxi_nodes}
    for key, value in demand_taxi_dic_dense.items():
        pass_taxi_dic[key[0]] += value
        pass_taxi_dic[key[1]] += value

    nonemp_taxi_nodes = [key for key, value in pass_taxi_dic.items() if value > 0]

    bus_nodes_dense = _select_bus_nodes(taxi_nodes, nonemp_taxi_nodes, pass_taxi_dic, g_dists, num_bus_nodes)
    print("len(bus_nodes_dense): ", len(bus_nodes_dense))

    with open(f"{folder}/bus_nodes_dense.pkl", "wb") as file:
        pickle.dump(bus_nodes_dense, file)

    bus_edges_dense = _build_bus_edges(bus_nodes_dense, g_dists, bus_edge_threshold)
    print("number of bus edges (dense): ", len(bus_edges_dense))

    with open(f"{folder}/bus_edges_1.25mile_dense.pkl", "wb") as file:
        pickle.dump(bus_edges_dense, file)


if __name__ == "__main__":
    radius = 4800  # ~3 miles

    city_centers = {
        "Boston": (42.3601, -71.0589),
        "Chicago": (41.8819, -87.6301),
        "Atlanta": (33.7681, -84.3806),
    }
    csv_files = {
        "Boston": "temp_requests_boston.csv",
        "Chicago": "temp_requests_chicago.csv",
        "Atlanta": "temp_requests_atlanta.csv",
    }
    num_bus_nodes_by_city = {
        "Boston": 100,
        "Chicago": 70,
        "Atlanta": 117,
    }

    walking_distance = 400
    clustering_distance = 100
    bus_edge_threshold = 2000  # ~1.25 miles
    

    for city, center in city_centers.items():
        check_demand(city, csv_files[city], center, radius)

    for city in city_centers:
        filter_demand(city, walking_distance, clustering_distance)

    for city in city_centers:
        match_networkx(city, walking_distance)

    for city in city_centers:
        reassign(city, walking_distance)

    for city, num_bus_nodes in num_bus_nodes_by_city.items():
        prepare_demand_and_networks(city, num_bus_nodes, bus_edge_threshold)
