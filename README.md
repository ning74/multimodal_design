# Multi-Modal Transit Network Design

This repository provides the Python implementation to design urban multi-modal transit systems. The project contains two formulations:

| Problem | Objective | Key knobs |
|---|---|---|
| `minCost` | Minimize total system cost while serving at least a given fraction of demand (`--demand_perc`) | `gamma`, `cap`, `demand_perc` |
| `maxRide` | Maximize ridership (weighted by `alpha`) subject to an operating budget (`--budget`) | `alpha`, `budget`, `--transit` flag for a bus-only system |

## Project structure

```
minCost_and_maxRide/
├── root/
│   ├── instance.py        # Loads network and demand data; builds a problem instance
│   └── paths.py           # All data/output path helpers, pickle save/load
│                          #   (respects MULTIMODAL_DATA_DIR / MULTIMODAL_OUTPUT_DIR env vars)
├── data/
│   ├── process_demand.py  # Preprocessing: builds the road network (OSMnx), filters and
│   │                      #   clusters trip requests, selects bus nodes, builds bus edges
│   ├── Atlanta/           # Preprocessed inputs per city:
│   ├── Boston/            #   temp_requests_<city>.csv   raw trip requests
│   └── Chicago/           #   demand_taxi_dic_*.pkl      OD demand dictionaries (dense/sparse)
│                          #   bus_nodes_*.pkl            candidate bus stop nodes
│                          #   bus_edges_1.25mile_*.pkl   candidate bus edges (≤1.25 mi)
│                          #   cost_edges.pkl             edge cost/distance dictionary
├── minCost/src/
│   ├── minCost_experiment.py             
│   ├── minCost_generate_lines_naive.py   # Warm-start lines
│   ├── minCost_generate_lines_CG_*.py    # Column generation (exact / relaxed / mixed subproblems)
│   ├── minCostLP.py, minCostLPRelaxed.py # Restricted master LP and its relaxation
│   └── minCostMIP.py, minCostMIP_heur.py # Final MIP evaluation (+ heuristic variant)
└── maxRide/src/
    ├── maxRide_experiment.py             
    ├── maxRide_experiment_benchmark.py   # Benchmark lines for comparison
    ├── maxRide_generate_lines_naive.py   # Warm-start lines
    ├── maxRide_generate_lines_CG_*.py    # Column generation (exact / relaxed / mixed)
    ├── maxRideLP.py, maxRideLPRelaxed.py # Restricted master LP and its relaxation
    └── maxRideMIP.py, maxRideMIP_heur.py # Budget-constrained MIP (+ heuristic variant)
```

## Requirements

- Python 3.9+
- [Gurobi](https://www.gurobi.com/) with a valid license (`gurobipy`)
- `networkx`, `numpy`, `pandas`, `dill`, `scipy`, `matplotlib`
- Preprocessing only (`data/process_demand.py`): `osmnx`, `geopandas`, `geopy`, `folium`, `scikit-learn`

```bash
pip install -r requirements.txt
```

## Workflow

### 1. Preprocess demand and networks (optional, outputs are included)

The per-city `.pkl` files in `data/<City>/` are the results of this step, so the optimization can be run directly and **this step can be skipped entirely**.

**Where to find the raw trip-request CSVs.** The raw CSVs (`temp_requests_<city>.csv`, ~110 MB total) are not tracked in this repository. They are attached to the [`v1.0-data` release](https://github.com/ning74/multi_design/releases/tag/v1.0-data) as `temp_requests.zip`. The requests are synthetic demand generated from public US Census [LODES](https://lehd.ces.census.gov/data/) origin–destination data using [scripts-for-simulator](https://github.com/DMadhuranga/scripts-for-simulator); they are only needed if you want to re-run the preprocessing yourself. To set them up:

```bash
curl -L -o temp_requests.zip https://github.com/ning74/multimodal_design/releases/download/v1.0-data/temp_requests.zip
unzip temp_requests.zip -d minCost_and_maxRide/data/
```

so that each file sits at `data/<City>/temp_requests_<city>.csv`.

**Running the step.** Run from within the `data/` directory (`cd minCost_and_maxRide/data && python process_demand.py`); the script reads `<City>/temp_requests_<city>.csv` and writes its intermediate and final files into each `<City>/` folder relative to the current working directory.

`data/process_demand.py` takes a raw trip-request CSV (`temp_requests_<city>.csv`) and:
1. Downloads and cleans the drivable road network around the city center (OSMnx, largest strongly connected component).
2. Filters requests to the study area and clusters pickups/dropoffs to network nodes.
3. Selects candidate bus stop nodes and builds candidate bus edges within a distance threshold.
4. Saves the `*.pkl` inputs consumed by `root/instance.py`.

### 2. Design a network (line generation via column generation)

Both experiment drivers follow the same phases: generate naive warm-start lines that together cover every bus edge (heavily cost-penalized so CG replaces them), run mixed relaxed/exact column generation to add candidate lines, strip the warm-start lines and save the final line pool.

**minCost** (serve ≥ `demand_perc` of demand at minimum cost), e.g.:

```bash
cd minCost_and_maxRide/minCost/src
python minCost_experiment.py \
  --city Boston --gamma 5 --cap 50 --unit_dist 4000 --flex_dist 1000 \
  --max_travel 20000 --saved_folder multimodal_budget --detour_coeff 2 \
  --num_rounds 800 --mip_gap 0.05 --num_sol 5 --time_limit 600 \
  --cost_factor_naive 10000 --demand_perc 0.9
```

**maxRide** (maximize ridership under a budget; add `--transit` for a bus-only system), e.g.:

```bash
cd minCost_and_maxRide/maxRide/src
python maxRide_experiment.py \
  --city Boston --gamma 5 --alpha 1 --cap 50 --budget <B> \
  --unit_dist 4000 --flex_dist 1000 --max_travel 20000 \
  --saved_folder multimodal_gamma5_alpha1 --detour_coeff 2 \
  --num_rounds 800 --mip_gap 0.05 --num_sol 5 --time_limit 600 \
  --cost_factor_naive 10000
```

Generated line sets and index dictionaries are pickled to `col_gen_output/<city>/<saved_folder>/` under the corresponding subproject (`minCost/` or `maxRide/`) (e.g. `lines_final.pkl`, `pair_lineInd_dict_final.pkl`, `dict_stlInd_uInd_final.pkl`). `maxRide_experiment_benchmark.py` builds the benchmark baseline for comparison.

### 3. MIP heuristic (large instances)

Runs the MIP heuristic over the final candidate line set saved by column
generation in `col_gen_output/<city>/<saved_folder>/`. Gurobi logs and
results are written to `opt_result/<city>/` under the corresponding
subproject.

**maxRideMIP_heur**, e.g.:

```bash
python maxRideMIP_heur.py \
  --city Boston --gamma 5 --alpha 1 --cap 50 --budget <B> \
  --flex_dist 1000 --mip_gap 0 --unit_dist 4000 \
  --top 0 --bottom 600 --saved_folder multimodal_gamma5_alpha1 \
  --firstSelect 200 --secondSelectStep 10 --num_finalLines 50 \
  --saved_folder_new multimodal_gamma5_alpha1_MIP --mip_focus 0 \ 
  --time_limit 86400 --transit
```

Candidate set

- `--saved_folder` column-generation run to read candidate lines from
- `--saved_folder_new` subfolder under `opt_result/<city>/` to write this
  run's logs and results to
- `--top`, `--bottom` index range of the candidate set to consider;
  `0 600` uses the first 600 lines

Selection heuristic

- `--firstSelect` number of candidates kept in the first pass
- `--secondSelectStep` stride used when narrowing that set
- `--num_finalLines` number of lines passed to the final MIP

Solver

- `--mip_gap` relative MIP optimality gap Gurobi must reach before
  stopping; `0` demands a proven optimum
- `--mip_focus` Gurobi `MIPFocus`: `0` balanced, `1` prioritize finding
  feasible solutions, `2` prioritize proving optimality, `3` improve the
  bound
- `--time_limit` wall-clock limit in seconds (`86400` = 24 hours)

> **Note:** `--mip_gap 0` with `--time_limit 86400` will run the full 24
> hours on any instance it can't close. For large instances where a good
> incumbent is enough, set `--mip_gap 0.01` and `--mip_focus 1`.
## Key parameters

| Flag | Meaning | Typical value |
|---|---|---|
| `--city` | `Atlanta`, `Boston`, or `Chicago` | — |
| `--gamma` | Per-unit-distance operating cost factor for bus lines | 5 |
| `--cap` | Bus vehicle physical capacity | 50 |
| `--unit_dist` | Reference line distance used to normalize cost/capacity (meters) | 4000 |
| `--flex_dist` | Walking/feeder flexibility radius around stops (meters) | 1000 |
| `--max_travel` | Maximum allowed passenger travel distance (meters) | 20000 |
| `--detour_coeff` | Maximum detour ratio relative to the direct trip | 2 |
| `--num_rounds` | Column-generation iterations | 800 |
| `--mip_gap`, `--num_sol`, `--time_limit` | Gurobi termination controls for CG subproblems | 0.05 / 5 / 600 |
| `--cost_factor_naive` | Cost penalty multiplier on seed lines so CG prices them out | 10000 |
| `--demand_perc` (minCost) | Minimum fraction of demand that must be served | 0.9 |
| `--alpha` (maxRide) | Per-unit-distance operating cost factor for taxi vehicles | 1, 0.5, 0.3 |
| `--budget` (maxRide) | Operating budget | instance-specific |
| `--transit` (maxRide) | Design a bus-only (no taxi feeder) system | flag |

## Data notes

- All input and output paths are resolved by `root/paths.py`, so the optimization scripts can be run from any working directory. Input data paths are anchored to the project root; output trees (`col_gen_output/`, `opt_result/`, `heur_output/`) are created under the subproject (`minCost/` or `maxRide/`) whose script is being run. Set `MULTIMODAL_DATA_DIR` to relocate the input data and `MULTIMODAL_OUTPUT_DIR` to relocate the output trees.
- `dense`/`sparse` file variants correspond to two densities of the demand pre-processing; the code defaults to the `dense` versions ("low-demand truncation with rescaling").
