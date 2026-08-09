import os
import sys
import dill as pickle

# Project root = the "minCost_and_maxRide" folder, i.e. the parent of this file's directory (root/).
# Anchoring to __file__ makes the default paths independent of the current working directory.
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DATA_DIR = os.environ.get("MULTIMODAL_DATA_DIR", os.path.join(_PROJECT_ROOT, "data"))


def _output_base():
    # Output trees (col_gen_output/, opt_result/, heur_output*/) are created under
    # the subproject (minCost/ or maxRide/) whose script is being run, e.g.
    # minCost/col_gen_output/... when running a script in minCost/src/.
    # Set MULTIMODAL_OUTPUT_DIR to override this location.
    env = os.environ.get("MULTIMODAL_OUTPUT_DIR")
    if env:
        return env
    main_script = sys.argv[0] if sys.argv else ""
    if main_script:
        script_dir = os.path.dirname(os.path.abspath(main_script))
        if os.path.basename(script_dir) == "src":
            return os.path.dirname(script_dir)
    return _PROJECT_ROOT


OUTPUT_DIR = _output_base()


def data_path(city, filename):
    # Path to a preprocessed input file, e.g. data_path('Boston', 'bus_nodes_dense.pkl').
    return os.path.join(DATA_DIR, city, filename)


def output_dir(city, saved_folder, create=True):
    # Folder for column-generation outputs (col_gen_output/).
    # Creates the folder when create=True (writes); pass create=False when
    # building a path only to read an existing file.
    folder = os.path.join(OUTPUT_DIR, 'col_gen_output', city, saved_folder)
    if create:
        os.makedirs(folder, exist_ok=True)
    return folder


def output_path(city, saved_folder, filename, create=True):
    # Path to a run output file. Pass create=False for read-only access so a
    # missing folder is not silently created.
    return os.path.join(output_dir(city, saved_folder, create=create), filename)


def heur_path(city, saved_folder, filename, tree='heur_output', create=True):
    # Path to a heuristic-run output file (heur_output/ by default).
    # Pass create=False when building a path only to read an existing file.
    folder = os.path.join(OUTPUT_DIR, tree, city, saved_folder)
    if create:
        os.makedirs(folder, exist_ok=True)
    return os.path.join(folder, filename)


def result_path(city, filename):
    # Path to an optimization result/log file (opt_result/), creating the folder if needed.
    folder = os.path.join(OUTPUT_DIR, 'opt_result', city)
    os.makedirs(folder, exist_ok=True)
    return os.path.join(folder, filename)


def save_pickle(obj, path):
    with open(path, "wb") as file:
        pickle.dump(obj, file)


def load_pickle(path):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"Expected input file not found: {path}\n"
            f"If this is a generated-lines file (e.g. lines_final.pkl), run the "
            f"line-generation step with the same --saved_folder first, or move "
            f"previously generated results into the folder above.")
    with open(path, "rb") as file:
        return pickle.load(file)
