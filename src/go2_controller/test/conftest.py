import importlib.util
import os
import sys

GO2_SIM_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "go2_sim"))

if importlib.util.find_spec("go2_sim") is None:
    sys.path.insert(0, GO2_SIM_SRC)
