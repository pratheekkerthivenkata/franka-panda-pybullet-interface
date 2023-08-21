import os

PANDA_SIM_TIMESTEP = 0.02
PANDA_REAL_TIMESTEP = 0.001

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(ROOT_DIR, '..', 'config')
ASSETS_DIR = os.path.join(ROOT_DIR, '..', 'assets')
