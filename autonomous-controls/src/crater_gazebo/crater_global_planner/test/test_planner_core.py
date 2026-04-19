import os
import sys

import numpy as np

THIS_DIR = os.path.dirname(__file__)
SCRIPTS_DIR = os.path.abspath(os.path.join(THIS_DIR, "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from planner_core import build_total_path, world_to_grid


def test_world_to_grid_maps_points():
    pts = np.array([[1.0, 1.0], [1.5, 2.0]])
    out = world_to_grid(pts, min_x=0.0, min_y=0.0, resolution=0.5)
    assert np.allclose(out, np.array([[2.0, 2.0], [3.0, 4.0]]))


def test_build_total_path_returns_closed_route():
    grid = np.ones((20, 20), dtype=int)
    route_world = np.array([
        [1.0, 1.0],   # base
        [8.0, 1.0],   # wp1
        [8.0, 8.0],   # wp2
        [1.0, 1.0],   # back to base
    ])
    wp_grid = world_to_grid(route_world, min_x=0.0, min_y=0.0, resolution=1.0)
    path, order = build_total_path(grid, wp_grid)
    assert path is not None
    assert len(path) > 3
    assert order[0] == 0
    assert order[-1] == len(route_world) - 1
