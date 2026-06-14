"""
Python-side benchmark: times the upstream NeuPAN DUNE / NRMP layers and the
full planner forward on the same corridor scenario used by dump_test_data.py.

Run from the upstream NeuPAN repo root with its venv:
    PYTHONPATH=<tools dir>:. python bench_python.py

Part of neupan_cpp, a C++ port of NeuPAN (Copyright (c) 2025 Ruihua Han),
distributed under the GNU General Public License v3 or later.
"""

import sys
import time

import numpy as np

from dump_test_data import corridor_points, diff_step

N_FRAMES = 50


def stats(name, us):
    us = sorted(us)
    mean = sum(us) / len(us)
    print(f"{name:<28} n={len(us):4d}  mean {mean:8.1f} us   "
          f"median {us[len(us) // 2]:8.1f} us   "
          f"p95 {us[int(len(us) * 0.95)]:8.1f} us")


def main():
    sys.path.insert(0, ".")
    from neupan import neupan

    planner = neupan.init_from_yaml("example/LON/planner.yaml")

    dune_us, nrmp_us, frame_us = [], [], []

    orig_nrmp = planner.pan.nrmp_layer.forward
    orig_dune = planner.pan.dune_layer.forward

    def nrmp_hook(*args, **kwargs):
        t0 = time.perf_counter()
        out = orig_nrmp(*args, **kwargs)
        nrmp_us.append((time.perf_counter() - t0) * 1e6)
        return out

    def dune_hook(*args, **kwargs):
        t0 = time.perf_counter()
        out = orig_dune(*args, **kwargs)
        dune_us.append((time.perf_counter() - t0) * 1e6)
        return out

    planner.pan.nrmp_layer.forward = nrmp_hook
    planner.pan.dune_layer.forward = dune_hook

    rng = np.random.default_rng(7)
    points = corridor_points(rng)
    state = np.array([[0.0], [20.0], [0.0]])

    for _ in range(N_FRAMES):
        t0 = time.perf_counter()
        action, _ = planner(state, points)
        frame_us.append((time.perf_counter() - t0) * 1e6)
        state = diff_step(state, action, planner.dt)

    # skip the first frame (torch/cvxpy lazy init)
    stats("DUNE forward (1100 pts)", dune_us[2:])
    stats("NRMP forward (cvxpy/ECOS)", nrmp_us[2:])
    stats("planner forward (frame)", frame_us[1:])


if __name__ == "__main__":
    main()
