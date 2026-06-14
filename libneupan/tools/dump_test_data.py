"""
Dump reference data from the upstream Python NeuPAN for the libneupan unit
tests (test_dune / test_nrmp / later test_replay).

Run from the upstream NeuPAN repo root with its venv:
    PYTHONPATH=<tools dir>:. python dump_test_data.py <out_dir>

Produces:
    dune_mlp.nptf        raw MLP forward on random points (mu comparison)
    frames.nptf          per-frame, per-PAM-iteration NRMP/DUNE inputs &
                         outputs from a short corridor simulation
    replay_planner.yaml  line-style planner config shared with the C++ test
    replay.nptf          50-frame end-to-end (state, points, action) replay

Part of neupan_cpp, a C++ port of NeuPAN (Copyright (c) 2025 Ruihua Han),
distributed under the GNU General Public License v3 or later.
"""

import os
import sys
from math import cos, sin

import numpy as np
import torch

from nptf import write_nptf

N_FRAMES = 10
N_MLP_POINTS = 500
N_REPLAY_FRAMES = 50


def tn(t):
    return t.detach().cpu().numpy().astype(np.float64)


def dump_mlp(planner, out_dir, rng):
    model = planner.pan.dune_layer.model
    pts = rng.uniform(-10.0, 10.0, size=(2, N_MLP_POINTS)).astype(np.float32)
    with torch.no_grad():
        mu = model(torch.from_numpy(pts.T)).T
    write_nptf(
        os.path.join(out_dir, "dune_mlp.nptf"),
        [("points", pts), ("mu", tn(mu))],
    )


def corridor_points(rng):
    """Static walls of the LON corridor plus a block to force avoidance."""
    xs = np.arange(0.0, 40.0, 0.25)
    lower = np.stack([xs, np.full_like(xs, 18.6)])
    upper = np.stack([xs, np.full_like(xs, 21.4)])
    bx = np.arange(7.5, 8.5, 0.1)
    block = np.stack([bx, np.full_like(bx, 19.6)])
    return np.hstack([lower, upper, block])


def replay_points(rng):
    """Wide corridor with two shallow blocks forming a passable slalom
    (robot width 2.0: block1 forces center y >= ~19.8, block2 <= ~20.2)."""
    xs = np.arange(0.0, 40.0, 0.25)
    lower = np.stack([xs, np.full_like(xs, 17.5)])
    upper = np.stack([xs, np.full_like(xs, 22.5)])
    b1 = np.arange(7.5, 8.5, 0.1)
    block1 = np.stack([b1, np.full_like(b1, 18.8)])
    b2 = np.arange(14.5, 15.5, 0.1)
    block2 = np.stack([b2, np.full_like(b2, 21.2)])
    return np.hstack([lower, upper, block1, block2])


def dump_replay(out_dir):
    import yaml as pyyaml
    from neupan import neupan

    with open("example/LON/planner.yaml") as f:
        cfg = pyyaml.safe_load(f)
    cfg["ipath"]["curve_style"] = "line"
    cfg["pan"]["dune_checkpoint"] = os.path.abspath(
        "example/model/diff_robot_default/model_5000.pth")
    yaml_path = os.path.join(out_dir, "replay_planner.yaml")
    with open(yaml_path, "w") as f:
        pyyaml.safe_dump(cfg, f)

    planner = neupan.init_from_yaml(yaml_path)
    rng = np.random.default_rng(11)
    points = replay_points(rng)
    state = np.array([[0.0], [20.0], [0.0]])

    records = [("meta.n_frames", np.array(float(N_REPLAY_FRAMES))),
               ("points", points)]

    for f in range(N_REPLAY_FRAMES):
        action, info = planner(state, points)
        pre = f"r{f:02d}"
        records.append((f"{pre}.state", state.copy()))
        records.append((f"{pre}.action", action))
        records.append((f"{pre}.min_dist",
                        np.array(float(planner.min_distance))))
        records.append((f"{pre}.stop", np.array(float(info["stop"]))))
        records.append((f"{pre}.arrive", np.array(float(info["arrive"]))))
        # full control sequence: lets the C++ test sync its cur_vel_array
        # for a strict open-loop, frame-by-frame comparison
        records.append((f"{pre}.opt_u", tn(info["vel_tensor"])))
        state = diff_step(state, action, planner.dt)

    write_nptf(os.path.join(out_dir, "replay.nptf"), records)
    print(f"dumped {N_REPLAY_FRAMES} replay frames -> replay.nptf")


def diff_step(state, action, dt):
    x, y, theta = state[0, 0], state[1, 0], state[2, 0]
    v, w = action[0, 0], action[1, 0]
    return np.array(
        [[x + v * cos(theta) * dt], [y + v * sin(theta) * dt],
         [theta + w * dt]]
    )


def main(out_dir):
    sys.path.insert(0, ".")
    from neupan import neupan

    os.makedirs(out_dir, exist_ok=True)
    rng = np.random.default_rng(7)

    planner = neupan.init_from_yaml("example/LON/planner.yaml")
    dump_mlp(planner, out_dir, rng)

    records = []
    adj = planner.pan.nrmp_layer
    records.append(("meta.q_s", np.full(3, float(adj.q_s))))
    records.append(("meta.p_u", np.array(float(adj.p_u))))
    records.append(("meta.eta", np.array(float(adj.eta))))
    records.append(("meta.d_max", np.array(float(adj.d_max))))
    records.append(("meta.d_min", np.array(float(adj.d_min))))
    records.append(("meta.ro_obs", np.array(float(adj.ro_obs))))
    records.append(("meta.bk", np.array(float(adj.bk))))
    records.append(("meta.max_num", np.array(float(adj.max_num))))
    records.append(("meta.T", np.array(float(planner.T))))
    records.append(("meta.dt", np.array(planner.dt)))
    records.append(("meta.n_frames", np.array(float(N_FRAMES))))
    records.append(("meta.G", planner.robot.G))
    records.append(("meta.h", planner.robot.h))
    records.append(("meta.max_speed", planner.robot.max_speed))
    records.append(("meta.acce_bound", planner.robot.acce_bound))

    # Hook NRMP and DUNE forwards to capture per-iteration inputs/outputs.
    frame = {"f": 0, "it": 0}

    orig_nrmp = planner.pan.nrmp_layer.forward
    orig_dune = planner.pan.dune_layer.forward

    def nrmp_hook(nom_s, nom_u, ref_s, ref_us,
                  mu_list=None, lam_list=None, point_list=None):
        pre = f"f{frame['f']:02d}_it{frame['it']}"
        records.append((f"{pre}.nom_s", tn(nom_s)))
        records.append((f"{pre}.nom_u", tn(nom_u)))
        records.append((f"{pre}.ref_s", tn(ref_s)))
        records.append((f"{pre}.ref_us", tn(ref_us)))
        out = orig_nrmp(nom_s, nom_u, ref_s, ref_us,
                        mu_list, lam_list, point_list)
        records.append((f"{pre}.opt_s", tn(out[0])))
        records.append((f"{pre}.opt_u", tn(out[1])))
        records.append((f"{pre}.opt_d", tn(out[2])))
        frame["it"] += 1
        return out

    def dune_hook(point_flow, R_list, obs_points_list=[]):
        pre = f"f{frame['f']:02d}_it{frame['it']}"
        out = orig_dune(point_flow, R_list, obs_points_list)
        mu_list, lam_list, sort_point_list = out
        for t, (pf, R, op) in enumerate(
                zip(point_flow, R_list, obs_points_list)):
            records.append((f"{pre}.pf_s{t:02d}", tn(pf)))
            records.append((f"{pre}.R_s{t:02d}", tn(R)))
            records.append((f"{pre}.op_s{t:02d}", tn(op)))
        for t, (mu, lam, sp) in enumerate(
                zip(mu_list, lam_list, sort_point_list)):
            records.append((f"{pre}.mu_s{t:02d}", tn(mu)))
            records.append((f"{pre}.lam_s{t:02d}", tn(lam)))
            records.append((f"{pre}.sp_s{t:02d}", tn(sp)))
        records.append(
            (f"{pre}.min_dist",
             np.array(float(planner.pan.dune_layer.min_distance))))
        return out

    planner.pan.nrmp_layer.forward = nrmp_hook
    planner.pan.dune_layer.forward = dune_hook

    points = corridor_points(rng)
    state = np.array([[0.0], [20.0], [0.0]])

    for f in range(N_FRAMES):
        frame["f"], frame["it"] = f, 0
        records.append((f"f{f:02d}.state", state.copy()))
        records.append((f"f{f:02d}.points", points))
        action, _ = planner(state, points)
        records.append((f"f{f:02d}.action", action))
        records.append((f"f{f:02d}.n_iters", np.array(float(frame["it"]))))
        state = diff_step(state, action, planner.dt)

    write_nptf(os.path.join(out_dir, "frames.nptf"), records)
    print(f"dumped {N_FRAMES} frames, {len(records)} records "
          f"-> {os.path.join(out_dir, 'frames.nptf')}")

    dump_replay(out_dir)


if __name__ == "__main__":
    main(sys.argv[1])
