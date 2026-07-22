"""Run the original NeuPAN IR-SIM Ackermann scenario with Python or C++.

The Python planner is also used as a sensor/path adapter for the C++ backend:
it converts IR-SIM lidar scans to global obstacle points and generates the
upstream Dubins reference path. Planning and optimization remain in C++.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import irsim
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ENV = REPO_ROOT / "example" / "corridor" / "acker" / "env.yaml"
DEFAULT_PY_CONFIG = REPO_ROOT / "example" / "corridor" / "acker" / "planner.yaml"
DEFAULT_CPP_CONFIG = (
    REPO_ROOT / "neupan_cpp" / "neupan_cpp_ros" / "config" / "planner_acker.yaml"
)
DEFAULT_CPP_MODEL = (
    REPO_ROOT / "neupan_cpp" / "libneupan" / "models" / "acker_default.bin"
)


def _load_cpp_module(build_dir: Path | None):
    if build_dir is not None:
        sys.path.insert(0, str(build_dir.resolve()))
    try:
        import neupan_cpp_py
    except ImportError as exc:
        raise RuntimeError(
            "neupan_cpp_py is not importable; pass --cpp-module-dir pointing "
            "to libneupan/build-python/python/Release"
        ) from exc
    return neupan_cpp_py


def _path_matrix(path) -> np.ndarray:
    rows = []
    for point in path:
        values = np.asarray(point, dtype=np.float64).reshape(-1)
        if values.size < 4:
            raise ValueError("upstream path point must contain x, y, yaw, and gear")
        # Dubins/Reeds-Shepp generators may include curvature before gear.
        rows.append([values[0], values[1], values[2], values[-1]])
    return np.asarray(rows, dtype=np.float64)


def _trajectory_columns(states: np.ndarray):
    return [states[:, i : i + 1] for i in range(states.shape[1])]


def run_episode(
    backend: str,
    env_file: Path,
    python_config: Path,
    cpp_config: Path,
    cpp_model: Path,
    cpp_module_dir: Path | None,
    max_steps: int,
    display: bool,
    save_animation: bool,
):
    from neupan import configuration, neupan

    env = irsim.make(
        str(env_file), save_ani=save_animation, display=display, full=False
    )
    adapter = neupan.init_from_yaml(str(python_config))
    configuration.time_print = False
    initial_state = np.asarray(env.get_robot_state(), dtype=np.float64)[:3].reshape(3, 1)
    adapter.set_initial_path_from_state(initial_state)

    if backend == "cpp":
        module = _load_cpp_module(cpp_module_dir)
        planner = module.Planner(str(cpp_config), str(cpp_model))
        planner.set_initial_path(_path_matrix(adapter.initial_path))
    else:
        planner = adapter

    states = []
    actions = []
    times_ms = []
    min_distances = []
    arrived = False
    stopped = False
    collided = False

    for step in range(max_steps):
        robot_state = np.asarray(env.get_robot_state(), dtype=np.float64)[:3].reshape(3, 1)
        lidar_scan = env.get_lidar_scan()
        points = adapter.scan_to_point(robot_state, lidar_scan)
        if points is None:
            points = np.empty((2, 0), dtype=np.float64)
        else:
            points = np.asarray(points, dtype=np.float64)
        if points.shape[1] == 0:
            # Upstream Python DUNE calls min() on the distance tensor and
            # cannot consume an empty scan. A remote sentinel preserves the
            # no-nearby-obstacle semantics and is fed to both backends.
            points = robot_state[:2] + np.array([[1e3], [1e3]])

        started = time.perf_counter_ns()
        if backend == "cpp":
            action, info = planner.forward(robot_state.reshape(3), points)
            action = np.asarray(action, dtype=np.float64).reshape(2, 1)
        else:
            action, info = planner(robot_state, points, None)
            action = np.asarray(action, dtype=np.float64).reshape(2, 1)
        times_ms.append((time.perf_counter_ns() - started) / 1e6)

        states.append(robot_state.reshape(3).copy())
        actions.append(action.reshape(2).copy())
        arrived = bool(info.get("arrive", False))
        stopped = bool(info.get("stop", False))
        min_distances.append(
            float("nan")
            if arrived
            else float(
                info.get("min_distance", np.nan)
                if backend == "cpp"
                else planner.min_distance
            )
        )

        if display:
            dune_points = info.get("dune_points", np.empty((2, 0)))
            nrmp_points = info.get("nrmp_points", np.empty((2, 0)))
            opt_s = info.get("opt_s")
            ref_s = info.get("ref_s")
            if backend == "python":
                dune_points = planner.dune_points
                nrmp_points = planner.nrmp_points
                opt_trajectory = planner.opt_trajectory
                ref_trajectory = planner.ref_trajectory
            else:
                opt_trajectory = _trajectory_columns(np.asarray(opt_s))
                ref_trajectory = _trajectory_columns(np.asarray(ref_s))
            env.draw_points(dune_points, s=25, c="g", refresh=True)
            env.draw_points(nrmp_points, s=13, c="r", refresh=True)
            env.draw_trajectory(opt_trajectory, "r", refresh=True)
            env.draw_trajectory(ref_trajectory, "b", refresh=True)
            if step == 0:
                env.draw_trajectory(
                    adapter.initial_path, traj_type="-k", show_direction=True
                )

        if arrived:
            break
        env.step(action)
        collided = bool(env.robot.collision)
        if display:
            env.render()
        if collided or env.done():
            break

    env.end(0, ani_name=f"irsim_ackermann_{backend}")
    states_np = np.asarray(states)
    actions_np = np.asarray(actions)
    timing_np = np.asarray(times_ms)
    return {
        "backend": backend,
        "steps": len(states),
        "arrived": arrived,
        "stopped": stopped,
        "collided": collided,
        "state": states_np,
        "action": actions_np,
        "min_distance": np.asarray(min_distances),
        "timing_ms": timing_np,
        "timing_mean_ms": float(timing_np.mean()) if timing_np.size else float("nan"),
        "timing_p95_ms": float(np.percentile(timing_np, 95)) if timing_np.size else float("nan"),
    }


def summarize(result: dict) -> dict:
    return {
        "backend": result["backend"],
        "steps": result["steps"],
        "arrived": result["arrived"],
        "stopped": result["stopped"],
        "collided": result["collided"],
        "first_action": result["action"][0].tolist() if result["steps"] else [],
        "final_state": result["state"][-1].tolist() if result["steps"] else [],
        "minimum_distance": float(np.nanmin(result["min_distance"])),
        "timing_mean_ms": result["timing_mean_ms"],
        "timing_p95_ms": result["timing_p95_ms"],
    }


def comparison(python_result: dict, cpp_result: dict) -> dict:
    count = min(python_result["steps"], cpp_result["steps"])
    state_error = np.abs(
        python_result["state"][:count] - cpp_result["state"][:count]
    )
    action_error = np.abs(
        python_result["action"][:count] - cpp_result["action"][:count]
    )
    return {
        "compared_steps": count,
        "state_mae": state_error.mean(axis=0).tolist(),
        "state_max_abs": state_error.max(axis=0).tolist(),
        "action_mae": action_error.mean(axis=0).tolist(),
        "action_max_abs": action_error.max(axis=0).tolist(),
        "speedup_mean": python_result["timing_mean_ms"]
        / cpp_result["timing_mean_ms"],
        "speedup_p95": python_result["timing_p95_ms"]
        / cpp_result["timing_p95_ms"],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("python", "cpp", "both"), default="cpp")
    parser.add_argument("--env", type=Path, default=DEFAULT_ENV)
    parser.add_argument("--python-config", type=Path, default=DEFAULT_PY_CONFIG)
    parser.add_argument("--cpp-config", type=Path, default=DEFAULT_CPP_CONFIG)
    parser.add_argument("--cpp-model", type=Path, default=DEFAULT_CPP_MODEL)
    parser.add_argument("--cpp-module-dir", type=Path)
    parser.add_argument("--max-steps", type=int, default=500)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--save-animation", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    backends = ("python", "cpp") if args.backend == "both" else (args.backend,)
    results = [
        run_episode(
            backend,
            args.env,
            args.python_config,
            args.cpp_config,
            args.cpp_model,
            args.cpp_module_dir,
            args.max_steps,
            args.display and len(backends) == 1,
            args.save_animation and len(backends) == 1,
        )
        for backend in backends
    ]

    report = {"episodes": [summarize(result) for result in results]}
    if len(results) == 2:
        report["comparison"] = comparison(results[0], results[1])
    rendered = json.dumps(report, indent=2, ensure_ascii=False)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
