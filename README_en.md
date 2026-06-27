# neupan_cpp

Original Chinese [Readme](https://github.com/zhangkaiyuan007/neupan_cpp/blob/main/README.md)

This is **[NeuPAN](https://github.com/hanruihua/NeuPAN)** 's C++ transplant.

**[NeuPAN](https://github.com/hanruihua/NeuPAN)**(Ruihua Han et al., T-RO 2025)

`neupan_cpp` is a complete reproduction of the NeuPAN algorithm (DUNE + NRMP + PAN). **NO PyTorch or cvxpy** required at runtime, so we can run it on lightweight computers like Intel NUC and coexist with robot's autonomy stack. 

This repo consists of a standalone core library (`libneupan`) and a ROS 2 node (`neupan_cpp_ros`).

> Differential drive (rectangular robot + global path reference) has been implemented and validated in simulation.
> Omnidirectional drive adaptation is up next. See [Project Status and Roadmap](https://github.com/zhangkaiyuan007/neupan_cpp/blob/main/README_en.md#project-status-and-roadmap).

<p align="center">
  <a href="https://b23.tv/tUGI5hV">
    <img src="https://i2.hdslb.com/bfs/archive/f82fd050b0d7d6c9e91261f04574e1d8f26d688e.jpg@308w_174h" alt="Video Demo(bilibili)" width="500">
  </a>
</p>
<p align="center">Video Demo</p>

## Why This?

The official implementation uses Python + PyTorch + cvxpy, which consumes excessive resources on a real robot:

* **Memory:** The Python version has a large memory cost. On a Sentry NUC already running autoaim and perception tasks, the performance drops significantly.
* **Latency & Jitter:** cvxpy reconstructs and standardizes the optimization at every single frame, causing big lag.

This project aims to provide a **lightweight**, **determined**, C++ implemented NeuPAN, leaving computational resources for other tasks on real robot.

## Performance

### Correctness

Item-by-item comparison against the original Python implementation under identical inputs (reproducible via unit tests `test_dune`, `test_nrmp`, and `test_replay`):

| Inspection Item                 | Difference from Python                                       |
| ------------------------------- | ------------------------------------------------------------ |
| DUNE MLP layer-by-layer output  | Max abs err **1.9e-7** (float32 aligned with PyTorch)        |
| DUNE full forward (µ, distance) | err **~3.6e-7**, obstacle point selection **identical**      |
| NRMP single-frame action        | err **~1.3%**, objective function relative err **~1e-4**     |
| 50 frame closed-loop replay     | Average action err **< 1%**, minimum distance matches **exactly frame-by-frame** |

The NRMP residual comes from using a different but equivalent QP solver backend (OSQP instead of cvxpy/ECOS).

### Efficiency

Platform: **AMD Ryzen 9 9955HX** 

single corridor scenario, with 1100 obstacle points, prediction horizon `T = 10`, `iter_num = 2`:

| Phase                 | Python (PyTorch/cvxpy) | neupan_cpp | Speedup   |
| --------------------- | ---------------------- | ---------- | --------- |
| DUNE Forward          | 657 µs                 | 336 µs     | **~2.0×** |
| NRMP Solution         | 4609 µs                | 790 µs     | **~5.8×** |
| Full Planning Frame   | ~7.9 ms                | ~2.3 ms    | **~3.5×** |
| **Peak Memory (RSS)** | **386 MB**             | **9 MB**   | **~43×**  |

Without  `import torch` / `cvxpy`, inter-frame jitter is drastically lower. The QP is pre-assembled and warm-started, there's no per-frame reconstruction issue.

> End-to-end benchmark data on an Intel NUC is still pending, **we welcome contributions!**

* You can reproduce these results using the built-in benchmarking tools: `libneupan/tools/bench.cpp` and `libneupan/tools/bench_python.py`.
* Alternatively, run the full pipeline and monitor via tools like ```top/htop/btop```.

## Use Cases

**No costmap maintenance required**. It performs obstacle avoidance directly from real-time point clouds. The global layer only needs to provide a **topologically feasible reference line** (an A* global planner example is included in this repository).

## C++ Porting Details (vs. Original Python)

| Feature             | Original Python                                              | neupan_cpp                                                   |
| ------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| DUNE Inference      | PyTorch `nn.Module`                                          | float32 MLP (Linear / LayerNorm / Tanh / ReLU) that reads exported weights. **NO libtorch**. |
| NRMP Solving        | cvxpy + cvxpylayers, **reconstructs & standardizes** the problem every frame (ECOS). | Hessian and sparse structure remain constant; only updates gradients/bounds/values in-place every frame with **warm start**. |
| Linear Algebra      | NumPy / Torch Tensors                                        | Eigen 3.4                                                    |
| Weights / Test Data | `.pth` / pickle                                              | Compact `NPTF` binary (exported once using `tools/export_dune_weights.py`). |
| Mem cost            | Python + Torch + cvxpy (~386 MB)                             | ~9 MB                                                        |
| Robustness          | Assumes the QP always solves successfully                    | Unconverged solutions are not passed down; output velocity is safeguarded by a **speed box**. |

DUNE explicitly runs in **float32** to numerically align with PyTorch.

## Project Status and Roadmap

**Implemented & Tested (Gazebo Differential Drive Simulation):**

* Differential drive, rectangular robot, straight line (`line`) reference
* DUNE + NRMP + PAN, solved via OSQP
* ROS 2 (Humble) node, topics compatible with upstream `neupan_ros`
* A reference global planner (A* → `/initial_path`)

**Roadmap:**

* [ ] Other kinematics adaptation
* [ ] TBD

## Architecture

```
neupan_cpp/
├── libneupan/         # Core Algorithm
│   ├── include/neupan/ 
│   ├── src/
│   ├── models/        # Exported DUNE weights (.bin)
│   ├── tests/         # Unit tests
│   └── tools/         # Weight exporting, benchmarks
└── neupan_cpp_ros/    # ROS 2 (Humble) Wrapper
    ├── src/neupan_node.cpp         #Planner node
    ├── src/astar_global_node.cpp   #Reference global planner
    ├── config/ 
    ├── launch/
    └── test/           

```

`libneupan` is a pure CMake library that can be used independently of ROS; `neupan_cpp_ros` is an `ament_cmake` package. Both are built together using ```colcon```.

## Building

Dependencies: Eigen 3.4, [osqp](https://github.com/osqp/osqp) + [osqp-eigen](https://github.com/robotology/osqp-eigen), yaml-cpp, GTest (for test), and ROS 2 Humble.

```bash
# osqp
cd ~
git clone https://github.com/osqp/osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
sudo cmake --build . --target install
```

```bash
# osqp-eigen
cd ~
git clone https://github.com/gbionics/osqp-eigen.git
cd osqp-eigen
mkdir build
cd build
cmake ../
make
sudo make install
```

```bash
# In your ros2 workspace
cd src
git clone https://github.com/zhangkaiyuan007/neupan_cpp.git
colcon build
source install/setup.bash
```

To run unit tests:

```bash
cd libneupan && cmake -B build -DNEUPAN_BUILD_TESTS=ON && cmake --build build && ctest --test-dir build
```

## Usage

### ROS 2

```bash
ros2 launch neupan_cpp_ros neupan.launch.py

```

| Direction | Topic                                                        | Type                                                  |
| --------- | ------------------------------------------------------------ | ----------------------------------------------------- |
| Input     | `/scan`                                                      | `sensor_msgs/LaserScan` (Obstacle points)             |
| Input     | `/initial_path`                                              | `nav_msgs/Path` (Global reference line)               |
| Input     | `/neupan_goal`                                               | `geometry_msgs/PoseStamped` (Straight line reference) |
| Output    | `/neupan_cmd_vel`                                            | `geometry_msgs/Twist`                                 |
| Output    | `/neupan_plan`, `/neupan_initial_path`, `/dune_point_markers` … | Visualization                                         |

The planner configuration uses `planner.yaml`, which is **fully compatible with the upstream version** (key names match original NeuPAN). 

Key tuning parameters: `collision_threshold` (hard emergency stop distance), `adjust.d_max` / `d_min` (how close it can get to obstacles), `ref_speed`, and `max_speed`.

### Reference Global Planner (Optional)

`astar_global_node` subscribes  `/map`, inflates it based on the robot's radius, and runs A* algorithm after subscribe `/goal_pose`, then publish the resulting path to `/initial_path`.

### Training a DUNE Model for Your Robot

The DUNE network encodes the robot's shape directly into its weights, so you must **train a model using your own robot's dimensions**. Train it once using the upstream Python tools, then export it to the `NPTF` format required by this repository:

```bash
python tools/export_dune_weights.py model_5000.pth your_robot.bin 4
```

## Acknowledgments and License

This project is a derivative work of **[NeuPAN](https://github.com/hanruihua/NeuPAN)** (Ruihua Han et al.). All credit for the algorithm goes to the original authors—please cite their T-RO 2025 paper if you use this work.
