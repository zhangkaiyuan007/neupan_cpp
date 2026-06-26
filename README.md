# neupan_cpp

**[NeuPAN](https://github.com/hanruihua/NeuPAN)** 的 C++ 移植(Ruihua Han 等,T-RO 2025)

`neupan_cpp` 完整复现了 NeuPAN 算法(DUNE + NRMP + PAN),但**运行时不依赖 PyTorch、
不依赖 cvxpy**,因此能塞进 NUC 且与机器人其余的自治栈共存。它包含一个独立核心库
(`libneupan`)和一个 ROS 2 节点(`neupan_cpp_ros`)。

> 差速 (矩形机器人 + 全局路径参考) 已实现并在仿真中验证。
> 接下来将首先完成全向适配。见 [项目状态与路线图](#项目状态与路线图)。

<p align="center">
  <a href="https://b23.tv/tUGI5hV">
    <img src="https://i2.hdslb.com/bfs/archive/f82fd050b0d7d6c9e91261f04574e1d8f26d688e.jpg@308w_174h" alt="效果展示" width="400">
  </a>
</p>
<p align="center">效果展示</p>

## 为什么做这个项目

官方实现是 Python + PyTorch + cvxpy,放到真实机器人上占用大:

- **内存:** Python 版内存占用大。在一台已经跑着自瞄和感知的哨兵 NUC 上,性能下降较为严重。
- **延迟与抖动:** cvxpy 每一帧都要重新构建并规范化优化问题,带来延迟。

本项目旨在提供一个精简、确定性强、CPP 实现的 NeuPAN,给机器上其它任务留出余量。

## 效果

### 正确性

在相同输入下与原版 Python 实现逐项对比(单元测试 `test_dune`、`test_nrmp`、`test_replay` 可复现):

| 检查项 | 与 Python 的差异 |
|---|---|
| DUNE MLP 逐层输出 | 最大绝对误差 **1.9e-7**(float32 与 PyTorch 对齐) |
| DUNE 完整前向(µ、距离) | 误差 **~3.6e-7**,障碍点选择**完全一致** |
| NRMP 单帧动作 | 误差 **~1.3%**,目标函数相对误差 **~1e-4** |
| 50 帧闭环回放 | 平均动作误差 **< 1%**,最小距离**逐帧精确一致** |

NRMP 残差来自不同但等价的 QP 求解后端(OSQP 替代 cvxpy/ECOS)。

### 性能

测试平台 **AMD Ryzen 9 9955HX**,单走廊场景,1100 个障碍点,预测时域 `T = 10`,`iter_num = 2`:

| 阶段 | Python(PyTorch/cvxpy) | neupan_cpp | 倍数 |
|---|---|---|---|
| DUNE 前向 | 657 µs | 336 µs | **~2.0×** |
| NRMP 求解 | 4609 µs | 790 µs | **~5.8×** |
| 完整规划帧 | ~7.9 ms | ~2.3 ms | **~3.5×** |
| **峰值内存(RSS)** | **386 MB** | **9 MB** | **~43×** |

另外，没有了 `import torch` / `cvxpy` 启动，帧间抖动也低得多，QP 是预装配 +
warm start 的,不存在每帧重建问题。

> NUC 上的端到端对比数据尚待补充——欢迎贡献。

- 用仓库自带工具可复现:`libneupan/tools/bench.cpp`(C++)、`libneupan/tools/bench_python.py`(Python)。

- 或直接跑全流程，并通过 top/htop/btop 工具查看

## 使用场景

**不用维护代价地图**,直接从实时点云避障。全局层只需要给一条**拓扑上可行的参考线**
(本仓库附带一个 A\* 全局规划器示例)

## C++ 移植(对比 Python 原版)

| | Python 原版 | neupan_cpp |
|---|---|---|
| DUNE 推理 | PyTorch `nn.Module` | 手写 float32 MLP(Linear / LayerNorm / Tanh / ReLU),读导出的权重,**不依赖 libtorch** |
| NRMP 求解 | cvxpy + cvxpylayers,**每帧重建并规范化**问题(ECOS) | Hessian 和稀疏结构恒定,每帧只就地更新梯度/边界/数值,并 **warm start** |
| 线性代数 | NumPy / Torch 张量 | Eigen 3.4 |
| 权重 / 测试数据 | `.pth` / pickle | 紧凑的 `NPTF` 二进制(用 `tools/export_dune_weights.py` 导出一次) |
| 占用 | Python + Torch + cvxpy(~386 MB) | ~9 MB |
| 鲁棒性 | 假设 QP 总能解出 | 未收敛的解不向下传播;输出速度用 speed box 兜底 |

DUNE 特意用 **float32** 运行以在数值上对齐 PyTorch。

## 项目状态与路线图

**已实现并测试(Gazebo 差速仿真):**
- 差速、矩形机器人、直线(`line`)参考
- DUNE + NRMP + PAN,OSQP 求解
- ROS 2(Humble)节点,话题与上游 `neupan_ros` 兼容
- 一个参考全局规划器( A\* → `/initial_path`)

**路线图:**
- [ ] 其他运动学适配
- [ ] 待定

## 架构

```
neupan_cpp/
├── libneupan/         # 核心算法
│   ├── include/neupan/ 
│   ├── src/
│   ├── models/          导出的 DUNE 权重 (.bin)
│   ├── tests/           单元测试
│   └── tools/           权重导出、基准测试
└── neupan_cpp_ros/    # ROS 2(Humble)封装
    ├── src/neupan_node.cpp          规划器节点
    ├── src/astar_global_node.cpp    参考全局规划器
    ├── config/ launch/
    └── test/          
```

`libneupan` 是纯 CMake 库,可脱离 ROS 单独使用;`neupan_cpp_ros` 是 `ament_cmake` 包。
两者一起用 colcon 构建。

## 构建

依赖:Eigen 3.4、[osqp](https://github.com/osqp/osqp) +
[osqp-eigen](https://github.com/robotology/osqp-eigen)、yaml-cpp、GTest(测试用)、
以及 ROS 2 Humble。

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
# 在自己的工作空间下
cd src
git clone https://github.com/zhangkaiyuan007/neupan_cpp.git
colcon build
source install/setup.bash
```

跑单元测试:

```bash
cd libneupan && cmake -B build -DNEUPAN_BUILD_TESTS=ON && cmake --build build && ctest --test-dir build
```

## 使用

### ROS 2 

```bash
ros2 launch neupan_cpp_ros neupan.launch.py
```

| 方向 | 话题 | 类型 |
|---|---|---|
| 输入 | `/scan` | `sensor_msgs/LaserScan`(障碍点) |
| 输入 | `/initial_path` | `nav_msgs/Path`(全局参考线) |
| 输入 | `/neupan_goal` | `geometry_msgs/PoseStamped`(直线参考) |
| 输出 | `/neupan_cmd_vel` | `geometry_msgs/Twist` |
| 输出 | `/neupan_plan`、`/neupan_initial_path`、`/dune_point_markers` … | 可视化 |

规划器配置是**与上游兼容**的 `planner.yaml`(键名与原版 NeuPAN 一致)。主要调参:
`collision_threshold`(硬急停距离)、`adjust.d_max` / `d_min`(贴障碍多近)、`ref_speed`、
`max_speed`。

### 一个参考全局规划器(可选)

`astar_global_node` 订阅 `/map` ,按机器人半径膨胀,通过订阅 `/goal_pose` 跑 A\*,把路径发到 `/initial_path`。

### 为你的机器人训练 DUNE 模型

DUNE 网络把机器人外形编码进了权重,所以**必须用你自己机器人的尺寸训一个模型**。用上游 Python
工具训练一次,再导出为本库读取的 `NPTF` 格式:

```bash
python tools/export_dune_weights.py model_5000.pth your_robot.bin 4
```

## 致谢与许可

本项目是 **[NeuPAN](https://github.com/hanruihua/NeuPAN)**(Ruihua Han 等)的衍生产物。
算法的全部功劳归原作者——如果你使用本工作,请引用他们的 T-RO 2025 论文。
