# neupan_cpp

**[NeuPAN](https://github.com/hanruihua/NeuPAN)** 的 C++ 移植——一个端到端、免地图、
直接吃点云的局部规划器(Ruihua Han 等,T-RO 2025)。

`neupan_cpp` 完整复现了 NeuPAN 算法(DUNE + NRMP + PAN),但**运行时不依赖 PyTorch、
不依赖 cvxpy**,因此能塞进嵌入式 NUC、和机器人其余的自治栈共存。它包含一个独立核心库
(`libneupan`)和一个 ROS 2 节点(`neupan_cpp_ros`)。

> 状态:**早期但可用。** 差速(矩形机器人 + 直线参考)已实现并在仿真中验证。
> 全向(omni)和阿克曼(Ackermann)在路线图上。见 [项目状态与路线图](#项目状态与路线图)。

---

## 为什么做这个项目

NeuPAN 本身是个很好的局部规划器——直接从原始点云避障,不需要代价地图,输出平滑且满足
运动学约束的轨迹。但官方实现是 Python + PyTorch + cvxpy,放到真实机器人上**太重了**:

- **内存:** Python 版常驻内存约 **386 MB**(PyTorch + cvxpy + 它们的原生库)。在一台
  已经跑着自瞄、视觉和其余导航的 RoboMaster 哨兵 NUC 上,这个占用根本吃不消。
- **延迟与抖动:** cvxpy 每一帧都要重新构建并规范化(canonicalize)优化问题,带来很长的
  尾延迟。
- **共存问题:** 自瞄已经把 CPU 占满,Python 版的 NeuPAN 抢不到稳定的算力——在目标硬件
  上根本跑不顺。

而且此前没有任何 NeuPAN 的 C++ 实现。这个项目就是来补这个空白:一个精简、确定性强、
可嵌入的 NeuPAN,给机器上其它任务留出余量。

---

## 效果

### 正确性——算法被忠实复现

在相同输入下与原版 Python 实现逐项对比(单元测试 `test_dune`、`test_nrmp`、
`test_replay` 可复现):

| 检查项 | 与 Python 的差异 |
|---|---|
| DUNE MLP 逐层输出 | 最大绝对误差 **1.9e-7**(float32 与 PyTorch 对齐) |
| DUNE 完整前向(µ、距离) | 误差 **~3.6e-7**,障碍点选择**完全一致** |
| NRMP 单帧动作 | 误差 **~1.3%**,目标函数相对误差 **~1e-4** |
| 50 帧闭环回放 | 平均动作误差 **< 1%**,最小距离**逐帧精确一致** |

NRMP 那点残差来自不同但等价的 QP 求解后端(OSQP 替代 cvxpy/ECOS),不是建模差异。

### 性能——同样的结果,只用一小部分开销

测试平台 **AMD Ryzen 9 9955HX**,单走廊场景,1100 个障碍点,预测时域 `T = 10`,
`iter_num = 2`:

| 阶段 | Python(PyTorch/cvxpy) | neupan_cpp | 倍数 |
|---|---|---|---|
| DUNE 前向 | 657 µs | 336 µs | **~2.0×** |
| NRMP 求解 | 4609 µs | 790 µs | **~5.8×** |
| 完整规划帧 | ~7.9 ms | ~2.3 ms | **~3.5×** |
| **峰值内存(RSS)** | **386 MB** | **9 MB** | **~43×** |

另外:**没有动辄数秒的 `import torch` / cvxpy 启动**;帧间抖动也低得多——QP 是预装配 +
warm start 的,不存在每帧重建问题。

### ⚠️ 关键:为什么官方只能跑 ~15 Hz,而上表 Python 看起来"还行"?

> 这是最容易被误读的地方,务必看清。

上表里 Python 单帧 7.9 ms(≈126 Hz)看着不慢——**因为我是在 AMD Ryzen 9 9955HX 这种
旗舰桌面 CPU 上测的**。这恰恰说明:**桌面 CPU 太强,把差距掩盖了**,所以倍数(3.5×)显得
不大。但机器人不在桌面上跑。

NeuPAN 官方部署实测只有 **~15 Hz(约 66 ms/帧)**,原因在于真实运行环境:

1. **硬件是嵌入式弱核**(NUC / Jetson / ARM),不是旗舰桌面 U。光这一项就是数倍到一个数量级
   的差距——66 ms vs 7.9 ms 之间那 ~8× 正是桌面与嵌入式的鸿沟。
2. **CPU 是共享的**:自瞄、视觉、定位同时在抢核,Python 只能拿到一小片算力。
3. **cvxpy 每帧重建求解**:不是简单地解一个固定 QP,而是每次重新规范化整个问题图,这部分
   开销在弱核上被严重放大,且随问题规模增长。
4. **386 MB 工作集在小缓存的嵌入式平台上反复 cache miss**,桌面微基准完全反映不出来。

C++ 版把这些开销逐条干掉:**预装配稀疏 QP + warm start**(无每帧重建)、**9 MB 常驻**
(无 cache 压力)、**无解释器 / autograd 运行时**。所以在弱核上,C++ 的领先幅度远大于
桌面那 3.5×;而 **43× 的内存下降是硬件无关的**,这正是能在已经跑着自瞄的 NUC 上和它
共存的根本原因。

> 诚实说明:目标板(NUC)上的端到端对比尚待补充——欢迎贡献。但上面的桌面数字已是**保守
> 下界**,且内存优势与硬件无关。

用仓库自带工具可复现:`libneupan/tools/bench.cpp`(C++)、`libneupan/tools/bench_python.py`(Python)。

---

## NeuPAN 的思想优势,以及为什么值得移植

NeuPAN 在一个交替最小化(PAN)循环里耦合两个模块:

- **DUNE** —— 一个小 MLP,把每个原始障碍点映射成隐空间距离特征(即"点到机器人多边形
  距离"问题的对偶变量)。正是它让 NeuPAN 能**直接处理每一个点**,无需占用栅格、膨胀或聚类。
- **NRMP** —— 一个模型预测 QP,把这些隐空间距离转成平滑、满足运动学、无碰撞的控制序列。

实际好处:**不需要地图、不需要维护代价地图**,直接从实时点云避障,而且输出的控制本身就已经
满足机器人运动学和速度/加速度限制。全局那一层只需要给一条**拓扑上可行的参考线**——精细的
局部避障是 NeuPAN 的活。(本仓库附带一个最小 A\* 全局规划器正好干这个,见下。)

## C++ 移植做了哪些改造(对比 Python 原版)

算法完全一致,但运行时是为嵌入式重写的:

| | Python 原版 | neupan_cpp |
|---|---|---|
| DUNE 推理 | PyTorch `nn.Module` | 手写 float32 MLP(Linear / LayerNorm / Tanh / ReLU),读导出的权重,**不依赖 libtorch** |
| NRMP 求解 | cvxpy + cvxpylayers,**每帧重建并规范化**问题(ECOS) | **预装配稀疏 QP**,用 **OSQP** + osqp-eigen;Hessian 和稀疏结构恒定,每帧只就地更新梯度/边界/数值,并 **warm start** |
| 线性代数 | NumPy / Torch 张量 | Eigen 3.4 |
| 权重 / 测试数据 | `.pth` / pickle | 紧凑的 `NPTF` 二进制(用 `tools/export_dune_weights.py` 导出一次) |
| 占用 | Python + Torch + cvxpy(~386 MB) | ~9 MB 原生进程,瞬时启动 |
| 健壮性 | 假设 QP 总能解出 | 未收敛的解绝不向下传播;输出速度按 speed box 钳位作为安全兜底 |

DUNE 特意用 **float32** 运行以在数值上对齐 PyTorch;其余部分用 double。

---

## 项目状态与路线图

**已实现并测试(Gazebo 差速仿真):**
- 差速、矩形机器人、直线(`line`)参考
- DUNE + NRMP + PAN,OSQP 求解
- ROS 2(Humble)节点,话题与上游 `neupan_ros` 兼容
- 一个最小参考全局规划器(占用栅格上的 A\* → `/initial_path`)

**路线图:**
- [ ] 全向(omni)运动学(下一个——需求最大)
- [ ] 阿克曼(Ackermann)
- [ ] Dubins / Reeds-Shepp 参考曲线
- [ ] 动态障碍点速度
- [ ] Nav2 controller 插件
- [ ] 依赖一键构建(FetchContent 内置 OSQP 等)

---

## 架构

```
neupan_cpp/
├── libneupan/         # 核心算法,不依赖 ROS
│   ├── include/neupan/  types, mlp, robot, dune, nrmp, pan, initial_path, neupan_planner
│   ├── src/
│   ├── models/          导出的 DUNE 权重 (.bin)
│   ├── tests/           gtest 单元 + 回放测试
│   └── tools/           权重导出、测试数据 dump、基准测试
└── neupan_cpp_ros/    # ROS 2(Humble)封装
    ├── src/neupan_node.cpp          规划器节点
    ├── src/astar_global_node.cpp    参考全局规划器
    ├── config/ launch/
    └── test/            无头集成测试
```

`libneupan` 是纯 CMake 库,也可脱离 ROS 单独使用;`neupan_cpp_ros` 是 `ament_cmake` 包。
两者一起用 colcon 构建(FAST-LIO / LIO-SAM 那种核心库 + ROS 包同仓库的方式)。

---

## 构建

依赖:Eigen 3.4、[osqp](https://github.com/osqp/osqp) +
[osqp-eigen](https://github.com/robotology/osqp-eigen)、yaml-cpp、GTest(测试用)、
以及 ROS 2 Humble(`neupan_cpp_ros`)。

```bash
# 在 colcon 工作区中
git clone <this repo> src/neupan_cpp
colcon build
source install/setup.bash
```

跑单元测试:

```bash
cd libneupan && cmake -B build -DNEUPAN_BUILD_TESTS=ON && cmake --build build && ctest --test-dir build
```

---

## 使用

### 库(无 ROS)

```cpp
#include "neupan/neupan_planner.hpp"

auto planner = neupan::NeuPANPlanner::fromYaml("planner.yaml", "diff_default.bin");

neupan::Vec3  state(x, y, theta);        // map 系下机器人位姿
neupan::Mat2X points(2, N);              // map 系下障碍点
neupan::NeuPANPlanner::Info info;

neupan::Vec2 action = planner.forward(state, points, info);  // [v, omega]
// info.arrive / info.stop / info.min_distance / info.opt_s(预测轨迹) ...
```

参考线可以来自:航点(`setWaypoints`)、目标点(`updateInitialPathFromGoal`,直线)、
或外部全局路径(`setInitialPath`,例如来自 A\*)。

### ROS 2 节点

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

`astar_global_node` 订阅 `/map` 上的 latched `nav_msgs/OccupancyGrid`,按机器人半径膨胀,
从当前 TF 位姿到 RViz "2D Nav Goal" 跑 A\*,把路径发到 `/initial_path`。它只需要**拓扑可行**
——精细避障交给 NeuPAN——所以轻度膨胀、不要代价地图就够了。

### 为你的机器人训练 DUNE 模型

DUNE 网络把机器人外形编码进了权重,所以**必须用你自己的外形训练一个模型**。用上游 Python
工具训练一次,再导出成本库读取的 `NPTF` 格式:

```bash
python tools/export_dune_weights.py model_5000.pth your_robot.bin 4
```

---

## 致谢与许可

本项目是 **[NeuPAN](https://github.com/hanruihua/NeuPAN)**(Ruihua Han 等)的衍生作品。
算法的全部功劳归原作者——如果你使用本工作,请引用他们的 T-RO 2025 论文。

采用 **GPL-3.0-or-later** 许可,与上游 NeuPAN / neupan_ros 保持一致。
