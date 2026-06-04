# 功能包: MADER (Multi-Agent and Dynamic Environment Replan)

## 1. 功能概述

*   **一句话总结**: 基于Gurobi/NLOPT优化器的多智能体动态环境下的轨迹规划算法，支持实时避障和重规划
*   **核心节点**: `mader_node`
*   **算法类型**: 基于凸优化的多智能体轨迹规划算法，使用分段多项式轨迹表示和动态凸约束生成

## 2. 依赖关系

*   **主要依赖项**:
    *   `roscpp`: C++客户端库，用于ROS节点通信
    *   `snapstack_msgs`: MIT快速自主飞行消息库，提供状态和目标消息类型
    *   `mader_msgs`: MADER自定义消息类型，包含轨迹和规划状态信息
    *   `gazebo_msgs`: Gazebo仿真环境消息类型
    *   `rviz_visual_tools`: RViz可视化工具库
    *   `decomp_util`/`decomp_ros_utils`: 凸分解算法库，用于环境几何表示
    *   `separator`: 分离超平面生成库
    *   `jsk_rviz_plugins`: JSK实验室RViz插件，用于高级可视化
    *   **外部依赖**: Gurobi优化器/NLOPT、CGAL几何算法库、Eigen3线性代数库

## 3. 接口说明 (API)

### 3.1 订阅的话题 (Inputs)

| 话题名称 | 消息类型 | 描述 |
| :--- | :--- | :--- |
| `~state` | `snapstack_msgs/State` | 无人机当前状态（位置、速度、姿态等），用于轨迹规划的初始条件。 |
| `~who_plans` | `mader_msgs/WhoPlans` | 多智能体规划协调信号，指示当前哪个智能体负责规划。 |
| `~term_goal` | `geometry_msgs/PoseStamped` | 终端目标位置，用户通过RViz 2D Nav Goal工具或程序发布。 |
| `~traj` | `mader_msgs/DynTraj` | 其他智能体的动态轨迹信息，用于避障约束生成。 |

### 3.2 发布的话题 (Outputs)

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| `~traj` | `mader_msgs/DynTraj` | ~10 | 发布自身规划的动态轨迹，供其他智能体避障使用。 |
| `~goal` | `snapstack_msgs/Goal` | ~100 | 发布当前时刻的目标状态点，供底层控制器使用。 |
| `~setpoint` | `geometry_msgs/PoseStamped` | ~100 | 发布控制设定点，与goal话题功能类似。 |
| `/markers/*` | `visualization_msgs/MarkerArray` | ~10 | 发布多种可视化标记（轨迹、障碍物、安全区域等）。 |

### 3.3 发布/订阅的坐标系 (TF)

*   **需要输入的TF**: `world` → `base_link` (无人机在世界坐标系中的位姿)
*   **发布的TF**: 无（MADER主要在world坐标系下工作）

### 3.4 提供的服务 (Services)

MADER主要通过话题进行通信，未提供标准ROS服务接口。

## 4. 核心算法原理

MADER (Multi-Agent Dynamic Environment Replanner) 是一种专为多智能体系统设计的实时轨迹规划算法，特别适用于动态环境下的无人机群体协同导航。

该算法的核心思想是将多智能体避障问题转化为一系列凸优化问题，通过动态生成分离超平面来确保智能体间的安全间距，同时保证轨迹的动力学可行性和平滑性。

#### 步骤1: 轨迹参数化 (Trajectory Parameterization)
MADER使用分段多项式（Piecewise Polynomial）来表示轨迹，具体可选择MINVO基函数、B-spline或Bezier曲线：
$$
\mathbf{r}(t) = \sum_{i=0}^{n} \mathbf{c}_i \phi_i(t), \quad t \in [t_0, t_f]
$$
其中 $\mathbf{c}_i$ 是控制点系数，$\phi_i(t)$ 是基函数。这种参数化确保了轨迹的连续性和可微性。

#### 步骤2: 动态约束生成 (Dynamic Constraint Generation)
对于每个时间间隔 $[t_k, t_{k+1}]$，算法为其他智能体的预测轨迹生成凸包(Convex Hull)：
$$
\text{ConvexHull}(\{\mathbf{r}_j(t) : t \in [t_k, t_{k+1}]\}) \quad \forall j \neq i
$$
然后通过求解线性规划问题找到分离超平面：
$$
\mathbf{n}^T(\mathbf{r}_i(t) - \mathbf{p}_{\text{ref}}) \geq d_{\text{safe}}, \quad \forall t \in [t_k, t_{k+1}]
$$
其中 $\mathbf{n}$ 是超平面法向量，$d_{\text{safe}}$ 是安全距离。

#### 步骤3: 凸优化求解 (Convex Optimization)
将轨迹规划问题表述为二次约束二次规划(QCQP)问题：
$$
\begin{align}
\min_{\mathbf{c}} \quad & J(\mathbf{c}) = w_1\|\mathbf{r}^{(k)}(t)\|^2 + w_2(\mathbf{r}(t_f) - \mathbf{g})^2 \\
\text{s.t.} \quad & \|\mathbf{v}(t)\| \leq v_{\max}, \quad \|\mathbf{a}(t)\| \leq a_{\max} \\
& \mathbf{n}_j^T(\mathbf{r}(t) - \mathbf{p}_j) \geq d_{\text{safe}}, \quad \forall j
\end{align}
$$
其中第一项惩罚高阶导数（平滑性），第二项确保到达目标点。

#### 步骤4: 实时重规划 (Real-time Replanning)
算法采用滚动时域策略，在每个规划周期：
1. 更新智能体状态和环境信息
2. 重新计算其他智能体轨迹的凸包
3. 更新约束条件并重新求解优化问题
4. 执行轨迹的前一小段，然后进入下一规划周期

#### 步骤5: 协调机制 (Coordination Mechanism)
多智能体间通过 `WhoPlans` 消息进行规划优先级协调：
- 优先级高的智能体先规划轨迹
- 优先级低的智能体将高优先级智能体的轨迹作为动态障碍物
- 通过这种分层规划避免求解大规模耦合优化问题

算法的优势在于：
1. **实时性**: 通过凸优化保证求解效率
2. **安全性**: 动态约束确保智能体间安全距离
3. **平滑性**: 分段多项式表示保证轨迹平滑
4. **可扩展性**: 分层协调机制支持大规模智能体群

## 5. 使用与配置

### 5.1 启动示例

```bash
# 单智能体仿真
roslaunch mader single_agent_simulation.launch

# 设置目标点（在RViz中使用2D Nav Goal工具，或通过命令行）
rostopic pub /SQ01s/term_goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "world"}, pose: {position: {x: 10, y: 0, z: 1}, orientation: {w: 1.0}}}'

# 多智能体仿真（需要四个终端）
# 终端1: 启动环境和可视化
roslaunch mader mader_general.launch type_of_environment:="dynamic_forest"

# 终端2: 启动多无人机
roslaunch mader many_drones.launch action:=start

# 终端3: 启动MADER规划器
roslaunch mader many_drones.launch action:=mader

# 终端4: 发送目标点
roslaunch mader many_drones.launch action:=send_goal

# Octopus搜索演示
roslaunch mader octopus_search.launch
```

### 5.2 关键参数配置

在 `param/mader.yaml` 中的重要参数：

```yaml
# 无人机物理参数
drone_radius: 0.05      # 无人机半径 (m)
v_max: [3.5, 3.5, 3.5] # 最大速度 (m/s)
a_max: [20.0, 20.0, 9.6] # 最大加速度 (m/s²)

# 规划参数
Ra: 4.0                 # 规划球半径 (m)
factor_alpha: 1.5       # 时间分配因子
alpha_shrink: 0.95      # 收缩因子

# 优化器设置
basis: "MINVO"          # 基函数类型 (MINVO/B_SPLINE/BEZIER)
num_pol: 4              # 多项式段数
deg_pol: 3              # 多项式阶数
```

### 5.3 依赖安装

```bash
# 安装Gurobi优化器（推荐）
# 下载并安装Gurobi，然后测试安装
gurobi.sh

# 自动安装脚本
cd ~/ws/src
git clone https://github.com/mit-acl/mader.git
cd ..
bash src/mader/install_and_compile.sh

# 备选：使用NLOPT（性能较低）
# 在CMakeLists.txt中设置 USE_GUROBI 为 OFF
bash src/mader/install_nlopt.sh
```

### 5.4 Docker支持

```bash
# 构建Docker镜像
cd mader/mader/docker
docker build -t mader .

# 运行（需要Gurobi Web License）
docker run --volume=$PWD/gurobi.lic:/opt/gurobi/gurobi.lic:ro -it mader
```