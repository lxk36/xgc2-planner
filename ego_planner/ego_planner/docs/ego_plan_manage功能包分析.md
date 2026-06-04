# 功能包: ego_plan_manage

## 1. 功能概述

*   **一句话总结**: 基于有限状态机(FSM)的无人机轨迹规划管理系统，协调路径搜索、轨迹优化和实时重规划
*   **核心节点**: `ego_planner_node`
*   **算法类型**: 有限状态机控制 + A*路径搜索 + B样条轨迹优化

## 2. 依赖关系

*   **主要依赖项**:
    *   `roscpp`: C++客户端库
    *   `visualization_msgs`: 可视化消息类型
    *   `ego_plan_env`: 环境地图管理模块
    *   `ego_path_searching`: A*路径搜索算法
    *   `ego_bspline_opt`: B样条轨迹优化算法
    *   `ego_traj_utils`: 轨迹工具库
    *   `message_generation`: 自定义消息生成
    *   `Eigen3`: 线性代数运算库

## 3. 接口说明 (API)

### 3.1 订阅的话题 (Inputs)

| 话题名称 | 消息类型 | 描述 |
| :--- | :--- | :--- |
| `/odom_world` | `nav_msgs/Odometry` | 无人机全局位姿信息 |
| `/planning/goal` | `geometry_msgs/PoseStamped` | 目标位置 |
| `/trigger_rviz` | `geometry_msgs/PoseStamped` | RViz中设置起点和终点 |
| `/mandatory_stop` | `std_msgs/Empty` | 强制停止信号 |
| `/map_ros/pose` | `geometry_msgs/PoseStamped` | 仿真环境位姿(可选) |
| `/map_ros/odometry` | `nav_msgs/Odometry` | 仿真环境里程计(可选) |

### 3.2 发布的话题 (Outputs)

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| `/planning/pos_cmd` | `quadrotor_msgs/PositionCommand` | 100 | 位置控制命令 |
| `/planning/bspline` | `ego_traj_utils/Bspline` | ~10 | B样条轨迹参数 |
| `/planning/data_display` | `ego_traj_utils/DataDisp` | ~10 | 轨迹数据可视化 |
| `/grid_map/depth` | `sensor_msgs/Image` | 30 | 深度图像(仿真模式) |

### 3.3 发布/订阅的坐标系 (TF)

*   **需要输入的TF**: 无
*   **发布的TF**: 无

### 3.4 提供的服务 (Services)

| 服务名称 | 服务类型 | 描述 |
| :--- | :--- | :--- |
| 无 | - | - |

## 4. 核心算法原理

### 有限状态机(FSM)架构

该系统基于6状态FSM进行规划管理：

#### 状态定义
1. **INIT**: 初始状态，等待里程计数据
2. **WAIT_TARGET**: 等待目标点输入
3. **GEN_NEW_TRAJ**: 生成新轨迹
4. **REPLAN_TRAJ**: 轨迹重规划
5. **EXEC_TRAJ**: 执行轨迹
6. **EMERGENCY_STOP**: 紧急停止

#### 状态转换逻辑

```
INIT → WAIT_TARGET (获得里程计)
WAIT_TARGET → GEN_NEW_TRAJ (接收目标点)
GEN_NEW_TRAJ → EXEC_TRAJ (规划成功) / WAIT_TARGET (规划失败)
EXEC_TRAJ → REPLAN_TRAJ (需要重规划) / WAIT_TARGET (到达目标)
REPLAN_TRAJ → EXEC_TRAJ (重规划成功) / GEN_NEW_TRAJ (重规划失败)
任意状态 → EMERGENCY_STOP (紧急停止触发)
```

### 分层规划架构

1. **前端路径搜索 (A*)**
   - 使用栅格地图进行快速路径搜索
   - 提供初始可行路径
   - 考虑障碍物膨胀距离

2. **后端轨迹优化 (B-spline)**
   - 将A*路径作为初始猜测
   - 多目标优化：
     $$J_{total} = \lambda_1 J_{smooth} + \lambda_2 J_{collision} + \lambda_3 J_{feasibility}$$
   - 生成满足动力学约束的平滑轨迹

3. **实时重规划机制**
   - 检测周期: 10ms
   - 触发条件：
     - 轨迹与障碍物碰撞
     - 偏离轨迹过远
     - 接近轨迹终点
   - 使用时间优化保证实时性

### 轨迹执行控制

```cpp
// 位置命令生成
cmd.position = traj.evaluateDeBoorT(t);
cmd.velocity = traj.getVelocity(t);
cmd.acceleration = traj.getAcceleration(t);

// 偏航角控制
cmd.yaw = calculateYaw(cmd.velocity);
cmd.yaw_dot = calculateYawRate(cmd.velocity, cmd.acceleration);
```

## 5. 使用与配置

### 5.1 启动示例

```bash
# 基础启动
roslaunch ego_planner simple_run.launch

# 高级配置启动
roslaunch ego_planner advanced_param_exp1.launch
```

### 5.2 关键参数配置

```yaml
fsm:
  flight_type: 1              # 1=手动模式, 2=预设轨迹
  thresh_replan: 1.0          # 重规划触发距离(米)
  thresh_no_replan: 2.0       # 停止重规划距离(米)

manager:
  max_vel: 2.0                # 最大速度(m/s)
  max_acc: 3.0                # 最大加速度(m/s²)
  control_points_distance: 0.4 # B样条控制点距离

optimization:
  lambda_smooth: 1.0          # 平滑性权重
  lambda_collision: 0.5       # 碰撞避免权重
  lambda_feasibility: 0.1     # 可行性权重
  dist0: 0.5                  # 安全距离(米)
```

### 5.3 典型应用场景

- **室内导航**: 复杂室内环境的自主导航
- **障碍物避让**: 动态障碍物环境的实时避障
- **多目标巡航**: 连续目标点的高效访问
- **仿真验证**: 算法开发的快速验证平台