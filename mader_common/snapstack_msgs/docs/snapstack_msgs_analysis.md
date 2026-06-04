# 功能包: snapstack_msgs

## 1. 功能概述

*   **一句话总结**: SNAP Stack系统中用于四旋翼无人机控制、状态估计和通信的标准消息定义库，支持轨迹跟踪、姿态控制、状态感知和多机协作等功能。
*   **核心节点**: 无（纯消息定义包）
*   **算法类型**: 消息接口定义包，为SNAP Stack架构中的轨迹规划、姿态控制、视觉惯性里程计(VIO)和多智能体协作提供标准化通信协议

## 2. 依赖关系

*   **主要依赖项**:
    *   `message_generation`: ROS消息生成工具
    *   `message_runtime`: ROS消息运行时库
    *   `std_msgs`: 标准消息类型库
    *   `geometry_msgs`: 几何相关消息类型（位置、姿态、速度等）
    *   `nav_msgs`: 导航相关消息类型
    *   `std_srvs`: 标准服务类型库
    *   `actionlib_msgs`: 动作库消息类型

## 3. 接口说明 (API)

### 3.1 消息类型定义

#### 3.1.1 控制相关消息

| 消息类型 | 用途 | 主要字段 | 应用场景 |
| :--- | :--- | :--- | :--- |
| `AttitudeCommand` | 姿态控制命令 | `q`(期望姿态), `w`(期望角速度), `F_W`(期望力), `power`(电机使能) | 姿态控制器输入，底层控制 |
| `Goal` | 轨迹跟踪目标 | `p,v,a,j`(位置/速度/加速度/急动度), `psi,dpsi`(偏航角), `mode_xy,mode_z`(控制模式) | 轨迹规划器输出，外环控制器输入 |
| `Motors` | 电机控制信号 | `m1~m8`(8个电机控制量) | 底层电机控制，支持八旋翼配置 |

#### 3.1.2 状态感知消息

| 消息类型 | 用途 | 主要字段 | 应用场景 |
| :--- | :--- | :--- | :--- |
| `State` | 无人机状态信息 | `pos`(位置), `vel`(速度), `quat`(姿态), `w`(角速度), `abias,gbias`(传感器偏差) | 状态估计器输出，控制器输入 |
| `VioFilterState` | VIO滤波器状态 | `pose,twist`(位姿和速度), `bw,ba`(角速度/加速度偏差), `extrinsics`(外参), `error_cov`(误差协方差) | 视觉惯性里程计状态估计 |
| `IMU` | IMU传感器数据 | `gyro`(陀螺仪), `accel`(加速度计), `loop_time`(采样周期) | 惯性传感器数据处理 |

#### 3.1.3 系统管理消息

| 消息类型 | 用途 | 主要字段 | 应用场景 |
| :--- | :--- | :--- | :--- |
| `QuadFlightMode` | 飞行模式管理 | `mode`(飞行状态): NOT_FLYING(0), TAKEOFF(1), LAND(2), INIT(3), GO(4), ESTOP(5), KILL(6) | 飞行状态机管理，安全控制 |
| `CommAge` | 通信延迟监控 | `vicon_age_secs`(Vicon数据延迟), `goal_age_secs`(目标数据延迟) | 多机协作中的通信质量评估 |

#### 3.1.4 调试分析消息

| 消息类型 | 用途 | 主要字段 | 应用场景 |
| :--- | :--- | :--- | :--- |
| `ControlLog` | 控制系统调试日志 | 位置/速度/加速度/急动度的参考值、实际值、误差值；姿态和角速度信号 | 控制器性能分析，参数调优 |
| `SMCData` | 滑模控制器数据 | `q_des,q_act,q_err`(期望/实际/误差姿态), `w_des,w_act,w_err`(期望/实际/误差角速度), `s,integrator`(滑模面和积分器) | 滑模控制器调试和分析 |

### 3.2 控制模式定义

`Goal`消息中定义了三种轨迹跟踪控制模式：
- **MODE_POSITION_CONTROL (0)**: 位置控制模式，使用位置和速度误差计算控制量
- **MODE_VELOCITY_CONTROL (1)**: 速度控制模式，仅使用速度误差
- **MODE_ACCELERATION_CONTROL (2)**: 加速度控制模式，不使用跟踪误差，适用于自定义控制信号

### 3.3 飞行状态定义

`QuadFlightMode`消息中定义了完整的飞行状态机：
- **NOT_FLYING (0)**: 未飞行状态
- **TAKEOFF (1)**: 起飞状态
- **LAND (2)**: 降落状态
- **INIT (3)**: 初始化状态
- **GO (4)**: 正常飞行状态
- **ESTOP (5)**: 紧急停止状态
- **KILL (6)**: 强制停机状态

## 4. 核心设计原理

### 4.1 SNAP Stack架构消息设计

snapstack_msgs是MIT ACL实验室开发的SNAP Stack四旋翼控制架构的核心消息库。该架构采用分层控制设计：

#### 外环控制器 (Outer Loop Controller)
- 接收`Goal`消息进行轨迹跟踪
- 输出`AttitudeCommand`给内环控制器
- 生成`ControlLog`用于性能分析

#### 内环控制器 (Inner Loop Controller)
- 接收`AttitudeCommand`进行姿态控制
- 输出`Motors`控制信号给执行器
- 可使用滑模控制，产生`SMCData`调试信息

#### 状态估计系统
- VIO系统输出`VioFilterState`消息
- 传感器数据通过`IMU`消息传递
- 融合后的状态通过`State`消息发布

### 4.2 多机协作支持

#### 通信质量监控
`CommAge`消息用于监控多机系统中的通信延迟：
- `vicon_age_secs`: 外部定位系统（如Vicon）数据的延迟时间
- `goal_age_secs`: 轨迹目标数据的延迟时间

这对于多机协作场景至关重要，因为：
1. **时间同步**: 确保各机器人使用相同时间基准的轨迹信息
2. **通信质量**: 评估网络通信的可靠性和实时性
3. **安全保障**: 当通信延迟过大时触发安全模式

#### 分布式控制支持
- 标准化的`Goal`消息格式支持集中式和分布式轨迹规划
- `QuadFlightMode`状态机支持多机协调的状态同步
- `State`消息提供其他机器人状态信息的标准格式

### 4.3 轨迹跟踪的几何约束处理

`Goal`消息中的`psi`角定义遵循文献[1]中的几何约束理论：

$$\psi$$角不等同于传统的航向角(yaw)，而是考虑了四旋翼动力学约束的几何角度。这种设计的优势：

1. **动力学一致性**: $$q_{\psi}$$已包含部分航向信息，避免奇异性
2. **轨迹平滑性**: 在非悬停状态下保持轨迹的几何连续性
3. **控制精度**: 更好地处理高速机动和复杂轨迹

参考文献：
[1] https://arxiv.org/pdf/2103.06372.pdf
[2] https://link.springer.com/chapter/10.1007/978-3-030-28619-4_20

## 5. 使用与配置

### 5.1 依赖安装

```bash
# 安装依赖包
sudo apt-get install ros-$ROS_DISTRO-geometry-msgs
sudo apt-get install ros-$ROS_DISTRO-nav-msgs
sudo apt-get install ros-$ROS_DISTRO-std-msgs
```

### 5.2 编译配置

```bash
# 在catkin工作空间中编译
cd catkin_ws
catkin_make

# 或使用catkin_tools
catkin build snapstack_msgs
```

### 5.3 消息使用示例

#### C++中使用消息

```cpp
#include <snapstack_msgs/Goal.h>
#include <snapstack_msgs/State.h>
#include <snapstack_msgs/QuadFlightMode.h>

// 发布轨迹目标
ros::Publisher goal_pub = nh.advertise<snapstack_msgs::Goal>("goal", 1);

snapstack_msgs::Goal goal_msg;
goal_msg.header.stamp = ros::Time::now();
goal_msg.p.x = 1.0; goal_msg.p.y = 2.0; goal_msg.p.z = 3.0;
goal_msg.v.x = 0.5; goal_msg.v.y = 0.0; goal_msg.v.z = 0.0;
goal_msg.mode_xy = snapstack_msgs::Goal::MODE_POSITION_CONTROL;
goal_msg.mode_z = snapstack_msgs::Goal::MODE_POSITION_CONTROL;
goal_msg.power = true;

goal_pub.publish(goal_msg);

// 订阅状态信息
void stateCallback(const snapstack_msgs::State::ConstPtr& msg) {
    ROS_INFO("Position: [%.2f, %.2f, %.2f]",
             msg->pos.x, msg->pos.y, msg->pos.z);
}

ros::Subscriber state_sub = nh.subscribe("state", 1, stateCallback);
```

#### Python中使用消息

```python
import rospy
from snapstack_msgs.msg import Goal, QuadFlightMode

# 发布飞行模式
mode_pub = rospy.Publisher('flight_mode', QuadFlightMode, queue_size=1)

mode_msg = QuadFlightMode()
mode_msg.header.stamp = rospy.Time.now()
mode_msg.mode = QuadFlightMode.TAKEOFF

mode_pub.publish(mode_msg)
```

### 5.4 典型应用场景

#### 5.4.1 单机轨迹跟踪
```
轨迹规划器 → Goal → 外环控制器 → AttitudeCommand → 内环控制器 → Motors → 执行器
                 ↑
              State ← 状态估计器 ← VioFilterState ← VIO系统
```

#### 5.4.2 多机协作系统
```
集中规划器 → Goal (broadcast) → 各机器人外环控制器
     ↑                               ↓
CommAge监控 ← 通信网络 ← State (机器人状态广播)
```

## 6. 与MADER系统的集成

### 6.1 在MADER中的角色

snapstack_msgs作为MADER(Multi-Agent Dynamic Environment Replanning)系统的重要组成部分，提供了：

1. **标准化通信接口**: 为MADER的多智能体系统提供一致的消息格式
2. **实时状态共享**: 通过`State`和`CommAge`消息支持多机器人状态感知
3. **分布式控制**: `Goal`消息支持MADER的分布式轨迹规划输出
4. **安全管理**: `QuadFlightMode`提供多机协作中的安全状态管理

### 6.2 MADER系统架构中的消息流

```
MADER规划器 → Goal → SNAP控制器 → AttitudeCommand → 底层控制
      ↑                ↓
   CommAge ← 其他智能体 State 广播
      ↑                ↓
   碰撞检测 ← 环境感知 ← VioFilterState
```

### 6.3 多智能体协作特性

- **实时重规划**: Goal消息支持MADER的动态轨迹更新
- **状态同步**: State消息实现多机器人状态实时共享
- **通信监控**: CommAge消息确保协作系统的通信质量
- **安全协调**: QuadFlightMode支持群体安全状态管理

## 7. 总结

snapstack_msgs是一个设计精良的ROS消息定义包，为四旋翼无人机控制系统提供了完整的通信协议。其特点包括：

1. **分层设计**: 支持外环/内环分层控制架构
2. **多模式支持**: 提供多种控制模式适应不同应用场景
3. **调试友好**: 丰富的调试消息便于系统分析和优化
4. **多机协作**: 原生支持多智能体系统的通信需求
5. **几何约束**: 考虑四旋翼动力学特性的轨迹参数设计

该消息库不仅适用于单机控制，更在MADER等多智能体系统中发挥重要作用，为复杂的动态环境下多机协作提供了可靠的通信基础。