# 功能包: mader_msgs

## 1. 功能概述

*   **一句话总结**: MADER多智能体动态环境重规划框架的消息定义包，提供轨迹表示和通信协议
*   **核心功能**: 为MADER系统提供分段多项式轨迹、动态对象信息和规划器标识等核心消息类型
*   **消息类型**: 轨迹优化相关的自定义消息定义

## 2. 依赖关系

*   **主要依赖项**:
    *   `std_msgs`: 标准消息类型，提供Header等基础消息
    *   `geometry_msgs`: 几何消息类型，提供Vector3等空间位置表示
    *   `nav_msgs`: 导航消息类型，用于机器人导航相关信息
    *   `actionlib_msgs`: 动作库消息类型，用于异步任务处理
    *   `message_generation`: 编译时消息生成工具
    *   `message_runtime`: 运行时消息支持

## 3. 消息定义详细说明

### 3.1 CoeffPoly3.msg - 三次多项式系数

**用途**: 表示三次多项式的系数，用于轨迹的数学描述

**字段说明**:
```
float64 a    # 三次项系数 (at^3)
float64 b    # 二次项系数 (bt^2)
float64 c    # 一次项系数 (ct)
float64 d    # 常数项系数 (d)
```

**数学表示**: 多项式形式为 `at³ + bt² + ct + d`，用于描述轨迹在单个维度上随时间的变化

### 3.2 PieceWisePolTraj.msg - 分段多项式轨迹

**用途**: 表示完整的三维分段多项式轨迹，由多个时间段和对应的多项式组成

**字段说明**:
```
float64[] times        # 时间节点数组，包含n+1个元素 [t0,t1,t2,...,tn+1]
CoeffPoly3[] coeff_x   # X轴方向的多项式系数数组，包含n个元素
CoeffPoly3[] coeff_y   # Y轴方向的多项式系数数组，包含n个元素
CoeffPoly3[] coeff_z   # Z轴方向的多项式系数数组，包含n个元素
```

**结构说明**:
- `times`数组定义了n个时间段：[t0,t1], [t1,t2], ..., [tn,tn+1]
- 每个时间段对应三个维度的三次多项式系数
- 轨迹在时间段[ti,ti+1]内由第i组多项式描述

### 3.3 DynTraj.msg - 动态轨迹信息

**用途**: 描述动态环境中的智能体或障碍物的完整信息，包括当前状态和未来轨迹

**字段说明**:
```
Header header                    # 消息头，包含时间戳和坐标系信息
string[] function               # 障碍物的函数描述（仅用于动态障碍物，is_agent==false）
float32[] bbox                  # 包围盒尺寸 [hx, hy, hz]，表示hx×hy×hz的长方体
geometry_msgs/Vector3 pos       # 当前位置坐标 (x,y,z)
int32 id                        # 对象的唯一标识符
bool is_agent                   # 对象类型标志：true为智能体，false为静态/动态障碍物
PieceWisePolTraj pwp           # 分段多项式轨迹（仅用于智能体，is_agent==true）
```

**使用说明**:
- **智能体**(`is_agent==true`): 使用`pwp`字段描述规划的轨迹
- **障碍物**(`is_agent==false`): 使用`function`字段描述运动模式，`bbox`描述空间占用

### 3.4 WhoPlans.msg - 规划器标识

**用途**: 标识当前活跃的路径规划器类型，用于多规划器系统的协调

**字段说明**:
```
Header header      # 消息头
uint8 value        # 规划器类型值

# 预定义常量
uint8 OTHER   = 0  # 其他规划器
uint8 MADER   = 1  # MADER规划器
```

**应用场景**: 在多智能体系统中，不同智能体可能使用不同的规划算法，此消息用于标识和协调

## 4. 服务定义详细说明

该功能包未定义任何服务（srv目录不存在）。

## 5. 使用示例

### 5.1 创建分段多项式轨迹

```cpp
#include <mader_msgs/PieceWisePolTraj.h>
#include <mader_msgs/CoeffPoly3.h>

// 创建一个简单的二段轨迹
mader_msgs::PieceWisePolTraj traj;

// 设置时间节点（3个时间点，2个时间段）
traj.times = {0.0, 1.0, 2.0};

// 为X轴创建两段多项式
mader_msgs::CoeffPoly3 coeff_x1, coeff_x2;
coeff_x1.a = 0.0; coeff_x1.b = 0.0; coeff_x1.c = 1.0; coeff_x1.d = 0.0;  // x = t
coeff_x2.a = 0.0; coeff_x2.b = 0.0; coeff_x2.c = 0.5; coeff_x2.d = 0.5;  // x = 0.5t + 0.5

traj.coeff_x = {coeff_x1, coeff_x2};
// 类似地设置coeff_y和coeff_z...
```

### 5.2 发布动态对象信息

```cpp
#include <mader_msgs/DynTraj.h>

// 创建智能体信息
mader_msgs::DynTraj agent_info;
agent_info.header.stamp = ros::Time::now();
agent_info.header.frame_id = "world";
agent_info.id = 1;
agent_info.is_agent = true;
agent_info.pos.x = 1.0; agent_info.pos.y = 2.0; agent_info.pos.z = 1.5;
agent_info.bbox = {0.5, 0.5, 0.3};  // 智能体的包围盒
agent_info.pwp = /* 规划的轨迹 */;

// 发布消息
ros::Publisher pub = nh.advertise<mader_msgs::DynTraj>("dynamic_objects", 10);
pub.publish(agent_info);
```

### 5.3 标识规划器类型

```cpp
#include <mader_msgs/WhoPlans.h>

mader_msgs::WhoPlans planner_id;
planner_id.header.stamp = ros::Time::now();
planner_id.value = mader_msgs::WhoPlans::MADER;

ros::Publisher pub = nh.advertise<mader_msgs::WhoPlans>("planner_status", 1);
pub.publish(planner_id);
```

## 6. 技术特点

### 6.1 轨迹表示优势
- **连续性保证**: 分段多项式保证轨迹的连续性和平滑性
- **计算效率**: 三次多项式在计算复杂度和表达能力间取得平衡
- **灵活性**: 可表示复杂的三维运动轨迹

### 6.2 多智能体支持
- **统一接口**: `DynTraj`消息同时支持智能体和障碍物的描述
- **实时通信**: 消息结构支持实时的状态和轨迹信息交换
- **碰撞检测**: 包围盒信息支持高效的碰撞检测算法

### 6.3 系统集成
- **模块化设计**: 消息定义独立于具体算法实现
- **扩展性**: 易于添加新的消息类型和字段
- **兼容性**: 基于标准ROS消息类型，具备良好的互操作性

## 7. 注意事项

1. **时间同步**: 使用`header.stamp`确保多智能体间的时间同步
2. **坐标系一致性**: 所有位置信息应在同一坐标系下表示
3. **轨迹有效性**: 确保`times`数组长度比多项式系数数组长度多1
4. **内存管理**: 大型轨迹数据传输时注意网络带宽和内存使用