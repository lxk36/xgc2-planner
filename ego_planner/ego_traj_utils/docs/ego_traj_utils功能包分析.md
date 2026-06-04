# 功能包: ego_traj_utils

## 1. 功能概述

*   **一句话总结**: 多项式轨迹表示与规划可视化工具库
*   **核心节点**: 无独立节点，提供轨迹工具类库
*   **算法类型**: 多项式轨迹生成 + minimum snap优化 + RViz可视化

## 2. 依赖关系

*   **主要依赖项**:
    *   `roscpp`: ROS C++基础通信库
    *   `std_msgs`: 标准消息类型
    *   `visualization_msgs`: RViz标记可视化
    *   `Eigen3`: 矩阵运算与轨迹计算
    *   `ego_bspline_opt`: B样条优化功能
    *   `ego_path_searching`: A*路径搜索
    *   `cv_bridge`: 图像处理桥接（可选）
    *   `message_generation`: 自定义消息生成

## 3. 接口说明 (API)

### 3.1 自定义消息类型

**Bspline.msg**:
```
int32 order                 # B样条阶数
int64 traj_id               # 轨迹ID
float64[] knots             # 节点向量
float64[] pos_pts           # 位置控制点
float64[] yaw_pts           # 偏航控制点
float64 yaw_dt              # 偏航时间间隔
float64 start_time          # 开始时间
```

**DataDisp.msg**:
```
int8 type_mask              # 显示类型掩码
Header header               # 时间戳和坐标系
float64[] data              # 显示数据数组
```

### 3.2 订阅的话题 (Inputs)

| 话题名称 | 消息类型 | 描述 |
| :--- | :--- | :--- |
| 无 | - | 作为工具库，不直接订阅话题 |

### 3.3 发布的话题 (Outputs)

通过`PlanningVisualization`类发布：

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| `goal_point` | `visualization_msgs/Marker` | 按需 | 目标点可视化 |
| `global_list` | `visualization_msgs/Marker` | 按需 | 全局路径可视化 |
| `init_list` | `visualization_msgs/Marker` | 按需 | 初始路径可视化 |
| `optimal_list` | `visualization_msgs/Marker` | 按需 | 优化轨迹可视化 |
| `a_star_list` | `visualization_msgs/Marker` | 按需 | A*路径可视化 |
| `guide_vector` | `visualization_msgs/MarkerArray` | 按需 | 引导向量可视化 |
| `intermediate_state` | `visualization_msgs/MarkerArray` | 按需 | 中间状态可视化 |

### 3.4 C++编程接口

#### 核心类

**1. PolynomialTraj类**
```cpp
class PolynomialTraj {
public:
    // 添加轨迹段
    void addSegment(const vector<double>& cx,
                    const vector<double>& cy,
                    const vector<double>& cz,
                    double t);

    // 轨迹评估
    Eigen::Vector3d evaluate(double t);        // 位置
    Eigen::Vector3d evaluateVel(double t);     // 速度
    Eigen::Vector3d evaluateAcc(double t);     // 加速度

    // 轨迹分析
    double getLength();                        // 轨迹长度
    double getMeanVel();                        // 平均速度
    double getMaxVel();                         // 最大速度
    double getAccCost();                        // 加速度代价
    double getJerk();                           // 急动度

    // 时间管理
    double getTotalDuration();                 // 总时长
    int locatePieceIdx(double t);              // 定位段索引
};
```

**2. PlanningVisualization类**
```cpp
class PlanningVisualization {
public:
    // 可视化接口
    void displayMarkerList(ros::Publisher& pub,
                           const vector<Eigen::Vector3d>& list,
                           double scale,
                           Eigen::Vector4d color,
                           int id);

    void displayGoalPoint(Eigen::Vector3d goal_point, int id);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, int id);
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, int id);
    void displayOptimalList(vector<Eigen::Vector3d> optimal_pts, int id);

    // 高级可视化
    void displayIntermediateState(
        vector<vector<Eigen::Vector3d>>& state_list, int id);
    void displayArrowList(ros::Publisher& pub,
                         const vector<Eigen::Vector3d>& list,
                         const vector<Eigen::Vector3d>& start,
                         int id);
};
```

## 4. 核心算法原理

### 多项式轨迹表示

#### 分段多项式模型

每段轨迹表示为n阶多项式：
$$p_i(t) = \sum_{j=0}^{n} c_{ij} t^j$$

其中：
- $p_i(t)$: 第i段轨迹
- $c_{ij}$: 第i段第j阶系数
- $t \in [0, T_i]$: 段内时间参数

#### 高阶导数计算

速度：
$$\dot{p}_i(t) = \sum_{j=1}^{n} j \cdot c_{ij} t^{j-1}$$

加速度：
$$\ddot{p}_i(t) = \sum_{j=2}^{n} j(j-1) \cdot c_{ij} t^{j-2}$$

### Minimum Snap轨迹生成

#### 优化目标

最小化snap（四阶导数）：
$$J = \int_0^T ||p^{(4)}(t)||^2 dt$$

#### 约束条件

1. **航路点约束**:
   - $p(t_i) = w_i$ (经过航路点)

2. **连续性约束**:
   - $p_i^{(k)}(T_i) = p_{i+1}^{(k)}(0)$ for $k = 0,1,2,3$

3. **边界条件**:
   - $p^{(k)}(0) = p_{start}^{(k)}$ (起始状态)
   - $p^{(k)}(T) = p_{end}^{(k)}$ (终止状态)

### 轨迹质量评估

#### 长度计算
```cpp
double length = 0.0;
for (int i = 0; i < N; i++) {
    double dt = 0.01;  // 采样间隔
    for (double t = 0; t < duration[i]; t += dt) {
        Vector3d vel = evaluateVel(t);
        length += vel.norm() * dt;
    }
}
```

#### 加速度代价
$$J_{acc} = \int_0^T ||\ddot{p}(t)||^2 dt$$

#### 急动度计算
$$J_{jerk} = \int_0^T ||\dddot{p}(t)||^2 dt$$

### 可视化系统架构

#### 标记生成策略

1. **球体链表示**: 用球体序列表示离散路径点
2. **线条连接**: 连续轨迹的线条表示
3. **箭头向量**: 速度/加速度方向可视化
4. **颜色编码**: 不同阶段用不同颜色区分

#### 批量发布优化
```cpp
// 使用MarkerArray批量发布
visualization_msgs::MarkerArray marker_array;
for (auto& point : trajectory_points) {
    marker_array.markers.push_back(createSphereMarker(point));
}
pub.publish(marker_array);
```

## 5. 使用与配置

### 5.1 基本使用示例

#### 创建多项式轨迹
```cpp
#include <ego_traj_utils/polynomial_traj.h>

// 创建轨迹对象
PolynomialTraj poly_traj;

// 定义5阶多项式系数 (从高阶到低阶)
vector<double> cx = {0.1, 0.2, 0.5, 1.0, 2.0, 0.0}; // x轴系数
vector<double> cy = {0.0, 0.1, 0.3, 0.5, 1.0, 0.0}; // y轴系数
vector<double> cz = {0.0, 0.0, 0.1, 0.2, 0.5, 1.0}; // z轴系数

// 添加轨迹段
poly_traj.addSegment(cx, cy, cz, 2.0);  // 时长2秒

// 初始化
poly_traj.init();

// 评估轨迹
double t = 1.0;
Eigen::Vector3d pos = poly_traj.evaluate(t);
Eigen::Vector3d vel = poly_traj.evaluateVel(t);
Eigen::Vector3d acc = poly_traj.evaluateAcc(t);

// 获取轨迹信息
double length = poly_traj.getLength();
double max_vel = poly_traj.getMaxVel();
```

#### 可视化轨迹
```cpp
#include <ego_traj_utils/planning_visualization.h>

// 创建可视化对象
PlanningVisualization::Ptr vis_ptr;
vis_ptr.reset(new PlanningVisualization(nh));

// 生成轨迹点
vector<Eigen::Vector3d> traj_pts;
for (double t = 0; t < total_time; t += 0.01) {
    traj_pts.push_back(poly_traj.evaluate(t));
}

// 显示优化轨迹
vis_ptr->displayOptimalList(traj_pts, 0);

// 显示目标点
Eigen::Vector3d goal(10, 10, 2);
vis_ptr->displayGoalPoint(goal, 0);
```

### 5.2 参数配置

```yaml
planning_visualization:
  # 可视化尺度
  goal_scale: 0.5           # 目标点大小
  path_scale: 0.1           # 路径点大小
  traj_scale: 0.15          # 轨迹点大小

  # 颜色配置 (RGBA)
  goal_color: [1.0, 0.0, 0.0, 1.0]     # 红色目标
  init_color: [0.0, 0.0, 1.0, 0.8]     # 蓝色初始路径
  optimal_color: [0.0, 1.0, 0.0, 0.8]  # 绿色优化轨迹

  # 发布频率
  vis_fps: 30               # 可视化帧率
```

### 5.3 典型应用场景

- **轨迹生成**: 从航路点生成平滑多项式轨迹
- **轨迹评估**: 计算轨迹长度、速度、加速度等指标
- **可视化调试**: 在RViz中显示规划过程和结果
- **数据记录**: 导出轨迹数据用于分析和回放