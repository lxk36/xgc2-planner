# 功能包: ego_bspline_opt

## 1. 功能概述

*   **一句话总结**: B样条轨迹优化库，提供基于梯度下降和LBFGS的轨迹优化算法，用于无人机路径规划中的碰撞避免和动力学约束满足
*   **核心节点**: 无独立节点，仅提供库函数
*   **算法类型**: B样条轨迹优化、梯度下降优化、LBFGS优化

## 2. 依赖关系

*   **主要依赖项**:
    *   `roscpp`: 核心ROS C++接口
    *   `rospy`: ROS Python接口
    *   `std_msgs`: 标准消息类型
    *   `visualization_msgs`: 可视化消息
    *   `cv_bridge`: 图像处理桥接
    *   `ego_plan_env`: 环境和地图接口
    *   `ego_path_searching`: 路径搜索算法（A*）
    *   `Eigen3`: 线性代数运算
    *   `PCL 1.7`: 点云处理库

## 3. 接口说明 (API)

### 3.1 订阅的话题 (Inputs)

| 话题名称 | 消息类型 | 描述 |
| :--- | :--- | :--- |
| 无 | - | 作为库函数被其他节点调用 |

### 3.2 发布的话题 (Outputs)

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| 无 | - | - | 作为库函数被其他节点调用 |

### 3.3 发布/订阅的坐标系 (TF)

*   **需要输入的TF**: 无
*   **发布的TF**: 无

### 3.4 C++编程接口

#### 核心类

**1. UniformBspline类**
```cpp
class UniformBspline {
public:
    // 构造函数
    UniformBspline(const Eigen::MatrixXd& points, const int& order,
                   const double& interval);

    // 德布尔算法求值
    Eigen::VectorXd evaluateDeBoorT(double t);

    // 获取导数
    Eigen::VectorXd getVelocity(double t);
    Eigen::VectorXd getAcceleration(double t);

    // 可行性检查
    bool checkFeasibility(double limit_vel, double limit_acc);

    // 时间调整
    void reallocateTime(bool show = false);
};
```

**2. BsplineOptimizer类**
```cpp
class BsplineOptimizer {
public:
    // 设置环境
    void setEnvironment(const GridMap::Ptr& map);

    // 设置参数
    void setParam(ros::NodeHandle& nh);

    // 初始化控制点
    std::vector<std::vector<Eigen::Vector3d>> initControlPoints(
        Eigen::MatrixXd& init_points, bool flag_first_init = true);

    // 反弹优化
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd& optimal_points,
                                    double ts);

    // 精细化优化
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd& init_points,
                                   const double& ts,
                                   Eigen::MatrixXd& optimal_points);
};
```

## 4. 核心算法原理

### B样条轨迹表示

均匀B样条曲线由控制点和节点向量定义：
$$\mathbf{p}(t) = \sum_{i=0}^{n} N_{i,p}(t) \mathbf{q}_i$$

其中：
- $\mathbf{q}_i$: 控制点
- $N_{i,p}(t)$: p阶B样条基函数
- $n$: 控制点数量

### 多目标优化框架

总代价函数：
$$J_{total} = \lambda_1 J_{smooth} + \lambda_2 J_{collision} + \lambda_3 J_{feasibility} + \lambda_4 J_{fitness}$$

#### 1. 平滑性代价 (Smoothness Cost)

**加速度范数**:
$$J_{acc} = \sum_{i=1}^{n-2} ||\mathbf{q}_{i+2} - 2\mathbf{q}_{i+1} + \mathbf{q}_i||^2$$

**加加速度范数**:
$$J_{jerk} = \sum_{i=1}^{n-3} ||\mathbf{q}_{i+3} - 3\mathbf{q}_{i+2} + 3\mathbf{q}_{i+1} - \mathbf{q}_i||^2$$

#### 2. 碰撞代价 (Collision Cost)

分段惩罚函数：
```cpp
if (dist < safe_distance) {
    // 三次惩罚
    cost = dist_err³;
} else if (dist < clearance) {
    // 二次惩罚
    cost = a·dist_err² + b·dist_err + c;
}
```

#### 3. 可行性代价 (Feasibility Cost)

动力学约束：
- 速度约束: $|\mathbf{v}_i| \leq v_{max}$
- 加速度约束: $|\mathbf{a}_i| \leq a_{max}$

### 两阶段优化策略

#### 第一阶段：反弹优化 (Rebound Optimization)

1. **碰撞段分割**: 检测轨迹中的碰撞段
2. **A*路径生成**: 为每个碰撞段生成避障路径
3. **梯度下降优化**:
   ```cpp
   // Armijo线搜索条件
   while (cost_new > cost_old + c1 * alpha * g.dot(d)) {
       alpha *= rho;  // 缩减步长
   }
   ```
4. **控制点反弹**: 沿梯度方向调整碰撞控制点

#### 第二阶段：精细化优化 (Refine Optimization)

使用LBFGS算法进行全局优化：
```cpp
// LBFGS更新公式
d_k = -H_k * g_k
x_{k+1} = x_k + alpha_k * d_k
```

其中$H_k$是Hessian矩阵的近似。

## 5. 使用与配置

### 5.1 基本使用示例

```cpp
#include <ego_bspline_opt/bspline_optimizer.h>
#include <ego_bspline_opt/uniform_bspline.h>

// 1. 创建优化器
BsplineOptimizer::Ptr optimizer = std::make_shared<BsplineOptimizer>();

// 2. 设置环境和参数
optimizer->setEnvironment(grid_map);
optimizer->setParam(nh);

// 3. 初始化控制点
Eigen::MatrixXd init_points; // 3 x N 矩阵
auto a_star_paths = optimizer->initControlPoints(init_points);

// 4. 执行两阶段优化
Eigen::MatrixXd optimal_points;
double time_interval = 0.1;

// 第一阶段：反弹优化
bool rebound_success = optimizer->BsplineOptimizeTrajRebound(
    optimal_points, time_interval);

// 第二阶段：精细化优化
if (rebound_success) {
    Eigen::MatrixXd refined_points;
    bool refine_success = optimizer->BsplineOptimizeTrajRefine(
        optimal_points, time_interval, refined_points);

    if (refine_success) {
        // 5. 创建最终B样条轨迹
        UniformBspline trajectory(refined_points, 3, time_interval);
    }
}
```

### 5.2 参数配置

```yaml
optimization:
  # 权重参数
  lambda_smooth: 1.0      # 平滑性权重
  lambda_collision: 1.0   # 碰撞避免权重
  lambda_feasibility: 1.0 # 可行性权重
  lambda_fitness: 1.0     # 拟合权重

  # 安全距离
  dist0: 0.5              # 安全距离 (米)
  clearance: 0.5          # 清除距离

  # 动力学约束
  max_vel: 2.0            # 最大速度 (m/s)
  max_acc: 2.0            # 最大加速度 (m/s²)

  # B样条参数
  order: 3                # B样条阶数

  # 优化参数
  max_iteration_num: 10   # 最大迭代次数
  max_iteration_time: 0.01 # 最大迭代时间(秒)
```

### 5.3 典型应用场景

- **轨迹平滑**: 将离散路径点平滑为连续可微轨迹
- **碰撞避免**: 实时调整轨迹避开障碍物
- **动力学约束满足**: 确保轨迹满足无人机动力学限制
- **实时重规划**: 快速响应环境变化的轨迹调整