# 功能包: decomp_util

## 1. 功能概述

*   **一句话总结**: 基于椭球膨胀的快速凸分解头文件库，用于从点云中生成安全飞行走廊
*   **核心节点**: 无ROS节点（纯头文件库）
*   **算法类型**: 基于椭球膨胀的区域分解算法，用于生成安全飞行走廊(Safe Flight Corridor)

## 2. 依赖关系

*   **主要依赖项**:
    *   `Eigen3`: 线性代数运算库，用于矩阵和向量计算
    *   `catkin`: ROS构建系统
    *   `Boost.Geometry` (测试用): 用于测试文件中的几何可视化
    *   `C++11`: 语言标准要求

## 3. 接口说明 (API)

### 3.1 主要类模板

此功能包为头文件库，不包含ROS话题接口，主要提供以下类模板：

| 类名 | 维度 | 描述 |
| :--- | :--- | :--- |
| `EllipsoidDecomp<Dim>` | 2D/3D | 主要分解类，基于椭球膨胀生成安全走廊 |
| `SeedDecomp<Dim>` | 2D/3D | 种子点分解，单点周围的凸分解 |
| `LineSegment<Dim>` | 2D/3D | 线段分解，对单条路径段进行分解 |
| `IterativeDecomp<Dim>` | 2D/3D | 迭代分解，逐步优化分解结果 |

### 3.2 核心数据结构

| 结构体 | 描述 |
| :--- | :--- |
| `Ellipsoid<Dim>` | 椭球结构，包含中心点和协方差矩阵 |
| `Polyhedron<Dim>` | 多面体结构，由超平面集合定义 |
| `Hyperplane<Dim>` | 超平面结构，包含点和法向量 |
| `LinearConstraint<Dim>` | 线性约束，表示为Ax ≤ b形式 |

### 3.3 主要API方法

#### EllipsoidDecomp类
```cpp
// 设置障碍物点云
void set_obs(const vec_Vecf<Dim> &obs);

// 设置局部边界框
void set_local_bbox(const Vecf<Dim> &bbox);

// 执行路径膨胀分解
void dilate(const vec_Vecf<Dim> &path, double offset_x = 0);

// 获取分解结果
vec_E<Polyhedron<Dim>> get_polyhedrons() const;
vec_E<Ellipsoid<Dim>> get_ellipsoids() const;
vec_E<LinearConstraint<Dim>> get_constraints() const;
```

## 4. 核心算法原理

该算法实现了基于椭球膨胀的凸分解，主要用于四旋翼等无人机的动态可行轨迹规划。算法的核心思想是将给定路径周围的自由空间分解为一系列凸多面体，形成安全飞行走廊。

### 算法流程

#### 步骤1: 路径段椭球初始化
对于给定路径上的每个线段，算法首先构造一个椭球：
- 椭球的长轴沿着线段方向
- 椭球的短轴垂直于线段方向
- 椭球尺寸由局部边界框参数确定

#### 步骤2: 障碍物筛选
使用局部边界框筛选相关障碍物点：
$$
O_{local} = \{o \in O | o \text{ inside local bbox}\}
$$
其中 $O$ 是全局障碍物集合，$O_{local}$ 是局部相关障碍物。

#### 步骤3: 椭球膨胀与超平面生成
对于每个椭球，算法迭代地找到最近的障碍物点，并生成分离超平面：

1. **找到最近障碍物点**:
   $$
   p_{closest} = \arg\min_{p \in O_{remain}} \|(C^{-1}(p - d))\|
   $$
   其中 $C$ 是椭球的形状矩阵，$d$ 是椭球中心。

2. **生成分离超平面**:
   超平面的法向量计算为：
   $$
   n = \frac{C^{-1}C^{-T}(p_{closest} - d)}{\|C^{-1}C^{-T}(p_{closest} - d)\|}
   $$

3. **更新障碍物集合**:
   移除被当前超平面分离的障碍物点。

#### 步骤4: 凸多面体构造
通过收集所有分离超平面，构造凸多面体：
$$
\mathcal{P} = \{x | n_i^T(x - p_i) \leq 0, \forall i\}
$$

### 算法特点

1. **椭球到多面体的精确转换**: 算法保证生成的多面体完全包含初始椭球
2. **增量式构造**: 逐步添加约束，避免冗余计算
3. **维度通用**: 支持2D和3D空间分解
4. **高效性**: 时间复杂度与障碍物数量呈线性关系

### 数学表示

最终的安全飞行走廊表示为线性约束集合：
$$
\text{SFC} = \bigcup_{i=1}^{n} \{x | A_i x \leq b_i\}
$$
其中每个 $\{x | A_i x \leq b_i\}$ 对应路径上一个线段的凸分解区域。

## 5. 使用与配置

### 5.1 基本使用示例

#### 2D分解示例
```cpp
#include <decomp_util/ellipsoid_decomp.h>

// 创建2D分解器
Vec2f origin(-2, -2);
Vec2f range(4, 4);
EllipsoidDecomp2D decomp(origin, range);

// 设置障碍物点
vec_Vec2f obstacles;
// ... 填充障碍物数据 ...
decomp.set_obs(obstacles);

// 设置局部边界框
decomp.set_local_bbox(Vec2f(2.0, 2.0));

// 定义路径
vec_Vec2f path;
path.push_back(Vec2f(1.0, 1.0));
path.push_back(Vec2f(0.0, 0.0));
path.push_back(Vec2f(-1.0, 1.0));

// 执行分解
decomp.dilate(path, 0.1);  // 0.1是膨胀半径

// 获取结果
auto polyhedrons = decomp.get_polyhedrons();
auto ellipsoids = decomp.get_ellipsoids();
auto constraints = decomp.get_constraints();
```

#### 3D分解示例
```cpp
#include <decomp_util/ellipsoid_decomp.h>

// 创建3D分解器
Vec3f origin(-5, -5, 0);
Vec3f range(10, 10, 5);
EllipsoidDecomp3D decomp(origin, range);

// 设置参数
decomp.set_obs(obstacles_3d);
decomp.set_local_bbox(Vec3f(3.0, 3.0, 2.0));
decomp.set_inflate_distance(0.2);

// 执行3D路径分解
vec_Vec3f path_3d;
// ... 定义3D路径 ...
decomp.dilate(path_3d);
```

### 5.2 CMake集成

在其他项目中使用此库：

```cmake
find_package(decomp_util REQUIRED)
include_directories(${DECOMP_UTIL_INCLUDE_DIRS})

add_executable(my_planner src/my_planner.cpp)
target_link_libraries(my_planner ${catkin_LIBRARIES})
```

### 5.3 关键参数配置

| 参数 | 类型 | 描述 | 默认值 |
| :--- | :--- | :--- | :--- |
| `local_bbox` | `Vecf<Dim>` | 局部边界框尺寸，控制考虑的障碍物范围 | 零向量 |
| `global_bbox` | `Vecf<Dim> × 2` | 全局边界框，限制整体分解区域 | 零向量 |
| `inflate_distance` | `double` | 额外膨胀距离，增加安全裕度 | 0.0 |
| `offset_x` | `double` | 长轴方向的额外偏移 | 0.0 |

### 5.4 性能优化建议

1. **障碍物预处理**: 使用适当的局部边界框减少不相关障碍物
2. **路径简化**: 移除冗余的路径点以减少计算量
3. **参数调优**: 根据应用场景调整膨胀参数平衡安全性和计算效率
4. **并行化**: 不同路径段的分解可以并行计算

## 6. 应用场景

### 6.1 主要应用领域
- **无人机路径规划**: 生成四旋翼安全飞行走廊
- **机器人导航**: 移动机器人的路径约束生成
- **轨迹优化**: 为轨迹优化器提供凸约束
- **避障规划**: 复杂环境中的实时避障

### 6.2 集成示例
该库通常与以下模块配合使用：
- **MADER**: 多智能体动态环境重规划
- **轨迹优化器**: 如MINCO、B-spline优化器
- **运动规划框架**: 如OMPL、MoveIt

### 6.3 相关论文
该算法基于以下研究成果：
> S. Liu, M. Watterson, K. Mohta, K. Sun, S. Bhattacharya, C.J. Taylor and V. Kumar. "Planning Dynamically Feasible Trajectories for Quadrotors using Safe Flight Corridors in 3-D Complex Environments." ICRA 2017.

## 7. 扩展功能

### 7.1 多面体收缩
```cpp
// 收缩多面体以增加安全裕度
decomp.shrink_polyhedrons(0.1);  // 收缩0.1米
```

### 7.2 约束导出
```cpp
// 导出为标准线性规划格式
auto constraints = decomp.get_constraints();
for(const auto& constraint : constraints) {
    MatDNf<Dim> A = constraint.A();
    VecDf b = constraint.b();
    // 使用A和b进行轨迹优化
}
```

### 7.3 可视化支持
该库提供了Doxygen文档和测试示例，支持：
- SVG格式的2D可视化
- RViz兼容的3D可视化（通过DecompROS包装器）
- Matplotlib Python绑定（可选）