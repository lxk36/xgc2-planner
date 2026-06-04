# 功能包: ego_path_searching

## 1. 功能概述

*   **一句话总结**: 基于A*算法的三维路径搜索库，为EGO-Planner提供快速路径规划服务
*   **核心节点**: 无独立节点，作为库文件供其他模块调用
*   **算法类型**: A*路径搜索算法（支持3D空间，含对角线启发式函数）

## 2. 依赖关系

*   **主要依赖项**:
    *   `roscpp`: ROS C++客户端库，提供基础ROS功能
    *   `rospy`: ROS Python客户端库
    *   `std_msgs`: ROS标准消息类型
    *   `visualization_msgs`: ROS可视化消息类型
    *   `cv_bridge`: OpenCV与ROS图像消息转换库
    *   `ego_plan_env`: EGO-Planner环境模块，提供GridMap栅格地图接口
    *   `Eigen3`: 线性代数运算库，用于向量和矩阵计算
    *   `PCL 1.7`: 点云处理库（在CMakeLists.txt中声明但代码中未直接使用）

## 3. 接口说明 (API)

### 3.1 订阅的话题 (Inputs)

| 话题名称 | 消息类型 | 描述 |
| :--- | :--- | :--- |
| 无 | - | 该功能包作为纯算法库，不直接订阅话题 |

### 3.2 发布的话题 (Outputs)

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| 无 | - | - | 该功能包作为纯算法库，不直接发布话题 |

### 3.3 发布/订阅的坐标系 (TF)

*   **需要输入的TF**: 无
*   **发布的TF**: 无

### 3.4 C++编程接口

**主要类**: `AStar`

```cpp
class AStar {
public:
    // 初始化栅格地图
    void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

    // 执行A*搜索
    bool AstarSearch(const double step_size,
                    Eigen::Vector3d start_pt,
                    Eigen::Vector3d end_pt);

    // 获取规划路径
    std::vector<Eigen::Vector3d> getPath();

    // 获取访问节点(用于可视化)
    std::vector<Eigen::Vector3d> getVisitedNodes();
};
```

## 4. 核心算法原理

### A*算法实现

该实现基于标准A*算法，针对3D空间路径规划进行了优化。

#### 数据结构设计

```cpp
struct GridNode {
    enum State {OPENSET, CLOSEDSET, UNDEFINED};

    State state;        // 节点状态
    int rounds;         // 轮次标识，支持多次搜索
    double gScore;      // g值：从起点到当前节点的实际代价
    double fScore;      // f值：f = g + h 总评估代价
    GridNodePtr cameFrom;  // 父节点指针，用于路径回溯
};
```

#### 启发式函数

1. **对角线启发式** (主要使用)
   $$h = \sqrt{3} \cdot d_{diag3} + \sqrt{2} \cdot d_{diag2} + d_{straight}$$
   其中：
   - $d_{diag3}$: 三维对角线移动距离
   - $d_{diag2}$: 二维对角线移动距离
   - $d_{straight}$: 直线移动距离

2. **曼哈顿距离**
   $$h = |x_g - x_c| + |y_g - y_c| + |z_g - z_c|$$

3. **欧几里得距离**
   $$h = \sqrt{(x_g - x_c)^2 + (y_g - y_c)^2 + (z_g - z_c)^2}$$

#### 搜索流程

```
1. 初始化
   - 设置起点gScore = 0
   - 计算起点fScore = gScore + h(start, goal)
   - 将起点加入openSet

2. 主循环 (while openSet不为空)
   a. 取出fScore最小的节点current
   b. 如果current是目标点，回溯路径并返回
   c. 将current移至closedSet
   d. 遍历current的26个邻居节点：
      - 计算tentative_gScore
      - 如果找到更优路径，更新节点信息
      - 将新节点加入openSet

3. 路径回溯
   - 从目标点开始，通过cameFrom指针回溯
   - 生成完整路径点序列
```

#### 关键优化技术

1. **轮次机制**: 通过`rounds_`标识避免每次搜索重置整个节点地图
2. **动态起终点调整**: 自动调整被障碍物占据的起终点位置
3. **时间限制**: 0.2秒超时保护，避免长时间阻塞
4. **内存池管理**: 预分配3D节点数组，避免频繁内存分配
5. **Tie-breaker**: 使用1.0001系数避免等价路径的随机选择

## 5. 使用与配置

### 5.1 基本使用示例

```cpp
#include <ego_path_searching/astar.h>

// 1. 创建A*实例
AStar::Ptr astar_searcher_;
astar_searcher_.reset(new AStar);

// 2. 初始化栅格地图
Eigen::Vector3i pool_size(100, 100, 50);  // 定义搜索空间大小
astar_searcher_->initGridMap(grid_map, pool_size);

// 3. 执行路径搜索
Eigen::Vector3d start_pt(0, 0, 1);   // 起点
Eigen::Vector3d end_pt(10, 10, 1);   // 终点
double step_size = 0.1;              // 栅格分辨率

bool success = astar_searcher_->AstarSearch(step_size, start_pt, end_pt);

// 4. 获取路径
if (success) {
    std::vector<Eigen::Vector3d> path = astar_searcher_->getPath();
    // 使用路径进行后续处理
}
```

### 5.2 性能参数

| 参数 | 说明 | 推荐值 |
| :--- | :--- | :--- |
| `pool_size` | 搜索空间大小 | 根据地图范围设置 |
| `step_size` | 栅格分辨率 | 0.1 - 0.2 m |
| `lambda_heu` | 启发式权重 | 1.0001 |
| `allocate_num` | 最大节点数 | 100000 |
| `time_limit` | 搜索超时时间 | 0.2 秒 |

### 5.3 典型应用场景

- **前端路径生成**: 为轨迹优化提供初始路径
- **动态重规划**: 在环境变化时快速重新搜索
- **安全路径**: 考虑机器人膨胀尺寸的安全路径规划
- **多分辨率搜索**: 支持不同精度需求的路径搜索