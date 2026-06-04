# 功能包: ego_plan_env

## 1. 功能概述

*   **一句话总结**: 基于深度相机的概率栅格地图构建与占用状态查询系统
*   **核心节点**: 无独立节点，作为库提供GridMap类供其他模块使用
*   **算法类型**: 概率占用栅格映射 + 体素光线投射 + 障碍物膨胀

## 2. 依赖关系

*   **主要依赖项**:
    *   `roscpp`: ROS C++客户端库
    *   `std_msgs`: 标准消息类型
    *   `visualization_msgs`: 可视化消息
    *   `sensor_msgs`: 传感器消息（深度图像、点云）
    *   `cv_bridge`: OpenCV与ROS图像消息桥接
    *   `message_filters`: 消息同步过滤器
    *   `geometry_msgs`: 几何消息类型
    *   `nav_msgs`: 导航消息类型
    *   `Eigen3`: 线性代数运算
    *   `PCL 1.7`: 点云数据结构
    *   `OpenCV`: 深度图像处理

## 3. 接口说明 (API)

### 3.1 订阅的话题 (Inputs)

| 话题名称 | 消息类型 | 描述 |
| :--- | :--- | :--- |
| `/grid_map/depth` | `sensor_msgs/Image` | 深度相机图像数据 |
| `/grid_map/pose` | `geometry_msgs/PoseStamped` | 相机位姿（单独位姿输入） |
| `/grid_map/odom` | `nav_msgs/Odometry` | 相机里程计（包含位姿和速度） |
| `/grid_map/cloud` | `sensor_msgs/PointCloud2` | 点云数据（可选输入方式） |

### 3.2 发布的话题 (Outputs)

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| 占用栅格可视化 | `visualization_msgs/Marker` | ~10 | 占用体素的立方体显示 |
| 膨胀栅格可视化 | `visualization_msgs/Marker` | ~10 | 膨胀后占用体素显示 |
| 未知区域可视化 | `visualization_msgs/Marker` | ~10 | 未探索区域显示 |

### 3.3 C++编程接口

```cpp
class GridMap {
public:
    // 初始化地图
    void initMap(ros::NodeHandle& nh);

    // 重置地图
    void resetBuffer();
    void resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos);

    // 占用查询
    int getOccupancy(Eigen::Vector3d pos);        // 获取占用状态
    int getInflateOccupancy(Eigen::Vector3d pos); // 获取膨胀后占用状态

    // 距离场查询
    double getDistance(Eigen::Vector3d pos);      // 获取到最近障碍物距离
    void getDistanceAndGradient(Eigen::Vector3d pos,
                                double& dist,
                                Eigen::Vector3d& grad);

    // 坐标转换
    void posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id);
    void indexToPos(Eigen::Vector3i id, Eigen::Vector3d& pos);

    // 地图参数
    Eigen::Vector3d getMapSize();
    double getResolution();

    // 可视化
    void publishMap();
    void publishMapInflate();
};
```

## 4. 核心算法原理

### 概率占用栅格映射

#### 贝叶斯更新模型

使用对数几率(log-odds)表示：
$$l_t = l_{t-1} + \log \frac{p(m|z_t)}{1-p(m|z_t)} - l_0$$

其中：
- $l_t$: 时刻t的对数几率
- $p(m|z_t)$: 观测概率
- $l_0$: 先验对数几率

#### 概率裁剪

```cpp
// 防止过度确定
if (l > l_max) l = l_max;  // p_max = 0.97
if (l < l_min) l = l_min;  // p_min = 0.12
```

### 深度图像处理与光线投射

#### 深度图像投影

将像素坐标转换为3D点：
$$\begin{bmatrix} x \\ y \\ z \end{bmatrix} =
z \begin{bmatrix}
(u - c_x)/f_x \\
(v - c_y)/f_y \\
1
\end{bmatrix}$$

其中：
- $(u,v)$: 像素坐标
- $z$: 深度值
- $(f_x, f_y)$: 焦距
- $(c_x, c_y)$: 主点

#### 改进的John Amanatides光线投射算法

```cpp
// 3D DDA算法核心
while (true) {
    // 标记当前体素为free
    setOccupancy(current_voxel, FREE);

    // 计算下一个体素
    tMax = min(tMaxX, tMaxY, tMaxZ);
    if (tMax == tMaxX) {
        current_voxel.x += stepX;
        tMaxX += tDeltaX;
    }
    // ... 类似处理Y和Z

    if (到达终点) break;
}
```

### 局部地图更新策略

1. **深度滤波**:
   ```cpp
   // 过滤无效深度
   if (depth < depth_min || depth > depth_max) continue;

   // 容差过滤
   if (fabs(depth - depth_last) > depth_filter_tolerance) {
       depth = depth_last;
   }
   ```

2. **增量更新**:
   - 仅更新相机视野范围内的体素
   - 使用局部边界框限制更新区域
   - 跳跃采样减少计算量

3. **障碍物膨胀**:
   ```cpp
   // 为每个占用体素进行膨胀
   for (each occupied voxel) {
       for (each neighbor in inflation_radius) {
           mark_as_inflated(neighbor);
       }
   }
   ```

### 优化技术

1. **体素哈希**: 使用哈希表加速体素查询
2. **光线缓存**: 缓存光线投射中间结果
3. **像素跳跃**: 降采样深度图像减少计算
4. **并行处理**: 多线程处理不同图像区域

## 5. 使用与配置

### 5.1 参数配置

```yaml
grid_map:
  # 地图参数
  resolution: 0.1              # 栅格分辨率(米)
  map_size_x: 20.0            # 地图x尺寸
  map_size_y: 20.0            # 地图y尺寸
  map_size_z: 5.0             # 地图z尺寸
  local_update_range_x: 5.0   # 局部更新范围x
  local_update_range_y: 5.0   # 局部更新范围y
  local_update_range_z: 4.0   # 局部更新范围z

  # 膨胀参数
  obstacles_inflation: 0.2     # 障碍物膨胀半径

  # 相机内参
  fx: 387.229                 # 焦距x
  fy: 387.229                 # 焦距y
  cx: 321.04                  # 主点x
  cy: 243.44                  # 主点y

  # 概率参数
  p_hit: 0.70                 # 击中概率
  p_miss: 0.35                # 未击中概率
  p_min: 0.12                 # 最小概率
  p_max: 0.97                 # 最大概率
  p_occ: 0.80                 # 占用阈值

  # 深度滤波
  depth_filter_maxdist: 4.5   # 最大深度距离
  depth_filter_mindist: 0.2   # 最小深度距离
  depth_filter_tolerance: 0.1 # 深度容差
  depth_filter_margin: 1     # 边缘像素忽略
  skip_pixel: 2               # 像素跳跃步长

  # 优化参数
  max_ray_length: 4.5         # 最大光线长度
  use_depth_filter: true      # 启用深度滤波
```

### 5.2 使用示例

```cpp
#include <ego_plan_env/grid_map.h>

class PlanningNode {
private:
    GridMap::Ptr grid_map_;

public:
    void init() {
        grid_map_.reset(new GridMap);
        grid_map_->initMap(nh);
    }

    void planPath(Eigen::Vector3d start, Eigen::Vector3d goal) {
        // 检查起终点是否可行
        if (grid_map_->getInflateOccupancy(start) > 0) {
            ROS_WARN("Start point occupied!");
            return;
        }

        // 使用地图进行路径规划
        for (auto& point : path) {
            int occ = grid_map_->getInflateOccupancy(point);
            if (occ == 1) {  // 占用
                // 需要重规划
            }
        }

        // 获取距离信息用于优化
        double dist;
        Eigen::Vector3d grad;
        grid_map_->getDistanceAndGradient(point, dist, grad);
    }
};
```

### 5.3 典型应用场景

- **深度相机建图**: 实时处理RGB-D相机数据构建地图
- **碰撞检测**: 为路径规划提供占用查询接口
- **距离场生成**: 用于轨迹优化的梯度信息
- **局部地图维护**: 移动机器人的滑动窗口地图更新