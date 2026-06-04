# 功能包: map_generator

## 1. 功能概述

*   **一句话总结**: 随机森林环境生成器，用于创建仿真测试环境的3D障碍物地图
*   **核心节点**: `random_forest` / `map_generator`
*   **算法类型**: 随机障碍物生成 + 点云发布

## 2. 依赖关系

*   **主要依赖项**:
    *   `roscpp`: ROS C++客户端库
    *   `geometry_msgs`: 几何消息类型
    *   `nav_msgs`: 导航消息（OccupancyGrid）
    *   `sensor_msgs`: 传感器消息（PointCloud2）
    *   `pcl_ros`: PCL点云库ROS接口
    *   `pcl_conversions`: PCL与ROS消息转换
    *   `Eigen3`: 线性代数运算

## 3. 接口说明 (API)

### 3.1 订阅的话题 (Inputs)

| 话题名称 | 消息类型 | 描述 |
| :--- | :--- | :--- |
| 无 | - | 该节点不订阅任何话题 |

### 3.2 发布的话题 (Outputs)

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| `global_cloud` | `sensor_msgs/PointCloud2` | 1 | 全局障碍物点云 |
| `global_map` | `nav_msgs/OccupancyGrid` | 1 | 2D占用栅格地图 |
| `all_map` | `sensor_msgs/PointCloud2` | 单次 | 完整地图点云（包含边界） |

### 3.3 发布/订阅的坐标系 (TF)

*   **需要输入的TF**: 无
*   **发布的TF**: 无
*   **使用的frame_id**: `world`

### 3.4 提供的服务 (Services)

| 服务名称 | 服务类型 | 描述 |
| :--- | :--- | :--- |
| 无 | - | 该节点不提供服务 |

## 4. 核心算法原理

### 随机森林生成算法

#### 障碍物生成流程

1. **地图初始化**
   ```cpp
   // 创建地图边界
   for (x = -x_size/2; x < x_size/2; x += resolution) {
       for (y = -y_size/2; y < y_size/2; y += resolution) {
           if (is_boundary(x, y)) {
               cloud_map.points.push_back(Point(x, y, z));
           }
       }
   }
   ```

2. **圆柱体障碍物生成**
   ```cpp
   struct Cylinder {
       double x, y;      // 中心位置
       double radius;    // 半径
       double height;    // 高度
   };
   ```

3. **随机放置策略**
   - 随机生成圆柱体位置: $(x, y) \in [-x_{size}/2, x_{size}/2] \times [-y_{size}/2, y_{size}/2]$
   - 随机生成半径: $r \in [r_{min}, r_{max}]$
   - 随机生成高度: $h \in [h_{min}, h_{max}]$

4. **碰撞检测**
   ```cpp
   bool isValidCylinder(Cylinder& c) {
       // 检查与已有圆柱体的碰撞
       for (auto& existing : cylinders) {
           double dist = sqrt(pow(c.x - existing.x, 2) +
                             pow(c.y - existing.y, 2));
           if (dist < c.radius + existing.radius + margin) {
               return false;
           }
       }
       return true;
   }
   ```

### 点云生成

#### 圆柱体表面采样

对每个圆柱体进行表面点云采样：
```cpp
for (double theta = 0; theta < 2*M_PI; theta += theta_step) {
    for (double h = 0; h < cylinder.height; h += height_step) {
        double x = cylinder.x + cylinder.radius * cos(theta);
        double y = cylinder.y + cylinder.radius * sin(theta);
        double z = h;
        cloud.points.push_back(Point(x, y, z));
    }
}
```

#### 密度控制

- **角度分辨率**: $\Delta\theta = 2\pi / n_{theta}$
- **高度分辨率**: $\Delta h = h_{max} / n_{height}$
- **总点数估计**: $N = n_{cylinders} \times n_{theta} \times n_{height}$

### 2D占用栅格生成

将3D点云投影到2D平面：
```cpp
for (auto& point : cloud_map.points) {
    int idx_x = floor((point.x - map_origin_x) / resolution);
    int idx_y = floor((point.y - map_origin_y) / resolution);

    if (idx_x >= 0 && idx_x < map_width &&
        idx_y >= 0 && idx_y < map_height) {
        occupancy_grid.data[idx_y * map_width + idx_x] = 100; // 占用
    }
}
```

### 地图发布策略

1. **定时发布**: 使用ROS Timer以固定频率发布
2. **点云优化**: 使用PCL的VoxelGrid滤波器降采样
3. **消息时间戳**: 同步所有发布消息的时间戳

## 5. 使用与配置

### 5.1 启动示例

```bash
# 基础启动
rosrun map_generator random_forest

# 使用launch文件
roslaunch map_generator random_forest.launch
```

### 5.2 参数配置

```yaml
map_generator:
  # 地图尺寸
  map/x_size: 40.0          # x方向尺寸(米)
  map/y_size: 40.0          # y方向尺寸(米)
  map/z_size: 5.0           # z方向尺寸(米)

  # 障碍物参数
  map/obs_num: 30           # 圆柱体数量
  map/circle_num: 30        # 圆形障碍物数量

  # 圆柱体尺寸范围
  ObstacleShape/lower_rad: 0.3    # 最小半径
  ObstacleShape/upper_rad: 0.8    # 最大半径
  ObstacleShape/lower_hei: 3.0    # 最小高度
  ObstacleShape/upper_hei: 7.0    # 最大高度

  # 分辨率设置
  map/resolution: 0.1       # 点云分辨率
  map/cloud_rate: 1.0       # 点云发布频率(Hz)

  # 初始化参数
  init_state_x: 0.0         # 初始x位置
  init_state_y: 0.0         # 初始y位置
```

### 5.3 RViz可视化配置

```yaml
# 点云显示
PointCloud2:
  Topic: /map_generator/global_cloud
  Size: 0.1
  Color Transformer: FlatColor
  Color: 255; 0; 0

# 栅格地图显示
Map:
  Topic: /map_generator/global_map
  Color Scheme: map
```

### 5.4 典型应用场景

#### 场景1：算法测试环境
```cpp
// 生成稀疏环境用于路径规划测试
map/obs_num: 10
ObstacleShape/lower_rad: 0.5
ObstacleShape/upper_rad: 1.0
```

#### 场景2：密集森林环境
```cpp
// 生成密集环境用于避障测试
map/obs_num: 50
ObstacleShape/lower_rad: 0.2
ObstacleShape/upper_rad: 0.5
```

#### 场景3：混合障碍物环境
```cpp
// 结合大小障碍物
// 通过多次调用生成不同尺寸障碍物
generateLargeObstacles(10, 1.0, 2.0);
generateSmallObstacles(30, 0.2, 0.5);
```

### 5.5 扩展功能

#### 自定义障碍物形状
```cpp
class CustomObstacle {
public:
    virtual void generatePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud) = 0;
};

class BoxObstacle : public CustomObstacle {
    void generatePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud) {
        // 生成长方体点云
    }
};
```

#### 动态障碍物支持
```cpp
void updateMovingObstacles(double time) {
    for (auto& obs : moving_obstacles) {
        obs.x += obs.vx * dt;
        obs.y += obs.vy * dt;
    }
    regeneratePointCloud();
}
```