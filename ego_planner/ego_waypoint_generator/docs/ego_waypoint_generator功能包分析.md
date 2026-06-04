# 功能包: ego_waypoint_generator

## 1. 功能概述

*   **一句话总结**: 航点生成器，支持多种轨迹模式生成和手动航点设置
*   **核心节点**: `ego_waypoint_generator`
*   **算法类型**: 轨迹规划和航点生成

## 2. 依赖关系

*   **主要依赖项**:
    *   `roscpp`: ROS C++核心库
    *   `nav_msgs`: 导航消息，使用Path和Odometry类型
    *   `geometry_msgs`: 几何消息，处理位姿和点信息
    *   `tf`: 坐标变换，用于四元数和欧拉角转换
    *   `Eigen`: 线性代数库，用于旋转计算

## 3. 接口说明 (API)

### 3.1 订阅的话题 (Inputs)

| 话题名称 | 消息类型 | 描述 |
| :--- | :--- | :--- |
| `odom` | `nav_msgs/Odometry` | 当前位置状态 |
| `goal` | `geometry_msgs/PoseStamped` | 目标点输入（手动模式） |
| `traj_start_trigger` | `geometry_msgs/PoseStamped` | 轨迹开始触发信号 |

### 3.2 发布的话题 (Outputs)

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| `waypoints` | `nav_msgs/Path` | 按需 | 生成的航点序列 |
| `waypoints_vis` | `geometry_msgs/PoseArray` | 按需 | 可视化用航点数组 |

### 3.3 发布/订阅的坐标系 (TF)

*   **需要输入的TF**: 无
*   **发布的TF**: 无
*   **使用的frame_id**: `world`

### 3.4 提供的服务 (Services)

| 服务名称 | 服务类型 | 描述 |
| :--- | :--- | :--- |
| 无 | - | 该节点不提供服务 |

## 4. 核心算法原理

### 轨迹生成模式

#### 1. 圆形轨迹 (Circle Mode)

生成多圈圆形飞行轨迹：

```cpp
void generateCircleTrajectory() {
    double radius = 5.0;        // 圆半径
    int points_per_circle = 30; // 每圈点数
    int num_circles = 3;        // 圈数

    for (int i = 0; i < num_circles * points_per_circle; i++) {
        double theta = 2 * M_PI * i / points_per_circle;

        Eigen::Vector3d waypoint;
        waypoint.x() = center.x() + radius * cos(theta);
        waypoint.y() = center.y() + radius * sin(theta);
        waypoint.z() = center.z() + 0.1 * i; // 螺旋上升

        waypoints.push_back(waypoint);
    }
}
```

#### 2. 八字形轨迹 (Eight Mode)

生成水平8字形轨迹：

```cpp
void generateEightTrajectory() {
    double a = 5.0;  // 8字形参数

    for (double t = 0; t < 4 * M_PI; t += 0.1) {
        Eigen::Vector3d waypoint;
        // 参数方程
        waypoint.x() = a * sin(t);
        waypoint.y() = a * sin(t) * cos(t);
        waypoint.z() = base_height + amplitude * sin(t/2);

        waypoints.push_back(waypoint);
    }
}
```

#### 3. 预定义点序列 (Points Mode)

从参数文件加载预定义航点：

```cpp
void loadPredefinedPoints() {
    // 从参数服务器加载
    XmlRpc::XmlRpcValue points_list;
    nh.getParam("waypoint_list", points_list);

    for (int i = 0; i < points_list.size(); i++) {
        Eigen::Vector3d pt;
        pt.x() = points_list[i]["x"];
        pt.y() = points_list[i]["y"];
        pt.z() = points_list[i]["z"];

        waypoints.push_back(pt);
    }
}
```

#### 4. 时间序列轨迹 (Series Mode)

基于时间触发的分段轨迹：

```cpp
struct Segment {
    double time_of_start;       // 开始时间
    vector<Eigen::Vector3d> points; // 航点序列
    double yaw;                 // 偏航角
};

void updateSeriesTrajectory() {
    double current_time = ros::Time::now().toSec() - start_time;

    for (const auto& segment : segments) {
        if (current_time >= segment.time_of_start &&
            !segment_published[segment.id]) {
            publishSegment(segment);
            segment_published[segment.id] = true;
        }
    }
}
```

#### 5. 手动交互模式 (Manual Mode)

通过RViz交互式设置航点：

```cpp
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    double z = msg->pose.position.z;

    if (z > 0) {
        // 添加新航点
        Eigen::Vector3d waypoint(msg->pose.position.x,
                                msg->pose.position.y,
                                z);
        manual_waypoints.push_back(waypoint);
        ROS_INFO("Added waypoint #%zu", manual_waypoints.size());

    } else if (z > -1.0 && z <= 0) {
        // 删除最后一个航点
        if (!manual_waypoints.empty()) {
            manual_waypoints.pop_back();
            ROS_INFO("Removed last waypoint");
        }

    } else {  // z <= -1.0
        // 完成并发布航点
        if (!manual_waypoints.empty()) {
            publishWaypoints(manual_waypoints);
            ROS_INFO("Published %zu waypoints", manual_waypoints.size());
            manual_waypoints.clear();
        }
    }
}
```

### 坐标系变换

#### 相对坐标变换

支持基于当前无人机朝向的相对坐标：

```cpp
Eigen::Vector3d transformToWorld(const Eigen::Vector3d& local_point,
                                const Eigen::Vector3d& origin,
                                double yaw) {
    // 创建旋转矩阵
    Eigen::Matrix3d R;
    R << cos(yaw), -sin(yaw), 0,
         sin(yaw),  cos(yaw), 0,
         0,         0,        1;

    // 应用变换
    return R * local_point + origin;
}
```

### 路径生成与发布

#### Path消息构建

```cpp
nav_msgs::Path constructPath(const vector<Eigen::Vector3d>& waypoints) {
    nav_msgs::Path path;
    path.header.frame_id = "world";
    path.header.stamp = ros::Time::now();

    for (const auto& wp : waypoints) {
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = wp.x();
        pose.pose.position.y = wp.y();
        pose.pose.position.z = wp.z();

        // 计算朝向
        if (i > 0) {
            Eigen::Vector3d dir = waypoints[i] - waypoints[i-1];
            double yaw = atan2(dir.y(), dir.x());
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        } else {
            pose.pose.orientation.w = 1.0;
        }

        path.poses.push_back(pose);
    }

    return path;
}
```

#### PoseArray可视化

```cpp
geometry_msgs::PoseArray constructPoseArray(
    const vector<Eigen::Vector3d>& waypoints) {
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "world";
    pose_array.header.stamp = ros::Time::now();

    for (const auto& wp : waypoints) {
        geometry_msgs::Pose pose;
        pose.position.x = wp.x();
        pose.position.y = wp.y();
        pose.position.z = wp.z();
        pose.orientation.w = 1.0;

        pose_array.poses.push_back(pose);
    }

    return pose_array;
}
```

## 5. 使用与配置

### 5.1 启动示例

```xml
<node name="waypoint_generator"
      pkg="ego_waypoint_generator"
      type="ego_waypoint_generator"
      output="screen">

    <!-- 轨迹类型选择 -->
    <param name="waypoint_type" value="manual"/>
    <!-- 可选: circle/eight/points/series/manual -->

    <!-- 话题映射 -->
    <remap from="odom" to="/mavros/local_position/odom"/>
    <remap from="goal" to="/move_base_simple/goal"/>
    <remap from="waypoints" to="/planning/waypoints"/>
</node>
```

### 5.2 参数配置

#### 基础参数
```yaml
waypoint_generator:
  waypoint_type: "manual"    # 轨迹类型
  output_frame: "world"      # 输出坐标系
```

#### 圆形轨迹参数
```yaml
circle:
  radius: 5.0               # 半径(米)
  num_points: 30            # 每圈点数
  num_circles: 3            # 圈数
  center_x: 0.0             # 中心x坐标
  center_y: 0.0             # 中心y坐标
  height: 2.0               # 飞行高度
```

#### 八字形轨迹参数
```yaml
eight:
  scale: 5.0                # 尺寸缩放
  height_base: 2.0          # 基础高度
  height_amplitude: 1.0     # 高度变化幅度
  num_points: 50            # 总点数
```

#### 预定义点参数
```yaml
waypoint_list:
  - {x: 0.0, y: 0.0, z: 1.0}
  - {x: 5.0, y: 0.0, z: 1.5}
  - {x: 5.0, y: 5.0, z: 2.0}
  - {x: 0.0, y: 5.0, z: 2.5}
  - {x: 0.0, y: 0.0, z: 3.0}
```

#### 分段轨迹参数
```yaml
segment_cnt: 2              # 段数

seg0:
  time_of_start: 0.0        # 开始时间(秒)
  yaw: 0.0                  # 偏航角(弧度)
  x: [0.0, 2.0, 4.0, 6.0]  # x坐标数组
  y: [0.0, 0.0, 0.0, 0.0]  # y坐标数组
  z: [1.0, 1.5, 2.0, 2.5]  # z坐标数组

seg1:
  time_of_start: 10.0
  yaw: 1.57                 # π/2
  x: [6.0, 6.0, 6.0, 6.0]
  y: [0.0, 2.0, 4.0, 6.0]
  z: [2.5, 2.0, 1.5, 1.0]
```

### 5.3 典型使用场景

#### 场景1：演示飞行
```bash
# 圆形轨迹演示
roslaunch waypoint_generator circle_demo.launch

# 八字形轨迹演示
roslaunch waypoint_generator eight_demo.launch
```

#### 场景2：任务规划
```cpp
// 设置巡检任务航点
vector<Eigen::Vector3d> inspection_points = {
    {0, 0, 5},    // 起飞点
    {10, 0, 5},   // 检查点1
    {10, 10, 5},  // 检查点2
    {0, 10, 5},   // 检查点3
    {0, 0, 5},    // 返回起点
    {0, 0, 0}     // 降落
};
```

#### 场景3：交互式路径设置
```bash
# 启动手动模式
rosrun waypoint_generator waypoint_generator _waypoint_type:=manual

# 在RViz中使用2D Nav Goal工具设置航点
# z > 0: 添加航点
# -1 < z <= 0: 删除最后航点
# z <= -1: 发布所有航点
```

### 5.4 与规划器集成

```cpp
class PlannerInterface {
    ros::Subscriber waypoint_sub;
    vector<Eigen::Vector3d> waypoints;

    void waypointCallback(const nav_msgs::Path::ConstPtr& msg) {
        waypoints.clear();
        for (const auto& pose : msg->poses) {
            waypoints.emplace_back(
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z
            );
        }

        // 触发规划
        planTrajectory(waypoints);
    }
};
```

### 5.5 扩展功能

#### 自定义轨迹生成器
```cpp
class CustomTrajectoryGenerator {
public:
    virtual vector<Eigen::Vector3d> generate() = 0;
};

class SpiralTrajectory : public CustomTrajectoryGenerator {
    vector<Eigen::Vector3d> generate() override {
        vector<Eigen::Vector3d> waypoints;
        // 生成螺旋轨迹
        for (double t = 0; t < 10; t += 0.1) {
            double r = 0.5 * t;
            waypoints.emplace_back(
                r * cos(t),
                r * sin(t),
                0.2 * t
            );
        }
        return waypoints;
    }
};
```