# 功能包: ego_local_sensing

## 1. 功能概述

*   **一句话总结**: 基于全局/局部点云地图的本地传感器仿真器，支持CUDA加速的深度图像渲染
*   **核心节点**: `ego_pcl_render_node` (非CUDA版本) / `pcl_render_node` (CUDA版本)
*   **算法类型**: 点云处理、深度图像渲染、传感器仿真

## 2. 依赖关系

*   **主要依赖项**:
    *   `pcl_ros`: 点云数据处理和ROS消息转换
    *   `cv_bridge`: OpenCV与ROS图像消息的桥接
    *   `image_transport`: 高效的图像传输
    *   `sensor_msgs`: 传感器数据消息定义
    *   `geometry_msgs`: 几何消息定义
    *   `nav_msgs`: 导航消息定义
    *   `quadrotor_msgs`: 四旋翼特定消息
    *   `Eigen3`: 线性代数运算
    *   `OpenCV`: 图像处理
    *   `CUDA` (可选): GPU加速计算

## 3. 接口说明 (API)

### 3.1 订阅的话题 (Inputs)

| 话题名称 | 消息类型 | 描述 |
| :--- | :--- | :--- |
| `global_map` | `sensor_msgs/PointCloud2` | 全局点云地图 |
| `local_map` | `sensor_msgs/PointCloud2` | 局部点云地图 |
| `odometry` | `nav_msgs/Odometry` | 机体里程计信息 |

### 3.2 发布的话题 (Outputs)

**非CUDA版本**：

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| `/pcl_render_node/cloud` | `sensor_msgs/PointCloud2` | 30 | 渲染的局部点云 |

**CUDA版本**：

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| `depth` | `sensor_msgs/Image` | 30 | 深度图像 |
| `colordepth` | `sensor_msgs/Image` | 30 | 彩色深度图像 |
| `camera_pose` | `geometry_msgs/PoseStamped` | 30 | 相机位姿 |
| `rendered_pcl` | `sensor_msgs/PointCloud2` | 30 | 渲染的点云 |

### 3.3 发布/订阅的坐标系 (TF)

*   **需要输入的TF**: 无
*   **发布的TF**: 无
*   **使用的frame_id**: `world`

### 3.4 提供的服务 (Services)

| 服务名称 | 服务类型 | 描述 |
| :--- | :--- | :--- |
| 无 | - | 该功能包不提供服务 |

## 4. 核心算法原理

### 非CUDA版本算法 (pointcloud_render_node.cpp)

#### 局部感知算法

1. **KD树半径搜索**
   ```cpp
   // 构建KD树进行高效搜索
   pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
   kdtree.setInputCloud(global_cloud);

   // 半径搜索
   vector<int> indices;
   vector<float> distances;
   kdtree.radiusSearch(searchPoint, sensing_horizon,
                      indices, distances);
   ```

2. **视角过滤**
   ```cpp
   // 高度角过滤 (仰角 < 30°)
   double relative_height = fabs(pt.z - odom_z);
   if ((relative_height / distance) > tan(M_PI/6.0))
       continue;  // 跳过过高/过低的点

   // 方向过滤 (前向视野)
   Vector3d pt_vec = (pt - odom_pos).normalized();
   if (pt_vec.dot(yaw_vec) < 0.5)  // cos(60°) = 0.5
       continue;  // 跳过侧后方的点
   ```

3. **点云处理流程**
   ```
   输入: 全局点云 + 里程计
   1. 提取机体位置和朝向
   2. KD树半径搜索(sensing_horizon)
   3. 应用高度角过滤
   4. 应用方向过滤
   5. 坐标变换到机体系
   输出: 局部感知点云
   ```

### CUDA版本算法 (pcl_render_node.cpp)

#### 深度渲染管线

1. **坐标变换矩阵**
   ```cpp
   // 机体到相机变换
   Matrix4d body2camera;
   body2camera << 0, 0, 1, 0.1,
                 -1, 0, 0, 0,
                  0, -1, 0, 0,
                  0, 0, 0, 1;

   // 世界到相机变换
   Matrix4d world2camera = body2camera * world2body;
   ```

2. **相机投影模型**

   针孔相机模型：
   $$\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} =
   \frac{1}{z} \begin{bmatrix}
   f_x & 0 & c_x \\
   0 & f_y & c_y \\
   0 & 0 & 1
   \end{bmatrix}
   \begin{bmatrix} x \\ y \\ z \end{bmatrix}$$

3. **CUDA深度渲染**
   ```cuda
   __global__ void renderDepth(float* points,
                               float* depth_buffer,
                               int width, int height) {
       int idx = blockIdx.x * blockDim.x + threadIdx.x;
       if (idx >= num_points) return;

       // 投影到像素坐标
       float u = fx * x / z + cx;
       float v = fy * y / z + cy;

       // Z-buffer测试
       int pixel_idx = int(v) * width + int(u);
       atomicMin(&depth_buffer[pixel_idx], z);
   }
   ```

4. **点云重建**
   ```cpp
   // 从深度图重建3D点
   for (int v = 0; v < height; v++) {
       for (int u = 0; u < width; u++) {
           float depth = depth_image.at<float>(v, u);
           if (depth > 0) {
               // 相机坐标系
               Vector3d pt_cam;
               pt_cam.x() = (u - cx) * depth / fx;
               pt_cam.y() = (v - cy) * depth / fy;
               pt_cam.z() = depth;

               // 世界坐标系
               Vector3d pt_world = cam2world * pt_cam;
               cloud.points.push_back(pt_world);
           }
       }
   }
   ```

### 传感器仿真特性

#### 深度相机参数

- **内参矩阵**:
  ```
  K = [fx  0  cx]
      [0  fy  cy]
      [0   0   1]
  ```

- **典型参数值**:
  - 焦距: fx = fy = 387.229
  - 主点: cx = 321.04, cy = 243.44
  - 分辨率: 640×480

#### 传感器噪声模型

1. **深度噪声**:
   $$d_{measured} = d_{true} + \sigma_d \cdot \mathcal{N}(0,1)$$
   其中$\sigma_d = 0.01 \cdot d_{true}$ (1%相对误差)

2. **遮挡处理**:
   - Z-buffer算法处理遮挡
   - 边缘滤波去除飞点

## 5. 使用与配置

### 5.1 启动示例

```bash
# 非CUDA版本
roslaunch ego_local_sensing ego_local_sensing.launch

# CUDA版本
roslaunch ego_local_sensing cuda_local_sensing.launch
```

### 5.2 参数配置

```yaml
# 传感器参数
sensing_horizon: 5.0      # 感知范围 (米)
sensing_rate: 30.0        # 更新频率 (Hz)
estimation_rate: 100.0    # 估计频率 (Hz)

# 相机参数 (CUDA版本)
cam_width: 640           # 图像宽度
cam_height: 480          # 图像高度
cam_fx: 387.229          # 焦距x
cam_fy: 387.229          # 焦距y
cam_cx: 321.04           # 主点x
cam_cy: 243.44           # 主点y

# 地图参数
map:
  x_size: 40.0           # 地图x范围
  y_size: 40.0           # 地图y范围
  z_size: 5.0            # 地图z范围

# 初始位置
init_state_x: 0.0
init_state_y: 0.0
init_state_z: 0.0
```

### 5.3 典型使用场景

#### 场景1：仿真深度相机
```xml
<node name="local_sensing" pkg="ego_local_sensing" type="pcl_render_node">
    <remap from="global_map" to="/map_generator/global_cloud"/>
    <remap from="odometry" to="/mavros/local_position/odom"/>
    <remap from="depth" to="/camera/depth/image_raw"/>
</node>
```

#### 场景2：局部点云提取
```xml
<node name="local_sensing" pkg="ego_local_sensing" type="ego_pcl_render_node">
    <param name="sensing_horizon" value="10.0"/>
    <param name="sensing_rate" value="10.0"/>
    <remap from="/pcl_render_node/cloud" to="/local_pointcloud"/>
</node>
```

### 5.4 性能优化

#### CUDA优化策略

1. **内存管理**:
   ```cpp
   // 预分配GPU内存
   cudaMalloc(&d_points, max_points * sizeof(float3));
   cudaMalloc(&d_depth, width * height * sizeof(float));
   ```

2. **并行渲染**:
   ```cuda
   dim3 blockSize(256);
   dim3 gridSize((num_points + blockSize.x - 1) / blockSize.x);
   renderDepth<<<gridSize, blockSize>>>(d_points, d_depth);
   ```

3. **流水线优化**:
   ```cpp
   // 使用CUDA流实现并行
   cudaStream_t stream1, stream2;
   cudaStreamCreate(&stream1);
   cudaStreamCreate(&stream2);

   // 异步内存传输和计算
   cudaMemcpyAsync(..., stream1);
   renderKernel<<<..., stream1>>>(...);
   ```

#### CPU优化策略

1. **KD树缓存**: 避免重复构建
2. **多线程处理**: OpenMP并行化点云过滤
3. **内存池**: 预分配点云内存避免频繁分配