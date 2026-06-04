/**
 * @file pcl_render_node.cpp
 * @brief 点云渲染节点 - 使用GPU加速模拟深度相机传感器
 *
 * 该节点接收全局/局部点云地图和无人机位姿，通过GPU加速的深度渲染器
 * 模拟深度相机输出，生成深度图像和彩色深度图，同时发布相机视野内的点云
 *
 * 主要功能：
 * - 接收全局/局部点云地图
 * - 订阅无人机里程计信息
 * - 基于相机位姿渲染深度图
 * - 发布深度图像、彩色深度图和可见点云
 */

// ============== C++标准库 ==============
#include <iostream>  // 标准输入输出流
#include <fstream>   // 文件流（未使用）
#include <vector>    // 动态数组容器

// ============== ROS核心库 ==============
#include <ros/ros.h>  // ROS基础功能

// ============== ROS消息过滤器库 ==============
// 用于同步多个话题的消息（本代码中未实际使用）
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// ============== ROS标准消息类型 ==============
#include <geometry_msgs/PoseStamped.h>      // 带时间戳的位姿消息
#include <geometry_msgs/TransformStamped.h>  // 带时间戳的变换消息（未使用）
#include <image_transport/image_transport.h> // 图像传输工具（未使用）
#include <dynamic_reconfigure/server.h>      // 动态参数配置（未使用）
#include <nav_msgs/Odometry.h>               // 里程计消息（位置+速度）
#include <nav_msgs/Path.h>                   // 路径消息（未使用）
#include <sensor_msgs/Imu.h>                 // IMU消息（未使用）
#include <sensor_msgs/PointCloud2.h>         // 点云消息
#include <std_msgs/Bool.h>                   // 布尔消息（未使用）

// ============== TF坐标变换库 ==============
#include "tf/tf.h"                      // TF核心功能
#include "tf/transform_datatypes.h"     // TF数据类型
#include <tf/transform_broadcaster.h>   // TF广播器（已注释）

// ============== PCL点云处理库 ==============
#include <pcl/io/pcd_io.h>               // PCD文件I/O（未使用）
#include <pcl/io/ply_io.h>               // PLY文件I/O（未使用）
#include <pcl/point_cloud.h>             // 点云数据结构
#include <pcl/point_types.h>             // 点云数据类型
#include <pcl_conversions/pcl_conversions.h>  // PCL与ROS消息转换

// ============== Eigen线性代数库 ==============
#include <Eigen/Eigen>  // Eigen核心功能（矩阵、向量运算）

// ============== OpenCV图像处理库 ==============
#include "opencv2/highgui/highgui.hpp"  // 图像显示和GUI
#include <opencv2/opencv.hpp>           // OpenCV核心功能
#include <opencv2/core/eigen.hpp>       // OpenCV与Eigen数据转换（未使用）
#include <cv_bridge/cv_bridge.h>        // OpenCV与ROS图像消息转换

#include "depth_render.cuh"  // GPU深度渲染器头文件（CUDA实现）
#include "quadrotor_msgs/PositionCommand.h"  // 四旋翼位置命令消息（未使用）

// 使用命名空间简化代码
using namespace cv;     // OpenCV库命名空间
using namespace std;    // C++标准库命名空间
using namespace Eigen;  // Eigen线性代数库命名空间

// ============== 全局变量定义 ==============

// 深度渲染相关
int *depth_hostptr;       // 深度图像主机端指针（从GPU拷贝的深度数据）
cv::Mat depth_mat;        // OpenCV深度图像矩阵

// 相机内参
int width, height;        // 图像宽度和高度（像素）
double fx, fy, cx, cy;    // 相机内参：焦距(fx,fy)和主点(cx,cy)

// 深度渲染器实例
DepthRender depthrender;  // GPU深度渲染器对象

// ROS发布者
ros::Publisher pub_depth;       // 发布深度图像
ros::Publisher pub_color;       // 发布彩色深度图像
ros::Publisher pub_pose;        // 发布相机位姿
ros::Publisher pub_pcl_wolrd;   // 发布世界坐标系下的可见点云

// 点云消息
sensor_msgs::PointCloud2 local_map_pcl;    // 局部地图点云
sensor_msgs::PointCloud2 local_depth_pcl;  // 深度点云

// ROS订阅者
ros::Subscriber odom_sub;                   // 里程计订阅者
ros::Subscriber global_map_sub, local_map_sub;  // 全局和局部地图订阅者

// ROS定时器
ros::Timer local_sensing_timer;   // 局部感知定时器
ros::Timer estimation_timer;      // 状态估计定时器

// 状态标志
bool has_global_map(false);  // 是否已接收全局地图
bool has_local_map(false);   // 是否已接收局部地图
bool has_odom(false);        // 是否已接收里程计数据

// 坐标变换
Matrix4d cam02body;              // 相机到机体的变换矩阵
Matrix4d cam2world;              // 相机到世界坐标系的变换矩阵
Eigen::Quaterniond cam2world_quat;  // 相机到世界坐标系的四元数
nav_msgs::Odometry _odom;        // 里程计数据

// 传感器参数
double sensing_horizon;     // 感知范围（米）
double sensing_rate;        // 感知频率（Hz）
double estimation_rate;     // 估计频率（Hz）

// 地图参数
double _x_size, _y_size, _z_size;       // 地图尺寸（米）
double _gl_xl, _gl_yl, _gl_zl;          // 地图原点坐标
double _resolution, _inv_resolution;     // 地图分辨率及其倒数
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;    // 地图栅格尺寸

// 位姿记录
ros::Time last_odom_stamp = ros::TIME_MAX;  // 上一次里程计时间戳
Eigen::Vector3d last_pose_world;            // 上一次世界坐标系位置

// ============== 函数声明 ==============
void render_currentpose();  // 渲染当前相机位姿的深度图
void render_pcl_world();    // 渲染世界坐标系下的点云

/**
 * @brief 将栅格索引转换为世界坐标
 * @param index 栅格索引 (i, j, k)
 * @return 对应的世界坐标 (x, y, z)
 */
inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index)
{
    Eigen::Vector3d pt;
    // 栅格中心坐标 = (索引 + 0.5) * 分辨率 + 地图原点
    pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
    pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
    pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;

    return pt;
};

/**
 * @brief 将世界坐标转换为栅格索引
 * @param pt 世界坐标 (x, y, z)
 * @return 对应的栅格索引 (i, j, k)，并限制在地图范围内
 */
inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt)
{
    Eigen::Vector3i idx;
    // 索引 = (坐标 - 地图原点) / 分辨率，并限制在有效范围内
    idx(0) = std::min( std::max( int( (pt(0) - _gl_xl) * _inv_resolution), 0), _GLX_SIZE - 1);
    idx(1) = std::min( std::max( int( (pt(1) - _gl_yl) * _inv_resolution), 0), _GLY_SIZE - 1);
    idx(2) = std::min( std::max( int( (pt(2) - _gl_zl) * _inv_resolution), 0), _GLZ_SIZE - 1);

    return idx;
};

/**
 * @brief 里程计数据回调函数
 * @param odom 接收到的里程计消息，包含无人机的位置和姿态
 *
 * 功能：
 * 1. 提取无人机位姿信息
 * 2. 计算相机在世界坐标系下的位姿
 * 3. 更新相机变换矩阵和四元数
 */
void rcvOdometryCallbck(const nav_msgs::Odometry& odom)
{
  // 可选：等待全局地图加载后再处理里程计数据
  /*if(!has_global_map)
    return;*/

  has_odom = true;  // 标记已接收到里程计数据
  _odom = odom;     // 保存里程计数据供后续使用

  // 初始化位姿变换矩阵（4x4齐次变换矩阵）
  Matrix4d Pose_receive = Matrix4d::Identity();

  // ============== 从里程计消息中提取位置信息 ==============
  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = odom.pose.pose.position.x;
  request_position.y() = odom.pose.pose.position.y;
  request_position.z() = odom.pose.pose.position.z;

  // ============== 从里程计消息中提取姿态信息（四元数） ==============
  request_pose.x() = odom.pose.pose.orientation.x;
  request_pose.y() = odom.pose.pose.orientation.y;
  request_pose.z() = odom.pose.pose.orientation.z;
  request_pose.w() = odom.pose.pose.orientation.w;

  // ============== 构建机体（无人机）的4x4位姿变换矩阵 ==============
  // 左上角3x3块：旋转矩阵（从四元数转换）
  Pose_receive.block<3,3>(0,0) = request_pose.toRotationMatrix();
  // 右上角3x1块：平移向量（位置）
  Pose_receive(0,3) = request_position(0);  // X坐标
  Pose_receive(1,3) = request_position(1);  // Y坐标
  Pose_receive(2,3) = request_position(2);  // Z坐标
  // 底部一行保持为[0, 0, 0, 1]（齐次坐标）

  Matrix4d body_pose = Pose_receive;

  // ============== 计算相机在世界坐标系下的位姿 ==============
  // T_cam2world = T_body2world * T_cam2body
  // 其中 cam02body 是相机到机体的固定外参变换
  cam2world = body_pose * cam02body;

  // 从变换矩阵中提取旋转部分并转换为四元数
  cam2world_quat = cam2world.block<3,3>(0,0);

  // ============== 记录时间戳和当前位置 ==============
  last_odom_stamp = odom.header.stamp;  // 保存时间戳用于同步

  // 保存无人机当前位置（用于后续的感知范围过滤）
  last_pose_world(0) = odom.pose.pose.position.x;
  last_pose_world(1) = odom.pose.pose.position.y;
  last_pose_world(2) = odom.pose.pose.position.z;

  // ============== 可选：发布TF变换（已注释） ==============
  // 如果需要在RViz中可视化相机坐标系，可以取消注释以下代码
  /*static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(cam2world(0,3), cam2world(1,3), cam2world(2,3) ));
  transform.setRotation(tf::Quaternion(cam2world_quat.x(), cam2world_quat.y(), cam2world_quat.z(), cam2world_quat.w()));
  br.sendTransform(tf::StampedTransform(transform, last_odom_stamp, "world", "camera")); //publish transform from world frame to quadrotor frame.*/
}

/**
 * @brief 发布相机位姿的定时回调函数
 * @param event ROS定时器事件
 *
 * 功能：将相机在世界坐标系下的位姿发布为PoseStamped消息
 *
 * 用途：供其他节点（如路径规划、SLAM等）使用相机位姿信息
 * 调用频率：由 estimation_rate 参数控制（默认为较高频率）
 */
void pubCameraPose(const ros::TimerEvent & event)
{
  // 构建相机位姿消息
  geometry_msgs::PoseStamped camera_pose;
  camera_pose.header = _odom.header;       // 继承里程计消息的header
  camera_pose.header.frame_id = "/map";    // 参考坐标系设置为地图坐标系（世界坐标系）

  // ============== 从相机变换矩阵中提取位置信息 ==============
  camera_pose.pose.position.x = cam2world(0,3);  // X坐标
  camera_pose.pose.position.y = cam2world(1,3);  // Y坐标
  camera_pose.pose.position.z = cam2world(2,3);  // Z坐标

  // ============== 从四元数中提取姿态信息 ==============
  // 注意：ROS中四元数的顺序为 (x, y, z, w)
  camera_pose.pose.orientation.w = cam2world_quat.w();
  camera_pose.pose.orientation.x = cam2world_quat.x();
  camera_pose.pose.orientation.y = cam2world_quat.y();
  camera_pose.pose.orientation.z = cam2world_quat.z();

  // 发布相机位姿
  pub_pose.publish(camera_pose);
}

/**
 * @brief 渲染感知点云的定时回调函数
 * @param event ROS定时器事件
 *
 * 功能：
 * 1. 检查是否已接收地图和里程计数据
 * 2. 调用GPU渲染器生成深度图
 * 3. 将深度图反投影为世界坐标系点云并发布
 *
 * 调用频率：由 sensing_rate 参数控制（例如：10Hz、30Hz等）
 * 这是核心的传感器模拟循环，周期性生成模拟的深度相机数据
 */
void renderSensedPoints(const ros::TimerEvent & event)
{
  // ============== 检查数据完整性 ==============
  // 至少需要一个地图（全局地图或局部地图）
  if( !has_global_map && !has_local_map) return;

  // 必须有里程计数据（确定相机位姿）
  if( !has_odom ) return;

  // ============== 执行渲染流程 ==============
  render_currentpose();  // 调用GPU渲染器生成深度图，并发布深度图像
  render_pcl_world();    // 将深度图反投影为世界坐标系点云并发布
}

// ============== 点云数据存储容器 ==============
// 用于存储点云坐标的一维浮点数组，按 (x1,y1,z1, x2,y2,z2, ...) 格式存储
// 这种内存布局是为了方便GPU处理和提高缓存命中率
vector<float> cloud_data;

/**
 * @brief 全局点云地图回调函数
 * @param pointcloud_map 接收到的全局点云地图消息
 *
 * 功能：
 * 1. 接收并转换全局点云地图
 * 2. 将点云数据传递给GPU深度渲染器
 * 3. 分配深度图像缓冲区
 *
 * 注意：只在第一次接收时处理，避免重复加载
 */
void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map )
{
  // 如果已经有全局地图，直接返回
  if(has_global_map)
    return;

  ROS_WARN("Global Pointcloud received..");

  // 加载全局地图
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;

  // 将ROS消息转换为PCL点云格式
  pcl::fromROSMsg(pointcloud_map, cloudIn);

  // 将点云数据展开为浮点数组 (x1,y1,z1, x2,y2,z2, ...)
  // GPU渲染器需要连续的内存布局，每个点3个连续的浮点数
  for(int i = 0; i < int(cloudIn.points.size()); i++){
    pt_in = cloudIn.points[i];
    cloud_data.push_back(pt_in.x);
    cloud_data.push_back(pt_in.y);
    cloud_data.push_back(pt_in.z);
  }
  printf("global map has points: %d.\n", (int)cloud_data.size() / 3 );

  // 将点云数据传递给深度渲染器（上传到GPU显存）
  // 这一步会将点云从CPU内存拷贝到GPU显存，用于后续的快速渲染
  depthrender.set_data(cloud_data);

  // 分配主机端深度图像缓冲区（用于存储从GPU拷贝回来的深度数据）
  // 每个像素用一个整数表示深度值（单位：毫米）
  depth_hostptr = (int*) malloc(width * height * sizeof(int));

  has_global_map = true;  // 标记已接收全局地图
}

/**
 * @brief 局部点云地图回调函数
 * @param pointcloud_map 接收到的局部点云地图消息
 *
 * 功能：
 * 1. 接收并转换局部点云地图（动态更新）
 * 2. 将点云数据传递给GPU深度渲染器
 * 3. 分配深度图像缓冲区
 *
 * 注意：与全局地图不同，局部地图可以持续更新
 */
void rcvLocalPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map )
{
  //ROS_WARN("Local Pointcloud received..");

  // 加载局部地图
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;

  // 将ROS消息转换为PCL点云格式
  pcl::fromROSMsg(pointcloud_map, cloudIn);

  // 如果点云为空，直接返回
  if(cloudIn.points.size() == 0) return;

  // 将点云数据展开为浮点数组
  for(int i = 0; i < int(cloudIn.points.size()); i++){
    pt_in = cloudIn.points[i];
    Eigen::Vector3d pose_pt(pt_in.x, pt_in.y, pt_in.z);
    // 可选：将点坐标对齐到栅格中心，确保一致性
    //pose_pt = gridIndex2coord(coord2gridIndex(pose_pt));
    cloud_data.push_back(pose_pt(0));
    cloud_data.push_back(pose_pt(1));
    cloud_data.push_back(pose_pt(2));
  }
  //printf("local map has points: %d.\n", (int)cloud_data.size() / 3 );

  // 将局部点云数据传递给深度渲染器（上传到GPU显存）
  // 局部地图会持续更新，每次调用都会覆盖之前的数据
  depthrender.set_data(cloud_data);

  // 分配主机端深度图像缓冲区（如果之前未分配）
  depth_hostptr = (int*) malloc(width * height * sizeof(int));

  has_local_map = true;  // 标记已接收局部地图
}

/**
 * @brief 将深度图反投影为世界坐标系点云
 *
 * 功能：
 * 1. 遍历深度图的每个像素
 * 2. 根据相机内参将像素坐标和深度值反投影到相机坐标系
 * 3. 通过相机位姿变换将点转换到世界坐标系
 * 4. 过滤超出感知范围的点
 * 5. 发布可见点云
 *
 * 算法原理：
 * - 使用针孔相机模型逆过程（反投影）
 * - 像素坐标(u,v) + 深度d -> 3D点(X,Y,Z)
 * - 坐标变换：相机系 -> 世界系
 * - 距离过滤：仅保留sensing_horizon范围内的点
 */
void render_pcl_world()
{
  // 创建PCL点云对象，用于存储反投影的3D点
  pcl::PointCloud<pcl::PointXYZ> localMap;
  pcl::PointXYZ pt_in;  // 单个点的临时存储

  // 齐次坐标变量定义
  Eigen::Vector4d pose_in_camera;  // 相机坐标系下的齐次坐标 [x, y, z, 1]
  Eigen::Vector4d pose_in_world;   // 世界坐标系下的齐次坐标 [x, y, z, 1]
  Eigen::Vector3d pose_pt;         // 3D点坐标 [x, y, z]

  // 遍历深度图的每个像素（列优先，先遍历宽度再遍历高度）
  for(int u = 0; u < width; u++)
    for(int v = 0; v < height; v++){
      float depth = depth_mat.at<float>(v,u);

      // 跳过无效深度值（0表示无深度信息或超出最大范围）
      if(depth == 0.0)
        continue;

      // 使用针孔相机模型进行反投影：
      // 像素坐标(u,v)和深度值depth反投影到相机坐标系(X_cam, Y_cam, Z_cam)
      // X_cam = (u - cx) * Z / fx  （横向偏移）
      // Y_cam = (v - cy) * Z / fy  （纵向偏移）
      // Z_cam = depth              （深度值）
      pose_in_camera(0) = (u - cx) * depth / fx;
      pose_in_camera(1) = (v - cy) * depth / fy;
      pose_in_camera(2) = depth;
      pose_in_camera(3) = 1.0;  // 齐次坐标的第四维

      // 将相机坐标系点转换到世界坐标系
      // P_world = T_cam2world * P_camera
      pose_in_world = cam2world * pose_in_camera;

      // 过滤超出感知范围的点
      // 计算点到无人机当前位置的距离，超出sensing_horizon则丢弃
      if( (pose_in_world.segment(0,3) - last_pose_world).norm() > sensing_horizon )
          continue;

      pose_pt = pose_in_world.head(3);  // 提取3D坐标（x,y,z）
      // 可选：将点坐标对齐到栅格中心，用于体素化
      //pose_pt = gridIndex2coord(coord2gridIndex(pose_pt));

      // 添加到点云容器
      pt_in.x = pose_pt(0);
      pt_in.y = pose_pt(1);
      pt_in.z = pose_pt(2);

      localMap.points.push_back(pt_in);
    }

  // ============== 设置点云属性 ==============
  // PCL点云可以是有序的（类似图像）或无序的
  localMap.width = localMap.points.size();  // 无序点云，宽度=点的总数
  localMap.height = 1;                       // 高度为1表示无序点云（一维数组）
  localMap.is_dense = true;                  // 标记点云中没有NaN或Inf值

  // ============== 转换为ROS消息并发布 ==============
  pcl::toROSMsg(localMap, local_map_pcl);    // 将PCL点云转换为ROS PointCloud2消息
  local_map_pcl.header.frame_id  = "/map";   // 设置坐标系为地图坐标系
  local_map_pcl.header.stamp     = last_odom_stamp;  // 使用里程计的时间戳

  // 发布渲染的世界坐标系点云（供路径规划等节点使用）
  pub_pcl_wolrd.publish(local_map_pcl);
}

/**
 * @brief 渲染当前相机位姿的深度图
 *
 * 功能：
 * 1. 计算相机位姿的逆变换（世界到相机）
 * 2. 调用GPU深度渲染器生成深度图
 * 3. 将深度数据从GPU拷贝到主机并转换为OpenCV格式
 * 4. 发布原始深度图和彩色深度图
 */
void render_currentpose()
{
  double this_time = ros::Time::now().toSec();

  // 计算世界坐标系到相机坐标系的变换（cam2world的逆）
  // 渲染器需要将世界坐标系的点变换到相机坐标系进行投影
  Matrix4d cam_pose = cam2world.inverse();

  // 将Eigen矩阵转换为列主序数组，供GPU渲染器使用
  // OpenGL/CUDA使用列主序存储，与Eigen的默认列主序一致
  double pose[4 * 4];
  for(int i = 0; i < 4; i ++)
    for(int j = 0; j < 4; j ++)
      pose[j + 4 * i] = cam_pose(i, j);

  // 调用GPU渲染器生成深度图（结果存储在depth_hostptr中）
  // 渲染器会将点云投影到图像平面，计算每个像素的最小深度值
  depthrender.render_pose(pose, depth_hostptr);
  //depthrender.render_pose(cam_pose, depth_hostptr);

  // 初始化深度图矩阵（单通道浮点型）
  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);
  double min = 0.5;  // 最小深度阈值（米）
  double max = 1.0f; // 最大深度值（用于归一化）

  // 将深度数据从整型转换为浮点型（单位：米）
  for(int i = 0; i < height; i++)
  	for(int j = 0; j < width; j++)
  	{
  		// GPU渲染器输出的深度值单位是毫米，需要除以1000转为米
  		float depth = (float)depth_hostptr[i * width + j] / 1000.0f;
  		// 限制最大深度为500米，超出则视为无效（置0）
  		depth = depth < 500.0f ? depth : 0;
  		max = depth > max ? depth : max;  // 记录最大深度值，用于后续归一化
  		depth_mat.at<float>(i,j) = depth;
  	}
  //ROS_INFO("render cost %lf ms.", (ros::Time::now().toSec() - this_time) * 1000.0f);
  //printf("max_depth %lf.\n", max);

  // ============== 发布原始深度图（32位浮点型） ==============
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = last_odom_stamp;      // 使用里程计时间戳
  out_msg.header.frame_id = "camera";          // 参考坐标系为相机坐标系
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;  // 单通道浮点型编码
  out_msg.image = depth_mat.clone();           // 深度图数据（单位：米）
  pub_depth.publish(out_msg.toImageMsg());

  // ============== 生成彩色深度图用于可视化 ==============
  cv::Mat adjMap;
  // 将深度值从浮点型归一化到0-255的8位无符号整型
  // 假设最大深度为13米，超出部分会被截断
  // depth_mat.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);  // 根据实际最大值归一化
  depth_mat.convertTo(adjMap,CV_8UC1, 255 /13.0, -min);  // 固定最大深度13米

  // 应用彩虹色彩映射（RAINBOW colormap）
  // 将灰度图转换为伪彩色图：蓝色(近) -> 绿色 -> 黄色 -> 红色(远)
  cv::Mat falseColorsMap;
  cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);

  // 发布彩色深度图用于RViz可视化
  cv_bridge::CvImage cv_image_colored;
  cv_image_colored.header.frame_id = "depthmap";  // 参考坐标系
  cv_image_colored.header.stamp = last_odom_stamp;  // 时间戳
  cv_image_colored.encoding = sensor_msgs::image_encodings::BGR8;  // BGR格式
  cv_image_colored.image = falseColorsMap;  // 彩色深度图
  pub_color.publish(cv_image_colored.toImageMsg());

  // 可选：在本地窗口显示深度图（需要取消注释main函数中的cv::namedWindow）
  //cv::imshow("depth_image", adjMap);
}

/**
 * @brief 主函数 - 深度传感器模拟节点初始化和主循环
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出码
 *
 * 主要流程：
 * 1. 初始化ROS节点和参数
 * 2. 配置相机内参和深度渲染器
 * 3. 设置相机到机体的固定变换
 * 4. 创建订阅者和发布者
 * 5. 启动定时器进行周期性渲染
 * 6. 进入ROS主循环
 */
int main(int argc, char **argv)
{
  // 初始化ROS节点，节点名称为"pcl_render"
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");  // 使用私有命名空间

  // ============== 1. 从参数服务器获取相机参数 ==============
  nh.getParam("cam_width", width);    // 图像宽度（像素）
  nh.getParam("cam_height", height);  // 图像高度（像素）
  nh.getParam("cam_fx", fx);          // 焦距x方向
  nh.getParam("cam_fy", fy);          // 焦距y方向
  nh.getParam("cam_cx", cx);          // 主点x坐标
  nh.getParam("cam_cy", cy);          // 主点y坐标
  nh.getParam("sensing_horizon", sensing_horizon);  // 传感器感知范围（米）
  nh.getParam("sensing_rate",    sensing_rate);     // 传感器更新频率（Hz）
  nh.getParam("estimation_rate", estimation_rate);  // 位姿发布频率（Hz）

  // 获取地图尺寸参数
  nh.getParam("map/x_size",     _x_size);  // 地图X方向尺寸（米）
  nh.getParam("map/y_size",     _y_size);  // 地图Y方向尺寸（米）
  nh.getParam("map/z_size",     _z_size);  // 地图Z方向尺寸（米）

  // ============== 2. 配置GPU深度渲染器参数 ==============
  depthrender.set_para(fx, fy, cx, cy, width, height);

  // ============== 3. 设置相机到机体坐标系的固定变换 ==============
  // 注释：另一种可能的外参配置（来自实际传感器标定）
  // cam02body <<  0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
  //               0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
  //               -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
  //               0.0, 0.0, 0.0, 1.0;

  // 当前使用的变换：相机朝向前方（+Z），右侧为-X，下方为-Y
  // 这对应于标准的相机坐标系（光轴沿+Z）
  cam02body << 0.0, 0.0, 1.0, 0.0,   // 相机Z轴对应机体X轴
              -1.0, 0.0, 0.0, 0.0,   // 相机X轴对应机体-Y轴
               0.0, -1.0,0.0, 0.0,   // 相机Y轴对应机体-Z轴
               0.0, 0.0, 0.0, 1.0;   // 齐次坐标行

  // 初始化相机到世界坐标系的变换为单位矩阵
  cam2world = Matrix4d::Identity();

  // ============== 4. 创建订阅者 ==============
  // 订阅全局静态地图（通常只接收一次）
  global_map_sub = nh.subscribe( "global_map", 1,  rcvGlobalPointCloudCallBack);
  // 订阅局部动态地图（持续更新）
  local_map_sub  = nh.subscribe( "local_map",  1,  rcvLocalPointCloudCallBack);
  // 订阅无人机里程计信息（高频更新，队列大小50）
  odom_sub       = nh.subscribe( "odometry",   50, rcvOdometryCallbck   );

  // ============== 5. 创建发布者 ==============
  pub_depth = nh.advertise<sensor_msgs::Image>("depth",1000);  // 发布原始深度图
  pub_color = nh.advertise<sensor_msgs::Image>("colordepth",1000);  // 发布彩色可视化深度图
  pub_pose  = nh.advertise<geometry_msgs::PoseStamped>("camera_pose",1000);  // 发布相机位姿
  pub_pcl_wolrd = nh.advertise<sensor_msgs::PointCloud2>("rendered_pcl",1);  // 发布渲染的世界坐标系点云

  // ============== 6. 计算定时器周期 ==============
  double sensing_duration  = 1.0 / sensing_rate;     // 深度渲染周期
  double estimate_duration = 1.0 / estimation_rate;  // 位姿发布周期

  // ============== 7. 创建定时器 ==============
  // 定时器1：周期性渲染深度图和点云
  local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration),  renderSensedPoints);
  // 定时器2：周期性发布相机位姿
  estimation_timer    = nh.createTimer(ros::Duration(estimate_duration), pubCameraPose);

  // 可选：创建OpenCV窗口用于调试
  //cv::namedWindow("depth_image",1);

  // ============== 8. 计算地图栅格参数 ==============
  _inv_resolution = 1.0 / _resolution;  // 分辨率的倒数，用于加速计算

  // 地图原点坐标（地图中心为原点）
  _gl_xl = -_x_size/2.0;  // X方向起始坐标
  _gl_yl = -_y_size/2.0;  // Y方向起始坐标
  _gl_zl =   0.0;         // Z方向起始坐标（地面为0）

  // 计算地图栅格数量
  _GLX_SIZE = (int)(_x_size * _inv_resolution);  // X方向栅格数
  _GLY_SIZE = (int)(_y_size * _inv_resolution);  // Y方向栅格数
  _GLZ_SIZE = (int)(_z_size * _inv_resolution);  // Z方向栅格数

  // ============== 9. ROS主循环 ==============
  ros::Rate rate(100);  // 设置循环频率为100Hz
  bool status = ros::ok();
  while(status)
  {
    ros::spinOnce();  // 处理回调函数
    status = ros::ok();  // 检查ROS运行状态
    rate.sleep();  // 休眠以维持循环频率
  }
}
