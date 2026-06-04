/**
 * @file pcl_render_node.cpp
 * @brief 基于CUDA的点云深度渲染节点
 *
 * 该节点实现了从点云地图生成深度图像的功能，用于UAV的局部感知系统。
 * 主要功能：
 * 1. 接收全局/局部点云地图
 * 2. 接收无人机里程计信息
 * 3. 使用CUDA加速将点云投影到虚拟相机平面
 * 4. 生成深度图像和彩色深度图
 * 5. 发布相机位姿和渲染后的点云
 */

#include <iostream>
#include <fstream>
#include <vector>
//include ros dep. (包含ROS相关依赖)
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
//include pcl dep (包含PCL点云库依赖)
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
 #include <pcl_conversions/pcl_conversions.h>
//include opencv and eigen (包含OpenCV和Eigen依赖)
#include <Eigen/Eigen>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

//#include <cloud_banchmark/cloud_banchmarkConfig.h>
#include "depth_render.cuh"  // CUDA深度渲染器头文件
#include "quadrotor_msgs/PositionCommand.h"
using namespace cv;
using namespace std;
using namespace Eigen;

/* ==================== 全局变量定义 ==================== */

// 深度图相关
int *depth_hostptr;      // 深度图主机端指针，用于从GPU传回深度数据
cv::Mat depth_mat;       // 深度图矩阵，存储渲染后的深度值

// 相机内参
int width, height;       // 图像宽度和高度（像素）
double fx, fy, cx, cy;   // 相机内参：焦距(fx, fy)和光心(cx, cy)

// CUDA深度渲染器
DepthRender depthrender; // 使用GPU加速的深度渲染器

// ROS发布器
ros::Publisher pub_depth;       // 发布深度图像
ros::Publisher pub_color;       // 发布彩色深度图像
ros::Publisher pub_pose;        // 发布相机位姿
ros::Publisher pub_pcl_wolrd;   // 发布渲染后的世界坐标系点云

// 点云消息
sensor_msgs::PointCloud2 local_map_pcl;    // 局部地图点云消息
sensor_msgs::PointCloud2 local_depth_pcl;  // 深度点云消息

// ROS订阅器
ros::Subscriber odom_sub;                  // 里程计订阅器
ros::Subscriber global_map_sub, local_map_sub;  // 全局/局部地图订阅器

// 定时器
ros::Timer local_sensing_timer;   // 局部感知定时器，定期渲染传感数据
ros::Timer estimation_timer;      // 估计定时器，定期发布相机位姿

// 状态标志
bool has_global_map(false);  // 是否已接收全局地图
bool has_local_map(false);   // 是否已接收局部地图
bool has_odom(false);        // 是否已接收里程计数据

// 坐标变换
Matrix4d cam02body;              // 相机坐标系到机体坐标系的变换矩阵
Matrix4d cam2world;              // 相机坐标系到世界坐标系的变换矩阵
Eigen::Quaterniond cam2world_quat;  // 相机到世界坐标系的四元数表示
nav_msgs::Odometry _odom;        // 最新的里程计消息

// 传感和地图参数
double sensing_horizon, sensing_rate, estimation_rate;  // 感知范围、感知频率、估计频率
double _x_size, _y_size, _z_size;  // 地图尺寸（米）
double _gl_xl, _gl_yl, _gl_zl;     // 地图左下角坐标
double _resolution, _inv_resolution;  // 地图分辨率及其倒数
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;  // 地图栅格尺寸

// 位姿记录
ros::Time last_odom_stamp = ros::TIME_MAX;  // 上次里程计时间戳
Eigen::Vector3d last_pose_world;            // 上次无人机在世界坐标系中的位置

/* ==================== 函数声明 ==================== */
void render_currentpose();  // 渲染当前位姿的深度图
void render_pcl_world();    // 将深度图转换为世界坐标系点云

/**
 * @brief 将栅格索引转换为世界坐标
 * @param index 三维栅格索引
 * @return 对应的世界坐标（米）
 *
 * 计算公式：坐标 = (索引 + 0.5) * 分辨率 + 左下角偏移
 * 使用0.5是为了取栅格中心点
 */
inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index)
{
    Eigen::Vector3d pt;
    pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
    pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
    pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;

    return pt;
};

/**
 * @brief 将世界坐标转换为栅格索引
 * @param pt 世界坐标点（米）
 * @return 对应的三维栅格索引，结果会被限制在地图范围内
 *
 * 计算公式：索引 = (坐标 - 左下角偏移) * 分辨率倒数
 * 使用min/max确保索引在有效范围[0, SIZE-1]内
 */
inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt)
{
    Eigen::Vector3i idx;
    idx(0) = std::min( std::max( int( (pt(0) - _gl_xl) * _inv_resolution), 0), _GLX_SIZE - 1);
    idx(1) = std::min( std::max( int( (pt(1) - _gl_yl) * _inv_resolution), 0), _GLY_SIZE - 1);
    idx(2) = std::min( std::max( int( (pt(2) - _gl_zl) * _inv_resolution), 0), _GLZ_SIZE - 1);

    return idx;
};

/**
 * @brief 里程计回调函数
 * @param odom 接收到的里程计消息，包含无人机的位置和姿态
 *
 * 功能：
 * 1. 提取无人机机体的位置和姿态
 * 2. 将机体位姿转换为4x4变换矩阵
 * 3. 计算相机在世界坐标系中的位姿（机体位姿 * 相机到机体的变换）
 * 4. 记录时间戳和世界坐标位置
 */
void rcvOdometryCallbck(const nav_msgs::Odometry& odom)
{
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  _odom = odom;
  Matrix4d Pose_receive = Matrix4d::Identity();

  // 提取位置信息
  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = odom.pose.pose.position.x;
  request_position.y() = odom.pose.pose.position.y;
  request_position.z() = odom.pose.pose.position.z;

  // 提取姿态信息（四元数）
  request_pose.x() = odom.pose.pose.orientation.x;
  request_pose.y() = odom.pose.pose.orientation.y;
  request_pose.z() = odom.pose.pose.orientation.z;
  request_pose.w() = odom.pose.pose.orientation.w;

  // 构造机体位姿的4x4变换矩阵 [R | t]
  //                          [0 | 1]
  Pose_receive.block<3,3>(0,0) = request_pose.toRotationMatrix();  // 旋转矩阵部分
  Pose_receive(0,3) = request_position(0);  // 平移向量x
  Pose_receive(1,3) = request_position(1);  // 平移向量y
  Pose_receive(2,3) = request_position(2);  // 平移向量z

  Matrix4d body_pose = Pose_receive;
  // 将机体位姿转换为相机位姿
  // cam2world = T_world_body * T_body_cam
  cam2world = body_pose * cam02body;
  cam2world_quat = cam2world.block<3,3>(0,0);  // 提取旋转矩阵转为四元数

  // 记录时间戳和位置
  last_odom_stamp = odom.header.stamp;
  last_pose_world(0) = odom.pose.pose.position.x;
  last_pose_world(1) = odom.pose.pose.position.y;
  last_pose_world(2) = odom.pose.pose.position.z;

  // 发布TF变换（可选，当前已注释）
  /*static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(cam2world(0,3), cam2world(1,3), cam2world(2,3) ));
  transform.setRotation(tf::Quaternion(cam2world_quat.x(), cam2world_quat.y(), cam2world_quat.z(), cam2world_quat.w()));
  br.sendTransform(tf::StampedTransform(transform, last_odom_stamp, "world", "camera")); //publish transform from world frame to quadrotor frame.*/
}

/**
 * @brief 定时发布相机位姿
 * @param event 定时器事件
 *
 * 功能：以固定频率发布相机在世界坐标系中的位姿
 * 用于可视化或其他模块使用
 */
void pubCameraPose(const ros::TimerEvent & event)
{
  //cout<<"pub cam pose"
  geometry_msgs::PoseStamped camera_pose;
  camera_pose.header = _odom.header;
  camera_pose.header.frame_id = "/map";  // 参考坐标系为地图坐标系

  // 从变换矩阵中提取位置（平移向量）
  camera_pose.pose.position.x = cam2world(0,3);
  camera_pose.pose.position.y = cam2world(1,3);
  camera_pose.pose.position.z = cam2world(2,3);

  // 设置姿态（四元数）
  camera_pose.pose.orientation.w = cam2world_quat.w();
  camera_pose.pose.orientation.x = cam2world_quat.x();
  camera_pose.pose.orientation.y = cam2world_quat.y();
  camera_pose.pose.orientation.z = cam2world_quat.z();

  pub_pose.publish(camera_pose);
}

/**
 * @brief 定时渲染感知点云
 * @param event 定时器事件
 *
 * 功能：
 * 1. 检查是否已接收到地图和里程计数据
 * 2. 调用render_currentpose()渲染当前位姿的深度图
 * 3. 调用render_pcl_world()将深度图转换为世界坐标系点云并发布
 */
void renderSensedPoints(const ros::TimerEvent & event)
{
  //if(! has_global_map || ! has_odom) return;
  // 至少需要全局地图或局部地图之一
  if( !has_global_map && !has_local_map) return;

  // 必须有里程计数据才能知道相机位姿
  if( !has_odom ) return;

  render_currentpose();  // 渲染深度图
  render_pcl_world();    // 转换为点云并发布
}

// 点云数据容器，存储格式为 [x1, y1, z1, x2, y2, z2, ...]
vector<float> cloud_data;

/**
 * @brief 全局点云地图回调函数
 * @param pointcloud_map 接收到的全局点云地图消息
 *
 * 功能：
 * 1. 接收并转换全局点云地图（只接收一次）
 * 2. 将点云坐标提取为浮点数组
 * 3. 传递给CUDA深度渲染器
 * 4. 分配深度图像的主机端内存
 *
 * 注意：全局地图只接收一次，后续消息会被忽略
 */
void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map )
{
  // 如果已有全局地图，直接返回（避免重复加载）
  if(has_global_map)
    return;

  ROS_WARN("Global Pointcloud received..");

  // 加载全局地图
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;

  // 将ROS消息转换为PCL点云格式
  pcl::fromROSMsg(pointcloud_map, cloudIn);

  // 遍历所有点，将坐标提取到cloud_data数组中
  for(int i = 0; i < int(cloudIn.points.size()); i++){
    pt_in = cloudIn.points[i];
    cloud_data.push_back(pt_in.x);
    cloud_data.push_back(pt_in.y);
    cloud_data.push_back(pt_in.z);
  }
  printf("global map has points: %d.\n", (int)cloud_data.size() / 3 );

  // 将点云数据传递给GPU深度渲染器
  depthrender.set_data(cloud_data);

  // 分配深度图像主机端内存（width × height个整数）
  depth_hostptr = (int*) malloc(width * height * sizeof(int));

  has_global_map = true;
}

/**
 * @brief 局部点云地图回调函数
 * @param pointcloud_map 接收到的局部点云地图消息
 *
 * 功能：
 * 1. 接收并转换局部点云地图（可重复接收更新）
 * 2. 将点云坐标提取为浮点数组
 * 3. 传递给CUDA深度渲染器
 * 4. 分配深度图像的主机端内存
 *
 * 注意：与全局地图不同，局部地图会持续更新
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

  // 遍历所有点，将坐标提取到cloud_data数组中
  for(int i = 0; i < int(cloudIn.points.size()); i++){
    pt_in = cloudIn.points[i];
    Eigen::Vector3d pose_pt(pt_in.x, pt_in.y, pt_in.z);
    //pose_pt = gridIndex2coord(coord2gridIndex(pose_pt)); // 可选的栅格化处理
    cloud_data.push_back(pose_pt(0));
    cloud_data.push_back(pose_pt(1));
    cloud_data.push_back(pose_pt(2));
  }
  //printf("local map has points: %d.\n", (int)cloud_data.size() / 3 );

  // 将点云数据传递给GPU深度渲染器
  depthrender.set_data(cloud_data);

  // 分配深度图像主机端内存（width × height个整数）
  depth_hostptr = (int*) malloc(width * height * sizeof(int));

  has_local_map = true;
}

/**
 * @brief 将深度图反投影为世界坐标系点云
 *
 * 功能：
 * 1. 遍历深度图像的每个像素
 * 2. 使用相机内参将像素坐标和深度值转换为相机坐标系中的3D点
 * 3. 通过cam2world变换矩阵转换到世界坐标系
 * 4. 过滤超出感知范围的点
 * 5. 生成并发布世界坐标系点云
 *
 * 坐标变换过程：
 * 像素坐标(u,v,depth) -> 相机坐标 -> 世界坐标
 */
void render_pcl_world()
{
  // 用于调试的局部地图点云
  pcl::PointCloud<pcl::PointXYZ> localMap;
  pcl::PointXYZ pt_in;

  Eigen::Vector4d pose_in_camera;  // 相机坐标系中的点（齐次坐标）
  Eigen::Vector4d pose_in_world;   // 世界坐标系中的点（齐次坐标）
  Eigen::Vector3d pose_pt;         // 最终的3D点坐标

  // 遍历深度图像的每个像素
  for(int u = 0; u < width; u++)
    for(int v = 0; v < height; v++){
      float depth = depth_mat.at<float>(v,u);

      // 跳过无效深度值
      if(depth == 0.0)
        continue;

      // 使用针孔相机模型将像素坐标反投影到相机坐标系
      // X_cam = (u - cx) * Z / fx
      // Y_cam = (v - cy) * Z / fy
      // Z_cam = Z
      pose_in_camera(0) = (u - cx) * depth / fx;
      pose_in_camera(1) = (v - cy) * depth / fy;
      pose_in_camera(2) = depth;
      pose_in_camera(3) = 1.0;  // 齐次坐标

      // 将相机坐标系点转换到世界坐标系
      pose_in_world = cam2world * pose_in_camera;

      // 过滤掉超出感知范围的点
      if( (pose_in_world.segment(0,3) - last_pose_world).norm() > sensing_horizon )
          continue;

      // 提取3D坐标
      pose_pt = pose_in_world.head(3);
      //pose_pt = gridIndex2coord(coord2gridIndex(pose_pt)); // 可选的栅格化处理

      // 添加到点云中
      pt_in.x = pose_pt(0);
      pt_in.y = pose_pt(1);
      pt_in.z = pose_pt(2);
      localMap.points.push_back(pt_in);
    }

  // 设置点云属性
  localMap.width = localMap.points.size();
  localMap.height = 1;  // 无序点云
  localMap.is_dense = true;

  // 转换为ROS消息格式
  pcl::toROSMsg(localMap, local_map_pcl);
  local_map_pcl.header.frame_id  = "/map";
  local_map_pcl.header.stamp     = last_odom_stamp;

  // 发布点云
  pub_pcl_wolrd.publish(local_map_pcl);
}

/**
 * @brief 渲染当前相机位姿的深度图
 *
 * 功能：
 * 1. 计算世界坐标系到相机坐标系的变换矩阵（cam2world的逆）
 * 2. 调用CUDA渲染器生成深度图
 * 3. 处理深度值（单位转换、范围限制）
 * 4. 发布原始深度图（32位浮点）
 * 5. 生成并发布彩色深度图（用于可视化）
 *
 * 深度图处理流程：
 * GPU渲染 -> 主机内存 -> OpenCV矩阵 -> ROS消息
 */
void render_currentpose()
{
  double this_time = ros::Time::now().toSec();

  // 计算相机位姿的逆变换（世界坐标系到相机坐标系）
  Matrix4d cam_pose = cam2world.inverse();

  // 将Eigen矩阵转换为数组格式供CUDA使用
  // 转换为列优先存储
  double pose[4 * 4];
  for(int i = 0; i < 4; i ++)
    for(int j = 0; j < 4; j ++)
      pose[j + 4 * i] = cam_pose(i, j);

  // 调用CUDA渲染器生成深度图
  depthrender.render_pose(pose, depth_hostptr);
  //depthrender.render_pose(cam_pose, depth_hostptr);

  // 初始化深度图矩阵（32位浮点，单通道）
  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);
  double min = 0.5;
  double max = 1.0f;

  // 处理深度值：从整数转换为浮点米单位
  for(int i = 0; i < height; i++)
  	for(int j = 0; j < width; j++)
  	{
  		// 深度值从毫米转换为米（除以1000）
  		float depth = (float)depth_hostptr[i * width + j] / 1000.0f;

  		// 限制最大深度为500米，超过则视为无效
  		depth = depth < 500.0f ? depth : 0;

  		// 更新最大深度值
  		max = depth > max ? depth : max;

  		// 存储到深度矩阵
  		depth_mat.at<float>(i,j) = depth;
  	}
  //ROS_INFO("render cost %lf ms.", (ros::Time::now().toSec() - this_time) * 1000.0f);
  //printf("max_depth %lf.\n", max);

  // 发布原始深度图（32位浮点）
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = last_odom_stamp;
  out_msg.header.frame_id = "camera";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_mat.clone();
  pub_depth.publish(out_msg.toImageMsg());

  // 生成彩色深度图用于可视化
  cv::Mat adjMap;
  // 将深度值归一化到0-255范围
  // depth_mat.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);
  depth_mat.convertTo(adjMap,CV_8UC1, 255 /13.0, -min);

  // 应用彩虹色彩映射
  cv::Mat falseColorsMap;
  cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);

  // 发布彩色深度图
  cv_bridge::CvImage cv_image_colored;
  cv_image_colored.header.frame_id = "depthmap";
  cv_image_colored.header.stamp = last_odom_stamp;
  cv_image_colored.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image_colored.image = falseColorsMap;
  pub_color.publish(cv_image_colored.toImageMsg());
  //cv::imshow("depth_image", adjMap);  // 可选的OpenCV窗口显示
}

/**
 * @brief 主函数 - PCL渲染节点入口
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 *
 * 功能：
 * 1. 初始化ROS节点
 * 2. 加载相机参数和地图参数
 * 3. 设置相机到机体的固定变换
 * 4. 创建订阅器和发布器
 * 5. 创建定时器定期渲染和发布数据
 * 6. 进入ROS主循环
 */
int main(int argc, char **argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  // 从参数服务器加载相机内参
  nh.getParam("cam_width", width);    // 图像宽度
  nh.getParam("cam_height", height);  // 图像高度
  nh.getParam("cam_fx", fx);          // 焦距x
  nh.getParam("cam_fy", fy);          // 焦距y
  nh.getParam("cam_cx", cx);          // 光心x
  nh.getParam("cam_cy", cy);          // 光心y

  // 加载感知参数
  nh.getParam("sensing_horizon", sensing_horizon);  // 感知范围（米）
  nh.getParam("sensing_rate",    sensing_rate);     // 感知频率（Hz）
  nh.getParam("estimation_rate", estimation_rate);  // 估计频率（Hz）

  // 加载地图尺寸参数
  nh.getParam("map/x_size", _x_size);
  nh.getParam("map/y_size", _y_size);
  nh.getParam("map/z_size", _z_size);

  // 设置深度渲染器参数
  depthrender.set_para(fx, fy, cx, cy, width, height);

  // 设置相机到机体的变换矩阵
  // 注释掉的是实际相机标定的外参矩阵
  // cam02body <<  0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
  //               0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
  //               -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
  //               0.0, 0.0, 0.0, 1.0;

  // 使用简化的变换矩阵（相机朝前，Z轴向前，Y轴向下）
  cam02body << 0.0,  0.0, 1.0, 0.0,
              -1.0,  0.0, 0.0, 0.0,
               0.0, -1.0, 0.0, 0.0,
               0.0,  0.0, 0.0, 1.0;

  // 初始化cam2world变换为单位矩阵
  cam2world = Matrix4d::Identity();

  // 订阅点云和里程计话题
  global_map_sub = nh.subscribe( "global_map", 1,  rcvGlobalPointCloudCallBack);
  local_map_sub  = nh.subscribe( "local_map",  1,  rcvLocalPointCloudCallBack);
  odom_sub       = nh.subscribe( "odometry",   50, rcvOdometryCallbck   );

  // 创建发布器
  pub_depth = nh.advertise<sensor_msgs::Image>("depth",1000);                     // 深度图
  pub_color = nh.advertise<sensor_msgs::Image>("colordepth",1000);                // 彩色深度图
  pub_pose  = nh.advertise<geometry_msgs::PoseStamped>("camera_pose",1000);       // 相机位姿
  pub_pcl_wolrd = nh.advertise<sensor_msgs::PointCloud2>("rendered_pcl",1);       // 渲染点云

  // 计算定时器周期
  double sensing_duration  = 1.0 / sensing_rate;
  double estimate_duration = 1.0 / estimation_rate;

  // 创建定时器
  local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration),  renderSensedPoints);
  estimation_timer    = nh.createTimer(ros::Duration(estimate_duration), pubCameraPose);
  //cv::namedWindow("depth_image",1);  // 可选的OpenCV显示窗口

  // 计算地图参数
  _inv_resolution = 1.0 / _resolution;  // 分辨率倒数用于快速计算

  // 设置地图左下角坐标（地图中心在原点）
  _gl_xl = -_x_size/2.0;
  _gl_yl = -_y_size/2.0;
  _gl_zl =   0.0;

  // 计算地图栅格尺寸
  _GLX_SIZE = (int)(_x_size * _inv_resolution);
  _GLY_SIZE = (int)(_y_size * _inv_resolution);
  _GLZ_SIZE = (int)(_z_size * _inv_resolution);

  // ROS主循环
  ros::Rate rate(100);  // 100Hz循环频率
  bool status = ros::ok();
  while(status)
  {
    ros::spinOnce();  // 处理回调函数
    status = ros::ok();
    rate.sleep();
  }
}
