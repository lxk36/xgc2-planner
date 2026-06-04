/**
 * @file pointcloud_render_node.cpp
 * @brief 点云渲染节点 - 模拟传感器感知范围内的点云数据
 * @details 该节点从全局地图中提取无人机当前传感器视野范围内的点云，
 *          模拟真实传感器（如深度相机、激光雷达）的感知效果。
 *          主要功能包括：
 *          1. 接收全局点云地图并进行体素化降采样
 *          2. 根据无人机位姿实时提取局部可见点云
 *          3. 应用传感器视场角和距离限制
 *          4. 发布模拟的局部感知点云
 */

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

// ==================== ROS通信接口 ====================
ros::Publisher pub_cloud;  // 发布局部感知点云的发布者

sensor_msgs::PointCloud2 local_map_pcl;    // 局部地图点云消息
sensor_msgs::PointCloud2 local_depth_pcl;  // 深度点云消息（预留）

ros::Subscriber odom_sub;                   // 里程计订阅者
ros::Subscriber global_map_sub, local_map_sub;  // 全局和局部地图订阅者

ros::Timer local_sensing_timer;  // 定时器，周期性触发传感器感知

// ==================== 状态标志位 ====================
bool has_global_map(false);  // 是否已接收到全局地图
bool has_local_map(false);   // 是否已接收到局部地图
bool has_odom(false);        // 是否已接收到里程计数据

// ==================== 无人机状态数据 ====================
nav_msgs::Odometry _odom;  // 当前无人机里程计信息（位置、姿态、速度等）

// ==================== 传感器参数 ====================
double sensing_horizon;    // 传感器感知距离（米）
double sensing_rate;       // 传感器感知频率（Hz）
double estimation_rate;    // 状态估计频率（Hz）

// ==================== 地图参数 ====================
double _x_size, _y_size, _z_size;  // 地图在x、y、z方向的尺寸（米）
double _gl_xl, _gl_yl, _gl_zl;     // 地图左下角的全局坐标（地图原点偏移）
double _resolution, _inv_resolution;  // 地图分辨率及其倒数（米/栅格）
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;  // 地图在x、y、z方向的栅格数量

// ==================== 时间戳 ====================
ros::Time last_odom_stamp = ros::TIME_MAX;  // 最后一次里程计时间戳

/**
 * @brief 栅格索引转换为世界坐标
 * @param index 三维栅格索引 (ix, iy, iz)
 * @return 对应的世界坐标 (x, y, z)
 * @details 将离散的栅格索引转换为连续的世界坐标系中的点。
 *          转换公式：coord = (index + 0.5) * resolution + origin
 *          其中 +0.5 是为了取栅格中心点的坐标
 */
inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index) {
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;  // x坐标
  pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;  // y坐标
  pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;  // z坐标

  return pt;
};

/**
 * @brief 世界坐标转换为栅格索引
 * @param pt 世界坐标系中的点 (x, y, z)
 * @return 对应的栅格索引 (ix, iy, iz)
 * @details 将连续的世界坐标转换为离散的栅格索引。
 *          转换公式：index = (coord - origin) / resolution
 *          使用 min/max 进行边界检查，确保索引在有效范围 [0, SIZE-1] 内
 */
inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt) {
  Eigen::Vector3i idx;
  // x方向索引，限制在 [0, _GLX_SIZE-1] 范围内
  idx(0) = std::min(std::max(int((pt(0) - _gl_xl) * _inv_resolution), 0),
                    _GLX_SIZE - 1);
  // y方向索引，限制在 [0, _GLY_SIZE-1] 范围内
  idx(1) = std::min(std::max(int((pt(1) - _gl_yl) * _inv_resolution), 0),
                    _GLY_SIZE - 1);
  // z方向索引，限制在 [0, _GLZ_SIZE-1] 范围内
  idx(2) = std::min(std::max(int((pt(2) - _gl_zl) * _inv_resolution), 0),
                    _GLZ_SIZE - 1);

  return idx;
};

/**
 * @brief 里程计数据回调函数
 * @param odom 接收到的里程计消息，包含无人机的位置、姿态、速度等信息
 * @details 更新全局里程计数据，用于后续的局部点云提取。
 *          里程计提供无人机在世界坐标系中的位姿信息。
 */
void rcvOdometryCallbck(const nav_msgs::Odometry& odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;  // 标记已接收到里程计数据
  _odom = odom;     // 保存最新的里程计数据
}

// ==================== 点云数据结构 ====================
pcl::PointCloud<pcl::PointXYZ> _cloud_all_map;  // 存储全局地图点云（经过体素化降采样）
pcl::PointCloud<pcl::PointXYZ> _local_map;      // 存储提取的局部可见点云
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;   // 体素化降采样器，用于减少点云密度
sensor_msgs::PointCloud2 _local_map_pcd;        // 局部点云的ROS消息格式

// ==================== K-D树搜索相关 ====================
pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;  // K-D树，用于快速半径搜索
vector<int> _pointIdxRadiusSearch;                   // 半径搜索结果：点的索引列表
vector<float> _pointRadiusSquaredDistance;           // 半径搜索结果：距离的平方列表

/**
 * @brief 全局点云地图回调函数
 * @param pointcloud_map 接收到的全局点云地图消息
 * @details 处理全局点云地图的主要步骤：
 *          1. 检查是否已接收过地图（只接收一次）
 *          2. 将ROS消息格式转换为PCL点云格式
 *          3. 使用体素化降采样减少点云密度（叶子大小0.1m）
 *          4. 构建K-D树用于后续的快速近邻搜索
 */
void rcvGlobalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  if (has_global_map) return;  // 如果已经接收过地图，则直接返回

  ROS_WARN("Global Pointcloud received..");

  // 将ROS消息转换为PCL点云格式
  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);

  // 体素化降采样：将点云划分为0.1m×0.1m×0.1m的体素，每个体素只保留一个点
  // 目的是减少点云数据量，提高后续处理效率
  _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(_cloud_all_map);

  // 构建K-D树索引，用于快速半径搜索
  _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());

  has_global_map = true;  // 标记已成功接收并处理全局地图
}

/**
 * @brief 渲染传感器感知范围内的点云（定时器回调函数）
 * @param event ROS定时器事件
 * @details 该函数模拟传感器的感知过程，主要步骤：
 *          1. 检查数据是否就绪（全局地图和里程计）
 *          2. 提取无人机当前位姿信息
 *          3. 使用K-D树进行半径搜索，找出感知范围内的所有点
 *          4. 应用传感器视场角限制（俯仰角和偏航角）
 *          5. 发布过滤后的局部点云
 *
 * 传感器模型：
 * - 感知距离：sensing_horizon（球形范围）
 * - 俯仰角限制：±30度（tan(π/6)）
 * - 偏航角限制：前向视野，cos(θ) > 0.5 即 θ < 60度
 */
void renderSensedPoints(const ros::TimerEvent& event) {
  // 检查必要数据是否已就绪
  if (!has_global_map || !has_odom) return;

  // ==================== 提取无人机姿态信息 ====================
  // 从里程计中提取四元数姿态
  Eigen::Quaterniond q;
  q.x() = _odom.pose.pose.orientation.x;
  q.y() = _odom.pose.pose.orientation.y;
  q.z() = _odom.pose.pose.orientation.z;
  q.w() = _odom.pose.pose.orientation.w;

  // 将四元数转换为旋转矩阵
  Eigen::Matrix3d rot;
  rot = q;
  // 提取机体坐标系的x轴方向（前向方向），用于判断点是否在视野内
  Eigen::Vector3d yaw_vec = rot.col(0);

  // ==================== K-D树半径搜索 ====================
  _local_map.points.clear();  // 清空上一次的局部点云

  // 设置搜索中心点为无人机当前位置
  pcl::PointXYZ searchPoint(_odom.pose.pose.position.x,
                            _odom.pose.pose.position.y,
                            _odom.pose.pose.position.z);
  _pointIdxRadiusSearch.clear();        // 清空上次的搜索结果索引
  _pointRadiusSquaredDistance.clear();  // 清空上次的距离结果

  pcl::PointXYZ pt;
  // 在K-D树中进行半径搜索，找出距离小于sensing_horizon的所有点
  if (_kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon,
                                   _pointIdxRadiusSearch,
                                   _pointRadiusSquaredDistance) > 0) {
    // ==================== 遍历搜索结果并应用视场角过滤 ====================
    for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i) {
      pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];

      // ========== 俯仰角过滤 ==========
      // 计算点相对于无人机的高度差与感知距离的比值
      // 如果该比值大于tan(30度)，说明俯仰角超出范围，剔除该点
      // if ((fabs(pt.z - _odom.pose.pose.position.z) / (pt.x - _odom.pose.pose.position.x)) >
      //     tan(M_PI / 12.0))  // 原注释代码：15度限制
      //   continue;
      if ((fabs(pt.z - _odom.pose.pose.position.z) / sensing_horizon) >
          tan(M_PI / 6.0))  // 30度俯仰角限制
        continue;

      // ========== 偏航角过滤 ==========
      // 计算点相对于无人机的方向向量
      Vector3d pt_vec(pt.x - _odom.pose.pose.position.x,
                      pt.y - _odom.pose.pose.position.y,
                      pt.z - _odom.pose.pose.position.z);

      // 计算方向向量与机体前向的夹角余弦值
      // 如果cos(θ) < 0.5，即θ > 60度，说明点不在前向视野内，剔除
      if (pt_vec.normalized().dot(yaw_vec) < 0.5) continue;

      // 通过所有过滤条件的点加入局部地图
      _local_map.points.push_back(pt);
    }
  } else {
    // 如果半径搜索没有找到任何点，直接返回
    return;
  }

  // ==================== 设置点云属性并发布 ====================
  _local_map.width = _local_map.points.size();  // 点云宽度（点的总数）
  _local_map.height = 1;                         // 点云高度（无序点云设为1）
  _local_map.is_dense = true;                    // 点云是否密集（无NaN点）

  // 将PCL点云转换为ROS消息格式
  pcl::toROSMsg(_local_map, _local_map_pcd);
  _local_map_pcd.header.frame_id = "map";  // 设置坐标系为map

  // 发布局部感知点云
  pub_cloud.publish(_local_map_pcd);
}

/**
 * @brief 局部点云地图回调函数
 * @param pointcloud_map 接收到的局部点云地图消息
 * @details 当前未实现，预留接口供后续扩展
 */
void rcvLocalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  // do nothing, fix later
  // TODO: 实现局部点云地图的处理逻辑
}

/**
 * @brief 主函数 - 点云渲染节点入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 * @details 主要功能：
 *          1. 初始化ROS节点
 *          2. 加载参数（传感器参数、地图参数）
 *          3. 设置订阅者和发布者
 *          4. 创建定时器周期性渲染传感器点云
 *          5. 进入ROS事件循环
 */
int main(int argc, char** argv) {
  // ==================== ROS节点初始化 ====================
  ros::init(argc, argv, "pcl_render");  // 初始化节点，名称为"pcl_render"
  ros::NodeHandle nh("~");               // 创建私有命名空间的节点句柄

  // ==================== 加载传感器参数 ====================
  nh.getParam("sensing_horizon", sensing_horizon);  // 传感器感知距离
  nh.getParam("sensing_rate", sensing_rate);        // 传感器感知频率
  nh.getParam("estimation_rate", estimation_rate);  // 状态估计频率

  // ==================== 加载地图参数 ====================
  nh.getParam("map/x_size", _x_size);  // 地图x方向尺寸
  nh.getParam("map/y_size", _y_size);  // 地图y方向尺寸
  nh.getParam("map/z_size", _z_size);  // 地图z方向尺寸

  // ==================== 设置订阅者 ====================
  // 订阅全局点云地图（队列长度1，只保留最新消息）
  global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  // 订阅局部点云地图（当前未使用）
  local_map_sub = nh.subscribe("local_map", 1, rcvLocalPointCloudCallBack);
  // 订阅里程计信息（队列长度50，确保不丢失）
  odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);

  // ==================== 设置发布者 ====================
  // 发布局部感知点云（队列长度10）
  pub_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/cloud", 10);

  // ==================== 创建定时器 ====================
  // 计算感知周期：2.5倍的感知周期作为定时器触发间隔
  double sensing_duration = 1.0 / sensing_rate * 2.5;
  // 创建定时器，周期性调用renderSensedPoints函数
  local_sensing_timer =
      nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);

  // ==================== 初始化地图参数 ====================
  _inv_resolution = 1.0 / _resolution;  // 计算分辨率的倒数

  // 设置地图原点（地图中心在世界坐标系的位置）
  _gl_xl = -_x_size / 2.0;  // x方向原点
  _gl_yl = -_y_size / 2.0;  // y方向原点
  _gl_zl = 0.0;             // z方向原点（地面）

  // 计算地图的栅格数量
  _GLX_SIZE = (int)(_x_size * _inv_resolution);
  _GLY_SIZE = (int)(_y_size * _inv_resolution);
  _GLZ_SIZE = (int)(_z_size * _inv_resolution);

  // ==================== 进入ROS事件循环 ====================
  ros::Rate rate(100);  // 设置循环频率为100Hz
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();      // 处理一次回调函数
    status = ros::ok();   // 检查节点状态
    rate.sleep();         // 按照设定频率休眠
  }
}
