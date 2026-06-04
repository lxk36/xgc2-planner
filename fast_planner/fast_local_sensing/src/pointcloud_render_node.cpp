/**
 * @file pointcloud_render_node.cpp
 * @brief 点云渲染节点 - 模拟传感器在局部范围内对全局点云地图的感知
 *
 * 该节点的主要功能：
 * 1. 接收全局点云地图
 * 2. 根据无人机当前位置和姿态，提取感知范围内的点云
 * 3. 模拟传感器的视场角限制（FOV限制）
 * 4. 发布局部可见点云，用于规划和避障
 *
 * 应用场景：
 * - 无人机仿真环境中的传感器模拟
 * - 局部地图构建
 * - 避障规划的输入数据生成
 */

// ========== ROS消息类型 ==========
#include <nav_msgs/Odometry.h>        // 里程计消息，包含位置和姿态
#include <nav_msgs/Path.h>            // 路径消息（当前未使用）
#include <sensor_msgs/PointCloud2.h>  // 点云消息（ROS标准格式）
#include <ros/ros.h>                  // ROS核心库

// ========== PCL点云处理库 ==========
#include <pcl/filters/voxel_grid.h>      // 体素降采样滤波器
#include <pcl/kdtree/kdtree_flann.h>     // FLANN算法的KD树实现
#include <pcl/point_cloud.h>             // 点云数据结构
#include <pcl/point_types.h>             // 点云类型定义（PointXYZ等）
#include <pcl/search/kdtree.h>           // KD树搜索接口
#include <pcl/search/impl/kdtree.hpp>    // KD树实现（模板类）
#include <pcl_conversions/pcl_conversions.h> // PCL与ROS消息转换

// ========== 数学和工具库 ==========
#include <Eigen/Dense>  // Eigen矩阵运算库，用于三维坐标变换
#include <fstream>      // 文件流（当前未使用）
#include <iostream>     // 标准输入输出
#include <vector>       // 标准向量容器

using namespace std;
using namespace Eigen;

// ========== ROS Publishers ==========
ros::Publisher pub_cloud;  // 发布局部感知点云

// ========== 点云数据 ==========
sensor_msgs::PointCloud2 local_map_pcl;   // 局部地图点云
sensor_msgs::PointCloud2 local_depth_pcl; // 深度点云（当前未使用）

// ========== ROS Subscribers ==========
ros::Subscriber odom_sub;                  // 订阅里程计信息
ros::Subscriber global_map_sub, local_map_sub; // 订阅全局/局部地图

// ========== 定时器 ==========
ros::Timer local_sensing_timer;  // 局部感知定时器，周期性提取可见点云

// ========== 状态标志 ==========
bool has_global_map(false);  // 是否已接收全局地图
bool has_local_map(false);   // 是否已接收局部地图
bool has_odom(false);        // 是否已接收里程计数据

// ========== 里程计数据 ==========
nav_msgs::Odometry _odom;  // 当前无人机位姿

// ========== 传感器参数 ==========
double sensing_horizon;   // 感知距离（m）
double sensing_rate;      // 感知频率（Hz）
double estimation_rate;   // 估计频率（Hz）

// ========== 地图参数 ==========
double _x_size, _y_size, _z_size;           // 地图尺寸（m）
double _gl_xl, _gl_yl, _gl_zl;              // 地图起始坐标（左下角）
double _resolution, _inv_resolution;        // 地图分辨率及其倒数
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;        // 地图栅格数量

// ========== 时间戳 ==========
ros::Time last_odom_stamp = ros::TIME_MAX;  // 上次里程计时间戳

/**
 * @brief 将栅格索引转换为三维坐标
 *
 * @param index 栅格索引 (i, j, k)
 * @return Eigen::Vector3d 对应的三维世界坐标 (x, y, z)
 *
 * 说明：栅格中心点坐标 = (索引 + 0.5) × 分辨率 + 地图起始坐标
 *
 * 示例：
 * 假设 _resolution = 0.1m, _gl_xl = -5.0m
 * 索引 i=0 对应 x = (0 + 0.5) × 0.1 + (-5.0) = -4.95m（第一个栅格的中心）
 * 索引 i=10 对应 x = (10 + 0.5) × 0.1 + (-5.0) = -3.95m
 *
 * 注意：+0.5是为了取栅格中心点而非左下角点
 */
inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index) {
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;  // x坐标
  pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;  // y坐标
  pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;  // z坐标

  return pt;
};

/**
 * @brief 将三维坐标转换为栅格索引
 *
 * @param pt 三维世界坐标 (x, y, z)
 * @return Eigen::Vector3i 对应的栅格索引 (i, j, k)
 *
 * 说明：
 * 1. 索引计算：(坐标 - 起始坐标) / 分辨率
 * 2. 边界限制：确保索引在 [0, SIZE-1] 范围内
 *
 * 示例：
 * 假设 _resolution = 0.1m, _inv_resolution = 10, _gl_xl = -5.0m, _GLX_SIZE = 100
 * 坐标 x = -4.95m 对应 i = (-4.95 - (-5.0)) × 10 = 0（第一个栅格）
 * 坐标 x = -3.95m 对应 i = (-3.95 - (-5.0)) × 10 = 10
 * 坐标 x = 6.0m（超出范围）对应 i = min(110, 99) = 99（边界限制）
 *
 * 注意：使用_inv_resolution避免除法运算，提高计算效率
 */
inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt) {
  Eigen::Vector3i idx;
  // x方向索引，限制在 [0, _GLX_SIZE-1]
  idx(0) = std::min(std::max(int((pt(0) - _gl_xl) * _inv_resolution), 0),
                    _GLX_SIZE - 1);
  // y方向索引，限制在 [0, _GLY_SIZE-1]
  idx(1) = std::min(std::max(int((pt(1) - _gl_yl) * _inv_resolution), 0),
                    _GLY_SIZE - 1);
  // z方向索引，限制在 [0, _GLZ_SIZE-1]
  idx(2) = std::min(std::max(int((pt(2) - _gl_zl) * _inv_resolution), 0),
                    _GLZ_SIZE - 1);

  return idx;
};

/**
 * @brief 里程计消息回调函数
 *
 * @param odom 接收到的里程计消息，包含无人机的位置和姿态信息
 *
 * 功能：更新无人机当前位姿，用于后续的局部点云提取
 *
 * 注意：注释掉的代码原本用于检查全局地图是否已加载，
 *      现在允许在没有全局地图时也能接收里程计信息
 */
void rcvOdometryCallbck(const nav_msgs::Odometry& odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;  // 标记已接收到里程计数据
  _odom = odom;     // 保存里程计信息
}

// ========== PCL点云数据结构 ==========
pcl::PointCloud<pcl::PointXYZ> _cloud_all_map;  // 全局地图点云（体素降采样后）
pcl::PointCloud<pcl::PointXYZ> _local_map;      // 当前感知到的局部点云
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;   // 体素降采样器，用于减少点云密度
sensor_msgs::PointCloud2 _local_map_pcd;        // ROS格式的局部点云消息

// ========== KD树及搜索结果 ==========
pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;  // KD树，用于快速半径搜索
vector<int> _pointIdxRadiusSearch;                   // 半径搜索结果：点的索引
vector<float> _pointRadiusSquaredDistance;           // 半径搜索结果：距离的平方

/**
 * @brief 全局点云地图回调函数
 *
 * @param pointcloud_map 接收到的全局点云地图
 *
 * 功能：
 * 1. 接收全局点云地图（仅接收一次）
 * 2. 进行体素降采样，减少点云密度，提高处理效率
 * 3. 构建KD树，用于后续快速搜索感知范围内的点
 *
 * 说明：体素大小设置为0.1m，可以有效减少点云数量
 */
void rcvGlobalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  if (has_global_map) return;  // 只接收一次全局地图

  ROS_WARN("Global Pointcloud received..");

  // 将ROS消息格式转换为PCL格式
  // ROS使用sensor_msgs::PointCloud2，PCL使用pcl::PointCloud<PointXYZ>
  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(pointcloud_map, cloud_input);

  // 设置体素降采样参数：体素大小为0.1m×0.1m×0.1m
  // 体素降采样原理：将空间划分为0.1m的立方体网格，每个网格内的所有点用中心点代替
  // 作用：减少点云密度，降低计算量，同时保持地图的主要特征
  _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(_cloud_all_map);  // 执行降采样，输出到_cloud_all_map

  // 构建KD树，用于快速半径搜索
  // KD树是一种空间分割数据结构，可以实现O(log n)时间复杂度的最近邻搜索
  _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());

  has_global_map = true;  // 标记已接收全局地图
}

/**
 * @brief 渲染传感器感知点云（定时器回调函数）
 *
 * @param event ROS定时器事件
 *
 * 功能：
 * 1. 以无人机当前位置为中心，提取sensing_horizon范围内的点云
 * 2. 模拟传感器的视场角限制：
 *    - 俯仰角限制：±15度（通过高度差/距离 < tan(15°)实现）
 *    - 前向视场：只保留机体前方的点（通过点向量与yaw方向点积>0判断）
 * 3. 发布局部可见点云
 *
 * 算法流程：
 * 1. 提取无人机位姿（位置+姿态）
 * 2. 计算机体坐标系的前向方向（yaw方向）
 * 3. KD树半径搜索：找出sensing_horizon范围内的所有点
 * 4. 视场角过滤：
 *    - 过滤俯仰角过大的点（模拟相机垂直FOV）
 *    - 过滤机体后方的点（模拟相机水平FOV）
 * 5. 发布过滤后的局部点云
 */
void renderSensedPoints(const ros::TimerEvent& event) {
  // 检查是否已接收全局地图和里程计
  if (!has_global_map || !has_odom) return;

  // ========== 1. 提取无人机姿态信息 ==========
  // 从里程计消息中提取四元数表示的姿态
  Eigen::Quaterniond q;
  q.x() = _odom.pose.pose.orientation.x;  // 四元数虚部x分量
  q.y() = _odom.pose.pose.orientation.y;  // 四元数虚部y分量
  q.z() = _odom.pose.pose.orientation.z;  // 四元数虚部z分量
  q.w() = _odom.pose.pose.orientation.w;  // 四元数实部w分量

  // 将四元数转换为旋转矩阵（3×3矩阵）
  // 旋转矩阵的三列分别表示机体坐标系的x、y、z轴在世界坐标系中的方向
  Eigen::Matrix3d rot;
  rot = q;
  // 提取旋转矩阵的第一列，即机体坐标系x轴方向（前向）
  // 在FLU坐标系中，x轴指向前方
  Eigen::Vector3d yaw_vec = rot.col(0);

  // ========== 2. 准备局部点云和搜索参数 ==========
  _local_map.points.clear();  // 清空上一次的局部点云
  // 设置搜索中心点（无人机当前位置）
  pcl::PointXYZ searchPoint(_odom.pose.pose.position.x,
                            _odom.pose.pose.position.y,
                            _odom.pose.pose.position.z);
  _pointIdxRadiusSearch.clear();        // 清空搜索结果：点的索引数组
  _pointRadiusSquaredDistance.clear();  // 清空搜索结果：距离平方数组

  // ========== 3. KD树半径搜索 ==========
  pcl::PointXYZ pt;
  // 在sensing_horizon半径内搜索所有点
  // radiusSearch返回找到的点的数量
  // 参数：搜索中心点、搜索半径、结果索引数组、结果距离平方数组
  if (_kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon,
                                   _pointIdxRadiusSearch,
                                   _pointRadiusSquaredDistance) > 0) {
    // ========== 4. 视场角过滤 ==========
    // 遍历半径搜索返回的所有点，进行视场角过滤
    for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i) {
      pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];  // 获取点云中的点

      // 过滤1：俯仰角限制
      // 原理：tan(俯仰角) = 高度差 / 水平距离
      // 如果点的高度差与感知距离的比值 > tan(15°)，则过滤掉
      // 这模拟了传感器的垂直视场角约为±15度（共30度）
      // M_PI/12 = 15° = 0.2618 rad, tan(15°) ≈ 0.268
      if ((fabs(pt.z - _odom.pose.pose.position.z) / (sensing_horizon)) >
          tan(M_PI / 12.0))  // M_PI/12 = 15度
        continue;

      // 计算点相对于无人机的向量（从无人机指向该点）
      Vector3d pt_vec(pt.x - _odom.pose.pose.position.x,
                      pt.y - _odom.pose.pose.position.y,
                      pt.z - _odom.pose.pose.position.z);

      // 过滤2：前向视场限制
      // 原理：点积 = |a| * |b| * cos(夹角)
      // 如果点向量与机体前向方向的点积 < 0，说明夹角 > 90度，点在后方
      // 这模拟了传感器只能看到前方180度范围（前半球）
      if (pt_vec.dot(yaw_vec) < 0) continue;

      // 通过所有过滤条件，添加到局部地图
      _local_map.points.push_back(pt);
    }
  } else {
    // 半径搜索失败或没有找到点
    return;
  }

  // ========== 5. 设置点云属性 ==========
  _local_map.width = _local_map.points.size();  // 点云宽度（点的数量）
  _local_map.height = 1;                        // 点云高度（非结构化点云为1）
  // 点云类型说明：
  // - 结构化点云（organized）：width×height = 总点数，类似图像
  // - 非结构化点云（unorganized）：height=1，width=总点数
  _local_map.is_dense = true;                   // 点云是否稠密（无NaN点）

  // ========== 6. 转换并发布点云 ==========
  pcl::toROSMsg(_local_map, _local_map_pcd);  // PCL格式转ROS消息格式
  _local_map_pcd.header.frame_id = "map";     // 设置坐标系为world/map坐标系

  pub_cloud.publish(_local_map_pcd);  // 发布局部点云，供下游节点使用（如规划器）
}

/**
 * @brief 局部点云地图回调函数
 *
 * @param pointcloud_map 接收到的局部点云地图
 *
 * 功能：当前版本未实现，预留接口
 */
void rcvLocalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  // do nothing, fix later
  // 当前未实现，可能用于接收其他传感器的局部点云
}

/**
 * @brief 主函数 - 点云渲染节点初始化
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出状态
 *
 * 功能：
 * 1. 初始化ROS节点
 * 2. 读取配置参数（传感器参数、地图参数）
 * 3. 设置订阅者和发布者
 * 4. 创建定时器，周期性提取局部点云
 * 5. 进入ROS事件循环
 */
int main(int argc, char** argv) {
  // ========== 1. ROS节点初始化 ==========
  ros::init(argc, argv, "pcl_render");  // 初始化节点，名称为"pcl_render"
  ros::NodeHandle nh("~");              // 创建私有命名空间的节点句柄

  // ========== 2. 读取传感器参数 ==========
  // 从ROS参数服务器读取参数，这些参数通常在launch文件或yaml配置文件中定义
  nh.getParam("sensing_horizon", sensing_horizon);  // 感知距离（m），通常5-10m
  nh.getParam("sensing_rate", sensing_rate);        // 感知频率（Hz），通常10-30Hz
  nh.getParam("estimation_rate", estimation_rate);  // 估计频率（Hz），用于状态估计

  // ========== 3. 读取地图参数 ==========
  // 地图尺寸定义了点云的有效范围
  nh.getParam("map/x_size", _x_size);  // 地图x方向尺寸（m）
  nh.getParam("map/y_size", _y_size);  // 地图y方向尺寸（m）
  nh.getParam("map/z_size", _z_size);  // 地图z方向尺寸（m）

  // ========== 4. 设置订阅者 ==========
  // 订阅全局点云地图（队列长度1，只保留最新）
  global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  // 订阅局部点云地图（当前未使用）
  local_map_sub = nh.subscribe("local_map", 1, rcvLocalPointCloudCallBack);
  // 订阅里程计信息（队列长度50，保持高频更新）
  odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);

  // ========== 5. 设置发布者 ==========
  // 发布局部感知点云（队列长度10）
  pub_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/cloud", 10);

  // ========== 6. 创建感知定时器 ==========
  // 计算感知周期：2.5倍的感知周期（降低频率，减少计算负担）
  // 例如：如果sensing_rate=10Hz，则实际感知周期=1/10*2.5=0.25s，频率=4Hz
  double sensing_duration = 1.0 / sensing_rate * 2.5;

  // 创建定时器，周期性调用renderSensedPoints函数
  // 定时器会以固定频率触发，执行点云渲染和发布
  local_sensing_timer =
      nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);

  // ========== 7. 初始化地图参数 ==========
  // 注意：_resolution变量在此之前未被初始化，这可能是一个bug
  // 通常_resolution应该从参数服务器读取或设置默认值
  _inv_resolution = 1.0 / _resolution;  // 计算分辨率的倒数，用于快速计算

  // 设置地图起始坐标（地图中心在原点，起始点在左下角）
  _gl_xl = -_x_size / 2.0;  // x方向起始坐标
  _gl_yl = -_y_size / 2.0;  // y方向起始坐标
  _gl_zl = 0.0;             // z方向起始坐标（地面高度为0）
  // 示例：如果_x_size=10m，则地图x范围为[-5m, 5m]，起始坐标_gl_xl=-5m

  // 计算栅格数量
  // 公式：栅格数 = 地图尺寸 / 分辨率
  _GLX_SIZE = (int)(_x_size * _inv_resolution);  // x方向栅格数
  _GLY_SIZE = (int)(_y_size * _inv_resolution);  // y方向栅格数
  _GLZ_SIZE = (int)(_z_size * _inv_resolution);  // z方向栅格数
  // 示例：如果_x_size=10m，_resolution=0.1m，则_GLX_SIZE=100个栅格

  // ========== 8. ROS事件循环 ==========
  ros::Rate rate(100);  // 设置循环频率为100Hz
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();      // 处理回调函数队列中的所有待处理消息
    status = ros::ok();   // 检查ROS状态（节点是否被Ctrl+C终止等）
    rate.sleep();         // 按照设定频率休眠，确保循环以100Hz运行
  }
  // 注意：实际的点云感知频率由定时器控制（约4Hz，取决于sensing_rate参数）
  //      这里的100Hz主要用于及时处理回调函数
}
