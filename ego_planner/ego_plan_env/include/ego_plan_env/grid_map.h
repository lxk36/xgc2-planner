#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <ego_plan_env/raycast.h>

// logit函数定义：将概率值转换为对数几率形式，用于占用概率的数值稳定计算
#define logit(x) (log((x) / (1 - (x))))

using namespace std;

/**
 * @brief 体素哈希函数模板
 *
 * 用于将Eigen矩阵类型（通常是3D索引）转换为哈希值，以便在unordered_map等容器中使用
 * 采用标准的哈希组合方法，确保不同索引产生不同的哈希值
 */
template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    // 遍历矩阵的每个元素，使用标准哈希组合算法
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      // 0x9e3779b9是黄金比例的32位表示，用于哈希分散
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

/**
 * @brief 地图参数结构体
 *
 * 存储栅格地图的所有配置参数，包括地图属性、相机参数、深度图像处理、
 * 光线投射、可视化等相关设置
 */
struct MappingParameters {

  /* ========== 地图属性 ========== */
  Eigen::Vector3d map_origin_, map_size_;                // 地图原点和尺寸（米）
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // 地图在位置空间的边界范围
  Eigen::Vector3i map_voxel_num_;                        // 地图在索引空间的体素数量
  Eigen::Vector3d local_update_range_;                   // 局部更新范围（以相机为中心）
  double resolution_, resolution_inv_;                   // 地图分辨率及其倒数
  double obstacles_inflation_;                           // 障碍物膨胀距离
  string frame_id_;                                      // 坐标系名称
  int pose_type_;                                        // 位姿类型（POSE_STAMPED或ODOMETRY）

  /* ========== 相机参数 ========== */
  double cx_, cy_, fx_, fy_;                            // 相机内参：光心坐标和焦距

  /* ========== 深度图像投影滤波参数 ========== */
  double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;  // 深度滤波的最大/最小距离和容差
  int depth_filter_margin_;                              // 深度图像边缘裕度（像素）
  bool use_depth_filter_;                                // 是否使用深度滤波
  double k_depth_scaling_factor_;                        // 深度值缩放因子
  int skip_pixel_;                                       // 像素跳跃间隔，用于加速处理

  /* ========== 光线投射参数 ========== */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;       // 占用概率：命中、未命中、最小、最大、占用阈值
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                                // 占用概率的对数几率形式，用于数值稳定
  double min_ray_length_, max_ray_length_;               // 光线投射的最小和最大长度

  /* ========== 局部地图更新和清除参数 ========== */
  int local_map_margin_;                                 // 局部地图边界裕度（体素数）

  /* ========== 可视化和计算时间显示参数 ========== */
  double visualization_truncate_height_, virtual_ceil_height_, ground_height_;  // 可视化截断高度、虚拟天花板和地面高度
  bool show_occ_time_;                                   // 是否显示占用更新计算时间

  /* ========== 主动建图参数 ========== */
  double unknown_flag_;                                  // 未知区域标志值
};

/**
 * @brief 地图数据结构体
 *
 * 存储地图的中间数据，用于深度图像融合、光线投射和占用概率更新
 * 包含地图缓冲区、相机数据、深度图像、状态标志和性能统计等
 */
struct MappingData {
  /* ========== 主要地图数据 ========== */
  std::vector<double> occupancy_buffer_;        // 占用概率缓冲区（对数几率形式）
  std::vector<char> occupancy_buffer_inflate_;  // 膨胀后的占用缓冲区（用于碰撞检测）

  /* ========== 相机位置和位姿数据 ========== */
  Eigen::Vector3d camera_pos_, last_camera_pos_;      // 当前和上一帧相机位置
  Eigen::Quaterniond camera_q_, last_camera_q_;       // 当前和上一帧相机姿态（四元数）

  /* ========== 深度图像数据 ========== */
  cv::Mat depth_image_, last_depth_image_;      // 当前和上一帧深度图像
  int image_cnt_;                               // 图像帧计数器

  Eigen::Matrix4d cam2body_;                    // 相机坐标系到机体坐标系的变换矩阵

  /* ========== 地图状态标志 ========== */
  bool occ_need_update_, local_updated_;        // 占用地图是否需要更新、局部地图是否已更新
  bool has_first_depth_;                        // 是否接收到第一帧深度图像
  bool has_odom_, has_cloud_;                   // 是否接收到里程计、点云数据

  /* ========== 深度图像投影点云 ========== */
  vector<Eigen::Vector3d> proj_points_;         // 投影后的3D点集合
  int proj_points_cnt;                          // 投影点数量

  /* ========== 光线投射加速标志缓冲区 ========== */
  vector<short> count_hit_, count_hit_and_miss_;  // 命中计数、命中和未命中计数
  vector<char> flag_traverse_, flag_rayend_;      // 遍历标志、射线终点标志
  char raycast_num_;                              // 光线投射编号（用于区分不同帧）
  queue<Eigen::Vector3i> cache_voxel_;            // 缓存的体素队列

  /* ========== 更新范围 ========== */
  Eigen::Vector3i local_bound_min_, local_bound_max_;  // 局部更新的边界索引范围

  /* ========== 计算时间统计 ========== */
  double fuse_time_, max_fuse_time_;            // 融合耗时、最大融合耗时
  int update_num_;                              // 更新次数

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief 栅格地图类
 *
 * 基于概率占用的3D栅格地图，用于无人机路径规划的环境表示
 *
 * 主要功能：
 * 1. 深度图像融合：将深度相机数据融合到3D占用地图
 * 2. 光线投射：使用射线投射算法更新占用概率
 * 3. 障碍物膨胀：对障碍物进行膨胀以保证飞行安全
 * 4. 局部地图更新：仅更新相机视野范围内的局部区域
 * 5. 未知空间管理：区分已知自由、已知占用和未知空间
 *
 * 数据结构：
 * - 使用一维数组存储3D栅格数据，通过索引映射访问
 * - 每个体素存储占用概率的对数几率值
 * - 维护膨胀地图用于碰撞检测
 *
 * 应用场景：
 * - 无人机自主导航的环境感知
 * - 实时路径规划的障碍物检测
 * - 未知环境探索
 */
class GridMap {
public:
  GridMap() {}
  ~GridMap() {}

  // 位姿类型枚举和无效索引常量
  enum { POSE_STAMPED = 1, ODOMETRY = 2, INVALID_IDX = -10000 };

  /* ========== 占用地图管理 ========== */

  /**
   * @brief 重置整个地图缓冲区
   * 将所有体素重置为未知状态
   */
  void resetBuffer();

  /**
   * @brief 重置指定区域的地图缓冲区
   * @param min 重置区域的最小边界
   * @param max 重置区域的最大边界
   */
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  /* ========== 坐标转换函数 ========== */

  /**
   * @brief 将3D位置转换为栅格索引
   * @param pos 3D空间位置（米）
   * @param id 输出的栅格索引
   */
  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);

  /**
   * @brief 将栅格索引转换为3D位置（体素中心）
   * @param id 栅格索引
   * @param pos 输出的3D空间位置（米）
   */
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);

  /**
   * @brief 将3D索引转换为一维数组地址
   * @param id 3D栅格索引
   * @return 一维数组中的地址
   */
  inline int toAddress(const Eigen::Vector3i& id);

  /**
   * @brief 将x,y,z索引转换为一维数组地址
   * @param x,y,z 栅格索引的三个分量
   * @return 一维数组中的地址
   */
  inline int toAddress(int& x, int& y, int& z);

  /**
   * @brief 检查3D位置是否在地图范围内
   * @param pos 3D空间位置
   * @return true表示在地图内
   */
  inline bool isInMap(const Eigen::Vector3d& pos);

  /**
   * @brief 检查索引是否在地图范围内
   * @param idx 栅格索引
   * @return true表示在地图内
   */
  inline bool isInMap(const Eigen::Vector3i& idx);

  /* ========== 占用状态查询和设置 ========== */

  /**
   * @brief 设置指定位置的占用概率
   * @param pos 3D空间位置
   * @param occ 占用概率（0或1）
   */
  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);

  /**
   * @brief 将指定位置设置为占用状态
   * @param pos 3D空间位置
   */
  inline void setOccupied(Eigen::Vector3d pos);

  /**
   * @brief 获取指定位置的占用状态
   * @param pos 3D空间位置
   * @return 0表示自由，1表示占用，-1表示越界
   */
  inline int getOccupancy(Eigen::Vector3d pos);

  /**
   * @brief 获取指定索引的占用状态
   * @param id 栅格索引
   * @return 0表示自由，1表示占用，-1表示越界
   */
  inline int getOccupancy(Eigen::Vector3i id);

  /**
   * @brief 获取指定位置的膨胀占用状态
   * @param pos 3D空间位置
   * @return 0表示自由，1表示占用（包含膨胀区域），-1表示越界
   */
  inline int getInflateOccupancy(Eigen::Vector3d pos);

  /* ========== 索引和状态判断 ========== */

  /**
   * @brief 将索引限制在地图边界内
   * @param id 待限制的索引，会被就地修改
   */
  inline void boundIndex(Eigen::Vector3i& id);

  /**
   * @brief 检查指定索引是否为未知状态
   * @param id 栅格索引
   * @return true表示未知
   */
  inline bool isUnknown(const Eigen::Vector3i& id);

  /**
   * @brief 检查指定位置是否为未知状态
   * @param pos 3D空间位置
   * @return true表示未知
   */
  inline bool isUnknown(const Eigen::Vector3d& pos);

  /**
   * @brief 检查指定索引是否为已知自由空间
   * @param id 栅格索引
   * @return true表示已知自由
   */
  inline bool isKnownFree(const Eigen::Vector3i& id);

  /**
   * @brief 检查指定索引是否为已知占用空间
   * @param id 栅格索引
   * @return true表示已知占用
   */
  inline bool isKnownOccupied(const Eigen::Vector3i& id);

  /* ========== 地图初始化和发布 ========== */

  /**
   * @brief 初始化地图，从ROS参数服务器加载配置
   * @param nh ROS节点句柄
   */
  void initMap(ros::NodeHandle& nh);

  /**
   * @brief 发布占用地图（用于可视化）
   */
  void publishMap();

  /**
   * @brief 发布膨胀后的占用地图
   * @param all_info 是否发布全部信息
   */
  void publishMapInflate(bool all_info = false);

  /**
   * @brief 发布未知区域（用于主动探索）
   */
  void publishUnknown();

  /**
   * @brief 发布深度图像（用于调试）
   */
  void publishDepth();

  /* ========== 状态查询 ========== */

  /**
   * @brief 检查是否有深度观测数据
   * @return true表示有深度数据
   */
  bool hasDepthObservation();

  /**
   * @brief 检查里程计数据是否有效
   * @return true表示里程计有效
   */
  bool odomValid();

  /**
   * @brief 获取地图区域信息
   * @param ori 输出地图原点
   * @param size 输出地图尺寸
   */
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);

  /**
   * @brief 获取地图分辨率
   * @return 分辨率（米/体素）
   */
  inline double getResolution();

  /**
   * @brief 获取地图原点
   * @return 地图原点坐标
   */
  Eigen::Vector3d getOrigin();

  /**
   * @brief 获取体素总数
   * @return 体素数量
   */
  int getVoxelNum();

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;  // 地图参数配置
  MappingData md_;        // 地图数据

  /* ========== 回调函数 ========== */

  /**
   * @brief 深度图像和位姿同步回调
   * @param img 深度图像消息
   * @param pose 相机位姿消息
   */
  void depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                         const geometry_msgs::PoseStampedConstPtr& pose);

  /**
   * @brief 深度图像和里程计同步回调
   * @param img 深度图像消息
   * @param odom 里程计消息
   */
  void depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);

  /**
   * @brief 点云数据回调
   * @param img 点云消息
   */
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img);

  /**
   * @brief 里程计数据回调
   * @param odom 里程计消息
   */
  void odomCallback(const nav_msgs::OdometryConstPtr& odom);

  /**
   * @brief 占用地图更新定时器回调
   * 触发光线投射和地图更新过程
   */
  void updateOccupancyCallback(const ros::TimerEvent& /*event*/);

  /**
   * @brief 可视化定时器回调
   * 定期发布地图用于RViz显示
   */
  void visCallback(const ros::TimerEvent& /*event*/);

  /* ========== 主要更新流程 ========== */

  /**
   * @brief 投影深度图像到3D空间
   * 使用相机内参将2D深度图像转换为3D点云
   */
  void projectDepthImage();

  /**
   * @brief 光线投射处理
   * 对投影的3D点执行光线投射，更新沿路径的占用概率
   */
  void raycastProcess();

  /**
   * @brief 清除和膨胀局部地图
   * 清除超出视野范围的旧数据，并对障碍物进行膨胀
   */
  void clearAndInflateLocalMap();

  /* ========== 辅助函数 ========== */

  /**
   * @brief 膨胀单个点
   * @param pt 中心点索引
   * @param step 膨胀步长（体素数）
   * @param pts 输出的膨胀点集合
   */
  inline void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);

  /**
   * @brief 设置缓存占用状态
   * @param pos 3D位置
   * @param occ 占用状态
   * @return 设置结果状态码
   */
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);

  /**
   * @brief 找到地图内最接近给定点的位置
   * @param pt 目标点
   * @param camera_pt 相机位置
   * @return 地图内最近的有效点
   */
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);

  /* ========== ROS消息同步策略 ========== */
  // 精确时间同步策略（已注释，使用近似时间同步）
  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // geometry_msgs::PoseStamped> SyncPolicyImagePose;

  // 近似时间同步策略：允许深度图像和位姿/里程计消息有一定时间差
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  /* ========== ROS通信接口 ========== */
  ros::NodeHandle node_;                                                  // ROS节点句柄
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;            // 深度图像订阅器
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;    // 位姿订阅器
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;             // 里程计订阅器
  SynchronizerImagePose sync_image_pose_;                                 // 图像-位姿同步器
  SynchronizerImageOdom sync_image_odom_;                                 // 图像-里程计同步器

  ros::Subscriber indep_cloud_sub_, indep_odom_sub_;                      // 独立的点云和里程计订阅器
  ros::Publisher map_pub_, map_inf_pub_;                                  // 地图发布器：原始地图和膨胀地图
  ros::Publisher unknown_pub_;                                            // 未知区域发布器
  ros::Timer occ_timer_, vis_timer_;                                      // 定时器：占用更新和可视化

  /* ========== 随机数生成器 ========== */
  uniform_real_distribution<double> rand_noise_;   // 均匀分布随机噪声
  normal_distribution<double> rand_noise2_;        // 正态分布随机噪声
  default_random_engine eng_;                      // 随机数引擎
};

/* ============================== 内联函数实现 ============================== */

/**
 * @brief 将3D索引转换为一维数组地址
 *
 * 使用行优先顺序（x主序）将3D索引映射到1D数组
 * 公式: address = x * (Ny * Nz) + y * Nz + z
 *
 * @param id 3D栅格索引
 * @return 一维数组中的地址
 */
inline int GridMap::toAddress(const Eigen::Vector3i& id) {
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

/**
 * @brief 将x,y,z索引转换为一维数组地址（重载版本）
 *
 * @param x,y,z 栅格索引的三个分量
 * @return 一维数组中的地址
 */
inline int GridMap::toAddress(int& x, int& y, int& z) {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

/**
 * @brief 将索引限制在地图边界内
 *
 * 将超出边界的索引夹紧到[0, map_voxel_num-1]范围内
 *
 * @param id 待限制的索引，会被就地修改
 */
inline void GridMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
  id = id1;
}

/**
 * @brief 检查指定索引是否为未知状态
 *
 * 如果占用概率的对数几率值小于最小阈值，则认为是未知区域
 *
 * @param id 栅格索引
 * @return true表示未知
 */
inline bool GridMap::isUnknown(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

/**
 * @brief 检查指定位置是否为未知状态（重载版本）
 *
 * @param pos 3D空间位置
 * @return true表示未知
 */
inline bool GridMap::isUnknown(const Eigen::Vector3d& pos) {
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

/**
 * @brief 检查指定索引是否为已知自由空间
 *
 * 已知自由的条件：
 * 1. 占用概率高于最小阈值（已被观测）
 * 2. 膨胀后的占用缓冲区值为0（不在障碍物膨胀区域内）
 *
 * @param id 栅格索引
 * @return true表示已知自由
 */
inline bool GridMap::isKnownFree(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // 旧版判断方式（已注释）：
  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

/**
 * @brief 检查指定索引是否为已知占用空间
 *
 * 直接检查膨胀后的占用缓冲区，包括障碍物本身和膨胀区域
 *
 * @param id 栅格索引
 * @return true表示已知占用
 */
inline bool GridMap::isKnownOccupied(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

/**
 * @brief 将指定位置设置为占用状态
 *
 * 直接在膨胀缓冲区中标记为占用，用于手动设置障碍物
 *
 * @param pos 3D空间位置
 */
inline void GridMap::setOccupied(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                id(1) * mp_.map_voxel_num_(2) + id(2)] = 1;
}

/**
 * @brief 设置指定位置的占用概率
 *
 * 用于直接设置原始占用缓冲区的值（非膨胀缓冲区）
 *
 * @param pos 3D空间位置
 * @param occ 占用概率（必须为0或1）
 */
inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ) {
  if (occ != 1 && occ != 0) {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_[toAddress(id)] = occ;
}

/**
 * @brief 获取指定位置的占用状态
 *
 * 根据占用概率的对数几率值判断是否占用
 *
 * @param pos 3D空间位置
 * @return 0表示自由，1表示占用，-1表示越界
 */
inline int GridMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

/**
 * @brief 获取指定位置的膨胀占用状态
 *
 * 直接返回膨胀缓冲区的值，用于碰撞检测
 *
 * @param pos 3D空间位置
 * @return 0表示自由，1表示占用（包含膨胀区域），-1表示越界
 */
inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

/**
 * @brief 获取指定索引的占用状态（重载版本）
 *
 * @param id 栅格索引
 * @return 0表示自由，1表示占用，-1表示越界
 */
inline int GridMap::getOccupancy(Eigen::Vector3i id) {
  if (id(0) < 0 || id(0) >= mp_.map_voxel_num_(0) || id(1) < 0 || id(1) >= mp_.map_voxel_num_(1) ||
      id(2) < 0 || id(2) >= mp_.map_voxel_num_(2))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

/**
 * @brief 检查3D位置是否在地图范围内
 *
 * 使用小容差值（1e-4）处理浮点数精度问题
 *
 * @param pos 3D空间位置
 * @return true表示在地图内
 */
inline bool GridMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

/**
 * @brief 检查索引是否在地图范围内（重载版本）
 *
 * @param idx 栅格索引
 * @return true表示在地图内
 */
inline bool GridMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
      idx(2) > mp_.map_voxel_num_(2) - 1) {
    return false;
  }
  return true;
}

/**
 * @brief 将3D位置转换为栅格索引
 *
 * 转换公式: id = floor((pos - origin) / resolution)
 * 使用向下取整确保位置映射到正确的体素
 *
 * @param pos 3D空间位置（米）
 * @param id 输出的栅格索引
 */
inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

/**
 * @brief 将栅格索引转换为3D位置（体素中心）
 *
 * 转换公式: pos = (id + 0.5) * resolution + origin
 * 加0.5使位置对应体素中心而非角点
 *
 * @param id 栅格索引
 * @param pos 输出的3D空间位置（米）
 */
inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

/**
 * @brief 膨胀单个点，生成周围邻域点集
 *
 * 当前使用立方体膨胀策略，生成以pt为中心、边长为(2*step+1)的立方体内所有点
 * 也可选择"+"形膨胀策略（已注释），仅膨胀沿坐标轴方向的点
 *
 * @param pt 中心点索引
 * @param step 膨胀步长（体素数）
 * @param pts 输出的膨胀点集合（需预分配空间）
 */
inline void GridMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
  int num = 0;

  /* ---------- "+"形膨胀策略（已注释，不使用） ---------- */
  // 仅膨胀沿x、y、z轴方向的点，形成"+"字形状
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- 立方体完全膨胀策略（当前使用） ---------- */
  // 膨胀所有方向的点，形成立方体
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z) {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}

/**
 * @brief 获取地图分辨率
 *
 * @return 分辨率（米/体素）
 */
inline double GridMap::getResolution() { return mp_.resolution_; }

#endif