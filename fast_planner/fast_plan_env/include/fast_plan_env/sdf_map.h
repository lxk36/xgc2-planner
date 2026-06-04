/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef _SDF_MAP_H
#define _SDF_MAP_H

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

#include <fast_plan_env/raycast.h>

// 对数几率函数宏定义，将概率值转换为对数几率值
#define logit(x) (log((x) / (1 - (x))))

using namespace std;

// 体素哈希结构体，用于实现基于矩阵的哈希函数
template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  // 哈希函数运算符重载，计算矩阵的哈希值
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// 地图参数结构体，存储所有常量配置参数

struct MappingParameters {

  /* 地图属性 */
  Eigen::Vector3d map_origin_, map_size_;                 // 地图原点和尺寸
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;   // 地图在位置坐标中的范围
  Eigen::Vector3i map_voxel_num_;                         // 地图在索引坐标中的范围（体素数量）
  Eigen::Vector3i map_min_idx_, map_max_idx_;             // 地图最小和最大索引
  Eigen::Vector3d local_update_range_;                    // 局部更新范围
  double resolution_, resolution_inv_;                    // 地图分辨率及其倒数
  double obstacles_inflation_;                            // 障碍物膨胀距离
  string frame_id_;                                       // 坐标系ID
  int pose_type_;                                         // 位姿类型
  string map_input_;                                      // 地图输入类型：1: 位姿+深度图; 2: 里程计+点云

  /* 相机参数 */
  double cx_, cy_, fx_, fy_;                              // 相机内参：光心坐标(cx,cy)和焦距(fx,fy)

  /* 深度图投影滤波参数 */
  double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;  // 深度滤波的最大/最小距离和容差
  int depth_filter_margin_;                               // 深度滤波边缘
  bool use_depth_filter_;                                 // 是否使用深度滤波
  double k_depth_scaling_factor_;                         // 深度缩放因子
  int skip_pixel_;                                        // 跳过的像素数

  /* 光线投射参数 */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;         // 占据概率：击中、未击中、最小、最大、占据阈值
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                                 // 占据概率的对数几率值
  double min_ray_length_, max_ray_length_;                // 光线投射的最小和最大长度范围

  /* 局部地图更新和清除参数 */
  double local_bound_inflate_;                            // 局部边界膨胀距离
  int local_map_margin_;                                  // 局部地图边缘

  /* 可视化和计算时间显示参数 */
  double esdf_slice_height_, visualization_truncate_height_, virtual_ceil_height_, ground_height_;  // ESDF切片高度、可视化截断高度、虚拟天花板高度、地面高度
  bool show_esdf_time_, show_occ_time_;                   // 是否显示ESDF和占据地图的计算时间

  /* 主动建图参数 */
  double unknown_flag_;                                   // 未知区域标志值
};

// 中间建图数据结构体，用于融合和ESDF计算

struct MappingData {
  // 主要地图数据：每个体素的占据概率和欧几里得距离

  std::vector<double> occupancy_buffer_;              // 占据概率缓冲区
  std::vector<char> occupancy_buffer_neg;             // 负占据概率缓冲区
  std::vector<char> occupancy_buffer_inflate_;        // 膨胀后的占据概率缓冲区
  std::vector<double> distance_buffer_;               // 正距离场缓冲区
  std::vector<double> distance_buffer_neg_;           // 负距离场缓冲区
  std::vector<double> distance_buffer_all_;           // 完整距离场缓冲区
  std::vector<double> tmp_buffer1_;                   // 临时缓冲区1
  std::vector<double> tmp_buffer2_;                   // 临时缓冲区2

  // 相机位置和位姿数据

  Eigen::Vector3d camera_pos_, last_camera_pos_;      // 当前和上一次相机位置
  Eigen::Quaterniond camera_q_, last_camera_q_;       // 当前和上一次相机姿态（四元数）

  // 深度图数据

  cv::Mat depth_image_, last_depth_image_;            // 当前和上一次深度图
  int image_cnt_;                                     // 深度图计数器

  // 地图状态标志

  bool occ_need_update_, local_updated_, esdf_need_update_;  // 占据地图需要更新、局部已更新、ESDF需要更新标志
  bool has_first_depth_;                              // 是否已接收第一帧深度图
  bool has_odom_, has_cloud_;                         // 是否有里程计数据、是否有点云数据

  // 深度图投影点云

  vector<Eigen::Vector3d> proj_points_;               // 投影点云数组
  int proj_points_cnt;                                // 投影点数量

  // 加速光线投射的标志缓冲区

  vector<short> count_hit_, count_hit_and_miss_;      // 击中计数、击中和未击中计数
  vector<char> flag_traverse_, flag_rayend_;          // 遍历标志、光线终点标志
  char raycast_num_;                                  // 光线投射编号
  queue<Eigen::Vector3i> cache_voxel_;                // 缓存体素队列

  // ESDF更新范围

  Eigen::Vector3i local_bound_min_, local_bound_max_; // 局部边界的最小和最大索引

  // 计算时间统计

  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;  // 融合时间、ESDF时间、最大融合时间、最大ESDF时间
  int update_num_;                                    // 更新次数

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief SDF地图类，实现基于有符号距离场(Signed Distance Field)的环境地图表示
 *
 * 该类提供了占据地图和ESDF的完整功能，包括：
 * - 深度图融合和光线投射
 * - 占据概率更新
 * - ESDF计算和查询
 * - 地图可视化
 */
class SDFMap {
public:
  SDFMap() : node_(nullptr) {}
  ~SDFMap() {}

  // 枚举类型定义：位姿类型和无效索引
  enum { POSE_STAMPED = 1, ODOMETRY = 2, INVALID_IDX = -10000 };

  // 占据地图管理函数
  void resetBuffer();                                                    // 重置所有缓冲区
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);          // 重置指定范围的缓冲区

  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);  // 位置坐标转换为体素索引
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);  // 体素索引转换为位置坐标
  inline int toAddress(const Eigen::Vector3i& id);                          // 将3D索引转换为1D数组地址
  inline int toAddress(int& x, int& y, int& z);                             // 将3D索引坐标转换为1D数组地址
  inline bool isInMap(const Eigen::Vector3d& pos);                          // 检查位置是否在地图范围内
  inline bool isInMap(const Eigen::Vector3i& idx);                          // 检查索引是否在地图范围内

  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);            // 设置指定位置的占据概率
  inline void setOccupied(Eigen::Vector3d pos);                             // 将指定位置设置为占据
  inline int getOccupancy(Eigen::Vector3d pos);                             // 获取指定位置的占据状态
  inline int getOccupancy(Eigen::Vector3i id);                              // 获取指定索引的占据状态
  inline int getInflateOccupancy(Eigen::Vector3d pos);                      // 获取膨胀后的占据状态

  inline void boundIndex(Eigen::Vector3i& id);                              // 将索引限制在地图范围内
  inline bool isUnknown(const Eigen::Vector3i& id);                         // 检查索引位置是否为未知
  inline bool isUnknown(const Eigen::Vector3d& pos);                        // 检查位置是否为未知
  inline bool isKnownFree(const Eigen::Vector3i& id);                       // 检查索引位置是否为已知自由空间
  inline bool isKnownOccupied(const Eigen::Vector3i& id);                   // 检查索引位置是否为已知占据

  // 距离场管理函数
  inline double getDistance(const Eigen::Vector3d& pos);                    // 获取指定位置的距离值
  inline double getDistance(const Eigen::Vector3i& id);                     // 获取指定索引的距离值
  inline double getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad);  // 使用三线性插值获取距离和梯度
  void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d& diff);  // 获取周围点用于插值

  void updateESDF3d();                                                      // 更新3D ESDF
  void getSliceESDF(const double height, const double res, const Eigen::Vector4d& range,
                    vector<Eigen::Vector3d>& slice, vector<Eigen::Vector3d>& grad,
                    int sign = 1);                                          // 获取ESDF切片，sign: 1正距离, 2负距离, 3组合
  void initMap(ros::NodeHandle& nh);                                        // 初始化地图

  void publishMap();                                                        // 发布占据地图
  void publishMapInflate(bool all_info = false);                           // 发布膨胀后的地图
  void publishESDF();                                                       // 发布ESDF
  void publishUpdateRange();                                                // 发布更新范围

  void publishUnknown();                                                    // 发布未知区域
  void publishDepth();                                                      // 发布深度图

  void checkDist();                                                         // 检查距离场
  bool hasDepthObservation();                                               // 是否有深度观测
  bool odomValid();                                                         // 里程计是否有效
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);             // 获取地图区域
  double getResolution();                                                   // 获取地图分辨率
  Eigen::Vector3d getOrigin();                                              // 获取地图原点
  int getVoxelNum();                                                        // 获取体素总数

  typedef std::shared_ptr<SDFMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;  // 地图参数
  MappingData md_;        // 地图数据

  // ESDF填充函数模板，用于在指定维度上填充距离场
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

  // 获取深度图和相机位姿的回调函数
  void depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                         const geometry_msgs::PoseStampedConstPtr& pose);      // 深度图+位姿回调
  void depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);  // 深度图+里程计回调
  void depthCallback(const sensor_msgs::ImageConstPtr& img);                   // 深度图回调
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img);             // 点云回调
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose);           // 位姿回调
  void odomCallback(const nav_msgs::OdometryConstPtr& odom);                   // 里程计回调

  // 通过光线投射更新占据地图，并更新ESDF
  void updateOccupancyCallback(const ros::TimerEvent& /*event*/);              // 占据地图更新定时回调
  void updateESDFCallback(const ros::TimerEvent& /*event*/);                   // ESDF更新定时回调
  void visCallback(const ros::TimerEvent& /*event*/);                          // 可视化定时回调

  // 主要更新流程
  void projectDepthImage();                                                     // 投影深度图到3D点云
  void raycastProcess();                                                        // 光线投射处理
  void clearAndInflateLocalMap();                                               // 清除并膨胀局部地图

  inline void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);  // 膨胀单个点
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);                          // 设置缓存占据状态
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);  // 获取地图内最近点

  // 消息同步策略类型定义（使用近似时间同步）
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;                                                      // 深度图-里程计同步策略
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;                                                      // 深度图-位姿同步策略
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;  // 深度图-位姿同步器
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;  // 深度图-里程计同步器

  // ROS通信相关成员变量
  ros::NodeHandle* node_;                                                       // ROS节点句柄
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;      // 深度图订阅器
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;  // 位姿订阅器
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;       // 里程计订阅器
  SynchronizerImagePose sync_image_pose_;                                       // 深度图-位姿同步器实例
  SynchronizerImageOdom sync_image_odom_;                                       // 深度图-里程计同步器实例

  ros::Subscriber indep_depth_sub_, indep_odom_sub_, indep_pose_sub_, indep_cloud_sub_;  // 独立订阅器：深度图、里程计、位姿、点云
  ros::Publisher map_pub_, esdf_pub_, map_inf_pub_, update_range_pub_;         // 发布器：占据地图、ESDF、膨胀地图、更新范围
  ros::Publisher unknown_pub_, depth_pub_;                                      // 发布器：未知区域、深度图
  ros::Timer occ_timer_, esdf_timer_, vis_timer_;                               // 定时器：占据地图更新、ESDF更新、可视化

  // 随机数生成器，用于添加噪声
  uniform_real_distribution<double> rand_noise_;                                // 均匀分布随机噪声
  normal_distribution<double> rand_noise2_;                                     // 正态分布随机噪声
  default_random_engine eng_;                                                   // 随机数引擎
};

/* ============================== 内联函数定义 ============================== */

// 将3D索引转换为1D数组地址
inline int SDFMap::toAddress(const Eigen::Vector3i& id) {
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

// 将3D索引坐标转换为1D数组地址
inline int SDFMap::toAddress(int& x, int& y, int& z) {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

// 将索引限制在地图有效范围内
inline void SDFMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
  id = id1;
}

// 获取指定位置的距离值
inline double SDFMap::getDistance(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  boundIndex(id);

  return md_.distance_buffer_all_[toAddress(id)];
}

// 获取指定索引的距离值
inline double SDFMap::getDistance(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.distance_buffer_all_[toAddress(id1)];
}

// 检查索引位置是否为未知区域
inline bool SDFMap::isUnknown(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

// 检查位置是否为未知区域
inline bool SDFMap::isUnknown(const Eigen::Vector3d& pos) {
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

// 检查索引位置是否为已知自由空间
inline bool SDFMap::isKnownFree(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // 占据概率大于最小阈值且膨胀缓冲区为0，表示已知自由空间
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

// 检查索引位置是否为已知占据
inline bool SDFMap::isKnownOccupied(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

// 使用三线性插值获取距离值和梯度
inline double SDFMap::getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad) {
  if (!isInMap(pos)) {
    grad.setZero();
    return 0;
  }

  /* 使用三线性插值 */
  // 偏移半个体素以进行插值
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();

  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);

  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);

  // 计算插值权重
  diff = (pos - idx_pos) * mp_.resolution_inv_;

  // 获取周围8个体素的距离值
  double values[2][2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }
    }
  }

  // 三线性插值计算距离值
  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  // 计算梯度（距离场的导数）
  grad[2] = (v1 - v0) * mp_.resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_.resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

  grad[0] *= mp_.resolution_inv_;

  return dist;
}

// 将指定位置设置为占据状态
inline void SDFMap::setOccupied(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                id(1) * mp_.map_voxel_num_(2) + id(2)] = 1;
}

// 设置指定位置的占据概率值
inline void SDFMap::setOccupancy(Eigen::Vector3d pos, double occ) {
  if (occ != 1 && occ != 0) {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_[toAddress(id)] = occ;
}

// 获取指定位置的占据状态（基于位置坐标）
inline int SDFMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

// 获取膨胀后的占据状态
inline int SDFMap::getInflateOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

// 获取指定索引的占据状态（基于索引坐标）
inline int SDFMap::getOccupancy(Eigen::Vector3i id) {
  if (id(0) < 0 || id(0) >= mp_.map_voxel_num_(0) || id(1) < 0 || id(1) >= mp_.map_voxel_num_(1) ||
      id(2) < 0 || id(2) >= mp_.map_voxel_num_(2))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

// 检查位置坐标是否在地图范围内
inline bool SDFMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

// 检查索引坐标是否在地图范围内
inline bool SDFMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
      idx(2) > mp_.map_voxel_num_(2) - 1) {
    return false;
  }
  return true;
}

// 将位置坐标转换为体素索引
inline void SDFMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

// 将体素索引转换为位置坐标（体素中心）
inline void SDFMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

// 膨胀单个点，生成周围的膨胀点集
inline void SDFMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
  int num = 0;

  /* ---------- + 形状膨胀（已注释） ---------- */
  // 仅在x、y、z三个轴向上膨胀，形成+字形
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

  /* ---------- 全方向膨胀（当前使用） ---------- */
  // 在所有方向上膨胀，形成立方体
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z) {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}

#endif