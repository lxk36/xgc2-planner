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



#include "fast_plan_env/sdf_map.h"

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

/**
 * @brief 初始化SDF地图
 * @param nh ROS节点句柄
 *
 * 功能说明：
 * 1. 从ROS参数服务器读取地图配置参数
 * 2. 初始化地图数据缓冲区
 * 3. 设置订阅器和发布器
 * 4. 启动定时器回调
 */
void SDFMap::initMap(ros::NodeHandle& nh) {
  node_ = &nh;

  // 初始化智能指针为nullptr，防止未定义行为
  sync_image_pose_.reset();
  sync_image_odom_.reset();
  depth_sub_.reset();
  pose_sub_.reset();
  odom_sub_.reset();

  /* 从参数服务器获取地图参数 */
  double x_size, y_size, z_size;
  // 地图基本参数
  node_->param("sdf_map/resolution", mp_.resolution_, -1.0);  // 地图分辨率(m)
  node_->param("sdf_map/map_size_x", x_size, -1.0);  // 地图X方向尺寸(m)
  node_->param("sdf_map/map_size_y", y_size, -1.0);  // 地图Y方向尺寸(m)
  node_->param("sdf_map/map_size_z", z_size, -1.0);  // 地图Z方向尺寸(m)
  node_->param("sdf_map/local_update_range_x", mp_.local_update_range_(0), -1.0);  // 局部更新范围X
  node_->param("sdf_map/local_update_range_y", mp_.local_update_range_(1), -1.0);  // 局部更新范围Y
  node_->param("sdf_map/local_update_range_z", mp_.local_update_range_(2), -1.0);  // 局部更新范围Z
  node_->param("sdf_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);  // 障碍物膨胀半径(m)

  // 相机内参
  node_->param("sdf_map/fx", mp_.fx_, -1.0);  // 焦距fx
  node_->param("sdf_map/fy", mp_.fy_, -1.0);  // 焦距fy
  node_->param("sdf_map/cx", mp_.cx_, -1.0);  // 主点cx
  node_->param("sdf_map/cy", mp_.cy_, -1.0);  // 主点cy

  // 深度图滤波参数
  node_->param("sdf_map/use_depth_filter", mp_.use_depth_filter_, true);  // 是否使用深度滤波
  node_->param("sdf_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);  // 深度滤波容差
  node_->param("sdf_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);  // 深度滤波最大距离
  node_->param("sdf_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);  // 深度滤波最小距离
  node_->param("sdf_map/depth_filter_margin", mp_.depth_filter_margin_, -1);  // 深度图边缘裕度
  node_->param("sdf_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);  // 深度缩放因子
  node_->param("sdf_map/skip_pixel", mp_.skip_pixel_, -1);  // 跳过的像素数(降采样)

  // 概率占据网格参数 (用于贝叶斯更新)
  node_->param("sdf_map/p_hit", mp_.p_hit_, 0.70);  // 射线击中障碍物的概率
  node_->param("sdf_map/p_miss", mp_.p_miss_, 0.35);  // 射线未击中障碍物的概率
  node_->param("sdf_map/p_min", mp_.p_min_, 0.12);  // 最小占据概率
  node_->param("sdf_map/p_max", mp_.p_max_, 0.97);  // 最大占据概率
  node_->param("sdf_map/p_occ", mp_.p_occ_, 0.80);  // 占据阈值概率
  node_->param("sdf_map/min_ray_length", mp_.min_ray_length_, -0.1);  // 最小射线长度
  node_->param("sdf_map/max_ray_length", mp_.max_ray_length_, -0.1);  // 最大射线长度

  // ESDF和可视化参数
  node_->param("sdf_map/esdf_slice_height", mp_.esdf_slice_height_, -0.1);  // ESDF切片高度
  node_->param("sdf_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);  // 可视化截断高度
  node_->param("sdf_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);  // 虚拟天花板高度

  // 调试和显示参数
  node_->param("sdf_map/show_occ_time", mp_.show_occ_time_, false);  // 是否显示占据更新时间
  node_->param("sdf_map/show_esdf_time", mp_.show_esdf_time_, false);  // 是否显示ESDF更新时间
  node_->param("sdf_map/pose_type", mp_.pose_type_, 1);  // 位姿类型 (1=POSE_STAMPED, 2=ODOMETRY)

  // 地图坐标系和边界参数
  node_->param("sdf_map/frame_id", mp_.frame_id_, string("world"));  // 坐标系ID
  node_->param("sdf_map/local_bound_inflate", mp_.local_bound_inflate_, 1.0);  // 局部边界膨胀
  node_->param("sdf_map/local_map_margin", mp_.local_map_margin_, 1);  // 局部地图裕度
  node_->param("sdf_map/ground_height", mp_.ground_height_, 1.0);  // 地面高度

  // 计算派生参数
  mp_.local_bound_inflate_ = max(mp_.resolution_, mp_.local_bound_inflate_);  // 确保膨胀至少为一个体素大小
  mp_.resolution_inv_ = 1 / mp_.resolution_;  // 分辨率的倒数，用于加速计算
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);  // 地图原点(地图中心在xy平面原点)
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);  // 地图尺寸

  // 将概率转换为log-odds形式，用于高效的贝叶斯更新
  mp_.prob_hit_log_ = logit(mp_.p_hit_);  // 击中的log-odds
  mp_.prob_miss_log_ = logit(mp_.p_miss_);  // 未击中的log-odds
  mp_.clamp_min_log_ = logit(mp_.p_min_);  // 最小log-odds
  mp_.clamp_max_log_ = logit(mp_.p_max_);  // 最大log-odds
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);  // 占据阈值的log-odds
  mp_.unknown_flag_ = 0.01;  // 未知区域标志

  // 输出log-odds参数，用于调试
  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  // 计算每个维度的体素数量
  for (int i = 0; i < 3; ++i) mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  // 设置地图边界
  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // 设置地图索引边界
  mp_.map_min_idx_ = Eigen::Vector3i::Zero();
  mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  // 初始化数据缓冲区

  // 计算缓冲区大小
  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  // 占据概率相关缓冲区
  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);  // 占据概率(log-odds)
  md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);  // 负距离场的占据标志
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);  // 膨胀后的占据网格

  // 距离场相关缓冲区
  md_.distance_buffer_ = vector<double>(buffer_size, 10000);  // 正距离场(到最近障碍物的距离)
  md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);  // 负距离场(障碍物内部到边界的距离)
  md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);  // 完整距离场(正负组合)

  // 射线投射相关缓冲区
  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);  // 射线击中和未击中的总次数
  md_.count_hit_ = vector<short>(buffer_size, 0);  // 射线击中次数
  md_.flag_rayend_ = vector<char>(buffer_size, -1);  // 射线终点标志(用于去重)
  md_.flag_traverse_ = vector<char>(buffer_size, -1);  // 射线穿过标志(用于去重)

  // ESDF计算的临时缓冲区
  md_.tmp_buffer1_ = vector<double>(buffer_size, 0);  // 临时缓冲区1
  md_.tmp_buffer2_ = vector<double>(buffer_size, 0);  // 临时缓冲区2
  md_.raycast_num_ = 0;  // 射线投射计数器

  // 深度图投影点缓冲区
  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);  // 预分配投影点数组
  md_.proj_points_cnt = 0;  // 投影点计数

  /* 初始化回调函数 */

  // 订阅深度图话题
  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(*node_, "/sdf_map/depth", 50));

  // 根据位姿类型设置不同的消息同步策略
  if (mp_.pose_type_ == POSE_STAMPED) {
    // 使用PoseStamped类型的位姿
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(*node_, "/sdf_map/pose", 25));

    // 同步深度图和位姿消息
    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&SDFMap::depthPoseCallback, this, _1, _2));

  } else if (mp_.pose_type_ == ODOMETRY) {
    // 使用Odometry类型的位姿
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(*node_, "/sdf_map/odom", 100));

    // 同步深度图和里程计消息
    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&SDFMap::depthOdomCallback, this, _1, _2));
  }

  // 使用独立的里程计和点云订阅(不需要同步)

  // 独立订阅点云和里程计(用于点云输入模式)
  indep_cloud_sub_ =
      node_->subscribe<sensor_msgs::PointCloud2>("/sdf_map/cloud", 10, &SDFMap::cloudCallback, this);
  indep_odom_sub_ =
      node_->subscribe<nav_msgs::Odometry>("/sdf_map/odom", 10, &SDFMap::odomCallback, this);

  // 创建定时器，定期执行地图更新和可视化
  occ_timer_ = node_->createTimer(ros::Duration(0.05), &SDFMap::updateOccupancyCallback, this);  // 更新占据网格(20Hz)
  esdf_timer_ = node_->createTimer(ros::Duration(0.05), &SDFMap::updateESDFCallback, this);  // 更新ESDF(20Hz)
  vis_timer_ = node_->createTimer(ros::Duration(0.05), &SDFMap::visCallback, this);  // 可视化(20Hz)

  // 创建发布器
  map_pub_ = node_->advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy", 10);  // 占据网格
  map_inf_pub_ = node_->advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_inflate", 10);  // 膨胀后的占据网格
  esdf_pub_ = node_->advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);  // ESDF
  update_range_pub_ = node_->advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);  // 更新范围可视化

  unknown_pub_ = node_->advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);  // 未知区域
  depth_pub_ = node_->advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);  // 深度点云

  // 初始化状态标志
  md_.occ_need_update_ = false;  // 占据网格需要更新
  md_.local_updated_ = false;  // 局部地图已更新
  md_.esdf_need_update_ = false;  // ESDF需要更新
  md_.has_first_depth_ = false;  // 是否收到第一帧深度图
  md_.has_odom_ = false;  // 是否收到里程计数据
  md_.has_cloud_ = false;  // 是否收到点云数据
  md_.image_cnt_ = 0;  // 图像帧计数

  // 初始化性能统计变量
  md_.esdf_time_ = 0.0;  // ESDF累计更新时间
  md_.fuse_time_ = 0.0;  // 融合累计时间
  md_.update_num_ = 0;  // 更新次数
  md_.max_esdf_time_ = 0.0;  // ESDF最大更新时间
  md_.max_fuse_time_ = 0.0;  // 最大融合时间

  // 初始化随机数生成器(用于深度噪声模拟)
  rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);  // 均匀分布噪声
  rand_noise2_ = normal_distribution<double>(0, 0.2);  // 高斯分布噪声
  random_device rd;
  eng_ = default_random_engine(rd());
}

/**
 * @brief 重置整个地图缓冲区
 *
 * 功能说明：将整个地图范围的缓冲区重置为初始状态
 */
void SDFMap::resetBuffer() {
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

/**
 * @brief 重置指定区域的地图缓冲区
 * @param min_pos 区域最小位置
 * @param max_pos 区域最大位置
 *
 * 功能说明：清空指定区域内的占据网格和距离场
 */
void SDFMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos) {

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);  // 位置转索引
  posToIndex(max_pos, max_id);

  boundIndex(min_id);  // 限制索引在地图范围内
  boundIndex(max_id);

  /* 重置占据网格和距离场缓冲区 */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;  // 清空占据标志
        md_.distance_buffer_[toAddress(x, y, z)] = 10000;  // 重置距离为无穷大
      }
}

/**
 * @brief 沿指定维度填充ESDF (使用Felzenszwalb算法的一维距离变换)
 * @param f_get_val 获取值的函数对象
 * @param f_set_val 设置值的函数对象
 * @param start 起始索引
 * @param end 结束索引
 * @param dim 维度(0=x, 1=y, 2=z)
 *
 * 算法说明：
 * 这是Felzenszwalb和Huttenlocher的快速距离变换算法的实现
 * 时间复杂度O(n)，通过下包络线(lower envelope)计算平方距离变换
 * 分为两个阶段：
 * 1. 前向扫描：构建抛物线的下包络线
 * 2. 后向扫描：根据下包络线计算每个点的最小距离
 */
template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_.map_voxel_num_(dim)];  // 存储抛物线顶点的索引
  double z[mp_.map_voxel_num_(dim) + 1];  // 存储相邻抛物线交点的位置

  // 初始化第一个抛物线
  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();  // 第一个交点在负无穷
  z[start + 1] = std::numeric_limits<double>::max();  // 第二个交点在正无穷

  // 第一阶段：构建下包络线
  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    // 移除被新抛物线遮挡的旧抛物线
    do {
      k--;
      // 计算q处抛物线与v[k]处抛物线的交点
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    // 添加新的抛物线到下包络线
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  // 第二阶段：根据下包络线计算每个点的距离值
  for (int q = start; q <= end; q++) {
    // 找到q点对应的抛物线
    while (z[k + 1] < q) k++;
    // 计算q到v[k]的平方距离
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

/**
 * @brief 更新3D ESDF (欧几里得符号距离场)
 *
 * 算法说明：
 * 使用分离轴算法(Separable Algorithm)计算3D欧几里得距离变换
 * 分三个步骤：
 * 1. 沿Z轴计算1D距离变换
 * 2. 沿Y轴计算1D距离变换
 * 3. 沿X轴计算1D距离变换并转换为实际距离
 * 最后组合正负距离场得到完整的符号距离场
 */
void SDFMap::updateESDF3d() {
  Eigen::Vector3i min_esdf = md_.local_bound_min_;
  Eigen::Vector3i max_esdf = md_.local_bound_max_;

  /* ========== 计算正距离场 (自由空间到障碍物的距离) ========== */

  // 第一步：沿Z轴进行距离变换
  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            // 如果是障碍物，距离为0；否则为无穷大
            return md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  // 第三步：沿X轴进行距离变换，并转换为实际距离(米)
  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 // 将平方距离转换为实际距离(乘以分辨率)
                 md_.distance_buffer_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
                 //  min(mp_.resolution_ * std::sqrt(val),
                 //      md_.distance_buffer_[toAddress(x, y, z)]);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== 计算负距离场 (障碍物内部到边界的距离) ========== */
  // 反转占据网格：自由空间变为障碍物，障碍物变为自由空间
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        if (md_.occupancy_buffer_inflate_[idx] == 0) {
          md_.occupancy_buffer_neg[idx] = 1;  // 自由空间反转为障碍物

        } else if (md_.occupancy_buffer_inflate_[idx] == 1) {
          md_.occupancy_buffer_neg[idx] = 0;  // 障碍物反转为自由空间
        } else {
          ROS_ERROR("what?");
        }
      }

  ros::Time t1, t2;

  // 对反转后的占据网格沿Z轴进行距离变换
  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            return md_.occupancy_buffer_neg[x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                            y * mp_.map_voxel_num_(2) + z] == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  // 沿Y轴进行距离变换
  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  // 沿X轴进行距离变换，得到负距离场
  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 md_.distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== 组合正负距离场得到完整的符号距离场 ========== */
  // 组合正负距离场：
  // - 正距离表示自由空间到最近障碍物的距离(正值)
  // - 负距离表示障碍物内部到边界的距离(负值)
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

        // 如果点在障碍物内部，将距离设为负值
        if (md_.distance_buffer_neg_[idx] > 0.0)
          md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
      }
}

/**
 * @brief 缓存体素的占据状态更新
 * @param pos 体素位置
 * @param occ 占据状态 (1=占据, 0=自由)
 * @return 体素的线性索引，如果参数无效则返回INVALID_IDX
 *
 * 功能说明：
 * 将射线投射过程中观测到的占据信息暂存到缓存中
 * 使用计数方式统计每个体素被观测到的次数和被击中的次数
 * 这些统计信息将在raycastProcess()结束时用于更新占据概率
 */
int SDFMap::setCacheOccupancy(Eigen::Vector3d pos, int occ) {
  if (occ != 1 && occ != 0) return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;  // 增加观测总次数

  // 如果是第一次观测到这个体素，加入缓存队列
  if (md_.count_hit_and_miss_[idx_ctns] == 1) {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1) md_.count_hit_[idx_ctns] += 1;  // 如果被击中，增加击中次数

  return idx_ctns;
}

/**
 * @brief 将深度图投影到3D空间
 *
 * 功能说明：
 * 1. 遍历深度图的每个像素(或按skip_pixel跳过采样)
 * 2. 使用相机内参将像素坐标和深度值转换为相机坐标系下的3D点
 * 3. 使用相机位姿将3D点转换到世界坐标系
 * 4. 可选：使用深度滤波器过滤不一致或无效的深度值
 */
void SDFMap::projectDepthImage() {
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;  // 重置投影点计数

  uint16_t* row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();  // 相机旋转矩阵

  // cout << "rotate: " << md_.camera_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.camera_pos_ << std::endl;

  // 模式1：不使用深度滤波，直接投影所有像素
  if (!mp_.use_depth_filter_) {
    for (int v = 0; v < rows; v++) {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u++) {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;  // 深度值缩放
        // 使用针孔相机模型将像素坐标转换为相机坐标系
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        // 转换到世界坐标系
        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240) std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* 模式2：使用深度滤波 */
  else {

    // 第一帧深度图直接标记，不进行滤波
    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else {
      // 从第二帧开始进行深度滤波
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_q_.inverse();  // 上一帧相机旋转的逆
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      // 跳过边缘像素，并按skip_pixel降采样
      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_) {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_) {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // 深度滤波 (可选：添加噪声用于仿真测试)
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          // 处理无效深度值
          if (*row_ptr == 0) {
            depth = mp_.max_ray_length_ + 0.1;  // 无深度值，设为最大射线长度
          } else if (depth < mp_.depth_filter_mindist_) {
            continue;  // 深度太近，跳过
          } else if (depth > mp_.depth_filter_maxdist_) {
            depth = mp_.max_ray_length_ + 0.1;  // 深度太远，设为最大射线长度
          }

          // 投影到世界坐标系
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // 深度一致性检查 (当前禁用)
          // 将当前点重投影到上一帧，检查深度是否一致
          if (false) {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows) {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_) {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            } else {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* 保存当前相机位姿和深度图，用于下一帧的一致性检查 */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_q_ = md_.camera_q_;
  md_.last_depth_image_ = md_.depth_image_;
}

/**
 * @brief 射线投射处理
 *
 * 功能说明：
 * 1. 对每个投影点，从相机中心向该点发射射线
 * 2. 沿射线标记穿过的体素为自由空间
 * 3. 标记射线终点为占据或自由(取决于深度值)
 * 4. 使用log-odds更新占据概率
 * 5. 计算局部地图更新边界
 */
void SDFMap::raycastProcess() {
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0) return;  // 没有投影点，直接返回

  ros::Time t1, t2;

  md_.raycast_num_ += 1;  // 射线投射计数器，用于去重标记

  int vox_idx;
  double length;

  // 初始化更新区域的包围盒
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;  // Bresenham 3D射线投射器
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);  // 体素中心偏移
  Eigen::Vector3d ray_pt, pt_w;

  // 遍历所有投影点
  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt_w = md_.proj_points_[i];

    // 处理投影点并设置占据标志

    if (!isInMap(pt_w)) {
      // 点在地图外，找到射线与地图边界的交点
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_) {
        // 射线长度超过最大值，截断
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);  // 标记为自由

    } else {
      // 点在地图内
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_) {
        // 射线长度超过最大值，截断并标记为自由
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      } else {
        // 有效深度值，标记为占据
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    // 更新包围盒范围
    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // 从相机中心到投影点进行射线投射

    // 检查射线终点是否已经处理过(去重)
    if (vox_idx != INVALID_IDX) {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_) {
        continue;  // 已处理过，跳过
      } else {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;  // 标记为已处理
      }
    }

    // 设置射线投射器的起点和终点(转换到体素索引空间)
    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    // 沿射线迭代，标记穿过的体素为自由空间
    while (raycaster.step(ray_pt)) {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;  // 体素中心位置
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);  // 标记为自由

      // 检查是否已经穿过这个体素(去重)
      if (vox_idx != INVALID_IDX) {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_) {
          break;  // 已穿过，停止(避免重复处理)
        } else {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;  // 标记为已穿过
        }
      }
    }
  }

  // 确定ESDF更新的局部包围盒
  // 包围盒需要包含所有投影点和相机位置
  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);  // 至少包含地面高度

  // 转换为体素索引
  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  // 膨胀包围盒以考虑ESDF计算需要的额外边界
  int esdf_inf = ceil(mp_.local_bound_inflate_ / mp_.resolution_);
  md_.local_bound_max_ += esdf_inf * Eigen::Vector3i(1, 1, 0);  // xy方向膨胀
  md_.local_bound_min_ -= esdf_inf * Eigen::Vector3i(1, 1, 0);
  boundIndex(md_.local_bound_min_);  // 限制在地图范围内
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;  // 标记局部地图已更新

  // 更新缓存队列中的占据概率
  // 定义局部更新范围(以相机为中心的立方体区域)
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  // 处理缓存队列中的所有体素
  while (!md_.cache_voxel_.empty()) {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    // 根据击中/未击中次数决定log-odds更新值
    // 如果击中次数>=未击中次数，使用hit概率；否则使用miss概率
    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ?
        mp_.prob_hit_log_ :
        mp_.prob_miss_log_;

    // 重置计数器
    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    // 检查是否达到上下限
    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_) {
      continue;  // 已达到最大占据概率
    } else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;  // 已达到最小占据概率
    }

    // 检查体素是否在局部更新范围内
    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
        idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local) {
      // 不在局部范围内，重置为最小概率
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    // 使用log-odds进行贝叶斯更新，并限制在[min, max]范围内
    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

/**
 * @brief 找到从相机到目标点的射线与地图边界的最近交点
 * @param pt 目标点(在地图外)
 * @param camera_pt 相机位置(射线起点)
 * @return 射线与地图边界的交点
 *
 * 算法说明：
 * 计算射线与地图6个面(x_min, x_max, y_min, y_max, z_min, z_max)的交点
 * 返回最近的有效交点(t>0且最小)
 */
Eigen::Vector3d SDFMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;  // 射线方向
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;  // 最小参数t

  // 对每个坐标轴，计算射线与对应平面的交点
  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {

      // 与最大边界面的交点参数
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;

      // 与最小边界面的交点参数
      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }

  // 返回交点，稍微向内偏移1e-3避免边界问题
  return camera_pt + (min_t - 1e-3) * diff;
}

/**
 * @brief 清空局部地图外的区域并膨胀障碍物
 *
 * 功能说明：
 * 1. 清空局部范围外的占据数据，释放内存
 * 2. 对局部范围内的障碍物进行膨胀，以补偿机器人尺寸
 * 3. 添加虚拟天花板限制飞行高度(如果配置)
 */
void SDFMap::clearAndInflateLocalMap() {
  /*清空局部范围外的区域*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  // 计算清空区域的范围
  Eigen::Vector3i min_cut = md_.local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  // 添加额外裕度
  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // 清空局部范围外的数据(分Z、Y、X三个方向清空边界区域)

  // 清空Z方向上下边界外的区域
  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {

      // 清空下边界外
      for (int z = min_cut_m(2); z < min_cut(2); ++z) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;  // 标记为未知
        md_.distance_buffer_all_[idx] = 10000;  // 重置距离
      }

      // 清空上边界外
      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  // 清空Y方向左右边界外的区域
  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {

      // 清空左边界外
      for (int y = min_cut_m(1); y < min_cut(1); ++y) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      // 清空右边界外
      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  // 清空X方向前后边界外的区域
  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {

      // 清空前边界外
      for (int x = min_cut_m(0); x < min_cut(0); ++x) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      // 清空后边界外
      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  // 膨胀占据体素以补偿机器人尺寸

  // 计算膨胀半径(体素数)
  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));  // 预分配膨胀点数组
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // 清空旧的膨胀数据
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // 对所有占据体素进行膨胀
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {

        // 如果体素被占据(概率大于阈值)
        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_) {
          // 获取该点周围的膨胀点
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          // 标记所有膨胀点为占据
          for (int k = 0; k < (int)inf_pts.size(); ++k) {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            // 检查索引有效性
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // 添加虚拟天花板以限制飞行高度
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

/**
 * @brief 可视化定时器回调函数
 * @param event 定时器事件(未使用)
 *
 * 功能说明：定期发布地图可视化消息供RViz显示
 */
void SDFMap::visCallback(const ros::TimerEvent& /*event*/) {
  publishMap();  // 发布占据网格
  publishMapInflate(false);  // 发布膨胀后的占据网格
  // publishUpdateRange();  // 发布更新范围(调试用)
  // publishESDF();  // 发布ESDF切片(调试用)

  // publishUnknown();  // 发布未知区域(调试用)
  // publishDepth();  // 发布深度点云(调试用)
}

/**
 * @brief 占据网格更新定时器回调函数
 * @param event 定时器事件(未使用)
 *
 * 功能说明：
 * 1. 将深度图投影到3D空间
 * 2. 执行射线投射更新占据概率
 * 3. 清空局部地图外的区域并膨胀障碍物
 */
void SDFMap::updateOccupancyCallback(const ros::TimerEvent& /*event*/) {
  if (!md_.occ_need_update_) return;  // 没有新数据，跳过

  /* 更新占据网格 */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  projectDepthImage();  // 将深度图投影为3D点云
  raycastProcess();  // 射线投射更新占据概率

  if (md_.local_updated_) clearAndInflateLocalMap();  // 清空外部区域并膨胀障碍物

  t2 = ros::Time::now();

  // 统计性能数据
  md_.fuse_time_ += (t2 - t1).toSec();
  md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  if (mp_.show_occ_time_)
    ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  if (md_.local_updated_) md_.esdf_need_update_ = true;  // 触发ESDF更新
  md_.local_updated_ = false;
}

/**
 * @brief ESDF更新定时器回调函数
 * @param event 定时器事件(未使用)
 *
 * 功能说明：根据最新的占据网格计算欧几里得符号距离场
 */
void SDFMap::updateESDFCallback(const ros::TimerEvent& /*event*/) {
  if (!md_.esdf_need_update_) return;  // 不需要更新，跳过

  /* 更新ESDF */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  updateESDF3d();  // 计算3D ESDF

  t2 = ros::Time::now();

  // 统计性能数据
  md_.esdf_time_ += (t2 - t1).toSec();
  md_.max_esdf_time_ = max(md_.max_esdf_time_, (t2 - t1).toSec());

  if (mp_.show_esdf_time_)
    ROS_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             md_.esdf_time_ / md_.update_num_, md_.max_esdf_time_);

  md_.esdf_need_update_ = false;
}

/**
 * @brief 深度图和位姿同步回调函数
 * @param img 深度图消息
 * @param pose 位姿消息(PoseStamped类型)
 *
 * 功能说明：
 * 接收同步的深度图和相机位姿，准备更新占据网格
 */
void SDFMap::depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  /* 获取深度图 */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  // 如果深度图是float32类型，转换为uint16类型
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* 获取相机位姿 */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                     pose->pose.orientation.y, pose->pose.orientation.z);
  // 检查相机是否在地图内
  if (isInMap(md_.camera_pos_)) {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;  // 触发占据网格更新
  } else {
    md_.occ_need_update_ = false;  // 相机在地图外，跳过更新
  }
}

/**
 * @brief 独立里程计回调函数
 * @param odom 里程计消息
 *
 * 功能说明：在点云模式下接收相机位姿(仅在未收到深度图时使用)
 */
void SDFMap::odomCallback(const nav_msgs::OdometryConstPtr& odom) {
  if (md_.has_first_depth_) return;  // 已收到深度图，不再需要

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;
}

/**
 * @brief 点云回调函数
 * @param img 点云消息
 *
 * 功能说明：
 * 直接使用点云数据更新地图(不需要深度图投影)
 * 1. 将点云中的点直接标记为障碍物
 * 2. 对障碍物进行膨胀
 * 3. 触发ESDF更新
 */
void SDFMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img) {

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  // 检查是否有有效的位姿数据
  if (!md_.has_odom_) {
    // std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0) return;  // 空点云，跳过

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2))) return;  // 位姿无效

  // 重置局部更新范围内的缓冲区
  this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                    md_.camera_pos_ + mp_.local_update_range_);

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);  // 水平方向膨胀半径
  int inf_step_z = 1;  // 垂直方向膨胀半径(较小)

  double max_x, max_y, max_z, min_x, min_y, min_z;

  // 初始化包围盒
  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  // 遍历点云中的所有点
  for (size_t i = 0; i < latest_cloud.points.size(); ++i) {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* 检查点是否在局部更新范围内 */
    Eigen::Vector3d devi = p3d - md_.camera_pos_;
    Eigen::Vector3i inf_pt;

    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
        fabs(devi(2)) < mp_.local_update_range_(2)) {

      /* 膨胀该点 */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z) {

            // 计算膨胀点的位置
            p3d_inf(0) = pt.x + x * mp_.resolution_;
            p3d_inf(1) = pt.y + y * mp_.resolution_;
            p3d_inf(2) = pt.z + z * mp_.resolution_;

            // 更新包围盒
            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            posToIndex(p3d_inf, inf_pt);

            if (!isInMap(inf_pt)) continue;  // 点在地图外，跳过

            int idx_inf = toAddress(inf_pt);

            md_.occupancy_buffer_inflate_[idx_inf] = 1;  // 标记为占据
          }
    }
  }

  // 包围盒需要包含相机位置
  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);  // 至少包含地面高度

  // 转换为体素索引
  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.esdf_need_update_ = true;  // 触发ESDF更新
}

/**
 * @brief 发布占据地图可视化
 *
 * 功能说明：将膨胀后的占据网格转换为点云消息发布给RViz显示
 */
void SDFMap::publishMap() {
  // 以下是发布原始占据概率的代码(已禁用)
  // pcl::PointXYZ pt;
  // pcl::PointCloud<pcl::PointXYZ> cloud;

  // Eigen::Vector3i min_cut = md_.local_bound_min_ -
  //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  // Eigen::Vector3i max_cut = md_.local_bound_max_ +
  //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);

  // boundIndex(min_cut);
  // boundIndex(max_cut);

  // for (int x = min_cut(0); x <= max_cut(0); ++x)
  //   for (int y = min_cut(1); y <= max_cut(1); ++y)
  //     for (int z = min_cut(2); z <= max_cut(2); ++z) {

  //       if (md_.occupancy_buffer_[toAddress(x, y, z)] <= mp_.min_occupancy_log_) continue;

  //       Eigen::Vector3d pos;
  //       indexToPos(Eigen::Vector3i(x, y, z), pos);
  //       if (pos(2) > mp_.visualization_truncate_height_) continue;

  //       pt.x = pos(0);
  //       pt.y = pos(1);
  //       pt.z = pos(2);
  //       cloud.points.push_back(pt);
  //     }

  // cloud.width = cloud.points.size();
  // cloud.height = 1;
  // cloud.is_dense = true;
  // cloud.header.frame_id = mp_.frame_id_;

  // sensor_msgs::PointCloud2 cloud_msg;
  // pcl::toROSMsg(cloud, cloud_msg);
  // map_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");

  // 发布膨胀后的占据网格
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // 计算发布范围(局部边界加一半裕度)
  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  // 遍历所有体素，将占据的体素加入点云
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;  // 跳过自由空间

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;  // 跳过高于截断高度的点

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  // 设置点云属性并发布
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

/**
 * @brief 发布膨胀后的占据地图可视化
 * @param all_info 是否发布完整信息(包含裕度区域)
 *
 * 功能说明：发布膨胀后的占据网格用于可视化
 */
void SDFMap::publishMapInflate(bool all_info) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  // 如果需要完整信息，扩大发布范围
  if (all_info) {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  // 遍历所有体素，将占据的体素加入点云
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;  // 跳过自由空间

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;  // 跳过高于截断高度的点

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  // 设置点云属性并发布
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

/**
 * @brief 发布未知区域可视化(调试用)
 *
 * 功能说明：发布占据概率低于最小阈值的体素(未被观测到的区域)
 */
void SDFMap::publishUnknown() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  // 遍历所有体素，找出未知区域
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {

        // 占据概率小于最小值说明是未知区域
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > mp_.visualization_truncate_height_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  // 输出未知区域比例(调试用)
  // auto sz = max_cut - min_cut;
  // std::cout << "unknown ratio: " << cloud.width << "/" << sz(0) * sz(1) * sz(2) << "="
  //           << double(cloud.width) / (sz(0) * sz(1) * sz(2)) << std::endl;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

/**
 * @brief 发布深度点云可视化(调试用)
 *
 * 功能说明：发布从深度图投影得到的原始3D点云
 */
void SDFMap::publishDepth() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // 将所有投影点加入点云
  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt.x = md_.proj_points_[i][0];
    pt.y = md_.proj_points_[i][1];
    pt.z = md_.proj_points_[i][2];
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_.publish(cloud_msg);
}

/**
 * @brief 发布更新范围可视化(调试用)
 *
 * 功能说明：用半透明红色立方体表示当前的局部更新边界
 */
void SDFMap::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;

  // 将索引转换为位置
  indexToPos(md_.local_bound_min_, esdf_min_pos);
  indexToPos(md_.local_bound_max_, esdf_max_pos);

  // 计算立方体中心和尺寸
  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;

  // 设置Marker属性
  mk.header.frame_id = mp_.frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);

  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);

  mk.color.a = 0.3;  // 半透明
  mk.color.r = 1.0;  // 红色
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

/**
 * @brief 发布ESDF切片可视化(调试用)
 *
 * 功能说明：
 * 在指定高度切一个水平平面，将该平面上每个点到最近障碍物的距离
 * 用颜色强度表示(强度越大距离越远)
 */
void SDFMap::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;  // 带强度的点云
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  // 计算发布范围
  Eigen::Vector3i min_cut = md_.local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  // 在xy平面上采样，Z设置为切片高度
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {

      Eigen::Vector3d pos;
      indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = mp_.esdf_slice_height_;  // 设置为切片高度

      // 查询该点的距离值
      dist = getDistance(pos);
      dist = min(dist, max_dist);  // 限制在[min_dist, max_dist]范围
      dist = max(dist, min_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;  // 显示在稍低的高度
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);  // 归一化距离作为强度
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}

/**
 * @brief 获取指定高度的ESDF切片
 * @param height 切片高度
 * @param res 采样分辨率
 * @param range 采样范围 [x_min, x_max, y_min, y_max]
 * @param slice 输出：每个采样点的(x, y, dist)
 * @param grad 输出：每个采样点的梯度
 * @param sign 符号(未使用)
 *
 * 功能说明：在指定高度的水平面上均匀采样，查询每个点的距离值和梯度
 */
void SDFMap::getSliceESDF(const double height, const double res, const Eigen::Vector4d& range,
                          vector<Eigen::Vector3d>& slice, vector<Eigen::Vector3d>& grad, int sign) {
  double dist;
  Eigen::Vector3d gd;
  // 在xy平面上均匀采样
  for (double x = range(0); x <= range(1); x += res)
    for (double y = range(2); y <= range(3); y += res) {

      // 查询距离和梯度(使用三线性插值)
      dist = this->getDistWithGradTrilinear(Eigen::Vector3d(x, y, height), gd);
      slice.push_back(Eigen::Vector3d(x, y, dist));
      grad.push_back(gd);
    }
}

/**
 * @brief 检查距离场是否有异常值(调试用)
 *
 * 功能说明：遍历所有体素，检查距离值是否超过阈值
 */
void SDFMap::checkDist() {
  for (int x = 0; x < mp_.map_voxel_num_(0); ++x)
    for (int y = 0; y < mp_.map_voxel_num_(1); ++y)
      for (int z = 0; z < mp_.map_voxel_num_(2); ++z) {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        Eigen::Vector3d grad;
        double dist = getDistWithGradTrilinear(pos, grad);

        // 检查是否有异常值(这里只检查但不输出)
        if (fabs(dist) > 10.0) {
        }
      }
}

// 查询函数：返回状态标志
bool SDFMap::odomValid() { return md_.has_odom_; }

bool SDFMap::hasDepthObservation() { return md_.has_first_depth_; }

// 查询函数：返回地图参数
double SDFMap::getResolution() { return mp_.resolution_; }

Eigen::Vector3d SDFMap::getOrigin() { return mp_.map_origin_; }

int SDFMap::getVoxelNum() {
  return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
}

void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
  ori = mp_.map_origin_, size = mp_.map_size_;
}

/**
 * @brief 获取用于三线性插值的周围8个体素位置
 * @param pos 查询位置
 * @param pts 输出：周围8个体素的位置 pts[x][y][z], x,y,z ∈ {0,1}
 * @param diff 输出：查询点在单位立方体内的归一化位置
 *
 * 功能说明：
 * 用于三线性插值的准备工作，找到包含查询点的体素立方体的8个顶点
 */
void SDFMap::getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2],
                            Eigen::Vector3d& diff) {
  if (!isInMap(pos)) {
    // cout << "pos invalid for interpolation." << endl;
  }

  /* 计算插值位置 */
  // 将查询点向体素中心对齐
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  Eigen::Vector3d idx_pos;

  posToIndex(pos_m, idx);  // 找到左下角体素索引
  indexToPos(idx, idx_pos);  // 转回位置
  diff = (pos - idx_pos) * mp_.resolution_inv_;  // 计算归一化偏移量

  // 获取周围8个体素的位置
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        Eigen::Vector3d current_pos;
        indexToPos(current_idx, current_pos);
        pts[x][y][z] = current_pos;
      }
    }
  }
}

/**
 * @brief 深度图和里程计同步回调函数
 * @param img 深度图消息
 * @param odom 里程计消息(Odometry类型)
 *
 * 功能说明：接收同步的深度图和相机位姿，准备更新占据网格
 */
void SDFMap::depthOdomCallback(const sensor_msgs::ImageConstPtr& img,
                               const nav_msgs::OdometryConstPtr& odom) {
  /* 获取相机位姿 */
  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  /* 获取深度图 */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;  // 触发占据网格更新
}

/**
 * @brief 独立深度图回调函数(调试用)
 * @param img 深度图消息
 */
void SDFMap::depthCallback(const sensor_msgs::ImageConstPtr& img) {
  std::cout << "depth: " << img->header.stamp << std::endl;
}

/**
 * @brief 独立位姿回调函数(调试用)
 * @param pose 位姿消息
 */
void SDFMap::poseCallback(const geometry_msgs::PoseStampedConstPtr& pose) {
  std::cout << "pose: " << pose->header.stamp << std::endl;

  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
}

// SDFMap
