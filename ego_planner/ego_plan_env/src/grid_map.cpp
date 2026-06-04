#include "ego_plan_env/grid_map.h"

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

/**
 * @brief 初始化栅格地图
 * @param nh ROS节点句柄
 *
 * 该函数负责初始化栅格地图的所有参数，包括：
 * 1. 地图尺寸和分辨率
 * 2. 相机内参
 * 3. 深度滤波参数
 * 4. 概率占用参数
 * 5. 初始化数据缓冲区
 * 6. 设置ROS回调函数
 */
void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* 获取地图参数 */
  double x_size, y_size, z_size;
  // 地图分辨率：每个体素的尺寸（米）
  node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  // 地图尺寸：X、Y、Z方向的总长度（米）
  node_.param("grid_map/map_size_x", x_size, -1.0);
  node_.param("grid_map/map_size_y", y_size, -1.0);
  node_.param("grid_map/map_size_z", z_size, -1.0);
  // 局部更新范围：以相机为中心的局部更新区域尺寸
  node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  // 障碍物膨胀距离：用于考虑机器人尺寸的安全边界
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

  // 相机内参：用于将深度图投影到3D空间
  node_.param("grid_map/fx", mp_.fx_, -1.0);  // X方向焦距
  node_.param("grid_map/fy", mp_.fy_, -1.0);  // Y方向焦距
  node_.param("grid_map/cx", mp_.cx_, -1.0);  // X方向光心
  node_.param("grid_map/cy", mp_.cy_, -1.0);  // Y方向光心

  // 深度滤波参数：用于提高深度数据质量和一致性
  node_.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);  // 是否启用深度滤波
  node_.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);  // 深度一致性容差
  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);  // 最大有效深度
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);  // 最小有效深度
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);  // 图像边缘裕度
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);  // 深度缩放因子
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);  // 像素跳跃间隔（降采样）

  // 概率占用栅格地图参数（基于对数几率模型）
  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);    // 击中概率：传感器检测到障碍物时的更新概率
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);  // 未击中概率：射线穿过自由空间的更新概率
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);    // 最小占用概率：下界限制
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);    // 最大占用概率：上界限制
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);    // 占用阈值：超过此值认为是障碍物
  node_.param("grid_map/min_ray_length", mp_.min_ray_length_, -0.1);  // 最小射线长度
  node_.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);  // 最大射线长度

  // 可视化参数
  node_.param("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_, 999.0);  // 可视化截断高度
  node_.param("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);  // 虚拟天花板高度限制

  // 其他配置参数
  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);  // 是否显示占用更新时间
  node_.param("grid_map/pose_type", mp_.pose_type_, 1);  // 位姿类型：1为PoseStamped，2为Odometry

  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));  // 坐标系ID
  node_.param("grid_map/local_map_margin", mp_.local_map_margin_, 1);  // 局部地图边界裕度
  node_.param("grid_map/ground_height", mp_.ground_height_, 1.0);  // 地面高度

  // 计算派生参数
  mp_.resolution_inv_ = 1 / mp_.resolution_;  // 分辨率的倒数，用于快速计算
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);  // 地图原点（中心对齐）
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);  // 地图尺寸向量

  // 将概率值转换为对数几率（log-odds）表示，用于概率占用栅格地图更新
  // logit(p) = log(p / (1 - p))
  mp_.prob_hit_log_ = logit(mp_.p_hit_);      // 击中的对数几率
  mp_.prob_miss_log_ = logit(mp_.p_miss_);    // 未击中的对数几率
  mp_.clamp_min_log_ = logit(mp_.p_min_);     // 最小占用概率的对数几率
  mp_.clamp_max_log_ = logit(mp_.p_max_);     // 最大占用概率的对数几率
  mp_.min_occupancy_log_ = logit(mp_.p_occ_); // 占用阈值的对数几率
  mp_.unknown_flag_ = 0.01;  // 未知区域标志值

  // 打印对数几率参数，用于调试
  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  // 计算每个维度的体素数量
  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  // 设置地图边界
  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  /* 初始化数据缓冲区 */

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  // 占用概率缓冲区：存储每个体素的对数几率值，初始化为未知状态
  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  // 膨胀后的占用缓冲区：二值化的占用信息（0=自由，1=占用）
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

  // 射线投射统计缓冲区
  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);  // 总的观测次数
  md_.count_hit_ = vector<short>(buffer_size, 0);           // 击中次数
  md_.flag_rayend_ = vector<char>(buffer_size, -1);         // 射线端点标记
  md_.flag_traverse_ = vector<char>(buffer_size, -1);       // 射线穿越标记

  md_.raycast_num_ = 0;  // 射线投射计数器

  // 投影点云缓冲区：根据图像分辨率和跳跃像素计算大小
  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;
  // 相机到机体的变换矩阵（固定变换）
  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, -0.02,
      0.0, 0.0, 0.0, 1.0;

  /* 初始化ROS回调函数 */

  // 订阅深度图话题
  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/grid_map/depth", 50));

  // 根据位姿类型选择不同的同步策略
  if (mp_.pose_type_ == POSE_STAMPED)
  {
    // 使用PoseStamped类型的位姿
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/grid_map/pose", 25));

    // 同步深度图和位姿
    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    // 使用Odometry类型的位姿
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/grid_map/odom", 100));

    // 同步深度图和里程计
    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
  }

  // 独立订阅点云和里程计（用于直接点云建图）
  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("/grid_map/cloud", 10, &GridMap::cloudCallback, this);
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("/grid_map/odom", 10, &GridMap::odomCallback, this);

  // 定时器：定期更新占用栅格和可视化
  occ_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this);  // 20Hz
  vis_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::visCallback, this);              // 20Hz

  // 发布者：发布地图点云
  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy", 10);          // 原始占用地图
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/occupancy_inflate", 10);  // 膨胀后的占用地图
  unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/grid_map/unknown", 10);        // 未知区域

  // 初始化状态标志
  md_.occ_need_update_ = false;    // 是否需要更新占用
  md_.local_updated_ = false;      // 局部地图是否已更新
  md_.has_first_depth_ = false;    // 是否接收到第一帧深度图
  md_.has_odom_ = false;           // 是否接收到里程计数据
  md_.has_cloud_ = false;          // 是否接收到点云数据
  md_.image_cnt_ = 0;              // 图像计数器

  // 性能统计变量
  md_.fuse_time_ = 0.0;       // 融合总时间
  md_.update_num_ = 0;        // 更新次数
  md_.max_fuse_time_ = 0.0;   // 最大融合时间

  // 随机噪声生成器（已禁用）
  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

/**
 * @brief 重置整个地图的缓冲区
 *
 * 重置整个地图范围内的膨胀占用缓冲区，并更新局部边界为整个地图
 */
void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

/**
 * @brief 重置指定区域的缓冲区
 * @param min_pos 区域最小位置（世界坐标）
 * @param max_pos 区域最大位置（世界坐标）
 *
 * 将指定区域内的膨胀占用缓冲区清零，用于清除过时的障碍物信息
 */
void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);  // 世界坐标转换为体素索引
  posToIndex(max_pos, max_id);

  boundIndex(min_id);  // 确保索引在地图范围内
  boundIndex(max_id);

  /* 重置膨胀占用缓冲区 */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
}

/**
 * @brief 缓存体素的占用观测
 * @param pos 体素位置（世界坐标）
 * @param occ 占用标志（1=击中/障碍物，0=未击中/自由空间）
 * @return 体素索引，如果occ值无效则返回INVALID_IDX
 *
 * 该函数用于累积对某个体素的观测：
 * 1. 记录该体素被观测的次数
 * 2. 如果是第一次观测，将其加入缓存队列
 * 3. 如果是击中（障碍物），增加击中计数
 */
int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;  // 增加总观测次数

  // 如果是首次观测该体素，加入缓存队列
  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  // 如果是击中（障碍物），增加击中计数
  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

/**
 * @brief 将深度图投影到3D空间
 *
 * 该函数将深度图中的像素投影为3D点云：
 * 1. 使用相机内参将像素坐标和深度值转换为相机坐标系下的3D点
 * 2. 使用相机位姿将点转换到世界坐标系
 * 3. 如果启用深度滤波，进行深度一致性检查（与上一帧对比）
 */
void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;  // 重置投影点计数

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  // 相机旋转矩阵：从相机坐标系到世界坐标系
  Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();

  // cout << "rotate: " << md_.camera_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.camera_pos_ << std::endl;

  // 如果不使用深度滤波，直接投影所有像素
  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v++)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u++)
      {

        Eigen::Vector3d proj_pt;
        // 深度值缩放
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        // 针孔相机模型：像素坐标 -> 相机坐标
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        // 转换到世界坐标系
        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* 使用深度滤波 */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;  // 标记已接收第一帧
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      // 上一帧相机旋转的逆矩阵，用于深度一致性检查
      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_q_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      // 跳过图像边缘的像素，并按skip_pixel降采样
      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // 深度滤波（已禁用）
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          // 深度值有效性检查
          if (*row_ptr == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;  // 无效深度，设为最大射线长度
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;  // 深度太小，跳过
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;  // 深度太大，设为最大射线长度
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

          // 与上一帧图像的深度一致性检查（已禁用）
          if (false)
          {
            // 将世界坐标点重投影到上一帧相机坐标系
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            // 检查重投影点是否在图像范围内
            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              // 比较当前深度与上一帧对应位置的深度差异
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* 保存当前帧信息，用于下一帧的一致性检查 */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_q_ = md_.camera_q_;
  md_.last_depth_image_ = md_.depth_image_;
}

/**
 * @brief 射线投射处理
 *
 * 该函数是概率占用栅格地图更新的核心：
 * 1. 对每个投影的3D点，从相机中心发射射线到该点
 * 2. 射线穿过的体素标记为自由空间（未击中）
 * 3. 射线终点（障碍物）标记为占用（击中）
 * 4. 使用对数几率模型累积更新每个体素的占用概率
 * 5. 计算更新区域的边界框
 */
void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  ros::Time t1, t2;

  md_.raycast_num_ += 1;  // 射线投射计数器递增

  int vox_idx;
  double length;

  // 更新区域的边界框（用于后续的局部地图更新）
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;  // 3D射线投射器
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);  // 体素中心偏移
  Eigen::Vector3d ray_pt, pt_w;

  // 遍历所有投影点
  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    /* 处理投影点的占用状态 */

    if (!isInMap(pt_w))
    {
      // 点在地图外，找到射线与地图边界的交点
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        // 超过最大射线长度，截断
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);  // 标记为自由空间
    }
    else
    {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        // 超过最大射线长度，截断并标记为自由空间
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        // 在有效范围内，标记为障碍物（击中）
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    // 更新边界框
    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    /* 从相机中心到投影点之间进行射线投射 */

    if (vox_idx != INVALID_IDX)
    {
      // 检查该体素是否已经被当前帧的射线终点标记过
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;  // 已处理过，跳过
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;  // 标记为当前帧的射线终点
      }
    }

    // 设置射线投射器：从相机位置到投影点（转换为体素坐标）
    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    // 逐步遍历射线经过的体素
    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;  // 转换回世界坐标
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);  // 射线穿过的体素标记为自由空间

      if (vox_idx != INVALID_IDX)
      {
        // 检查该体素是否已经被当前帧的射线穿越过
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;  // 已穿越过，提前终止（避免重复处理）
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;  // 标记为已穿越
        }
      }
    }
  }

  // 将相机位置也纳入边界框
  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);  // 确保包含地面高度

  // 转换为体素索引并保存局部边界
  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;  // 标记局部地图已更新

  /* 更新缓存队列中体素的占用概率 */
  // 定义局部更新范围
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  // 处理缓存队列中的所有体素
  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    // 根据击中和未击中次数确定对数几率更新值
    // 如果击中次数 >= 未击中次数，使用prob_hit_log_，否则使用prob_miss_log_
    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    // 重置计数器
    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    // 检查是否已达到概率上下界
    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;  // 已达到最大值，跳过
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;  // 已达到最小值，跳过
    }

    // 检查体素是否在局部更新范围内
    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      // 不在局部范围内的体素重置为最小占用概率
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    // 使用对数几率模型更新占用概率，并限制在[clamp_min, clamp_max]范围内
    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

/**
 * @brief 找到从相机到点的射线与地图边界的最近交点
 * @param pt 目标点（可能在地图外）
 * @param camera_pt 相机位置
 * @return 射线与地图边界的交点
 *
 * 该函数计算从camera_pt到pt的射线与地图边界框的交点。
 * 使用参数化射线方程：P = camera_pt + t * (pt - camera_pt)
 * 找到最小的正参数t，使得射线与地图边界相交
 */
Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;  // 射线方向向量
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;  // 到最大边界的向量
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;  // 到最小边界的向量

  double min_t = 1000000;  // 初始化为大值

  // 对每个轴（X, Y, Z）计算射线与边界面的交点参数
  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];  // 与最大边界面的交点参数
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];  // 与最小边界面的交点参数
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  // 返回交点，稍微向内偏移1e-3以确保在地图内
  return camera_pt + (min_t - 1e-3) * diff;
}

/**
 * @brief 清理局部地图外的区域并膨胀障碍物
 *
 * 该函数执行两个主要任务：
 * 1. 清理局部地图范围外的占用数据（设为未知）
 * 2. 对局部地图内的障碍物进行膨胀，以考虑机器人尺寸
 */
void GridMap::clearAndInflateLocalMap()
{
  /* 清理局部地图外的区域 */
  const int vec_margin = 5;  // 额外的裕度
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  // 计算局部地图的边界（加上裕度）
  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  // 计算清理区域的边界（再加上额外裕度）
  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // 清理局部范围外的数据，按三个轴分别处理

  // Z轴方向：清理上下边界外的区域
  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;  // 设为未知
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // Y轴方向：清理前后边界外的区域
  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // X轴方向：清理左右边界外的区域
  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  /* 膨胀占用体素以补偿机器人尺寸 */

  // 计算膨胀步长（体素数）
  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  // 预分配膨胀点缓冲区（立方体区域）
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // 清理局部范围内过时的膨胀数据
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // 对所有占用的体素进行膨胀
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {

        // 如果该体素被占用（超过阈值）
        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          // 获取该点周围的膨胀点集合
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          // 标记所有膨胀点为占用
          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            // 检查索引有效性
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2))
            {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;  // 标记为占用
          }
        }
      }

  // 添加虚拟天花板以限制飞行高度
  if (mp_.virtual_ceil_height_ > -0.5)
  {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

/**
 * @brief 可视化定时器回调函数
 * @param event 定时器事件（未使用）
 *
 * 定期发布地图可视化信息，包括原始占用地图和膨胀后的地图
 */
void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{

  publishMap();               // 发布原始占用地图
  publishMapInflate(true);    // 发布膨胀后的占用地图
}

/**
 * @brief 占用地图更新定时器回调函数
 * @param event 定时器事件（未使用）
 *
 * 定期更新占用地图的核心处理流程：
 * 1. 投影深度图到3D空间
 * 2. 执行射线投射更新占用概率
 * 3. 清理和膨胀局部地图
 */
void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (!md_.occ_need_update_)
    return;

  /* 更新占用地图 */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  projectDepthImage();  // 步骤1：投影深度图
  // t2 = ros::Time::now();
  raycastProcess();     // 步骤2：射线投射处理
  // t3 = ros::Time::now();

  // 步骤3：如果局部地图已更新，进行清理和膨胀
  if (md_.local_updated_)
    clearAndInflateLocalMap();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;  // 重置更新标志
  md_.local_updated_ = false;    // 重置局部更新标志
}

/**
 * @brief 深度图和位姿同步回调函数
 * @param img 深度图消息
 * @param pose 位姿消息（PoseStamped类型）
 *
 * 处理同步的深度图和位姿数据：
 * 1. 提取并转换深度图
 * 2. 提取相机位姿
 * 3. 触发占用地图更新
 */
void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
  /* 获取深度图 */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  // 如果深度图是32位浮点格式，转换为16位无符号整数
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
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
  // 检查相机位置是否在地图范围内
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;  // 触发占用更新
  }
  else
  {
    md_.occ_need_update_ = false;  // 相机不在地图内，不更新
  }
}

/**
 * @brief 独立的里程计回调函数
 * @param odom 里程计消息
 *
 * 在没有深度图时，仅更新相机位置
 */
void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if (md_.has_first_depth_)
    return;  // 如果已有深度图，不处理

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;
}

/**
 * @brief 点云回调函数
 * @param img 点云消息
 *
 * 使用点云直接更新占用地图（不使用射线投射）：
 * 1. 将点云中的点直接标记为障碍物
 * 2. 对每个点进行膨胀处理
 * 3. 更新局部地图边界
 */
void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (!md_.has_odom_)
  {
    std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
    return;

  // 重置局部更新范围内的缓冲区
  this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                    md_.camera_pos_ + mp_.local_update_range_);

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);  // 水平方向膨胀步长
  int inf_step_z = 1;  // Z方向膨胀步长（较小）

  double max_x, max_y, max_z, min_x, min_y, min_z;

  // 初始化边界框
  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  // 遍历点云中的所有点
  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* 检查点是否在更新范围内 */
    Eigen::Vector3d devi = p3d - md_.camera_pos_;
    Eigen::Vector3i inf_pt;

    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
        fabs(devi(2)) < mp_.local_update_range_(2))
    {

      /* 膨胀该点 */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z)
          {

            p3d_inf(0) = pt.x + x * mp_.resolution_;
            p3d_inf(1) = pt.y + y * mp_.resolution_;
            p3d_inf(2) = pt.z + z * mp_.resolution_;

            // 更新边界框
            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            posToIndex(p3d_inf, inf_pt);

            if (!isInMap(inf_pt))
              continue;

            int idx_inf = toAddress(inf_pt);

            md_.occupancy_buffer_inflate_[idx_inf] = 1;  // 标记为占用
          }
    }
  }

  // 将相机位置纳入边界框
  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  // 保存局部边界
  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
}

/**
 * @brief 发布原始占用地图
 *
 * 将占用概率超过阈值的体素转换为点云并发布，用于可视化
 */
void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;  // 没有订阅者，不发布

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  // 扩展发布范围（加上裕度的一半）
  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  // 遍历局部地图范围内的体素
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        // 如果占用概率低于阈值，跳过
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        // 超过可视化截断高度，跳过
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  // 设置点云属性
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

/**
 * @brief 发布膨胀后的占用地图
 * @param all_info 是否发布完整信息（包含更大的裕度范围）
 *
 * 将膨胀后的占用体素转换为点云并发布，用于可视化和路径规划
 */
void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;  // 没有订阅者，不发布

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  // 如果需要发布完整信息，扩展发布范围
  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  // 遍历局部地图范围内的体素
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        // 如果膨胀缓冲区为0（自由空间），跳过
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        // 超过可视化截断高度，跳过
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  // 设置点云属性
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
 * @brief 发布未知区域
 *
 * 将未被观测过的体素（占用概率低于最小值）转换为点云并发布
 */
void GridMap::publishUnknown()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {

        // 如果占用概率低于最小值（未知区域）
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          // 超过可视化截断高度，跳过
          if (pos(2) > mp_.visualization_truncate_height_)
            continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  // 设置点云属性
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

// 查询函数：检查是否有有效的里程计数据
bool GridMap::odomValid() { return md_.has_odom_; }

// 查询函数：检查是否已接收到深度观测
bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

// 查询函数：获取地图原点
Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

/**
 * @brief 获取地图区域信息
 * @param ori 输出：地图原点
 * @param size 输出：地图尺寸
 */
void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

/**
 * @brief 深度图和里程计同步回调函数
 * @param img 深度图消息
 * @param odom 里程计消息（Odometry类型）
 *
 * 处理同步的深度图和里程计数据：
 * 1. 从机体里程计获取位姿
 * 2. 使用cam2body变换矩阵计算相机位姿
 * 3. 提取并转换深度图
 * 4. 触发占用地图更新
 */
void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{
  /* 获取机体位姿并转换到相机坐标系 */
  // 机体姿态四元数
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  // 构建机体到世界的变换矩阵
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  // 相机到世界的变换矩阵 = 机体到世界 * 相机到机体
  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_q_ = Eigen::Quaterniond(cam_T.block<3, 3>(0, 0));

  /* 获取深度图 */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  // 如果深度图是32位浮点格式，转换为16位无符号整数
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;  // 触发占用更新
}

// GridMap类实现结束
