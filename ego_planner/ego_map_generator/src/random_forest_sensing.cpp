/*
 * @file: random_forest_sensing.cpp
 * @brief: 随机森林感知模拟器
 * @功能: 为无人机仿真环境生成随机障碍物地图,包括柱状障碍物和圆环障碍物
 * @主要模块:
 *   1. 随机障碍物生成(柱状和圆环)
 *   2. 基于KD树的局部地图感知
 *   3. 点云发布与可视化
 *   4. 交互式障碍物添加
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

using namespace std;

// ==================== KD树与点云搜索相关 ====================
// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;  // KD树用于快速最近邻搜索
vector<int> pointIdxRadiusSearch;                // 半径搜索返回的点索引
vector<float> pointRadiusSquaredDistance;        // 半径搜索返回的距离平方

// ==================== 随机数生成器 ====================
random_device rd;                                // 随机设备用于生成种子
//default_random_engine eng(0);                  // 固定种子(用于调试)
default_random_engine eng(rd());                 // 使用随机种子的随机数引擎
uniform_real_distribution<double> rand_x;        // x坐标随机分布
uniform_real_distribution<double> rand_y;        // y坐标随机分布
uniform_real_distribution<double> rand_w;        // 障碍物宽度随机分布
uniform_real_distribution<double> rand_h;        // 障碍物高度随机分布
uniform_real_distribution<double> rand_inf;      // 膨胀系数随机分布

// ==================== ROS通信相关 ====================
ros::Publisher _local_map_pub;                   // 局部地图发布器
ros::Publisher _all_map_pub;                     // 全局地图发布器
ros::Publisher click_map_pub_;                   // 点击添加障碍物发布器
ros::Subscriber _odom_sub;                       // 里程计订阅器

// ==================== 状态变量 ====================
vector<double> _state;                           // 无人机状态[x,y,z,vx,vy,vz,ax,ay,az]

// ==================== 地图参数 ====================
int _obs_num;                                    // 障碍物数量
double _x_size, _y_size, _z_size;               // 地图尺寸(x,y,z方向)
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;  // 各维度上下限
double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;  // z限制,感知范围,分辨率,感知频率,初始位置
double _min_dist;                                // 障碍物之间的最小距离

// ==================== 状态标志 ====================
bool _map_ok = false;                            // 地图是否生成完成
bool _has_odom = false;                          // 是否接收到里程计数据

// ==================== 圆环障碍物参数 ====================
int circle_num_;                                 // 圆环障碍物数量
double radius_l_, radius_h_, z_l_, z_h_;        // 半径和高度范围
double theta_;                                   // 旋转角度范围
uniform_real_distribution<double> rand_radius_;  // 主半径随机分布
uniform_real_distribution<double> rand_radius2_; // 副半径随机分布
uniform_real_distribution<double> rand_theta_;   // 旋转角度随机分布
uniform_real_distribution<double> rand_z_;       // z坐标随机分布

// ==================== 点云数据 ====================
sensor_msgs::PointCloud2 globalMap_pcd;          // 全局地图点云消息
pcl::PointCloud<pcl::PointXYZ> cloudMap;         // 全局地图点云(PCL格式)

sensor_msgs::PointCloud2 localMap_pcd;           // 局部地图点云消息
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;   // 点击添加的障碍物点云

/**
 * @brief 生成随机地图(包含方柱障碍物和圆环障碍物)
 * @功能:
 *   1. 生成随机分布的方柱形障碍物
 *   2. 生成椭圆环形障碍物
 *   3. 避免在起点和目标点附近生成障碍物
 *   4. 构建KD树用于快速空间查询
 * @note: 已弃用,推荐使用RandomMapGenerateCylinder()
 */
void RandomMapGenerate() {
  pcl::PointXYZ pt_random;

  // 初始化各维度的随机数分布
  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  // 初始化圆环障碍物的随机数分布
  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // ==================== 生成方柱形障碍物 ====================
  for (int i = 0; i < _obs_num; i++) {
    double x, y, w, h;
    x = rand_x(eng);  // 随机生成x坐标
    y = rand_y(eng);  // 随机生成y坐标
    w = rand_w(eng);  // 随机生成宽度

    // 检查是否距离起点太近(2米范围内),如果太近则重新生成
    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    // 检查是否距离目标点(19,0)太近,如果太近则重新生成
    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }

    // 将坐标对齐到网格中心
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    // 计算障碍物宽度占据的网格数
    int widNum = ceil(w / _resolution);

    // 三重循环生成方柱体的所有点云
    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        h = rand_h(eng);  // 随机生成高度
        int heiNum = ceil(h / _resolution);
        for (int t = -20; t < heiNum; t++) {  // 从地下-20层开始,确保障碍物接地
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;
          cloudMap.points.push_back(pt_random);
        }
      }
  }

  // ==================== 生成圆环障碍物 ====================
  for (int i = 0; i < circle_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    // 检查是否距离起点太近
    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    // 检查是否距离目标点太近
    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }

    // 将坐标对齐到网格
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    // 生成随机旋转角度和旋转矩阵
    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    // 生成椭圆的两个半径
    double radius1 = rand_radius_(eng);   // 主半径
    double radius2 = rand_radius2_(eng);  // 副半径

    // 绘制椭圆环:在YZ平面上生成椭圆,然后旋转和平移
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);  // Y方向使用主半径
      cpt(2) = radius2 * sin(angle);  // Z方向使用副半径

      // 膨胀处理(当前设置为0,不膨胀)
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);  // 旋转后平移
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  // 设置点云属性
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  // 构建KD树用于快速空间搜索
  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

/**
 * @brief 生成圆柱形障碍物地图(推荐使用)
 * @功能:
 *   1. 生成随机分布的圆柱形障碍物
 *   2. 生成椭圆环形障碍物
 *   3. 保证障碍物之间的最小间距
 *   4. 避免在起点和目标点附近生成障碍物
 *   5. 支持障碍物半径的随机膨胀
 * @note: 相比RandomMapGenerate(),使用圆柱体代替方柱,更加真实
 */
void RandomMapGenerateCylinder() {
  pcl::PointXYZ pt_random;

  vector<Eigen::Vector2d> obs_position;  // 记录已生成障碍物的位置,用于检查最小间距

  // 初始化各维度的随机数分布
  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);
  rand_inf = uniform_real_distribution<double>(0.5, 1.5);  // 膨胀系数范围[0.5, 1.5]

  // 初始化圆环障碍物的随机数分布
  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // ==================== 生成圆柱形障碍物 ====================
  for (int i = 0; i < _obs_num && ros::ok(); i++) {
    double x, y, w, h, inf;
    x = rand_x(eng);      // 随机x坐标
    y = rand_y(eng);      // 随机y坐标
    w = rand_w(eng);      // 随机宽度(直径)
    inf = rand_inf(eng);  // 随机膨胀系数

    // 检查是否距离起点太近(2米范围内)
    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    // 检查是否距离目标点(19,0)太近
    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }

    // 检查与已有障碍物的最小间距
    bool flag_continue = false;
    for ( auto p : obs_position )
      if ( (Eigen::Vector2d(x,y) - p).norm() < _min_dist /*metres*/ )
      {
        i--;
        flag_continue = true;
        break;
      }
    if ( flag_continue ) continue;

    // 记录当前障碍物位置
    obs_position.push_back( Eigen::Vector2d(x,y) );


    // 将坐标对齐到网格中心
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    // 计算膨胀后的宽度和半径
    int widNum = ceil((w*inf) / _resolution);
    double radius = (w*inf) / 2;

    // 在包围盒内生成点,但只保留圆柱体内的点
    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        h = rand_h(eng);  // 随机高度
        int heiNum = ceil(h / _resolution);
        for (int t = -30; t < heiNum; t++) {  // 从地下-30层开始
          double temp_x = x + (r + 0.5) * _resolution + 1e-2;
          double temp_y = y + (s + 0.5) * _resolution + 1e-2;
          double temp_z = (t + 0.5) * _resolution + 1e-2;
          // 只添加在圆柱体内的点(通过距离圆心的距离判断)
          if ( (Eigen::Vector2d(temp_x,temp_y) - Eigen::Vector2d(x,y)).norm() <= radius )
          {
            pt_random.x = temp_x;
            pt_random.y = temp_y;
            pt_random.z = temp_z;
            cloudMap.points.push_back(pt_random);
          }
        }
      }
  }

  // ==================== 生成圆环障碍物 ====================
  for (int i = 0; i < circle_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);

    // 检查是否距离起点太近
    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    // 检查是否距离目标点太近
    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }

    // 将坐标对齐到网格
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);

    // 生成随机旋转角度和旋转矩阵(绕Z轴旋转)
    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    // 生成椭圆的两个半径
    double radius1 = rand_radius_(eng);   // 主半径
    double radius2 = rand_radius2_(eng);  // 副半径

    // 绘制椭圆环:在YZ平面上生成椭圆,然后旋转和平移
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
      cpt(0) = 0.0;
      cpt(1) = radius1 * cos(angle);  // Y方向
      cpt(2) = radius2 * sin(angle);  // Z方向

      // 膨胀处理(当前设置为0,不膨胀)
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);  // 旋转后平移
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);
          }
    }
  }

  // ==================== 生成地板(可选,已注释) ====================
  // pcl::PointXYZ pt;
  // pt.z = 0.1;
  // for ( pt.x = _x_l; pt.x <= _x_h; pt.x += _resolution )
  //   for ( pt.y = _y_l; pt.y <= _y_h; pt.y += _resolution )
  //   {
  //     cloudMap.push_back(pt);
  //   }

  // 设置点云属性
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  // 构建KD树用于快速空间搜索
  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

/**
 * @brief 里程计数据回调函数
 * @param odom 接收到的里程计消息
 * @功能: 更新无人机的位置和速度状态
 * @note: 过滤掉child_frame_id为"X"或"O"的消息(可能是标记消息)
 */
void rcvOdometryCallbck(const nav_msgs::Odometry odom) {
  if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return;
  _has_odom = true;

  // 更新状态向量: [x, y, z, vx, vy, vz, ax, ay, az]
  _state = {odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            0.0,  // 加速度信息未提供,设为0
            0.0,
            0.0};
}

int i = 0;  // 计数器(未使用)
/**
 * @brief 发布感知到的点云数据
 * @功能:
 *   1. 发布全局地图点云(默认行为)
 *   2. 发布局部地图点云(当前被禁用,通过early return)
 * @note:
 *   - 当前实现直接发布全局地图
 *   - 局部地图发布功能被注释,需要时可以移除return语句启用
 */
void pubSensedPoints() {
  // if (i < 10) {
  // 发布全局地图点云
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd);
  // }

  return;  // 提前返回,禁用下面的局部地图发布

  /* ==================== 仅发布当前位置周围的点云(已禁用) ==================== */
  if (!_map_ok || !_has_odom) return;

  pcl::PointCloud<pcl::PointXYZ> localMap;

  // 使用当前位置作为搜索中心
  pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;

  // 检查位置是否有效
  if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
    return;

  // 使用KD树进行半径搜索,找出感知范围内的所有障碍物点
  if (kdtreeLocalMap.radiusSearch(searchPoint, _sensing_range,
                                  pointIdxRadiusSearch,
                                  pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      pt = cloudMap.points[pointIdxRadiusSearch[i]];
      localMap.points.push_back(pt);
    }
  } else {
    ROS_ERROR("[Map server] No obstacles .");
    return;
  }

  // 设置局部地图点云属性
  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  // 发布局部地图
  pcl::toROSMsg(localMap, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  _local_map_pub.publish(localMap_pcd);
}

/**
 * @brief 鼠标点击回调函数(交互式添加障碍物)
 * @param msg 点击位置的姿态消息
 * @功能:
 *   1. 在鼠标点击位置生成一个随机尺寸的方柱障碍物
 *   2. 同时更新clicked_cloud_和全局cloudMap
 *   3. 发布新添加的障碍物点云
 * @应用: 用于动态添加障碍物,测试规划算法的实时性能
 */
void clickCallback(const geometry_msgs::PoseStamped& msg) {
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  double w = rand_w(eng);  // 随机宽度
  double h;
  pcl::PointXYZ pt_random;

  // 将坐标对齐到网格中心
  x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  y = floor(y / _resolution) * _resolution + _resolution / 2.0;

  // 计算障碍物宽度占据的网格数
  int widNum = ceil(w / _resolution);

  // 生成方柱障碍物的所有点
  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
      h = rand_h(eng);  // 随机高度
      int heiNum = ceil(h / _resolution);
      for (int t = -1; t < heiNum; t++) {  // 从-1层开始,略低于地面
        pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
        pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
        pt_random.z = (t + 0.5) * _resolution + 1e-2;
        clicked_cloud_.points.push_back(pt_random);  // 添加到点击点云
        cloudMap.points.push_back(pt_random);         // 添加到全局地图
      }
    }

  // 设置点击点云属性
  clicked_cloud_.width = clicked_cloud_.points.size();
  clicked_cloud_.height = 1;
  clicked_cloud_.is_dense = true;

  // 发布点击添加的障碍物
  pcl::toROSMsg(clicked_cloud_, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  click_map_pub_.publish(localMap_pcd);

  // 更新全局地图点云宽度
  cloudMap.width = cloudMap.points.size();

  return;
}

/**
 * @brief 主函数
 * @功能:
 *   1. 初始化ROS节点和参数
 *   2. 创建发布器和订阅器
 *   3. 生成随机障碍物地图
 *   4. 周期性发布点云数据
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");

  // ==================== 创建发布器 ====================
  _local_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
  _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

  // ==================== 创建订阅器 ====================
  _odom_sub = n.subscribe("odometry", 50, rcvOdometryCallbck);

  click_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 1);
  // ros::Subscriber click_sub = n.subscribe("/goal", 10, clickCallback);

  // ==================== 读取ROS参数 ====================
  // 初始状态参数
  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);

  // 地图尺寸参数
  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/obs_num", _obs_num, 30);         // 障碍物数量
  n.param("map/resolution", _resolution, 0.1);  // 地图分辨率
  n.param("map/circle_num", circle_num_, 30);   // 圆环障碍物数量

  // 障碍物形状参数(柱状)
  n.param("ObstacleShape/lower_rad", _w_l, 0.3);  // 最小半径
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);  // 最大半径
  n.param("ObstacleShape/lower_hei", _h_l, 3.0);  // 最小高度
  n.param("ObstacleShape/upper_hei", _h_h, 7.0);  // 最大高度

  // 障碍物形状参数(圆环)
  n.param("ObstacleShape/radius_l", radius_l_, 7.0);  // 最小半径
  n.param("ObstacleShape/radius_h", radius_h_, 7.0);  // 最大半径
  n.param("ObstacleShape/z_l", z_l_, 7.0);            // 最小高度
  n.param("ObstacleShape/z_h", z_h_, 7.0);            // 最大高度
  n.param("ObstacleShape/theta", theta_, 7.0);        // 旋转角度范围

  // 感知参数
  n.param("sensing/radius", _sensing_range, 10.0);  // 感知半径
  n.param("sensing/radius", _sense_rate, 10.0);     // 感知频率(注意:参数名可能有误)

  // 障碍物间距参数
  n.param("min_distance", _min_dist, 1.0);  // 障碍物之间的最小距离

  // ==================== 计算地图边界 ====================
  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  // 限制障碍物数量(避免过密)
  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  // 等待ROS系统初始化完成
  ros::Duration(0.5).sleep();

  // ==================== 生成随机地图 ====================
  // RandomMapGenerate();           // 方柱障碍物版本(已弃用)
  RandomMapGenerateCylinder();      // 圆柱障碍物版本(推荐)

  // ==================== 主循环:周期性发布点云 ====================
  ros::Rate loop_rate(_sense_rate);

  while (ros::ok()) {
    pubSensedPoints();  // 发布点云数据
    ros::spinOnce();    // 处理回调函数
    loop_rate.sleep();  // 保持循环频率
  }
}