/**
 * @file random_forest_sensing.cpp
 * @brief 随机森林地图生成和感知模拟器
 *
 * 该文件实现了一个基于ROS的随机3D地图生成器，用于无人机仿真环境。
 * 主要功能包括：
 * 1. 生成随机柱状障碍物（森林模拟）
 * 2. 生成随机圆环障碍物（复杂障碍物）
 * 3. 基于里程计信息发布局部感知点云
 * 4. 发布全局地图点云用于可视化
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

// ==================== 点云处理相关 ====================
// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;  // KD树，用于快速半径搜索局部点云
vector<int> pointIdxRadiusSearch;                 // 半径搜索结果的点索引
vector<float> pointRadiusSquaredDistance;         // 半径搜索结果的距离平方

// ==================== 随机数生成器 ====================
random_device rd;                                 // 随机数种子
default_random_engine eng(rd());                  // 随机数引擎
uniform_real_distribution<double> rand_x;         // x坐标的均匀分布随机数生成器
uniform_real_distribution<double> rand_y;         // y坐标的均匀分布随机数生成器
uniform_real_distribution<double> rand_w;         // 障碍物宽度的均匀分布随机数生成器
uniform_real_distribution<double> rand_h;         // 障碍物高度的均匀分布随机数生成器

// ==================== ROS发布者和订阅者 ====================
ros::Publisher _local_map_pub;                    // 局部地图点云发布者
ros::Publisher _all_map_pub;                      // 全局地图点云发布者
ros::Publisher click_map_pub_;                    // 点击地图点云发布者（用于交互式添加障碍物）
ros::Subscriber _odom_sub;                        // 里程计订阅者

// ==================== 状态变量 ====================
vector<double> _state;                            // 无人机状态：[x, y, z, vx, vy, vz, ax, ay, az]

// ==================== 地图参数 ====================
int _obs_num;                                     // 柱状障碍物数量
double _x_size, _y_size, _z_size;                // 地图尺寸（x, y, z方向）
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;  // 障碍物生成范围和尺寸范围
double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;  // z高度限制、感知范围、分辨率、感知频率、初始位置

// ==================== 状态标志 ====================
bool _map_ok = false;                             // 地图是否已生成完成
bool _has_odom = false;                           // 是否已接收到里程计数据

// ==================== 圆环障碍物参数 ====================
int circle_num_;                                  // 圆环障碍物数量
double radius_l_, radius_h_, z_l_, z_h_;         // 圆环半径范围和z坐标范围
double theta_;                                    // 圆环旋转角度范围
uniform_real_distribution<double> rand_radius_;   // 圆环主半径的随机数生成器
uniform_real_distribution<double> rand_radius2_;  // 圆环次半径的随机数生成器
uniform_real_distribution<double> rand_theta_;    // 圆环旋转角度的随机数生成器
uniform_real_distribution<double> rand_z_;        // 圆环z坐标的随机数生成器

// ==================== 点云数据 ====================
sensor_msgs::PointCloud2 globalMap_pcd;           // 全局地图点云消息
pcl::PointCloud<pcl::PointXYZ> cloudMap;          // 全局地图点云数据

sensor_msgs::PointCloud2 localMap_pcd;            // 局部地图点云消息
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;    // 点击添加的障碍物点云

/**
 * @brief 随机地图生成函数
 *
 * 生成两种类型的障碍物：
 * 1. 柱状障碍物：模拟森林环境中的树木
 * 2. 圆环障碍物：增加环境复杂度的空间障碍物
 *
 * 算法流程：
 * - 初始化各参数的随机数分布
 * - 生成指定数量的柱状障碍物（避开初始点和目标点附近）
 * - 生成指定数量的圆环障碍物
 * - 构建KD树用于快速查询
 */
void RandomMapGenerate() {
  pcl::PointXYZ pt_random;

  // 初始化随机数分布范围
  rand_x = uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);

  rand_radius_ = uniform_real_distribution<double>(radius_l_, radius_h_);
  rand_radius2_ = uniform_real_distribution<double>(radius_l_, 1.2);
  rand_theta_ = uniform_real_distribution<double>(-theta_, theta_);
  rand_z_ = uniform_real_distribution<double>(z_l_, z_h_);

  // ==================== 生成柱状障碍物 ====================
  // 柱状障碍物用于模拟森林环境中的树木
  for (int i = 0; i < _obs_num; i++) {
    double x, y, w, h;
    x = rand_x(eng);  // 随机生成障碍物中心x坐标
    y = rand_y(eng);  // 随机生成障碍物中心y坐标
    w = rand_w(eng);  // 随机生成障碍物宽度

    // 检查障碍物是否距离初始点过近（2米以内），若是则重新生成
    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    // 检查障碍物是否距离目标点(19, 0)过近（2米以内），若是则重新生成
    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }

    // 将障碍物中心对齐到栅格中心
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    // 计算障碍物在栅格中的宽度（占用多少个栅格）
    int widNum = ceil(w / _resolution);

    // 在x-y平面上生成障碍物的横截面（正方形）
    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        h = rand_h(eng);  // 为每个柱子随机生成高度
        int heiNum = ceil(h / _resolution);  // 计算高度占用的栅格数
        // 沿z方向填充点云（从-30开始，模拟地面以下）
        for (int t = -30; t < heiNum; t++) {
          pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
          pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;
          cloudMap.points.push_back(pt_random);  // 添加点到全局地图
        }
      }
  }

  // ==================== 生成圆环障碍物 ====================
  // 圆环障碍物增加环境的复杂度，提供更具挑战性的规划场景
  for (int i = 0; i < circle_num_; ++i) {
    double x, y, z;
    x = rand_x(eng);   // 随机生成圆环中心x坐标
    y = rand_y(eng);   // 随机生成圆环中心y坐标
    z = rand_z_(eng);  // 随机生成圆环中心z坐标

    // 检查圆环是否距离初始点过近（2米以内），若是则重新生成
    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0) {
      i--;
      continue;
    }

    // 检查圆环是否距离目标点过近（2米以内），若是则重新生成
    if (sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
      i--;
      continue;
    }

    // 将圆环中心对齐到栅格中心
    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    Eigen::Vector3d translate(x, y, z);  // 平移向量

    // 生成随机旋转角度和旋转矩阵（绕z轴旋转）
    double theta = rand_theta_(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0,
        1;

    // 生成圆环的两个半径（椭圆环）
    double radius1 = rand_radius_(eng);   // 主半径（y方向）
    double radius2 = rand_radius2_(eng);  // 次半径（z方向）

    // 绘制以(x,y,z)为中心的圆环
    Eigen::Vector3d cpt;
    for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
      cpt(0) = 0.0;                    // x方向不偏移（圆环在yz平面）
      cpt(1) = radius1 * cos(angle);   // y方向的圆周运动
      cpt(2) = radius2 * sin(angle);   // z方向的圆周运动

      // 膨胀操作（可选，当前未启用）
      // 用于增加障碍物厚度，提高安全性
      Eigen::Vector3d cpt_if;
      for (int ifx = -0; ifx <= 0; ++ifx)
        for (int ify = -0; ify <= 0; ++ify)
          for (int ifz = -0; ifz <= 0; ++ifz) {
            cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                           ifz * _resolution);
            cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);  // 旋转并平移
            pt_random.x = cpt_if(0);
            pt_random.y = cpt_if(1);
            pt_random.z = cpt_if(2);
            cloudMap.push_back(pt_random);  // 添加点到全局地图
          }
    }
  }

  // 设置点云属性
  cloudMap.width = cloudMap.points.size();  // 点云宽度（点的总数）
  cloudMap.height = 1;                       // 点云高度（1表示无序点云）
  cloudMap.is_dense = true;                  // 点云是否稠密（无无效点）

  ROS_WARN("Finished generate random map ");

  // 构建KD树用于快速半径搜索
  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;  // 标记地图生成完成
}

/**
 * @brief 里程计回调函数
 *
 * 接收无人机的里程计信息，更新当前状态。
 * 状态包括位置(x,y,z)和速度(vx,vy,vz)，加速度设为0。
 *
 * @param odom 里程计消息
 */
void rcvOdometryCallbck(const nav_msgs::Odometry odom) {
  // 过滤掉特殊的里程计消息（child_frame_id为"X"或"O"的消息）
  if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return;
  _has_odom = true;  // 标记已接收到里程计数据

  // 更新无人机状态：位置、速度、加速度（加速度默认为0）
  _state = {odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            0.0,
            0.0,
            0.0};
}

int i = 0;
/**
 * @brief 发布感知点云
 *
 * 该函数执行两个主要任务：
 * 1. 发布全局地图点云（用于可视化完整地图）
 * 2. 发布局部感知点云（模拟传感器的有限感知范围）
 *
 * 局部感知通过KD树进行半径搜索，只发布在感知范围内的障碍物点。
 */
void pubSensedPoints() {
  // ==================== 发布全局地图点云 ====================
  // if (i < 10) {
  pcl::toROSMsg(cloudMap, globalMap_pcd);      // 将PCL点云转换为ROS消息
  globalMap_pcd.header.frame_id = "world";     // 设置坐标系为world
  _all_map_pub.publish(globalMap_pcd);         // 发布全局地图
  // }

  return;  // 当前直接返回，不执行局部感知发布（可根据需要启用）

  /* ---------- 仅发布当前位置周围的点云（局部感知） ---------- */
  if (!_map_ok || !_has_odom) return;  // 确保地图已生成且有里程计数据

  pcl::PointCloud<pcl::PointXYZ> localMap;  // 局部地图点云

  // 设置搜索点为当前无人机位置
  pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
  pointIdxRadiusSearch.clear();           // 清空上次搜索结果
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;

  // 检查搜索点坐标是否有效
  if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
    return;

  // 在KD树中进行半径搜索，查找感知范围内的所有点
  if (kdtreeLocalMap.radiusSearch(searchPoint, _sensing_range,
                                  pointIdxRadiusSearch,
                                  pointRadiusSquaredDistance) > 0) {
    // 将搜索到的点添加到局部地图
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      pt = cloudMap.points[pointIdxRadiusSearch[i]];
      localMap.points.push_back(pt);
    }
  } else {
    ROS_ERROR("[Map server] No obstacles .");
    return;
  }

  // 设置局部点云属性
  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  // 转换并发布局部地图点云
  pcl::toROSMsg(localMap, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  _local_map_pub.publish(localMap_pcd);
}

/**
 * @brief 点击回调函数
 *
 * 用于交互式地在地图中添加障碍物。
 * 当用户在RViz中点击一个位置时，在该位置生成一个随机大小的柱状障碍物。
 *
 * @param msg 点击位置的Pose消息
 */
void clickCallback(const geometry_msgs::PoseStamped& msg) {
  double x = msg.pose.position.x;  // 获取点击位置的x坐标
  double y = msg.pose.position.y;  // 获取点击位置的y坐标
  double w = rand_w(eng);          // 随机生成障碍物宽度
  double h;
  pcl::PointXYZ pt_random;

  // 将点击位置对齐到栅格中心
  x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  y = floor(y / _resolution) * _resolution + _resolution / 2.0;

  // 计算障碍物宽度占用的栅格数
  int widNum = ceil(w / _resolution);

  // 生成柱状障碍物
  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
      h = rand_h(eng);  // 随机生成高度
      int heiNum = ceil(h / _resolution);
      for (int t = -1; t < heiNum; t++) {
        pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
        pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
        pt_random.z = (t + 0.5) * _resolution + 1e-2;
        clicked_cloud_.points.push_back(pt_random);  // 添加到点击障碍物点云
        cloudMap.points.push_back(pt_random);        // 添加到全局地图
      }
    }

  // 设置点击障碍物点云属性
  clicked_cloud_.width = clicked_cloud_.points.size();
  clicked_cloud_.height = 1;
  clicked_cloud_.is_dense = true;

  // 发布点击添加的障碍物点云
  pcl::toROSMsg(clicked_cloud_, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  click_map_pub_.publish(localMap_pcd);

  // 更新全局地图点云大小
  cloudMap.width = cloudMap.points.size();

  return;
}

/**
 * @brief 主函数
 *
 * 程序入口，初始化ROS节点，设置发布者和订阅者，
 * 读取参数，生成随机地图，并周期性发布感知点云。
 *
 * 主要流程：
 * 1. 初始化ROS节点和通信接口
 * 2. 从参数服务器读取配置参数
 * 3. 生成随机地图
 * 4. 周期性发布点云数据
 */
int main(int argc, char** argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");  // 使用私有命名空间

  // ==================== 设置发布者 ====================
  _local_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);   // 局部地图发布者
  _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);    // 全局地图发布者

  // ==================== 设置订阅者 ====================
  _odom_sub = n.subscribe("odometry", 50, rcvOdometryCallbck);  // 订阅里程计信息

  // 点击地图发布者（用于交互式添加障碍物）
  click_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 1);
  // ros::Subscriber click_sub = n.subscribe("/goal", 10, clickCallback);  // 点击订阅者（可选）

  // ==================== 读取参数 ====================
  // 初始位置参数
  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);

  // 地图尺寸和障碍物数量参数
  n.param("map/x_size", _x_size, 50.0);          // 地图x方向尺寸
  n.param("map/y_size", _y_size, 50.0);          // 地图y方向尺寸
  n.param("map/z_size", _z_size, 5.0);           // 地图z方向尺寸
  n.param("map/obs_num", _obs_num, 30);          // 柱状障碍物数量
  n.param("map/resolution", _resolution, 0.1);   // 地图分辨率
  n.param("map/circle_num", circle_num_, 30);    // 圆环障碍物数量

  // 柱状障碍物形状参数
  n.param("ObstacleShape/lower_rad", _w_l, 0.3); // 障碍物最小半径
  n.param("ObstacleShape/upper_rad", _w_h, 0.8); // 障碍物最大半径
  n.param("ObstacleShape/lower_hei", _h_l, 3.0); // 障碍物最小高度
  n.param("ObstacleShape/upper_hei", _h_h, 7.0); // 障碍物最大高度

  // 圆环障碍物形状参数
  n.param("ObstacleShape/radius_l", radius_l_, 7.0);  // 圆环最小半径
  n.param("ObstacleShape/radius_h", radius_h_, 7.0);  // 圆环最大半径
  n.param("ObstacleShape/z_l", z_l_, 7.0);            // 圆环最小z坐标
  n.param("ObstacleShape/z_h", z_h_, 7.0);            // 圆环最大z坐标
  n.param("ObstacleShape/theta", theta_, 7.0);        // 圆环旋转角度范围

  // 感知参数
  n.param("sensing/radius", _sensing_range, 10.0);  // 感知半径
  n.param("sensing/radius", _sense_rate, 10.0);     // 感知频率（注意：这里应该是rate，参数名可能有误）

  // ==================== 计算派生参数 ====================
  _x_l = -_x_size / 2.0;  // x方向最小值
  _x_h = +_x_size / 2.0;  // x方向最大值

  _y_l = -_y_size / 2.0;  // y方向最小值
  _y_h = +_y_size / 2.0;  // y方向最大值

  _obs_num = min(_obs_num, (int)_x_size * 10);  // 限制障碍物数量
  _z_limit = _z_size;                            // z高度限制

  // 等待系统初始化
  ros::Duration(0.5).sleep();

  // ==================== 生成随机地图 ====================
  RandomMapGenerate();

  // ==================== 主循环：周期性发布感知点云 ====================
  ros::Rate loop_rate(_sense_rate);  // 设置循环频率

  while (ros::ok()) {
    pubSensedPoints();  // 发布感知点云
    ros::spinOnce();    // 处理回调函数
    loop_rate.sleep();  // 按频率休眠
  }
}