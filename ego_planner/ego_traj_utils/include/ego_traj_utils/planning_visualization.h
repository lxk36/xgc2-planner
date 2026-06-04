/**
 * @file planning_visualization.h
 * @brief 规划可视化工具类头文件
 * @details 提供ROS环境下的轨迹规划可视化功能，支持显示目标点、路径、箭头等
 */

#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <ego_bspline_opt/uniform_bspline.h>
#include <iostream>
#include <ego_traj_utils/polynomial_traj.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>

using std::vector;
namespace ego_planner
{
  /**
   * @class PlanningVisualization
   * @brief 规划可视化类
   * @details 封装了各种轨迹规划相关的可视化功能，包括：
   *          - 目标点可视化
   *          - 全局路径可视化
   *          - 初始路径可视化
   *          - 优化后路径可视化
   *          - A*路径可视化
   *          - 引导向量可视化
   *          - 中间状态可视化
   */
  class PlanningVisualization
  {
  private:
    ros::NodeHandle node; // ROS节点句柄

    ros::Publisher goal_point_pub;        // 目标点发布器
    ros::Publisher global_list_pub;       // 全局路径点列表发布器
    ros::Publisher init_list_pub;         // 初始路径点列表发布器
    ros::Publisher optimal_list_pub;      // 优化后路径点列表发布器
    ros::Publisher a_star_list_pub;       // A*算法路径列表发布器
    ros::Publisher guide_vector_pub;      // 引导向量发布器
    ros::Publisher intermediate_state_pub; // 中间状态发布器

  public:
    /**
     * @brief 默认构造函数
     */
    PlanningVisualization(/* args */) {}

    /**
     * @brief 析构函数
     */
    ~PlanningVisualization() {}

    /**
     * @brief 带ROS节点句柄的构造函数
     * @param nh ROS节点句柄引用，用于初始化各种发布器
     */
    PlanningVisualization(ros::NodeHandle &nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr; // 智能指针类型定义

    /**
     * @brief 显示标记点列表
     * @param pub ROS发布器引用
     * @param list 三维点列表，待可视化的点集合
     * @param scale 标记点的尺寸比例
     * @param color 颜色向量(R, G, B, A)，取值范围[0,1]
     * @param id 标记的唯一标识符
     */
    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id);

    /**
     * @brief 生成路径显示数组
     * @param array 输出的MarkerArray，存储生成的可视化标记
     * @param list 三维点列表，表示路径上的点
     * @param scale 路径线段的宽度
     * @param color 颜色向量(R, G, B, A)
     * @param id 标记的唯一标识符
     */
    void generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);

    /**
     * @brief 生成箭头显示数组
     * @param array 输出的MarkerArray，存储生成的箭头标记
     * @param list 三维点列表，每个点作为箭头的起点或方向
     * @param scale 箭头的尺寸比例
     * @param color 颜色向量(R, G, B, A)
     * @param id 标记的唯一标识符
     */
    void generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);

    /**
     * @brief 显示目标点
     * @param goal_point 三维目标点坐标
     * @param color 颜色向量(R, G, B, A)
     * @param scale 目标点标记的尺寸
     * @param id 标记的唯一标识符
     */
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);

    /**
     * @brief 显示全局路径点列表
     * @param global_pts 全局路径的三维点列表
     * @param scale 路径线段的宽度
     * @param id 标记的唯一标识符
     */
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);

    /**
     * @brief 显示初始路径点列表
     * @param init_pts 初始路径的三维点列表
     * @param scale 路径线段的宽度
     * @param id 标记的唯一标识符
     */
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);

    /**
     * @brief 显示优化后的路径列表
     * @param optimal_pts 优化后的路径点矩阵，每列代表一个三维点
     * @param id 标记的唯一标识符
     */
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);

    /**
     * @brief 显示A*算法生成的多条路径
     * @param a_star_paths A*路径的向量，每条路径是一个三维点列表
     * @param id 标记的唯一标识符
     */
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);

    /**
     * @brief 显示箭头列表
     * @param pub ROS发布器引用
     * @param list 三维点列表，定义箭头的位置和方向
     * @param scale 箭头的尺寸比例
     * @param color 颜色向量(R, G, B, A)
     * @param id 标记的唯一标识符
     */
    void displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);

    // 注释掉的函数：显示中间状态
    // void displayIntermediateState(ros::Publisher& intermediate_pub, ego_planner::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration);

    // 注释掉的函数：显示新的引导箭头
    // void displayNewArrow(ros::Publisher& guide_vector_pub, ego_planner::BsplineOptimizer::Ptr optimizer);
  };
} // namespace ego_planner
#endif