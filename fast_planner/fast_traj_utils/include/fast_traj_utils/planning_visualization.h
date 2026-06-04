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



// 防止头文件重复包含
#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline/non_uniform_bspline.h>
#include <iostream>
#include <fast_path_searching/topo_prm.h>
#include <fast_plan_env/obj_predictor.h>
#include <poly_traj/polynomial_traj.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>

using std::vector;
namespace fast_planner {
/**
 * @class PlanningVisualization
 * @brief 规划可视化类
 *
 * 该类负责在RViz中可视化各种规划结果，包括：
 * - 轨迹路径（几何路径、B样条轨迹、多项式轨迹）
 * - 拓扑图和路径
 * - 目标点和预测结果
 * - 偏航角轨迹
 * - 可见性约束和前沿搜索结果
 */
class PlanningVisualization {
private:
  /**
   * @enum TRAJECTORY_PLANNING_ID
   * @brief 轨迹规划可视化对象的ID枚举
   *
   * 用于区分不同的可视化标记，确保在RViz中正确显示和更新
   */
  enum TRAJECTORY_PLANNING_ID {
    GOAL = 1,              // 目标点标记ID
    PATH = 200,            // 路径标记ID
    BSPLINE = 300,         // B样条轨迹标记ID
    BSPLINE_CTRL_PT = 400, // B样条控制点标记ID
    POLY_TRAJ = 500        // 多项式轨迹标记ID
  };

  /**
   * @enum TOPOLOGICAL_PATH_PLANNING_ID
   * @brief 拓扑路径规划可视化对象的ID枚举
   *
   * 用于拓扑图和路径的可视化标记ID
   */
  enum TOPOLOGICAL_PATH_PLANNING_ID {
    GRAPH_NODE = 1,      // 图节点标记ID
    GRAPH_EDGE = 100,    // 图边标记ID
    RAW_PATH = 200,      // 原始路径标记ID
    FILTERED_PATH = 300, // 滤波后路径标记ID
    SELECT_PATH = 400    // 选定路径标记ID
  };

  /* data */
  /* visib_pub is seperated from previous ones for different info */
  ros::NodeHandle node;          // ROS节点句柄

  // ROS发布器 - 用于发布不同类型的可视化消息
  ros::Publisher traj_pub_;      // 0: 轨迹发布器
  ros::Publisher topo_pub_;      // 1: 拓扑图发布器
  ros::Publisher predict_pub_;   // 2: 预测结果发布器
  ros::Publisher visib_pub_;     // 3: 可见性约束发布器
  ros::Publisher frontier_pub_;  // 4: 前沿搜索发布器
  ros::Publisher yaw_pub_;       // 5: 偏航角轨迹发布器
  vector<ros::Publisher> pubs_;  // 发布器集合，用于统一管理

  // 记录上一次显示的可视化对象数量，用于清除旧的可视化标记
  int last_topo_path1_num_;      // 上一次拓扑路径阶段1的数量
  int last_topo_path2_num_;      // 上一次拓扑路径阶段2的数量
  int last_bspline_phase1_num_;  // 上一次B样条阶段1的数量
  int last_bspline_phase2_num_;  // 上一次B样条阶段2的数量
  int last_frontier_num_;        // 上一次前沿点的数量

public:
  /** 默认构造函数 */
  PlanningVisualization(/* args */) {}

  /** 析构函数 */
  ~PlanningVisualization() {}

  /**
   * @brief 构造函数，初始化ROS节点句柄和发布器
   * @param nh ROS节点句柄
   */
  PlanningVisualization(ros::NodeHandle& nh);

  // ==================== 基础形状绘制 ====================

  /**
   * @brief 显示球体列表
   * @param list 球心位置列表
   * @param resolution 球体半径
   * @param color 颜色(RGBA)
   * @param id 可视化标记ID
   * @param pub_id 发布器ID，默认为0（轨迹发布器）
   */
  void displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                         const Eigen::Vector4d& color, int id, int pub_id = 0);

  /**
   * @brief 显示立方体列表
   * @param list 立方体中心位置列表
   * @param resolution 立方体边长
   * @param color 颜色(RGBA)
   * @param id 可视化标记ID
   * @param pub_id 发布器ID，默认为0（轨迹发布器）
   */
  void displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                       const Eigen::Vector4d& color, int id, int pub_id = 0);

  /**
   * @brief 显示线段列表
   * @param list1 线段起点列表
   * @param list2 线段终点列表
   * @param line_width 线宽
   * @param color 颜色(RGBA)
   * @param id 可视化标记ID
   * @param pub_id 发布器ID，默认为0（轨迹发布器）
   */
  void displayLineList(const vector<Eigen::Vector3d>& list1, const vector<Eigen::Vector3d>& list2,
                       double line_width, const Eigen::Vector4d& color, int id, int pub_id = 0);

  // ==================== 路径和轨迹绘制 ====================

  /**
   * @brief 绘制分段直线路径
   * @param path 路径点列表
   * @param resolution 路径点球体的大小
   * @param color 颜色(RGBA)
   * @param id 可视化标记ID，默认为0
   */
  void drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution,
                         const Eigen::Vector4d& color, int id = 0);

  /**
   * @brief 绘制多项式轨迹
   * @param poly_traj 多项式轨迹对象
   * @param resolution 采样分辨率
   * @param color 颜色(RGBA)
   * @param id 可视化标记ID，默认为0
   */
  void drawPolynomialTraj(PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d& color,
                          int id = 0);

  /**
   * @brief 绘制B样条轨迹
   * @param bspline B样条对象
   * @param size 轨迹线宽
   * @param color 轨迹颜色(RGBA)
   * @param show_ctrl_pts 是否显示控制点，默认为false
   * @param size2 控制点大小，默认为0.1
   * @param color2 控制点颜色，默认为黄色(1,1,0,1)
   * @param id1 轨迹标记ID，默认为0
   * @param id2 控制点标记ID，默认为0
   */
  void drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color,
                   bool show_ctrl_pts = false, double size2 = 0.1,
                   const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0,
                   int id2 = 0);

  /**
   * @brief 绘制阶段1生成的一组B样条轨迹
   * @param bsplines B样条轨迹列表
   * @param size 轨迹线宽
   */
  void drawBsplinesPhase1(vector<NonUniformBspline>& bsplines, double size);

  /**
   * @brief 绘制阶段2生成的一组B样条轨迹
   * @param bsplines B样条轨迹列表
   * @param size 轨迹线宽
   */
  void drawBsplinesPhase2(vector<NonUniformBspline>& bsplines, double size);

  // ==================== 拓扑图和路径绘制 ====================

  /**
   * @brief 绘制拓扑图
   * @param graph 图节点列表
   * @param point_size 节点大小
   * @param line_width 边的线宽
   * @param color1 节点颜色
   * @param color2 边颜色
   * @param color3 特殊节点颜色
   * @param id 可视化标记ID，默认为0
   */
  void drawTopoGraph(list<GraphNode::Ptr>& graph, double point_size, double line_width,
                     const Eigen::Vector4d& color1, const Eigen::Vector4d& color2,
                     const Eigen::Vector4d& color3, int id = 0);

  /**
   * @brief 绘制阶段1的拓扑路径集合
   * @param paths 路径集合，每条路径由一系列3D点组成
   * @param line_width 路径线宽
   */
  void drawTopoPathsPhase1(vector<vector<Eigen::Vector3d>>& paths, double line_width);

  /**
   * @brief 绘制阶段2的拓扑路径集合
   * @param paths 路径集合，每条路径由一系列3D点组成
   * @param line_width 路径线宽
   */
  void drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>>& paths, double line_width);

  // ==================== 目标点和预测绘制 ====================

  /**
   * @brief 绘制目标点
   * @param goal 目标点3D坐标
   * @param resolution 目标点标记大小
   * @param color 颜色(RGBA)
   * @param id 可视化标记ID，默认为0
   */
  void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);

  /**
   * @brief 绘制物体预测轨迹
   * @param pred 预测对象
   * @param resolution 标记大小
   * @param color 颜色(RGBA)
   * @param id 可视化标记ID，默认为0
   */
  void drawPrediction(ObjPrediction pred, double resolution, const Eigen::Vector4d& color, int id = 0);

  // ==================== 工具函数 ====================

  /**
   * @brief 根据色调值生成颜色
   * @param h 色调值（0-1之间）
   * @param alpha 透明度，默认为1.0
   * @return 返回RGBA颜色向量
   */
  Eigen::Vector4d getColor(double h, double alpha = 1.0);

  /** 智能指针类型定义 */
  typedef std::shared_ptr<PlanningVisualization> Ptr;

  // ==================== 偏航角可视化（开发中） ====================

  /**
   * @brief 绘制偏航角轨迹（基于B样条）
   * @param pos 位置B样条轨迹
   * @param yaw 偏航角B样条轨迹
   * @param dt 时间步长
   */
  void drawYawTraj(NonUniformBspline& pos, NonUniformBspline& yaw, const double& dt);

  /**
   * @brief 绘制偏航角路径（基于离散偏航角）
   * @param pos 位置B样条轨迹
   * @param yaw 偏航角序列
   * @param dt 时间步长
   */
  void drawYawPath(NonUniformBspline& pos, const vector<double>& yaw, const double& dt);
};
}  // namespace fast_planner
#endif