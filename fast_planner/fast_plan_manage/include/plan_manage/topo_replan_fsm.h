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



#ifndef _TOPO_REPLAN_FSM_H_
#define _TOPO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <fast_bspline_opt/bspline_optimizer.h>
#include <fast_path_searching/kinodynamic_astar.h>
#include <fast_plan_env/edt_environment.h>
#include <fast_plan_env/obj_predictor.h>
#include <fast_plan_env/sdf_map.h>
#include <fast_plan_manage/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <fast_traj_utils/planning_visualization.h>

using std::vector;

namespace fast_planner {

/**
 * @brief 拓扑重规划有限状态机类
 *
 * 该类实现了基于拓扑路径的重规划有限状态机，用于无人机的在线轨迹规划和重规划。
 * 主要功能包括：
 * 1. 管理规划状态机的状态转换
 * 2. 处理目标点设置和里程计数据
 * 3. 执行轨迹生成和优化
 * 4. 碰撞检测和安全监控
 * 5. 基于拓扑路径的轨迹优化
 */
class TopoReplanFSM {
private:
  /* ---------- flag ---------- */
  /**
   * @brief 有限状态机执行状态枚举
   *
   * INIT: 初始化状态
   * WAIT_TARGET: 等待目标点状态
   * GEN_NEW_TRAJ: 生成新轨迹状态
   * REPLAN_TRAJ: 重规划轨迹状态
   * EXEC_TRAJ: 执行轨迹状态
   * REPLAN_NEW: 重新规划新轨迹状态
   */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };

  /**
   * @brief 目标点类型枚举
   *
   * MANUAL_TARGET: 手动选择的目标点
   * PRESET_TARGET: 预设的目标点
   * REFENCE_PATH: 参考路径
   */
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;      // 快速规划管理器智能指针，负责路径搜索和轨迹优化
  PlanningVisualization::Ptr visualization_;     // 规划可视化智能指针，用于在RViz中显示轨迹和路径

  /* parameters */
  int target_type_;                              // 目标点类型：1-手动选择，2-硬编码预设，3-参考路径
  double replan_distance_threshold_;             // 重规划距离阈值，当距离目标点小于此值时触发重规划
  double replan_time_threshold_;                 // 重规划时间阈值，当剩余时间小于此值时触发重规划
  double waypoints_[50][3];                      // 路径点数组，最多存储50个路径点，每个点包含xyz三维坐标
  int waypoint_num_;                             // 当前路径点的数量
  bool act_map_;                                 // 是否激活动态地图标志

  /* planning data */
  bool trigger_;                                 // 触发标志，表示是否触发规划
  bool have_target_;                             // 是否有目标点标志
  bool have_odom_;                               // 是否接收到里程计数据标志
  bool collide_;                                 // 碰撞标志，表示是否检测到碰撞
  FSM_EXEC_STATE exec_state_;                    // 当前有限状态机的执行状态

  Eigen::Vector3d odom_pos_;                     // 里程计位置（x, y, z）
  Eigen::Vector3d odom_vel_;                     // 里程计速度（vx, vy, vz）
  Eigen::Quaterniond odom_orient_;               // 里程计姿态四元数

  Eigen::Vector3d start_pt_;                     // 起始位置
  Eigen::Vector3d start_vel_;                    // 起始速度
  Eigen::Vector3d start_acc_;                    // 起始加速度
  Eigen::Vector3d start_yaw_;                    // 起始偏航角（yaw, yaw_dot, yaw_ddot）
  Eigen::Vector3d target_point_;                 // 目标位置
  Eigen::Vector3d end_vel_;                      // 终点速度
  int current_wp_;                               // 当前路径点索引

  /* ROS utils */
  ros::NodeHandle node_;                         // ROS节点句柄
  ros::Timer exec_timer_;                        // 执行定时器，定期触发状态机执行回调
  ros::Timer safety_timer_;                      // 安全定时器，定期检查碰撞
  ros::Timer vis_timer_;                         // 可视化定时器，定期更新可视化
  ros::Timer frontier_timer_;                    // 前沿定时器（未使用）
  ros::Subscriber waypoint_sub_;                 // 路径点订阅器，接收目标路径点
  ros::Subscriber odom_sub_;                     // 里程计订阅器，接收机器人位姿和速度
  ros::Publisher replan_pub_;                    // 重规划发布器，发布重规划触发信号
  ros::Publisher new_pub_;                       // 新轨迹发布器，发布新生成的轨迹
  ros::Publisher bspline_pub_;                   // B样条曲线发布器，发布优化后的B样条轨迹

  /* helper functions */
  /**
   * @brief 调用搜索和优化算法
   *
   * 该函数执行完整的规划流程，包括前端路径搜索（使用Kinodynamic A*）
   * 和后端轨迹优化（使用B样条优化）
   *
   * @return true 规划成功
   * @return false 规划失败
   */
  bool callSearchAndOptimization();

  /**
   * @brief 调用基于拓扑路径的轨迹生成
   *
   * 使用拓扑路径引导的梯度优化方法生成轨迹
   *
   * @param step 规划步骤：1-生成新轨迹，2-重规划轨迹
   * @return true 轨迹生成成功
   * @return false 轨迹生成失败
   */
  bool callTopologicalTraj(int step);

  /**
   * @brief 改变有限状态机的执行状态
   *
   * @param new_state 新的状态
   * @param pos_call 调用位置的描述字符串，用于调试
   */
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);

  /**
   * @brief 打印当前有限状态机的执行状态
   *
   * 用于调试，将当前状态输出到控制台
   */
  void printFSMExecState();

  /* ROS functions */
  /**
   * @brief 有限状态机执行回调函数
   *
   * 定期被exec_timer_调用，根据当前状态执行相应的规划逻辑
   * 包括状态转换、轨迹生成、重规划等
   *
   * @param e 定时器事件
   */
  void execFSMCallback(const ros::TimerEvent& e);

  /**
   * @brief 碰撞检查回调函数
   *
   * 定期被safety_timer_调用，检查当前轨迹是否与障碍物碰撞
   * 如果检测到碰撞，设置碰撞标志并触发重规划
   *
   * @param e 定时器事件
   */
  void checkCollisionCallback(const ros::TimerEvent& e);

  /**
   * @brief 路径点回调函数
   *
   * 接收来自waypoint_sub_的路径点消息，更新目标点或路径点序列
   *
   * @param msg 路径点消息指针
   */
  void waypointCallback(const nav_msgs::PathConstPtr& msg);

  /**
   * @brief 里程计回调函数
   *
   * 接收来自odom_sub_的里程计消息，更新机器人当前位置、速度和姿态
   *
   * @param msg 里程计消息指针
   */
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

public:
  /**
   * @brief 构造函数
   */
  TopoReplanFSM(/* args */) {}

  /**
   * @brief 析构函数
   */
  ~TopoReplanFSM() {}

  /**
   * @brief 初始化函数
   *
   * 初始化有限状态机，包括：
   * 1. 加载ROS参数
   * 2. 初始化规划器管理器
   * 3. 初始化可视化工具
   * 4. 创建订阅器、发布器和定时器
   * 5. 设置初始状态
   *
   * @param nh ROS节点句柄
   */
  void init(ros::NodeHandle& nh);

  /**
   * @brief Eigen库内存对齐宏
   *
   * 确保使用Eigen库时的内存对齐，避免在使用固定大小的可向量化类型时出现问题
   */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif