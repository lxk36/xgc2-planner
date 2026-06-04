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



#ifndef _KINO_REPLAN_FSM_H_
#define _KINO_REPLAN_FSM_H_

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

// 测试类（用于开发调试）
class Test {
private:
  /* data */
  int test_;                    // 测试整数值
  std::vector<int> test_vec_;   // 测试整数向量
  ros::NodeHandle nh_;          // ROS节点句柄

public:
  // 构造函数：通过整数值初始化
  // @param v 测试整数值
  Test(const int& v) {
    test_ = v;
  }
  // 构造函数：通过ROS节点句柄初始化
  // @param node ROS节点句柄
  Test(ros::NodeHandle& node) {
    nh_ = node;
  }
  // 析构函数
  ~Test() {
  }
  // 打印测试值到标准输出
  void print() {
    std::cout << "test: " << test_ << std::endl;
  }
};

/**
 * @class KinoReplanFSM
 * @brief 基于运动学约束的重规划有限状态机类
 *
 * 该类实现了一个完整的轨迹规划与重规划系统，采用有限状态机(FSM)架构管理规划流程。
 * 主要功能包括：
 * - 基于kinodynamic A*的前端路径搜索
 * - 基于B样条的后端轨迹优化
 * - 实时碰撞检测与轨迹重规划
 * - 支持多种目标点输入方式（手动选择、预设路径点、参考路径）
 */
class KinoReplanFSM {

private:
  /* ---------- flag ---------- */
  /**
   * @enum FSM_EXEC_STATE
   * @brief 有限状态机执行状态枚举
   *
   * INIT: 初始化状态，等待系统准备就绪
   * WAIT_TARGET: 等待目标点状态，等待用户输入或接收目标点
   * GEN_NEW_TRAJ: 生成新轨迹状态，首次规划生成完整轨迹
   * REPLAN_TRAJ: 重规划轨迹状态，在执行过程中检测到碰撞风险时触发
   * EXEC_TRAJ: 执行轨迹状态，正常跟踪已规划的轨迹
   * REPLAN_NEW: 重新规划新轨迹状态，当前轨迹无法修复时重新规划
   */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };

  /**
   * @enum TARGET_TYPE
   * @brief 目标点类型枚举
   *
   * MANUAL_TARGET: 手动选择目标点（通过RViz等工具手动指定）
   * PRESET_TARGET: 预设目标点（通过参数文件预先配置的路径点序列）
   * REFENCE_PATH: 参考路径（跟随预定义的参考轨迹）
   */
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;     // 规划管理器，负责前端路径搜索和后端轨迹优化
  PlanningVisualization::Ptr visualization_;    // 可视化工具，用于在RViz中显示轨迹、路径等信息

  /* parameters */
  int target_type_;                             // 目标点类型：1-手动选择，2-预设硬编码，3-参考路径
  double no_replan_thresh_;                     // 不需要重规划的安全距离阈值（米）
  double replan_thresh_;                        // 触发重规划的碰撞距离阈值（米）
  double waypoints_[50][3];                     // 预设路径点数组，最多存储50个三维路径点 [x, y, z]
  int waypoint_num_;                            // 实际使用的路径点数量

  /* planning data */
  bool trigger_;                                // 触发规划标志，用于启动规划流程
  bool have_target_;                            // 是否已接收到目标点标志
  bool have_odom_;                              // 是否已接收到里程计数据标志
  FSM_EXEC_STATE exec_state_;                   // 当前有限状态机执行状态

  Eigen::Vector3d odom_pos_, odom_vel_;         // 里程计状态：位置和速度（来自传感器或状态估计）
  Eigen::Quaterniond odom_orient_;              // 里程计姿态：四元数表示的机器人朝向

  Eigen::Vector3d start_pt_;                    // 规划起点位置
  Eigen::Vector3d start_vel_;                   // 规划起点速度
  Eigen::Vector3d start_acc_;                   // 规划起点加速度
  Eigen::Vector3d start_yaw_;                   // 规划起点偏航角（yaw, pitch, roll）
  Eigen::Vector3d end_pt_;                      // 目标点位置
  Eigen::Vector3d end_vel_;                     // 目标点速度
  int current_wp_;                              // 当前正在追踪的路径点索引

  /* ROS utils */
  ros::NodeHandle node_;                        // ROS节点句柄，用于创建发布者、订阅者和定时器
  ros::Timer exec_timer_;                       // FSM执行定时器，周期性触发状态机更新
  ros::Timer safety_timer_;                     // 安全检查定时器，周期性检测碰撞风险
  ros::Timer vis_timer_;                        // 可视化定时器，周期性更新RViz显示
  ros::Timer test_something_timer_;             // 测试定时器，用于开发调试
  ros::Subscriber waypoint_sub_;                // 路径点订阅者，接收目标点或路径序列
  ros::Subscriber odom_sub_;                    // 里程计订阅者，接收机器人位置、速度、姿态信息
  ros::Publisher replan_pub_;                   // 重规划发布者，发布重规划事件信号
  ros::Publisher new_pub_;                      // 新轨迹发布者，发布新生成的轨迹
  ros::Publisher bspline_pub_;                  // B样条轨迹发布者，发布优化后的B样条轨迹

  /* helper functions */
  /**
   * @brief 调用运动学约束的重规划算法
   *
   * 该函数集成了前端路径搜索和后端轨迹优化：
   * 1. 使用kinodynamic A*算法搜索考虑动力学约束的可行路径
   * 2. 将路径转换为B样条曲线
   * 3. 优化B样条轨迹以满足平滑性、安全性和动力学约束
   *
   * @return true 规划成功，生成可行轨迹
   * @return false 规划失败，无法找到可行路径或优化失败
   */
  bool callKinodynamicReplan();

  /**
   * @brief 调用拓扑引导的轨迹优化算法
   *
   * 使用拓扑路径引导的基于梯度的优化方法：
   * 1. 利用拓扑路径库提供多条备选路径
   * 2. 对每条拓扑路径进行梯度优化
   * 3. 选择代价最优的轨迹
   *
   * @param step 优化步骤类型：1-生成新轨迹，2-重规划已有轨迹
   * @return true 优化成功，生成可行轨迹
   * @return false 优化失败，无法找到满足约束的轨迹
   */
  bool callTopologicalTraj(int step);

  /**
   * @brief 改变有限状态机的执行状态
   *
   * @param new_state 新的FSM状态
   * @param pos_call 调用位置标识字符串，用于调试追踪状态转换来源
   */
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);

  /**
   * @brief 打印当前FSM执行状态
   *
   * 在终端输出当前状态机所处状态，用于调试和监控
   */
  void printFSMExecState();

  /* ROS functions */
  /**
   * @brief FSM执行回调函数
   *
   * 由exec_timer_定时触发，负责状态机主循环：
   * 1. 根据当前状态执行相应的规划或控制逻辑
   * 2. 检查状态转换条件
   * 3. 更新状态机状态
   * 4. 发布轨迹和可视化信息
   *
   * @param e ROS定时器事件，包含触发时间等信息
   */
  void execFSMCallback(const ros::TimerEvent& e);

  /**
   * @brief 碰撞检查回调函数
   *
   * 由safety_timer_定时触发，负责安全监控：
   * 1. 检查当前轨迹是否与障碍物碰撞
   * 2. 根据碰撞距离判断是否需要重规划
   * 3. 若检测到危险，触发重规划流程
   *
   * @param e ROS定时器事件
   */
  void checkCollisionCallback(const ros::TimerEvent& e);

  /**
   * @brief 路径点回调函数
   *
   * 订阅路径点话题，接收目标点或路径序列：
   * 1. 解析接收到的路径点消息
   * 2. 更新目标点或路径点列表
   * 3. 设置have_target_标志，触发规划流程
   *
   * @param msg 路径消息指针，包含一系列路径点的位置信息
   */
  void waypointCallback(const nav_msgs::PathConstPtr& msg);

  /**
   * @brief 里程计回调函数
   *
   * 订阅里程计话题，接收机器人状态信息：
   * 1. 更新当前位置、速度、姿态
   * 2. 设置have_odom_标志
   * 3. 为规划提供起始状态
   *
   * @param msg 里程计消息指针，包含位置、速度、姿态等状态信息
   */
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

public:
  /**
   * @brief 默认构造函数
   *
   * 创建KinoReplanFSM对象，成员变量将在init()函数中初始化
   */
  KinoReplanFSM(/* args */) {
  }

  /**
   * @brief 析构函数
   *
   * 清理资源，停止定时器和释放内存
   */
  ~KinoReplanFSM() {
  }

  /**
   * @brief 初始化函数
   *
   * 完成FSM系统的初始化工作：
   * 1. 从参数服务器加载配置参数
   * 2. 初始化规划管理器和可视化工具
   * 3. 创建ROS发布者、订阅者和定时器
   * 4. 设置初始状态
   *
   * @param nh ROS节点句柄，用于创建ROS通信组件
   */
  void init(ros::NodeHandle& nh);

  // Eigen库内存对齐宏，确保使用SSE指令集时的正确内存对齐
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif