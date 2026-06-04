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



#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <fast_bspline_opt/bspline_optimizer.h>  // B样条优化器
#include <bspline/non_uniform_bspline.h>         // 非均匀B样条

#include <fast_path_searching/astar.h>            // A*路径搜索
#include <fast_path_searching/kinodynamic_astar.h> // 动力学A*路径搜索
#include <fast_path_searching/topo_prm.h>         // 拓扑PRM

#include <fast_plan_env/edt_environment.h>        // 欧几里得距离变换环境

#include <plan_manage/plan_container.hpp>         // 规划数据容器

#include <ros/ros.h>

namespace fast_planner {

/**
 * @brief Fast Planner 管理器
 *
 * 该类是Fast Planner的核心管理器，负责调用关键的建图和规划算法。
 * 主要功能包括：
 * - 动力学重规划（kinodynamic replanning）
 * - 全局轨迹规划（global trajectory planning）
 * - 拓扑引导的轨迹优化（topology-guided optimization）
 * - 航向角规划（yaw planning）
 */
class FastPlannerManager {
  // SECTION stable
public:
  /**
   * @brief 构造函数
   */
  FastPlannerManager();

  /**
   * @brief 析构函数
   */
  ~FastPlannerManager();

  /* ========== 主要规划接口 ========== */

  /**
   * @brief 动力学重规划
   *
   * 基于当前状态和目标状态进行动力学约束下的轨迹重规划。
   * 该方法考虑速度、加速度等动力学限制。
   *
   * @param start_pt  起始位置 (x, y, z)
   * @param start_vel 起始速度 (vx, vy, vz)
   * @param start_acc 起始加速度 (ax, ay, az)
   * @param end_pt    目标位置 (x, y, z)
   * @param end_vel   目标速度 (vx, vy, vz)
   * @return true 规划成功
   * @return false 规划失败
   */
  bool kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                         Eigen::Vector3d end_pt, Eigen::Vector3d end_vel);

  /**
   * @brief 全局轨迹规划
   *
   * 从起始位置规划通过所有全局路径点的轨迹。
   *
   * @param start_pos 起始位置
   * @return true 规划成功
   * @return false 规划失败
   */
  bool planGlobalTraj(const Eigen::Vector3d& start_pos);

  /**
   * @brief 拓扑重规划
   *
   * 基于拓扑路径进行轨迹重规划，可以处理碰撞情况。
   *
   * @param collide 是否检测到碰撞
   * @return true 重规划成功
   * @return false 重规划失败
   */
  bool topoReplan(bool collide);

  /**
   * @brief 航向角规划
   *
   * 规划无人机的航向角（yaw）轨迹。
   *
   * @param start_yaw 起始航向角信息 (yaw, yaw_dot, yaw_ddot)
   */
  void planYaw(const Eigen::Vector3d& start_yaw);

  /**
   * @brief 初始化规划模块
   *
   * 初始化所有规划所需的模块，包括路径搜索、优化器、环境等。
   *
   * @param nh ROS节点句柄，用于读取参数
   */
  void initPlanModules(ros::NodeHandle& nh);

  /**
   * @brief 设置全局路径点
   *
   * 设置全局规划需要经过的路径点序列。
   *
   * @param waypoints 路径点向量
   */
  void setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints);

  /**
   * @brief 检查轨迹碰撞
   *
   * 检查当前轨迹是否与障碍物碰撞，并返回最近碰撞点的距离。
   *
   * @param distance 输出参数，返回到最近障碍物的距离
   * @return true 存在碰撞
   * @return false 无碰撞
   */
  bool checkTrajCollision(double& distance);

  /* ========== 公共数据成员 ========== */

  PlanParameters pp_;                  // 规划参数配置
  LocalTrajData local_data_;           // 局部轨迹数据
  GlobalTrajData global_data_;         // 全局轨迹数据
  MidPlanData plan_data_;              // 中间规划数据
  EDTEnvironment::Ptr edt_environment_; // 欧几里得距离变换环境指针

private:
  /* ========== 主要规划算法和模块 ========== */

  SDFMap::Ptr sdf_map_;                              // 有符号距离场地图指针

  unique_ptr<Astar> geo_path_finder_;                // 几何A*路径搜索器
  unique_ptr<KinodynamicAstar> kino_path_finder_;    // 动力学A*路径搜索器
  unique_ptr<TopologyPRM> topo_prm_;                 // 拓扑PRM路径规划器
  vector<BsplineOptimizer::Ptr> bspline_optimizers_; // B样条优化器向量（支持多条候选轨迹）

  /**
   * @brief 更新轨迹信息
   *
   * 更新规划结果中的轨迹相关信息，如持续时间、控制点等。
   */
  void updateTrajInfo();

  /* ========== 拓扑引导优化 ========== */

  /**
   * @brief 查找碰撞范围
   *
   * 在轨迹上查找碰撞段的起点和终点，用于后续的局部重规划。
   *
   * @param colli_start 输出参数，碰撞段起点向量
   * @param colli_end   输出参数，碰撞段终点向量
   * @param start_pts   输出参数，重规划起点向量
   * @param end_pts     输出参数，重规划终点向量
   */
  void findCollisionRange(vector<Eigen::Vector3d>& colli_start, vector<Eigen::Vector3d>& colli_end,
                          vector<Eigen::Vector3d>& start_pts, vector<Eigen::Vector3d>& end_pts);

  /**
   * @brief 优化拓扑B样条轨迹
   *
   * 基于拓扑引导路径优化B样条轨迹。
   *
   * @param start_t    起始时间
   * @param duration   轨迹持续时间
   * @param guide_path 引导路径点序列
   * @param traj_id    轨迹ID（用于多候选轨迹）
   */
  void optimizeTopoBspline(double start_t, double duration, vector<Eigen::Vector3d> guide_path,
                           int traj_id);

  /**
   * @brief 重参数化局部轨迹（自适应时间间隔）
   *
   * 对局部轨迹进行重参数化，自动确定时间间隔和持续时间。
   *
   * @param start_t  起始时间
   * @param dt       输出参数，时间间隔
   * @param duration 输出参数，轨迹持续时间
   * @return Eigen::MatrixXd 重参数化后的控制点矩阵
   */
  Eigen::MatrixXd reparamLocalTraj(double start_t, double& dt, double& duration);

  /**
   * @brief 重参数化局部轨迹（指定段数）
   *
   * 对局部轨迹进行重参数化，使用指定的段数。
   *
   * @param start_t  起始时间
   * @param duration 轨迹持续时间
   * @param seg_num  B样条段数
   * @param dt       输出参数，时间间隔
   * @return Eigen::MatrixXd 重参数化后的控制点矩阵
   */
  Eigen::MatrixXd reparamLocalTraj(double start_t, double duration, int seg_num, double& dt);

  /**
   * @brief 选择最优轨迹
   *
   * 从多条候选轨迹中选择最优的一条。
   *
   * @param traj 输出参数，返回最优轨迹
   */
  void selectBestTraj(NonUniformBspline& traj);

  /**
   * @brief 精化轨迹
   *
   * 对最优轨迹进行进一步优化和精化。
   *
   * @param best_traj 输入/输出参数，需要精化的最优轨迹
   * @param time_inc  输出参数，时间增量
   */
  void refineTraj(NonUniformBspline& best_traj, double& time_inc);

  /**
   * @brief 重参数化B样条
   *
   * 按照给定比例对B样条进行重参数化，调整时间尺度。
   *
   * @param bspline  输入B样条
   * @param ratio    时间缩放比例
   * @param ctrl_pts 输出参数，新的控制点
   * @param dt       输出参数，新的时间间隔
   * @param time_inc 输出参数，时间增量
   */
  void reparamBspline(NonUniformBspline& bspline, double ratio, Eigen::MatrixXd& ctrl_pts, double& dt,
                      double& time_inc);

  /* ========== 航向角规划 ========== */

  /**
   * @brief 计算下一个航向角
   *
   * 基于上一个航向角计算下一个航向角，保证航向角的连续性。
   *
   * @param last_yaw 上一个航向角
   * @param yaw      输出参数，下一个航向角
   */
  void calcNextYaw(const double& last_yaw, double& yaw);

  // !SECTION stable

  // SECTION developing

public:
  /**
   * @brief 智能指针类型定义
   *
   * 用于方便地管理FastPlannerManager对象的生命周期。
   */
  typedef unique_ptr<FastPlannerManager> Ptr;

  // !SECTION
};
}  // namespace fast_planner

#endif