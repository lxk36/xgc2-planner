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



#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <bspline/non_uniform_bspline.h>
#include <poly_traj/polynomial_traj.h>
#include <fast_path_searching/topo_prm.h>

using std::vector;

namespace fast_planner {

/**
 * @brief 全局轨迹数据类
 *
 * 该类管理全局轨迹和局部轨迹的数据，支持轨迹的设置、查询和评估。
 * 全局轨迹使用多项式表示，局部轨迹使用B样条表示。
 * 支持位置、速度、加速度的实时查询和轨迹段的提取。
 */
class GlobalTrajData {
private:
public:
  PolynomialTraj global_traj_;          // 全局多项式轨迹
  vector<NonUniformBspline> local_traj_; // 局部B样条轨迹数组（包含位置、速度、加速度）

  double global_duration_;       // 全局轨迹总时长
  ros::Time global_start_time_;  // 全局轨迹起始时间戳
  double local_start_time_, local_end_time_;  // 局部轨迹的起始和结束时间
  double time_increase_;         // 累计时间增量
  double last_time_inc_;         // 上次时间增量

  /**
   * @brief 默认构造函数
   */
  GlobalTrajData(/* args */) {}

  /**
   * @brief 析构函数
   */
  ~GlobalTrajData() {}

  /**
   * @brief 检查局部轨迹是否到达目标点
   *
   * @return true 如果局部轨迹结束时间与全局轨迹时长相差小于0.1秒，表示已到达目标
   * @return false 否则
   */
  bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

  /**
   * @brief 设置全局轨迹
   *
   * @param traj 多项式轨迹对象
   * @param time 全局轨迹起始时间戳
   *
   * 该函数初始化全局轨迹，并清除局部轨迹信息。
   */
  void setGlobalTraj(const PolynomialTraj& traj, const ros::Time& time) {
    global_traj_ = traj;
    global_traj_.init();
    global_duration_ = global_traj_.getTimeSum();
    global_start_time_ = time;

    local_traj_.clear();
    local_start_time_ = -1;
    local_end_time_ = -1;
    time_increase_ = 0.0;
    last_time_inc_ = 0.0;
  }

  /**
   * @brief 设置局部轨迹
   *
   * @param traj 非均匀B样条轨迹（位置）
   * @param local_ts 局部轨迹起始时间
   * @param local_te 局部轨迹结束时间
   * @param time_inc 时间增量
   *
   * 该函数设置局部轨迹及其一阶、二阶导数（速度和加速度），
   * 并更新全局时长和时间增量。
   */
  void setLocalTraj(NonUniformBspline traj, double local_ts, double local_te, double time_inc) {
    local_traj_.resize(3);
    local_traj_[0] = traj;
    local_traj_[1] = local_traj_[0].getDerivative();
    local_traj_[2] = local_traj_[1].getDerivative();

    local_start_time_ = local_ts;
    local_end_time_ = local_te;
    global_duration_ += time_inc;
    time_increase_ += time_inc;
    last_time_inc_ = time_inc;
  }

  /**
   * @brief 获取指定时间点的位置
   *
   * @param t 查询时间
   * @return Eigen::Vector3d 三维位置向量
   *
   * 根据查询时间所在的区间，从全局轨迹或局部轨迹中获取位置：
   * - 如果t在局部轨迹开始前，从全局轨迹获取
   * - 如果t在局部轨迹结束后，从全局轨迹获取（考虑时间增量）
   * - 如果t在局部轨迹内，从局部B样条轨迹获取
   */
  Eigen::Vector3d getPosition(double t) {
    if (t >= -1e-3 && t <= local_start_time_) {
      return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
    } else if (t >= local_end_time_ && t <= global_duration_ + 1e-3) {
      return global_traj_.evaluate(t - time_increase_);
    } else {
      double tm, tmp;
      local_traj_[0].getTimeSpan(tm, tmp);
      return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
    }
  }

  /**
   * @brief 获取指定时间点的速度
   *
   * @param t 查询时间
   * @return Eigen::Vector3d 三维速度向量
   *
   * 根据查询时间所在的区间，从全局轨迹或局部轨迹中获取速度：
   * - 如果t在局部轨迹开始前，从全局轨迹获取
   * - 如果t在局部轨迹结束后，从全局轨迹获取（考虑时间增量）
   * - 如果t在局部轨迹内，从局部B样条轨迹的一阶导数获取
   */
  Eigen::Vector3d getVelocity(double t) {
    if (t >= -1e-3 && t <= local_start_time_) {
      return global_traj_.evaluateVel(t);
    } else if (t >= local_end_time_ && t <= global_duration_ + 1e-3) {
      return global_traj_.evaluateVel(t - time_increase_);
    } else {
      double tm, tmp;
      local_traj_[0].getTimeSpan(tm, tmp);
      return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
    }
  }

  /**
   * @brief 获取指定时间点的加速度
   *
   * @param t 查询时间
   * @return Eigen::Vector3d 三维加速度向量
   *
   * 根据查询时间所在的区间，从全局轨迹或局部轨迹中获取加速度：
   * - 如果t在局部轨迹开始前，从全局轨迹获取
   * - 如果t在局部轨迹结束后，从全局轨迹获取（考虑时间增量）
   * - 如果t在局部轨迹内，从局部B样条轨迹的二阶导数获取
   */
  Eigen::Vector3d getAcceleration(double t) {
    if (t >= -1e-3 && t <= local_start_time_) {
      return global_traj_.evaluateAcc(t);
    } else if (t >= local_end_time_ && t <= global_duration_ + 1e-3) {
      return global_traj_.evaluateAcc(t - time_increase_);
    } else {
      double tm, tmp;
      local_traj_[0].getTimeSpan(tm, tmp);
      return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_);
    }
  }

  /**
   * @brief 在球形范围内获取局部轨迹的B样条参数化数据
   *
   * @param start_t 轨迹起始时间
   * @param des_radius 期望的球形半径（从起始点开始计算）
   * @param dist_pt 离散点之间的距离
   * @param point_set [输出] 离散化的轨迹点集合
   * @param start_end_derivative [输出] 起点和终点的速度和加速度（依次为：起点速度、终点速度、起点加速度、终点加速度）
   * @param dt [输出] 相邻两点的时间间隔
   * @param seg_duration [输出] 截取的轨迹段时长
   *
   * 该函数从起始时间开始向前截取轨迹，直到轨迹距离起始点的半径超过期望半径或达到全局时长。
   * 然后根据期望的点密度对轨迹段进行离散化采样。
   */
  void getTrajByRadius(const double& start_t, const double& des_radius, const double& dist_pt,
                       vector<Eigen::Vector3d>& point_set, vector<Eigen::Vector3d>& start_end_derivative,
                       double& dt, double& seg_duration) {
    double seg_length = 0.0;  // 截取轨迹段的长度
    double seg_time = 0.0;    // 截取轨迹段的时长
    double radius = 0.0;      // 当前点到轨迹段起点的距离

    double delt = 0.2;
    Eigen::Vector3d first_pt = getPosition(start_t);  // 轨迹段的第一个点
    Eigen::Vector3d prev_pt = first_pt;               // 上一个采样点
    Eigen::Vector3d cur_pt;                           // 当前采样点

    // 向前推进直到轨迹超过指定半径或达到全局时间

    while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3) {
      seg_time += delt;
      seg_time = min(seg_time, global_duration_ - start_t);

      cur_pt = getPosition(start_t + seg_time);
      seg_length += (cur_pt - prev_pt).norm();
      prev_pt = cur_pt;
      radius = (cur_pt - first_pt).norm();
    }

    // 根据期望的点密度计算参数化的时间间隔dt
    int seg_num = floor(seg_length / dist_pt);

    // 获取输出结果

    seg_duration = seg_time;  // 截取轨迹段的时长
    dt = seg_time / seg_num;  // 相邻两点之间的时间间隔

    for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt) {
      cur_pt = getPosition(start_t + tp);
      point_set.push_back(cur_pt);
    }

    start_end_derivative.push_back(getVelocity(start_t));
    start_end_derivative.push_back(getVelocity(start_t + seg_time));
    start_end_derivative.push_back(getAcceleration(start_t));
    start_end_derivative.push_back(getAcceleration(start_t + seg_time));
  }

  /**
   * @brief 获取固定时长的局部轨迹的B样条参数化数据
   *
   * @param start_t 轨迹起始时间
   * @param duration 轨迹段的时长
   * @param seg_num 将轨迹段离散化成seg_num段
   * @param point_set [输出] 离散化的轨迹点集合
   * @param start_end_derivative [输出] 起点和终点的速度和加速度（依次为：起点速度、终点速度、起点加速度、终点加速度）
   * @param dt [输出] 相邻两点的时间间隔
   *
   * 该函数从起始时间开始截取固定时长的轨迹段，并根据指定的段数进行均匀离散化采样。
   */
  void getTrajByDuration(double start_t, double duration, int seg_num,
                         vector<Eigen::Vector3d>& point_set,
                         vector<Eigen::Vector3d>& start_end_derivative, double& dt) {
    dt = duration / seg_num;
    Eigen::Vector3d cur_pt;
    for (double tp = 0.0; tp <= duration + 1e-4; tp += dt) {
      cur_pt = getPosition(start_t + tp);
      point_set.push_back(cur_pt);
    }

    start_end_derivative.push_back(getVelocity(start_t));
    start_end_derivative.push_back(getVelocity(start_t + duration));
    start_end_derivative.push_back(getAcceleration(start_t));
    start_end_derivative.push_back(getAcceleration(start_t + duration));
  }
};

/**
 * @brief 规划参数结构体
 *
 * 该结构体存储规划算法的各种参数，包括物理限制、轨迹参数、处理时间等。
 */
struct PlanParameters {
  /* 规划算法参数 */
  double max_vel_, max_acc_, max_jerk_;  // 物理限制：最大速度、最大加速度、最大加加速度
  double local_traj_len_;                // 局部重规划轨迹长度
  double ctrl_pt_dist;                   // 相邻B样条控制点之间的距离
  double clearance_;                     // 安全间隙（与障碍物的最小距离）
  int dynamic_;                          // 动态环境标志（1表示动态环境，0表示静态环境）
  /* 处理时间统计 */
  double time_search_ = 0.0;             // 路径搜索耗时
  double time_optimize_ = 0.0;           // 轨迹优化耗时
  double time_adjust_ = 0.0;             // 轨迹调整耗时
};

/**
 * @brief 局部轨迹数据结构体
 *
 * 该结构体存储生成的局部轨迹的完整信息，包括位置、速度、加速度、偏航角及其导数的B样条表示。
 */
struct LocalTrajData {
  /* 生成轨迹的信息 */

  int traj_id_;                  // 轨迹ID，用于标识不同的轨迹
  double duration_;              // 轨迹持续时间
  ros::Time start_time_;         // 轨迹起始时间戳
  Eigen::Vector3d start_pos_;    // 轨迹起始位置
  NonUniformBspline position_traj_, velocity_traj_, acceleration_traj_, yaw_traj_, yawdot_traj_,
      yawdotdot_traj_;           // 位置、速度、加速度、偏航角、偏航角速度、偏航角加速度的B样条轨迹
};

/**
 * @brief 中间规划数据类
 *
 * 该类存储规划过程中的中间数据，包括全局路径点、初始轨迹段、
 * 运动学路径、拓扑路径、拓扑轨迹、可见性约束和航向规划等信息。
 * 主要用于在规划的不同阶段之间传递和共享数据。
 */
class MidPlanData {
public:
  /**
   * @brief 默认构造函数
   */
  MidPlanData(/* args */) {}

  /**
   * @brief 析构函数
   */
  ~MidPlanData() {}

  vector<Eigen::Vector3d> global_waypoints_;  // 全局路径点集合

  // 初始轨迹段
  NonUniformBspline initial_local_segment_;         // 初始的局部轨迹段（B样条）
  vector<Eigen::Vector3d> local_start_end_derivative_;  // 局部轨迹起点和终点的导数信息

  // 运动学路径
  vector<Eigen::Vector3d> kino_path_;  // 运动学A*搜索得到的路径

  // 拓扑路径
  list<GraphNode::Ptr> topo_graph_;                      // 拓扑图的节点列表
  vector<vector<Eigen::Vector3d>> topo_paths_;           // 拓扑路径集合
  vector<vector<Eigen::Vector3d>> topo_filtered_paths_;  // 过滤后的拓扑路径
  vector<vector<Eigen::Vector3d>> topo_select_paths_;    // 选择的拓扑路径

  // 多条拓扑轨迹
  vector<NonUniformBspline> topo_traj_pos1_;  // 第一阶段的拓扑轨迹集合
  vector<NonUniformBspline> topo_traj_pos2_;  // 第二阶段的拓扑轨迹集合
  vector<NonUniformBspline> refines_;         // 精化后的轨迹集合

  // 可见性约束
  vector<Eigen::Vector3d> block_pts_;  // 阻挡点集合（用于可见性约束）
  Eigen::MatrixXd ctrl_pts_;           // 控制点矩阵

  // 航向规划
  vector<double> path_yaw_;   // 路径上的偏航角序列
  double dt_yaw_;             // 偏航角的时间间隔
  double dt_yaw_path_;        // 路径偏航角的时间间隔

  /**
   * @brief 清除所有拓扑路径相关数据
   *
   * 该函数清空拓扑轨迹、拓扑图和所有拓扑路径的数据，
   * 用于重新开始拓扑路径规划。
   */
  void clearTopoPaths() {
    topo_traj_pos1_.clear();
    topo_traj_pos2_.clear();
    topo_graph_.clear();
    topo_paths_.clear();
    topo_filtered_paths_.clear();
    topo_select_paths_.clear();
  }

  /**
   * @brief 添加拓扑路径数据
   *
   * @param graph 拓扑图的节点列表
   * @param paths 所有拓扑路径
   * @param filtered_paths 过滤后的拓扑路径
   * @param selected_paths 选择的拓扑路径
   *
   * 该函数将拓扑路径搜索的结果保存到类成员变量中。
   */
  void addTopoPaths(list<GraphNode::Ptr>& graph, vector<vector<Eigen::Vector3d>>& paths,
                    vector<vector<Eigen::Vector3d>>& filtered_paths,
                    vector<vector<Eigen::Vector3d>>& selected_paths) {
    topo_graph_ = graph;
    topo_paths_ = paths;
    topo_filtered_paths_ = filtered_paths;
    topo_select_paths_ = selected_paths;
  }
};

}  // namespace fast_planner

#endif