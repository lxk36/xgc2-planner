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



// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace fast_planner {

// SECTION interfaces for setup and query

/**
 * @brief FastPlannerManager构造函数
 *
 * 默认构造函数，用于初始化规划管理器实例
 */
FastPlannerManager::FastPlannerManager() {}

/**
 * @brief FastPlannerManager析构函数
 *
 * 析构函数，输出销毁信息用于调试
 */
FastPlannerManager::~FastPlannerManager() { std::cout << "des manager" << std::endl; }

/**
 * @brief 初始化规划模块
 *
 * 从ROS参数服务器读取配置参数，并初始化各个规划模块（几何路径规划、动力学路径规划、拓扑路径规划、B样条优化等）
 *
 * @param nh ROS节点句柄，用于读取参数
 */
void FastPlannerManager::initPlanModules(ros::NodeHandle& nh) {
  /* read algorithm parameters */
  /* 读取算法参数 */

  // 读取最大速度、加速度、加加速度等运动学约束参数
  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  // 读取是否为动态环境的标志
  nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
  // 读取安全间隙阈值
  nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
  // 读取局部轨迹段长度
  nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
  // 读取控制点之间的距离
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);

  // 读取各个规划模块的使能标志
  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization, use_active_perception;
  nh.param("manager/use_geometric_path", use_geometric_path, false);      // 是否使用几何路径规划
  nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);  // 是否使用动力学路径规划
  nh.param("manager/use_topo_path", use_topo_path, false);                // 是否使用拓扑路径规划
  nh.param("manager/use_optimization", use_optimization, false);          // 是否使用轨迹优化

  // 初始化轨迹ID
  local_data_.traj_id_ = 0;

  // 创建并初始化符号距离场(SDF)地图
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);

  // 创建并初始化欧几里得距离变换(EDT)环境
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  // 如果启用几何路径规划，初始化A*算法
  if (use_geometric_path) {
    geo_path_finder_.reset(new Astar);
    geo_path_finder_->setParam(nh);
    geo_path_finder_->setEnvironment(edt_environment_);
    geo_path_finder_->init();
  }

  // 如果启用动力学路径规划，初始化Kinodynamic A*算法
  if (use_kinodynamic_path) {
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
  }

  // 如果启用轨迹优化，初始化B样条优化器数组（最多10个优化器，用于并行优化多条轨迹）
  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  // 如果启用拓扑路径规划，初始化拓扑PRM算法
  if (use_topo_path) {
    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setEnvironment(edt_environment_);
    topo_prm_->init(nh);
  }
}

/**
 * @brief 设置全局航点
 *
 * 设置全局路径规划所需的航点序列
 *
 * @param waypoints 航点的3D坐标向量
 */
void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

/**
 * @brief 检查轨迹碰撞
 *
 * 从当前时刻开始，沿着轨迹向前检查是否存在碰撞。检查范围为当前点周围6米半径内。
 *
 * @param distance 输出参数，如果发生碰撞，返回碰撞点到当前点的距离
 * @return true 轨迹安全，无碰撞
 * @return false 轨迹存在碰撞
 */
bool FastPlannerManager::checkTrajCollision(double& distance) {

  // 计算当前时刻相对于轨迹起始时间的时间差
  double t_now = (ros::Time::now() - local_data_.start_time_).toSec();

  // 获取轨迹的时间范围
  double tm, tmp;
  local_data_.position_traj_.getTimeSpan(tm, tmp);
  // 计算当前位置点
  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now);

  double          radius = 0.0;    // 当前检查点到起始点的距离
  Eigen::Vector3d fut_pt;          // 未来检查点
  double          fut_t = 0.02;    // 向前检查的时间步长（20ms）

  // 在6米半径范围内，沿轨迹向前检查碰撞
  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    // 获取未来时刻的轨迹点
    fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);

    // 评估该点到最近障碍物的距离
    double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
    if (dist < 0.1) {  // 距离小于0.1米认为发生碰撞
      distance = radius;
      return false;
    }

    // 更新检查半径和时间
    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

// !SECTION

// SECTION kinodynamic replanning

/**
 * @brief 动力学重规划
 *
 * 使用Kinodynamic A*算法进行路径搜索，然后将路径参数化为B样条曲线，
 * 最后进行轨迹优化和时间分配调整
 *
 * @param start_pt 起始位置
 * @param start_vel 起始速度
 * @param start_acc 起始加速度
 * @param end_pt 目标位置
 * @param end_vel 目标速度
 * @return true 规划成功
 * @return false 规划失败（目标点过近或无法找到路径）
 */
bool FastPlannerManager::kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
                                           Eigen::Vector3d end_vel) {

  std::cout << "[kino replan]: -----------------------" << std::endl;
  cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << ", "
       << start_acc.transpose() << "\ngoal:" << end_pt.transpose() << ", " << end_vel.transpose()
       << endl;

  // 如果起点和终点距离过近，不进行规划
  if ((start_pt - end_pt).norm() < 0.2) {
    cout << "Close goal" << endl;
    return false;
  }

  ros::Time t1, t2;

  // 记录轨迹开始时间
  local_data_.start_time_ = ros::Time::now();
  // 初始化各阶段耗时统计变量
  double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

  // 保存初始状态
  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // kinodynamic path searching
  /* 动力学路径搜索 */

  t1 = ros::Time::now();

  // 重置路径查找器
  kino_path_finder_->reset();

  // 使用Kinodynamic A*算法搜索路径（初始状态连续）
  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);

  if (status == KinodynamicAstar::NO_PATH) {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // retry searching with discontinuous initial state
    /* 如果搜索失败，使用不连续的初始状态重试 */
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[kino replan]: Can't find path." << endl;
      return false;
    } else {
      cout << "[kino replan]: retry search success." << endl;
    }

  } else {
    cout << "[kino replan]: kinodynamic search success." << endl;
  }

  // 以0.01秒的采样间隔获取动力学轨迹
  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

  // 记录搜索耗时
  t_search = (ros::Time::now() - t1).toSec();

  // parameterize the path to bspline
  /* 将路径参数化为B样条曲线 */

  // 计算时间间隔：控制点距离除以最大速度
  double                  ts = pp_.ctrl_pt_dist / pp_.max_vel_;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  // 对路径进行采样，获取采样点集和起止点导数
  kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

  // 将采样点参数化为B样条控制点
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  NonUniformBspline init(ctrl_pts, 3, ts);

  // bspline trajectory optimization
  /* B样条轨迹优化 */

  t1 = ros::Time::now();

  // 设置代价函数为正常阶段（包含平滑性、碰撞避免、动力学可行性等）
  int cost_function = BsplineOptimizer::NORMAL_PHASE;

  // 如果没有到达终点，添加终点约束
  if (status != KinodynamicAstar::REACH_END) {
    cost_function |= BsplineOptimizer::ENDPOINT;
  }

  // 执行B样条轨迹优化
  ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

  // 记录优化耗时
  t_opt = (ros::Time::now() - t1).toSec();

  // iterative time adjustment
  /* 迭代式时间调整 */

  t1                    = ros::Time::now();
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);

  // 记录原始总时间
  double to = pos.getTimeSum();
  // 设置物理限制（速度和加速度上限）
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  // 检查轨迹是否满足动力学约束
  bool feasible = pos.checkFeasibility(false);

  int iter_num = 0;
  // 如果不满足约束，进行时间重新分配（最多迭代3次）
  while (!feasible && ros::ok()) {

    feasible = pos.reallocateTime();

    if (++iter_num >= 3) break;
  }

  // pos.checkFeasibility(true);
  // cout << "[Main]: iter num: " << iter_num << endl;

  // 记录调整后的总时间
  double tn = pos.getTimeSum();

  // 输出时间重分配比例
  cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
  if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  // 记录时间调整耗时
  t_adjust = (ros::Time::now() - t1).toSec();

  // save planned results
  /* 保存规划结果 */

  local_data_.position_traj_ = pos;

  // 计算并输出总耗时和各阶段耗时
  double t_total = t_search + t_opt + t_adjust;
  cout << "[kino replan]: time: " << t_total << ", search: " << t_search << ", optimize: " << t_opt
       << ", adjust time:" << t_adjust << endl;

  // 保存各阶段耗时统计
  pp_.time_search_   = t_search;
  pp_.time_optimize_ = t_opt;
  pp_.time_adjust_   = t_adjust;

  // 更新轨迹相关信息（速度、加速度、持续时间等）
  updateTrajInfo();

  return true;
}

// !SECTION

// SECTION topological replanning

/**
 * @brief 全局轨迹规划
 *
 * 根据给定的全局航点序列，生成平滑的全局参考轨迹（使用minimum snap方法），
 * 并从中截取一段局部轨迹用于后续的局部规划
 *
 * @param start_pos 起始位置
 * @return true 规划成功
 * @return false 规划失败
 */
bool FastPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos) {
  // 清空之前的拓扑路径数据
  plan_data_.clearTopoPaths();

  // generate global reference trajectory
  /* 生成全局参考轨迹 */

  // 获取全局航点
  vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
  if (points.size() == 0) std::cout << "no global waypoints!" << std::endl;

  // 将起始位置插入到航点序列开头
  points.insert(points.begin(), start_pos);

  // insert intermediate points if too far
  /* 如果航点间距离过远，插入中间点 */
  vector<Eigen::Vector3d> inter_points;
  const double            dist_thresh = 4.0;  // 距离阈值：4米

  for (int i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();

    // 如果两个航点之间距离超过阈值，进行插值
    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;

      // 线性插值生成中间点
      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }

  inter_points.push_back(points.back());
  // 如果只有两个点，在中间插入一个点（确保至少有3个点用于轨迹生成）
  if (inter_points.size() == 2) {
    Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
    inter_points.insert(inter_points.begin() + 1, mid);
  }

  // write position matrix
  /* 构造位置矩阵和时间分配 */
  int             pt_num = inter_points.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = inter_points[i];

  Eigen::Vector3d zero(0, 0, 0);  // 零向量，用于速度和加速度边界条件
  Eigen::VectorXd time(pt_num - 1);
  // 根据航点间距离和最大速度估算每段的时间
  for (int i = 0; i < pt_num - 1; ++i) {
    time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);
  }

  // 增加起始段和结束段的时间（乘以2），确保平滑加减速
  time(0) *= 2.0;
  time(0) = max(1.0, time(0));
  time(time.rows() - 1) *= 2.0;
  time(time.rows() - 1) = max(1.0, time(time.rows() - 1));

  // 使用minimum snap方法生成多项式轨迹
  PolynomialTraj gl_traj = minSnapTraj(pos, zero, zero, zero, zero, time);

  // 记录全局轨迹的起始时间
  auto time_now = ros::Time::now();
  global_data_.setGlobalTraj(gl_traj, time_now);

  // truncate a local trajectory
  /* 从全局轨迹中截取局部轨迹段 */

  double            dt, duration;
  // 重新参数化局部轨迹为B样条形式
  Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(0.0, dt, duration);
  NonUniformBspline bspline(ctrl_pts, 3, dt);

  // 设置局部轨迹
  global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
  local_data_.position_traj_ = bspline;
  local_data_.start_time_    = time_now;
  ROS_INFO("global trajectory generated.");

  // 更新轨迹相关信息
  updateTrajInfo();

  return true;
}

/**
 * @brief 拓扑重规划
 *
 * 当检测到轨迹碰撞时，使用拓扑路径规划方法生成多条拓扑不同的候选路径，
 * 对每条路径进行并行优化，最后选择代价最小的轨迹
 *
 * @param collide 是否检测到碰撞
 * @return true 规划成功
 * @return false 规划失败（无法找到有效路径）
 */
bool FastPlannerManager::topoReplan(bool collide) {
  ros::Time t1, t2;

  /* truncate a new local segment for replanning */
  /* 截取新的局部轨迹段用于重规划 */
  ros::Time time_now = ros::Time::now();
  double    t_now    = (time_now - global_data_.global_start_time_).toSec();
  double    local_traj_dt, local_traj_duration;
  double    time_inc = 0.0;  // 时间增量

  // 从全局轨迹中重新参数化局部轨迹
  Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(t_now, local_traj_dt, local_traj_duration);
  NonUniformBspline init_traj(ctrl_pts, 3, local_traj_dt);
  local_data_.start_time_ = time_now;

  if (!collide) {  // simply truncate the segment and do nothing
    // 如果没有碰撞，只需对轨迹进行精化处理
    refineTraj(init_traj, time_inc);
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);

  } else {
    // 发生碰撞，需要进行拓扑重规划
    plan_data_.initial_local_segment_ = init_traj;
    vector<Eigen::Vector3d> colli_start, colli_end, start_pts, end_pts;
    // 查找碰撞区域的起止范围
    findCollisionRange(colli_start, colli_end, start_pts, end_pts);

    if (colli_start.size() == 1 && colli_end.size() == 0) {
      // 初始轨迹终点在障碍物内，无法重规划
      ROS_WARN("Init traj ends in obstacle, no replanning.");
      local_data_.position_traj_ = init_traj;
      global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);

    } else {
      NonUniformBspline best_traj;  // 存储最优轨迹

      // local segment is in collision, call topological replanning
      /* search topological distinctive paths */
      /* 搜索拓扑不同的路径 */
      ROS_INFO("[Topo]: ---------");
      plan_data_.clearTopoPaths();
      list<GraphNode::Ptr>            graph;
      vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
      // 查找拓扑不同的路径（避开碰撞区域）
      topo_prm_->findTopoPaths(colli_start.front(), colli_end.back(), start_pts, end_pts, graph,
                               raw_paths, filtered_paths, select_paths);

      if (select_paths.size() == 0) {
        ROS_WARN("No path.");
        return false;
      }
      // 添加拓扑路径到规划数据中
      plan_data_.addTopoPaths(graph, raw_paths, filtered_paths, select_paths);

      /* optimize trajectory using different topo paths */
      /* 使用不同的拓扑路径优化轨迹 */
      ROS_INFO("[Optimize]: ---------");
      t1 = ros::Time::now();

      // 为每条拓扑路径分配存储空间
      plan_data_.topo_traj_pos1_.resize(select_paths.size());
      plan_data_.topo_traj_pos2_.resize(select_paths.size());
      vector<thread> optimize_threads;
      // 并行优化每条拓扑路径（使用多线程）
      for (int i = 0; i < select_paths.size(); ++i) {
        optimize_threads.emplace_back(&FastPlannerManager::optimizeTopoBspline, this, t_now,
                                      local_traj_duration, select_paths[i], i);
        // optimizeTopoBspline(t_now, local_traj_duration,
        // select_paths[i], origin_len, i);
      }
      // 等待所有优化线程完成
      for (int i = 0; i < select_paths.size(); ++i) optimize_threads[i].join();

      double t_opt = (ros::Time::now() - t1).toSec();
      cout << "[planner]: optimization time: " << t_opt << endl;
      // 从优化后的轨迹中选择最优的一条
      selectBestTraj(best_traj);
      // 对最优轨迹进行精化
      refineTraj(best_traj, time_inc);

      // 保存最优轨迹
      local_data_.position_traj_ = best_traj;
      global_data_.setLocalTraj(local_data_.position_traj_, t_now,
                                local_traj_duration + time_inc + t_now, time_inc);
    }
  }
  // 更新轨迹信息
  updateTrajInfo();
  return true;
}

/**
 * @brief 选择最优轨迹
 *
 * 从多条候选轨迹中选择加加速度(jerk)最小的轨迹作为最优轨迹
 *
 * @param traj 输出参数，返回选中的最优轨迹
 */
void FastPlannerManager::selectBestTraj(NonUniformBspline& traj) {
  // sort by jerk
  /* 按加加速度排序 */
  vector<NonUniformBspline>& trajs = plan_data_.topo_traj_pos2_;
  sort(trajs.begin(), trajs.end(),
       [&](NonUniformBspline& tj1, NonUniformBspline& tj2) { return tj1.getJerk() < tj2.getJerk(); });
  // 选择jerk最小的轨迹
  traj = trajs[0];
}

/**
 * @brief 精化轨迹
 *
 * 对轨迹进行精化处理，包括检查动力学约束、时间重新分配和再次优化
 *
 * @param best_traj 输入输出参数，待精化的轨迹，会被修改为精化后的结果
 * @param time_inc 输出参数，返回时间增量
 */
void FastPlannerManager::refineTraj(NonUniformBspline& best_traj, double& time_inc) {
  ros::Time t1 = ros::Time::now();
  time_inc     = 0.0;
  double    dt, t_inc;
  const int max_iter = 1;

  // int cost_function = BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::VISIBILITY;
  Eigen::MatrixXd ctrl_pts      = best_traj.getControlPoint();
  int             cost_function = BsplineOptimizer::NORMAL_PHASE;

  // 设置物理限制并检查可行性比率
  best_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  double ratio = best_traj.checkRatio();
  std::cout << "ratio: " << ratio << std::endl;
  // 根据比率重新参数化B样条（如果不满足约束则拉长时间）
  reparamBspline(best_traj, ratio, ctrl_pts, dt, t_inc);
  time_inc += t_inc;

  // 再次优化轨迹
  ctrl_pts  = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, dt, cost_function, 1, 1);
  best_traj = NonUniformBspline(ctrl_pts, 3, dt);
  ROS_WARN_STREAM("[Refine]: cost " << (ros::Time::now() - t1).toSec()
                                    << " seconds, time change is: " << time_inc);
}

/**
 * @brief 更新轨迹信息
 *
 * 根据位置轨迹计算速度轨迹、加速度轨迹，更新起始位置、持续时间和轨迹ID
 */
void FastPlannerManager::updateTrajInfo() {
  // 计算速度轨迹（位置轨迹的一阶导数）
  local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
  // 计算加速度轨迹（速度轨迹的一阶导数）
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  // 获取轨迹起始位置
  local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
  // 获取轨迹总时长
  local_data_.duration_          = local_data_.position_traj_.getTimeSum();
  // 轨迹ID自增
  local_data_.traj_id_ += 1;
}

/**
 * @brief 重新参数化B样条曲线
 *
 * 根据给定的比率调整B样条的时间分配，然后重新采样并参数化
 *
 * @param bspline 输入的B样条曲线
 * @param ratio 时间拉伸比率（>1表示拉长时间）
 * @param ctrl_pts 输出参数，返回新的控制点
 * @param dt 输出参数，返回新的时间间隔
 * @param time_inc 输出参数，返回时间增量
 */
void FastPlannerManager::reparamBspline(NonUniformBspline& bspline, double ratio,
                                        Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
  int    prev_num    = bspline.getControlPoint().rows();
  double time_origin = bspline.getTimeSum();
  int    seg_num     = bspline.getControlPoint().rows() - 3;  // B样条段数
  // double length = bspline.getLength(0.1);
  // int seg_num = ceil(length / pp_.ctrl_pt_dist);

  // 限制比率不超过1.01
  ratio = min(1.01, ratio);
  // 拉长时间
  bspline.lengthenTime(ratio);
  double duration = bspline.getTimeSum();
  dt              = duration / double(seg_num);
  time_inc        = duration - time_origin;

  // 重新采样轨迹点
  vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt) {
    point_set.push_back(bspline.evaluateDeBoorT(time));
  }
  // 将采样点重新参数化为B样条控制点
  NonUniformBspline::parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_,
                                           ctrl_pts);
  // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
}

/**
 * @brief 优化拓扑B样条轨迹
 *
 * 对给定的拓扑路径进行两阶段优化：
 * 第一阶段：路径引导优化，使轨迹靠近引导路径
 * 第二阶段：正常优化，优化平滑性和动力学可行性
 *
 * @param start_t 起始时间
 * @param duration 持续时间
 * @param guide_path 引导路径的航点序列
 * @param traj_id 轨迹ID，用于在并行优化中标识不同的轨迹
 */
void FastPlannerManager::optimizeTopoBspline(double start_t, double duration,
                                             vector<Eigen::Vector3d> guide_path, int traj_id) {
  ros::Time t1;
  double    tm1, tm2, tm3;  // 各阶段耗时

  t1 = ros::Time::now();

  // parameterize B-spline according to the length of guide path
  /* 根据引导路径的长度参数化B样条 */
  int             seg_num = topo_prm_->pathLength(guide_path) / pp_.ctrl_pt_dist;
  Eigen::MatrixXd ctrl_pts;
  double          dt;

  // 重新参数化局部轨迹
  ctrl_pts = reparamLocalTraj(start_t, duration, seg_num, dt);
  // std::cout << "ctrl pt num: " << ctrl_pts.rows() << std::endl;

  // discretize the guide path and align it with B-spline control points
  /* 离散化引导路径，并与B样条控制点对齐 */
  vector<Eigen::Vector3d> guide_pt;
  guide_pt = topo_prm_->pathToGuidePts(guide_path, int(ctrl_pts.rows()) - 2);

  // 移除首尾各两个点（边界点由起止条件约束）
  guide_pt.pop_back();
  guide_pt.pop_back();
  guide_pt.erase(guide_pt.begin(), guide_pt.begin() + 2);

  // std::cout << "guide pt num: " << guide_pt.size() << std::endl;
  if (guide_pt.size() != int(ctrl_pts.rows()) - 6) ROS_WARN("what guide");

  tm1 = (ros::Time::now() - t1).toSec();
  t1  = ros::Time::now();

  // first phase, path-guided optimization
  /* 第一阶段：路径引导优化 */

  // 设置引导路径
  bspline_optimizers_[traj_id]->setGuidePath(guide_pt);
  // 执行引导阶段优化（使轨迹靠近引导路径）
  Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      ctrl_pts, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);

  // 保存第一阶段优化结果
  plan_data_.topo_traj_pos1_[traj_id] = NonUniformBspline(opt_ctrl_pts1, 3, dt);

  tm2 = (ros::Time::now() - t1).toSec();
  t1  = ros::Time::now();

  // second phase, normal optimization
  /* 第二阶段：正常优化 */

  // 在第一阶段结果基础上进行正常优化（优化平滑性、碰撞避免、动力学约束等）
  Eigen::MatrixXd opt_ctrl_pts2 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      opt_ctrl_pts1, dt, BsplineOptimizer::NORMAL_PHASE, 1, 1);

  // 保存第二阶段优化结果（最终轨迹）
  plan_data_.topo_traj_pos2_[traj_id] = NonUniformBspline(opt_ctrl_pts2, 3, dt);

  tm3 = (ros::Time::now() - t1).toSec();
  ROS_INFO("optimization %d cost %lf, %lf, %lf seconds.", traj_id, tm1, tm2, tm3);
}

/**
 * @brief 重新参数化局部轨迹（基于半径）
 *
 * 从全局轨迹中提取指定半径范围内的局部轨迹段，并将其参数化为B样条形式
 *
 * @param start_t 起始时间
 * @param dt 输出参数，返回时间间隔
 * @param duration 输出参数，返回持续时间
 * @return Eigen::MatrixXd 返回B样条控制点矩阵
 */
Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double& dt, double& duration) {
  /* get the sample points local traj within radius */
  /* 获取半径范围内的局部轨迹采样点 */

  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  // 从全局轨迹中按半径提取局部轨迹段
  global_data_.getTrajByRadius(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set,
                               start_end_derivative, dt, duration);

  /* parameterization of B-spline */
  /* B样条参数化 */

  Eigen::MatrixXd ctrl_pts;
  // 将采样点参数化为B样条控制点
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  // 保存局部轨迹的起止导数（速度和加速度）
  plan_data_.local_start_end_derivative_ = start_end_derivative;
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

/**
 * @brief 重新参数化局部轨迹（基于持续时间和段数）
 *
 * 从全局轨迹中提取指定时长的局部轨迹段，并按照给定的段数进行采样和参数化
 *
 * @param start_t 起始时间
 * @param duration 持续时间
 * @param seg_num B样条段数
 * @param dt 输出参数，返回时间间隔
 * @return Eigen::MatrixXd 返回B样条控制点矩阵
 */
Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double duration, int seg_num,
                                                     double& dt) {
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  // 从全局轨迹中按持续时间提取局部轨迹段
  global_data_.getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);
  // 保存局部轨迹的起止导数
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  /* parameterization of B-spline */
  /* B样条参数化 */
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

/**
 * @brief 查找碰撞区域范围
 *
 * 沿着初始轨迹搜索碰撞区域，找出碰撞段的起止点，
 * 以及碰撞前的安全段和碰撞后的安全段采样点
 *
 * @param colli_start 输出参数，碰撞区域的起始点集合
 * @param colli_end 输出参数，碰撞区域的结束点集合
 * @param start_pts 输出参数，碰撞前安全段的采样点
 * @param end_pts 输出参数，碰撞后安全段的采样点
 */
void FastPlannerManager::findCollisionRange(vector<Eigen::Vector3d>& colli_start,
                                            vector<Eigen::Vector3d>& colli_end,
                                            vector<Eigen::Vector3d>& start_pts,
                                            vector<Eigen::Vector3d>& end_pts) {
  bool               last_safe = true, safe;  // 上一个点和当前点的安全状态
  double             t_m, t_mp;
  NonUniformBspline* initial_traj = &plan_data_.initial_local_segment_;
  initial_traj->getTimeSpan(t_m, t_mp);

  /* find range of collision */
  /* 查找碰撞范围 */
  double t_s = -1.0, t_e;  // 碰撞起始和结束时间
  // 以0.05秒间隔沿轨迹检查碰撞
  for (double tc = t_m; tc <= t_mp + 1e-4; tc += 0.05) {

    Eigen::Vector3d ptc = initial_traj->evaluateDeBoor(tc);
    // 检查当前点是否安全（距离是否大于安全间隙）
    safe = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < topo_prm_->clearance_ ? false : true;

    if (last_safe && !safe) {
      // 从安全进入碰撞，记录碰撞起始点
      colli_start.push_back(initial_traj->evaluateDeBoor(tc - 0.05));
      if (t_s < 0.0) t_s = tc - 0.05;
    } else if (!last_safe && safe) {
      // 从碰撞恢复到安全，记录碰撞结束点
      colli_end.push_back(ptc);
      t_e = tc;
    }

    last_safe = safe;
  }

  // 如果没有碰撞，直接返回
  if (colli_start.size() == 0) return;

  // 如果轨迹终点在障碍物内，也直接返回
  if (colli_start.size() == 1 && colli_end.size() == 0) return;

  /* find start and end safe segment */
  /* 查找起始和结束安全段 */

  // 采样碰撞前的安全段（从轨迹起点到碰撞起始点）
  double dt = initial_traj->getInterval();
  int    sn = ceil((t_s - t_m) / dt);
  dt        = (t_s - t_m) / sn;

  for (double tc = t_m; tc <= t_s + 1e-4; tc += dt) {
    start_pts.push_back(initial_traj->evaluateDeBoor(tc));
  }

  // 采样碰撞后的安全段（从碰撞结束点到轨迹终点）
  dt = initial_traj->getInterval();
  sn = ceil((t_mp - t_e) / dt);
  dt = (t_mp - t_e) / sn;
  // std::cout << "dt: " << dt << std::endl;
  // std::cout << "sn: " << sn << std::endl;
  // std::cout << "t_m: " << t_m << std::endl;
  // std::cout << "t_mp: " << t_mp << std::endl;
  // std::cout << "t_s: " << t_s << std::endl;
  // std::cout << "t_e: " << t_e << std::endl;

  if (dt > 1e-4) {
    for (double tc = t_e; tc <= t_mp + 1e-4; tc += dt) {
      end_pts.push_back(initial_traj->evaluateDeBoor(tc));
    }
  } else {
    // 如果时间间隔过小，直接添加终点
    end_pts.push_back(initial_traj->evaluateDeBoor(t_mp));
  }
}

// !SECTION

/**
 * @brief 规划偏航角轨迹
 *
 * 根据位置轨迹的运动方向规划平滑的偏航角轨迹，
 * 使无人机朝向其运动方向（前向飞行）
 *
 * @param start_yaw 起始偏航角状态 [yaw, yaw_dot, yaw_ddot]
 */
void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw) {
  ROS_INFO("plan yaw");
  auto t1 = ros::Time::now();
  // calculate waypoints of heading
  /* 计算航向角的航点 */

  auto&  pos      = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  // 设置偏航角的时间间隔为0.3秒
  double dt_yaw  = 0.3;
  int    seg_num = ceil(duration / dt_yaw);
  dt_yaw         = duration / seg_num;

  const double            forward_t = 2.0;  // 前向时间：2秒
  double                  last_yaw  = start_yaw(0);
  vector<Eigen::Vector3d> waypts;      // 偏航角航点
  vector<int>             waypt_idx;   // 航点索引

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states
  /* 生成偏航角航点（排除边界状态） */

  for (int i = 0; i < seg_num; ++i) {
    double          tc = i * dt_yaw;
    // 当前位置
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    // 前向时间点（向前看2秒）
    double          tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    // 方向向量
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
      // 根据方向向量计算偏航角
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      // 计算连续的偏航角（避免跳变）
      calcNextYaw(last_yaw, waypt(0));
    } else {
      // 如果方向向量过小，使用前一个航点的偏航角
      waypt = waypts.back();
    }
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints
  /* 计算带边界状态约束的初始控制点 */

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  // 状态到控制点的转换矩阵
  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
      dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  // 根据起始状态计算前3个控制点
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  // 根据终点速度方向计算终点偏航角
  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  // 根据终点状态计算后3个控制点
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // solve
  /* 求解优化问题 */
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  // 代价函数包含平滑性和航点约束
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  yaw           = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);

  // update traj info
  /* 更新轨迹信息 */
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  // 计算偏航角速度轨迹
  local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
  // 计算偏航角加速度轨迹
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  // 保存路径偏航角数据用于可视化
  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_    = path_yaw;
  plan_data_.dt_yaw_      = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  std::cout << "plan heading: " << (ros::Time::now() - t1).toSec() << std::endl;
}

/**
 * @brief 计算下一个偏航角
 *
 * 确保偏航角的连续性，避免在-PI和PI之间的跳变
 *
 * @param last_yaw 上一个偏航角
 * @param yaw 输入输出参数，当前偏航角，会被修正为连续的值
 */
void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]
  /* 将偏航角归一化到[-PI, PI]范围 */

  double round_last = last_yaw;

  // 将上一个偏航角归一化到[-PI, PI]
  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  // 计算偏航角差值
  double diff = yaw - round_last;

  // 根据差值选择合适的偏航角，确保连续性（不发生跳变）
  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

}  // namespace fast_planner
