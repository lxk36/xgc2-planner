/**
 * @file plan_container.hpp
 * @brief 轨迹规划容器头文件，定义了全局轨迹数据、局部轨迹数据和规划参数的数据结构
 *
 * 本文件包含了ego-planner中用于存储和管理轨迹规划相关数据的核心数据结构：
 * - GlobalTrajData: 管理全局多项式轨迹和局部B样条轨迹的融合
 * - LocalTrajData: 存储局部轨迹的详细信息
 * - PlanParameters: 规划算法的参数配置
 *
 * ============================================================================
 * 系统架构说明
 * ============================================================================
 *
 * Ego-Planner采用分层规划架构：
 *
 * 1. 全局层（Global Layer）
 *    - 使用多项式轨迹（PolynomialTraj）进行长期路径规划
 *    - 提供从起点到终点的全局导航
 *    - 在已知静态环境中生成初始轨迹
 *
 * 2. 局部层（Local Layer）
 *    - 使用均匀B样条（UniformBspline）进行局部轨迹优化
 *    - 响应动态障碍物和未知环境
 *    - 在线重规划，保持实时性能
 *
 * 3. 轨迹融合（Trajectory Fusion）
 *    - GlobalTrajData类负责无缝融合全局和局部轨迹
 *    - 根据当前时间自动选择使用全局轨迹还是局部轨迹
 *    - 维护时间一致性，处理重规划导致的时间增量
 *
 * ============================================================================
 * 关键概念
 * ============================================================================
 *
 * - 时间增量（time_increase_）：
 *   由于局部重规划可能改变轨迹时长，需要累计跟踪总的时间变化
 *
 * - 时间对齐（time alignment）：
 *   确保从局部轨迹切换回全局轨迹时，时间参数保持连续
 *
 * - B样条表示：
 *   使用B样条可以方便地进行优化和导数计算，适合实时轨迹生成
 *
 * - 边界条件：
 *   在生成局部轨迹时保持起止点的速度和加速度连续性
 */

#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

// 标准库头文件
#include <Eigen/Eigen>  // Eigen线性代数库，用于矩阵和向量运算
#include <vector>       // STL向量容器
#include <ros/ros.h>    // ROS核心库，提供时间戳等功能

// Ego-Planner相关头文件
#include <ego_bspline_opt/uniform_bspline.h>    // 均匀B样条类，用于表示和操作B样条曲线
#include <ego_traj_utils/polynomial_traj.h>     // 多项式轨迹类，用于全局路径规划

using std::vector;

namespace ego_planner
{

  /**
   * @class GlobalTrajData
   * @brief 全局轨迹数据类，管理全局多项式轨迹和局部B样条轨迹的融合
   *
   * 该类维护了一条全局多项式轨迹和一段局部优化的B样条轨迹。
   * 局部轨迹用于在线重规划，当检测到新的障碍物时，会在当前位置附近
   * 生成新的局部轨迹来避障，而不影响整体的全局规划。
   *
   * 轨迹融合策略：
   * - 在局部轨迹开始时间之前：使用全局轨迹
   * - 在局部轨迹时间段内：使用局部B样条轨迹
   * - 在局部轨迹结束之后：使用全局轨迹
   *
   * 设计理念：
   * 该类采用了"全局规划+局部优化"的分层架构，全局轨迹提供长期导航，
   * 而局部轨迹在遇到动态障碍物或需要精细避障时进行实时调整。
   * 这种设计既保证了全局最优性，又具备局部反应能力。
   */
  class GlobalTrajData
  {
  private:
  public:
    /* 轨迹数据成员 */
    PolynomialTraj global_traj_;        // 全局多项式轨迹，用于长期规划
    vector<UniformBspline> local_traj_; // 局部B样条轨迹，包含位置、速度、加速度三条曲线
                                        // local_traj_[0]: 位置轨迹
                                        // local_traj_[1]: 速度轨迹（位置的一阶导数）
                                        // local_traj_[2]: 加速度轨迹（速度的一阶导数）

    /* 时间管理成员 */
    double global_duration_;            // 全局轨迹的总时长（包括时间增量）
    ros::Time global_start_time_;       // 全局轨迹的起始时间戳
    double local_start_time_, local_end_time_; // 局部轨迹在全局时间轴上的起止时间
    double time_increase_;              // 累计的时间增量（由于局部重规划导致的时间延长）
    double last_time_inc_;              // 上一次局部重规划产生的时间增量
    double last_progress_time_;         // 上一次进度更新的时间

    /**
     * @brief 默认构造函数
     */
    GlobalTrajData(/* args */) {}

    /**
     * @brief 析构函数
     */
    ~GlobalTrajData() {}

    /**
     * @brief 判断局部轨迹是否已经到达目标点
     * @return true 如果局部轨迹的结束时间与全局轨迹总时长的差值小于0.1秒
     * @return false 否则返回false
     *
     * 该函数通过比较局部轨迹结束时间和全局轨迹总时长来判断是否接近终点
     */
    bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

    /**
     * @brief 设置全局多项式轨迹
     * @param traj 全局多项式轨迹对象
     * @param time 轨迹的起始时间戳
     *
     * 该函数用于初始化或更新全局轨迹，同时会重置所有局部轨迹相关的状态。
     * 这通常在收到新的全局路径规划结果时调用。
     */
    void setGlobalTraj(const PolynomialTraj &traj, const ros::Time &time)
    {
      global_traj_ = traj;                         // 保存全局轨迹
      global_traj_.init();                          // 初始化全局轨迹
      global_duration_ = global_traj_.getTimeSum(); // 获取轨迹总时长
      global_start_time_ = time;                    // 记录起始时间

      // 清除局部轨迹相关数据
      local_traj_.clear();
      local_start_time_ = -1;
      local_end_time_ = -1;
      time_increase_ = 0.0;     // 重置累计时间增量
      last_time_inc_ = 0.0;     // 重置上次时间增量
      last_progress_time_ = 0.0; // 重置进度时间
    }

    /**
     * @brief 设置局部B样条轨迹
     * @param traj 局部B样条轨迹（位置）
     * @param local_ts 局部轨迹的起始时间（在全局时间轴上）
     * @param local_te 局部轨迹的结束时间（在全局时间轴上）
     * @param time_inc 由于局部重规划产生的时间增量
     *
     * 该函数设置新的局部轨迹，并自动计算其一阶导数（速度）和二阶导数（加速度）。
     * 局部轨迹用于在线避障和轨迹优化，可能会延长或缩短总飞行时间。
     *
     * 工作流程：
     * 1. 存储位置B样条轨迹
     * 2. 通过B样条求导自动生成速度和加速度轨迹
     * 3. 更新局部轨迹的时间窗口
     * 4. 更新全局时长和时间增量
     *
     * 注意：
     * - time_inc可以为正（轨迹延长）或负（轨迹缩短）
     * - B样条的导数仍然是B样条，可以高效计算
     */
    void setLocalTraj(UniformBspline traj, double local_ts, double local_te, double time_inc)
    {
      local_traj_.resize(3);
      local_traj_[0] = traj;                            // 位置轨迹（输入的B样条）
      local_traj_[1] = local_traj_[0].getDerivative(); // 速度轨迹（位置的一阶导数）
      local_traj_[2] = local_traj_[1].getDerivative(); // 加速度轨迹（速度的一阶导数，即位置的二阶导数）

      local_start_time_ = local_ts;      // 设置局部轨迹起始时间
      local_end_time_ = local_te;        // 设置局部轨迹结束时间
      global_duration_ += time_inc;      // 更新全局总时长（原时长 + 时间增量）
      time_increase_ += time_inc;        // 累加时间增量（用于时间对齐）
      last_time_inc_ = time_inc;         // 记录本次时间增量（用于处理轨迹切换）
    }

    /**
     * @brief 获取指定时刻的位置
     * @param t 查询时间（在全局时间轴上）
     * @return Eigen::Vector3d 该时刻的三维位置坐标
     *
     * 根据查询时间所在的区间，选择使用全局轨迹或局部轨迹：
     * 1. 如果时间在局部轨迹开始之前：使用全局轨迹（需要时间偏移补偿）
     * 2. 如果时间在局部轨迹结束之后：使用全局轨迹（减去累计时间增量）
     * 3. 如果时间在局部轨迹区间内：使用局部B样条轨迹
     *
     * 时间偏移说明：
     * - time_increase_: 累计的时间增量，每次局部重规划可能延长或缩短飞行时间
     * - last_time_inc_: 上一次重规划的时间增量，用于处理新旧局部轨迹切换时的时间对齐
     */
    Eigen::Vector3d getPosition(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        // 局部轨迹开始前，使用全局轨迹
        // 需要补偿上一次的时间增量，因为当前的local_traj还没生效
        // 时间映射: t_global = t - time_increase_ + last_time_inc_
        return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        // 局部轨迹结束后，使用全局轨迹
        // 需要减去累计的时间增量以对齐到原始全局轨迹的时间轴
        // 时间映射: t_global = t - time_increase_
        return global_traj_.evaluate(t - time_increase_);
      }
      else
      {
        // 在局部轨迹时间段内，使用局部B样条轨迹
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp); // 获取B样条的时间跨度[tm, tmp]
        // 将全局时间t映射到局部B样条的时间参数空间
        // t_local = tm + (t - local_start_time_)
        return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    /**
     * @brief 获取指定时刻的速度
     * @param t 查询时间（在全局时间轴上）
     * @return Eigen::Vector3d 该时刻的三维速度向量
     *
     * 速度获取策略与位置类似，根据时间区间选择不同的轨迹源：
     * 1. 局部轨迹开始前：使用全局轨迹的速度
     * 2. 局部轨迹结束后：使用全局轨迹的速度（需时间偏移）
     * 3. 局部轨迹区间内：使用局部B样条轨迹的一阶导数
     *
     * 注意：速度轨迹local_traj_[1]是通过位置轨迹的getDerivative()自动计算得到的
     */
    Eigen::Vector3d getVelocity(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        // 局部轨迹开始前，使用全局轨迹的速度
        // 注意：这里直接使用t而不做时间偏移，因为速度是对时间的导数
        return global_traj_.evaluateVel(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        // 局部轨迹结束后，使用全局轨迹的速度
        // 需要时间偏移以对齐到原始全局轨迹时间轴
        return global_traj_.evaluateVel(t - time_increase_);
      }
      else
      {
        // 在局部轨迹时间段内，使用局部B样条的一阶导数（速度）
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        // local_traj_[1]已经是速度曲线，直接求值即可
        return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    /**
     * @brief 获取指定时刻的加速度
     * @param t 查询时间（在全局时间轴上）
     * @return Eigen::Vector3d 该时刻的三维加速度向量
     *
     * 加速度获取策略与位置、速度类似：
     * 1. 局部轨迹开始前：使用全局轨迹的加速度
     * 2. 局部轨迹结束后：使用全局轨迹的加速度（需时间偏移）
     * 3. 局部轨迹区间内：使用局部B样条轨迹的二阶导数
     *
     * 注意：加速度轨迹local_traj_[2]是通过速度轨迹的getDerivative()自动计算得到的
     * 即 local_traj_[2] = d/dt(local_traj_[1]) = d²/dt²(local_traj_[0])
     */
    Eigen::Vector3d getAcceleration(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        // 局部轨迹开始前，使用全局轨迹的加速度
        return global_traj_.evaluateAcc(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        // 局部轨迹结束后，使用全局轨迹的加速度
        // 需要时间偏移以对齐到原始全局轨迹时间轴
        return global_traj_.evaluateAcc(t - time_increase_);
      }
      else
      {
        // 在局部轨迹时间段内，使用局部B样条的二阶导数（加速度）
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        // local_traj_[2]已经是加速度曲线，直接求值即可
        return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    /**
     * @brief 根据指定半径范围获取B样条参数化数据
     * @param start_t 轨迹段的起始时间
     * @param des_radius 期望的球形范围半径（从起始点开始）
     * @param dist_pt 离散点之间的期望距离
     * @param point_set [输出] 离散化的轨迹点集合
     * @param start_end_derivative [输出] 起止点的速度和加速度 [v_start, v_end, a_start, a_end]
     * @param dt [输出] 相邻两个离散点之间的时间间隔
     * @param seg_duration [输出] 截断的轨迹段时长
     *
     * 该函数从指定起始时间开始，沿着轨迹前进，直到距离起始点的距离超过指定半径
     * 或到达全局轨迹终点。然后根据期望的点间距离对该段轨迹进行离散化采样。
     *
     * 应用场景：在局部重规划时，需要在当前位置周围的一定范围内提取轨迹段进行优化。
     *
     * 算法流程：
     * 1. 从起始点出发，以固定时间步长0.2s沿轨迹前进
     * 2. 累加轨迹弧长，计算当前点到起始点的直线距离
     * 3. 当距离超过期望半径或到达终点时停止
     * 4. 根据累计的弧长和期望点间距计算需要采样的点数
     * 5. 对确定的轨迹段进行均匀时间采样
     * 6. 提取起止点的导数信息作为边界条件
     */
    void getTrajByRadius(const double &start_t, const double &des_radius, const double &dist_pt,
                         vector<Eigen::Vector3d> &point_set, vector<Eigen::Vector3d> &start_end_derivative,
                         double &dt, double &seg_duration)
    {
      double seg_length = 0.0; // 截断轨迹段的弧长（累加相邻点之间的距离）
      double seg_time = 0.0;   // 截断轨迹段的时间长度
      double radius = 0.0;     // 当前点到轨迹段起始点的直线距离（欧氏距离）

      double delt = 0.2;  // 前向搜索时的时间步长（秒）
      Eigen::Vector3d first_pt = getPosition(start_t); // 轨迹段的第一个点（起始点）
      Eigen::Vector3d prev_pt = first_pt;              // 前一个点（用于计算弧长增量）
      Eigen::Vector3d cur_pt;                          // 当前点

      /* 第一步：确定轨迹段的范围 */
      // 沿轨迹前进，直到超出指定半径或到达全局轨迹终点
      while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3)
      {
        seg_time += delt;
        seg_time = min(seg_time, global_duration_ - start_t); // 确保不超过全局轨迹终点

        cur_pt = getPosition(start_t + seg_time);
        seg_length += (cur_pt - prev_pt).norm(); // 累加轨迹弧长（曲线长度）
        prev_pt = cur_pt;
        radius = (cur_pt - first_pt).norm();     // 更新当前点到起始点的距离（直线距离）
      }

      /* 第二步：计算离散化参数 */
      // 根据期望的点间距离和实际弧长计算离散化段数
      int seg_num = floor(seg_length / dist_pt);

      // 设置输出参数
      seg_duration = seg_time; // 轨迹段的总时长
      dt = seg_time / seg_num; // 相邻离散点之间的时间间隔（均匀采样）

      /* 第三步：对轨迹段进行均匀时间采样 */
      // 按照计算出的时间间隔对轨迹段进行采样
      for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      /* 第四步：提取边界条件 */
      // 保存起止点的速度和加速度信息，用于B样条拟合时的边界条件
      // 这些导数信息确保新生成的B样条与原轨迹在起止点处平滑连接
      start_end_derivative.push_back(getVelocity(start_t));           // 起始速度
      start_end_derivative.push_back(getVelocity(start_t + seg_time)); // 终止速度
      start_end_derivative.push_back(getAcceleration(start_t));        // 起始加速度
      start_end_derivative.push_back(getAcceleration(start_t + seg_time)); // 终止加速度
    }

    /**
     * @brief 根据固定时长获取B样条参数化数据
     * @param start_t 轨迹段的起始时间
     * @param duration 轨迹段的时间长度
     * @param seg_num 将轨迹段离散化的段数
     * @param point_set [输出] 离散化的轨迹点集合
     * @param start_end_derivative [输出] 起止点的速度和加速度 [v_start, v_end, a_start, a_end]
     * @param dt [输出] 相邻两个离散点之间的时间间隔
     *
     * 该函数从指定起始时间开始，提取固定时长的轨迹段，并按照给定的段数进行
     * 均匀离散化采样。与getTrajByRadius不同，该函数直接指定轨迹段的时长，
     * 而不是根据空间半径来确定。
     *
     * 应用场景：当需要在固定时间窗口内进行轨迹优化时使用。
     *
     * 算法流程：
     * 1. 根据给定的时长和段数计算时间采样间隔
     * 2. 从起始时间开始，按固定时间间隔采样轨迹点
     * 3. 提取起止点的导数信息作为边界条件
     *
     * 与getTrajByRadius的区别：
     * - getTrajByRadius: 基于空间半径，段数由弧长和点间距自动计算
     * - getTrajByDuration: 基于时间长度，段数由用户指定
     */
    void getTrajByDuration(double start_t, double duration, int seg_num,
                           vector<Eigen::Vector3d> &point_set,
                           vector<Eigen::Vector3d> &start_end_derivative, double &dt)
    {
      dt = duration / seg_num; // 计算均匀时间间隔
      Eigen::Vector3d cur_pt;

      /* 对固定时长的轨迹段进行均匀时间采样 */
      // 按照固定时间间隔对轨迹段进行均匀采样
      for (double tp = 0.0; tp <= duration + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      /* 提取边界条件 */
      // 保存起止点的速度和加速度信息，用于B样条拟合时的边界条件
      // 这些导数信息确保新生成的B样条与原轨迹在起止点处平滑连接
      start_end_derivative.push_back(getVelocity(start_t));            // 起始速度
      start_end_derivative.push_back(getVelocity(start_t + duration)); // 终止速度
      start_end_derivative.push_back(getAcceleration(start_t));        // 起始加速度
      start_end_derivative.push_back(getAcceleration(start_t + duration)); // 终止加速度
    }
  };

  /**
   * @struct PlanParameters
   * @brief 轨迹规划算法的参数配置结构体
   *
   * 该结构体包含了运动规划算法所需的各种参数，包括物理限制、
   * B样条参数、可行性容差以及性能统计信息。
   *
   * 参数说明：
   * - 物理限制参数用于约束生成的轨迹满足动力学可行性
   * - B样条参数影响轨迹的光滑度和控制精度
   * - 可行性容差提供一定的鲁棒性，允许轨迹略微超出物理限制
   * - 时间统计用于性能分析和调试
   */
  struct PlanParameters
  {
    /* planning algorithm parameters */
    /* 规划算法参数 */
    double max_vel_, max_acc_, max_jerk_; // 物理限制：最大速度、最大加速度、最大加加速度（jerk）
                                          // 这些限制基于无人机的动力学能力和安全要求
    double ctrl_pt_dist;                  // B样条相邻控制点之间的距离
                                          // 控制点距离越小，轨迹分辨率越高，但计算量也越大
    double feasibility_tolerance_;        // 速度/加速度超出限制的容许比例（用于判断轨迹可行性）
                                          // 例如，tolerance=0.1 表示允许超出限制10%
    double planning_horizen_;             // 规划时域/视野（规划的时间范围，单位：秒）
                                          // 决定了局部规划器能够"看到"多远的未来

    /* processing time */
    /* 处理时间统计（单位：秒） */
    double time_search_ = 0.0;    // 路径搜索耗时（如A*或动态A*搜索）
    double time_optimize_ = 0.0;  // 轨迹优化耗时（B样条优化）
    double time_adjust_ = 0.0;    // 轨迹调整耗时（如时间分配调整）
  };

  /**
   * @struct LocalTrajData
   * @brief 局部轨迹数据结构体
   *
   * 该结构体存储了生成的局部轨迹的详细信息，包括轨迹ID、时长、
   * 起始状态以及位置、速度、加速度的B样条表示。
   *
   * 应用场景：
   * 在ego-planner中，局部轨迹用于在线重规划。当检测到新障碍物或需要
   * 避障时，规划器会生成新的局部轨迹来替代部分全局轨迹。
   *
   * 时间对齐问题：
   * 由于局部重规划可能改变轨迹的时长，需要使用global_time_offset来
   * 保持局部轨迹与全局时间的对齐，确保在切换回全局轨迹时时间连续。
   */
  struct LocalTrajData
  {
    /* info of generated traj */
    /* 生成的轨迹信息 */

    int traj_id_;              // 轨迹的唯一标识ID（用于区分不同的重规划迭代）
    double duration_;          // 轨迹的总时长（单位：秒）
    double global_time_offset; // 全局时间偏移量
                               // 这是因为当局部轨迹结束并准备切换回全局轨迹时，
                               // 全局轨迹的时间不再与世界时间匹配，需要进行时间对齐
                               // offset = 当前世界时间 - 全局轨迹对应时间
    ros::Time start_time_;     // 轨迹的起始时间戳（ROS时间，用于时间同步）
    Eigen::Vector3d start_pos_; // 轨迹的起始位置（三维坐标，单位：米）

    /* B样条轨迹表示 */
    UniformBspline position_traj_;      // 位置B样条曲线（三维位置随时间变化）
    UniformBspline velocity_traj_;      // 速度B样条曲线（position_traj_的一阶导数）
    UniformBspline acceleration_traj_;  // 加速度B样条曲线（velocity_traj_的一阶导数）
                                        // 三条曲线共同完整描述了运动状态
  };

} // namespace ego_planner

#endif