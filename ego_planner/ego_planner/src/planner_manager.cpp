/**
 * @file planner_manager.cpp
 * @brief EGO-Planner规划器管理器实现文件
 *
 * 该文件实现了EGO-Planner的核心规划管理功能，包括：
 *
 * 核心功能模块：
 * 1. 初始化模块 (initPlanModules)
 *    - 从ROS参数服务器读取规划参数
 *    - 初始化栅格地图、B样条优化器、A*搜索器
 *    - 设置可视化模块
 *
 * 2. 重反弹轨迹重规划 (reboundReplan)
 *    - 这是EGO-Planner的核心算法
 *    - 包含三个主要步骤：
 *      a) 初始化：生成初始轨迹（多项式轨迹或基于前一轨迹）
 *      b) 优化：使用B样条优化器进行轨迹优化（考虑避障、平滑性、动力学约束）
 *      c) 精化：如需要则重新分配时间以满足速度/加速度限制
 *
 * 3. 全局轨迹规划 (planGlobalTraj, planGlobalTrajWaypoints)
 *    - 生成经过一个或多个航点的全局参考轨迹
 *    - 使用多项式轨迹（minimum snap）
 *    - 自动处理航点间距过大的情况（插入中间点）
 *
 * 4. 紧急停止 (EmergencyStop)
 *    - 生成原地静止轨迹
 *    - 用于紧急情况处理
 *
 * 5. 辅助功能
 *    - refineTrajAlgo: 轨迹精化算法
 *    - reparamBspline: B样条重新参数化
 *    - updateTrajInfo: 更新轨迹信息
 *
 * 关键数据结构：
 * - local_data_: 存储当前局部轨迹信息
 * - global_data_: 存储全局参考轨迹
 * - grid_map_: 栅格地图（用于障碍物检测）
 * - bspline_optimizer_rebound_: B样条优化器
 *
 * 算法特点：
 * - 使用均匀B样条表示轨迹，便于优化和求导
 * - 采用伪弧长法进行轨迹重采样，保证点分布均匀
 * - 支持动态重规划，可基于前一轨迹生成新轨迹
 * - 自适应时间分配，确保满足动力学约束
 */

// #include <fstream>
#include <ego_plan_manage/planner_manager.h>
#include <thread>

namespace ego_planner
{

  // SECTION interfaces for setup and query
  // 接口部分：初始化和查询

  /**
   * @brief EGO规划器管理器默认构造函数
   */
  EGOPlannerManager::EGOPlannerManager() {}

  /**
   * @brief EGO规划器管理器析构函数
   */
  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  /**
   * @brief 初始化规划模块
   * @param nh ROS节点句柄，用于读取参数
   * @param vis 可视化对象指针
   *
   * 该函数完成以下初始化工作：
   * 1. 从参数服务器读取规划器参数（速度/加速度/加加速度限制等）
   * 2. 初始化栅格地图
   * 3. 初始化B样条优化器和A*路径搜索器
   * 4. 设置可视化模块
   */
  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */
    /* 读取算法参数 */

    // 读取最大速度限制
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    // 读取最大加速度限制
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    // 读取最大加加速度（jerk）限制
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    // 读取可行性容忍度
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    // 读取控制点间距
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    // 读取规划时域（planning horizon）
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);

    // 初始化轨迹ID为0
    local_data_.traj_id_ = 0;
    // 创建栅格地图对象
    grid_map_.reset(new GridMap);
    // 初始化栅格地图
    grid_map_->initMap(nh);

    // 创建B样条优化器（rebound版本）
    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    // 设置优化器参数
    bspline_optimizer_rebound_->setParam(nh);
    // 设置优化器的环境地图
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    // 创建A*搜索器
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    // 初始化A*搜索器的栅格地图（搜索空间大小为100x100x100）
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    // 设置可视化模块
    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning
  // 重反弹重规划部分

  /**
   * @brief 重反弹（Rebound）轨迹重规划函数
   * @param start_pt 起始位置
   * @param start_vel 起始速度
   * @param start_acc 起始加速度
   * @param local_target_pt 局部目标位置
   * @param local_target_vel 局部目标速度
   * @param flag_polyInit 是否使用多项式初始化标志
   * @param flag_randomPolyTraj 是否使用随机多项式轨迹标志
   * @return 规划成功返回true，失败返回false
   *
   * Rebound重规划算法流程：
   * STEP 1: 初始化 - 生成初始轨迹（多项式或基于前一轨迹）
   * STEP 2: 优化 - 使用B样条优化器进行轨迹优化
   * STEP 3: 精化 - 如有必要重新分配时间以满足动力学约束
   */
  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    // 静态计数器，用于记录重规划次数
    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    // 检查是否已经接近目标点（距离小于0.2米）
    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    // 记录开始时间，用于性能统计
    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    /*** 步骤1: 初始化 ***/

    // 计算时间步长ts
    // 如果距离目标点较远(>0.1m)，使用较小的时间步长；否则使用较大的时间步长
    // 乘以1.2或5是为了防止时间过于紧张导致超出速度/加速度限制
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits

    // 存储路径点集合和起止点的导数信息
    vector<Eigen::Vector3d> point_set, start_end_derivatives;

    // 静态标志：首次调用和强制使用多项式标志
    static bool flag_first_call = true, flag_force_polynomial = false;
    // 是否需要重新生成初始路径的标志
    bool flag_regenerate = false;

    // 初始化路径生成循环，如果路径不合理会重新生成
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      // 分支1：使用多项式轨迹生成初始路径
      // 当首次调用、强制多项式初始化或强制使用多项式时执行
      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        // 重置标志位
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        // 计算起点到目标点的距离
        double dist = (start_pt - local_target_pt).norm();
        // 计算运动时间：考虑加速、匀速、减速过程
        // 如果 v_max^2/a_max > dist，说明无法达到最大速度，时间为 sqrt(dist/a_max)
        // 否则时间为加速时间 + 匀速时间 + 减速时间
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        // 根据随机轨迹标志选择不同的轨迹生成方式
        if (!flag_randomPolyTraj)
        {
          // 生成直接的单段最小snap轨迹（从起点到终点）
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          // 生成带随机中间点的轨迹，用于避障或增加轨迹多样性

          // 计算水平方向（垂直于起点-终点连线和Z轴的方向）
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          // 计算竖直方向（垂直于起点-终点连线和水平方向）
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();

          // 在起点和终点中间插入一个随机点
          // 随机偏移量随着连续失败次数增加而减小（通过-0.978/(count+0.989)+0.989函数）
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

          // 构建位置矩阵：起点 -> 随机中间点 -> 终点
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;

          // 时间分配：两段轨迹各占一半时间
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;

          // 生成最小snap轨迹
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;

        // 调整时间步长，确保采样点间距合理
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;  // 逐步减小时间步长
          point_set.clear();
          flag_too_far = false;

          // 从轨迹起点开始采样
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            // 检查相邻采样点距离是否过大
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
                                                         // 确保初始路径有足够的点（至少7个点）

        t -= ts;  // 回退到最后一个有效采样时间点

        // 保存起始和终止点的速度、加速度信息
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));       // 起始速度
        start_end_derivatives.push_back(local_target_vel);             // 目标速度
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));       // 起始加速度
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));       // 终止加速度
      }
      else // Initial path generated from previous trajectory.
           // 分支2：基于前一轨迹生成初始路径（正常重规划情况）
      {

        double t;
        // 获取当前时间相对于轨迹起始时间的偏移
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        // 伪弧长法重采样轨迹
        // 存储累积弧长和轨迹段点
        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;

        // 初始弧长为0
        pseudo_arc_length.push_back(0.0);

        // 从当前时间点开始，采样前一轨迹的剩余部分
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            // 累积弧长：当前段长度 + 之前的累积弧长
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        // 计算从前一轨迹末端到新目标点所需的时间（预留2倍裕量）
        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;

        // 如果所需时间大于一个时间步长，生成连接轨迹
        if (poly_time > ts)
        {
          // 从前一轨迹的末端状态连接到新目标点
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          // 对连接轨迹进行采样
          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        // 使用伪弧长法进行均匀重采样
        double sample_length = 0;  // 当前采样弧长
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
                                                   // 控制点间距（将在循环中除以1.5）
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;  // 逐步减小控制点间距
          point_set.clear();
          sample_length = 0;
          id = 0;

          // 按照固定弧长间距进行采样
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            // 找到sample_length所在的弧长区间[id, id+1]
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              // 线性插值计算采样点位置
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;  // 前进到下一个采样位置
            }
            else
              id++;  // 移动到下一个区间
          }
          point_set.push_back(local_target_pt);  // 确保终点被包含
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help
                                        // 确保至少有7个点（如果起点和终点很近，这会有帮助）

        // 保存起始和终止点的导数信息
        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));  // 当前速度
        start_end_derivatives.push_back(local_target_vel);                                   // 目标速度
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));  // 当前加速度
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());                            // 目标加速度

        // 检查初始路径是否异常过长
        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          // 如果过长，强制使用多项式重新生成
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);  // 如果需要重新生成，继续循环

    // 将采样点集转换为B样条控制点
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    // 初始化控制点并进行A*路径搜索（用于生成安全走廊）
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    // 记录初始化阶段耗时
    t_init = ros::Time::now() - t_start;

    // 可视化初始路径和A*路径
    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_pathes, vis_id);

    // 重置计时器，准备优化阶段
    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    /*** 步骤2: 优化 ***/

    // 执行B样条轨迹优化（Rebound方法）
    // 该方法会优化控制点位置以满足：避障、平滑性、动力学约束等
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;

    // 如果第一步优化失败，返回失败
    if (!flag_step_1_success)
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continous_failures_count_++;
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    // 记录优化阶段耗时
    t_opt = ros::Time::now() - t_start;
    t_start = ros::Time::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    /*** 步骤3: 精化（如有必要重新分配时间） ***/

    // 使用优化后的控制点创建B样条轨迹
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    // 设置物理限制（最大速度、最大加速度、容忍度）
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;  // 时间缩放比例
    bool flag_step_2_success = true;

    // 检查轨迹是否满足动力学可行性
    if (!pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      // 如果不满足，需要重新分配时间并重新优化
      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    // 如果精化步骤失败（轨迹碰撞障碍物）
    if (!flag_step_2_success)
    {
      printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
      continous_failures_count_++;
      return false;
    }

    // 记录精化阶段耗时
    t_refine = ros::Time::now() - t_start;

    // save planned results
    // 保存规划结果
    updateTrajInfo(pos, ros::Time::now());

    // 输出各阶段耗时统计
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

    // success. YoY
    // 成功，重置连续失败计数器
    continous_failures_count_ = 0;
    return true;
  }

  /**
   * @brief 紧急停止函数
   * @param stop_pos 停止位置
   * @return 总是返回true
   *
   * 该函数生成一个静止轨迹，所有控制点都设置为相同的停止位置
   * 用于紧急情况下快速让无人机停在指定位置
   */
  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    // 创建6个控制点的矩阵（3阶B样条至少需要4个控制点）
    Eigen::MatrixXd control_points(3, 6);

    // 将所有控制点设置为相同的停止位置
    // 这样生成的B样条轨迹将是一个静止点
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    // 更新轨迹信息（时间步长设为1.0秒）
    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  /**
   * @brief 规划经过多个航点的全局轨迹
   * @param start_pos 起始位置
   * @param start_vel 起始速度
   * @param start_acc 起始加速度
   * @param waypoints 航点列表
   * @param end_vel 终止速度
   * @param end_acc 终止加速度
   * @return 规划成功返回true，失败返回false
   *
   * 该函数生成经过一系列航点的全局参考轨迹（多项式轨迹）
   * 如果航点之间距离过远，会自动插入中间点以提高轨迹质量
   */
  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory
    // 生成全局参考轨迹

    // 构建包含起点和所有航点的点集
    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    // 计算总路径长度
    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    // 如果航点之间距离过远，插入中间点
    vector<Eigen::Vector3d> inter_points;
    // 距离阈值：总长度的1/8或4米，取较大值
    double dist_thresh = max(total_len / 8, 4.0);

    // 遍历相邻点对，检查是否需要插入中间点
    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      // 如果距离超过阈值，插入中间点
      if (dist > dist_thresh)
      {
        // 计算需要插入的点数
        int id_num = floor(dist / dist_thresh) + 1;

        // 线性插值生成中间点
        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    // 添加最后一个点
    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    // 构建位置矩阵
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    // 计算每段的时间分配（基于距离和最大速度）
    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    // 首尾段时间加倍，留出加速和减速的余地
    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    // 根据点数选择不同的轨迹生成方法
    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      // 多段轨迹：使用最小snap轨迹生成
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      // 单段轨迹：使用单段轨迹生成
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      // 点数不足，返回失败
      return false;

    // 保存全局轨迹
    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  /**
   * @brief 规划从起点到终点的全局轨迹（单目标点版本）
   * @param start_pos 起始位置
   * @param start_vel 起始速度
   * @param start_acc 起始加速度
   * @param end_pos 终止位置
   * @param end_vel 终止速度
   * @param end_acc 终止加速度
   * @return 规划成功返回true，失败返回false
   *
   * 该函数生成从起点到终点的全局参考轨迹（多项式轨迹）
   * 与planGlobalTrajWaypoints类似，但只有一个目标点
   * 如果起点和终点距离过远，会自动插入中间点
   */
  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory
    // 生成全局参考轨迹

    // 构建包含起点和终点的点集
    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    // 如果距离过远，插入中间点
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;  // 距离阈值4米

    // 遍历相邻点对，检查是否需要插入中间点
    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      // 如果距离超过阈值，插入中间点
      if (dist > dist_thresh)
      {
        // 计算需要插入的点数
        int id_num = floor(dist / dist_thresh) + 1;

        // 线性插值生成中间点
        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    // 添加最后一个点
    inter_points.push_back(points.back());

    // write position matrix
    // 构建位置矩阵
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    // 计算每段的时间分配（基于距离和最大速度）
    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    // 首尾段时间加倍，留出加速和减速的余地
    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    // 根据点数选择不同的轨迹生成方法
    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      // 多段轨迹：使用最小snap轨迹生成
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      // 单段轨迹：使用单段轨迹生成
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      // 点数不足，返回失败
      return false;

    // 保存全局轨迹
    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  /**
   * @brief 轨迹精化算法
   * @param traj 输入的B样条轨迹
   * @param start_end_derivative 起止点的导数约束
   * @param ratio 时间缩放比例
   * @param ts 时间步长（引用，会被更新）
   * @param optimal_control_points 输出的优化后控制点
   * @return 优化成功返回true，失败返回false
   *
   * 该函数用于精化不满足动力学约束的轨迹：
   * 1. 重新参数化B样条（拉长时间）
   * 2. 生成参考点
   * 3. 重新优化控制点
   */
  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;  // 时间增量

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    // 重新参数化B样条：根据ratio拉长时间，使轨迹更平缓以满足动力学约束
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    // 使用新的控制点和时间步长重新构建B样条
    traj = UniformBspline(ctrl_pts, 3, ts);

    // 生成参考点序列，用于优化时的参考
    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    // 执行B样条轨迹精化优化
    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  /**
   * @brief 更新轨迹信息
   * @param position_traj 位置轨迹（B样条）
   * @param time_now 当前时间
   *
   * 该函数更新本地轨迹数据，包括：
   * - 位置、速度、加速度轨迹
   * - 起始时间和位置
   * - 轨迹持续时间
   * - 轨迹ID（递增）
   */
  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    // 设置轨迹起始时间
    local_data_.start_time_ = time_now;
    // 保存位置轨迹
    local_data_.position_traj_ = position_traj;
    // 通过对位置轨迹求导得到速度轨迹
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    // 通过对速度轨迹求导得到加速度轨迹
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    // 计算轨迹起始位置
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    // 计算轨迹总持续时间
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    // 轨迹ID递增
    local_data_.traj_id_ += 1;
  }

  /**
   * @brief B样条重新参数化函数
   * @param bspline 输入的B样条轨迹
   * @param start_end_derivative 起止点的导数约束
   * @param ratio 时间缩放比例
   * @param ctrl_pts 输出的新控制点
   * @param dt 输出的新时间步长
   * @param time_inc 输出的时间增量
   *
   * 该函数通过拉长时间来重新参数化B样条轨迹，使其更平缓以满足动力学约束
   * 步骤：
   * 1. 根据ratio拉长轨迹时间
   * 2. 重新采样轨迹点
   * 3. 将采样点重新转换为B样条控制点
   */
  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    // 获取原始轨迹的总时间
    double time_origin = bspline.getTimeSum();
    // 计算段数（n+1个控制点定义n-p段，p=3）
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    // 根据ratio拉长时间
    bspline.lengthenTime(ratio);
    // 获取拉长后的总时间
    double duration = bspline.getTimeSum();
    // 计算新的时间步长
    dt = duration / double(seg_num);
    // 计算时间增量
    time_inc = duration - time_origin;

    // 对拉长后的轨迹重新采样
    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }

    // 将采样点集重新转换为B样条控制点
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
