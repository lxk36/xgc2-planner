
/**
 * @file ego_replan_fsm.cpp
 * @brief EGO-Planner重规划有限状态机实现文件
 *
 * 本文件实现了EGO-Planner的核心重规划逻辑，使用有限状态机(FSM)管理规划器的执行状态。
 * 主要功能包括：全局轨迹规划、局部轨迹重规划、碰撞检测、紧急停止等。
 */

#include <ego_plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  /**
   * @brief 初始化EGO重规划有限状态机
   * @param nh ROS节点句柄，用于参数读取和话题订阅
   *
   * 功能说明：
   * 1. 初始化状态机变量（当前状态、目标状态、里程计接收状态等）
   * 2. 从ROS参数服务器读取FSM相关参数（重规划阈值、规划视距、航点信息等）
   * 3. 初始化可视化模块和规划器管理模块
   * 4. 设置定时器回调（状态机执行回调、安全检查回调）
   * 5. 订阅里程计话题，发布B样条轨迹和数据显示话题
   * 6. 根据目标类型选择手动设置或预设航点模式
   */
  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    // 初始化状态机变量
    current_wp_ = 0;                           // 当前航点索引
    exec_state_ = FSM_EXEC_STATE::INIT;        // 初始执行状态为INIT
    have_target_ = false;                       // 是否有目标点标志
    have_odom_ = false;                         // 是否接收到里程计数据标志

    /*  读取FSM参数  */
    nh.param("fsm/flight_type", target_type_, -1);                      // 飞行类型（手动/预设）
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);                // 重规划阈值（距离起点的最小距离）
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);          // 不重规划阈值（距离终点的最小距离）
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);          // 规划视距（局部目标点的最大距离）
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0); // 规划时间视距
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);              // 紧急停止时间阈值（默认1.0秒）

    // 读取预设航点
    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* 初始化主要模块 */
    visualization_.reset(new PlanningVisualization(nh));  // 可视化模块
    planner_manager_.reset(new EGOPlannerManager);        // 规划器管理模块
    planner_manager_->initPlanModules(nh, visualization_);

    /* 设置回调函数 */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);          // 100Hz状态机执行回调
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this); // 20Hz安全检查回调

    // 订阅里程计话题
    odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    // 发布B样条轨迹和数据显示话题
    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);

    // 根据目标类型进行不同的初始化
    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
      // 手动目标模式：订阅航点生成器话题
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      // 预设目标模式：等待里程计数据后直接规划全局轨迹
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      planGlobalTrajbyGivenWps();  // 根据给定航点规划全局轨迹
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  /**
   * @brief 根据给定的航点规划全局轨迹
   *
   * 功能说明：
   * 1. 将配置文件中的航点转换为Eigen::Vector3d格式
   * 2. 调用规划器管理器生成穿过所有航点的全局轨迹
   * 3. 在RViz中可视化所有航点和生成的全局轨迹
   * 4. 规划成功后，将FSM状态切换到GEN_NEW_TRAJ，准备生成局部轨迹
   */
  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    // 将配置的航点转换为Eigen格式
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];

      end_pt_ = wps.back();  // 记录最后一个航点作为终点
    }

    // 调用规划器生成穿过所有航点的全局轨迹
    // 参数：起点位置、起点速度、起点加速度、航点列表、终点速度、终点加速度
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 在RViz中显示所有航点
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {

      /*** 可视化全局轨迹 ***/
      constexpr double step_size_t = 0.1;  // 采样时间步长
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      // 在全局轨迹上均匀采样点用于可视化
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      // 设置终点速度为零（悬停）
      end_vel_.setZero();
      have_target_ = true;        // 标记已有目标点
      have_new_target_ = true;    // 标记有新目标点

      /*** 状态机切换 ***/
      // 切换到GEN_NEW_TRAJ状态，开始生成局部轨迹
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");

      // 在RViz中显示全局路径
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  /**
   * @brief 航点回调函数，接收手动设置的目标点
   * @param msg 包含目标点信息的路径消息
   *
   * 功能说明：
   * 1. 接收来自航点生成器的目标点消息
   * 2. 检查目标点有效性（z坐标不能为负）
   * 3. 从当前位置到目标点规划全局轨迹
   * 4. 根据当前状态切换到相应的FSM状态（生成新轨迹或重规划）
   */
  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    // 检查目标点有效性，z坐标小于-0.1表示无效目标
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;          // 设置触发标志
    init_pt_ = odom_pos_;     // 记录初始位置

    bool success = false;
    // 设置目标点，固定z高度为1.0米
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    // 规划从当前位置到目标点的全局轨迹
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 在RViz中显示目标点
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** 可视化全局轨迹 ***/
      constexpr double step_size_t = 0.1;  // 采样时间步长
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      // 设置终点速度为零
      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** 状态机切换 ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");    // 等待目标状态 -> 生成新轨迹
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");     // 执行轨迹状态 -> 重规划轨迹

      // 显示全局路径
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  /**
   * @brief 里程计回调函数，接收机器人当前状态信息
   * @param msg 包含位置、速度和姿态信息的里程计消息
   *
   * 功能说明：
   * 从里程计消息中提取机器人当前的位置、速度和姿态信息，用于轨迹规划
   */
  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    // 提取位置信息
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    // 提取速度信息
    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // 加速度估计（当前被注释）
    //odom_acc_ = estimateAcc( msg );

    // 提取姿态信息（四元数）
    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;  // 标记已接收里程计数据
  }

  /**
   * @brief 改变FSM执行状态
   * @param new_state 新的FSM状态
   * @param pos_call 调用位置标识符（用于调试日志）
   *
   * 功能说明：
   * 1. 跟踪连续调用同一状态的次数
   * 2. 记录状态转换信息并输出日志
   * 3. 支持调试和问题定位
   */
  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    // 如果新状态与当前状态相同，增加连续调用计数
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;  // 否则重置为1

    // FSM状态名称数组
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    // 输出状态转换日志
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  /**
   * @brief 获取连续调用当前状态的次数
   * @return pair<调用次数, 当前状态>
   */
  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  /**
   * @brief 打印当前FSM执行状态
   */
  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  /**
   * @brief FSM执行回调函数，100Hz频率运行
   * @param e 定时器事件
   *
   * 功能说明：
   * 这是整个规划器的核心状态机，根据不同状态执行相应的逻辑：
   * - INIT: 初始化状态，等待里程计和触发信号
   * - WAIT_TARGET: 等待目标点
   * - GEN_NEW_TRAJ: 生成新轨迹
   * - REPLAN_TRAJ: 重规划轨迹
   * - EXEC_TRAJ: 执行轨迹，并判断是否需要重规划
   * - EMERGENCY_STOP: 紧急停止
   */
  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {

    // 定期打印状态机状态（每100次循环，即每1秒打印一次）
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    // 状态机主逻辑
    switch (exec_state_)
    {
    case INIT:
    {
      // 初始化状态：等待里程计数据和触发信号
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      // 等待目标点状态
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      // 生成新轨迹状态
      start_pt_ = odom_pos_;      // 起点为当前位置
      start_vel_ = odom_vel_;     // 起点速度为当前速度
      start_acc_.setZero();       // 起点加速度设为零

      // 偏航角计算（当前被注释）
      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      // 判断是否使用随机多项式初始化
      // 第一次调用时不使用，后续调用使用（用于重试失败的规划）
      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      // 调用Rebound重规划算法
      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;  // 允许从紧急停止状态恢复
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");  // 失败则重试
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      // 重规划轨迹状态：从当前轨迹重新规划
      if (planFromCurrentTraj())
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");  // 失败则重试
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* 判断是否需要重规划 */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      // 计算当前应该在轨迹上的位置
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      // 判断是否需要重规划的三个条件：
      if (t_cur > info->duration_ - 1e-2)
      {
        // 条件1：轨迹执行完毕
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // 条件2：接近终点，不需要重规划
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // 条件3：距离轨迹起点太近，不需要重规划
        return;
      }
      else
      {
        // 否则进行重规划
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      // 紧急停止状态
      if (flag_escape_emergency_) // 避免重复调用
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        // 当速度降低到0.1以下时，尝试重新生成轨迹
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    // 发布数据显示消息
    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  /**
   * @brief 从当前执行的轨迹进行重规划
   * @return 重规划是否成功
   *
   * 功能说明：
   * 1. 根据当前时刻在轨迹上的状态（位置、速度、加速度）作为新规划的起点
   * 2. 采用渐进式尝试策略：
   *    - 首先尝试不使用多项式初始化的重规划
   *    - 失败则尝试使用多项式初始化的重规划
   *    - 再失败则尝试使用随机多项式初始化的重规划
   * 3. 这种策略可以提高重规划的成功率
   */
  bool EGOReplanFSM::planFromCurrentTraj()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    // 从当前轨迹中提取当前时刻的状态作为新规划的起点
    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);      // 位置
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);     // 速度
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur); // 加速度

    // 尝试策略1：不使用多项式初始化进行重规划
    bool success = callReboundReplan(false, false);

    if (!success)
    {
      // 尝试策略2：使用多项式初始化进行重规划
      success = callReboundReplan(true, false);
      if (!success)
      {
        // 尝试策略3：使用随机多项式初始化进行重规划（最后的尝试）
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false;  // 所有策略都失败
        }
      }
    }

    return true;
  }

  /**
   * @brief 碰撞检查回调函数，20Hz频率运行
   * @param e 定时器事件
   *
   * 功能说明：
   * 1. 定期检查当前执行轨迹是否会与障碍物碰撞
   * 2. 检查范围：从当前时刻到轨迹结束（或2/3轨迹长度）
   * 3. 碰撞处理策略：
   *    - 首先尝试从当前状态重规划
   *    - 如果重规划失败且碰撞时间小于紧急时间阈值，执行紧急停止
   *    - 否则切换到重规划状态
   */
  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    // 如果处于等待目标状态或轨迹未初始化，则不进行碰撞检查
    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- 检查轨迹碰撞 ---------- */
    constexpr double time_step = 0.01;  // 检查时间步长10ms
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;  // 轨迹2/3处的时间点

    // 从当前时刻开始沿轨迹检查碰撞
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      // 如果当前时刻小于2/3处，则只检查前2/3的轨迹（为重规划留出时间）
      if (t_cur < t_2_3 && t >= t_2_3)
        break;

      // 检查该时刻轨迹点是否在膨胀障碍物内
      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
      {
        // 发现碰撞，先尝试重规划
        if (planFromCurrentTraj())
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          // 重规划失败，判断是否需要紧急停止
          if (t - t_cur < emergency_time_) // 碰撞时间小于紧急时间阈值（默认1.0秒）
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            // 有足够时间进行重规划
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  /**
   * @brief 调用Rebound重规划算法
   * @param flag_use_poly_init 是否使用多项式初始化
   * @param flag_randomPolyTraj 是否使用随机多项式轨迹
   * @return 规划是否成功
   *
   * 功能说明：
   * 1. 计算局部目标点（基于全局轨迹和规划视距）
   * 2. 调用规划器管理器的reboundReplan方法进行B样条轨迹优化
   * 3. 将优化后的B样条轨迹发布给控制器
   * 4. 在RViz中可视化轨迹控制点
   */
  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    // 根据全局轨迹和规划视距计算局部目标点
    getLocalTarget();

    // 调用Rebound重规划算法
    // 参数：起点位置、起点速度、起点加速度、局部目标点、局部目标速度、是否使用多项式初始化、是否使用随机多项式
    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;  // 重置新目标标志

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* 发布B样条轨迹消息 */
      ego_planner::Bspline bspline;
      bspline.order = 3;                    // B样条阶数为3
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      // 提取位置B样条控制点
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      // 提取B样条节点向量
      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      // 发布B样条轨迹
      bspline_pub_.publish(bspline);

      // 在RViz中显示优化后的轨迹控制点
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_success;
  }

  /**
   * @brief 调用紧急停止功能
   * @param stop_pos 紧急停止位置
   * @return 总是返回true
   *
   * 功能说明：
   * 1. 在检测到即将碰撞且没有足够时间重规划时调用
   * 2. 生成一条快速减速到零速度的B样条轨迹
   * 3. 发布紧急停止轨迹给控制器执行
   */
  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    // 调用规划器管理器的紧急停止功能，生成减速轨迹
    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* 发布紧急停止轨迹 */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    // 提取位置B样条控制点
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    // 提取B样条节点向量
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    // 发布紧急停止B样条轨迹
    bspline_pub_.publish(bspline);

    return true;
  }

  /**
   * @brief 从全局轨迹中获取局部目标点
   *
   * 功能说明：
   * 1. 沿全局轨迹搜索，找到距离当前位置planning_horizen_距离的点作为局部目标
   * 2. 记录搜索过程中距离起点最近的时刻，用于下次搜索的起点（避免倒退）
   * 3. 如果到达全局轨迹终点，则将终点作为局部目标
   * 4. 根据局部目标点与终点的距离，设置局部目标速度：
   *    - 如果接近终点（距离小于减速距离），设置目标速度为零（悬停）
   *    - 否则使用全局轨迹在该点的速度
   */
  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    // 计算搜索时间步长（基于规划视距和最大速度）
    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;  // 记录最小距离及对应时刻

    // 从上次进度时刻开始沿全局轨迹搜索
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();  // 计算该点到起点的距离

      // 错误检查：上次进度时刻不应该距离起点太远
      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        return;
      }

      // 更新最小距离及对应时刻（用于记录进度）
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      // 找到规划视距范围内的点，作为局部目标
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;  // 更新进度时刻
        break;
      }
    }

    // 如果遍历完全局轨迹，使用终点作为局部目标
    if (t > planner_manager_->global_data_.global_duration_)
    {
      local_target_pt_ = end_pt_;
    }

    // 计算局部目标速度
    // 计算从最大速度减速到零所需的距离
    double stopping_dist = (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_);

    if ((end_pt_ - local_target_pt_).norm() < stopping_dist)
    {
      // 如果局部目标点接近终点（距离小于减速距离），设置目标速度为零
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      // 否则使用全局轨迹在该时刻的速度
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }
  }

} // namespace ego_planner
