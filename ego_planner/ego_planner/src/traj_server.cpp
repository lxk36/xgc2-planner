/**
 * @file traj_server.cpp
 * @brief EGO-Planner轨迹服务器节点
 * @details 该节点负责接收B样条轨迹并将其转换为位置控制指令，包含偏航角控制逻辑
 *          主要功能：
 *          1. 接收规划器生成的B样条轨迹
 *          2. 实时计算当前时刻的位置、速度、加速度
 *          3. 计算偏航角和偏航角速度（带速度限制）
 *          4. 发布位置控制指令给飞控系统
 */

#include "ego_bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

// 位置控制指令发布器
ros::Publisher pos_cmd_pub;

// 位置控制指令消息
quadrotor_msgs::PositionCommand cmd;
// 位置增益控制参数（PID控制器）
double pos_gain[3] = {0, 0, 0};
// 速度增益控制参数（PID控制器）
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

// 轨迹接收标志位
bool receive_traj_ = false;
// 轨迹存储：traj_[0]为位置轨迹，traj_[1]为速度轨迹，traj_[2]为加速度轨迹
vector<UniformBspline> traj_;
// 轨迹总时长
double traj_duration_;
// 轨迹开始执行时间
ros::Time start_time_;
// 轨迹ID标识
int traj_id_;

// 偏航角控制相关变量
double last_yaw_, last_yaw_dot_;     // 上一时刻的偏航角和偏航角速度
double time_forward_;                // 前向预测时间（用于计算期望偏航角）

/**
 * @brief B样条轨迹接收回调函数
 * @param msg B样条轨迹消息，包含控制点、节点向量、阶数等信息
 * @details 该函数解析接收到的B样条轨迹消息，构建位置轨迹以及通过求导获得速度和加速度轨迹
 */
void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
  // ========== 解析位置轨迹 ==========

  // 创建位置控制点矩阵（3行×控制点数量列）
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  // 解析节点向量（knot vector）
  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  // 将ROS消息中的位置控制点转换为Eigen矩阵格式
  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  // 构建均匀B样条位置轨迹对象（控制点、阶数、时间间隔0.1s）
  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // ========== 解析偏航角轨迹（已注释，当前版本未使用） ==========

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  // 记录轨迹开始时间和ID
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  // 清空并构建轨迹数组：位置、速度（一阶导数）、加速度（二阶导数）
  traj_.clear();
  traj_.push_back(pos_traj);                   // traj_[0]: 位置轨迹
  traj_.push_back(traj_[0].getDerivative());   // traj_[1]: 速度轨迹
  traj_.push_back(traj_[1].getDerivative());   // traj_[2]: 加速度轨迹

  // 获取轨迹总时长
  traj_duration_ = traj_[0].getTimeSum();

  // 标记已接收到轨迹
  receive_traj_ = true;
}

/**
 * @brief 计算偏航角和偏航角速度
 * @param t_cur 当前时间（相对于轨迹起始时间）
 * @param pos 当前位置
 * @param time_now 当前ROS时间戳
 * @param time_last 上一次调用时的ROS时间戳
 * @return 偏航角和偏航角速度的配对值
 * @details 该函数实现了带速度限制的偏航角控制算法：
 *          1. 根据前向预测位置计算期望偏航角方向
 *          2. 考虑偏航角的周期性（-π到π的跳变）
 *          3. 限制偏航角速度不超过最大值（π rad/s）
 *          4. 使用低通滤波器平滑偏航角和偏航角速度
 */
std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;  // 最大偏航角速度：π rad/s (180度/秒)
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  // 计算期望飞行方向：使用前向预测位置与当前位置的差值
  // 如果前向预测未超出轨迹，使用预测位置；否则使用轨迹终点
  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;

  // 计算期望偏航角：仅当方向向量足够大时更新，否则保持上一次的偏航角
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;

  // 计算本时间步允许的最大偏航角变化量
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();

  /*
   * 处理偏航角的周期性跳变问题（-π到π）
   * 分三种情况处理：
   * 1. yaw_temp - last_yaw_ > PI：期望偏航角跨越+π边界（实际上应该反向旋转）
   * 2. yaw_temp - last_yaw_ < -PI：期望偏航角跨越-π边界（实际上应该反向旋转）
   * 3. 其他情况：正常范围内的偏航角变化
   */

  // ========== 情况1：期望偏航角跨越+π边界 ==========
  if (yaw_temp - last_yaw_ > PI)
  {
    // 检查是否需要限制旋转速度（考虑-2π修正后的角度差）
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      // 旋转速度超限，按最大速度反向旋转
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;  // 保持在[-π, π]范围内

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      // 旋转速度未超限，直接使用期望偏航角
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  // ========== 情况2：期望偏航角跨越-π边界 ==========
  else if (yaw_temp - last_yaw_ < -PI)
  {
    // 检查是否需要限制旋转速度（考虑+2π修正后的角度差）
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      // 旋转速度超限，按最大速度正向旋转
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;  // 保持在[-π, π]范围内

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      // 旋转速度未超限，直接使用期望偏航角
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  // ========== 情况3：正常范围内的偏航角变化 ==========
  else
  {
    // 检查反向旋转是否超速
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    // 检查正向旋转是否超速
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    // 旋转速度在限制范围内
    else
    {
      yaw = yaw_temp;
      // 处理边界情况的偏航角速度
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  // 应用低通滤波器（Low Pass Filter）平滑偏航角和偏航角速度
  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // 简单的一阶低通滤波器
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;

  // 更新历史值
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  // 返回计算结果
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

/**
 * @brief 定时器回调函数，定期发布位置控制指令
 * @param e 定时器事件（未使用）
 * @details 该函数以固定频率（100Hz）执行以下操作：
 *          1. 检查是否已接收到轨迹
 *          2. 根据当前时间计算轨迹上的位置、速度、加速度
 *          3. 计算偏航角和偏航角速度
 *          4. 构建并发布位置控制指令消息
 */
void cmdCallback(const ros::TimerEvent &e)
{
  /* 在接收到轨迹之前不发布任何指令 */
  if (!receive_traj_)
    return;

  // 获取当前时间和相对轨迹起始时间的时间差
  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  // 初始化位置、速度、加速度和前向位置
  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();

  // ========== 情况1：轨迹执行中 ==========
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    // 使用DeBoor算法评估B样条曲线在当前时刻的值
    pos = traj_[0].evaluateDeBoorT(t_cur);  // 位置
    vel = traj_[1].evaluateDeBoorT(t_cur);  // 速度
    acc = traj_[2].evaluateDeBoorT(t_cur);  // 加速度

    /*** 计算偏航角和偏航角速度 ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);

    // 计算前向位置（用于预测，最多前向2秒）
    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  // ========== 情况2：轨迹执行完毕 ==========
  else if (t_cur >= traj_duration_)
  {
    /* 轨迹结束后悬停在终点 */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);  // 轨迹终点位置
    vel.setZero();  // 速度置零
    acc.setZero();  // 加速度置零

    // 保持最后的偏航角，偏航角速度置零
    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  // ========== 情况3：时间无效（负时间） ==========
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  // ========== 构建位置控制指令消息 ==========
  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;  // 轨迹状态：就绪
  cmd.trajectory_id = traj_id_;

  // 填充位置信息
  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  // 填充速度信息
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  // 填充加速度信息
  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  // 填充偏航角信息
  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  // 发布位置控制指令
  pos_cmd_pub.publish(cmd);
}

/**
 * @brief 主函数
 * @details 初始化ROS节点、订阅者、发布者和定时器
 *          主要流程：
 *          1. 初始化ROS节点
 *          2. 创建轨迹订阅器（订阅B样条轨迹）
 *          3. 创建位置控制指令发布器
 *          4. 创建定时器（100Hz频率发布控制指令）
 *          5. 初始化控制参数
 *          6. 进入ROS事件循环
 */
int main(int argc, char **argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;        // 全局命名空间节点句柄
  ros::NodeHandle nh("~");      // 私有命名空间节点句柄

  // 订阅B样条轨迹话题（由规划器发布）
  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);

  // 发布位置控制指令话题（发送给飞控系统）
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  // 创建定时器，以100Hz频率（0.01秒）调用cmdCallback函数
  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  /* 初始化控制参数 */
  // 位置PID控制增益
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  // 速度PID控制增益
  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  // 从参数服务器读取前向预测时间参数（用于偏航角计算）
  nh.param("traj_server/time_forward", time_forward_, -1.0);

  // 初始化偏航角相关变量
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  // 等待1秒，确保所有节点启动完成
  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  // 进入ROS事件循环，等待回调函数触发
  ros::spin();

  return 0;
}