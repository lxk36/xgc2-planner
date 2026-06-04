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



// 轨迹服务器头文件
#include "bspline/non_uniform_bspline.h"          // 非均匀B样条轨迹表示
#include "nav_msgs/Odometry.h"                     // 里程计消息
#include "fast_plan_manage/Bspline.h"              // B样条轨迹消息定义
#include "quadrotor_msgs/PositionCommand.h"        // 四旋翼位置命令消息
#include "std_msgs/Empty.h"                        // 空消息用于触发事件
#include "visualization_msgs/Marker.h"             // 可视化标记消息
#include <ros/ros.h>

// ROS发布器
// cmd_vis_pub: 发布位置命令的可视化
// pos_cmd_pub: 发布位置命令给控制器
// traj_pub: 发布轨迹可视化
ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;

// 里程计数据，存储当前位置和速度信息
nav_msgs::Odometry odom;

// 位置命令消息，发送给底层控制器
quadrotor_msgs::PositionCommand cmd;

// PD控制器增益参数
// pos_gain: 位置增益，分别对应x, y, z轴
double pos_gain[3] = { 5.7, 5.7, 6.2 };
// vel_gain: 速度增益，分别对应x, y, z轴
double vel_gain[3] = { 3.4, 3.4, 4.0 };

using fast_planner::NonUniformBspline;

// 轨迹接收标志，表示是否已接收到新轨迹
bool receive_traj_ = false;

// 轨迹存储向量，包含以下内容：
// traj_[0]: 位置轨迹
// traj_[1]: 速度轨迹（位置的一阶导数）
// traj_[2]: 加速度轨迹（速度的一阶导数）
// traj_[3]: 偏航角轨迹
// traj_[4]: 偏航角速率轨迹（偏航角的一阶导数）
vector<NonUniformBspline> traj_;

// 轨迹总时长（秒）
double traj_duration_;

// 轨迹开始执行的时刻
ros::Time start_time_;

// 轨迹ID，用于标识不同的轨迹
int traj_id_;

// 偏航角控制相关变量
double last_yaw_;        // 上一次的偏航角
double time_forward_;    // 前向时间参数

// 轨迹记录向量，用于可视化
vector<Eigen::Vector3d> traj_cmd_;   // 命令轨迹（期望轨迹）
vector<Eigen::Vector3d> traj_real_;  // 实际轨迹（从里程计获取）

/**
 * @brief 使用彩色球体列表显示轨迹
 * @param path 轨迹点序列
 * @param resolution 球体大小（米）
 * @param color RGBA颜色向量，范围[0,1]
 * @param id 可视化标记的唯一ID
 */
void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;

  // 先删除旧的标记
  traj_pub.publish(mk);

  // 创建新的标记
  mk.action = visualization_msgs::Marker::ADD;
  // 设置姿态（无旋转）
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  // 设置颜色
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  // 设置球体大小
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  // 添加所有轨迹点
  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

/**
 * @brief 绘制命令向量（箭头可视化）
 * @param pos 箭头起点位置
 * @param vec 箭头方向向量
 * @param id 可视化标记的唯一ID
 * @param color RGBA颜色向量，范围[0,1]
 */
void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  // 箭头尺寸参数：轴直径、箭头直径、箭头长度
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  // 箭头起点
  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  // 箭头终点
  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  // 设置颜色
  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

/**
 * @brief B样条轨迹接收回调函数
 * @param msg B样条轨迹消息，包含位置控制点、偏航角控制点、节点向量等信息
 *
 * 功能：解析接收到的B样条轨迹消息，构建位置轨迹和偏航角轨迹，
 *       并计算它们的导数（速度、加速度、偏航角速率）
 */
void bsplineCallback(fast_plan_manage::BsplineConstPtr msg) {
  // 解析位置轨迹
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);

  // 提取节点向量（knot vector）
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  // 提取位置控制点
  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  // 构建位置B样条轨迹
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // 解析偏航角轨迹

  // 提取偏航角控制点
  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(i, 0) = msg->yaw_pts[i];
  }

  // 构建偏航角B样条轨迹
  NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  // 记录轨迹的开始时间和ID
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  // 构建完整的轨迹序列
  traj_.clear();
  traj_.push_back(pos_traj);                    // traj_[0]: 位置
  traj_.push_back(traj_[0].getDerivative());    // traj_[1]: 速度
  traj_.push_back(traj_[1].getDerivative());    // traj_[2]: 加速度
  traj_.push_back(yaw_traj);                    // traj_[3]: 偏航角
  traj_.push_back(yaw_traj.getDerivative());    // traj_[4]: 偏航角速率

  // 计算轨迹总时长
  traj_duration_ = traj_[0].getTimeSum();

  // 标记已接收到轨迹
  receive_traj_ = true;
}

/**
 * @brief 重规划事件回调函数
 * @param msg 空消息，仅用于触发事件
 *
 * 功能：当收到重规划通知时，缩短当前轨迹的执行时长，
 *       使系统快速停止当前轨迹并准备接收新轨迹
 */
void replanCallback(std_msgs::Empty msg) {
  /* 重置轨迹持续时间 */
  const double time_out = 0.01;  // 超时时间，用于平滑过渡
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out;
  // 将轨迹时长设置为当前时刻+超时时间，提前终止轨迹
  traj_duration_ = min(t_stop, traj_duration_);
}

/**
 * @brief 新任务开始回调函数
 * @param msg 空消息，仅用于触发事件
 *
 * 功能：清空历史轨迹记录，为新的规划任务做准备
 */
void newCallback(std_msgs::Empty msg) {
  traj_cmd_.clear();   // 清空命令轨迹记录
  traj_real_.clear();  // 清空实际轨迹记录
}

/**
 * @brief 里程计数据回调函数
 * @param msg 里程计消息，包含位置、速度、姿态等信息
 *
 * 功能：接收并存储里程计数据，记录实际飞行轨迹用于可视化
 */
void odomCallbck(const nav_msgs::Odometry& msg) {
  // 过滤掉特殊标记的里程计消息
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;

  odom = msg;

  // 记录当前位置到实际轨迹
  traj_real_.push_back(
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  // 限制轨迹记录长度，防止内存溢出
  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

/**
 * @brief 可视化定时器回调函数
 * @param e 定时器事件
 *
 * 功能：定期更新轨迹的可视化显示
 */
void visCallback(const ros::TimerEvent& e) {
  // displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964, 1), 1);
  // 可选：显示实际轨迹（紫色）

  // 显示命令轨迹（绿色）
  displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
}

/**
 * @brief 位置命令发布定时器回调函数
 * @param e 定时器事件
 *
 * 功能：以固定频率（100Hz）发布位置命令给底层控制器
 *       根据当前时刻计算轨迹上的期望位置、速度、加速度和偏航角
 */
void cmdCallback(const ros::TimerEvent& e) {
  /* 未接收到轨迹前不发布命令 */
  if (!receive_traj_) return;

  // 计算当前时刻相对于轨迹起始时刻的时间
  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();

  // 轨迹状态变量
  Eigen::Vector3d pos, vel, acc, pos_f;  // 位置、速度、加速度、前向位置
  double yaw, yawdot;                     // 偏航角、偏航角速率

  // 根据当前时刻计算轨迹状态
  if (t_cur < traj_duration_ && t_cur >= 0.0) {
    // 轨迹执行中：使用De Boor算法评估B样条在当前时刻的值
    pos = traj_[0].evaluateDeBoorT(t_cur);      // 位置
    vel = traj_[1].evaluateDeBoorT(t_cur);      // 速度
    acc = traj_[2].evaluateDeBoorT(t_cur);      // 加速度
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];   // 偏航角
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0]; // 偏航角速率

    // 计算前向位置（用于偏航角控制，可选）
    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);

  } else if (t_cur >= traj_duration_) {
    /* 轨迹执行完毕后悬停 */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);  // 保持在终点位置
    vel.setZero();                                    // 速度置零
    acc.setZero();                                    // 加速度置零
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = traj_[4].evaluateDeBoorT(traj_duration_)[0];

    pos_f = pos;

  } else {
    // 时间异常（不应该发生）
    cout << "[Traj server]: invalid time." << endl;
  }

  // 填充位置命令消息
  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  // 期望位置
  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  // 期望速度
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  // 期望加速度
  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  // 期望偏航角和偏航角速率
  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;

  // 可选的偏航角控制策略（基于位置误差）
  auto pos_err = pos_f - pos;
  // if (pos_err.norm() > 1e-3) {
  //   cmd.yaw = atan2(pos_err(1), pos_err(0));
  // } else {
  //   cmd.yaw = last_yaw_;
  // }
  // cmd.yaw_dot = 1.0;

  last_yaw_ = cmd.yaw;

  // 发布位置命令
  pos_cmd_pub.publish(cmd);

  // 可视化命令向量
  // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));  // 速度向量（绿色）
  // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));  // 加速度向量（蓝色）

  // 绘制偏航方向（黄色箭头）
  Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
  drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
  // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));

  // 记录命令位置到轨迹历史
  traj_cmd_.push_back(pos);
  // 限制轨迹记录长度，防止内存溢出
  if (traj_cmd_.size() > 10000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

/**
 * @brief 主函数
 *
 * 功能：初始化ROS节点，设置订阅者和发布者，启动定时器
 *       轨迹服务器的主要作用是：
 *       1. 接收规划器生成的B样条轨迹
 *       2. 实时计算当前时刻的期望状态（位置、速度、加速度、偏航角）
 *       3. 将期望状态发布给底层控制器
 *       4. 可视化轨迹执行情况
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  // 订阅话题
  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);  // B样条轨迹
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);     // 重规划事件
  ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);              // 新任务事件
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);              // 里程计数据

  // 发布话题
  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);  // 命令可视化
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);         // 位置命令
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);          // 轨迹可视化

  // 创建定时器
  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);  // 100Hz发布位置命令
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);  // 4Hz更新可视化

  /* 设置控制参数 */
  cmd.kx[0] = pos_gain[0];  // x轴位置增益
  cmd.kx[1] = pos_gain[1];  // y轴位置增益
  cmd.kx[2] = pos_gain[2];  // z轴位置增益

  cmd.kv[0] = vel_gain[0];  // x轴速度增益
  cmd.kv[1] = vel_gain[1];  // y轴速度增益
  cmd.kv[2] = vel_gain[2];  // z轴速度增益

  // 从参数服务器读取前向时间参数
  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;

  // 等待其他节点启动
  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  // 进入事件循环
  ros::spin();

  return 0;
}
