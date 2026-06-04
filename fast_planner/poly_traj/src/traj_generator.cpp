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



#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <Eigen/Eigen>
#include <ros/ros.h>

#include <swarmtal_msgs/drone_onboard_command.h>
#include <traj_generator/polynomial_traj.hpp>

using namespace std;

// ROS发布器：状态可视化、位置指令、轨迹可视化
ros::Publisher state_pub, pos_cmd_pub, traj_pub;

// 里程计数据及其有效性标志
nav_msgs::Odometry odom;
bool have_odom;

/**
 * @brief 以指定颜色显示路径点序列
 * @param path 路径点向量
 * @param resolution 球体分辨率（大小）
 * @param color RGBA颜色向量
 * @param id Marker的唯一标识符
 */
void displayPathWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;  // 使用球体列表类型
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;

  // 先删除旧的marker
  traj_pub.publish(mk);

  // 设置marker为添加模式
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  // 设置颜色（RGBA）
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  // 设置球体大小
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  // 将路径点添加到marker中
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
 * @brief 绘制状态向量（如速度、加速度）
 * @param pos 起始位置
 * @param vec 状态向量（速度或加速度）
 * @param id Marker的唯一标识符
 * @param color RGBA颜色向量
 */
void drawState(Eigen::Vector3d pos, Eigen::Vector3d vec, int id, Eigen::Vector4d color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;  // 使用箭头类型表示向量
  mk_state.action = visualization_msgs::Marker::ADD;
  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;  // 箭头杆的直径
  mk_state.scale.y = 0.2;  // 箭头头部的直径
  mk_state.scale.z = 0.3;  // 箭头头部的长度

  // 设置箭头的起点（当前位置）
  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  // 设置箭头的终点（位置+向量）
  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  // 设置箭头颜色
  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);
  state_pub.publish(mk_state);
}

/**
 * @brief 里程计数据回调函数
 * @param msg 里程计消息
 */
void odomCallbck(const nav_msgs::Odometry& msg) {
  // 过滤特殊标记的消息
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;

  odom = msg;
  have_odom = true;
}

/**
 * @brief 主函数 - 轨迹生成器节点
 * 使用闭式解最小Jerk方法生成多段多项式轨迹，并实时发布控制指令
 */
int main(int argc, char** argv) {
  /* ---------- 初始化ROS节点 ---------- */
  ros::init(argc, argv, "traj_generator");
  ros::NodeHandle node;

  // 订阅里程计话题
  ros::Subscriber odom_sub = node.subscribe("/uwb_vicon_odom", 50, odomCallbck);

  // 发布轨迹可视化消息
  traj_pub = node.advertise<visualization_msgs::Marker>("/traj_generator/traj_vis", 10);
  // 发布状态（速度、加速度）可视化消息
  state_pub = node.advertise<visualization_msgs::Marker>("/traj_generator/cmd_vis", 10);

  // pos_cmd_pub =
  // node.advertise<quadrotor_msgs::PositionCommand>("/traj_generator/position_cmd",
  // 50);

  // 发布无人机机载控制指令
  pos_cmd_pub =
      node.advertise<swarmtal_msgs::drone_onboard_command>("/drone_commander/onboard_command", 10);

  ros::Duration(1.0).sleep();

  /* ---------- 等待里程计数据就绪 ---------- */
  have_odom = false;
  while (!have_odom && ros::ok()) {
    cout << "no odomeetry." << endl;
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }

  /* ---------- 使用闭式解最小Jerk方法生成轨迹 ---------- */
  // 定义路径点矩阵：9个路径点，每个点3维坐标(x,y,z)
  Eigen::MatrixXd pos(9, 3);
  // pos.row(0) =
  //     Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y,
  //     odom.pose.pose.position.z);

  // 起始点：当前里程计位置
  pos.row(0) =
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  // pos.row(0) = Eigen::Vector3d(-2, 0, 1);

  // 预定义的8字形轨迹路径点
  pos.row(1) = Eigen::Vector3d(-0.5, 0.5, 1);
  pos.row(2) = Eigen::Vector3d(0, 0, 1);
  pos.row(3) = Eigen::Vector3d(0.5, -0.5, 1);
  pos.row(4) = Eigen::Vector3d(1, 0, 1);
  pos.row(5) = Eigen::Vector3d(0.5, 0.5, 1);
  pos.row(6) = Eigen::Vector3d(0, 0, 1);
  pos.row(7) = Eigen::Vector3d(-0.5, -0.5, 1);
  pos.row(8) = Eigen::Vector3d(-1, 0, 1);

  // 定义每段轨迹的时间分配（8个段）
  Eigen::VectorXd time(8);
  time(0) = 2.0;  // 第1段时长
  time(1) = 1.5;  // 第2段时长
  time(2) = 1.5;  // 第3段时长
  time(3) = 1.5;  // 第4段时长
  time(4) = 1.5;  // 第5段时长
  time(5) = 1.5;  // 第6段时长
  time(6) = 1.5;  // 第7段时长
  time(7) = 2.0;  // 第8段时长

  // 生成最小Jerk轨迹的多项式系数矩阵
  Eigen::MatrixXd poly = generateTraj(pos, time);
  cout << "poly:\n" << poly << endl;

  cout << "pos:\n" << pos << endl;

  cout << "pos 0 1 2: " << pos(0) << ", " << pos(1) << ", " << pos(2) << endl;

  /* ---------- 使用多项式系数构建轨迹对象 ---------- */
  PolynomialTraj poly_traj;
  // 遍历每一段轨迹
  for (int i = 0; i < poly.rows(); ++i) {
    // 提取x、y、z三个方向的多项式系数（每个方向6个系数，对应5阶多项式）
    vector<double> cx(6), cy(6), cz(6);
    for (int j = 0; j < 6; ++j) {
      cx[j] = poly(i, j);       // x方向系数
      cy[j] = poly(i, j + 6);   // y方向系数
      cz[j] = poly(i, j + 12);  // z方向系数
    }
    // 反转系数顺序（从高阶到低阶变为从低阶到高阶）
    reverse(cx.begin(), cx.end());
    reverse(cy.begin(), cy.end());
    reverse(cz.begin(), cz.end());
    double ts = time(i);
    // 添加该段轨迹
    poly_traj.addSegment(cx, cy, cz, ts);
  }
  // 初始化轨迹对象
  poly_traj.init();
  // 获取用于可视化的轨迹点序列
  vector<Eigen::Vector3d> traj_vis = poly_traj.getTraj();

  // 以红色显示生成的轨迹
  displayPathWithColor(traj_vis, 0.05, Eigen::Vector4d(1, 0, 0, 1), 1);

  /* ---------- 发布控制指令 ---------- */
  ros::Time start_time = ros::Time::now();  // 记录轨迹开始时间
  ros::Time time_now;

  ros::Duration(0.1).sleep();

  // 初始化无人机机载控制指令
  swarmtal_msgs::drone_onboard_command cmd;
  cmd.command_type = swarmtal_msgs::drone_onboard_command::CTRL_POS_COMMAND;  // 位置控制模式
  cmd.param1 = 0;   // x位置（稍后更新）
  cmd.param2 = 0;   // y位置（稍后更新）
  cmd.param3 = 0;   // z位置（稍后更新）
  cmd.param4 = 666666;  // 特殊标记
  cmd.param5 = 0;   // x速度（稍后更新）
  cmd.param6 = 0;   // y速度（稍后更新）
  cmd.param7 = 0;   // z速度
  cmd.param8 = 0;   // x加速度（稍后更新）
  cmd.param9 = 0;   // y加速度（稍后更新）
  cmd.param10 = 0;  // z加速度

  // 主控制循环：实时计算并发布轨迹上的位置、速度、加速度指令
  while (ros::ok()) {
    time_now = ros::Time::now();
    double tn = (time_now - start_time).toSec();  // 计算当前时刻相对于轨迹起始的时间

    // 在当前时刻评估轨迹，获取位置、速度、加速度
    Eigen::Vector3d pt = poly_traj.evaluate(tn);       // 位置
    Eigen::Vector3d vel = poly_traj.evaluateVel(tn);   // 速度
    Eigen::Vector3d acc = poly_traj.evaluateAcc(tn);   // 加速度

    // 将位置信息编码到指令参数中（乘以10000进行定点化）
    cmd.param1 = int(pt(0) * 10000);   // x位置
    cmd.param2 = int(pt(1) * 10000);   // y位置
    cmd.param3 = int(pt(2) * 10000);   // z位置

    // 将速度信息编码到指令参数中
    cmd.param5 = int(vel(0) * 10000);  // x速度
    cmd.param6 = int(vel(1) * 10000);  // y速度

    // 将加速度信息编码到指令参数中
    cmd.param7 = 0;
    cmd.param8 = int(acc(0) * 10000);  // x加速度
    cmd.param9 = int(acc(1) * 10000);  // y加速度

    // 发布控制指令
    pos_cmd_pub.publish(cmd);

    // 可视化速度向量（绿色箭头）
    drawState(pt, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
    // 可视化加速度向量（蓝色箭头）
    drawState(pt, acc, 1, Eigen::Vector4d(0, 0, 1, 1));

    ros::Duration(0.01).sleep();  // 100Hz控制频率
  }

  ros::spin();
  return 0;
}
