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



#ifndef _LINEAR_OBJ_MODEL_H_
#define _LINEAR_OBJ_MODEL_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

/**
 * @brief 线性物体运动模型类
 *
 * 用于模拟动态障碍物的线性运动行为，包括位置、速度、姿态等状态的更新，
 * 以及碰撞检测和边界处理等功能。主要用于Fast-Planner中的动态障碍物仿真。
 */
class LinearObjModel {
private:
  /* data */
public:
  /**
   * @brief 构造函数
   */
  LinearObjModel(/* args */);

  /**
   * @brief 析构函数
   */
  ~LinearObjModel();

  /**
   * @brief 初始化物体的状态参数
   * @param p 初始位置 (x, y, z)
   * @param v 初始速度 (vx, vy, vz)
   * @param a 初始加速度 (ax, ay, az)
   * @param yaw 初始偏航角
   * @param yaw_dot 初始偏航角速度
   * @param color 物体颜色 (r, g, b)，用于可视化
   * @param scale 物体尺寸 (length, width, height)
   */
  void initialize(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw, double yaw_dot,
                  Eigen::Vector3d color, Eigen::Vector3d scale);

  /**
   * @brief 设置物体的运动限制参数
   * @param bound 空间边界范围 (x_bound, y_bound, z_bound)
   * @param vel 速度限制 (v_min, v_max)
   * @param acc 加速度限制 (a_min, a_max)
   */
  void setLimits(Eigen::Vector3d bound, Eigen::Vector2d vel, Eigen::Vector2d acc);

  /**
   * @brief 更新物体状态（使用线性积分器模型）
   * @param dt 时间步长
   *
   * 基于当前速度和时间步长更新位置，处理边界碰撞和反弹
   */
  void update(double dt);  // linear trippler integrator model

  /**
   * @brief 静态方法：检测两个物体之间的碰撞
   * @param obj1 第一个物体
   * @param obj2 第二个物体
   * @return 如果发生碰撞返回true，否则返回false
   *
   * 使用AABB（轴对齐包围盒）碰撞检测算法，碰撞时会自动处理速度反弹和位置调整
   */
  static bool collide(LinearObjModel& obj1, LinearObjModel& obj2);

  /**
   * @brief 设置物体的输入速度
   * @param vel 目标速度向量 (vx, vy, vz)
   */
  // void setInput(Eigen::Vector3d acc) { acc_ = acc; }
  void setInput(Eigen::Vector3d vel) {
    vel_ = vel;
  }

  /**
   * @brief 设置偏航角速度
   * @param yaw_dot 偏航角速度 (rad/s)
   */
  void setYawDot(double yaw_dot) {
    yaw_dot_ = yaw_dot;
  }

  /**
   * @brief 获取当前位置
   * @return 位置向量 (x, y, z)
   */
  Eigen::Vector3d getPosition() {
    return pos_;
  }

  /**
   * @brief 设置当前位置
   * @param pos 位置向量 (x, y, z)
   */
  void setPosition(Eigen::Vector3d pos) {
    pos_ = pos;
  }

  /**
   * @brief 获取当前速度
   * @return 速度向量 (vx, vy, vz)
   */
  Eigen::Vector3d getVelocity() {
    return vel_;
  }

  /**
   * @brief 设置速度的各个分量
   * @param x x方向速度
   * @param y y方向速度
   * @param z z方向速度
   */
  void setVelocity(double x, double y, double z) {
    vel_ = Eigen::Vector3d(x, y, z);
  }

  /**
   * @brief 获取物体颜色
   * @return 颜色向量 (r, g, b)
   */
  Eigen::Vector3d getColor() {
    return color_;
  }

  /**
   * @brief 获取物体尺寸
   * @return 尺寸向量 (length, width, height)
   */
  Eigen::Vector3d getScale() {
    return scale_;
  }

  /**
   * @brief 获取当前偏航角
   * @return 偏航角 (rad)
   */
  double getYaw() {
    return yaw_;
  }

private:
  Eigen::Vector3d pos_, vel_, acc_;  // 位置、速度、加速度状态变量
  Eigen::Vector3d color_, scale_;    // 物体颜色和尺寸（用于可视化和碰撞检测）
  double yaw_, yaw_dot_;             // 偏航角和偏航角速度

  Eigen::Vector3d bound_;            // 空间边界范围 (x, y, z)
  Eigen::Vector2d limit_v_, limit_a_;// 速度和加速度的限制范围
};

LinearObjModel::LinearObjModel(/* args */) {
}

LinearObjModel::~LinearObjModel() {
}

/**
 * @brief 初始化函数实现
 *
 * 设置物体的初始运动状态和外观参数
 */
void LinearObjModel::initialize(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw,
                                double yaw_dot, Eigen::Vector3d color, Eigen::Vector3d scale) {
  pos_ = p;        // 设置初始位置
  vel_ = v;        // 设置初始速度
  acc_ = a;        // 设置初始加速度
  color_ = color;  // 设置颜色
  scale_ = scale;  // 设置尺寸

  yaw_ = yaw;          // 设置初始偏航角
  yaw_dot_ = yaw_dot;  // 设置初始偏航角速度
}

/**
 * @brief 设置限制参数函数实现
 *
 * 配置物体的运动边界和速度、加速度限制
 */
void LinearObjModel::setLimits(Eigen::Vector3d bound, Eigen::Vector2d vel, Eigen::Vector2d acc) {
  bound_ = bound;    // 设置空间边界
  limit_v_ = vel;    // 设置速度限制
  limit_a_ = acc;    // 设置加速度限制
}

/**
 * @brief 状态更新函数实现
 *
 * 基于线性积分器模型更新物体状态，包括位置、姿态更新和边界碰撞处理
 */
void LinearObjModel::update(double dt) {
  Eigen::Vector3d p0, v0, a0;
  p0 = pos_, v0 = vel_, a0 = acc_;  // 保存当前状态

  /* ---------- 使用加速度作为输入（已注释） ---------- */
  // vel_ = v0 + acc_ * dt;
  // for (int i = 0; i < 3; ++i)
  // {
  //   if (vel_(i) > 0) vel_(i) = std::max(limit_v_(0), std::min(vel_(i),
  //   limit_v_(1)));
  //   if (vel_(i) <= 0) vel_(i) = std::max(-limit_v_(1), std::min(vel_(i),
  //   -limit_v_(0)));
  // }

  // pos_ = p0 + v0 * dt + 0.5 * acc_ * pow(dt, 2);
  // for (int i = 0; i < 2; ++i)
  // {
  //   pos_(i) = std::min(pos_(i), bound_(i));
  //   pos_(i) = std::max(pos_(i), -bound_(i));
  // }
  // pos_(2) = std::min(pos_(2), bound_(2));
  // pos_(2) = std::max(pos_(2), 0.0);

  /* ---------- 使用速度作为输入（当前使用） ---------- */
  // 基于一阶积分更新位置：p = p0 + v * dt
  pos_ = p0 + v0 * dt;

  // 限制x和y坐标在边界范围内
  for (int i = 0; i < 2; ++i) {
    pos_(i) = std::min(pos_(i), bound_(i));    // 上界限制
    pos_(i) = std::max(pos_(i), -bound_(i));   // 下界限制
  }
  // 限制z坐标在[0, bound_z]范围内
  pos_(2) = std::min(pos_(2), bound_(2));
  pos_(2) = std::max(pos_(2), 0.0);

  // 更新偏航角
  yaw_ += yaw_dot_ * dt;

  // 将偏航角限制在[0, 2π]范围内
  const double PI = 3.1415926;
  if (yaw_ > 2 * PI) yaw_ -= 2 * PI;

  /* ---------- 边界碰撞时反弹处理 ---------- */
  const double tol = 0.1;  // 边界容差，防止物体卡在边界上

  // X轴正边界碰撞检测与反弹
  if (pos_(0) > bound_(0) - tol) {
    pos_(0) = bound_(0) - tol;
    vel_(0) = -vel_(0);  // 速度反向
  }
  // X轴负边界碰撞检测与反弹
  if (pos_(0) < -bound_(0) + tol) {
    pos_(0) = -bound_(0) + tol;
    vel_(0) = -vel_(0);
  }

  // Y轴正边界碰撞检测与反弹
  if (pos_(1) > bound_(1) - tol) {
    pos_(1) = bound_(1) - tol;
    vel_(1) = -vel_(1);
  }
  // Y轴负边界碰撞检测与反弹
  if (pos_(1) < -bound_(1) + tol) {
    pos_(1) = -bound_(1) + tol;
    vel_(1) = -vel_(1);
  }

  // Z轴上边界碰撞检测与反弹
  if (pos_(2) > bound_(2) - tol) {
    pos_(2) = bound_(2) - tol;
    vel_(2) = -vel_(2);
  }
  // Z轴下边界（地面）碰撞检测与反弹
  if (pos_(2) < tol) {
    pos_(2) = tol;
    vel_(2) = -vel_(2);
  }
}

/**
 * @brief 碰撞检测和处理函数实现
 *
 * 使用AABB（轴对齐包围盒）算法检测两个物体是否碰撞，
 * 如果碰撞则自动处理速度反弹和位置分离，模拟弹性碰撞效果
 */
bool LinearObjModel::collide(LinearObjModel& obj1, LinearObjModel& obj2) {
  Eigen::Vector3d pos1, pos2, vel1, vel2, scale1, scale2;

  // 获取第一个物体的状态
  pos1 = obj1.getPosition();
  vel1 = obj1.getVelocity();
  scale1 = obj1.getScale();

  // 获取第二个物体的状态
  pos2 = obj2.getPosition();
  vel2 = obj2.getVelocity();
  scale2 = obj2.getScale();

  /* ---------- 碰撞检测（AABB算法） ---------- */
  // 检查三个轴向上是否都发生重叠
  bool collide = fabs(pos1(0) - pos2(0)) < 0.5 * (scale1(0) + scale2(0)) &&
      fabs(pos1(1) - pos2(1)) < 0.5 * (scale1(1) + scale2(1)) &&
      fabs(pos1(2) - pos2(2)) < 0.5 * (scale1(2) + scale2(2));

  if (collide) {
    // 计算每个轴向上的重叠量
    double tol[3];
    tol[0] = 0.5 * (scale1(0) + scale2(0)) - fabs(pos1(0) - pos2(0));  // X轴重叠量
    tol[1] = 0.5 * (scale1(1) + scale2(1)) - fabs(pos1(1) - pos2(1));  // Y轴重叠量
    tol[2] = 0.5 * (scale1(2) + scale2(2)) - fabs(pos1(2) - pos2(2));  // Z轴重叠量

    // 找到重叠量最小的轴，沿该轴进行分离和速度反弹
    for (int i = 0; i < 3; ++i) {
      if (tol[i] < tol[(i + 1) % 3] && tol[i] < tol[(i + 2) % 3]) {
        // 在该轴向上反转两个物体的速度（弹性碰撞）
        vel1(i) = -vel1(i);
        vel2(i) = -vel2(i);
        obj1.setVelocity(vel1(0), vel1(1), vel1(2));
        obj2.setVelocity(vel2(0), vel2(1), vel2(2));

        // 沿该轴向分离两个物体，消除重叠
        if (pos1(i) >= pos2(i)) {
          pos1(i) += tol[i];  // 物体1向正方向移动
          pos2(i) -= tol[i];  // 物体2向负方向移动
        } else {
          pos1(i) -= tol[i];  // 物体1向负方向移动
          pos2(i) += tol[i];  // 物体2向正方向移动
        }
        obj1.setPosition(pos1);
        obj2.setPosition(pos2);

        break;  // 只需在一个轴向上处理
      }
    }

    return true;  // 发生碰撞
  } else {
    return false;  // 未发生碰撞
  }
}

#endif