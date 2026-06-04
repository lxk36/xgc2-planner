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



#ifndef _OBJ_PREDICTOR_H_
#define _OBJ_PREDICTOR_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <list>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
// 前向声明多项式预测类
class PolynomialPrediction;
// 物体预测轨迹类型：指向多项式预测向量的智能指针
typedef shared_ptr<vector<PolynomialPrediction>> ObjPrediction;
// 物体尺度类型：指向三维向量的智能指针，用于存储物体的尺度信息
typedef shared_ptr<vector<Eigen::Vector3d>> ObjScale;

/* ========== prediction polynomial ========== */
/**
 * @brief 多项式预测类
 * @details 使用5阶多项式表示物体的预测轨迹，可以对未来的位置进行预测
 */
class PolynomialPrediction {
private:
  vector<Eigen::Matrix<double, 6, 1>> polys;  // 多项式系数向量，分别对应x、y、z三个维度的5阶多项式
  double t1, t2;  // 预测的时间范围：起始时间和结束时间

public:
  // 默认构造函数
  PolynomialPrediction(/* args */) {
  }
  // 析构函数
  ~PolynomialPrediction() {
  }

  /**
   * @brief 设置多项式系数
   * @param pls 多项式系数向量，包含x、y、z三个维度的系数
   */
  void setPolynomial(vector<Eigen::Matrix<double, 6, 1>>& pls) {
    polys = pls;
  }

  /**
   * @brief 设置预测的时间范围
   * @param t1 起始时间
   * @param t2 结束时间
   */
  void setTime(double t1, double t2) {
    this->t1 = t1;
    this->t2 = t2;
  }

  /**
   * @brief 检查多项式是否有效
   * @return 如果包含3个维度的多项式（x、y、z）则返回true
   */
  bool valid() {
    return polys.size() == 3;
  }

  /**
   * @brief 计算给定时间t的预测位置（5阶多项式）
   * @param t 时间点，应在[t1, t2]范围内
   * @return 预测的三维位置坐标
   * @note t应该在[t1, t2]范围内
   */
  Eigen::Vector3d evaluate(double t) {
    Eigen::Matrix<double, 6, 1> tv;
    // 构建时间向量 [1, t, t^2, t^3, t^4, t^5]
    tv << 1.0, pow(t, 1), pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);

    Eigen::Vector3d pt;
    // 计算各维度的位置：pt = 系数向量 · 时间向量
    pt(0) = tv.dot(polys[0]), pt(1) = tv.dot(polys[1]), pt(2) = tv.dot(polys[2]);

    return pt;
  }

  /**
   * @brief 计算给定时间t的预测位置（恒定速度模型，1阶多项式）
   * @param t 时间点
   * @return 预测的三维位置坐标
   * @details 仅使用多项式的前两项（常数项和一次项），即匀速运动模型
   */
  Eigen::Vector3d evaluateConstVel(double t) {
    Eigen::Matrix<double, 2, 1> tv;
    // 构建时间向量 [1, t]
    tv << 1.0, pow(t, 1);

    Eigen::Vector3d pt;
    // 仅使用多项式的前两项计算位置
    pt(0) = tv.dot(polys[0].head(2)), pt(1) = tv.dot(polys[1].head(2)), pt(2) = tv.dot(polys[2].head(2));

    return pt;
  }
};

/* ========== subscribe and record object history ========== */
/**
 * @brief 物体历史记录类
 * @details 订阅物体位姿话题，记录并维护物体的历史轨迹数据，用于预测
 */
class ObjHistory {
public:
  static int skip_num_;              // 跳过的帧数，用于降采样
  static int queue_size_;            // 历史记录队列的大小
  static ros::Time global_start_time_;  // 全局起始时间，用于统一时间基准

  // 默认构造函数
  ObjHistory() {
  }
  // 析构函数
  ~ObjHistory() {
  }

  /**
   * @brief 初始化物体历史记录
   * @param id 物体的索引ID
   */
  void init(int id);

  /**
   * @brief 位姿话题的回调函数
   * @param msg 接收到的位姿消息
   * @details 记录物体的位置和时间戳到历史队列中
   */
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  /**
   * @brief 清空历史记录
   */
  void clear() {
    history_.clear();
  }

  /**
   * @brief 获取历史记录
   * @param his 输出参数，存储历史轨迹数据
   */
  void getHistory(list<Eigen::Vector4d>& his) {
    his = history_;
  }

private:
  list<Eigen::Vector4d> history_;  // 历史位置和时间记录，格式：(x, y, z, t)
  int skip_;                        // 当前跳过的帧计数器
  int obj_idx_;                     // 物体索引
  Eigen::Vector3d scale_;           // 物体的尺度（长宽高）
};

/* ========== predict future trajectory using history ========== */
/**
 * @brief 物体预测器类
 * @details 基于历史轨迹数据预测动态物体未来的运动轨迹，支持多项式拟合和恒速模型两种预测方式
 */
class ObjPredictor {
private:
  ros::NodeHandle node_handle_;  // ROS节点句柄

  int obj_num_;                   // 需要跟踪的物体数量
  double lambda_;                 // 多项式拟合的正则化参数，用于平滑轨迹
  double predict_rate_;           // 预测更新频率（Hz）

  vector<ros::Subscriber> pose_subs_;              // 物体位姿订阅者列表
  ros::Subscriber marker_sub_;                     // 可视化标记订阅者，用于获取物体尺度
  ros::Timer predict_timer_;                       // 预测定时器，按固定频率触发预测
  vector<shared_ptr<ObjHistory>> obj_histories_;   // 物体历史记录列表

  /* share data with planner */
  ObjPrediction predict_trajs_;  // 预测轨迹数据，与规划器共享
  ObjScale obj_scale_;           // 物体尺度数据，与规划器共享
  vector<bool> scale_init_;      // 标记各物体尺度是否已初始化

  /**
   * @brief 可视化标记话题的回调函数
   * @param msg 接收到的Marker消息
   * @details 从Marker消息中提取物体的尺度信息（长宽高）
   */
  void markerCallback(const visualization_msgs::MarkerConstPtr& msg);

  /**
   * @brief 预测定时器的回调函数
   * @param e 定时器事件
   * @details 周期性触发，调用预测算法更新物体的未来轨迹
   */
  void predictCallback(const ros::TimerEvent& e);

  /**
   * @brief 使用多项式拟合方法预测轨迹
   * @details 基于历史数据拟合5阶多项式，生成平滑的预测轨迹
   */
  void predictPolyFit();

  /**
   * @brief 使用恒定速度模型预测轨迹
   * @details 假设物体以当前速度匀速运动，生成线性预测轨迹
   */
  void predictConstVel();

public:
  // 默认构造函数
  ObjPredictor(/* args */);

  /**
   * @brief 构造函数
   * @param node ROS节点句柄
   */
  ObjPredictor(ros::NodeHandle& node);

  // 析构函数
  ~ObjPredictor();

  /**
   * @brief 初始化预测器
   * @details 从参数服务器读取配置，创建订阅者和定时器，初始化数据结构
   */
  void init();

  /**
   * @brief 获取预测轨迹
   * @return 指向预测轨迹向量的智能指针
   */
  ObjPrediction getPredictionTraj();

  /**
   * @brief 获取物体尺度信息
   * @return 指向物体尺度向量的智能指针
   */
  ObjScale getObjScale();

  // 定义智能指针类型别名
  typedef shared_ptr<ObjPredictor> Ptr;
};

}  // namespace fast_planner

#endif