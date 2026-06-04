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



#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <utility>

#include <fast_plan_env/obj_predictor.h>
#include <fast_plan_env/sdf_map.h>

using std::cout;
using std::endl;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
/**
 * @brief EDT环境类，用于评估欧几里得距离变换（Euclidean Distance Transform）
 *
 * 该类提供了基于SDF地图的距离场查询功能，支持动态障碍物预测和三线性插值梯度计算。
 * EDT用于在路径规划中快速计算空间中任意点到最近障碍物的距离及其梯度。
 */
class EDTEnvironment {
private:
  /* data */
  ObjPrediction obj_prediction_;  // 动态障碍物预测数据，用于预测障碍物未来位置
  ObjScale obj_scale_;             // 障碍物尺寸缩放参数，用于调整障碍物边界
  double resolution_inv_;          // 地图分辨率的倒数，用于快速坐标转换（避免除法运算）

  /**
   * @brief 计算指定点到某个边界框的最小距离
   * @param idx 边界框的索引
   * @param pos 查询点的位置
   * @param time 查询时刻（用于动态障碍物预测）
   * @return 点到边界框的最小距离
   */
  double distToBox(int idx, const Eigen::Vector3d& pos, const double& time);

  /**
   * @brief 计算指定点到所有边界框的最小距离
   * @param pos 查询点的位置
   * @param time 查询时刻（用于动态障碍物预测）
   * @return 点到所有边界框的最小距离
   */
  double minDistToAllBox(const Eigen::Vector3d& pos, const double& time);

public:
  /**
   * @brief 默认构造函数
   */
  EDTEnvironment(/* args */) {
  }

  /**
   * @brief 析构函数
   */
  ~EDTEnvironment() {
  }

  SDFMap::Ptr sdf_map_;  // SDF（Signed Distance Field）地图指针，存储环境的距离场信息

  /**
   * @brief 初始化EDT环境
   */
  void init();

  /**
   * @brief 设置SDF地图
   * @param map SDF地图的智能指针
   */
  void setMap(SDFMap::Ptr map);

  /**
   * @brief 设置动态障碍物预测信息
   * @param prediction 障碍物预测数据
   */
  void setObjPrediction(ObjPrediction prediction);

  /**
   * @brief 设置障碍物尺寸缩放参数
   * @param scale 障碍物缩放数据
   */
  void setObjScale(ObjScale scale);

  /**
   * @brief 获取查询点周围8个网格顶点的坐标和距离值
   * @param pts 输出参数，8个顶点的坐标（2x2x2数组）
   * @param dists 输出参数，8个顶点的距离值（2x2x2数组）
   */
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);

  /**
   * @brief 使用三线性插值计算距离值和梯度
   * @param values 输入参数，8个顶点的距离值（2x2x2数组）
   * @param diff 查询点在网格单元内的相对位置（归一化坐标）
   * @param value 输出参数，插值得到的距离值
   * @param grad 输出参数，插值得到的梯度向量
   */
  void interpolateTrilinear(double values[2][2][2], const Eigen::Vector3d& diff,
                            double& value, Eigen::Vector3d& grad);

  /**
   * @brief 评估指定位置和时间的EDT值及其梯度（精确计算）
   * @param pos 查询点的位置
   * @param time 查询时刻
   * @param dist 输出参数，到最近障碍物的距离
   * @param grad 输出参数，距离场的梯度向量
   */
  void evaluateEDTWithGrad(const Eigen::Vector3d& pos, double time,
                           double& dist, Eigen::Vector3d& grad);

  /**
   * @brief 评估指定位置和时间的粗略EDT值（不计算梯度）
   * @param pos 查询点的位置
   * @param time 查询时刻
   * @return 到最近障碍物的粗略距离估计
   */
  double evaluateCoarseEDT(Eigen::Vector3d& pos, double time);

  /**
   * @brief 获取地图的空间范围
   * @param ori 输出参数，地图的原点坐标
   * @param size 输出参数，地图的尺寸
   */
  void getMapRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
    sdf_map_->getRegion(ori, size);
  }

  typedef shared_ptr<EDTEnvironment> Ptr;  // 定义智能指针类型，用于管理EDTEnvironment对象的生命周期
};

}  // namespace fast_planner

#endif