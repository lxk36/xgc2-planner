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



#include <fast_plan_env/edt_environment.h>

namespace fast_planner {
/* ============================== edt_environment ==============================
 */

/**
 * @brief 初始化EDT环境
 * @note 当前为空实现，预留用于未来的初始化操作
 */
void EDTEnvironment::init() {
}

/**
 * @brief 设置SDF地图
 * @param map 共享指针指向的SDF地图对象
 * @note 同时计算并保存地图分辨率的倒数，用于后续快速计算
 */
void EDTEnvironment::setMap(shared_ptr<SDFMap> map) {
  this->sdf_map_ = map;  // 保存SDF地图的共享指针
  resolution_inv_ = 1 / sdf_map_->getResolution();  // 计算分辨率的倒数，避免重复除法运算
}

/**
 * @brief 设置动态障碍物的预测轨迹
 * @param prediction 动态障碍物的预测信息，包含障碍物的未来位置预测
 * @note 用于处理动态环境中的移动障碍物
 */
void EDTEnvironment::setObjPrediction(ObjPrediction prediction) {
  this->obj_prediction_ = prediction;
}

/**
 * @brief 设置动态障碍物的尺寸信息
 * @param scale 障碍物的尺寸缩放参数，定义每个障碍物的包围盒大小
 */
void EDTEnvironment::setObjScale(ObjScale scale) {
  this->obj_scale_ = scale;
}

/**
 * @brief 计算给定位置到指定动态障碍物包围盒的距离
 * @param idx 障碍物的索引
 * @param pos 查询点的3D位置
 * @param time 时间戳，用于预测障碍物在该时刻的位置
 * @return 返回点到包围盒的欧几里得距离
 * @note 使用匀速模型预测障碍物位置，计算点到轴对齐包围盒(AABB)的距离
 */
double EDTEnvironment::distToBox(int idx, const Eigen::Vector3d& pos, const double& time) {
  // 使用匀速模型评估障碍物在给定时间的位置
  // Eigen::Vector3d pos_box = obj_prediction_->at(idx).evaluate(time);  // 原始评估方法（已注释）
  Eigen::Vector3d pos_box = obj_prediction_->at(idx).evaluateConstVel(time);

  // 计算包围盒的上下边界（基于障碍物中心位置和尺寸）
  Eigen::Vector3d box_max = pos_box + 0.5 * obj_scale_->at(idx);  // 包围盒最大顶点
  Eigen::Vector3d box_min = pos_box - 0.5 * obj_scale_->at(idx);  // 包围盒最小顶点

  Eigen::Vector3d dist;

  // 计算每个轴方向上到包围盒的距离
  for (int i = 0; i < 3; i++) {
    // 如果点在包围盒内该维度的范围内，距离为0；否则取到两个边界的最小距离
    dist(i) = pos(i) >= box_min(i) && pos(i) <= box_max(i) ? 0.0 : min(fabs(pos(i) - box_min(i)),
                                                                       fabs(pos(i) - box_max(i)));
  }

  // 返回三个轴向距离的欧几里得范数
  return dist.norm();
}

/**
 * @brief 计算给定位置到所有动态障碍物包围盒的最小距离
 * @param pos 查询点的3D位置
 * @param time 时间戳，用于预测所有障碍物在该时刻的位置
 * @return 返回点到所有障碍物中最近的距离
 * @note 遍历所有障碍物，返回最小距离值
 */
double EDTEnvironment::minDistToAllBox(const Eigen::Vector3d& pos, const double& time) {
  double dist = 10000000.0;  // 初始化为一个很大的值
  // 遍历所有预测的动态障碍物
  for (int i = 0; i < obj_prediction_->size(); i++) {
    double di = distToBox(i, pos, time);  // 计算到第i个障碍物的距离
    if (di < dist) dist = di;  // 更新最小距离
  }

  return dist;
}

/**
 * @brief 获取周围8个顶点的距离场值
 * @param pts 输入的8个周围点坐标（2x2x2立方体的8个顶点）
 * @param dists 输出的8个点对应的距离场值
 * @note 用于三线性插值的准备步骤，获取立方体8个顶点的SDF值
 */
void EDTEnvironment::getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]) {
  // 遍历立方体的8个顶点（2x2x2）
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        // 从SDF地图中获取每个顶点的距离值
        dists[x][y][z] = sdf_map_->getDistance(pts[x][y][z]);
      }
    }
  }
}

/**
 * @brief 三线性插值计算距离场值和梯度
 * @param values 8个顶点的距离场值（2x2x2立方体）
 * @param diff 查询点在体素内的归一化位置（0-1之间的相对坐标）
 * @param value 输出的插值后的距离场值
 * @param grad 输出的距离场梯度向量
 * @note 使用三线性插值方法在离散网格中平滑地估计距离场值及其梯度
 *       梯度计算基于插值公式的偏导数
 */
void EDTEnvironment::interpolateTrilinear(double values[2][2][2],
                                          const Eigen::Vector3d& diff,
                                          double& value,
                                          Eigen::Vector3d& grad) {
  // 三线性插值算法
  // 第一步：沿x轴方向插值，得到4个中间值
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];  // z=0, y=0处的x方向插值
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];  // z=1, y=0处的x方向插值
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];  // z=0, y=1处的x方向插值
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];  // z=1, y=1处的x方向插值

  // 第二步：沿y轴方向插值，得到2个中间值
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;  // z=0处的y方向插值
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;  // z=1处的y方向插值

  // 第三步：沿z轴方向插值，得到最终距离值
  value = (1 - diff(2)) * v0 + diff(2) * v1;

  // 计算梯度：对插值公式分别求x、y、z三个方向的偏导数
  // z方向梯度（最简单，直接由v0和v1差分得到）
  grad[2] = (v1 - v0) * resolution_inv_;

  // y方向梯度（需要考虑z方向的加权）
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv_;

  // x方向梯度（需要考虑y和z两个方向的加权，分四项计算）
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);  // z=0, y=0的贡献
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);       // z=0, y=1的贡献
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);       // z=1, y=0的贡献
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);             // z=1, y=1的贡献
  grad[0] *= resolution_inv_;  // 乘以分辨率倒数，完成差分到导数的转换
}

/**
 * @brief 评估欧几里得距离变换(EDT)并计算梯度
 * @param pos 查询点的3D位置
 * @param time 时间戳（当前未使用，预留用于动态环境）
 * @param dist 输出的距离场值
 * @param grad 输出的距离场梯度向量
 * @note 通过三线性插值获得平滑的距离场值和梯度，用于优化和碰撞检测
 */
void EDTEnvironment::evaluateEDTWithGrad(const Eigen::Vector3d& pos,
                                         double time, double& dist,
                                         Eigen::Vector3d& grad) {
  Eigen::Vector3d diff;  // 存储查询点在体素内的归一化位置
  Eigen::Vector3d sur_pts[2][2][2];  // 存储周围8个体素顶点的坐标

  // 获取查询点周围的8个网格顶点坐标和相对位置
  sdf_map_->getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];  // 存储8个顶点的距离场值
  // 查询这8个顶点的距离场值
  getSurroundDistance(sur_pts, dists);

  // 使用三线性插值计算平滑的距离值和梯度
  interpolateTrilinear(dists, diff, dist, grad);
}

/**
 * @brief 评估粗略的欧几里得距离变换
 * @param pos 查询点的3D位置
 * @param time 时间戳，用于考虑动态障碍物
 * @return 返回到最近障碍物（静态或动态）的距离
 * @note 如果time < 0，只考虑静态障碍物；否则同时考虑静态和动态障碍物，返回较小值
 */
double EDTEnvironment::evaluateCoarseEDT(Eigen::Vector3d& pos, double time) {
  // 获取到静态障碍物的距离（从SDF地图）
  double d1 = sdf_map_->getDistance(pos);

  if (time < 0.0) {
    // 时间无效时，只返回静态障碍物距离
    return d1;
  } else {
    // 计算到所有动态障碍物的最小距离
    double d2 = minDistToAllBox(pos, time);
    // 返回静态和动态障碍物中的最小距离
    return min(d1, d2);
  }
}

// EDTEnvironment::
}  // namespace fast_planner