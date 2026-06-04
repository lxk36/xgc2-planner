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



#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

// 返回x的符号函数值：x>0返回1，x<0返回-1，x=0返回0
double signum(double x);

// 计算value对modulus的模运算
double mod(double value, double modulus);

// 计算射线与网格边界的交点参数t
// s: 起点坐标分量, ds: 方向向量分量
double intbound(double s, double ds);

/**
 * @brief 3D Bresenham射线投射算法 - 数组输出版本
 * @param start 射线起点（世界坐标）
 * @param end 射线终点（世界坐标）
 * @param min 网格空间最小边界
 * @param max 网格空间最大边界
 * @param output_points_cnt 输出的体素点数量
 * @param output 输出的体素点数组（需预先分配内存）
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);

/**
 * @brief 3D Bresenham射线投射算法 - vector输出版本
 * @param start 射线起点（世界坐标）
 * @param end 射线终点（世界坐标）
 * @param min 网格空间最小边界
 * @param max 网格空间最大边界
 * @param output 输出的体素点容器（自动管理内存）
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

/**
 * @brief 3D射线投射器类
 * @details 基于DDA（数字微分分析器）算法的增量式射线遍历
 *          用于高效地遍历射线经过的所有体素网格
 */
class RayCaster {
private:
  /* 射线参数 */
  Eigen::Vector3d start_;      // 射线起点（世界坐标）
  Eigen::Vector3d end_;        // 射线终点（世界坐标）
  Eigen::Vector3d direction_;  // 射线方向向量（归一化）
  Eigen::Vector3d min_;        // 网格空间最小边界
  Eigen::Vector3d max_;        // 网格空间最大边界

  /* 当前体素坐标 */
  int x_;  // 当前x轴体素索引
  int y_;  // 当前y轴体素索引
  int z_;  // 当前z轴体素索引

  /* 终点体素坐标 */
  int endX_;  // 终点x轴体素索引
  int endY_;  // 终点y轴体素索引
  int endZ_;  // 终点z轴体素索引

  /* 射线属性 */
  double maxDist_;  // 射线最大长度（起点到终点的欧氏距离）
  double dx_;       // 射线x方向分量
  double dy_;       // 射线y方向分量
  double dz_;       // 射线z方向分量

  /* 步进方向（+1或-1） */
  int stepX_;  // x轴步进方向
  int stepY_;  // y轴步进方向
  int stepZ_;  // z轴步进方向

  /* tMax: 射线参数t到达下一个x/y/z网格边界的值 */
  double tMaxX_;  // 到达下一个x边界的t值
  double tMaxY_;  // 到达下一个y边界的t值
  double tMaxZ_;  // 到达下一个z边界的t值

  /* tDelta: 射线参数t在各轴向前进一个体素的增量 */
  double tDeltaX_;  // x方向前进一格的t增量
  double tDeltaY_;  // y方向前进一格的t增量
  double tDeltaZ_;  // z方向前进一格的t增量

  double dist_;  // 当前已遍历的射线长度

  int step_num_;  // 已执行的步进次数

public:
  // 默认构造函数
  RayCaster(/* args */) {
  }

  // 析构函数
  ~RayCaster() {
  }

  /**
   * @brief 设置射线投射的起点和终点
   * @param start 射线起点（世界坐标）
   * @param end 射线终点（世界坐标）
   * @return 设置成功返回true，否则返回false
   */
  bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  /**
   * @brief 沿射线前进一步，返回下一个体素点
   * @param ray_pt 输出参数，当前体素的中心点坐标
   * @return 如果还有下一个体素返回true，到达终点返回false
   */
  bool step(Eigen::Vector3d& ray_pt);
};

#endif  // RAYCAST_H_