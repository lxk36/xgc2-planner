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



#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <fast_plan_env/raycast.h>

/**
 * @brief 符号函数 - 返回整数的符号
 * @param x 输入整数
 * @return 如果x为0返回0，如果x<0返回-1，如果x>0返回1
 */
int signum(int x) {
  return x == 0 ? 0 : x < 0 ? -1 : 1;
}

/**
 * @brief 取模运算 - 计算正数模运算结果
 * @param value 被除数
 * @param modulus 除数
 * @return 始终返回正数的模运算结果
 * @note 使用两次fmod确保结果始终为正数
 */
double mod(double value, double modulus) {
  return fmod(fmod(value, modulus) + modulus, modulus);
}

/**
 * @brief 计算整数边界 - 找到最小的正数t使得s+t*ds是整数
 * @param s 起始位置
 * @param ds 方向增量
 * @return 使s+t*ds成为整数的最小正数t值
 * @note 这个函数用于计算射线到达下一个体素边界所需的参数t
 */
double intbound(double s, double ds) {
  // 找到最小的正数t，使得s+t*ds是整数
  if (ds < 0) {
    // 如果方向为负，通过取反转换为正方向情况
    return intbound(-s, -ds);
  } else {
    // 将s转换到[0,1)范围内
    s = mod(s, 1);
    // 现在问题变为求解 s+t*ds = 1
    return (1 - s) / ds;
  }
}

/**
 * @brief 基于DDA的快速体素遍历算法（数组输出版本）
 * @param start 射线起点
 * @param end 射线终点
 * @param min 有效体素范围的最小边界
 * @param max 有效体素范围的最大边界
 * @param output_points_cnt 输出点的数量（输入输出参数）
 * @param output 输出的体素坐标数组
 *
 * @note 基于论文 "A Fast Voxel Traversal Algorithm for Ray Tracing"
 *       作者: John Amanatides and Andrew Woo, 1987
 *       论文链接: http://www.cse.yorku.ca/~amana/research/grid.pdf
 *
 * 算法扩展:
 *   • 添加了距离限制
 *   • 提供了穿过当前体素所经过的面信息
 *
 * 算法原理:
 *   该算法基于射线的参数化表示: origin + t * direction
 *   但实际上不存储t值，而是在遍历过程中跟踪tMax值，
 *   即在每个轴上跨越体素边界（改变坐标整数部分）所需的较大t值
 *   这些值存储在变量 tMaxX, tMaxY, tMaxZ 中
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output) {
  //    std::cout << start << ' ' << end << std::endl;
  // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
  // by John Amanatides and Andrew Woo, 1987
  // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
  // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
  // Extensions to the described algorithm:
  //   • Imposed a distance limit.
  //   • The face passed through to reach the current cube is provided to
  //     the callback.

  // The foundation of this algorithm is a parameterized representation of
  // the provided ray,
  //                    origin + t * direction,
  // except that t is not actually stored; rather, at any given point in the
  // traversal, we keep track of the *greater* t values which we would have
  // if we took a step sufficient to cross a cube boundary along that axis
  // (i.e. change the integer part of the coordinate) in the variables
  // tMaxX, tMaxY, and tMaxZ.

  // 包含起点的体素坐标（向下取整获得体素索引）
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  // 包含终点的体素坐标
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  // 计算射线方向向量
  Eigen::Vector3d direction = (end - start);
  // 计算最大距离的平方（用于距离限制）
  double maxDist = direction.squaredNorm();

  // 分解方向向量为各个分量
  double dx = endX - x;
  double dy = endY - y;
  double dz = endZ - z;

  // 计算x,y,z各方向的步进方向（+1表示正方向，-1表示负方向，0表示不移动）
  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);
  int stepZ = (int)signum((int)dz);

  // 计算到达下一个体素边界的参数t值（初始值取决于起点的小数部分）
  // tMaxX: 沿X轴到达下一个体素边界所需的t值
  // tMaxY: 沿Y轴到达下一个体素边界所需的t值
  // tMaxZ: 沿Z轴到达下一个体素边界所需的t值
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  // 计算每步进一个体素时t的变化量（总是正数）
  // tDelta表示在每个方向上移动一个完整体素所需的t增量
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  // 避免无限循环：如果起点和终点在同一体素内，直接返回
  if (stepX == 0 && stepY == 0 && stepZ == 0) return;

  double dist = 0;
  // DDA主循环：逐体素遍历射线路径
  while (true) {
    // 检查当前体素是否在有效范围内
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
      // 将当前体素坐标添加到输出数组
      output[output_points_cnt](0) = x;
      output[output_points_cnt](1) = y;
      output[output_points_cnt](2) = z;

      output_points_cnt++;
      // 计算当前点到起点的欧氏距离
      dist = sqrt((x - start(0)) * (x - start(0)) + (y - start(1)) * (y - start(1)) +
                  (z - start(2)) * (z - start(2)));

      // 如果超过最大距离，停止遍历
      if (dist > maxDist) return;

      /*            if (output_points_cnt > 1500) {
                      std::cerr << "Error, too many racyast voxels." <<
         std::endl;
                      throw std::out_of_range("Too many raycast voxels");
                  }*/
    }

    // 如果到达终点体素，结束遍历
    if (x == endX && y == endY && z == endZ) break;

    // DDA核心步进逻辑：
    // tMaxX存储沿X轴穿越体素边界时的t值，Y和Z轴同理
    // 选择最小的tMax意味着选择最近的体素边界进行穿越
    // 这样确保按照射线路径顺序遍历所有相交的体素
    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        // X轴方向最近，更新X坐标
        x += stepX;
        // 调整tMaxX到下一个X轴方向的边界
        tMaxX += tDeltaX;
      } else {
        // Z轴方向最近，更新Z坐标
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        // Y轴方向最近，更新Y坐标
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        // Z轴方向最近，更新Z坐标
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }
}

/**
 * @brief 基于DDA的快速体素遍历算法（vector输出版本）
 * @param start 射线起点
 * @param end 射线终点
 * @param min 有效体素范围的最小边界
 * @param max 有效体素范围的最大边界
 * @param output 输出的体素坐标向量（使用std::vector存储）
 *
 * @note 基于论文 "A Fast Voxel Traversal Algorithm for Ray Tracing"
 *       作者: John Amanatides and Andrew Woo, 1987
 *       论文链接: http://www.cse.yorku.ca/~amana/research/grid.pdf
 *
 * 算法扩展:
 *   • 添加了距离限制
 *   • 提供了穿过当前体素所经过的面信息
 *
 * 算法原理:
 *   该算法基于射线的参数化表示: origin + t * direction
 *   但实际上不存储t值，而是在遍历过程中跟踪tMax值，
 *   即在每个轴上跨越体素边界（改变坐标整数部分）所需的较大t值
 *   这些值存储在变量 tMaxX, tMaxY, tMaxZ 中
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output) {
  //    std::cout << start << ' ' << end << std::endl;
  // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
  // by John Amanatides and Andrew Woo, 1987
  // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
  // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
  // Extensions to the described algorithm:
  //   • Imposed a distance limit.
  //   • The face passed through to reach the current cube is provided to
  //     the callback.

  // The foundation of this algorithm is a parameterized representation of
  // the provided ray,
  //                    origin + t * direction,
  // except that t is not actually stored; rather, at any given point in the
  // traversal, we keep track of the *greater* t values which we would have
  // if we took a step sufficient to cross a cube boundary along that axis
  // (i.e. change the integer part of the coordinate) in the variables
  // tMaxX, tMaxY, and tMaxZ.

  // 包含起点的体素坐标（向下取整获得体素索引）
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  // 包含终点的体素坐标
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  // 计算射线方向向量
  Eigen::Vector3d direction = (end - start);
  // 计算最大距离的平方（用于距离限制）
  double maxDist = direction.squaredNorm();

  // 分解方向向量为各个分量
  double dx = endX - x;
  double dy = endY - y;
  double dz = endZ - z;

  // 计算x,y,z各方向的步进方向（+1表示正方向，-1表示负方向，0表示不移动）
  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);
  int stepZ = (int)signum((int)dz);

  // 计算到达下一个体素边界的参数t值（初始值取决于起点的小数部分）
  // tMaxX: 沿X轴到达下一个体素边界所需的t值
  // tMaxY: 沿Y轴到达下一个体素边界所需的t值
  // tMaxZ: 沿Z轴到达下一个体素边界所需的t值
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  // 计算每步进一个体素时t的变化量（总是正数）
  // tDelta表示在每个方向上移动一个完整体素所需的t增量
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  // 清空输出向量
  output->clear();

  // 避免无限循环：如果起点和终点在同一体素内，直接返回
  if (stepX == 0 && stepY == 0 && stepZ == 0) return;

  double dist = 0;
  // DDA主循环：逐体素遍历射线路径
  while (true) {
    // 检查当前体素是否在有效范围内
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
      // 将当前体素坐标添加到输出向量
      output->push_back(Eigen::Vector3d(x, y, z));

      // 计算当前点到起点的平方距离
      dist = (Eigen::Vector3d(x, y, z) - start).squaredNorm();

      // 如果超过最大距离，停止遍历
      if (dist > maxDist) return;

      // 安全检查：防止输出体素数量过多
      if (output->size() > 1500) {
        std::cerr << "Error, too many racyast voxels." << std::endl;
        throw std::out_of_range("Too many raycast voxels");
      }
    }

    // 如果到达终点体素，结束遍历
    if (x == endX && y == endY && z == endZ) break;

    // DDA核心步进逻辑：
    // tMaxX存储沿X轴穿越体素边界时的t值，Y和Z轴同理
    // 选择最小的tMax意味着选择最近的体素边界进行穿越
    // 这样确保按照射线路径顺序遍历所有相交的体素
    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        // X轴方向最近，更新X坐标
        x += stepX;
        // 调整tMaxX到下一个X轴方向的边界
        tMaxX += tDeltaX;
      } else {
        // Z轴方向最近，更新Z坐标
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        // Y轴方向最近，更新Y坐标
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        // Z轴方向最近，更新Z坐标
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }
}

/**
 * @brief RayCaster类的初始化函数 - 设置射线投射的起点和终点
 * @param start 射线起点
 * @param end 射线终点
 * @return 如果起点和终点相同则返回false（避免无限循环），否则返回true
 *
 * @note 该函数初始化所有内部状态变量，准备进行增量式的射线遍历
 *       使用增量式方法允许逐步调用step()函数进行体素遍历
 */
bool RayCaster::setInput(const Eigen::Vector3d& start,
                         const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                         const Eigen::Vector3d& max */) {
  // 保存起点和终点
  start_ = start;
  end_ = end;
  // max_ = max;
  // min_ = min;

  // 计算包含起点的体素坐标
  x_ = (int)std::floor(start_.x());
  y_ = (int)std::floor(start_.y());
  z_ = (int)std::floor(start_.z());
  // 计算包含终点的体素坐标
  endX_ = (int)std::floor(end_.x());
  endY_ = (int)std::floor(end_.y());
  endZ_ = (int)std::floor(end_.z());
  // 计算射线方向向量
  direction_ = (end_ - start_);
  // 计算最大距离的平方
  maxDist_ = direction_.squaredNorm();

  // 分解方向向量为各个分量
  dx_ = endX_ - x_;
  dy_ = endY_ - y_;
  dz_ = endZ_ - z_;

  // 计算x,y,z各方向的步进方向
  stepX_ = (int)signum((int)dx_);
  stepY_ = (int)signum((int)dy_);
  stepZ_ = (int)signum((int)dz_);

  // 计算到达下一个体素边界的参数t值
  // 初始值取决于起点的小数部分
  tMaxX_ = intbound(start_.x(), dx_);
  tMaxY_ = intbound(start_.y(), dy_);
  tMaxZ_ = intbound(start_.z(), dz_);

  // 计算每步进一个体素时t的变化量（总是正数）
  tDeltaX_ = ((double)stepX_) / dx_;
  tDeltaY_ = ((double)stepY_) / dy_;
  tDeltaZ_ = ((double)stepZ_) / dz_;

  // 初始化距离计数器
  dist_ = 0;

  // 初始化步数计数器
  step_num_ = 0;

  // 避免无限循环：如果起点和终点在同一体素内，返回false
  if (stepX_ == 0 && stepY_ == 0 && stepZ_ == 0)
    return false;
  else
    return true;
}

/**
 * @brief RayCaster的单步遍历函数 - 移动到射线路径上的下一个体素
 * @param ray_pt 输出参数，返回当前体素的坐标
 * @return 如果还有下一个体素则返回true，如果已到达终点则返回false
 *
 * @note 该函数实现增量式的DDA遍历，每次调用移动到下一个体素
 *       通过比较tMaxX, tMaxY, tMaxZ来确定下一步应该沿哪个轴移动
 */
bool RayCaster::step(Eigen::Vector3d& ray_pt) {
  // if (x_ >= min_.x() && x_ < max_.x() && y_ >= min_.y() && y_ < max_.y() &&
  // z_ >= min_.z() && z_ <
  // max_.z())
  // 输出当前体素坐标
  ray_pt = Eigen::Vector3d(x_, y_, z_);

  // step_num_++;

  // dist_ = (Eigen::Vector3d(x_, y_, z_) - start_).squaredNorm();

  // 检查是否到达终点体素
  if (x_ == endX_ && y_ == endY_ && z_ == endZ_) {
    return false;
  }

  // if (dist_ > maxDist_)
  // {
  //   return false;
  // }

  // DDA核心步进逻辑：
  // tMaxX存储沿X轴穿越体素边界时的t值，Y和Z轴同理
  // 选择最小的tMax意味着选择最近的体素边界进行穿越
  // 这样确保按照射线路径顺序遍历所有相交的体素
  if (tMaxX_ < tMaxY_) {
    if (tMaxX_ < tMaxZ_) {
      // X轴方向最近，更新当前体素的X坐标
      x_ += stepX_;
      // 调整tMaxX到下一个X轴方向的边界
      tMaxX_ += tDeltaX_;
    } else {
      // Z轴方向最近，更新Z坐标
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  } else {
    if (tMaxY_ < tMaxZ_) {
      // Y轴方向最近，更新Y坐标
      y_ += stepY_;
      tMaxY_ += tDeltaY_;
    } else {
      // Z轴方向最近，更新Z坐标
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  }

  return true;
}