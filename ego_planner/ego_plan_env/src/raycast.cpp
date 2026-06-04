/**
 * @file raycast.cpp
 * @brief 基于体素的快速光线投射算法实现
 * @details 实现了Amanatides和Woo的快速体素遍历算法，用于在3D栅格地图中进行射线追踪
 *          主要用于碰撞检测、可见性检测等运动规划应用
 */

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <ego_plan_env/raycast.h>

/**
 * @brief 符号函数，返回整数的符号
 * @param x 输入整数
 * @return 如果x为0返回0，如果x为负返回-1，如果x为正返回1
 */
int signum(int x) {
  return x == 0 ? 0 : x < 0 ? -1 : 1;
}

/**
 * @brief 计算模运算（总是返回正值）
 * @param value 被除数
 * @param modulus 除数
 * @return 返回value对modulus的模，结果总是非负的
 * @details 与标准fmod不同，该函数确保结果总是在[0, modulus)范围内
 */
double mod(double value, double modulus) {
  return fmod(fmod(value, modulus) + modulus, modulus);
}

/**
 * @brief 计算射线到达下一个整数边界所需的参数t
 * @param s 当前位置坐标（可以是小数）
 * @param ds 射线方向在该轴上的分量
 * @return 返回使得s+t*ds为整数的最小正数t
 * @details 这个函数用于计算射线何时会穿过体素的边界
 *          如果ds为负，通过取反转换为正方向处理
 */
double intbound(double s, double ds) {
  // 找到最小的正数t，使得s+t*ds是整数
  if (ds < 0) {
    // 如果方向为负，转换为正方向计算
    return intbound(-s, -ds);
  } else {
    // 将s归一化到[0,1)区间
    s = mod(s, 1);
    // 现在问题变为求解 s+t*ds = 1
    return (1 - s) / ds;
  }
}

/**
 * @brief 快速体素遍历射线投射算法（数组输出版本）
 * @param start 射线起点（世界坐标）
 * @param end 射线终点（世界坐标）
 * @param min 有效体素范围的最小边界
 * @param max 有效体素范围的最大边界
 * @param output_points_cnt 输出点计数器（引用传递，函数内会递增）
 * @param output 输出数组，存储射线经过的体素索引
 *
 * @details 基于Amanatides和Woo的快速体素遍历算法（1987）
 *          参考文献：
 *          - http://www.cse.yorku.ca/~amana/research/grid.pdf
 *          - http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443
 *
 *          算法扩展：
 *          • 增加了距离限制
 *          • 只输出在有效范围[min, max)内的体素
 *
 *          算法原理：
 *          使用射线的参数化表示：origin + t * direction
 *          不直接存储t值，而是跟踪tMaxX、tMaxY、tMaxZ，
 *          它们表示射线在各轴上穿过下一个体素边界时的t值。
 *          每次选择最小的tMax对应的轴进行步进。
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output) {
  //    std::cout << start << ' ' << end << std::endl;
  // 来源："A Fast Voxel Traversal Algorithm for Ray Tracing"
  // 作者：John Amanatides and Andrew Woo, 1987
  // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
  // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
  // 对原始算法的扩展：
  //   • 施加了距离限制
  //   • 提供了到达当前立方体所穿过的面信息给回调函数

  // 该算法的基础是射线的参数化表示：
  //                    origin + t * direction,
  // 但实际上不存储t值；而是在遍历的任何给定点，
  // 我们跟踪更大的t值，这些值是我们在该轴上
  // 采取足够的步长以跨越立方体边界（即改变坐标的整数部分）时会得到的值
  // 这些值存储在变量tMaxX、tMaxY和tMaxZ中。

  // 包含起点的体素索引（向下取整）
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  // 包含终点的体素索引
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  // 射线方向向量
  Eigen::Vector3d direction = (end - start);
  // 最大距离的平方（用于距离判断）
  double maxDist = direction.squaredNorm();

  // 分解方向向量到各个轴
  double dx = endX - x;
  double dy = endY - y;
  double dz = endZ - z;

  // 步进方向：每个轴上的增量方向（-1, 0, 或 1）
  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);
  int stepZ = (int)signum((int)dz);

  // tMax表示射线参数t到达下一个体素边界的值
  // 初始值取决于起点的小数部分
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  // tDelta表示沿某个轴移动一个体素时t的变化量（总是正值）
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  // 避免无限循环：如果起点和终点在同一个体素中，直接返回
  if (stepX == 0 && stepY == 0 && stepZ == 0) return;

  double dist = 0;
  // 主循环：沿射线遍历体素
  while (true) {
    // 检查当前体素是否在有效范围内
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
      // 将当前体素索引存入输出数组
      output[output_points_cnt](0) = x;
      output[output_points_cnt](1) = y;
      output[output_points_cnt](2) = z;

      output_points_cnt++;
      // 计算当前点到起点的距离
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

    // tMaxX存储沿X轴穿过立方体边界时的t值，Y和Z类似
    // 因此，选择最小的tMax就是选择最近的立方体边界
    // 只有第一种情况有详细注释

    // 核心步进逻辑：选择tMax最小的轴进行步进
    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        // X轴的tMax最小，沿X轴步进
        // 更新当前所在的体素
        x += stepX;
        // 调整tMaxX到下一个X方向的边界穿越点
        tMaxX += tDeltaX;
      } else {
        // Z轴的tMax最小，沿Z轴步进
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        // Y轴的tMax最小，沿Y轴步进
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        // Z轴的tMax最小，沿Z轴步进
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }
}

/**
 * @brief 快速体素遍历射线投射算法（vector输出版本）
 * @param start 射线起点（世界坐标）
 * @param end 射线终点（世界坐标）
 * @param min 有效体素范围的最小边界
 * @param max 有效体素范围的最大边界
 * @param output 输出向量，存储射线经过的体素索引
 *
 * @details 与第一个版本功能相同，但使用std::vector作为输出容器
 *          包含额外的安全检查：限制最多1500个体素
 *          算法原理参见第一个Raycast函数的详细说明
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output) {
  //    std::cout << start << ' ' << end << std::endl;
  // 来源："A Fast Voxel Traversal Algorithm for Ray Tracing"
  // 作者：John Amanatides and Andrew Woo, 1987
  // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
  // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
  // 对原始算法的扩展：
  //   • 施加了距离限制
  //   • 提供了到达当前立方体所穿过的面信息给回调函数

  // 该算法的基础是射线的参数化表示：
  //                    origin + t * direction,
  // 但实际上不存储t值；而是在遍历的任何给定点，
  // 我们跟踪更大的t值，这些值是我们在该轴上
  // 采取足够的步长以跨越立方体边界（即改变坐标的整数部分）时会得到的值
  // 这些值存储在变量tMaxX、tMaxY和tMaxZ中。

  // 包含起点的体素索引（向下取整）
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  // 包含终点的体素索引
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  // 射线方向向量
  Eigen::Vector3d direction = (end - start);
  // 最大距离的平方（用于距离判断）
  double maxDist = direction.squaredNorm();

  // 分解方向向量到各个轴
  double dx = endX - x;
  double dy = endY - y;
  double dz = endZ - z;

  // 步进方向：每个轴上的增量方向（-1, 0, 或 1）
  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);
  int stepZ = (int)signum((int)dz);

  // tMax表示射线参数t到达下一个体素边界的值
  // 初始值取决于起点的小数部分
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  // tDelta表示沿某个轴移动一个体素时t的变化量（总是正值）
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  // 清空输出容器
  output->clear();

  // 避免无限循环：如果起点和终点在同一个体素中，直接返回
  if (stepX == 0 && stepY == 0 && stepZ == 0) return;

  double dist = 0;
  // 主循环：沿射线遍历体素
  while (true) {
    // 检查当前体素是否在有效范围内
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
      // 将当前体素索引添加到输出向量
      output->push_back(Eigen::Vector3d(x, y, z));

      // 计算当前点到起点的距离平方
      dist = (Eigen::Vector3d(x, y, z) - start).squaredNorm();

      // 如果超过最大距离，停止遍历
      if (dist > maxDist) return;

      // 安全检查：防止输出过多体素导致内存问题
      if (output->size() > 1500) {
        std::cerr << "Error, too many racyast voxels." << std::endl;
        throw std::out_of_range("Too many raycast voxels");
      }
    }

    // 如果到达终点体素，结束遍历
    if (x == endX && y == endY && z == endZ) break;

    // tMaxX存储沿X轴穿过立方体边界时的t值，Y和Z类似
    // 因此，选择最小的tMax就是选择最近的立方体边界
    // 只有第一种情况有详细注释

    // 核心步进逻辑：选择tMax最小的轴进行步进
    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        // X轴的tMax最小，沿X轴步进
        // 更新当前所在的体素
        x += stepX;
        // 调整tMaxX到下一个X方向的边界穿越点
        tMaxX += tDeltaX;
      } else {
        // Z轴的tMax最小，沿Z轴步进
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        // Y轴的tMax最小，沿Y轴步进
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        // Z轴的tMax最小，沿Z轴步进
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }
}

/**
 * @brief 设置射线投射器的输入参数并初始化
 * @param start 射线起点（世界坐标）
 * @param end 射线终点（世界坐标）
 * @return 如果输入有效返回true，如果起点和终点在同一体素返回false
 *
 * @details 这是RayCaster类的迭代式接口，允许逐步遍历射线经过的体素
 *          本函数初始化所有必要的内部状态变量，为后续的step()调用做准备
 */
bool RayCaster::setInput(const Eigen::Vector3d& start,
                         const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                         const Eigen::Vector3d& max */) {
  // 保存起点和终点
  start_ = start;
  end_ = end;
  // max_ = max;
  // min_ = min;

  // 计算包含起点的体素索引
  x_ = (int)std::floor(start_.x());
  y_ = (int)std::floor(start_.y());
  z_ = (int)std::floor(start_.z());
  // 计算包含终点的体素索引
  endX_ = (int)std::floor(end_.x());
  endY_ = (int)std::floor(end_.y());
  endZ_ = (int)std::floor(end_.z());
  // 计算射线方向向量
  direction_ = (end_ - start_);
  // 计算最大距离的平方
  maxDist_ = direction_.squaredNorm();

  // 分解方向向量到各个轴
  dx_ = endX_ - x_;
  dy_ = endY_ - y_;
  dz_ = endZ_ - z_;

  // 计算步进方向：每个轴上的增量方向（-1, 0, 或 1）
  stepX_ = (int)signum((int)dx_);
  stepY_ = (int)signum((int)dy_);
  stepZ_ = (int)signum((int)dz_);

  // tMax表示射线参数t到达下一个体素边界的值
  // 初始值取决于起点的小数部分
  tMaxX_ = intbound(start_.x(), dx_);
  tMaxY_ = intbound(start_.y(), dy_);
  tMaxZ_ = intbound(start_.z(), dz_);

  // tDelta表示沿某个轴移动一个体素时t的变化量（总是正值）
  tDeltaX_ = ((double)stepX_) / dx_;
  tDeltaY_ = ((double)stepY_) / dy_;
  tDeltaZ_ = ((double)stepZ_) / dz_;

  // 初始化距离为0
  dist_ = 0;

  // 初始化步数计数器
  step_num_ = 0;

  // 避免无限循环：如果起点和终点在同一个体素中，返回false
  if (stepX_ == 0 && stepY_ == 0 && stepZ_ == 0)
    return false;
  else
    return true;
}

/**
 * @brief 执行一步射线遍历，获取下一个体素
 * @param ray_pt 输出参数，返回当前体素的索引坐标
 * @return 如果还有下一个体素返回true，如果已到达终点返回false
 *
 * @details 每次调用step()会：
 *          1. 输出当前体素坐标到ray_pt
 *          2. 根据DDA算法计算并移动到下一个体素
 *          3. 如果到达终点体素，返回false
 *
 *          使用方法：
 *          RayCaster raycaster;
 *          raycaster.setInput(start, end);
 *          Eigen::Vector3d pt;
 *          while(raycaster.step(pt)) {
 *              // 处理体素pt
 *          }
 */
bool RayCaster::step(Eigen::Vector3d& ray_pt) {
  // if (x_ >= min_.x() && x_ < max_.x() && y_ >= min_.y() && y_ < max_.y() &&
  // z_ >= min_.z() && z_ <
  // max_.z())

  // 输出当前体素坐标
  ray_pt = Eigen::Vector3d(x_, y_, z_);

  // step_num_++;

  // dist_ = (Eigen::Vector3d(x_, y_, z_) - start_).squaredNorm();

  // 检查是否到达终点
  if (x_ == endX_ && y_ == endY_ && z_ == endZ_) {
    return false;
  }

  // if (dist_ > maxDist_)
  // {
  //   return false;
  // }

  // tMaxX存储沿X轴穿过立方体边界时的t值，Y和Z类似
  // 因此，选择最小的tMax就是选择最近的立方体边界
  // 只有第一种情况有详细注释

  // 核心步进逻辑：选择tMax最小的轴进行步进（DDA算法）
  if (tMaxX_ < tMaxY_) {
    if (tMaxX_ < tMaxZ_) {
      // X轴的tMax最小，沿X轴步进
      // 更新当前所在的体素
      x_ += stepX_;
      // 调整tMaxX到下一个X方向的边界穿越点
      tMaxX_ += tDeltaX_;
    } else {
      // Z轴的tMax最小，沿Z轴步进
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  } else {
    if (tMaxY_ < tMaxZ_) {
      // Y轴的tMax最小，沿Y轴步进
      y_ += stepY_;
      tMaxY_ += tDeltaY_;
    } else {
      // Z轴的tMax最小，沿Z轴步进
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  }

  // 返回true表示还有下一个体素
  return true;
}