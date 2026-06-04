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



#ifndef _POLYNOMIAL_TRAJ_H
#define _POLYNOMIAL_TRAJ_H

#include <Eigen/Eigen>
#include <vector>

using std::vector;

/**
 * @brief 多项式轨迹类
 * @details 用于表示和操作分段多项式轨迹，每段轨迹由多项式系数和时间长度定义
 *          支持位置、速度、加速度的求值，以及轨迹质量评估（长度、Jerk、加速度代价等）
 */
class PolynomialTraj {
private:
  vector<double> times;        // 每段轨迹的时间长度
  vector<vector<double>> cxs;  // 每段轨迹在x方向的多项式系数，从高次到低次排列 (n-1 -> 0)
  vector<vector<double>> cys;  // 每段轨迹在y方向的多项式系数，从高次到低次排列
  vector<vector<double>> czs;  // 每段轨迹在z方向的多项式系数，从高次到低次排列

  double time_sum;  // 所有轨迹段的总时间
  int num_seg;      // 轨迹段的数量

  /* 轨迹评估相关变量 */
  vector<Eigen::Vector3d> traj_vec3d;  // 离散化后的轨迹点序列
  double length;                        // 轨迹总长度

public:
  /**
   * @brief 构造函数
   */
  PolynomialTraj(/* args */) {
  }

  /**
   * @brief 析构函数
   */
  ~PolynomialTraj() {
  }

  /**
   * @brief 重置轨迹，清除所有数据
   */
  void reset() {
    times.clear(), cxs.clear(), cys.clear(), czs.clear();
    time_sum = 0.0, num_seg = 0;
  }

  /**
   * @brief 添加一段轨迹
   * @param cx x方向的多项式系数向量
   * @param cy y方向的多项式系数向量
   * @param cz z方向的多项式系数向量
   * @param t 该段轨迹的时间长度
   */
  void addSegment(vector<double> cx, vector<double> cy, vector<double> cz, double t) {
    cxs.push_back(cx), cys.push_back(cy), czs.push_back(cz), times.push_back(t);
  }

  /**
   * @brief 初始化轨迹，计算总时间和段数
   */
  void init() {
    num_seg = times.size();
    time_sum = 0.0;
    // 累加所有段的时间得到总时间
    for (int i = 0; i < times.size(); ++i) {
      time_sum += times[i];
    }
  }

  /**
   * @brief 在给定时刻t求值轨迹位置
   * @param t 从轨迹起点开始的时间
   * @return 三维位置坐标
   */
  Eigen::Vector3d evaluate(double t) {
    /* 确定时刻t所在的轨迹段索引 */
    int idx = 0;
    while (times[idx] + 1e-4 < t) {
      t -= times[idx];  // 将t转换为相对于当前段起点的时间
      ++idx;
    }

    /* 计算位置：p(t) = c0*t^0 + c1*t^1 + ... + cn*t^n */
    int order = cxs[idx].size();  // 多项式阶数
    Eigen::VectorXd cx(order), cy(order), cz(order), tv(order);
    for (int i = 0; i < order; ++i) {
      cx(i) = cxs[idx][i], cy(i) = cys[idx][i], cz(i) = czs[idx][i];
      tv(order - 1 - i) = std::pow(t, double(i));  // 时间向量：[t^(n-1), t^(n-2), ..., t^1, t^0]
    }

    // 通过内积计算位置：系数向量与时间幂次向量的点积
    Eigen::Vector3d pt;
    pt(0) = tv.dot(cx), pt(1) = tv.dot(cy), pt(2) = tv.dot(cz);
    return pt;
  }

  /**
   * @brief 在给定时刻t求值轨迹速度
   * @param t 从轨迹起点开始的时间
   * @return 三维速度向量
   * @details 速度是位置的一阶导数：v(t) = dp/dt
   */
  Eigen::Vector3d evaluateVel(double t) {
    /* 确定时刻t所在的轨迹段索引 */
    int idx = 0;
    while (times[idx] + 1e-4 < t) {
      t -= times[idx];
      ++idx;
    }

    /* 计算速度多项式 */
    int order = cxs[idx].size();
    Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);

    /* 计算速度多项式系数：对位置多项式求导
     * 若 p(t) = c_n*t^n + ... + c_1*t + c_0
     * 则 v(t) = n*c_n*t^(n-1) + ... + c_1
     */
    for (int i = 0; i < order - 1; ++i) {
      vx(i) = double(i + 1) * cxs[idx][order - 2 - i];
      vy(i) = double(i + 1) * cys[idx][order - 2 - i];
      vz(i) = double(i + 1) * czs[idx][order - 2 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 1);
    // 构建时间幂次向量：[t^0, t^1, ..., t^(n-2)]
    for (int i = 0; i < order - 1; ++i)
      tv(i) = pow(ts, i);

    // 通过内积计算速度
    Eigen::Vector3d vel;
    vel(0) = tv.dot(vx), vel(1) = tv.dot(vy), vel(2) = tv.dot(vz);
    return vel;
  }

  /**
   * @brief 在给定时刻t求值轨迹加速度
   * @param t 从轨迹起点开始的时间
   * @return 三维加速度向量
   * @details 加速度是位置的二阶导数：a(t) = d^2p/dt^2
   */
  Eigen::Vector3d evaluateAcc(double t) {
    /* 确定时刻t所在的轨迹段索引 */
    int idx = 0;
    while (times[idx] + 1e-4 < t) {
      t -= times[idx];
      ++idx;
    }

    /* 计算加速度多项式 */
    int order = cxs[idx].size();
    Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

    /* 计算加速度多项式系数：对位置多项式求二阶导数
     * 若 p(t) = c_n*t^n + ... + c_2*t^2 + c_1*t + c_0
     * 则 a(t) = n*(n-1)*c_n*t^(n-2) + ... + 2*c_2
     */
    for (int i = 0; i < order - 2; ++i) {
      ax(i) = double((i + 2) * (i + 1)) * cxs[idx][order - 3 - i];
      ay(i) = double((i + 2) * (i + 1)) * cys[idx][order - 3 - i];
      az(i) = double((i + 2) * (i + 1)) * czs[idx][order - 3 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 2);
    // 构建时间幂次向量：[t^0, t^1, ..., t^(n-3)]
    for (int i = 0; i < order - 2; ++i)
      tv(i) = pow(ts, i);

    // 通过内积计算加速度
    Eigen::Vector3d acc;
    acc(0) = tv.dot(ax), acc(1) = tv.dot(ay), acc(2) = tv.dot(az);
    return acc;
  }

  /**
   * @brief 获取轨迹总时间
   * @return 所有轨迹段的时间之和
   * @note 用于评估轨迹时，应该按顺序调用!!!
   */
  double getTimeSum() {
    return this->time_sum;
  }

  /**
   * @brief 获取离散化的轨迹点序列
   * @return 轨迹点的向量
   * @details 以0.01秒为间隔对整条轨迹进行采样
   */
  vector<Eigen::Vector3d> getTraj() {
    double eval_t = 0.0;
    traj_vec3d.clear();
    // 从起点到终点，每隔0.01秒采样一次
    while (eval_t < time_sum) {
      Eigen::Vector3d pt = evaluate(eval_t);
      traj_vec3d.push_back(pt);
      eval_t += 0.01;
    }
    return traj_vec3d;
  }

  /**
   * @brief 计算轨迹总长度
   * @return 轨迹的空间长度
   * @details 通过累加离散轨迹点之间的欧氏距离来近似计算
   */
  double getLength() {
    length = 0.0;

    Eigen::Vector3d p_l = traj_vec3d[0], p_n;  // p_l: 上一个点, p_n: 当前点
    // 遍历所有离散点，累加相邻点之间的距离
    for (int i = 1; i < traj_vec3d.size(); ++i) {
      p_n = traj_vec3d[i];
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  /**
   * @brief 获取平均速度
   * @return 平均速度大小
   * @details 通过轨迹总长度除以总时间计算
   */
  double getMeanVel() {
    double mean_vel = length / time_sum;
  }

  /**
   * @brief 计算加速度代价
   * @return 加速度的平方积分（加速度代价）
   * @details 通过每段轨迹的二次项系数来近似计算加速度的平方积分
   *          这是轨迹优化中常用的平滑性度量
   */
  double getAccCost() {
    double cost = 0.0;
    int order = cxs[0].size();

    // 遍历每一段轨迹
    for (int s = 0; s < times.size(); ++s) {
      Eigen::Vector3d um;
      // 提取二次项系数（order-3位置），乘以2得到加速度的常数项
      um(0) = 2 * cxs[s][order - 3], um(1) = 2 * cys[s][order - 3], um(2) = 2 * czs[s][order - 3];
      // 累加加速度的平方乘以时间长度
      cost += um.squaredNorm() * times[s];
    }

    return cost;
  }

  /**
   * @brief 计算Jerk代价（加加速度代价）
   * @return Jerk的平方积分
   * @details Jerk是加速度的导数（位置的三阶导数），其平方积分反映轨迹的平滑程度
   *          通过构建Jerk矩阵并计算二次型来得到解析解
   */
  double getJerk() {
    double jerk = 0.0;

    /* 计算每段轨迹的Jerk代价 */
    for (int s = 0; s < times.size(); ++s) {
      Eigen::VectorXd cxv(cxs[s].size()), cyv(cys[s].size()), czv(czs[s].size());
      /* 转换系数向量：从高次到低次改为从低次到高次 */
      int order = cxs[s].size();
      for (int j = 0; j < order; ++j) {
        cxv(j) = cxs[s][order - 1 - j], cyv(j) = cys[s][order - 1 - j], czv(j) = czs[s][order - 1 - j];
      }
      double ts = times[s];

      /* 构建Jerk积分矩阵
       * Jerk = d^3p/dt^3，对于多项式 p(t) = sum(c_i * t^i)
       * Jerk(t) = sum(i*(i-1)*(i-2)*c_i*t^(i-3))
       * 积分 int[Jerk^2 dt] = c^T * M * c，其中M是Jerk积分矩阵
       */
      Eigen::MatrixXd mat_jerk(order, order);
      mat_jerk.setZero();
      // 只有三次及以上的项对Jerk有贡献
      for (double i = 3; i < order; i += 1)
        for (double j = 3; j < order; j += 1) {
          // Jerk积分矩阵元素的解析公式
          mat_jerk(i, j) =
              i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) * pow(ts, i + j - 5) / (i + j - 5);
        }

      // 计算x, y, z三个方向的Jerk代价并累加
      jerk += (cxv.transpose() * mat_jerk * cxv)(0, 0);
      jerk += (cyv.transpose() * mat_jerk * cyv)(0, 0);
      jerk += (czv.transpose() * mat_jerk * czv)(0, 0);
    }

    return jerk;
  }

  /**
   * @brief 计算平均速度和最大速度
   * @param mean_v 输出参数：平均速度大小
   * @param max_v 输出参数：最大速度大小
   * @details 通过在每段轨迹上以0.01秒间隔采样速度来统计
   */
  void getMeanAndMaxVel(double& mean_v, double& max_v) {
    int num = 0;  // 采样点计数
    mean_v = 0.0, max_v = -1.0;
    // 遍历每一段轨迹
    for (int s = 0; s < times.size(); ++s) {
      int order = cxs[s].size();
      Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);

      /* 计算速度多项式系数 */
      for (int i = 0; i < order - 1; ++i) {
        vx(i) = double(i + 1) * cxs[s][order - 2 - i];
        vy(i) = double(i + 1) * cys[s][order - 2 - i];
        vz(i) = double(i + 1) * czs[s][order - 2 - i];
      }
      double ts = times[s];

      // 在当前段上以0.01秒间隔采样
      double eval_t = 0.0;
      while (eval_t < ts) {
        Eigen::VectorXd tv(order - 1);
        for (int i = 0; i < order - 1; ++i)
          tv(i) = pow(ts, i);
        Eigen::Vector3d vel;
        vel(0) = tv.dot(vx), vel(1) = tv.dot(vy), vel(2) = tv.dot(vz);
        double vn = vel.norm();  // 速度大小
        mean_v += vn;            // 累加用于计算平均值
        if (vn > max_v) max_v = vn;  // 更新最大值
        ++num;

        eval_t += 0.01;
      }
    }

    mean_v = mean_v / double(num);  // 计算平均速度
  }

  /**
   * @brief 计算平均加速度和最大加速度
   * @param mean_a 输出参数：平均加速度大小
   * @param max_a 输出参数：最大加速度大小
   * @details 通过在每段轨迹上以0.01秒间隔采样加速度来统计
   */
  void getMeanAndMaxAcc(double& mean_a, double& max_a) {
    int num = 0;  // 采样点计数
    mean_a = 0.0, max_a = -1.0;
    // 遍历每一段轨迹
    for (int s = 0; s < times.size(); ++s) {
      int order = cxs[s].size();
      Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

      /* 计算加速度多项式系数 */
      for (int i = 0; i < order - 2; ++i) {
        ax(i) = double((i + 2) * (i + 1)) * cxs[s][order - 3 - i];
        ay(i) = double((i + 2) * (i + 1)) * cys[s][order - 3 - i];
        az(i) = double((i + 2) * (i + 1)) * czs[s][order - 3 - i];
      }
      double ts = times[s];

      // 在当前段上以0.01秒间隔采样
      double eval_t = 0.0;
      while (eval_t < ts) {
        Eigen::VectorXd tv(order - 2);
        for (int i = 0; i < order - 2; ++i)
          tv(i) = pow(ts, i);
        Eigen::Vector3d acc;
        acc(0) = tv.dot(ax), acc(1) = tv.dot(ay), acc(2) = tv.dot(az);
        double an = acc.norm();  // 加速度大小
        mean_a += an;            // 累加用于计算平均值
        if (an > max_a) max_a = an;  // 更新最大值
        ++num;

        eval_t += 0.01;
      }
    }

    mean_a = mean_a / double(num);  // 计算平均加速度
  }
};

/**
 * @brief 生成最小Snap轨迹
 * @param Pos 路径点矩阵，大小为Nx3（N个三维路径点）
 * @param start_vel 起始速度
 * @param end_vel 终止速度
 * @param start_acc 起始加速度
 * @param end_acc 终止加速度
 * @param Time 每段轨迹的时间分配
 * @return 生成的多项式轨迹
 * @details 最小Snap轨迹是指最小化Snap（加速度的二阶导数）的轨迹，
 *          这种轨迹具有良好的平滑性，适合四旋翼等飞行器
 */
PolynomialTraj minSnapTraj(const Eigen::MatrixXd& Pos, const Eigen::Vector3d& start_vel,
                           const Eigen::Vector3d& end_vel, const Eigen::Vector3d& start_acc,
                           const Eigen::Vector3d& end_acc, const Eigen::VectorXd& Time);

/**
 * @brief 生成快速直线轨迹（4阶多项式）
 * @param start 起始位置
 * @param end 终止位置
 * @param max_vel 最大速度约束
 * @param max_acc 最大加速度约束
 * @param max_jerk 最大Jerk约束
 * @return 生成的多项式轨迹
 * @details 在起点和终点之间生成一条满足速度、加速度和Jerk约束的直线轨迹
 */
PolynomialTraj fastLine4deg(Eigen::Vector3d start, Eigen::Vector3d end, double max_vel, double max_acc,
                            double max_jerk);

/**
 * @brief 生成快速直线轨迹（3阶多项式）
 * @param start 起始位置
 * @param end 终止位置
 * @param max_vel 最大速度约束
 * @param max_acc 最大加速度约束
 * @return 生成的多项式轨迹
 * @details 在起点和终点之间生成一条满足速度和加速度约束的直线轨迹
 */
PolynomialTraj fastLine3deg(Eigen::Vector3d start, Eigen::Vector3d end, double max_vel, double max_acc);

#endif