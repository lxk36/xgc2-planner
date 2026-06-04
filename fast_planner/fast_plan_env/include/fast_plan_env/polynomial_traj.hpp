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
 *
 * 该类用于表示和操作分段多项式轨迹，支持三维空间中的轨迹规划。
 * 轨迹由多个多项式段组成，每段在x、y、z三个方向上分别用多项式表示。
 * 提供了轨迹求值、速度/加速度计算、轨迹评估等功能。
 */
class PolynomialTraj {
private:
  vector<double> times;        // 每段轨迹的持续时间
  vector<vector<double>> cxs;  // 每段轨迹x方向的多项式系数，从高阶到低阶排列
  vector<vector<double>> cys;  // 每段轨迹y方向的多项式系数，从高阶到低阶排列
  vector<vector<double>> czs;  // 每段轨迹z方向的多项式系数，从高阶到低阶排列

  double time_sum;  // 总时间（所有段的时间之和）
  int num_seg;      // 轨迹段的数量

  /* 轨迹评估相关 */
  vector<Eigen::Vector3d> traj_vec3d;  // 存储轨迹采样点的三维坐标序列
  double length;                        // 轨迹的总长度

public:
  /**
   * @brief 默认构造函数
   */
  PolynomialTraj(/* args */) {
  }

  /**
   * @brief 析构函数
   */
  ~PolynomialTraj() {
  }

  /**
   * @brief 重置轨迹
   *
   * 清空所有轨迹段的系数和时间信息，将轨迹恢复到初始状态。
   */
  void reset() {
    times.clear(), cxs.clear(), cys.clear(), czs.clear();
    time_sum = 0.0, num_seg = 0;
  }

  /**
   * @brief 添加一个轨迹段
   *
   * @param cx x方向的多项式系数向量（从高阶到低阶）
   * @param cy y方向的多项式系数向量（从高阶到低阶）
   * @param cz z方向的多项式系数向量（从高阶到低阶）
   * @param t 该段轨迹的持续时间
   */
  void addSegment(vector<double> cx, vector<double> cy, vector<double> cz, double t) {
    cxs.push_back(cx), cys.push_back(cy), czs.push_back(cz), times.push_back(t);
  }

  /**
   * @brief 初始化轨迹
   *
   * 计算轨迹段数量和总时间。在添加完所有轨迹段后必须调用此函数。
   */
  void init() {
    num_seg = times.size();
    time_sum = 0.0;
    for (int i = 0; i < times.size(); ++i) {
      time_sum += times[i];
    }
  }

  /**
   * @brief 计算指定时刻的轨迹位置
   *
   * @param t 从轨迹起点开始的时间
   * @return Eigen::Vector3d 该时刻的三维位置坐标
   */
  Eigen::Vector3d evaluate(double t) {
    /* 确定时刻t所在的轨迹段索引 */
    int idx = 0;
    while (times[idx] < t) {
      t -= times[idx];
      ++idx;
    }

    /* 计算该段轨迹在时刻t的位置 */
    int order = cxs[idx].size();  // 多项式阶数
    Eigen::VectorXd cx(order), cy(order), cz(order), tv(order);
    for (int i = 0; i < order; ++i) {
      cx(i) = cxs[idx][i], cy(i) = cys[idx][i], cz(i) = czs[idx][i];
      tv(order - 1 - i) = std::pow(t, double(i));  // 构造时间向量 [1, t, t^2, ..., t^(n-1)]
    }

    Eigen::Vector3d pt;
    pt(0) = tv.dot(cx), pt(1) = tv.dot(cy), pt(2) = tv.dot(cz);  // 多项式求值
    return pt;
  }

  /**
   * @brief 计算指定时刻的轨迹速度（位置的一阶导数）
   *
   * @param t 从轨迹起点开始的时间
   * @return Eigen::Vector3d 该时刻的三维速度向量
   */
  Eigen::Vector3d evaluateVel(double t) {
    /* 确定时刻t所在的轨迹段索引 */
    int idx = 0;
    while (times[idx] < t) {
      t -= times[idx];
      ++idx;
    }

    /* 计算速度多项式的系数 */
    int order = cxs[idx].size();
    Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);

    /* 通过对位置多项式求导得到速度多项式的系数 */
    for (int i = 0; i < order - 1; ++i) {
      vx(i) = double(i + 1) * cxs[idx][order - 2 - i];
      vy(i) = double(i + 1) * cys[idx][order - 2 - i];
      vz(i) = double(i + 1) * czs[idx][order - 2 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 1);
    for (int i = 0; i < order - 1; ++i)
      tv(i) = pow(ts, i);  // 构造时间向量 [1, t, t^2, ..., t^(n-2)]

    Eigen::Vector3d vel;
    vel(0) = tv.dot(vx), vel(1) = tv.dot(vy), vel(2) = tv.dot(vz);  // 速度多项式求值
    return vel;
  }

  /**
   * @brief 计算指定时刻的轨迹加速度（位置的二阶导数）
   *
   * @param t 从轨迹起点开始的时间
   * @return Eigen::Vector3d 该时刻的三维加速度向量
   */
  Eigen::Vector3d evaluateAcc(double t) {
    /* 确定时刻t所在的轨迹段索引 */
    int idx = 0;
    while (times[idx] < t) {
      t -= times[idx];
      ++idx;
    }

    /* 计算加速度多项式的系数 */
    int order = cxs[idx].size();
    Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

    /* 通过对位置多项式求二阶导得到加速度多项式的系数 */
    for (int i = 0; i < order - 2; ++i) {
      ax(i) = double((i + 2) * (i + 1)) * cxs[idx][order - 3 - i];
      ay(i) = double((i + 2) * (i + 1)) * cys[idx][order - 3 - i];
      az(i) = double((i + 2) * (i + 1)) * czs[idx][order - 3 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 2);
    for (int i = 0; i < order - 2; ++i)
      tv(i) = pow(ts, i);  // 构造时间向量 [1, t, t^2, ..., t^(n-3)]

    Eigen::Vector3d acc;
    acc(0) = tv.dot(ax), acc(1) = tv.dot(ay), acc(2) = tv.dot(az);  // 加速度多项式求值
    return acc;
  }

  /**
   * @brief 获取轨迹的总时间
   *
   * 注意：用于轨迹评估时应按顺序调用相关函数！
   *
   * @return double 轨迹的总持续时间
   */
  double getTimeSum() {
    return this->time_sum;
  }

  /**
   * @brief 获取轨迹的采样点序列
   *
   * 以0.01秒的时间间隔对整条轨迹进行采样，生成轨迹点序列。
   *
   * @return vector<Eigen::Vector3d> 轨迹采样点的三维坐标序列
   */
  vector<Eigen::Vector3d> getTraj() {
    double eval_t = 0.0;
    traj_vec3d.clear();
    while (eval_t < time_sum) {
      Eigen::Vector3d pt = evaluate(eval_t);
      traj_vec3d.push_back(pt);
      eval_t += 0.01;  // 采样间隔为0.01秒
    }
    return traj_vec3d;
  }

  /**
   * @brief 计算轨迹的总长度
   *
   * 通过累加相邻采样点之间的欧氏距离来估算轨迹长度。
   * 注意：需要先调用getTraj()生成采样点序列。
   *
   * @return double 轨迹的总长度
   */
  double getLength() {
    length = 0.0;

    Eigen::Vector3d p_l = traj_vec3d[0], p_n;  // p_l为上一个点，p_n为当前点
    for (int i = 1; i < traj_vec3d.size(); ++i) {
      p_n = traj_vec3d[i];
      length += (p_n - p_l).norm();  // 累加相邻点之间的距离
      p_l = p_n;
    }
    return length;
  }

  /**
   * @brief 计算轨迹的平均速度
   *
   * 平均速度 = 轨迹总长度 / 总时间
   * 注意：需要先调用getLength()计算轨迹长度。
   *
   * @return double 轨迹的平均速度
   */
  double getMeanVel() {
    double mean_vel = length / time_sum;
  }

  /**
   * @brief 计算轨迹的加速度代价
   *
   * 对于每个轨迹段，提取二阶项系数（与加速度相关）并计算其平方范数与时间的乘积之和。
   * 该代价函数用于评估轨迹的平滑性，加速度越小，轨迹越平滑。
   *
   * @return double 轨迹的加速度代价值
   */
  double getAccCost() {
    double cost = 0.0;
    int order = cxs[0].size();

    for (int s = 0; s < times.size(); ++s) {
      Eigen::Vector3d um;
      um(0) = 2 * cxs[s][order - 3], um(1) = 2 * cys[s][order - 3], um(2) = 2 * czs[s][order - 3];
      cost += um.squaredNorm() * times[s];  // 累加加速度的平方范数
    }

    return cost;
  }

  /**
   * @brief 计算轨迹的Jerk代价（加速度的导数）
   *
   * Jerk是加速度对时间的导数（三阶导数），反映了轨迹的平滑程度。
   * 通过构建Jerk矩阵并计算积分形式的代价函数来评估轨迹质量。
   * Jerk越小，轨迹越平滑，执行时对机械系统的冲击越小。
   *
   * @return double 轨迹的Jerk代价值
   */
  double getJerk() {
    double jerk = 0.0;

    /* 计算每个轨迹段的Jerk */
    for (int s = 0; s < times.size(); ++s) {
      Eigen::VectorXd cxv(cxs[s].size()), cyv(cys[s].size()), czv(czs[s].size());
      /* 转换系数顺序：从高阶到低阶转为从低阶到高阶 */
      int order = cxs[s].size();
      for (int j = 0; j < order; ++j) {
        cxv(j) = cxs[s][order - 1 - j], cyv(j) = cys[s][order - 1 - j], czv(j) = czs[s][order - 1 - j];
      }
      double ts = times[s];

      /* 构建Jerk矩阵（用于计算三阶导数的积分） */
      Eigen::MatrixXd mat_jerk(order, order);
      mat_jerk.setZero();
      for (double i = 3; i < order; i += 1)
        for (double j = 3; j < order; j += 1) {
          // Jerk积分的解析形式：∫(d³p/dt³)² dt
          mat_jerk(i, j) =
              i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) * pow(ts, i + j - 5) / (i + j - 5);
        }

      // 计算x、y、z三个方向的Jerk并累加
      jerk += (cxv.transpose() * mat_jerk * cxv)(0, 0);
      jerk += (cyv.transpose() * mat_jerk * cyv)(0, 0);
      jerk += (czv.transpose() * mat_jerk * czv)(0, 0);
    }

    return jerk;
  }

  /**
   * @brief 计算轨迹的平均速度和最大速度
   *
   * 对每个轨迹段以0.01秒间隔采样，计算各采样点的速度大小，
   * 统计所有采样点的平均速度和最大速度。
   *
   * @param mean_v 输出参数，返回平均速度
   * @param max_v 输出参数，返回最大速度
   */
  void getMeanAndMaxVel(double& mean_v, double& max_v) {
    int num = 0;  // 采样点计数
    mean_v = 0.0, max_v = -1.0;
    for (int s = 0; s < times.size(); ++s) {
      int order = cxs[s].size();
      Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);

      /* 计算速度多项式的系数 */
      for (int i = 0; i < order - 1; ++i) {
        vx(i) = double(i + 1) * cxs[s][order - 2 - i];
        vy(i) = double(i + 1) * cys[s][order - 2 - i];
        vz(i) = double(i + 1) * czs[s][order - 2 - i];
      }
      double ts = times[s];

      double eval_t = 0.0;
      while (eval_t < ts) {
        Eigen::VectorXd tv(order - 1);
        for (int i = 0; i < order - 1; ++i)
          tv(i) = pow(ts, i);
        Eigen::Vector3d vel;
        vel(0) = tv.dot(vx), vel(1) = tv.dot(vy), vel(2) = tv.dot(vz);
        double vn = vel.norm();  // 速度的大小
        mean_v += vn;
        if (vn > max_v) max_v = vn;  // 更新最大速度
        ++num;

        eval_t += 0.01;  // 采样间隔为0.01秒
      }
    }

    mean_v = mean_v / double(num);  // 计算平均值
  }

  /**
   * @brief 计算轨迹的平均加速度和最大加速度
   *
   * 对每个轨迹段以0.01秒间隔采样，计算各采样点的加速度大小，
   * 统计所有采样点的平均加速度和最大加速度。
   *
   * @param mean_a 输出参数，返回平均加速度
   * @param max_a 输出参数，返回最大加速度
   */
  void getMeanAndMaxAcc(double& mean_a, double& max_a) {
    int num = 0;  // 采样点计数
    mean_a = 0.0, max_a = -1.0;
    for (int s = 0; s < times.size(); ++s) {
      int order = cxs[s].size();
      Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

      /* 计算加速度多项式的系数 */
      for (int i = 0; i < order - 2; ++i) {
        ax(i) = double((i + 2) * (i + 1)) * cxs[s][order - 3 - i];
        ay(i) = double((i + 2) * (i + 1)) * cys[s][order - 3 - i];
        az(i) = double((i + 2) * (i + 1)) * czs[s][order - 3 - i];
      }
      double ts = times[s];

      double eval_t = 0.0;
      while (eval_t < ts) {
        Eigen::VectorXd tv(order - 2);
        for (int i = 0; i < order - 2; ++i)
          tv(i) = pow(ts, i);
        Eigen::Vector3d acc;
        acc(0) = tv.dot(ax), acc(1) = tv.dot(ay), acc(2) = tv.dot(az);
        double an = acc.norm();  // 加速度的大小
        mean_a += an;
        if (an > max_a) max_a = an;  // 更新最大加速度
        ++num;

        eval_t += 0.01;  // 采样间隔为0.01秒
      }
    }

    mean_a = mean_a / double(num);  // 计算平均值
  }
};

#endif