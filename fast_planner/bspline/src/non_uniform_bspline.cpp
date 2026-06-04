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



#include "bspline/non_uniform_bspline.h"
#include <ros/ros.h>

namespace fast_planner {

/**
 * @brief 非均匀B样条构造函数
 * @param points 控制点矩阵，每行代表一个控制点
 * @param order B样条的阶数(p)，阶数为3时表示3次B样条
 * @param interval 节点向量的时间间隔
 */
NonUniformBspline::NonUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                     const double& interval) {
  setUniformBspline(points, order, interval);
}

/**
 * @brief 非均匀B样条析构函数
 */
NonUniformBspline::~NonUniformBspline() {}

/**
 * @brief 设置均匀B样条的参数，并初始化节点向量
 * @param points 控制点矩阵
 * @param order B样条阶数
 * @param interval 节点间隔
 * @note 对于均匀B样条，节点向量按照固定间隔均匀分布
 *       节点向量公式: u = {u_0, u_1, ..., u_m}，其中 m = n + p + 1
 */
void NonUniformBspline::setUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                          const double& interval) {
  control_points_ = points;  // 存储控制点
  p_              = order;   // 存储B样条阶数
  interval_       = interval; // 存储节点间隔

  n_ = points.rows() - 1;  // n为控制点的最大索引(控制点数量-1)
  m_ = n_ + p_ + 1;        // m为节点向量的最大索引，满足 m = n + p + 1

  // 初始化节点向量，长度为 m + 1
  // 节点向量的结构：[u_0, ..., u_p] | [u_{p+1}, ..., u_{m-p}] | [u_{m-p+1}, ..., u_m]
  //                  起始夹紧节点   |    均匀中间节点      |    终止夹紧节点
  u_ = Eigen::VectorXd::Zero(m_ + 1);
  for (int i = 0; i <= m_; ++i) {

    if (i <= p_) {
      // 前p+1个节点：从负值开始，用于夹紧B样条起点
      // 这使得曲线通过第一个控制点，并在起点处具有良好的导数性质
      u_(i) = double(-p_ + i) * interval_;
    } else if (i > p_ && i <= m_ - p_) {
      // 中间节点：均匀递增，间隔为interval_
      // 这是均匀B样条的核心特征，保证了参数化的均匀性
      u_(i) = u_(i - 1) + interval_;
    } else if (i > m_ - p_) {
      // 后p个节点：均匀递增，用于夹紧B样条终点
      // 这使得曲线通过最后一个控制点，并在终点处具有良好的导数性质
      u_(i) = u_(i - 1) + interval_;
    }
  }
}

/**
 * @brief 设置节点向量
 * @param knot 新的节点向量
 */
void NonUniformBspline::setKnot(const Eigen::VectorXd& knot) { this->u_ = knot; }

/**
 * @brief 获取节点向量
 * @return 节点向量
 */
Eigen::VectorXd NonUniformBspline::getKnot() { return this->u_; }

/**
 * @brief 获取B样条的有效时间范围
 * @param um 起始时间，对应节点 u(p)
 * @param um_p 结束时间，对应节点 u(m-p)
 * @note B样条曲线的有效参数范围为 [u(p), u(m-p)]
 */
void NonUniformBspline::getTimeSpan(double& um, double& um_p) {
  um   = u_(p_);       // 有效起始时间
  um_p = u_(m_ - p_);  // 有效结束时间
}

/**
 * @brief 获取控制点矩阵
 * @return 控制点矩阵
 */
Eigen::MatrixXd NonUniformBspline::getControlPoint() { return control_points_; }

/**
 * @brief 获取B样条曲线的起点和终点
 * @return 包含起点和终点的pair
 * @note 起点在时间 u(p) 处，终点在时间 u(m-p) 处
 */
pair<Eigen::VectorXd, Eigen::VectorXd> NonUniformBspline::getHeadTailPts() {
  Eigen::VectorXd head = evaluateDeBoor(u_(p_));      // 计算起点
  Eigen::VectorXd tail = evaluateDeBoor(u_(m_ - p_)); // 计算终点
  return make_pair(head, tail);
}

/**
 * @brief 使用deBoor算法计算B样条曲线在给定参数u处的点
 * @param u 参数值
 * @return 曲线上对应点的坐标
 * @note deBoor算法是计算B样条曲线的标准算法，通过递推方式计算
 *       时间复杂度为 O(p^2)，其中p为B样条阶数
 */
Eigen::VectorXd NonUniformBspline::evaluateDeBoor(const double& u) {

  // 将参数u限制在有效范围 [u(p), u(m-p)] 内
  // u(p)是曲线起始参数，u(m-p)是曲线终止参数
  // 限制参数范围是为了避免访问无效的控制点
  double ub = min(max(u_(p_), u), u_(m_ - p_));

  // 确定参数ub所在的节点区间 [u_k, u_{k+1}]
  // k是区间左端点的索引，满足 u_k ≤ ub < u_{k+1}
  // 从k=p开始搜索，因为有效区间从u_p开始
  int k = p_;
  while (true) {
    if (u_(k + 1) >= ub) break;
    ++k;
  }

  /* deBoor递推算法 */
  // 初始化：选择p+1个相关控制点 P_{k-p}, P_{k-p+1}, ..., P_k
  // 这些控制点对参数ub所在区间 [u_k, u_{k+1}] 有非零影响
  vector<Eigen::VectorXd> d;
  for (int i = 0; i <= p_; ++i) {
    d.push_back(control_points_.row(k - p_ + i));
    // cout << d[i].transpose() << endl;
  }

  // 递推计算：进行p次迭代，每次迭代降低一阶多项式
  // r表示当前迭代层次，从1到p
  for (int r = 1; r <= p_; ++r) {
    // 从后向前更新点，避免覆盖未使用的值
    for (int i = p_; i >= r; --i) {
      // 计算混合系数alpha，基于Cox-de Boor递推公式
      // alpha = (u - u_{i+k-p}) / (u_{i+1+k-r} - u_{i+k-p})
      double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
      // cout << "alpha: " << alpha << endl;
      // 线性插值：d[i]^(r) = (1-alpha)*d[i-1]^(r-1) + alpha*d[i]^(r-1)
      // 这是Cox-de Boor递推公式的核心，通过凸组合逐步求精
      d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
    }
  }

  // 返回最终计算结果 d[p]^(p)，即B样条曲线在参数ub处的点
  return d[p_];
}

/**
 * @brief 使用相对时间t计算B样条曲线上的点
 * @param t 相对时间，从0开始
 * @return 曲线上对应点的坐标
 * @note 将相对时间t转换为绝对参数(t + u(p))后调用evaluateDeBoor
 */
Eigen::VectorXd NonUniformBspline::evaluateDeBoorT(const double& t) {
  return evaluateDeBoor(t + u_(p_));
}

/**
 * @brief 计算B样条导数的控制点
 * @return 导数B样条的控制点矩阵
 * @note B样条的导数仍然是B样条，但阶数降为p-1
 *       导数控制点公式: Qi = p*(P_{i+1}-P_i)/(u_{i+p+1}-u_{i+1})
 */
Eigen::MatrixXd NonUniformBspline::getDerivativeControlPoints() {
  // B样条的导数也是B样条，阶数变为 p_-1
  // 导数控制点公式: Q_i = p*(P_{i+1}-P_i)/(u_{i+p+1}-u_{i+1})
  // 这是基于B样条求导的基本性质：C'(u) = Σ Q_i * N_{i,p-1}(u)
  // 其中导数控制点的数量比原控制点少1个
  Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(control_points_.rows() - 1, control_points_.cols());
  for (int i = 0; i < ctp.rows(); ++i) {
    // 对每个导数控制点，根据相邻原控制点的差分计算
    // 除以相应的节点区间长度，得到归一化的导数
    ctp.row(i) =
        p_ * (control_points_.row(i + 1) - control_points_.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
  }
  return ctp;
}

/**
 * @brief 获取B样条曲线的导数曲线
 * @return 导数B样条对象
 * @note 导数B样条的阶数为p-1，节点向量需要去掉首尾节点
 */
NonUniformBspline NonUniformBspline::getDerivative() {
  Eigen::MatrixXd   ctp = getDerivativeControlPoints();  // 计算导数控制点
  NonUniformBspline derivative(ctp, p_ - 1, interval_);  // 创建导数B样条，阶数降为p-1

  /* 去掉首尾节点 */
  // B样条求导后，节点向量需要去掉首尾各一个节点
  // 原节点向量: [u_0, u_1, ..., u_m]
  // 导数节点向量: [u_1, u_2, ..., u_{m-1}]
  Eigen::VectorXd knot(u_.rows() - 2);
  knot = u_.segment(1, u_.rows() - 2);  // 提取中间的节点（去掉首尾）
  derivative.setKnot(knot);              // 设置导数B样条的节点向量

  return derivative;
}

/**
 * @brief 获取节点间隔
 * @return 节点间隔值
 */
double NonUniformBspline::getInterval() { return interval_; }

/**
 * @brief 设置物理限制参数（速度和加速度）
 * @param vel 速度限制
 * @param acc 加速度限制
 */
void NonUniformBspline::setPhysicalLimits(const double& vel, const double& acc) {
  limit_vel_   = vel;  // 速度限制
  limit_acc_   = acc;  // 加速度限制
  limit_ratio_ = 1.1;  // 限制比例因子
}

/**
 * @brief 检查B样条轨迹是否满足物理约束（速度和加速度限制）
 * @param show 是否显示不满足约束的详细信息
 * @return true表示满足所有约束，false表示存在违反约束的情况
 * @note 检查所有控制点对应的速度和加速度是否在限制范围内
 */
bool NonUniformBspline::checkFeasibility(bool show) {
  bool fea = true;  // 可行性标志
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;

  Eigen::MatrixXd P         = control_points_;        // 控制点矩阵
  int             dimension = control_points_.cols(); // 空间维度

  /* 检查速度可行性 */
  // 通过导数控制点检查速度约束
  // 由于B样条的局部性，每段速度主要由相邻控制点决定
  double max_vel = -1.0;  // 记录最大速度
  for (int i = 0; i < P.rows() - 1; ++i) {
    // 计算第i个导数控制点，代表该段的速度特性
    // vel_i = p*(P_{i+1}-P_i)/(u_{i+p+1}-u_{i+1})
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    // 检查各维度速度是否超过限制（加1e-4容差以避免数值误差）
    if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
        fabs(vel(2)) > limit_vel_ + 1e-4) {

      if (show) cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
      fea = false;

      // 更新最大速度，用于后续计算违反比例
      for (int j = 0; j < dimension; ++j) {
        max_vel = max(max_vel, fabs(vel(j)));
      }
    }
  }

  /* 检查加速度可行性 */
  // 通过二阶导数控制点检查加速度约束
  double max_acc = -1.0;  // 记录最大加速度
  for (int i = 0; i < P.rows() - 2; ++i) {

    // 计算第i个二阶导数控制点，代表该段的加速度特性
    // 加速度是速度的导数，因此需要对速度控制点再求导
    // acc_i = p*(p-1)*[vel_{i+1}-vel_i]/(u_{i+p+1}-u_{i+2})
    // 其中 vel_j = p*(P_{j+1}-P_j)/(u_{j+p+1}-u_{j+1})
    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    // 检查各维度加速度是否超过限制（加1e-4容差以避免数值误差）
    if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
        fabs(acc(2)) > limit_acc_ + 1e-4) {

      if (show) cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
      fea = false;

      // 更新最大加速度，用于后续计算违反比例
      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)));
      }
    }
  }

  // 计算速度和加速度的违反比例
  // 速度比例：max_vel / limit_vel
  // 加速度比例：sqrt(max_acc / limit_acc)，用平方根是因为时间缩放对加速度影响是平方的
  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

  return fea;
}

/**
 * @brief 计算轨迹超过物理限制的比例
 * @return 超限比例，若小于1表示满足约束，大于1表示超过约束
 * @note 返回速度比例和加速度比例中的较大值
 *       加速度比例使用平方根以保持与速度相同的缩放特性
 */
double NonUniformBspline::checkRatio() {
  Eigen::MatrixXd P         = control_points_;        // 控制点矩阵
  int             dimension = control_points_.cols(); // 空间维度

  // 查找最大速度
  // 遍历所有导数控制点，找出各维度的最大速度值
  double max_vel = -1.0;
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
    for (int j = 0; j < dimension; ++j) {
      max_vel = max(max_vel, fabs(vel(j)));
    }
  }
  // 查找最大加速度
  // 遍历所有二阶导数控制点，找出各维度的最大加速度值
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));
    for (int j = 0; j < dimension; ++j) {
      max_acc = max(max_acc, fabs(acc(j)));
    }
  }
  // 计算超限比例：取速度比和加速度比的最大值
  // 速度比例 = max_vel / limit_vel
  // 加速度比例 = sqrt(max_acc / limit_acc)，用平方根是因为时间缩放对加速度的影响是平方的
  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));
  // 如果比例超过2.0，输出错误信息（严重超限）
  ROS_ERROR_COND(ratio > 2.0, "max vel: %lf, max acc: %lf.", max_vel, max_acc);

  return ratio;
}

/**
 * @brief 通过调整时间分配使轨迹满足物理约束
 * @param show 是否显示调整过程的详细信息
 * @return true表示原始轨迹满足约束，false表示进行了时间重分配
 * @note 通过延长节点间隔来降低速度和加速度，使其满足限制
 *       这种方法不改变轨迹的几何形状，只改变执行速度
 */
bool NonUniformBspline::reallocateTime(bool show) {
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;
  // cout << "origin knots:\n" << u_.transpose() << endl;
  bool fea = true;  // 可行性标志

  Eigen::MatrixXd P         = control_points_;        // 控制点矩阵
  int             dimension = control_points_.cols(); // 空间维度

  double max_vel, max_acc;

  /* 检查速度可行性并调整时间 */
  // 时间重分配策略：速度 ∝ 1/时间，因此延长时间可以降低速度
  for (int i = 0; i < P.rows() - 1; ++i) {
    // 计算第i段的速度
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    // 如果速度超过限制
    if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
        fabs(vel(2)) > limit_vel_ + 1e-4) {

      fea = false;
      if (show) cout << "[Realloc]: Infeasible vel " << i << " :" << vel.transpose() << endl;

      // 找出最大速度分量
      max_vel = -1.0;
      for (int j = 0; j < dimension; ++j) {
        max_vel = max(max_vel, fabs(vel(j)));
      }

      // 计算需要的时间延长比例
      // ratio = max_vel / limit_vel，表示需要将时间延长多少倍才能满足速度约束
      double ratio = max_vel / limit_vel_ + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_;  // 限制最大调整比例，避免过度延长

      // 计算时间调整量
      double time_ori = u_(i + p_ + 1) - u_(i + 1);  // 原始时间间隔
      double time_new = ratio * time_ori;             // 新的时间间隔
      double delta_t  = time_new - time_ori;          // 时间增量
      double t_inc    = delta_t / double(p_);         // 每个节点的时间增量（均匀分配）

      // 调整相关节点：线性增加，保持节点分布的平滑性
      // 从 u_{i+2} 到 u_{i+p+1}，逐渐增加时间偏移
      for (int j = i + 2; j <= i + p_ + 1; ++j) {
        u_(j) += double(j - i - 1) * t_inc;
        if (j <= 5 && j >= 1) {
          // cout << "vel j: " << j << endl;
        }
      }

      // 调整后续所有节点：整体平移，保持后续轨迹的相对关系不变
      for (int j = i + p_ + 2; j < u_.rows(); ++j) {
        u_(j) += delta_t;
      }
    }
  }

  /* 检查加速度可行性并调整时间 */
  // 时间重分配策略：加速度 ∝ 1/时间²，因此延长时间的平方根倍可以降低加速度
  for (int i = 0; i < P.rows() - 2; ++i) {

    // 计算第i段的加速度
    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    // 如果加速度超过限制
    if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
        fabs(acc(2)) > limit_acc_ + 1e-4) {

      fea = false;
      if (show) cout << "[Realloc]: Infeasible acc " << i << " :" << acc.transpose() << endl;

      // 找出最大加速度分量
      max_acc = -1.0;
      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)));
      }

      // 计算需要的时间延长比例（加速度与时间平方成反比，所以用平方根）
      // 若 acc_new = acc_old / ratio²，则需要 ratio = sqrt(acc_old / acc_limit)
      double ratio = sqrt(max_acc / limit_acc_) + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_;  // 限制最大调整比例，避免过度延长
      // cout << "ratio: " << ratio << endl;

      // 计算时间调整量
      double time_ori = u_(i + p_ + 1) - u_(i + 2);  // 原始时间间隔
      double time_new = ratio * time_ori;             // 新的时间间隔
      double delta_t  = time_new - time_ori;          // 时间增量
      double t_inc    = delta_t / double(p_ - 1);     // 每个节点的时间增量（均匀分配）

      // 特殊处理轨迹起始段（i==1或i==2）
      // 起始段需要特殊处理，因为影响范围包括夹紧节点
      if (i == 1 || i == 2) {
        // cout << "acc i: " << i << endl;
        // 调整节点2到5，线性递增时间偏移
        for (int j = 2; j <= 5; ++j) {
          u_(j) += double(j - 1) * t_inc;
        }

        // 调整后续所有节点，整体平移
        for (int j = 6; j < u_.rows(); ++j) {
          u_(j) += 4.0 * t_inc;
        }

      } else {
        // 一般情况的处理

        // 调整相关节点：线性增加，保持节点分布的平滑性
        // 从 u_{i+3} 到 u_{i+p+1}，逐渐增加时间偏移
        for (int j = i + 3; j <= i + p_ + 1; ++j) {
          u_(j) += double(j - i - 2) * t_inc;
          if (j <= 5 && j >= 1) {
            // cout << "acc j: " << j << endl;
          }
        }

        // 调整后续所有节点：整体平移，保持后续轨迹的相对关系不变
        for (int j = i + p_ + 2; j < u_.rows(); ++j) {
          u_(j) += delta_t;
        }
      }
    }
  }

  return fea;
}

/**
 * @brief 按比例延长B样条的执行时间
 * @param ratio 时间延长比例（>1表示延长，<1表示缩短）
 * @note 通过调整节点向量来改变轨迹的执行速度，但不改变几何形状
 *       忽略首尾各5个节点，只调整中间部分
 */
void NonUniformBspline::lengthenTime(const double& ratio) {
  int num1 = 5;                          // 起始调整位置（跳过前5个节点）
  int num2 = getKnot().rows() - 1 - 5;   // 结束调整位置（跳过后5个节点）

  // 计算总时间增量
  // delta_t = (ratio - 1) * 原始时间段长度
  double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
  double t_inc   = delta_t / double(num2 - num1);  // 每个节点的时间增量（均匀分配）

  // 线性调整中间节点，逐渐增加时间偏移
  // 保持节点的相对分布，使时间调整平滑过渡
  for (int i = num1 + 1; i <= num2; ++i) u_(i) += double(i - num1) * t_inc;

  // 整体平移后续节点，保持尾部轨迹的相对关系不变
  for (int i = num2 + 1; i < u_.rows(); ++i) u_(i) += delta_t;
}

/**
 * @brief 重新计算初始状态（预留接口，当前为空实现）
 */
void NonUniformBspline::recomputeInit() {}

/**
 * @brief 将路径点参数化为B样条曲线（通过求解线性系统）
 * @param ts 时间步长
 * @param point_set 路径点集合
 * @param start_end_derivative 起点和终点的导数约束（速度和加速度）
 *                             [起点速度, 终点速度, 起点加速度, 终点加速度]
 * @param ctrl_pts 输出的控制点矩阵
 * @note 使用最小二乘法求解超定方程组 Ax=b，其中A是基函数矩阵，b是路径点和导数约束
 *       这是一个逆向问题：给定路径点，求解控制点
 */
void NonUniformBspline::parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                              const vector<Eigen::Vector3d>& start_end_derivative,
                                              Eigen::MatrixXd&               ctrl_pts) {
  // 参数有效性检查
  if (ts <= 0) {
    cout << "[B-spline]:time step error." << endl;
    return;
  }

  if (point_set.size() < 2) {
    cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
    return;
  }

  if (start_end_derivative.size() != 4) {
    cout << "[B-spline]:derivatives error." << endl;
  }

  int K = point_set.size();  // 路径点数量

  // 构建系数矩阵A
  // 定义三次B样条的基函数模板（基于均匀B样条的性质）
  Eigen::Vector3d prow(3), vrow(3), arow(3);
  prow << 1, 4, 1;      // 位置基函数：(1/6)[1, 4, 1] 对应3个连续控制点的权重（三次B样条性质）
  vrow << -1, 0, 1;     // 速度基函数：(1/2/ts)[-1, 0, 1] 对应速度的差分（一阶导数）
  arow << 1, -2, 1;     // 加速度基函数：(1/ts^2)[1, -2, 1] 对应加速度的二阶差分（二阶导数）

  // 初始化系数矩阵A，大小为(K+4) × (K+2)
  // K个位置约束 + 2个速度约束 + 2个加速度约束 = K+4个方程
  // K+2个未知控制点
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

  // 填充位置约束行：每个路径点由3个连续控制点的加权和表示
  // 对于三次均匀B样条：C(u_i) = (1/6)(P_i + 4*P_{i+1} + P_{i+2})
  for (int i = 0; i < K; ++i) A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

  // 填充速度约束行
  // 速度由相邻控制点的差分表示：C'(u) ≈ (P_{i+1} - P_i) / (2*ts)
  A.block(K, 0, 1, 3)         = (1 / 2.0 / ts) * vrow.transpose();      // 起点速度约束
  A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();      // 终点速度约束

  // 填充加速度约束行
  // 加速度由二阶差分表示：C''(u) ≈ (P_{i+2} - 2*P_{i+1} + P_i) / ts²
  A.block(K + 2, 0, 1, 3)     = (1 / ts / ts) * arow.transpose();       // 起点加速度约束
  A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();       // 终点加速度约束
  // cout << "A:\n" << A << endl;

  // A.block(0, 0, K, K + 2) = (1 / 6.0) * A.block(0, 0, K, K + 2);
  // A.block(K, 0, 2, K + 2) = (1 / 2.0 / ts) * A.block(K, 0, 2, K + 2);
  // A.row(K + 4) = (1 / ts / ts) * A.row(K + 4);
  // A.row(K + 5) = (1 / ts / ts) * A.row(K + 5);

  // 构建右端向量b（约束值）
  // 分别为x、y、z三个维度构建约束向量
  Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);

  // 填充位置约束值（K个路径点）
  for (int i = 0; i < K; ++i) {
    bx(i) = point_set[i](0);
    by(i) = point_set[i](1);
    bz(i) = point_set[i](2);
  }

  // 填充导数约束值（2个速度 + 2个加速度）
  // start_end_derivative[0]: 起点速度
  // start_end_derivative[1]: 终点速度
  // start_end_derivative[2]: 起点加速度
  // start_end_derivative[3]: 终点加速度
  for (int i = 0; i < 4; ++i) {
    bx(K + i) = start_end_derivative[i](0);
    by(K + i) = start_end_derivative[i](1);
    bz(K + i) = start_end_derivative[i](2);
  }

  // 求解线性方程组 Ax = b
  // 使用列主元QR分解求解，适用于超定和欠定系统
  // 这是最小二乘法的标准求解方式，即最小化 ||Ax - b||^2
  Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
  Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
  Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

  // 将求解结果转换为控制点矩阵
  // 每行是一个控制点，每列是一个坐标维度
  ctrl_pts.resize(K + 2, 3);
  ctrl_pts.col(0) = px;  // x坐标
  ctrl_pts.col(1) = py;  // y坐标
  ctrl_pts.col(2) = pz;  // z坐标

  // cout << "[B-spline]: parameterization ok." << endl;
}

/**
 * @brief 获取B样条曲线的总时间
 * @return 曲线的总持续时间
 */
double NonUniformBspline::getTimeSum() {
  double tm, tmp;
  getTimeSpan(tm, tmp);  // 获取起始和结束时间
  return tmp - tm;       // 返回时间差
}

/**
 * @brief 计算B样条曲线的弧长
 * @param res 采样分辨率（时间步长）
 * @return 曲线的总弧长
 * @note 通过在曲线上均匀采样点，累加相邻点之间的欧式距离来近似弧长
 */
double NonUniformBspline::getLength(const double& res) {
  double          length = 0.0;                  // 累计长度
  double          dur    = getTimeSum();         // 总时间
  Eigen::VectorXd p_l    = evaluateDeBoorT(0.0), p_n;  // 上一个点和当前点

  // 以res为步长遍历整条曲线
  // 使用折线段逼近曲线，采样密度越高，近似越精确
  for (double t = res; t <= dur + 1e-4; t += res) {
    p_n = evaluateDeBoorT(t);          // 计算当前点
    length += (p_n - p_l).norm();      // 累加欧氏距离
    p_l = p_n;                         // 更新上一个点
  }
  return length;
}

/**
 * @brief 计算B样条轨迹的Jerk（加加速度）积分
 * @return Jerk的积分值，用于评估轨迹的平滑程度
 * @note Jerk是加速度的导数（三阶导数），较小的Jerk值表示更平滑的轨迹
 *       计算方法：对三阶导数B样条的控制点进行加权求和
 */
double NonUniformBspline::getJerk() {
  // 获取三阶导数B样条（Jerk轨迹）
  // 每次求导降低一阶，三次求导后得到零阶B样条（分段常数）
  NonUniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

  Eigen::VectorXd times     = jerk_traj.getKnot();         // 节点向量
  Eigen::MatrixXd ctrl_pts  = jerk_traj.getControlPoint(); // 控制点
  int             dimension = ctrl_pts.cols();             // 空间维度

  // 计算Jerk积分：∫||jerk(t)||^2 dt
  // 对于零阶B样条（分段常数），积分可以直接通过控制点计算
  double jerk = 0.0;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    for (int j = 0; j < dimension; ++j) {
      // 每段的贡献 = 时间间隔 × 控制点值的平方
      // 这是因为零阶B样条在每段上是常数，∫c^2 dt = c^2 * Δt
      jerk += (times(i + 1) - times(i)) * ctrl_pts(i, j) * ctrl_pts(i, j);
    }
  }

  return jerk;
}

/**
 * @brief 计算B样条轨迹的平均速度和最大速度
 * @param mean_v 输出参数，平均速度
 * @param max_v 输出参数，最大速度
 * @note 以0.01s为采样间隔遍历整条轨迹，计算速度的模
 */
void NonUniformBspline::getMeanAndMaxVel(double& mean_v, double& max_v) {
  NonUniformBspline vel = getDerivative();  // 获取速度B样条（一阶导数）
  double            tm, tmp;
  vel.getTimeSpan(tm, tmp);  // 获取速度曲线的时间范围

  double max_vel = -1.0, mean_vel = 0.0;
  int    num = 0;  // 采样点计数

  // 以0.01s为步长遍历轨迹
  // 采样间隔足够小以捕捉速度变化，同时保持计算效率
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd vxd = vel.evaluateDeBoor(t);  // 计算t时刻的速度向量
    double          vn  = vxd.norm();             // 计算速度的模（标量速率）

    mean_vel += vn;  // 累加速度，用于后续计算平均值
    ++num;
    if (vn > max_vel) {
      max_vel = vn;  // 更新最大速度
    }
  }

  // 计算平均速度：总速度 / 采样点数
  mean_vel = mean_vel / double(num);
  mean_v   = mean_vel;
  max_v    = max_vel;
}

/**
 * @brief 计算B样条轨迹的平均加速度和最大加速度
 * @param mean_a 输出参数，平均加速度
 * @param max_a 输出参数，最大加速度
 * @note 以0.01s为采样间隔遍历整条轨迹，计算加速度的模
 */
void NonUniformBspline::getMeanAndMaxAcc(double& mean_a, double& max_a) {
  NonUniformBspline acc = getDerivative().getDerivative();  // 获取加速度B样条（二阶导数）
  double            tm, tmp;
  acc.getTimeSpan(tm, tmp);  // 获取加速度曲线的时间范围

  double max_acc = -1.0, mean_acc = 0.0;
  int    num = 0;  // 采样点计数

  // 以0.01s为步长遍历轨迹
  // 采样间隔足够小以捕捉加速度变化，同时保持计算效率
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd axd = acc.evaluateDeBoor(t);  // 计算t时刻的加速度向量
    double          an  = axd.norm();             // 计算加速度的模（标量加速度大小）

    mean_acc += an;  // 累加加速度，用于后续计算平均值
    ++num;
    if (an > max_acc) {
      max_acc = an;  // 更新最大加速度
    }
  }

  // 计算平均加速度：总加速度 / 采样点数
  mean_acc = mean_acc / double(num);
  mean_a   = mean_acc;
  max_a    = max_acc;
}
}  // namespace fast_planner
