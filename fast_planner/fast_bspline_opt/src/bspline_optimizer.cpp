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



#include "fast_bspline_opt/bspline_optimizer.h"
#include <nlopt.hpp>
// using namespace std;

namespace fast_planner {

// ============================================================================
// 优化目标函数类型定义(使用位掩码表示，可通过位运算组合多个目标)
// ============================================================================
const int BsplineOptimizer::SMOOTHNESS  = (1 << 0);  // 平滑性: 最小化jerk(加加速度)
const int BsplineOptimizer::DISTANCE    = (1 << 1);  // 距离: 保持与障碍物的安全距离
const int BsplineOptimizer::FEASIBILITY = (1 << 2);  // 可行性: 满足速度和加速度约束
const int BsplineOptimizer::ENDPOINT    = (1 << 3);  // 端点: 确保终点位置准确
const int BsplineOptimizer::GUIDE       = (1 << 4);  // 引导: 跟随引导路径
const int BsplineOptimizer::WAYPOINTS   = (1 << 6);  // 路径点: 通过指定的中间路径点

// 优化阶段定义
const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;  // 引导阶段: 平滑性+引导
const int BsplineOptimizer::NORMAL_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;  // 常规阶段: 平滑性+距离+可行性

/**
 * @brief 从ROS参数服务器加载优化器参数
 * @param nh ROS节点句柄
 */
void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
  // 加载各项代价函数的权重系数
  nh.param("optimization/lambda1", lambda1_, -1.0);  // 平滑性权重
  nh.param("optimization/lambda2", lambda2_, -1.0);  // 距离代价权重
  nh.param("optimization/lambda3", lambda3_, -1.0);  // 可行性代价权重
  nh.param("optimization/lambda4", lambda4_, -1.0);  // 端点代价权重
  nh.param("optimization/lambda5", lambda5_, -1.0);  // 引导路径权重
  nh.param("optimization/lambda6", lambda6_, -1.0);  // 预留参数
  nh.param("optimization/lambda7", lambda7_, -1.0);  // 路径点代价权重
  nh.param("optimization/lambda8", lambda8_, -1.0);  // 预留参数

  // 加载约束参数
  nh.param("optimization/dist0", dist0_, -1.0);          // 安全距离阈值
  nh.param("optimization/max_vel", max_vel_, -1.0);      // 最大速度限制
  nh.param("optimization/max_acc", max_acc_, -1.0);      // 最大加速度限制
  nh.param("optimization/visib_min", visib_min_, -1.0);  // 最小可见性(预留)
  nh.param("optimization/dlmin", dlmin_, -1.0);          // 最小路径长度(预留)
  nh.param("optimization/wnl", wnl_, -1.0);              // 权重参数(预留)

  // 加载迭代终止条件 - 最大迭代次数(支持4种不同配置)
  nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
  nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
  nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
  nh.param("optimization/max_iteration_num4", max_iteration_num_[3], -1);

  // 加载迭代终止条件 - 最大迭代时间(支持4种不同配置)
  nh.param("optimization/max_iteration_time1", max_iteration_time_[0], -1.0);
  nh.param("optimization/max_iteration_time2", max_iteration_time_[1], -1.0);
  nh.param("optimization/max_iteration_time3", max_iteration_time_[2], -1.0);
  nh.param("optimization/max_iteration_time4", max_iteration_time_[3], -1.0);

  // 加载优化算法选择
  nh.param("optimization/algorithm1", algorithm1_, -1);  // 二次型问题的算法
  nh.param("optimization/algorithm2", algorithm2_, -1);  // 非二次型问题的算法
  nh.param("optimization/order", order_, -1);            // B样条曲线的阶数
}

/**
 * @brief 设置环境地图(用于距离场查询)
 * @param env 欧几里得距离变换(EDT)环境指针
 */
void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

/**
 * @brief 设置B样条控制点
 * @param points 控制点矩阵，每行为一个控制点，列数为维度(1D/2D/3D)
 */
void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd& points) {
  control_points_ = points;
  dim_            = control_points_.cols();  // 记录优化问题的维度
}

/**
 * @brief 设置B样条时间间隔(控制点之间的时间步长)
 * @param ts 时间间隔
 */
void BsplineOptimizer::setBsplineInterval(const double& ts) { bspline_interval_ = ts; }

/**
 * @brief 设置优化终止条件的配置索引
 * @param max_num_id 最大迭代次数配置ID(0-3)
 * @param max_time_id 最大迭代时间配置ID(0-3)
 */
void BsplineOptimizer::setTerminateCond(const int& max_num_id, const int& max_time_id) {
  max_num_id_  = max_num_id;
  max_time_id_ = max_time_id;
}

/**
 * @brief 设置优化的代价函数组合
 * @param cost_code 代价函数组合码(通过位运算组合多个目标)
 */
void BsplineOptimizer::setCostFunction(const int& cost_code) {
  cost_function_ = cost_code;

  // 打印当前启用的优化目标
  string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";    // 平滑性
  if (cost_function_ & DISTANCE) cost_str += " dist  |";      // 距离
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";   // 可行性
  if (cost_function_ & ENDPOINT) cost_str += " endpt |";      // 端点
  if (cost_function_ & GUIDE) cost_str += " guide |";         // 引导
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";     // 路径点

  ROS_INFO_STREAM("cost func: " << cost_str);
}

/**
 * @brief 设置引导路径点(用于引导优化方向)
 * @param guide_pt 引导路径点序列
 */
void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) { guide_pts_ = guide_pt; }

/**
 * @brief 设置必经路径点及其对应的控制点索引
 * @param waypts 必经路径点序列
 * @param waypt_idx 每个路径点对应的控制点索引
 */
void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d>& waypts,
                                    const vector<int>&             waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

/**
 * @brief B样条轨迹优化的主入口函数
 * @param points 初始控制点矩阵
 * @param ts B样条时间间隔
 * @param cost_function 代价函数组合
 * @param max_num_id 最大迭代次数配置ID
 * @param max_time_id 最大迭代时间配置ID
 * @return 优化后的控制点矩阵
 */
Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                                      const int& cost_function, int max_num_id,
                                                      int max_time_id) {
  setControlPoints(points);
  setBsplineInterval(ts);
  setCostFunction(cost_function);
  setTerminateCond(max_num_id, max_time_id);

  optimize();  // 执行优化
  return this->control_points_;
}

/**
 * @brief 执行B样条轨迹优化的核心函数
 *
 * 优化流程:
 * 1. 初始化求解器和梯度存储
 * 2. 确定优化变量数量
 * 3. 配置NLopt优化器
 * 4. 执行优化求解
 * 5. 提取最优结果
 */
void BsplineOptimizer::optimize() {
  /* ========== 初始化优化器 ========== */
  iter_num_        = 0;                                    // 迭代次数计数器
  min_cost_        = std::numeric_limits<double>::max();   // 最小代价初始化为最大值
  const int pt_num = control_points_.rows();               // 控制点总数

  // 为各项梯度分配存储空间
  g_q_.resize(pt_num);           // 总梯度
  g_smoothness_.resize(pt_num);  // 平滑性梯度
  g_distance_.resize(pt_num);    // 距离梯度
  g_feasibility_.resize(pt_num); // 可行性梯度
  g_endpoint_.resize(pt_num);    // 端点梯度
  g_waypoints_.resize(pt_num);   // 路径点梯度
  g_guide_.resize(pt_num);       // 引导梯度

  /* ========== 确定优化变量数量 ========== */
  if (cost_function_ & ENDPOINT) {
    // 端点约束模式: 优化除起点外的所有控制点
    variable_num_ = dim_ * (pt_num - order_);

    // 计算期望的终点位置(使用三次B样条的德布尔公式)
    // 对于三次B样条，终点位置 = (P_{n-3} + 4*P_{n-2} + P_{n-1}) / 6
    end_pt_ = (1 / 6.0) *
        (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
         control_points_.row(pt_num - 1));
  } else {
    // 常规模式: 固定起点和终点的order个控制点，只优化中间部分
    variable_num_ = max(0, dim_ * (pt_num - 2 * order_)) ;
  }

  /* ========== 配置NLopt优化器 ========== */
  // 根据问题类型选择优化算法: 二次型问题用algorithm1，非二次型用algorithm2
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);  // 设置目标函数
  opt.set_maxeval(max_iteration_num_[max_num_id_]);             // 设置最大迭代次数
  opt.set_maxtime(max_iteration_time_[max_time_id_]);           // 设置最大优化时间
  opt.set_xtol_rel(1e-5);                                       // 设置相对收敛阈值

  /* ========== 准备优化变量初值 ========== */
  // 将控制点矩阵转换为NLopt的一维向量格式
  vector<double> q(variable_num_);
  for (int i = order_; i < pt_num; ++i) {
    // 如果不是端点约束模式，则跳过最后order个控制点(固定不优化)
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - order_) + j] = control_points_(i, j);
    }
  }

  /* ========== 设置优化变量边界约束 ========== */
  if (dim_ != 1) {  // 对于2D/3D问题设置盒式约束
    vector<double> lb(variable_num_), ub(variable_num_);
    const double   bound = 10.0;  // 边界范围: 初值±10.0
    for (int i = 0; i < variable_num_; ++i) {
      lb[i] = q[i] - bound;  // 下界
      ub[i] = q[i] + bound;  // 上界
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  /* ========== 执行优化求解 ========== */
  try {
    // cout << fixed << setprecision(7);
    // vec_time_.clear();
    // vec_cost_.clear();
    // time_start_ = ros::Time::now();

    double        final_cost;
    nlopt::result result = opt.optimize(q, final_cost);  // 运行优化算法

    /* retrieve the optimization result */
    // cout << "Min cost:" << min_cost_ << endl;
  } catch (std::exception& e) {
    ROS_WARN("[Optimization]: nlopt exception");
    cout << e.what() << endl;
  }

  /* ========== 提取最优解并更新控制点 ========== */
  // 将优化得到的最优变量(best_variable_)转换回控制点矩阵格式
  for (int i = order_; i < control_points_.rows(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
    }
  }

  // 打印迭代次数(引导模式下不打印)
  if (!(cost_function_ & GUIDE)) ROS_INFO_STREAM("iter num: " << iter_num_);
}

/**
 * @brief 计算平滑性代价及其梯度
 *
 * 平滑性通过最小化jerk(加加速度)来实现，jerk是位置的三阶导数。
 * 对于三次B样条，jerk = q[i+3] - 3*q[i+2] + 3*q[i+1] - q[i]
 * 代价函数: cost = Σ ||jerk||²
 *
 * @param q 控制点序列
 * @param cost 输出的平滑性代价值
 * @param gradient 输出的梯度向量
 */
void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                                          vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  // 遍历所有可以计算jerk的控制点段
  for (int i = 0; i < q.size() - order_; i++) {
    /* 计算jerk(三阶差分) */
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    cost += jerk.squaredNorm();  // 累加jerk的平方范数
    temp_j = 2.0 * jerk;

    /* 计算jerk对各控制点的梯度 (链式法则) */
    // ∂(||jerk||²)/∂q[i] = 2*jerk * ∂jerk/∂q[i]
    gradient[i + 0] += -temp_j;       // ∂jerk/∂q[i] = -1
    gradient[i + 1] += 3.0 * temp_j;  // ∂jerk/∂q[i+1] = 3
    gradient[i + 2] += -3.0 * temp_j; // ∂jerk/∂q[i+2] = -3
    gradient[i + 3] += temp_j;        // ∂jerk/∂q[i+3] = 1
  }
}

/**
 * @brief 计算距离场代价及其梯度
 *
 * 使用欧几里得距离变换(EDT)保持轨迹与障碍物的安全距离。
 * 当控制点距离障碍物小于安全距离dist0_时，产生惩罚代价。
 * 代价函数: cost = Σ (dist - dist0_)² (当 dist < dist0_)
 *
 * @param q 控制点序列
 * @param cost 输出的距离代价值
 * @param gradient 输出的梯度向量
 */
void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double          dist;       // 到最近障碍物的距离
  Eigen::Vector3d dist_grad;  // 距离场梯度

  // 确定需要评估的控制点范围
  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    // 查询EDT距离场，获取距离和梯度
    edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);

    // 归一化距离梯度(避免数值不稳定)
    if (dist_grad.norm() > 1e-4) dist_grad.normalize();

    // 仅当距离小于安全阈值时才施加惩罚
    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);                     // 二次惩罚
      gradient[i] += 2.0 * (dist - dist0_) * dist_grad;  // 梯度: ∂cost/∂q[i] = 2(d-d0)*∇d
    }
  }
}

/**
 * @brief 计算动力学可行性代价及其梯度
 *
 * 确保轨迹满足速度和加速度约束。使用软约束(惩罚函数)处理不等式约束。
 * 速度约束: ||v||² ≤ v_max²  (各轴独立检查)
 * 加速度约束: ||a||² ≤ a_max²  (各轴独立检查)
 *
 * 对于B样条:
 * - 速度 v ≈ (q[i+1] - q[i]) / ts
 * - 加速度 a ≈ (q[i+2] - 2*q[i+1] + q[i]) / ts²
 *
 * @param q 控制点序列
 * @param cost 输出的可行性代价值
 * @param gradient 输出的梯度向量
 */
void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                                           vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  /* 预计算常用变量 */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;  // 最大速度平方
  am2 = max_acc_ * max_acc_;  // 最大加速度平方

  ts      = bspline_interval_;   // 时间间隔
  ts_inv2 = 1 / ts / ts;         // 1/ts²
  ts_inv4 = ts_inv2 * ts_inv2;   // 1/ts⁴

  /* ========== 速度可行性约束 ========== */
  for (int i = 0; i < q.size() - 1; i++) {
    // 计算速度的近似值(一阶差分)
    Eigen::Vector3d vi = q[i + 1] - q[i];

    // 对每个轴独立检查速度约束
    for (int j = 0; j < 3; j++) {
      // vd = v²*ts_inv2 - v_max² (违反约束时vd > 0)
      double vd = vi(j) * vi(j) * ts_inv2 - vm2;
      if (vd > 0.0) {  // 超出速度限制
        cost += pow(vd, 2);  // 二次惩罚

        // 计算梯度: ∂cost/∂q = 2*vd * ∂vd/∂q
        double temp_v = 4.0 * vd * ts_inv2;
        gradient[i + 0](j) += -temp_v * vi(j);  // ∂vi/∂q[i] = -1
        gradient[i + 1](j) += temp_v * vi(j);   // ∂vi/∂q[i+1] = 1
      }
    }
  }

  /* ========== 加速度可行性约束 ========== */
  for (int i = 0; i < q.size() - 2; i++) {
    // 计算加速度的近似值(二阶差分)
    Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];

    // 对每个轴独立检查加速度约束
    for (int j = 0; j < 3; j++) {
      // ad = a²*ts_inv4 - a_max² (违反约束时ad > 0)
      double ad = ai(j) * ai(j) * ts_inv4 - am2;
      if (ad > 0.0) {  // 超出加速度限制
        cost += pow(ad, 2);  // 二次惩罚

        // 计算梯度: ∂cost/∂q = 2*ad * ∂ad/∂q
        double temp_a = 4.0 * ad * ts_inv4;
        gradient[i + 0](j) += temp_a * ai(j);      // ∂ai/∂q[i] = 1
        gradient[i + 1](j) += -2 * temp_a * ai(j); // ∂ai/∂q[i+1] = -2
        gradient[i + 2](j) += temp_a * ai(j);      // ∂ai/∂q[i+2] = 1
      }
    }
  }
}

/**
 * @brief 计算端点约束代价及其梯度
 *
 * 确保B样条曲线的终点位置精确到达目标点。
 * 对于三次B样条，终点位置通过德布尔(De Boor)公式计算:
 * p_end = (q_{n-3} + 4*q_{n-2} + q_{n-1}) / 6
 *
 * 代价函数: cost = ||p_end - target||²
 *
 * @param q 控制点序列
 * @param cost 输出的端点代价值
 * @param gradient 输出的梯度向量
 */
void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // 提取最后三个控制点
  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  // 计算终点位置偏差(使用德布尔公式)
  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
  cost += dq.squaredNorm();  // 位置偏差的平方

  // 计算梯度: ∂cost/∂q = 2*dq * ∂p_end/∂q
  gradient[q.size() - 3] += 2 * dq * (1 / 6.0);  // ∂p_end/∂q_{n-3} = 1/6
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);  // ∂p_end/∂q_{n-2} = 4/6
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0);  // ∂p_end/∂q_{n-1} = 1/6
}

/**
 * @brief 计算路径点约束代价及其梯度
 *
 * 确保轨迹通过指定的中间路径点(waypoints)。
 * 对于每个路径点，使用B样条的德布尔公式计算对应位置:
 * p_waypoint = (q_i + 4*q_{i+1} + q_{i+2}) / 6
 *
 * 代价函数: cost = Σ ||p_waypoint - target_waypoint||²
 *
 * @param q 控制点序列
 * @param cost 输出的路径点代价值
 * @param gradient 输出的梯度向量
 */
void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                                         vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // 遍历所有指定的路径点
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];  // 目标路径点
    int             idx   = waypt_idx_[i];  // 对应的控制点索引

    // 提取相关的三个控制点
    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    // 计算B样条上对应点的位置偏差
    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();  // 累加位置偏差的平方

    // 计算梯度: ∂cost/∂q = 2*dq * ∂p/∂q
    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);  // 2*dq*(1/6)
  }
}

/**
 * @brief 计算引导路径代价及其梯度
 *
 * 使用几何路径上的均匀采样点来引导轨迹优化。
 * 为每个待优化的控制点分配一个引导点，惩罚它们之间的距离。
 * 这种方法常用于优化的初始阶段，提供一个良好的初始解。
 *
 * 代价函数: cost = Σ ||q_i - guide_pt_i||²
 *
 * @param q 控制点序列
 * @param cost 输出的引导代价值
 * @param gradient 输出的梯度向量
 */
void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost,
                                     vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // 只对中间的控制点施加引导约束(固定首尾order个点)
  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector3d gpt = guide_pts_[i - order_];  // 对应的引导点
    cost += (q[i] - gpt).squaredNorm();            // 距离的平方
    gradient[i] += 2 * (q[i] - gpt);               // 梯度: 2*(q - guide)
  }
}

/**
 * @brief 组合所有代价项并计算总代价及梯度
 *
 * 该函数负责:
 * 1. 将NLopt的优化变量(一维向量)转换为控制点格式
 * 2. 计算各项代价及其梯度
 * 3. 加权组合所有代价项
 * 4. 将梯度转换回NLopt格式
 *
 * @param x NLopt格式的优化变量(一维向量)
 * @param grad 输出的梯度(一维向量)
 * @param f_combine 输出的总代价
 */
void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
  /* ========== 将NLopt变量转换为控制点 ========== */

  // 求解器支持1D-3D的B样条优化，但统一使用Vector3d存储控制点
  // 对于1D情况，第2和第3个元素为0；2D情况类似

  // 复制固定的前order个控制点(起点附近)
  for (int i = 0; i < order_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i][j] = control_points_(i, j);
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i][j] = 0.0;
    }
  }

  // 复制优化变量(中间的控制点)
  for (int i = 0; i < variable_num_ / dim_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i + order_][j] = x[dim_ * i + j];
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i + order_][j] = 0.0;
    }
  }

  // 如果不是端点约束模式，复制固定的最后order个控制点(终点附近)
  if (!(cost_function_ & ENDPOINT)) {
    for (int i = 0; i < order_; i++) {

      for (int j = 0; j < dim_; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] =
            control_points_(control_points_.rows() - order_ + i, j);
      }
      for (int j = dim_; j < 3; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
      }
    }
  }

  /* ========== 初始化总代价和梯度 ========== */
  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);

  /* ========== 计算各项代价及其梯度 ========== */
  double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints;
  f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;

  // 1. 平滑性代价(最小化jerk)
  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
    f_combine += lambda1_ * f_smoothness;  // 加权累加到总代价
    // 将梯度转换为NLopt格式并累加
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);
  }

  // 2. 距离场代价(避障)
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += lambda2_ * f_distance;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);
  }

  // 3. 动力学可行性代价(速度和加速度约束)
  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
    f_combine += lambda3_ * f_feasibility;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);
  }

  // 4. 端点约束代价
  if (cost_function_ & ENDPOINT) {
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
    f_combine += lambda4_ * f_endpoint;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);
  }

  // 5. 引导路径代价
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += lambda5_ * f_guide;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);
  }

  // 6. 路径点约束代价
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += lambda7_ * f_waypoints;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);
  }
  /* ========== 调试输出(已注释) ========== */
  // 可用于调试时打印各项代价的详细信息
  // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
  //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
  //        << ", waypt: " << lambda7_ * f_waypoints << endl;
  // }

  // if (optimization_phase_ == SECOND_PHASE) {
  //  << ", smooth: " << lambda1_ * f_smoothness
  //  << " , dist:" << lambda2_ * f_distance
  //  << ", fea: " << lambda3_ * f_feasibility << endl;
  // << ", end: " << lambda4_ * f_endpoint
  // << ", guide: " << lambda5_ * f_guide
  // }
}

/**
 * @brief NLopt优化器的目标函数回调(静态函数)
 *
 * NLopt优化器会在每次迭代时调用此函数，计算当前状态的代价和梯度。
 * 该函数是静态成员函数，通过func_data指针访问BsplineOptimizer实例。
 *
 * @param x 当前的优化变量(NLopt格式)
 * @param grad 输出的梯度向量(NLopt格式)
 * @param func_data 指向BsplineOptimizer实例的指针
 * @return 当前状态的总代价值
 */
double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  // 将void*指针转换为BsplineOptimizer实例指针
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double            cost;

  // 计算组合代价及梯度
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;  // 迭代计数器加1

  /* 保存最优解 */
  if (cost < opt->min_cost_) {
    opt->min_cost_      = cost;
    opt->best_variable_ = x;  // 记录当前最优变量
  }
  return cost;

  // /* 性能评估代码(已注释) */
  // ros::Time te1 = ros::Time::now();
  // double time_now = (te1 - opt->time_start_).toSec();
  // opt->vec_time_.push_back(time_now);
  // if (opt->vec_cost_.size() == 0)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else if (opt->vec_cost_.back() > f_combine)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else
  // {
  //   opt->vec_cost_.push_back(opt->vec_cost_.back());
  // }
}

/**
 * @brief 将控制点矩阵转换为向量列表
 * @param ctrl_pts 控制点矩阵(每行一个控制点)
 * @return 控制点向量列表
 */
vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

/**
 * @brief 获取优化后的控制点
 * @return 控制点矩阵
 */
Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

/**
 * @brief 判断当前优化问题是否为二次型
 *
 * 二次型问题可以使用更高效的优化算法(如LBFGS)。
 * 目前判定为二次型的情况:
 * 1. 引导阶段(GUIDE_PHASE): 仅包含平滑性和引导代价
 * 2. 平滑性+路径点: 两者都是二次型代价
 *
 * @return true表示二次型问题，false表示非二次型问题
 */
bool BsplineOptimizer::isQuadratic() {
  if (cost_function_ == GUIDE_PHASE) {
    // 引导阶段: 平滑性(二次型) + 引导(二次型)
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    // 平滑性(二次型) + 路径点(二次型)
    return true;
  }
  return false;  // 其他情况(包含距离场、可行性约束等)为非二次型
}

}  // namespace fast_planner