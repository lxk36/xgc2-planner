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



#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <fast_plan_env/edt_environment.h>
#include <ros/ros.h>

// 基于梯度的B样条轨迹优化器
// 使用弹性带(Elastic Band)优化方法，结合符号距离场(SDF)进行轨迹优化

// 输入: 符号距离场(Signed Distance Field)和一系列控制点序列
// 输出: 优化后的控制点序列
// 控制点格式: N x 3 矩阵，每一行代表一个控制点的坐标(x, y, z)
namespace fast_planner {

/**
 * @class BsplineOptimizer
 * @brief B样条轨迹优化器类
 *
 * 该类实现了基于梯度下降的B样条轨迹优化算法，主要功能包括：
 * 1. 平滑性优化：最小化轨迹的急动度(jerk)和加速度
 * 2. 避障优化：基于ESDF保持与障碍物的安全距离
 * 3. 动力学可行性：满足速度、加速度等动力学约束
 * 4. 路径引导：利用几何引导路径优化轨迹形状
 * 5. 路径点约束：强制轨迹通过指定的路径点
 * 6. 可视性优化：针对视觉应用优化轨迹的可见性
 *
 * 优化目标函数是多个加权子目标的组合，使用LBFGS或其他优化算法求解
 */
class BsplineOptimizer {

public:
  // ==================== 代价函数组件常量定义 ====================
  // 这些常量用于位掩码，指定优化时要使用的代价函数组件
  static const int SMOOTHNESS;   // 平滑性代价：最小化轨迹的急动度(jerk)和加速度
  static const int DISTANCE;     // 距离代价：基于ESDF保持与障碍物的安全距离
  static const int FEASIBILITY;  // 可行性代价：确保轨迹满足速度和加速度约束
  static const int ENDPOINT;     // 终点代价：确保轨迹终点与目标点一致
  static const int GUIDE;        // 引导代价：使轨迹靠近引导路径
  static const int WAYPOINTS;    // 路径点代价：强制轨迹通过指定的中间路径点

  // ==================== 优化阶段常量定义 ====================
  static const int GUIDE_PHASE;  // 引导阶段：使用引导路径进行粗优化
  static const int NORMAL_PHASE; // 正常阶段：不使用引导路径的精细优化

  // 构造函数和析构函数
  BsplineOptimizer() {}
  ~BsplineOptimizer() {}

  // ==================== 主要API接口 ====================

  /**
   * @brief 设置环境地图(ESDF)
   * @param env 欧几里得距离变换环境指针，提供障碍物距离场信息
   */
  void setEnvironment(const EDTEnvironment::Ptr& env);

  /**
   * @brief 从ROS参数服务器加载优化参数
   * @param nh ROS节点句柄，用于读取参数
   */
  void setParam(ros::NodeHandle& nh);

  /**
   * @brief B样条轨迹优化的主函数（一站式接口）
   * @param points 初始B样条控制点矩阵 (N×3)
   * @param ts B样条时间间隔(knot span)，单位：秒
   * @param cost_function 代价函数组合标志（使用位掩码组合多个代价项）
   * @param max_num_id 最大迭代次数索引（用于索引max_iteration_num_数组）
   * @param max_time_id 最大优化时间索引（用于索引max_iteration_time_数组）
   * @return 优化后的控制点矩阵
   */
  Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                      const int& cost_function, int max_num_id, int max_time_id);

  // ==================== 辅助函数接口 ====================

  // ---------- 必需的输入参数 ----------

  /**
   * @brief 设置B样条控制点
   * @param points 控制点矩阵 (N×3)，每行是一个控制点的xyz坐标
   */
  void setControlPoints(const Eigen::MatrixXd& points);

  /**
   * @brief 设置B样条时间间隔
   * @param ts B样条的knot span（节点间隔），单位：秒
   */
  void setBsplineInterval(const double& ts);

  /**
   * @brief 设置代价函数类型
   * @param cost_function 代价函数组合（使用位掩码，如SMOOTHNESS | DISTANCE）
   */
  void setCostFunction(const int& cost_function);

  /**
   * @brief 设置优化终止条件
   * @param max_num_id 最大迭代次数的索引
   * @param max_time_id 最大优化时间的索引
   */
  void setTerminateCond(const int& max_num_id, const int& max_time_id);

  // ---------- 可选的输入参数 ----------

  /**
   * @brief 设置几何引导路径
   * @param guide_pt 引导路径点序列，用于引导优化方向
   * @note 通常包含N-6个点（因为B样条前后各有3个固定控制点）
   */
  void setGuidePath(const vector<Eigen::Vector3d>& guide_pt);

  /**
   * @brief 设置必须通过的中间路径点约束
   * @param waypts 路径点位置序列
   * @param waypt_idx 路径点对应的控制点索引序列
   * @note 最多支持N-2个约束（首尾控制点已固定）
   */
  void setWaypoints(const vector<Eigen::Vector3d>& waypts,
                    const vector<int>&             waypt_idx);

  /**
   * @brief 执行优化过程
   * @note 调用前需先通过set函数设置好所有必需参数
   */
  void optimize();

  /**
   * @brief 获取优化后的控制点
   * @return 控制点矩阵 (N×3)
   */
  Eigen::MatrixXd getControlPoints();

  /**
   * @brief 将控制点矩阵转换为向量序列
   * @param ctrl_pts 控制点矩阵 (N×3)
   * @return 控制点向量序列
   */
  vector<Eigen::Vector3d> matrixToVectors(const Eigen::MatrixXd& ctrl_pts);

private:
  // ==================== 环境信息 ====================
  EDTEnvironment::Ptr edt_environment_;  // 欧几里得距离变换环境，提供ESDF障碍物距离场

  // ==================== 主要输入数据 ====================
  Eigen::MatrixXd control_points_;     // B样条控制点矩阵，维度：N × dim（通常dim=3）
  double          bspline_interval_;   // B样条节点间隔(knot span)，单位：秒
  Eigen::Vector3d end_pt_;             // 轨迹终点目标位置
  int             dim_;                // B样条的维度（通常为3，表示3D空间）

  // ==================== 可选约束数据 ====================
  vector<Eigen::Vector3d> guide_pts_;  // 几何引导路径点，通常包含N-6个点（用于引导优化）
  vector<Eigen::Vector3d> waypoints_;  // 必须通过的中间路径点约束
  vector<int>             waypt_idx_;  // 路径点对应的控制点索引

  // ==================== 优化控制参数 ====================
  int    max_num_id_, max_time_id_;    // 优化终止条件：最大迭代次数和最大时间的索引
  int    cost_function_;               // 代价函数类型标志（位掩码组合）
  bool   dynamic_;                     // 是否考虑动态障碍物（移动障碍物）
  double start_time_;                  // 全局起始时间，用于动态障碍物的时间同步

  // ==================== 优化算法参数 ====================
  int    order_;                  // B样条阶数(degree)，通常为3（对应4阶B样条）

  // ---------- 代价函数权重参数 ----------
  double lambda1_;                // 平滑性权重：急动度(jerk)最小化的权重系数
  double lambda2_;                // 距离权重：障碍物距离惩罚的权重系数
  double lambda3_;                // 可行性权重：动力学约束违反惩罚的权重系数
  double lambda4_;                // 终点权重：终点偏差惩罚的权重系数
  double lambda5_;                // 引导权重：与引导路径偏差的权重系数
  double lambda6_;                // 可视性权重：视野遮挡惩罚的权重系数（用于视觉导航）
  double lambda7_;                // 路径点权重：中间路径点偏差的权重系数
  double lambda8_;                // 加速度平滑性权重：加速度变化率最小化的权重系数

  // ---------- 约束参数 ----------
  double dist0_;                  // 安全距离阈值，机器人与障碍物的最小安全距离
  double max_vel_, max_acc_;      // 动力学限制：最大速度和最大加速度
  double visib_min_;              // 可视性阈值：最小可见距离或可见性度量
  double wnl_;                    // 非线性权重参数（具体用途取决于实现）
  double dlmin_;                  // 最小距离限制参数

  // ---------- 优化算法选择 ----------
  int    algorithm1_;             // 二次型代价函数的优化算法（如梯度下降、LBFGS等）
  int    algorithm2_;             // 一般代价函数的优化算法（处理非二次项）

  // ---------- 终止条件 ----------
  int    max_iteration_num_[4];   // 不同优化阶段的最大迭代次数数组
  double max_iteration_time_[4];  // 不同优化阶段的最大优化时间数组（单位：秒）

  // ==================== 中间变量 ====================
  // 代价函数梯度缓存：预分配内存以避免优化过程中的反复分配和释放
  // 这些向量存储各个代价项对每个控制点的梯度
  vector<Eigen::Vector3d> g_q_;           // 总梯度：所有代价项梯度的加权和
  vector<Eigen::Vector3d> g_smoothness_;  // 平滑性代价的梯度（关于每个控制点）
  vector<Eigen::Vector3d> g_distance_;    // 距离代价的梯度（障碍物排斥力）
  vector<Eigen::Vector3d> g_feasibility_; // 可行性代价的梯度（动力学约束违反）
  vector<Eigen::Vector3d> g_endpoint_;    // 终点代价的梯度（终点偏差）
  vector<Eigen::Vector3d> g_guide_;       // 引导代价的梯度（引导路径吸引力）
  vector<Eigen::Vector3d> g_waypoints_;   // 路径点代价的梯度（路径点约束）

  // ---------- 优化状态变量 ----------
  int                 variable_num_;   // 优化变量总数（通常为 N×dim，N为控制点数）
  int                 iter_num_;       // 当前优化迭代次数计数器
  std::vector<double> best_variable_;  // 记录当前最优解的变量值（用于保存最优控制点）
  double              min_cost_;       // 记录当前最小代价值

  // ---------- 可视性计算相关 ----------
  vector<Eigen::Vector3d> block_pts_;  // 阻挡点集合，用于计算轨迹的可视性（视觉遮挡检测）

  // ==================== 代价函数计算 ====================
  // 以控制点序列q作为输入，计算各个代价项及其梯度

  /**
   * @brief 静态代价函数包装器
   * @param x 优化变量（展平的控制点坐标）
   * @param grad 梯度输出（与x维度相同）
   * @param func_data 函数数据指针（指向BsplineOptimizer对象）
   * @return 总代价值
   * @note 用于与NLopt等优化库接口，将成员函数包装为静态函数
   */
  static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);

  /**
   * @brief 组合所有代价项
   * @param x 优化变量（展平的控制点坐标）
   * @param grad 梯度输出
   * @param cost 总代价输出
   * @note 根据cost_function_标志位，加权组合各个代价项
   */
  void combineCost(const std::vector<double>& x, vector<double>& grad, double& cost);

  // ---------- 各个代价项的计算函数 ----------
  // 注意：q为控制点向量序列，包含所有控制点

  /**
   * @brief 计算平滑性代价（急动度最小化）
   * @param q 控制点序列
   * @param cost 代价输出
   * @param gradient 梯度输出（每个控制点的梯度）
   * @note 最小化轨迹的三阶导数（jerk），使轨迹更平滑
   */
  void calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                          vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 计算距离代价（障碍物避碰）
   * @param q 控制点序列
   * @param cost 代价输出
   * @param gradient 梯度输出
   * @note 基于ESDF，对距离障碍物过近的控制点施加惩罚
   */
  void calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                        vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 计算可行性代价（动力学约束）
   * @param q 控制点序列
   * @param cost 代价输出
   * @param gradient 梯度输出
   * @note 惩罚违反速度、加速度限制的轨迹段
   */
  void calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                           vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 计算终点代价（目标点约束）
   * @param q 控制点序列
   * @param cost 代价输出
   * @param gradient 梯度输出
   * @note 惩罚轨迹终点与目标位置的偏差
   */
  void calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                        vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 计算引导代价（引导路径约束）
   * @param q 控制点序列
   * @param cost 代价输出
   * @param gradient 梯度输出
   * @note 使轨迹靠近预先计算的几何引导路径
   */
  void calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 计算可视性代价（视野约束）
   * @param q 控制点序列
   * @param cost 代价输出
   * @param gradient 梯度输出
   * @note 针对视觉导航，惩罚视野被遮挡的轨迹段
   */
  void calcVisibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                          vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 计算路径点代价（中间路径点约束）
   * @param q 控制点序列
   * @param cost 代价输出
   * @param gradient 梯度输出
   * @note 强制轨迹通过指定的中间路径点
   */
  void calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                         vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 计算视角代价（相机视角优化）
   * @param q 控制点序列
   * @param cost 代价输出
   * @param gradient 梯度输出
   * @note 优化相机视角，用于主动视觉任务
   */
  void calcViewCost(const vector<Eigen::Vector3d>& q, double& cost, vector<Eigen::Vector3d>& gradient);

  /**
   * @brief 判断当前代价函数是否为二次型
   * @return 如果所有激活的代价项都是二次型则返回true
   * @note 二次型代价可以使用更高效的优化算法（如共轭梯度法）
   */
  bool isQuadratic();

  // ==================== 性能评估接口（仅用于基准测试） ====================
public:
  vector<double> vec_cost_;   // 优化过程中的代价历史记录
  vector<double> vec_time_;   // 优化过程中的时间历史记录
  ros::Time      time_start_; // 优化开始时刻

  /**
   * @brief 获取优化过程的代价曲线和时间序列
   * @param cost 代价历史输出（每次迭代的代价值）
   * @param time 时间历史输出（每次迭代的累计时间）
   * @note 用于分析优化器的收敛性能和调试
   */
  void getCostCurve(vector<double>& cost, vector<double>& time) {
    cost = vec_cost_;
    time = vec_time_;
  }

  // 智能指针类型定义，方便管理BsplineOptimizer对象的生命周期
  typedef unique_ptr<BsplineOptimizer> Ptr;

  // Eigen库要求：为包含固定大小Eigen成员的类提供内存对齐
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace fast_planner
#endif