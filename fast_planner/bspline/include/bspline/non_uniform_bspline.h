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



#ifndef _NON_UNIFORM_BSPLINE_H_
#define _NON_UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace fast_planner {
/**
 * @brief 非均匀B样条类
 *
 * 非均匀B样条（Non-Uniform B-Spline）是轨迹规划中的核心数学工具。
 * 支持不同维度的非均匀B样条实现，也可以表示均匀B样条（均匀B样条是非均匀B样条的特殊情况）。
 *
 * B样条数学原理：
 * - B样条曲线由控制点、节点向量和阶数定义
 * - p阶B样条在(p-1)阶导数处连续（C^{p-1}连续）
 * - 具有局部性：移动一个控制点只影响局部曲线段
 * - 凸包性质：曲线位于控制点的凸包内
 * - 通过调整节点向量可以实现非均匀参数化
 *
 * 在轨迹规划中的应用：
 * - 生成平滑的机器人运动轨迹
 * - 支持速度、加速度等物理约束
 * - 通过控制点优化实现轨迹优化
 * - 支持时间重分配以满足动力学约束
 *
 * 主要特性：
 * - 支持任意维度（2D/3D等空间）
 * - 可调节的阶数和节点向量
 * - 物理约束检查（速度、加速度限制）
 * - 时间重分配功能
 * - 高效的导数计算（通过De Boor算法）
 */
class NonUniformBspline {
private:
  /**
   * @brief 控制点矩阵，B样条曲线的核心数据结构
   *
   * 控制点定义了B样条曲线的形状。B样条曲线不一定经过控制点，但控制点决定了曲线的走向。
   * 矩阵结构：
   * - 每一行代表一个控制点的坐标
   * - 列数表示空间维度（2D/3D等）
   * - 行数为(n+1)个控制点
   *
   * 例如：3D空间中有N个控制点 -> Nx3矩阵
   *       每行为[x, y, z]坐标
   *
   * 控制点的局部性：修改第i个控制点只影响参数区间[u_i, u_{i+p+1}]内的曲线
   */
  Eigen::MatrixXd control_points_;

  /**
   * @brief B样条的基本参数
   *
   * p_: B样条的阶数（degree）
   *     - 决定B样条的平滑度：p阶B样条在(p-1)阶导数处连续
   *     - 常用值：3（三次B样条，C^2连续，适合轨迹规划）
   *     - 每段曲线由(p+1)个控制点决定
   *
   * n_: 控制点索引的最大值，控制点数量为(n+1)
   *
   * m_: 节点向量的最大索引，节点向量长度为(m+1)
   *     - 满足关系：m = n + p + 1
   *     - 对于开放的均匀B样条：前后各有(p+1)个重复节点
   */
  int             p_, n_, m_;

  /**
   * @brief 节点向量（Knot Vector）
   *
   * 节点向量u = [u_0, u_1, ..., u_m]是B样条的参数向量，定义了B样条的参数域。
   *
   * 性质：
   * - 非递减序列：u_i <= u_{i+1}
   * - 长度为m+1 = n+p+2
   * - 有效参数范围：[u_p, u_{m-p}]，在此范围内B样条有定义
   *
   * 节点向量类型：
   * - 均匀节点向量：节点等间隔分布（u_{i+1} - u_i = constant）
   * - 非均匀节点向量：节点间隔可以不同，提供更大的灵活性
   * - 开放节点向量：首尾节点重复(p+1)次，使曲线通过首尾控制点
   *
   * 在轨迹规划中，节点向量通常对应时间参数，节点间隔影响轨迹的时间分配
   */
  Eigen::VectorXd u_;

  /**
   * @brief 节点间隔 Δt
   *
   * 对于均匀B样条，所有相邻节点的间隔相等，即 u_{i+1} - u_i = interval_
   * 该参数决定了轨迹的时间尺度：
   * - 较小的interval_：轨迹执行更快，速度/加速度更大
   * - 较大的interval_：轨迹执行更慢，速度/加速度更小
   *
   * 在时间重分配中，通过调整interval_可以满足动力学约束
   */
  double          interval_;

  /**
   * @brief 获取导数B样条的控制点
   *
   * B样条的导数仍然是B样条，但阶数降低1。
   * 导数B样条的控制点可以通过原始控制点计算得到。
   *
   * 数学原理：
   * 若原B样条为p阶，控制点为P_i，节点向量为u，则其导数B样条为(p-1)阶，
   * 控制点Q_i满足：Q_i = p * (P_{i+1} - P_i) / (u_{i+p+1} - u_{i+1})
   *
   * @return Eigen::MatrixXd 导数B样条的控制点矩阵，行数比原控制点少1
   */
  Eigen::MatrixXd getDerivativeControlPoints();

  /**
   * @brief 物理约束参数
   *
   * limit_vel_: 速度上限 (m/s)
   *             用于检查轨迹是否满足速度约束
   *
   * limit_acc_: 加速度上限 (m/s^2)
   *             用于检查轨迹是否满足加速度约束
   *
   * limit_ratio_: 约束违反比率
   *               当实际速度/加速度超过限制时，记录超出的比率
   *               用于时间重分配，确定需要延长时间的倍数
   */
  double limit_vel_, limit_acc_, limit_ratio_;

public:
  /**
   * @brief 默认构造函数
   *
   * 创建一个空的B样条对象，需要后续调用setUniformBspline等方法初始化
   */
  NonUniformBspline() {}

  /**
   * @brief 构造函数，创建均匀B样条
   *
   * @param points 控制点矩阵，大小为(n+1)×dim，其中dim为空间维度
   * @param order B样条的阶数p，决定曲线的平滑度
   *              - order=3: 三次B样条，C^2连续，常用于轨迹规划
   *              - order=5: 五次B样条，C^4连续，用于更高阶平滑要求
   * @param interval 节点间隔Δt，决定轨迹的时间尺度
   *
   * 该构造函数会自动生成均匀节点向量，节点间隔为interval
   */
  NonUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);

  /**
   * @brief 析构函数
   */
  ~NonUniformBspline();

  /**
   * @brief 初始化为均匀B样条
   *
   * 使用给定的控制点、阶数和节点间隔初始化B样条对象。
   * 该方法会重新计算所有内部参数，包括节点向量。
   *
   * @param points 控制点矩阵，大小为(n+1)×dim
   * @param order B样条的阶数p
   * @param interval 均匀节点间隔Δt
   *
   * 注意：该方法会生成开放的均匀节点向量，使得B样条曲线通过首尾控制点
   */
  void setUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);

  // ========== 获取/设置基本B样条信息 ==========

  /**
   * @brief 设置节点向量
   *
   * 允许手动设置节点向量，用于创建非均匀B样条。
   * 节点向量必须满足非递减条件。
   *
   * @param knot 节点向量，长度必须为(m+1) = (n+p+2)
   *
   * 注意：设置节点向量后，interval_会被重新计算为平均节点间隔
   */
  void                                   setKnot(const Eigen::VectorXd& knot);

  /**
   * @brief 获取节点向量
   *
   * @return Eigen::VectorXd 当前的节点向量u = [u_0, u_1, ..., u_m]
   */
  Eigen::VectorXd                        getKnot();

  /**
   * @brief 获取控制点矩阵
   *
   * @return Eigen::MatrixXd 控制点矩阵，大小为(n+1)×dim
   *         每一行代表一个控制点的坐标
   */
  Eigen::MatrixXd                        getControlPoint();

  /**
   * @brief 获取节点间隔
   *
   * 对于均匀B样条，返回统一的节点间隔。
   * 对于非均匀B样条，返回平均节点间隔。
   *
   * @return double 节点间隔Δt (秒)
   */
  double                                 getInterval();

  /**
   * @brief 获取B样条的有效参数范围（时间跨度）
   *
   * B样条只在参数区间[u_p, u_{m-p}]内有定义。
   * 在轨迹规划中，这对应于轨迹的有效时间范围。
   *
   * @param um 起始参数值（输出），对应u_p
   * @param um_p 结束参数值（输出），对应u_{m-p}
   *
   * 注意：有效参数范围的长度为um_p - um，对应轨迹的总时间
   */
  void                                   getTimeSpan(double& um, double& um_p);

  /**
   * @brief 获取B样条轨迹的起点和终点位置
   *
   * 对于开放的B样条（首尾节点重复），曲线会通过首尾控制点。
   * 该方法计算参数u_p和u_{m-p}处的B样条值，得到轨迹的起止位置。
   *
   * @return pair<Eigen::VectorXd, Eigen::VectorXd>
   *         first: 起点位置坐标
   *         second: 终点位置坐标
   *
   * 应用：用于验证轨迹的起止点是否符合预期
   */
  pair<Eigen::VectorXd, Eigen::VectorXd> getHeadTailPts();

  // ========== 计算位置和导数 ==========

  /**
   * @brief 使用De Boor算法计算B样条在参数u处的值
   *
   * De Boor算法是计算B样条曲线值的标准算法，数值稳定且高效。
   * 算法复杂度为O(p^2)，其中p为B样条阶数。
   *
   * 算法原理：
   * 通过p次递归计算，从控制点逐步插值得到曲线上的点。
   * 对于参数u，算法找到对应的节点区间[u_k, u_{k+1}]，
   * 然后使用该区间附近的(p+1)个控制点进行插值计算。
   *
   * @param u 参数值，有效范围为[u_p, u_{m-p}]
   *          在轨迹规划中，u通常表示时间参数
   *
   * @return Eigen::VectorXd B样条在参数u处的位置，维度与控制点维度相同
   *
   * 注意：若u超出有效范围，结果可能不准确
   */
  Eigen::VectorXd   evaluateDeBoor(const double& u);

  /**
   * @brief 使用时间参数t计算B样条的值（相对时间版本）
   *
   * 该方法将时间参数t转换为B样条参数u，然后调用evaluateDeBoor。
   * 转换关系：u = u_p + t（假设u_p为起始参数）
   *
   * @param t 相对时间参数，范围为[0, duration]
   *          t=0对应轨迹起点，t=duration对应轨迹终点
   *          duration = u_{m-p} - u_p
   *
   * @return Eigen::VectorXd B样条在时间t处的位置
   *
   * 应用：在轨迹跟踪中，根据当前时刻t获取期望位置
   */
  Eigen::VectorXd   evaluateDeBoorT(const double& t);

  /**
   * @brief 获取B样条的导数
   *
   * 返回一个新的NonUniformBspline对象，表示原B样条的一阶导数。
   * 导数B样条的性质：
   * - 阶数降低1：若原B样条为p阶，导数为(p-1)阶
   * - 控制点数量减少1：若原B样条有(n+1)个控制点，导数有n个控制点
   * - 节点向量相同（去掉首尾各一个节点）
   *
   * 在轨迹规划中的应用：
   * - 一阶导数：速度轨迹
   * - 二阶导数：加速度轨迹（对导数B样条再求导）
   * - 三阶导数：Jerk轨迹（加加速度）
   *
   * @return NonUniformBspline 表示导数的B样条对象
   *
   * 注意：可以递归调用该方法获取高阶导数，例如：
   *       vel_traj = pos_traj.getDerivative();
   *       acc_traj = vel_traj.getDerivative();
   */
  NonUniformBspline getDerivative();

  /**
   * @brief 将离散点集参数化为3D B样条曲线，带有边界导数约束
   *
   * 这是一个静态工具方法，用于从离散路径点生成满足边界条件的B样条轨迹。
   * 常用于将A*等离散路径规划算法的输出转换为平滑的B样条轨迹。
   *
   * 方法原理：
   * 1. 根据输入点集和边界导数约束，构建线性方程组
   * 2. 求解得到B样条的控制点
   * 3. 确保生成的B样条满足：
   *    - 曲线接近或通过输入点集
   *    - 起点速度/加速度 = start_end_derivative[0]/[1]
   *    - 终点速度/加速度 = start_end_derivative[2]/[3]
   *
   * @param ts 时间间隔Δt，决定B样条的时间尺度
   *
   * @param point_set 输入路径点集，通常为(K+2)个点
   *                  这些点是期望轨迹经过或接近的位置
   *                  通常来自几何路径规划算法（如A*、RRT等）
   *
   * @param start_end_derivative 边界导数约束，包含4个3D向量：
   *                             [0]: 起点速度 (m/s)
   *                             [1]: 起点加速度 (m/s^2)
   *                             [2]: 终点速度 (m/s)
   *                             [3]: 终点加速度 (m/s^2)
   *
   * @param ctrl_pts 输出的控制点矩阵（输出参数）
   *                 大小通常为(K+6)×3，其中额外的控制点用于满足边界约束
   *
   * 应用场景：
   * - 将A*路径转换为B样条轨迹
   * - 重新参数化已有的离散轨迹
   * - 为优化算法提供初始化轨迹
   *
   * 注意：该方法假设输入点集已经是无碰撞的，不进行碰撞检查
   */
  static void parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                    const vector<Eigen::Vector3d>& start_end_derivative,
                                    Eigen::MatrixXd&               ctrl_pts);

  // ========== 可行性检查与时间调整 ==========

  /**
   * @brief 设置物理约束限制
   *
   * 为轨迹设置速度和加速度的最大允许值。
   * 这些限制通常来自机器人的动力学约束或安全要求。
   *
   * @param vel 速度上限 (m/s)，表示轨迹上任意点的速度模长不应超过该值
   * @param acc 加速度上限 (m/s^2)，表示轨迹上任意点的加速度模长不应超过该值
   *
   * 注意：设置限制后，可以使用checkFeasibility()检查轨迹是否满足约束
   */
  void   setPhysicalLimits(const double& vel, const double& acc);

  /**
   * @brief 检查轨迹的动力学可行性
   *
   * 沿轨迹采样多个点，计算每个点的速度和加速度，
   * 检查是否所有采样点都满足物理约束限制。
   *
   * 检查方法：
   * 1. 沿轨迹均匀采样（采样间隔通常为0.01秒）
   * 2. 在每个采样点计算速度（一阶导数）和加速度（二阶导数）
   * 3. 检查 ||velocity|| <= limit_vel_ 且 ||acceleration|| <= limit_acc_
   * 4. 若所有点都满足，返回true；否则返回false
   *
   * @param show 是否在终端显示详细检查信息，默认为false
   *             若为true，会输出违反约束的具体位置和数值
   *
   * @return bool
   *         true: 轨迹可行，满足所有物理约束
   *         false: 轨迹不可行，存在违反约束的点
   *
   * 应用：在轨迹优化后验证结果是否可执行
   */
  bool   checkFeasibility(bool show = false);

  /**
   * @brief 计算约束违反比率
   *
   * 遍历整条轨迹，找出速度和加速度相对于限制的最大超出比率。
   *
   * 计算方法：
   * ratio_vel = max(||velocity||) / limit_vel_
   * ratio_acc = max(||acceleration||) / limit_acc_
   * 返回 max(ratio_vel, ratio_acc)
   *
   * @return double 最大违反比率
   *         = 1.0: 刚好满足约束（临界状态）
   *         < 1.0: 满足约束，有裕量
   *         > 1.0: 违反约束，需要调整
   *
   * 应用：
   * - 确定时间重分配的缩放因子
   * - 评估轨迹的激进程度
   * - 优化过程中的可行性度量
   */
  double checkRatio();

  /**
   * @brief 延长轨迹的时间尺度
   *
   * 通过增加节点间隔来延长轨迹的执行时间。
   * 时间延长会降低速度和加速度，从而更容易满足动力学约束。
   *
   * 数学原理：
   * - 若节点间隔从Δt变为k*Δt（k>1），则：
   *   - 速度降低为原来的1/k
   *   - 加速度降低为原来的1/k^2
   *   - 轨迹形状保持不变（空间路径相同）
   *
   * @param ratio 时间延长比率（k值），必须≥1.0
   *              例如：ratio=1.5表示时间延长为原来的1.5倍
   *
   * 注意：
   * - 该方法不改变控制点，只改变节点间隔
   * - 延长时间会使轨迹执行更慢，但更安全
   */
  void   lengthenTime(const double& ratio);

  /**
   * @brief 自动重新分配时间以满足物理约束
   *
   * 该方法自动调整轨迹的时间尺度，使其刚好满足速度和加速度约束。
   * 这是一个迭代优化过程。
   *
   * 算法流程：
   * 1. 调用checkRatio()计算当前的违反比率r
   * 2. 若r > 1.0，则调用lengthenTime(r)延长时间
   * 3. 重复步骤1-2，直到r <= 1.0或达到最大迭代次数
   * 4. 返回是否成功满足约束
   *
   * @param show 是否显示重分配过程的详细信息，默认为false
   *             若为true，会输出每次迭代的比率和调整情况
   *
   * @return bool
   *         true: 成功重分配，轨迹现在满足约束
   *         false: 重分配失败（通常不会发生，除非参数设置有误）
   *
   * 应用场景：
   * - 轨迹优化后的后处理步骤
   * - 确保优化得到的轨迹可以安全执行
   * - 在线重规划中快速调整轨迹
   *
   * 注意：该方法会改变轨迹的执行速度，但不改变空间路径
   */
  bool   reallocateTime(bool show = false);

  // ========== 性能评估 ==========

  /**
   * @brief 获取轨迹的总执行时间
   *
   * 计算从轨迹起点到终点所需的时间。
   * 对于均匀B样条：time_sum = (m - 2*p) * interval_
   * 对于非均匀B样条：time_sum = u_{m-p} - u_p
   *
   * @return double 轨迹总时间（秒）
   *
   * 应用：
   * - 评估轨迹的时间效率
   * - 比较不同规划算法的性能
   * - 实时系统中的时间预算
   */
  double getTimeSum();

  /**
   * @brief 获取轨迹的空间长度（弧长）
   *
   * 通过数值积分计算B样条曲线的弧长。
   * 沿参数方向均匀采样，累加相邻采样点之间的欧氏距离。
   *
   * 计算公式（近似）：
   * length ≈ Σ ||S(t_i+1) - S(t_i)||，其中S(t)为B样条曲线
   *
   * @param res 采样时间分辨率（秒），默认为0.01秒
   *            更小的res会提高计算精度，但增加计算量
   *
   * @return double 轨迹的总长度（米）
   *
   * 应用：
   * - 计算平均速度：mean_vel = length / time_sum
   * - 路径效率评估
   * - 能量消耗估计
   */
  double getLength(const double& res = 0.01);

  /**
   * @brief 获取轨迹的Jerk积分（平滑度指标）
   *
   * Jerk（加加速度）是加速度的导数，其积分常用于评估轨迹的平滑度。
   * Jerk越小，轨迹越平滑，对机械系统的冲击越小。
   *
   * 计算公式：
   * Jerk_integral = ∫ ||S'''(t)||^2 dt，其中S'''为三阶导数（jerk）
   *
   * 数值计算方法：
   * 1. 获取B样条的三阶导数（连续求导三次）
   * 2. 沿时间方向采样
   * 3. 累加采样点处的jerk平方
   *
   * @return double Jerk的积分值（(m/s^3)^2 * s）
   *
   * 应用：
   * - 轨迹优化的目标函数（最小化jerk）
   * - 评估轨迹的平滑度和舒适度
   * - 减少机械磨损和振动
   *
   * 注意：
   * - 对于p<3的B样条，三阶导数可能不连续或不存在
   * - 通常使用三次或更高阶B样条以确保jerk有定义
   */
  double getJerk();

  /**
   * @brief 获取轨迹的平均速度和最大速度
   *
   * 通过采样整条轨迹，统计速度的分布特性。
   *
   * 计算方法：
   * 1. 沿轨迹等时间间隔采样（通常0.01秒）
   * 2. 在每个采样点计算速度模长 ||S'(t)||
   * 3. mean_v = (1/N) * Σ||S'(t_i)||
   * 4. max_v = max{||S'(t_i)||}
   *
   * @param mean_v 平均速度（输出参数，单位：m/s）
   *               表示轨迹的整体速度水平
   * @param max_v 最大速度（输出参数，单位：m/s）
   *              表示轨迹的速度峰值
   *
   * 应用：
   * - 验证是否满足速度限制
   * - 评估轨迹的激进程度
   * - 能量消耗估算
   */
  void   getMeanAndMaxVel(double& mean_v, double& max_v);

  /**
   * @brief 获取轨迹的平均加速度和最大加速度
   *
   * 通过采样整条轨迹，统计加速度的分布特性。
   *
   * 计算方法：
   * 1. 沿轨迹等时间间隔采样（通常0.01秒）
   * 2. 在每个采样点计算加速度模长 ||S''(t)||
   * 3. mean_a = (1/N) * Σ||S''(t_i)||
   * 4. max_a = max{||S''(t_i)||}
   *
   * @param mean_a 平均加速度（输出参数，单位：m/s^2）
   *               表示轨迹的整体加速度水平
   * @param max_a 最大加速度（输出参数，单位：m/s^2）
   *              表示轨迹的加速度峰值
   *
   * 应用：
   * - 验证是否满足加速度限制
   * - 评估对执行器的需求
   * - 力/力矩需求估算
   */
  void   getMeanAndMaxAcc(double& mean_a, double& max_a);

  /**
   * @brief 重新计算和初始化B样条的内部参数
   *
   * 当手动修改控制点或节点向量后，需要调用该方法更新内部状态。
   * 该方法会重新计算：
   * - 控制点数量 n
   * - 节点向量长度 m
   * - 平均节点间隔 interval_
   *
   * 注意：
   * - 通常在外部修改了control_points_或u_后调用
   * - 确保修改后的数据满足B样条的基本约束（m = n + p + 1）
   */
  void recomputeInit();

  /**
   * Eigen库的内存对齐宏
   * 确保包含Eigen对象的类在动态分配时正确对齐，
   * 以支持SIMD优化（SSE/AVX等）
   */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace fast_planner
#endif