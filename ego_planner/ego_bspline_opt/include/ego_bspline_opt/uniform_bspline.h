/**
 * @file uniform_bspline.h
 * @brief 均匀B样条曲线实现头文件
 * @details 提供了均匀B样条的完整实现，支持任意维度的B样条曲线生成、求导、可行性检查等功能
 */

#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace ego_planner
{
  /**
   * @class UniformBspline
   * @brief 均匀B样条曲线类
   * @details 实现了均匀B样条曲线的各种操作，包括：
   *          1. 基于控制点的B样条构造
   *          2. De Boor算法进行曲线求值
   *          3. 导数B样条的计算
   *          4. 从离散点拟合B样条（带边界约束）
   *          5. 动力学可行性检查（速度、加速度限制）
   *          6. 轨迹性能评估（长度、加加速度、平均/最大速度加速度等）
   *
   * B样条基础知识：
   * - p: B样条的阶数(degree)，p=3表示三次B样条
   * - n+1: 控制点数量
   * - m+1: 节点向量长度，满足 m = n + p + 1
   * - 节点向量u: 定义B样条的参数域，对于均匀B样条，节点间隔相等
   */
  class UniformBspline
  {
  private:
    /**
     * @brief B样条的控制点矩阵
     * @details 存储B样条的所有控制点，支持任意维度：
     *          - 每一行代表一个控制点
     *          - 列数决定了空间维度
     *          - 例如：N个控制点的3D空间B样条 -> N×3矩阵
     *          - 对于轨迹规划，通常是3维(x,y,z)或更高维(包含速度等)
     */
    Eigen::MatrixXd control_points_;

    /**
     * @brief B样条的关键参数
     * @param p_ B样条的阶数(degree)，通常p=3表示三次B样条
     * @param n_ 控制点索引的最大值，实际控制点数为n_+1
     * @param m_ 节点向量索引的最大值，满足关系式 m_ = n_ + p_ + 1
     */
    int p_, n_, m_;

    /**
     * @brief 节点向量(knot vector)
     * @details 定义B样条的参数域，长度为m_+1
     *          对于均匀B样条，相邻节点间隔相等，都等于interval_
     *          节点向量决定了B样条基函数的支撑区间
     */
    Eigen::VectorXd u_;

    /**
     * @brief 节点间隔 Δt
     * @details 对于均匀B样条，所有相邻节点的间隔都相等
     *          这个值直接影响轨迹的时间分配和速度特性
     */
    double interval_;

    /**
     * @brief 计算导数B样条的控制点
     * @return 导数B样条的控制点矩阵
     * @details 根据B样条求导的性质，导数B样条的阶数降低1(p-1)
     *          控制点数减少1个，通过差分公式计算得到
     */
    Eigen::MatrixXd getDerivativeControlPoints();

    /**
     * @brief 物理限制参数和可行性检查相关
     * @param limit_vel_ 速度上限(m/s)
     * @param limit_acc_ 加速度上限(m/s²)
     * @param limit_ratio_ 时间调整的限制比率，防止时间缩放过度
     * @param feasibility_tolerance_ 可行性检查的容差，允许轻微超出限制
     */
    double limit_vel_, limit_acc_, limit_ratio_, feasibility_tolerance_;

  public:
    /* ==================== 构造与析构函数 ==================== */

    /**
     * @brief 默认构造函数
     */
    UniformBspline() {}

    /**
     * @brief 参数化构造函数
     * @param points 控制点矩阵，每行一个控制点
     * @param order B样条的阶数(degree)
     * @param interval 节点间隔Δt
     * @details 直接通过控制点、阶数和节点间隔构造均匀B样条
     */
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    /**
     * @brief 析构函数
     */
    ~UniformBspline();

    /* ==================== 基本访问器 ==================== */

    /**
     * @brief 获取控制点矩阵
     * @return 控制点矩阵的拷贝
     */
    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    /* ==================== 初始化与设置 ==================== */

    /**
     * @brief 将对象初始化为均匀B样条
     * @param points 控制点矩阵
     * @param order B样条阶数
     * @param interval 节点间隔
     * @details 设置控制点、阶数和节点间隔，并自动生成均匀节点向量
     */
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    /* ==================== B样条基本信息的获取与设置 ==================== */

    /**
     * @brief 设置节点向量
     * @param knot 新的节点向量
     * @details 允许手动设置节点向量，可用于非均匀B样条
     */
    void setKnot(const Eigen::VectorXd &knot);

    /**
     * @brief 获取节点向量
     * @return 当前的节点向量
     */
    Eigen::VectorXd getKnot();

    /**
     * @brief 获取控制点矩阵
     * @return 控制点矩阵
     */
    Eigen::MatrixXd getControlPoint();

    /**
     * @brief 获取节点间隔
     * @return 节点间隔Δt
     */
    double getInterval();

    /**
     * @brief 获取B样条的有效时间跨度
     * @param um 输出参数：起始节点值 u[p]
     * @param um_p 输出参数：终止节点值 u[m-p]
     * @return 是否成功获取时间跨度
     * @details B样条曲线的有效定义域为[u[p], u[m-p]]，对应时间域[0, duration]
     */
    bool getTimeSpan(double &um, double &um_p);

    /* ==================== 曲线求值与导数计算 ==================== */

    /**
     * @brief 使用De Boor算法计算B样条在参数u处的值
     * @param u 参数值，范围为 [u[p], u[m-p]]
     * @return 计算得到的曲线上的点（向量形式，维度与控制点相同）
     * @details De Boor算法是B样条求值的经典高效算法，时间复杂度O(p²)
     *          对于p次B样条，需要p+1个控制点参与计算
     */
    Eigen::VectorXd evaluateDeBoor(const double &u);

    /**
     * @brief 使用时间参数t计算B样条的值（便捷接口）
     * @param t 时间参数，范围为 [0, duration]
     * @return 计算得到的曲线上的点
     * @details 将时间参数t转换为节点参数u = t + u[p]，然后调用evaluateDeBoor
     *          这样用户可以直接使用从0开始的时间参数，更加直观
     */
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); }

    /**
     * @brief 获取当前B样条的导数B样条
     * @return 导数B样条对象
     * @details 导数B样条的性质：
     *          - 阶数降低1：p-1
     *          - 控制点数减少1：n个
     *          - 通过控制点差分公式计算新控制点
     *          - 可以递归调用获取高阶导数
     */
    UniformBspline getDerivative();

    /* ==================== 轨迹点插值与拟合 ==================== */

    /**
     * @brief 通过离散3D点集生成B样条，带边界速度和加速度约束
     * @param ts 时间间隔（节点间隔）
     * @param point_set 输入的离散点集，包含K+2个点（K个中间点+2个边界点）
     * @param start_end_derivative 边界导数约束，包含4个向量：
     *                             [0]: 起始速度
     *                             [1]: 起始加速度
     *                             [2]: 终止速度
     *                             [3]: 终止加速度
     * @param ctrl_pts 输出参数：计算得到的控制点矩阵，共K+6个控制点
     * @details 这是一个静态函数，用于从离散轨迹点拟合B样条
     *          通过求解带约束的线性方程组，确保：
     *          1. B样条曲线通过（或接近）给定的点
     *          2. 起点和终点的速度、加速度满足给定约束
     *          输出的控制点可用于构造一个平滑的三次B样条轨迹
     */
    static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                      const vector<Eigen::Vector3d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);

    /* ==================== 可行性检查与时间调整 ==================== */

    /**
     * @brief 设置物理限制参数
     * @param vel 最大速度限制(m/s)
     * @param acc 最大加速度限制(m/s²)
     * @param tolerance 可行性检查的容差，通常设为0.0-0.1之间
     * @details 这些限制用于后续的可行性检查，确保生成的轨迹满足无人机的动力学约束
     */
    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);

    /**
     * @brief 检查轨迹的动力学可行性
     * @param ratio 输出参数：如果不可行，返回建议的时间缩放比率
     * @param show 是否显示详细检查信息（用于调试）
     * @return true表示轨迹可行，false表示违反了速度或加速度限制
     * @details 通过采样轨迹上的点，检查速度和加速度是否超过设定的限制
     *          如果超限，计算需要的时间缩放比率，使轨迹变慢以满足约束
     *          时间缩放比率 > 1 表示需要延长时间
     */
    bool checkFeasibility(double &ratio, bool show = false);

    /**
     * @brief 延长轨迹的时间分配
     * @param ratio 时间缩放比率（>1表示延长时间，<1表示缩短时间）
     * @details 通过调整节点间隔interval_来改变轨迹的时间分配
     *          新的interval = 原interval × ratio
     *          这不会改变轨迹的空间形状，只改变运动的快慢
     *          常用于将不可行的轨迹调整为可行
     */
    void lengthenTime(const double &ratio);

    /* ==================== 轨迹性能评估 ==================== */

    /**
     * @brief 获取轨迹的总时间
     * @return 轨迹总时长（秒）
     * @details 总时长 = (m - 2p) × interval
     *          即有效节点跨度乘以节点间隔
     */
    double getTimeSum();

    /**
     * @brief 计算轨迹的总长度
     * @param res 采样分辨率（默认0.01秒），越小越精确但计算量越大
     * @return 轨迹的弧长（米）
     * @details 通过在轨迹上均匀采样点，计算相邻点之间的欧氏距离累加
     *          用于评估轨迹的空间跨度
     */
    double getLength(const double &res = 0.01);

    /**
     * @brief 计算轨迹的Jerk（加加速度）积分
     * @return Jerk的积分值
     * @details Jerk是加速度的导数（三阶导数），反映轨迹的平滑程度
     *          Jerk越小，轨迹越平滑，对执行器越友好
     *          这是轨迹优化中常用的平滑性指标
     */
    double getJerk();

    /**
     * @brief 获取轨迹的平均速度和最大速度
     * @param mean_v 输出参数：平均速度(m/s)
     * @param max_v 输出参数：最大速度(m/s)
     * @details 通过采样轨迹上的速度（一阶导数），统计平均值和最大值
     *          用于评估轨迹的速度特性
     */
    void getMeanAndMaxVel(double &mean_v, double &max_v);

    /**
     * @brief 获取轨迹的平均加速度和最大加速度
     * @param mean_a 输出参数：平均加速度(m/s²)
     * @param max_a 输出参数：最大加速度(m/s²)
     * @details 通过采样轨迹上的加速度（二阶导数），统计平均值和最大值
     *          用于评估轨迹的加速度特性和动力学需求
     */
    void getMeanAndMaxAcc(double &mean_a, double &max_a);

    /**
     * @brief Eigen库的内存对齐宏
     * @details 确保类的内存对齐，对于包含Eigen固定大小矩阵的类是必需的
     *          防止在动态分配内存时出现对齐问题
     */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace ego_planner
#endif