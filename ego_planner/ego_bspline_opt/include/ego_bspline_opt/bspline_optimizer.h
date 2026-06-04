/**
 * @file bspline_optimizer.h
 * @brief B样条轨迹优化器头文件
 *
 * 该文件定义了EGO-Planner中的B样条轨迹优化器，使用梯度下降和弹性带优化方法
 * 对生成的轨迹进行优化，确保轨迹的平滑性、安全性和动力学可行性
 */

#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <ego_path_searching/dyn_a_star.h>
#include <ego_bspline_opt/uniform_bspline.h>
#include <ego_plan_env/grid_map.h>
#include <ros/ros.h>
#include "ego_bspline_opt/lbfgs.hpp"

// 梯度优化和弹性带优化
// 输入：有符号距离场和一系列点
// 输出：优化后的点序列
// 点的格式：N x 3 矩阵，每行代表一个点
namespace ego_planner
{

  /**
   * @brief 控制点类
   *
   * 存储B样条轨迹的控制点及其相关信息，包括安全距离、碰撞检测的基准点和方向向量等
   */
  class ControlPoints
  {
  public:
    double clearance;                                     // 安全间隙距离
    int size;                                             // 控制点数量
    Eigen::MatrixXd points;                               // 控制点坐标矩阵，3 x N，每列代表一个控制点的xyz坐标
    std::vector<std::vector<Eigen::Vector3d>> base_point; // 方向向量的起点（碰撞点），用于存储控制点对应的障碍物表面碰撞点
    std::vector<std::vector<Eigen::Vector3d>> direction;  // 方向向量，必须是归一化的，指向远离障碍物的方向
    std::vector<bool> flag_temp;                          // 临时标志位，在多个地方使用，每次使用前需要重新初始化
    // std::vector<bool> occupancy;                       // 占用标志（已注释）

    /**
     * @brief 重新设置控制点数据结构的大小
     * @param size_set 新的控制点数量
     */
    void resize(const int size_set)
    {
      size = size_set;

      // 清空所有向量
      base_point.clear();
      direction.clear();
      flag_temp.clear();
      // occupancy.clear();

      // 重新分配内存
      points.resize(3, size_set);      // 3行N列，每列是一个控制点
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
      // occupancy.resize(size);
    }
  };

  /**
   * @brief B样条轨迹优化器类
   *
   * 该类负责对B样条轨迹进行优化，包括：
   * 1. 平滑性优化（最小化jerk）
   * 2. 障碍物避障优化（距离场梯度）
   * 3. 动力学可行性优化（速度、加速度约束）
   * 4. 路径拟合优化（引导路径跟随）
   *
   * 优化过程分为两个阶段：
   * - Rebound阶段：处理碰撞，将控制点推离障碍物
   * - Refine阶段：精细优化，平衡各项代价
   */
  class BsplineOptimizer
  {

  public:
    BsplineOptimizer() {}
    ~BsplineOptimizer() {}

    /* ========== 主要API ========== */

    /**
     * @brief 设置环境地图
     * @param env 栅格地图指针，包含障碍物信息和距离场
     */
    void setEnvironment(const GridMap::Ptr &env);

    /**
     * @brief 从ROS参数服务器加载优化器参数
     * @param nh ROS节点句柄
     */
    void setParam(ros::NodeHandle &nh);

    /**
     * @brief B样条轨迹优化主函数
     * @param points 初始控制点矩阵
     * @param ts B样条时间间隔
     * @param cost_function 代价函数类型选择
     * @param max_num_id 最大迭代次数
     * @param max_time_id 最大优化时间
     * @return 优化后的控制点矩阵
     */
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);

    /* ========== 辅助函数 ========== */

    // 必需输入
    /**
     * @brief 设置控制点
     * @param points 控制点矩阵，3 x N
     */
    void setControlPoints(const Eigen::MatrixXd &points);

    /**
     * @brief 设置B样条时间间隔
     * @param ts 时间间隔（秒）
     */
    void setBsplineInterval(const double &ts);

    /**
     * @brief 设置代价函数类型
     * @param cost_function 代价函数ID
     */
    void setCostFunction(const int &cost_function);

    /**
     * @brief 设置优化终止条件
     * @param max_num_id 最大迭代次数
     * @param max_time_id 最大优化时间
     */
    void setTerminateCond(const int &max_num_id, const int &max_time_id);

    // 可选输入
    /**
     * @brief 设置引导路径点
     * @param guide_pt 引导路径点向量
     */
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);

    /**
     * @brief 设置路径点约束
     * @param waypts 路径点向量
     * @param waypt_idx 路径点对应的控制点索引，最多N-2个约束
     */
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most

    /**
     * @brief 执行优化
     */
    void optimize();

    /**
     * @brief 获取优化后的控制点
     * @return 控制点矩阵
     */
    Eigen::MatrixXd getControlPoints();

    AStar::Ptr a_star_;                         // A*路径搜索器指针
    std::vector<Eigen::Vector3d> ref_pts_;      // 参考路径点

    /**
     * @brief 初始化控制点（处理碰撞检测）
     * @param init_points 初始控制点矩阵
     * @param flag_first_init 是否是首次初始化标志
     * @return 每个控制点对应的碰撞信息（基准点和方向向量）
     */
    std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);

    /**
     * @brief B样条轨迹Rebound优化（碰撞处理阶段）
     * @param optimal_points 输出优化后的控制点
     * @param ts 时间间隔
     * @return 优化是否成功
     * @note 必须在initControlPoints()之后调用
     */
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts); // must be called after initControlPoints()

    /**
     * @brief B样条轨迹Refine优化（精细优化阶段）
     * @param init_points 初始控制点
     * @param ts 时间间隔
     * @param optimal_points 输出优化后的控制点
     * @return 优化是否成功
     */
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    /**
     * @brief 获取B样条阶数
     * @return B样条阶数
     */
    inline int getOrder(void) { return order_; }

  private:
    GridMap::Ptr grid_map_;  // 环境栅格地图指针

    /**
     * @brief 强制停止优化的类型枚举
     */
    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,         // 不停止，继续优化
      STOP_FOR_REBOUND,  // 为Rebound阶段停止（检测到碰撞）
      STOP_FOR_ERROR     // 因错误停止
    } force_stop_type_;

    /* ========== 主要输入参数 ========== */
    // Eigen::MatrixXd control_points_;     // B样条控制点，N x dim（已注释）
    double bspline_interval_;               // B样条节点跨度（时间间隔）
    Eigen::Vector3d end_pt_;                // 轨迹终点
    // int             dim_;                // B样条的维度（已注释）

    vector<Eigen::Vector3d> guide_pts_;     // 几何引导路径点，通常为N-6个点
    vector<Eigen::Vector3d> waypoints_;     // 路径点约束
    vector<int> waypt_idx_;                 // 路径点约束对应的控制点索引

    int max_num_id_, max_time_id_;          // 停止准则：最大迭代次数和最大时间
    int cost_function_;                     // 代价函数类型，用于确定目标函数
    double start_time_;                     // 全局时间，用于动态障碍物

    /* ========== 优化参数 ========== */
    int order_;                             // B样条阶数（degree）
    double lambda1_;                        // jerk平滑性权重
    double lambda2_, new_lambda2_;          // 距离场权重（障碍物避障）
    double lambda3_;                        // 可行性权重（动力学约束）
    double lambda4_;                        // 曲线拟合权重（路径跟随）

    int a;                                  // 辅助变量

    double dist0_;                          // 安全距离阈值
    double max_vel_, max_acc_;              // 动力学限制：最大速度和最大加速度

    int variable_num_;                      // 优化变量数量
    int iter_num_;                          // 求解器迭代次数
    Eigen::VectorXd best_variable_;         // 最优变量（最佳解）
    double min_cost_;                       // 最小代价值

    ControlPoints cps_;                     // 控制点数据结构

    /* ========== 代价函数 ========== */
    /* 以控制点q作为输入，计算代价函数的各个部分 */

    /**
     * @brief 通用代价函数接口（用于LBFGS求解器）
     * @param x 优化变量向量
     * @param grad 梯度向量（输出）
     * @param func_data 函数数据指针（传递this指针）
     * @return 代价函数值
     */
    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);

    /**
     * @brief 组合所有代价项
     * @param x 优化变量向量
     * @param grad 梯度向量（输出）
     * @param cost 总代价（输出）
     */
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    /**
     * @brief 计算平滑性代价（jerk最小化）
     * @param q 控制点矩阵，包含所有控制点
     * @param cost 平滑性代价（输出）
     * @param gradient 平滑性代价梯度（输出）
     * @param falg_use_jerk 是否使用jerk作为平滑性度量（默认true）
     */
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool falg_use_jerk = true);

    /**
     * @brief 计算可行性代价（动力学约束）
     * @param q 控制点矩阵
     * @param cost 可行性代价（输出）
     * @param gradient 可行性代价梯度（输出）
     */
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient);

    /**
     * @brief 计算Rebound阶段的距离场代价（障碍物避障）
     * @param q 控制点矩阵
     * @param cost 距离代价（输出）
     * @param gradient 距离代价梯度（输出）
     * @param iter_num 当前迭代次数
     * @param smoothness_cost 平滑性代价值（用于自适应权重调整）
     */
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);

    /**
     * @brief 计算路径拟合代价（引导路径跟随）
     * @param q 控制点矩阵
     * @param cost 拟合代价（输出）
     * @param gradient 拟合代价梯度（输出）
     */
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);

    /**
     * @brief 检查碰撞并计算弹回方向
     * @return 是否检测到碰撞
     */
    bool check_collision_and_rebound(void);

    /**
     * @brief 提前退出回调函数（用于LBFGS求解器）
     * @param func_data 函数数据指针
     * @param x 当前优化变量
     * @param g 当前梯度
     * @param fx 当前函数值
     * @param xnorm 变量范数
     * @param gnorm 梯度范数
     * @param step 步长
     * @param n 变量维度
     * @param k 迭代次数
     * @param ls 线搜索次数
     * @return 0继续，非0停止
     */
    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);

    /**
     * @brief Rebound阶段的代价函数（LBFGS接口）
     * @param func_data 函数数据指针（传递this指针）
     * @param x 优化变量数组
     * @param grad 梯度数组（输出）
     * @param n 变量维度
     * @return 代价函数值
     */
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);

    /**
     * @brief Refine阶段的代价函数（LBFGS接口）
     * @param func_data 函数数据指针（传递this指针）
     * @param x 优化变量数组
     * @param grad 梯度数组（输出）
     * @param n 变量维度
     * @return 代价函数值
     */
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

    /**
     * @brief 执行Rebound阶段的优化
     * @return 优化是否成功
     */
    bool rebound_optimize();

    /**
     * @brief 执行Refine阶段的优化
     * @return 优化是否成功
     */
    bool refine_optimize();

    /**
     * @brief Rebound阶段的组合代价函数
     * @param x 优化变量数组
     * @param grad 梯度数组（输出）
     * @param f_combine 组合代价值（输出）
     * @param n 变量维度
     */
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);

    /**
     * @brief Refine阶段的组合代价函数
     * @param x 优化变量数组
     * @param grad 梯度数组（输出）
     * @param f_combine 组合代价值（输出）
     * @param n 变量维度
     */
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);

    /* ========== 仅用于基准评估 ========== */
  public:
    typedef unique_ptr<BsplineOptimizer> Ptr;  // 智能指针类型定义

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen库内存对齐宏，确保使用SSE/AVX指令时内存对齐
  };

} // namespace ego_planner
#endif