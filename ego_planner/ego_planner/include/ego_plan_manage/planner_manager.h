/**
 * @file planner_manager.h
 * @brief EGO Planner规划管理器头文件
 * @details 定义了规划管理器类，作为整个规划系统的核心调度模块，
 *          负责协调地图、路径搜索和轨迹优化等各个模块的工作
 */

#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

// B样条优化器相关头文件
#include <ego_bspline_opt/bspline_optimizer.h>  // B样条轨迹优化器
#include <ego_bspline_opt/uniform_bspline.h>    // 均匀B样条轨迹表示
#include <ego_planner/DataDisp.h>               // 数据分发和通信
#include <ego_plan_env/grid_map.h>              // 栅格地图环境
#include <ego_plan_manage/plan_container.hpp>       // 规划数据容器
#include <ros/ros.h>                            // ROS核心功能
#include <ego_traj_utils/planning_visualization.h> // 规划可视化工具

namespace ego_planner
{

  /**
   * @class EGOPlannerManager
   * @brief EGO规划器管理器类
   * @details 这是整个规划系统的核心管理类，负责：
   *          1. 调度地图、路径搜索和轨迹优化等关键算法模块
   *          2. 提供全局规划和局部重规划的接口
   *          3. 管理轨迹数据和规划参数
   *          4. 处理规划失败和紧急停止等特殊情况
   */
  class EGOPlannerManager
  {
    // SECTION stable - 稳定版本的核心功能
  public:
    /**
     * @brief 构造函数
     */
    EGOPlannerManager();

    /**
     * @brief 析构函数
     */
    ~EGOPlannerManager();

    // Eigen库内存对齐宏，确保使用SSE/AVX指令集时的正确性
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* ==================== 主要规划接口函数 ==================== */

    /**
     * @brief 弹性重规划函数（核心局部规划接口）
     * @details 当检测到当前轨迹可能发生碰撞时，执行局部重规划。
     *          采用B样条优化方法，在保持轨迹平滑性的同时避开障碍物。
     *          "Rebound"指的是轨迹像弹性体一样从障碍物弹开。
     *
     * @param start_pt 起始位置 (x, y, z)
     * @param start_vel 起始速度 (vx, vy, vz)
     * @param start_acc 起始加速度 (ax, ay, az)
     * @param end_pt 目标位置 (x, y, z)
     * @param end_vel 目标速度 (vx, vy, vz)
     * @param flag_polyInit 是否使用多项式进行初始化
     * @param flag_randomPolyTraj 是否使用随机多项式轨迹进行初始化
     * @return bool 规划是否成功
     */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);

    /**
     * @brief 紧急停止函数
     * @details 在检测到危险情况时，生成一条快速停止的轨迹，
     *          使无人机在指定位置安全停止
     *
     * @param stop_pos 停止位置 (x, y, z)
     * @return bool 紧急停止轨迹生成是否成功
     */
    bool EmergencyStop(Eigen::Vector3d stop_pos);

    /**
     * @brief 全局轨迹规划函数
     * @details 根据起点和终点的完整状态（位置、速度、加速度），
     *          规划一条全局最优的B样条轨迹
     *
     * @param start_pos 起始位置 (x, y, z)
     * @param start_vel 起始速度 (vx, vy, vz)
     * @param start_acc 起始加速度 (ax, ay, az)
     * @param end_pos 目标位置 (x, y, z)
     * @param end_vel 目标速度 (vx, vy, vz)
     * @param end_acc 目标加速度 (ax, ay, az)
     * @return bool 全局规划是否成功
     */
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    /**
     * @brief 带路径点的全局轨迹规划函数
     * @details 规划一条经过多个中间路径点的全局轨迹，
     *          适用于需要经过特定位置的任务场景
     *
     * @param start_pos 起始位置 (x, y, z)
     * @param start_vel 起始速度 (vx, vy, vz)
     * @param start_acc 起始加速度 (ax, ay, az)
     * @param waypoints 中间路径点列表，按顺序访问
     * @param end_vel 目标速度 (vx, vy, vz)
     * @param end_acc 目标加速度 (ax, ay, az)
     * @return bool 路径点规划是否成功
     */
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    /**
     * @brief 初始化规划模块
     * @details 初始化规划器所需的各个子模块，包括：
     *          - 从ROS参数服务器加载配置参数
     *          - 初始化B样条优化器
     *          - 设置可视化工具
     *
     * @param nh ROS节点句柄，用于获取参数和话题通信
     * @param vis 可视化工具指针，用于显示规划结果（可选）
     */
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

    /* ==================== 公共成员变量 ==================== */

    PlanParameters pp_;           ///< 规划参数集合，包含速度/加速度限制、优化权重等
    LocalTrajData local_data_;    ///< 局部轨迹数据，存储当前的局部重规划轨迹
    GlobalTrajData global_data_;  ///< 全局轨迹数据，存储全局规划的参考轨迹
    GridMap::Ptr grid_map_;       ///< 栅格地图指针，用于障碍物检测和碰撞检查

  private:
    /* ==================== 主要规划算法和模块 ==================== */

    PlanningVisualization::Ptr visualization_; ///< 可视化工具指针，用于在RViz中显示轨迹、路径点等

    BsplineOptimizer::Ptr bspline_optimizer_rebound_; ///< B样条优化器指针，用于局部重规划中的轨迹优化

    int continous_failures_count_{0}; ///< 连续规划失败计数器，用于监控规划器稳定性

    /**
     * @brief 更新轨迹信息
     * @details 根据优化后的位置轨迹，计算并更新速度、加速度等导数信息，
     *          同时更新轨迹的时间戳和持续时间
     *
     * @param position_traj 位置轨迹的B样条表示
     * @param time_now 当前时间戳
     */
    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);

    /**
     * @brief B样条重参数化函数
     * @details 对B样条轨迹进行时间尺度的重新参数化，通过调整时间间隔dt
     *          来改变轨迹的执行速度，同时保持轨迹的几何形状不变
     *
     * @param bspline 待重参数化的B样条轨迹
     * @param start_end_derivative 起点和终点的导数约束（速度、加速度）
     * @param ratio 时间缩放比例因子
     * @param ctrl_pts 输出的控制点矩阵
     * @param dt 输出的时间间隔
     * @param time_inc 输出的时间增量
     */
    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    /**
     * @brief 轨迹细化优化算法
     * @details 对初始轨迹进行迭代优化，在满足动力学约束的前提下，
     *          优化轨迹的平滑性、安全性和时间最优性
     *
     * @param traj 输入的初始B样条轨迹
     * @param start_end_derivative 起点和终点的导数约束
     * @param ratio 时间缩放比例
     * @param ts 输出的优化后时间间隔
     * @param optimal_control_points 输出的优化后控制点矩阵
     * @return bool 优化是否成功
     */
    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing - 开发中的功能

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr; ///< 智能指针类型定义，方便内存管理

    // !SECTION
  };
} // namespace ego_planner

#endif