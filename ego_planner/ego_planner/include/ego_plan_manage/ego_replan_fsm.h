/**
 * @file ego_replan_fsm.h
 * @brief EGO-Planner重规划有限状态机头文件
 *
 * 该文件定义了EGO-Planner的重规划有限状态机(FSM)，用于管理无人机的轨迹规划状态转换和执行流程。
 * FSM负责协调路径规划、碰撞检测、紧急停止等核心功能。
 */

#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <ego_bspline_opt/bspline_optimizer.h>
#include <ego_plan_env/grid_map.h>
#include <ego_planner/Bspline.h>
#include <ego_planner/DataDisp.h>
#include <ego_plan_manage/planner_manager.h>
#include <ego_traj_utils/planning_visualization.h>

using std::vector;

namespace ego_planner
{

  /**
   * @class EGOReplanFSM
   * @brief EGO-Planner重规划有限状态机类
   *
   * 该类实现了基于有限状态机的轨迹重规划系统，负责管理无人机从初始化、等待目标、
   * 轨迹生成、重规划到执行的完整流程。支持手动目标、预设目标和参考路径三种目标类型。
   */
  class EGOReplanFSM
  {

  private:
    /* ---------- 状态标志 ---------- */
    /**
     * @enum FSM_EXEC_STATE
     * @brief 有限状态机执行状态枚举
     *
     * 定义了FSM的六个主要状态：
     * - INIT: 初始化状态，系统启动时的状态
     * - WAIT_TARGET: 等待目标状态，等待接收目标点
     * - GEN_NEW_TRAJ: 生成新轨迹状态，为新目标生成初始轨迹
     * - REPLAN_TRAJ: 重规划轨迹状态，对现有轨迹进行重新规划
     * - EXEC_TRAJ: 执行轨迹状态，正在执行当前轨迹
     * - EMERGENCY_STOP: 紧急停止状态，检测到碰撞风险时触发
     */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };

    /**
     * @enum TARGET_TYPE
     * @brief 目标类型枚举
     *
     * 定义了三种目标输入方式：
     * - MANUAL_TARGET: 手动选择目标，通过RViz等工具手动指定
     * - PRESET_TARGET: 预设目标，通过硬编码的路点序列
     * - REFENCE_PATH: 参考路径，跟随给定的参考轨迹
     */
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* ---------- 规划工具 ---------- */
    EGOPlannerManager::Ptr planner_manager_;        // 规划管理器指针，负责路径搜索和轨迹优化
    PlanningVisualization::Ptr visualization_;      // 可视化工具指针，用于在RViz中显示轨迹和路径
    ego_planner::DataDisp data_disp_;               // 数据显示对象，用于发布规划数据供调试和分析

    /* ---------- 参数配置 ---------- */
    int target_type_;                               // 目标类型：1-手动选择，2-硬编码预设，3-参考路径
    double no_replan_thresh_;                       // 不触发重规划的阈值，当偏离轨迹小于该值时不重规划
    double replan_thresh_;                          // 触发重规划的阈值，当偏离轨迹超过该值时必须重规划
    double waypoints_[50][3];                       // 预设路点数组，最多存储50个三维路点坐标
    int waypoint_num_;                              // 实际使用的路点数量
    double planning_horizen_;                       // 规划视野范围（米），局部规划的最大距离
    double planning_horizen_time_;                  // 规划时间视野（秒），局部规划的时间长度
    double emergency_time_;                         // 紧急停止时间（秒），检测到碰撞后的制动时间

    /* ---------- 规划数据 ---------- */
    bool trigger_;                                  // 触发标志，指示是否触发规划
    bool have_target_;                              // 目标标志，指示是否已接收到目标点
    bool have_odom_;                                // 里程计标志，指示是否已接收到里程计数据
    bool have_new_target_;                          // 新目标标志，指示是否接收到新的目标点
    FSM_EXEC_STATE exec_state_;                     // 当前FSM执行状态
    int continously_called_times_{0};               // 连续调用次数计数器，用于检测状态稳定性

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;// 里程计状态：位置、速度、加速度
    Eigen::Quaterniond odom_orient_;                // 里程计姿态四元数

    Eigen::Vector3d init_pt_;                       // 初始点位置
    Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_; // 起始状态：位置、速度、加速度、偏航角
    Eigen::Vector3d end_pt_, end_vel_;              // 目标状态：终点位置和速度
    Eigen::Vector3d local_target_pt_, local_target_vel_; // 局部目标状态：局部目标点位置和速度
    int current_wp_;                                // 当前路点索引，用于多路点导航

    bool flag_escape_emergency_;                    // 脱离紧急状态标志，指示是否已从紧急停止状态恢复

    /* ---------- ROS接口 ---------- */
    ros::NodeHandle node_;                          // ROS节点句柄
    ros::Timer exec_timer_;                         // FSM执行定时器，周期性触发状态机更新
    ros::Timer safety_timer_;                       // 安全检查定时器，周期性检测碰撞风险
    ros::Subscriber waypoint_sub_;                  // 路点订阅器，接收目标路点消息
    ros::Subscriber odom_sub_;                      // 里程计订阅器，接收位置和速度信息
    ros::Publisher replan_pub_;                     // 重规划发布器，发布重规划轨迹
    ros::Publisher new_pub_;                        // 新轨迹发布器，发布新生成的轨迹
    ros::Publisher bspline_pub_;                    // B样条发布器，发布B样条轨迹
    ros::Publisher data_disp_pub_;                  // 数据显示发布器，发布调试和分析数据

    /* ---------- 辅助函数 ---------- */
    /**
     * @brief 调用反弹重规划方法
     * @param flag_use_poly_init 是否使用多项式初始化轨迹
     * @param flag_randomPolyTraj 是否使用随机多项式轨迹
     * @return 规划是否成功
     *
     * 该函数结合前端路径搜索和后端轨迹优化，生成避障轨迹。
     * 当检测到当前轨迹不可行时调用此函数进行重规划。
     */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);

    /**
     * @brief 调用紧急停止
     * @param stop_pos 停止位置
     * @return 紧急停止是否成功执行
     *
     * 在检测到即将发生碰撞时，生成紧急制动轨迹，使无人机在指定位置安全停止。
     */
    bool callEmergencyStop(Eigen::Vector3d stop_pos);

    /**
     * @brief 从当前轨迹进行规划
     * @return 规划是否成功
     *
     * 基于当前执行的轨迹进行局部重规划，保持轨迹的连续性和平滑性。
     */
    bool planFromCurrentTraj();

    /**
     * @brief 改变FSM执行状态
     * @param new_state 新的FSM状态
     * @param pos_call 调用位置标识（用于调试）
     *
     * 切换FSM状态并记录状态转换信息，用于状态机的调试和监控。
     */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);

    /**
     * @brief 获取连续状态调用次数
     * @return 返回pair<连续调用次数, 连续调用的状态>
     *
     * 统计同一状态被连续调用的次数，用于检测状态机是否陷入某个状态。
     */
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    /**
     * @brief 打印FSM执行状态
     *
     * 输出当前FSM状态信息到控制台，用于调试和监控。
     */
    void printFSMExecState();

    /**
     * @brief 根据给定路点规划全局轨迹
     *
     * 使用预设或接收的路点序列，规划从起点到终点经过所有路点的全局轨迹。
     */
    void planGlobalTrajbyGivenWps();

    /**
     * @brief 获取局部目标点
     *
     * 从全局路径中提取当前规划视野内的局部目标点，用于局部轨迹规划。
     */
    void getLocalTarget();

    /* ---------- ROS回调函数 ---------- */
    /**
     * @brief FSM执行回调函数
     * @param e 定时器事件
     *
     * 周期性调用的主状态机更新函数，根据当前状态执行相应的规划逻辑。
     * 包括状态转换、轨迹生成、重规划等核心功能。
     */
    void execFSMCallback(const ros::TimerEvent &e);

    /**
     * @brief 碰撞检测回调函数
     * @param e 定时器事件
     *
     * 周期性检查当前轨迹是否会发生碰撞，如果检测到碰撞风险则触发紧急停止。
     */
    void checkCollisionCallback(const ros::TimerEvent &e);

    /**
     * @brief 路点回调函数
     * @param msg 路径消息指针
     *
     * 接收目标路点或路径消息，更新目标点和路点序列。
     */
    void waypointCallback(const nav_msgs::PathConstPtr &msg);

    /**
     * @brief 里程计回调函数
     * @param msg 里程计消息指针
     *
     * 接收里程计数据，更新无人机当前的位置、速度、加速度和姿态信息。
     */
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);

    /**
     * @brief 检查碰撞
     * @return 如果检测到碰撞返回true，否则返回false
     *
     * 检查当前轨迹是否与障碍物发生碰撞。
     */
    bool checkCollision();

  public:
    /**
     * @brief 构造函数
     *
     * 初始化EGO-Planner重规划FSM对象。
     */
    EGOReplanFSM(/* args */)
    {
    }

    /**
     * @brief 析构函数
     *
     * 清理EGO-Planner重规划FSM对象资源。
     */
    ~EGOReplanFSM()
    {
    }

    /**
     * @brief 初始化函数
     * @param nh ROS节点句柄
     *
     * 初始化FSM，包括：
     * - 加载参数配置
     * - 初始化规划管理器和可视化工具
     * - 设置ROS订阅器、发布器和定时器
     * - 设置初始FSM状态
     */
    void init(ros::NodeHandle &nh);

    /**
     * @brief Eigen内存对齐宏
     *
     * 确保类中的Eigen成员变量正确对齐，避免在使用SIMD指令时出现段错误。
     */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif