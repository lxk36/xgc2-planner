/**
 * @file kinodynamic_astar.h
 * @brief 动力学A*路径搜索算法的头文件
 *
 * 该文件定义了动力学约束下的A*路径搜索算法，考虑了机器人的速度、加速度等动力学特性
 * 相比传统A*，该算法能够生成满足动力学约束的可执行轨迹
 */

#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

// #include <fast_path_searching/matrix_hash.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include "fast_plan_env/edt_environment.h"

namespace fast_planner {
// #define REACH_HORIZON 1
// #define REACH_END 2
// #define NO_PATH 3
#define IN_CLOSE_SET 'a'   // 节点已在关闭集中（已扩展）
#define IN_OPEN_SET 'b'    // 节点已在开放集中（待扩展）
#define NOT_EXPAND 'c'     // 节点未被扩展
#define inf 1 >> 30        // 无穷大值定义

/**
 * @class PathNode
 * @brief 路径搜索节点类
 *
 * 存储A*搜索过程中的节点信息，包括位置、状态（位置+速度）、代价、输入控制等
 */
class PathNode {
 public:
  /* -------------------- */
  Eigen::Vector3i index;                // 节点在网格地图中的索引坐标(i,j,k)
  Eigen::Matrix<double, 6, 1> state;    // 节点的完整状态：[x, y, z, vx, vy, vz]，包含位置和速度
  double g_score, f_score;              // g_score: 从起点到当前节点的实际代价; f_score: 总代价(g+h)
  Eigen::Vector3d input;                // 输入控制量（加速度）：从父节点到当前节点的控制输入
  double duration;                      // 从父节点运动到当前节点所需的时间
  double time;                          // 当前节点的绝对时间（从起点开始计时）// dyn
  int time_idx;                         // 时间索引，用于时间离散化
  PathNode* parent;                     // 父节点指针，用于回溯路径
  char node_state;                      // 节点状态：IN_CLOSE_SET, IN_OPEN_SET, 或 NOT_EXPAND

  /* -------------------- */
  /**
   * @brief 构造函数
   *
   * 初始化父节点为空，节点状态为未扩展
   */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode* PathNodePtr;  // 路径节点指针类型定义

/**
 * @class NodeComparator
 * @brief 节点比较器类
 *
 * 用于优先队列中节点的比较，按照f_score从小到大排序
 * 实现最小堆，使得f_score最小的节点优先被扩展
 */
class NodeComparator {
 public:
  /**
   * @brief 节点比较操作符
   * @param node1 第一个节点
   * @param node2 第二个节点
   * @return 如果node1的f_score大于node2，返回true（用于构建最小堆）
   */
  bool operator()(PathNodePtr node1, PathNodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

/**
 * @struct matrix_hash
 * @brief 矩阵哈希函数模板
 *
 * 为Eigen矩阵类型提供哈希函数，使其能够作为unordered_map的键
 * 使用boost hash_combine算法生成哈希值
 *
 * @tparam T Eigen矩阵类型（如Vector3i, Vector4i等）
 */
template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  /**
   * @brief 哈希计算操作符
   * @param matrix 待计算哈希值的矩阵
   * @return 计算得到的哈希值
   *
   * 遍历矩阵的所有元素，使用boost的哈希组合算法生成唯一的哈希值
   * 0x9e3779b9是黄金分割比的近似值，用于更好的哈希分布
   */
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

/**
 * @class NodeHashTable
 * @brief 节点哈希表类
 *
 * 用于快速查找已访问的节点，支持3D空间索引和4D时空索引
 * 3D索引用于静态环境，4D索引用于动态环境（增加时间维度）
 */
class NodeHashTable {
 private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;  // 3D空间哈希表，键为(x,y,z)索引
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>>
      data_4d_;  // 4D时空哈希表，键为(x,y,z,t)索引

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}

  /**
   * @brief 插入节点到3D哈希表
   * @param idx 节点的3D空间索引
   * @param node 节点指针
   */
  void insert(Eigen::Vector3i idx, PathNodePtr node) {
    data_3d_.insert(std::make_pair(idx, node));
  }

  /**
   * @brief 插入节点到4D哈希表
   * @param idx 节点的3D空间索引
   * @param time_idx 节点的时间索引
   * @param node 节点指针
   */
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node) {
    data_4d_.insert(std::make_pair(
        Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  /**
   * @brief 从3D哈希表查找节点
   * @param idx 节点的3D空间索引
   * @return 找到的节点指针，未找到返回NULL
   */
  PathNodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }

  /**
   * @brief 从4D哈希表查找节点
   * @param idx 节点的3D空间索引
   * @param time_idx 节点的时间索引
   * @return 找到的节点指针，未找到返回NULL
   */
  PathNodePtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter =
        data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  /**
   * @brief 清空哈希表
   *
   * 清除3D和4D哈希表中的所有数据
   */
  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }
};

/**
 * @class KinodynamicAstar
 * @brief 动力学A*路径搜索算法核心类
 *
 * 实现考虑动力学约束的A*路径搜索算法，能够生成满足速度、加速度约束的可执行轨迹
 * 支持静态和动态环境下的路径规划，使用状态转移矩阵进行前向积分
 */
class KinodynamicAstar {
 private:
  /* ---------- main data structure ---------- */
  vector<PathNodePtr> path_node_pool_;  // 节点内存池，预分配节点避免频繁内存分配
  int use_node_num_, iter_num_;         // use_node_num_: 已使用的节点数; iter_num_: 迭代次数
  NodeHashTable expanded_nodes_;        // 已扩展节点的哈希表，用于快速查重
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator>
      open_set_;                        // 开放集，按f_score排序的优先队列
  std::vector<PathNodePtr> path_nodes_; // 最终路径的节点序列

  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;  // 起点速度、终点速度、起点加速度
  Eigen::Matrix<double, 6, 6> phi_;                  // 状态转移矩阵，用于前向积分 // state transit matrix
  // shared_ptr<SDFMap> sdf_map;
  EDTEnvironment::Ptr edt_environment_;              // EDT（欧几里得距离变换）环境指针，用于碰撞检测
  bool is_shot_succ_ = false;                        // one-shot轨迹是否成功（直接连接起点和终点）
  Eigen::MatrixXd coef_shot_;                        // one-shot轨迹的多项式系数
  double t_shot_;                                    // one-shot轨迹的时间
  bool has_path_ = false;                            // 是否找到可行路径

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_, init_max_tau_;       // 最大时间步长和初始最大时间步长
  double max_vel_, max_acc_;            // 最大速度和最大加速度约束
  double w_time_, horizon_, lambda_heu_;// w_time_: 时间权重; horizon_: 规划时域; lambda_heu_: 启发式权重
  int allocate_num_, check_num_;        // allocate_num_: 预分配节点数; check_num_: 碰撞检测采样数
  double tie_breaker_;                  // tie-breaker系数，用于打破f值相同的节点
  bool optimistic_;                     // 是否使用乐观启发式（不考虑障碍物）

  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
                                        // 空间分辨率及其倒数、时间分辨率及其倒数
  Eigen::Vector3d origin_, map_size_3d_;// 地图原点和地图尺寸
  double time_origin_;                  // 时间原点

  /* helper */
  /**
   * @brief 将连续位置转换为网格索引
   * @param pt 连续空间中的位置坐标
   * @return 对应的网格索引
   */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);

  /**
   * @brief 将连续时间转换为时间索引
   * @param time 连续时间
   * @return 离散化的时间索引
   */
  int timeToIndex(double time);

  /**
   * @brief 从终点节点回溯提取完整路径
   * @param end_node 终点节点指针
   *
   * 通过父节点指针反向遍历，构建从起点到终点的完整路径
   */
  void retrievePath(PathNodePtr end_node);

  /* shot trajectory */
  /**
   * @brief 求解三次方程
   * @param a,b,c,d 三次方程的系数: ax^3 + bx^2 + cx + d = 0
   * @return 方程的实数解集合
   */
  vector<double> cubic(double a, double b, double c, double d);

  /**
   * @brief 求解四次方程
   * @param a,b,c,d,e 四次方程的系数: ax^4 + bx^3 + cx^2 + dx + e = 0
   * @return 方程的实数解集合
   */
  vector<double> quartic(double a, double b, double c, double d, double e);

  /**
   * @brief 计算one-shot轨迹（直接连接起点和终点的最优轨迹）
   * @param state1 起点状态（位置+速度）
   * @param state2 终点状态（位置+速度）
   * @param time_to_goal 到达目标的时间
   * @return 是否成功计算出满足约束的轨迹
   *
   * 使用闭式解计算满足边界条件和动力学约束的多项式轨迹
   */
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                       double time_to_goal);

  /**
   * @brief 估计启发式代价（从x1到x2的最优时间和代价）
   * @param x1 起始状态
   * @param x2 目标状态
   * @param optimal_time 输出参数，最优时间
   * @return 估计的启发式代价
   *
   * 考虑动力学约束计算理论最优时间，作为A*的启发式函数
   */
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                           double& optimal_time);

  /* state propagation */
  /**
   * @brief 状态前向传播（运动学积分）
   * @param state0 初始状态 [x,y,z,vx,vy,vz]
   * @param state1 输出参数，传播后的状态
   * @param um 控制输入（加速度）
   * @param tau 时间步长
   *
   * 使用状态转移矩阵phi_进行前向积分，计算在给定控制输入下的下一状态
   */
  void stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                    Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um,
                    double tau);

 public:
  KinodynamicAstar(){};
  ~KinodynamicAstar();

  /**
   * @enum 搜索结果状态枚举
   */
  enum { REACH_HORIZON = 1,  // 到达规划时域边界
         REACH_END = 2,       // 到达目标点
         NO_PATH = 3,         // 未找到路径
         NEAR_END = 4 };      // 接近目标点

  /* main API */
  /**
   * @brief 从ROS参数服务器设置算法参数
   * @param nh ROS节点句柄
   *
   * 读取max_tau, init_max_tau, max_vel, max_acc, w_time等参数
   */
  void setParam(ros::NodeHandle& nh);

  /**
   * @brief 初始化算法
   *
   * 预分配节点内存池，初始化状态转移矩阵等
   */
  void init();

  /**
   * @brief 重置搜索状态
   *
   * 清空开放集、关闭集等，为下一次搜索做准备
   */
  void reset();

  /**
   * @brief 执行动力学A*路径搜索
   * @param start_pt 起点位置
   * @param start_vel 起点速度
   * @param start_acc 起点加速度
   * @param end_pt 终点位置
   * @param end_vel 终点速度
   * @param init 是否为初始化搜索
   * @param dynamic 是否为动态环境（考虑时间维度）
   * @param time_start 搜索起始时间（动态环境下使用）
   * @return 搜索结果状态（REACH_END, REACH_HORIZON, NO_PATH等）
   *
   * 核心搜索函数，考虑动力学约束进行A*搜索，生成可执行轨迹
   */
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
             Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
             Eigen::Vector3d end_vel, bool init, bool dynamic = false,
             double time_start = -1.0);

  /**
   * @brief 设置环境指针
   * @param env EDT环境指针
   *
   * 设置用于碰撞检测的环境模型
   */
  void setEnvironment(const EDTEnvironment::Ptr& env);

  /**
   * @brief 获取动力学轨迹的位置采样点
   * @param delta_t 采样时间间隔
   * @return 轨迹位置点序列
   *
   * 按照指定的时间间隔对路径进行采样，返回位置序列
   */
  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);

  /**
   * @brief 获取路径采样点和边界导数信息
   * @param ts 输出参数，采样时间间隔
   * @param point_set 输出参数，路径点集合
   * @param start_end_derivatives 输出参数，起点和终点的导数信息（速度、加速度）
   *
   * 用于后续的轨迹优化，提供初始路径和边界条件
   */
  void getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                  vector<Eigen::Vector3d>& start_end_derivatives);

  /**
   * @brief 获取所有访问过的节点
   * @return 访问节点列表
   *
   * 用于可视化搜索过程，调试算法
   */
  std::vector<PathNodePtr> getVisitedNodes();

  typedef shared_ptr<KinodynamicAstar> Ptr;  // 智能指针类型定义

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif