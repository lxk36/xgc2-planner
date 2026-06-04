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



#ifndef _ASTAR_H
#define _ASTAR_H

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
// #include "grad_spline/sdf_map.h"
#include "fast_plan_env/edt_environment.h"
#include <boost/functional/hash.hpp>
#include <queue>
namespace fast_planner {
// #define REACH_HORIZON 1
// #define REACH_END 2
// #define NO_PATH 3
#define IN_CLOSE_SET 'a'  // 节点在闭集中
#define IN_OPEN_SET 'b'   // 节点在开集中
#define NOT_EXPAND 'c'    // 节点未被扩展
#define inf 1 >> 30       // 无穷大值

/**
 * @brief A*算法的节点类
 *
 * 该类表示A*路径搜索算法中的一个节点，包含节点的位置、代价、父节点等信息。
 * 支持静态和动态（时间相关）的A*搜索。
 */
class Node {
public:
  /* -------------------- */
  Eigen::Vector3i index;     // 节点在网格地图中的索引坐标(i, j, k)
  Eigen::Vector3d position;  // 节点在世界坐标系中的实际位置(x, y, z)
  double g_score, f_score;   // g_score: 从起点到当前节点的实际代价; f_score: f = g + h，总代价估计
  Node* parent;              // 指向父节点的指针，用于路径回溯
  char node_state;           // 节点状态：IN_CLOSE_SET, IN_OPEN_SET, 或 NOT_EXPAND

  double time;     // 动态A*中的时间戳
  int time_idx;    // 时间离散化后的索引

  /* -------------------- */
  /**
   * @brief 构造函数
   * 初始化节点，将父节点设为NULL，状态设为未扩展
   */
  Node() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~Node(){};
};
typedef Node* NodePtr;  // 节点指针类型别名

/**
 * @brief 节点比较器，用于优先队列
 *
 * 在优先队列中，f_score较小的节点具有更高的优先级（最小堆）
 */
class NodeComparator0 {
public:
  /**
   * @brief 比较两个节点的f_score
   * @param node1 第一个节点
   * @param node2 第二个节点
   * @return 如果node1的f_score大于node2，返回true（用于构建最小堆）
   */
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

/**
 * @brief Eigen矩阵的哈希函数模板
 *
 * 用于将Eigen矩阵（如Vector3i, Vector4i）作为unordered_map的键。
 * 使用标准的哈希组合方法生成哈希值。
 *
 * @tparam T Eigen矩阵类型
 */
template <typename T>
struct matrix_hash0 : std::unary_function<T, size_t> {
  /**
   * @brief 计算Eigen矩阵的哈希值
   * @param matrix 输入的Eigen矩阵
   * @return 计算得到的哈希值
   */
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

/**
 * @brief 节点哈希表类
 *
 * 用于快速存储和查找已扩展的节点。
 * 支持3D空间搜索（仅位置）和4D时空搜索（位置+时间）。
 */
class NodeHashTable0 {
private:
  /* data */
  std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash0<Eigen::Vector3i>> data_3d_;  // 3D空间节点哈希表
  std::unordered_map<Eigen::Vector4i, NodePtr, matrix_hash0<Eigen::Vector4i>> data_4d_;  // 4D时空节点哈希表

public:
  /**
   * @brief 构造函数
   */
  NodeHashTable0(/* args */) {
  }

  /**
   * @brief 析构函数
   */
  ~NodeHashTable0() {
  }

  /**
   * @brief 插入3D节点到哈希表
   * @param idx 节点的3D索引
   * @param node 节点指针
   */
  void insert(Eigen::Vector3i idx, NodePtr node) {
    data_3d_.insert(make_pair(idx, node));
  }

  /**
   * @brief 插入4D时空节点到哈希表
   * @param idx 节点的3D空间索引
   * @param time_idx 时间索引
   * @param node 节点指针
   */
  void insert(Eigen::Vector3i idx, int time_idx, NodePtr node) {
    data_4d_.insert(make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  /**
   * @brief 在3D哈希表中查找节点
   * @param idx 节点的3D索引
   * @return 找到的节点指针，如果不存在则返回NULL
   */
  NodePtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }

  /**
   * @brief 在4D时空哈希表中查找节点
   * @param idx 节点的3D空间索引
   * @param time_idx 时间索引
   * @return 找到的节点指针，如果不存在则返回NULL
   */
  NodePtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  /**
   * @brief 清空所有哈希表
   */
  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }
};

/**
 * @brief A*路径搜索算法类
 *
 * 实现了经典的A*算法用于网格地图中的路径规划。
 * 支持静态环境下的3D路径搜索和动态环境下的4D时空路径搜索。
 * 使用欧几里得距离变换(EDT)地图进行碰撞检测。
 */
class Astar {
private:
  /* ---------- main data structure ---------- */
  vector<NodePtr> path_node_pool_;  // 节点内存池，预分配节点以提高效率
  int use_node_num_, iter_num_;     // use_node_num_: 当前使用的节点数量; iter_num_: 迭代次数
  NodeHashTable0 expanded_nodes_;   // 已扩展节点的哈希表，用于快速查找
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;  // 开集优先队列，按f_score排序
  std::vector<NodePtr> path_nodes_;  // 最终路径的节点序列

  /* ---------- record data ---------- */
  EDTEnvironment::Ptr edt_environment_;  // EDT环境指针，提供地图和碰撞检测
  bool has_path_ = false;                // 标志是否找到有效路径

  /* ---------- parameter ---------- */
  /* search */
  double lambda_heu_;    // 启发式函数权重系数
  double margin_;        // 安全裕度，避免路径过于接近障碍物
  int allocate_num_;     // 节点池预分配数量
  double tie_breaker_;   // 打破平局的小扰动系数
  /* map */
  double resolution_, inv_resolution_;                      // 地图分辨率及其倒数
  double time_resolution_, inv_time_resolution_;            // 时间分辨率及其倒数（用于动态搜索）
  Eigen::Vector3d origin_, map_size_3d_;                    // 地图原点和3D尺寸
  double time_origin_;                                      // 时间原点（用于动态搜索）

  /* helper */
  /**
   * @brief 将世界坐标转换为网格索引
   * @param pt 世界坐标系中的位置
   * @return 对应的网格索引
   */
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);

  /**
   * @brief 将时间戳转换为时间索引
   * @param time 时间戳
   * @return 离散化后的时间索引
   */
  int timeToIndex(double time);

  /**
   * @brief 从终点回溯路径
   * @param end_node 终点节点指针
   */
  void retrievePath(NodePtr end_node);

  /* heuristic function */
  /**
   * @brief 对角距离启发式函数
   * @param x1 起点位置
   * @param x2 终点位置
   * @return 启发式代价估计
   */
  double getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);

  /**
   * @brief 曼哈顿距离启发式函数
   * @param x1 起点位置
   * @param x2 终点位置
   * @return 启发式代价估计
   */
  double getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);

  /**
   * @brief 欧几里得距离启发式函数
   * @param x1 起点位置
   * @param x2 终点位置
   * @return 启发式代价估计
   */
  double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);

public:
  /**
   * @brief 默认构造函数
   */
  Astar(){};

  /**
   * @brief 析构函数
   */
  ~Astar();

  /**
   * @brief 搜索结果枚举
   */
  enum { REACH_END = 1, NO_PATH = 2 };  // REACH_END: 成功到达终点; NO_PATH: 无法找到路径

  /* main API */
  /**
   * @brief 从ROS参数服务器设置参数
   * @param nh ROS节点句柄
   */
  void setParam(ros::NodeHandle& nh);

  /**
   * @brief 初始化A*搜索器
   * 预分配节点内存池等资源
   */
  void init();

  /**
   * @brief 重置搜索器状态
   * 清空开集、闭集和路径，准备下一次搜索
   */
  void reset();

  /**
   * @brief 执行A*路径搜索
   * @param start_pt 起点的世界坐标
   * @param end_pt 终点的世界坐标
   * @param dynamic 是否执行动态搜索（考虑时间维度），默认为false
   * @param time_start 起始时间（仅在dynamic=true时使用），默认为-1.0
   * @return 搜索结果：REACH_END表示成功，NO_PATH表示失败
   */
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool dynamic = false,
             double time_start = -1.0);

  /**
   * @brief 设置环境指针
   * @param env EDT环境的智能指针
   */
  void setEnvironment(const EDTEnvironment::Ptr& env);

  /**
   * @brief 获取搜索到的路径
   * @return 路径点的向量（世界坐标）
   */
  std::vector<Eigen::Vector3d> getPath();

  /**
   * @brief 获取已访问的节点列表
   * @return 已访问节点的向量（用于可视化）
   */
  std::vector<NodePtr> getVisitedNodes();

  typedef shared_ptr<Astar> Ptr;  // A*搜索器的智能指针类型
};

}  // namespace fast_planner

#endif