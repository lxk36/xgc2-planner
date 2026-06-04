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



#include <fast_path_searching/astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace fast_planner {
/**
 * @brief 析构函数，释放预分配的节点内存池
 *
 * 遍历所有预分配的节点，逐一删除以避免内存泄漏
 */
Astar::~Astar() {
  for (int i = 0; i < allocate_num_; i++) {
    delete path_node_pool_[i];
  }
}

/**
 * @brief A*路径搜索主函数
 *
 * @param start_pt 起始点的三维位置
 * @param end_pt 目标点的三维位置
 * @param dynamic 是否为动态障碍物环境（考虑时间维度）
 * @param time_start 搜索起始时间（仅在dynamic=true时有效）
 * @return int 搜索结果状态：REACH_END（成功）或NO_PATH（失败）
 *
 * 算法流程：
 * 1. 初始化起始节点，设置g_score=0，计算启发式函数f_score
 * 2. 将起始节点加入open_set优先队列
 * 3. 循环处理open_set中的节点，每次取出f_score最小的节点
 * 4. 对当前节点进行26邻域扩展（3D空间）
 * 5. 检查扩展节点的可行性（地图范围、碰撞检测、close set）
 * 6. 更新或创建扩展节点，加入open_set
 * 7. 直到找到目标点或open_set为空
 */
int Astar::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool dynamic, double time_start) {
  /* ---------- initialize ---------- */
  // 初始化起始节点
  NodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;

  Eigen::Vector3d end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;

  // 计算目标点索引
  end_index = posToIndex(end_pt);
  // 计算启发式代价：f = g + h，这里g=0，所以f=h
  cur_node->f_score = lambda_heu_ * getEuclHeu(cur_node->position, end_pt);
  cur_node->node_state = IN_OPEN_SET;

  // 将起始节点加入优先队列
  open_set_.push(cur_node);
  use_node_num_ += 1;

  // 处理动态障碍物场景（时间-空间搜索）
  if (dynamic) {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
    // cout << "time start: " << time_start << endl;
  } else
    expanded_nodes_.insert(cur_node->index, cur_node);

  NodePtr neighbor = NULL;
  NodePtr terminate_node = NULL;

  /* ---------- search loop ---------- */
  // A*主搜索循环，直到找到路径或open_set为空
  while (!open_set_.empty()) {
    /* ---------- get lowest f_score node ---------- */
    // 从优先队列中取出f_score最小的节点（最有希望的节点）
    cur_node = open_set_.top();
    // cout << "pos: " << cur_node->state.head(3).transpose() << endl;
    // cout << "time: " << cur_node->time << endl;
    // cout << "dist: " <<
    // edt_environment_->evaluateCoarseEDT(cur_node->state.head(3),
    // cur_node->time) <<
    // endl;

    /* ---------- determine termination ---------- */

    // 检查是否到达目标点（允许1个网格的误差）
    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 && abs(cur_node->index(2) - end_index(2)) <= 1;

    if (reach_end) {
      // cout << "[Astar]:---------------------- " << use_node_num_ << endl;
      // cout << "use node num: " << use_node_num_ << endl;
      // cout << "iter num: " << iter_num_ << endl;
      // 到达目标，回溯路径并返回成功
      terminate_node = cur_node;
      retrievePath(terminate_node);
      has_path_ = true;

      return REACH_END;
    }

    /* ---------- pop node and add to close set ---------- */
    // 将当前节点从open_set移除，加入close_set（已扩展集合）
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    /* ---------- init neighbor expansion ---------- */

    // 获取当前节点位置，准备进行邻域扩展
    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d pro_pos;
    double pro_t;

    vector<Eigen::Vector3d> inputs;
    Eigen::Vector3d d_pos;

    /* ---------- expansion loop ---------- */
    // 对当前节点进行26邻域扩展（3x3x3-1=26个方向）
    for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
      for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
        for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
          d_pos << dx, dy, dz;

          // 跳过当前节点本身（位移为0）
          if (d_pos.norm() < 1e-3) continue;

          // 计算扩展节点的位置
          pro_pos = cur_pos + d_pos;

          /* ---------- check if in feasible space ---------- */
          /* inside map range */
          // 检查扩展节点是否在地图范围内
          if (pro_pos(0) <= origin_(0) || pro_pos(0) >= map_size_3d_(0) || pro_pos(1) <= origin_(1) ||
              pro_pos(1) >= map_size_3d_(1) || pro_pos(2) <= origin_(2) ||
              pro_pos(2) >= map_size_3d_(2)) {
            // cout << "outside map" << endl;
            continue;
          }

          /* not in close set */
          // 计算扩展节点的索引
          Eigen::Vector3i pro_id = posToIndex(pro_pos);
          int pro_t_id = timeToIndex(pro_t);

          // 在已扩展节点表中查找该节点
          NodePtr pro_node =
              dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);

          // 如果节点已经在close_set中，则跳过（已经找到更优路径）
          if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
            // cout << "in closeset" << endl;
            continue;
          }

          /* collision free */
          // double dist = dynamic ?
          // edt_environment_->evaluateCoarseEDT(pro_pos, cur_node->time + dt) :
          //                         edt_environment_->evaluateCoarseEDT(pro_pos,
          //                         -1.0);
          // 使用欧几里得距离变换(EDT)进行碰撞检测
          double dist = edt_environment_->evaluateCoarseEDT(pro_pos, -1.0);
          // 如果距离障碍物小于安全边距，则跳过（有碰撞风险）
          if (dist <= margin_) {
            continue;
          }

          /* ---------- compute cost ---------- */
          // 计算扩展节点的代价函数
          double time_to_goal, tmp_g_score, tmp_f_score;
          // g_score: 从起点到当前节点的实际代价（欧几里得距离的平方）
          tmp_g_score = d_pos.squaredNorm() + cur_node->g_score;
          // f_score: 总代价估计 = g_score + h_score（启发式函数）
          tmp_f_score = tmp_g_score + lambda_heu_ * getEuclHeu(pro_pos, end_pt);

          // 情况1: 扩展节点是首次访问（未在expanded_nodes中）
          if (pro_node == NULL) {
            // 从节点池中分配新节点
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->position = pro_pos;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            // 动态场景下需要设置时间戳
            if (dynamic) {
              pro_node->time = cur_node->time + 1.0;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            // 将新节点加入优先队列
            open_set_.push(pro_node);

            // 将节点加入已扩展表
            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            use_node_num_ += 1;
            // 检查节点池是否用尽
            if (use_node_num_ == allocate_num_) {
              cout << "run out of memory." << endl;
              return NO_PATH;
            }
          } else if (pro_node->node_state == IN_OPEN_SET) {
            // 情况2: 扩展节点已在open_set中，检查是否需要更新（找到更短路径）
            if (tmp_g_score < pro_node->g_score) {
              // pro_node->index = pro_id;
              // 更新节点信息为更优路径
              pro_node->position = pro_pos;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->parent = cur_node;
              if (dynamic) pro_node->time = cur_node->time + 1.0;
            }
          } else {
            // 情况3: 异常状态（节点状态不应为其他值）
            cout << "error type in searching: " << pro_node->node_state << endl;
          }

          /* ----------  ---------- */
        }
  }

  /* ---------- open set empty, no path ---------- */
  // open_set为空但未找到路径，说明无解
  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

/**
 * @brief 从ROS参数服务器加载A*算法参数
 *
 * @param nh ROS节点句柄
 *
 * 参数说明：
 * - resolution_astar: 搜索分辨率（网格大小）
 * - time_resolution: 时间分辨率（动态场景）
 * - lambda_heu: 启发式函数权重
 * - margin: 与障碍物的安全边距
 * - allocate_num: 预分配节点池大小
 * - tie_breaker: 打破相同f值的tie，避免搜索对称性问题
 */
void Astar::setParam(ros::NodeHandle& nh) {
  nh.param("astar/resolution_astar", resolution_, -1.0);
  nh.param("astar/time_resolution", time_resolution_, -1.0);
  nh.param("astar/lambda_heu", lambda_heu_, -1.0);
  nh.param("astar/margin", margin_, -1.0);
  nh.param("astar/allocate_num", allocate_num_, -1);
  tie_breaker_ = 1.0 + 1.0 / 10000;  // 微小的tie_breaker可以打破对称性

  cout << "margin:" << margin_ << endl;
}

/**
 * @brief 从终点节点回溯到起点，构建完整路径
 *
 * @param end_node 目标节点指针
 *
 * 通过parent指针从终点回溯到起点，然后反转得到从起点到终点的路径
 */
void Astar::retrievePath(NodePtr end_node) {
  NodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  // 沿着parent指针回溯，直到到达起点（parent为NULL）
  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  // 反转路径，使其从起点到终点
  reverse(path_nodes_.begin(), path_nodes_.end());
}

/**
 * @brief 获取路径点的位置序列
 *
 * @return std::vector<Eigen::Vector3d> 从起点到终点的三维位置序列
 *
 * 将路径节点列表转换为三维坐标向量列表
 */
std::vector<Eigen::Vector3d> Astar::getPath() {
  vector<Eigen::Vector3d> path;
  for (int i = 0; i < path_nodes_.size(); ++i) {
    path.push_back(path_nodes_[i]->position);
  }
  return path;
}

/**
 * @brief 计算对角线启发式函数（Diagonal Heuristic）
 *
 * @param x1 起始点
 * @param x2 目标点
 * @return double 启发式代价估计
 *
 * 对角线启发式考虑了3D空间中的对角线移动：
 * - 3D对角线移动（dx=dy=dz）: 距离 = sqrt(3)
 * - 2D对角线移动（如dx=dy, dz=0）: 距离 = sqrt(2)
 * - 1D直线移动: 距离 = 1.0
 * 这种启发式函数比欧几里得距离更接近实际网格路径长度
 */
double Astar::getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));

  double h;
  // 计算可以进行3D对角线移动的步数（三个方向同时移动）
  int diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  // 根据剩余的方向数计算总代价
  if (dx < 1e-4) {
    // x方向已完成，剩余y和z方向
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < 1e-4) {
    // y方向已完成，剩余x和z方向
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < 1e-4) {
    // z方向已完成，剩余x和y方向
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return tie_breaker_ * h;
}

/**
 * @brief 计算曼哈顿距离启发式函数（Manhattan Heuristic）
 *
 * @param x1 起始点
 * @param x2 目标点
 * @return double 启发式代价估计
 *
 * 曼哈顿距离 = |dx| + |dy| + |dz|
 * 这是一个可容许的启发式函数，适用于只允许沿坐标轴移动的场景
 */
double Astar::getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));

  return tie_breaker_ * (dx + dy + dz);
}

/**
 * @brief 计算欧几里得距离启发式函数（Euclidean Heuristic）
 *
 * @param x1 起始点
 * @param x2 目标点
 * @return double 启发式代价估计
 *
 * 欧几里得距离 = ||x2 - x1||
 * 这是最常用的启发式函数，表示两点之间的直线距离
 * 是可容许的（admissible）启发式，保证A*找到最优路径
 */
double Astar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  return tie_breaker_ * (x2 - x1).norm();
}

/**
 * @brief 初始化A*搜索器
 *
 * 主要功能：
 * 1. 计算分辨率的倒数（用于快速坐标转索引）
 * 2. 获取地图的范围和尺寸
 * 3. 预分配节点内存池，避免搜索过程中的动态内存分配
 * 4. 初始化计数器
 */
void Astar::init() {
  /* ---------- map params ---------- */
  // 计算分辨率倒数，用于快速坐标到索引的转换
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
  // 从环境中获取地图的原点和尺寸
  edt_environment_->getMapRegion(origin_, map_size_3d_);

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  /* ---------- pre-allocated node ---------- */
  // 预分配节点池，提高搜索效率
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new Node;
  }

  // 初始化计数器
  use_node_num_ = 0;
  iter_num_ = 0;
}

/**
 * @brief 设置环境指针
 *
 * @param env 欧几里得距离变换(EDT)环境的智能指针
 *
 * EDT环境用于快速碰撞检测，通过查询到最近障碍物的距离来判断是否安全
 */
void Astar::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

/**
 * @brief 重置A*搜索器状态，准备下一次搜索
 *
 * 清空所有搜索状态，包括：
 * 1. 已扩展节点表
 * 2. 路径节点列表
 * 3. 优先队列（open_set）
 * 4. 重置所有已使用节点的状态
 * 5. 重置计数器
 */
void Astar::reset() {
  expanded_nodes_.clear();
  path_nodes_.clear();

  // 清空优先队列（通过swap技巧）
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
  open_set_.swap(empty_queue);

  // 重置所有已使用节点的状态
  for (int i = 0; i < use_node_num_; i++) {
    NodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  // 重置计数器
  use_node_num_ = 0;
  iter_num_ = 0;
}

/**
 * @brief 获取所有访问过的节点
 *
 * @return std::vector<NodePtr> 访问过的节点列表
 *
 * 用于可视化调试，可以查看A*算法的搜索过程
 */
std::vector<NodePtr> Astar::getVisitedNodes() {
  vector<NodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

/**
 * @brief 将三维坐标转换为网格索引
 *
 * @param pt 三维空间坐标
 * @return Eigen::Vector3i 网格索引
 *
 * 转换公式：idx = floor((pt - origin) / resolution)
 * 使用预计算的inv_resolution_加速除法运算
 */
Eigen::Vector3i Astar::posToIndex(Eigen::Vector3d pt) {
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

  // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
  // origin_(1)) * inv_resolution_),
  //     floor((pt(2) - origin_(2)) * inv_resolution_);

  return idx;
}

/**
 * @brief 将时间转换为时间索引
 *
 * @param time 时间戳
 * @return int 时间索引
 *
 * 用于动态障碍物场景，将连续时间离散化为时间网格索引
 * 转换公式：idx = floor((time - time_origin) / time_resolution)
 */
int Astar::timeToIndex(double time) {
  int idx = floor((time - time_origin_) * inv_time_resolution_);
}

}  // namespace fast_planner
