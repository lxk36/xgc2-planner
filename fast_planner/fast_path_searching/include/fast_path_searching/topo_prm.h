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



#ifndef _TOPO_PRM_H
#define _TOPO_PRM_H

#include <fast_plan_env/edt_environment.h>
#include <fast_plan_env/raycast.h>
#include <random>

namespace fast_planner {

/* ---------- used for iterating all topo combination ---------- */
/**
 * @brief 拓扑路径组合迭代器类
 *
 * 该类用于遍历所有可能的拓扑路径组合。当有多个路径段，每段有不同的可选路径时，
 * 该迭代器可以系统地遍历所有可能的组合方式
 */
class TopoIterator {
private:
  /* data */
  vector<int> path_nums_;    // 每个路径段的可选路径数量
  vector<int> cur_index_;    // 当前组合中每个路径段选择的路径索引
  int combine_num_;          // 所有可能的组合总数
  int cur_num_;              // 当前已遍历的组合数量

  /**
   * @brief 递增指定位的索引（类似进位操作）
   * @param bit_num 要递增的位数
   *
   * 当某一位达到最大值时，该位归零并向高位进位
   */
  void increase(int bit_num) {
    cur_index_[bit_num] += 1;
    if (cur_index_[bit_num] >= path_nums_[bit_num]) {
      cur_index_[bit_num] = 0;
      increase(bit_num + 1);
    }
  }

public:
  /**
   * @brief 构造函数
   * @param pn 每个路径段的可选路径数量向量
   *
   * 初始化迭代器，计算所有可能的组合总数
   */
  TopoIterator(vector<int> pn) {
    path_nums_ = pn;
    cur_index_.resize(path_nums_.size());
    fill(cur_index_.begin(), cur_index_.end(), 0);
    cur_num_ = 0;

    combine_num_ = 1;
    for (int i = 0; i < path_nums_.size(); ++i) {
      combine_num_ *= path_nums_[i] > 0 ? path_nums_[i] : 1;
    }
    std::cout << "[Topo]: merged path num: " << combine_num_ << std::endl;
  }

  /**
   * @brief 默认构造函数
   */
  TopoIterator() {
  }

  /**
   * @brief 析构函数
   */
  ~TopoIterator() {
  }

  /**
   * @brief 获取下一个组合的索引
   * @param index 输出参数，存储当前组合的索引
   * @return 如果还有下一个组合返回true，否则返回false
   *
   * 该函数返回当前组合索引，并将迭代器移动到下一个组合
   */
  bool nextIndex(vector<int>& index) {
    index = cur_index_;
    cur_num_ += 1;

    if (cur_num_ == combine_num_) return false;

    // go to next combination
    increase(0);
    return true;
  }
};

/* ---------- node of topo graph ---------- */
/**
 * @brief 拓扑图节点类
 *
 * 该类表示拓扑路线图(PRM)中的节点，包括Guard节点（关键点）和Connector节点（连接点）
 */
class GraphNode {
private:
  /* data */

public:
  /**
   * @brief 节点类型枚举
   */
  enum NODE_TYPE { Guard = 1,      // Guard节点：关键点，通常位于环境的关键位置
                   Connector = 2 };  // Connector节点：连接点，用于连接Guard节点

  /**
   * @brief 节点状态枚举（用于图搜索算法）
   */
  enum NODE_STATE { NEW = 1,    // 新节点：尚未访问
                    CLOSE = 2,  // 关闭节点：已完成访问
                    OPEN = 3 };  // 开放节点：已访问但未完成

  /**
   * @brief 默认构造函数
   */
  GraphNode(/* args */) {
  }

  /**
   * @brief 构造函数
   * @param pos 节点的3D位置
   * @param type 节点类型（Guard或Connector）
   * @param id 节点的唯一标识符
   */
  GraphNode(Eigen::Vector3d pos, NODE_TYPE type, int id) {
    pos_ = pos;
    type_ = type;
    state_ = NEW;
    id_ = id;
  }

  /**
   * @brief 析构函数
   */
  ~GraphNode() {
  }

  vector<shared_ptr<GraphNode>> neighbors_;  // 邻居节点列表
  Eigen::Vector3d pos_;                      // 节点的3D位置坐标
  NODE_TYPE type_;                           // 节点类型
  NODE_STATE state_;                         // 节点状态（用于搜索）
  int id_;                                   // 节点唯一标识符

  typedef shared_ptr<GraphNode> Ptr;
};

/**
 * @brief 拓扑概率路线图(Topology PRM)类
 *
 * 该类实现基于概率路线图的拓扑路径规划算法。通过在配置空间中随机采样并构建路线图，
 * 能够找到多条具有不同拓扑结构的路径，适用于复杂环境下的路径规划
 */
class TopologyPRM {
private:
  /* data */
  EDTEnvironment::Ptr edt_environment_;  // 环境表示（欧几里得距离变换）

  // sampling generator
  random_device rd_;                           // 随机数设备
  default_random_engine eng_;                  // 随机数引擎
  uniform_real_distribution<double> rand_pos_;  // 均匀分布随机数生成器

  Eigen::Vector3d sample_r_;     // 采样区域半径
  Eigen::Vector3d translation_;  // 采样区域平移向量
  Eigen::Matrix3d rotation_;     // 采样区域旋转矩阵

  // roadmap data structure, 0:start, 1:goal, 2-n: others
  list<GraphNode::Ptr> graph_;                    // 路线图，节点0为起点，节点1为终点，其余为采样点
  vector<vector<Eigen::Vector3d>> raw_paths_;     // 原始路径集合
  vector<vector<Eigen::Vector3d>> short_paths_;   // 短路径集合（经过shortcut优化）
  vector<vector<Eigen::Vector3d>> final_paths_;   // 最终路径集合（经过剪枝和筛选）
  vector<Eigen::Vector3d> start_pts_, end_pts_;   // 起点和终点集合

  // raycasting
  vector<RayCaster> casters_;   // 射线投射器集合，用于可见性检测
  Eigen::Vector3d offset_;      // 偏移量

  // parameter
  double max_sample_time_;         // 最大采样时间（秒）
  int max_sample_num_;             // 最大采样点数量
  int max_raw_path_, max_raw_path2_;  // 最大原始路径数量的限制参数
  int short_cut_num_;              // shortcut优化的迭代次数
  Eigen::Vector3d sample_inflate_;  // 采样区域膨胀参数
  double resolution_;              // 地图分辨率

  double ratio_to_short_;  // 用于shortcut优化的比例参数
  int reserve_num_;        // 保留路径的数量

  bool parallel_shortcut_;  // 是否并行执行shortcut优化

  /* create topological roadmap */
  /* path searching, shortening, pruning and merging */
  /**
   * @brief 创建拓扑路线图
   * @param start 起点位置
   * @param end 终点位置
   * @return 构建的路线图节点列表
   *
   * 通过随机采样和可见性检测构建连接起点和终点的路线图
   */
  list<GraphNode::Ptr> createGraph(Eigen::Vector3d start, Eigen::Vector3d end);

  /**
   * @brief 在路线图中搜索路径
   * @return 搜索到的所有原始路径集合
   *
   * 使用深度优先搜索或其他图搜索算法找到从起点到终点的所有可能路径
   */
  vector<vector<Eigen::Vector3d>> searchPaths();

  /**
   * @brief 对路径进行shortcut优化
   *
   * 通过尝试直接连接路径上的非相邻点来缩短路径长度
   */
  void shortcutPaths();

  /**
   * @brief 剪除拓扑等价的路径
   * @param paths 待剪枝的路径集合
   * @return 剪枝后的路径集合
   *
   * 移除那些具有相同拓扑结构的冗余路径，只保留拓扑不同的路径
   */
  vector<vector<Eigen::Vector3d>> pruneEquivalent(vector<vector<Eigen::Vector3d>>& paths);

  /**
   * @brief 选择较短的路径
   * @param paths 待选择的路径集合
   * @param step 选择步长参数
   * @return 选择出的短路径集合
   *
   * 根据路径长度进行筛选，保留较短的路径
   */
  vector<vector<Eigen::Vector3d>> selectShortPaths(vector<vector<Eigen::Vector3d>>& paths, int step);

  /* ---------- helper ---------- */
  /**
   * @brief 在采样区域内生成随机采样点
   * @return 随机生成的3D采样点
   */
  inline Eigen::Vector3d getSample();

  /**
   * @brief 查找与给定点可见的Guard节点
   * @param pt 查询点位置
   * @return 与该点可见的Guard节点列表
   *
   * 通过射线检测判断哪些Guard节点与给定点之间没有障碍物遮挡
   */
  vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector3d pt);

  /**
   * @brief 测试是否需要在两个Guard节点之间建立新连接
   * @param g1 第一个Guard节点
   * @param g2 第二个Guard节点
   * @param pt 待添加的Connector节点位置
   * @return 如果需要建立连接返回true，否则返回false
   *
   * 检测是否已存在冗余的连接，避免重复连接
   */
  bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt);

  /**
   * @brief 检测两点之间的直线可见性（无碰撞）
   * @param p1 起点
   * @param p2 终点
   * @param thresh 安全距离阈值
   * @param pc 输出参数，碰撞点位置（如果发生碰撞）
   * @param caster_id 使用的射线投射器ID，默认为0
   * @return 如果两点之间可见返回true，否则返回false
   */
  bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh,
                 Eigen::Vector3d& pc, int caster_id = 0);

  /**
   * @brief 检测三角形区域的可见性
   * @param pt 顶点1
   * @param p1 顶点2
   * @param p2 顶点3
   * @return 如果三角形区域内可见返回true，否则返回false
   */
  bool triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2);

  /**
   * @brief 剪除路线图中的冗余节点和边
   *
   * 移除不连通的节点或对路径规划无贡献的节点
   */
  void pruneGraph();

  /**
   * @brief 深度优先搜索遍历图
   * @param vis 输出参数，存储访问过的节点列表
   */
  void depthFirstSearch(vector<GraphNode::Ptr>& vis);

  /**
   * @brief 将线段离散化为一系列点
   * @param p1 线段起点
   * @param p2 线段终点
   * @return 离散化后的点序列
   */
  vector<Eigen::Vector3d> discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2);

  /**
   * @brief 将多条路径离散化
   * @param path 待离散化的路径集合
   * @return 离散化后的路径集合
   */
  vector<vector<Eigen::Vector3d>> discretizePaths(vector<vector<Eigen::Vector3d>>& path);

  /**
   * @brief 将单条路径离散化
   * @param path 待离散化的路径
   * @return 离散化后的路径
   */
  vector<Eigen::Vector3d> discretizePath(vector<Eigen::Vector3d> path);

  /**
   * @brief 对单条路径进行shortcut优化
   * @param path 待优化的路径
   * @param path_id 路径ID
   * @param iter_num 迭代次数，默认为1
   *
   * 通过多次迭代尝试缩短路径
   */
  void shortcutPath(vector<Eigen::Vector3d> path, int path_id, int iter_num = 1);

  /**
   * @brief 将路径离散化为指定数量的点
   * @param path 待离散化的路径
   * @param pt_num 离散化后的点数量
   * @return 离散化后的路径
   */
  vector<Eigen::Vector3d> discretizePath(const vector<Eigen::Vector3d>& path, int pt_num);

  /**
   * @brief 判断两条路径是否具有相同的拓扑结构
   * @param path1 第一条路径
   * @param path2 第二条路径
   * @param thresh 判断阈值
   * @return 如果拓扑结构相同返回true，否则返回false
   *
   * 通过比较路径与障碍物的相对位置关系来判断拓扑等价性
   */
  bool sameTopoPath(const vector<Eigen::Vector3d>& path1, const vector<Eigen::Vector3d>& path2,
                    double thresh);

  /**
   * @brief 获取路径的正交点（用于拓扑判断）
   * @param path 输入路径
   * @return 正交点位置
   */
  Eigen::Vector3d getOrthoPoint(const vector<Eigen::Vector3d>& path);

  /**
   * @brief 找出最短路径的索引
   * @param paths 路径集合
   * @return 最短路径的索引
   */
  int shortestPath(vector<vector<Eigen::Vector3d>>& paths);

public:
  double clearance_;  // 路径与障碍物的安全间隙

  /**
   * @brief 构造函数
   */
  TopologyPRM(/* args */);

  /**
   * @brief 析构函数
   */
  ~TopologyPRM();

  /**
   * @brief 初始化函数
   * @param nh ROS节点句柄，用于读取参数
   *
   * 从ROS参数服务器读取配置参数并初始化内部变量
   */
  void init(ros::NodeHandle& nh);

  /**
   * @brief 设置环境表示
   * @param env 欧几里得距离变换环境指针
   */
  void setEnvironment(const EDTEnvironment::Ptr& env);

  /**
   * @brief 查找拓扑不同的路径
   * @param start 起点位置
   * @param end 终点位置
   * @param start_pts 起点附近的辅助点集合
   * @param end_pts 终点附近的辅助点集合
   * @param graph 输出参数，构建的路线图
   * @param raw_paths 输出参数，搜索得到的原始路径集合
   * @param filtered_paths 输出参数，经过shortcut优化的路径集合
   * @param select_paths 输出参数，最终筛选出的路径集合
   *
   * 这是主要的接口函数，执行完整的拓扑路径规划流程：
   * 1. 构建路线图
   * 2. 搜索路径
   * 3. 优化路径
   * 4. 剪枝和筛选
   */
  void findTopoPaths(Eigen::Vector3d start, Eigen::Vector3d end, vector<Eigen::Vector3d> start_pts,
                     vector<Eigen::Vector3d> end_pts, list<GraphNode::Ptr>& graph,
                     vector<vector<Eigen::Vector3d>>& raw_paths,
                     vector<vector<Eigen::Vector3d>>& filtered_paths,
                     vector<vector<Eigen::Vector3d>>& select_paths);

  /**
   * @brief 计算路径长度
   * @param path 输入路径
   * @return 路径的总长度
   */
  double pathLength(const vector<Eigen::Vector3d>& path);

  /**
   * @brief 将路径转换为引导点
   * @param path 输入路径
   * @param pt_num 引导点的数量
   * @return 引导点序列
   *
   * 从路径中提取指定数量的关键点，用于后续的轨迹优化
   */
  vector<Eigen::Vector3d> pathToGuidePts(vector<Eigen::Vector3d>& path, int pt_num);

};

}  // namespace fast_planner

#endif