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



#include <fast_path_searching/topo_prm.h>
#include <thread>

namespace fast_planner {
/**
 * @brief 默认构造函数
 */
TopologyPRM::TopologyPRM(/* args */) {}

/**
 * @brief 析构函数
 */
TopologyPRM::~TopologyPRM() {}

/**
 * @brief 初始化拓扑PRM规划器
 * @param nh ROS节点句柄，用于读取参数
 *
 * 主要功能：
 * 1. 初始化随机数生成器
 * 2. 从参数服务器加载配置参数
 * 3. 初始化射线投射器数组
 */
void TopologyPRM::init(ros::NodeHandle& nh) {
  graph_.clear();  // 清空拓扑图
  eng_ = default_random_engine(rd_());  // 初始化随机数引擎
  rand_pos_ = uniform_real_distribution<double>(-1.0, 1.0);  // 初始化随机位置分布(-1.0到1.0)

  // 从ROS参数服务器读取配置参数
  nh.param("topo_prm/sample_inflate_x", sample_inflate_(0), -1.0);  // X方向采样膨胀范围
  nh.param("topo_prm/sample_inflate_y", sample_inflate_(1), -1.0);  // Y方向采样膨胀范围
  nh.param("topo_prm/sample_inflate_z", sample_inflate_(2), -1.0);  // Z方向采样膨胀范围
  nh.param("topo_prm/clearance", clearance_, -1.0);  // 最小安全距离
  nh.param("topo_prm/short_cut_num", short_cut_num_, -1);  // 路径简化迭代次数
  nh.param("topo_prm/reserve_num", reserve_num_, -1);  // 保留的路径数量
  nh.param("topo_prm/ratio_to_short", ratio_to_short_, -1.0);  // 路径长度与最短路径的比值阈值
  nh.param("topo_prm/max_sample_num", max_sample_num_, -1);  // 最大采样点数量
  nh.param("topo_prm/max_sample_time", max_sample_time_, -1.0);  // 最大采样时间(秒)
  nh.param("topo_prm/max_raw_path", max_raw_path_, -1);  // 最大原始路径数量
  nh.param("topo_prm/max_raw_path2", max_raw_path2_, -1);  // 第二阶段最大原始路径数量
  nh.param("topo_prm/parallel_shortcut", parallel_shortcut_, false);  // 是否启用并行路径简化
  resolution_ = edt_environment_->sdf_map_->getResolution();  // 获取SDF地图分辨率
  offset_ = Eigen::Vector3d(0.5, 0.5, 0.5) - edt_environment_->sdf_map_->getOrigin() / resolution_;  // 计算索引偏移量

  // 为每条原始路径预分配一个射线投射器，用于可见性检测
  for (int i = 0; i < max_raw_path_; ++i) {
    casters_.push_back(RayCaster());
  }
}

/**
 * @brief 寻找拓扑不同的路径集合
 * @param start 起点位置
 * @param end 终点位置
 * @param start_pts 起点附近的路径点序列
 * @param end_pts 终点附近的路径点序列
 * @param graph 输出的拓扑图结构
 * @param raw_paths 输出的原始路径集合
 * @param filtered_paths 输出的过滤后路径集合
 * @param select_paths 输出的最终选择的路径集合
 *
 * 算法流程：
 * 1. 创建拓扑PRM图，通过随机采样和可见性检测构建路径网络
 * 2. 在图中搜索从起点到终点的所有可行路径
 * 3. 对每条路径进行简化优化，去除冗余路径点
 * 4. 剪除拓扑等价的路径，只保留拓扑不同的路径
 * 5. 选择最短的若干条路径作为最终输出
 */
void TopologyPRM::findTopoPaths(Eigen::Vector3d start, Eigen::Vector3d end,
                                vector<Eigen::Vector3d> start_pts, vector<Eigen::Vector3d> end_pts,
                                list<GraphNode::Ptr>& graph, vector<vector<Eigen::Vector3d>>& raw_paths,
                                vector<vector<Eigen::Vector3d>>& filtered_paths,
                                vector<vector<Eigen::Vector3d>>& select_paths) {
  ros::Time t1, t2;

  double graph_time, search_time, short_time, prune_time, select_time;
  /* ---------- 创建拓扑图 ---------- */
  t1 = ros::Time::now();

  start_pts_ = start_pts;  // 保存起点路径段
  end_pts_ = end_pts;      // 保存终点路径段

  graph = createGraph(start, end);  // 创建拓扑PRM图

  graph_time = (ros::Time::now() - t1).toSec();

  /* ---------- 在图中搜索路径 ---------- */
  t1 = ros::Time::now();

  raw_paths = searchPaths();  // 使用深度优先搜索获取所有可行路径

  search_time = (ros::Time::now() - t1).toSec();

  /* ---------- 路径简化 ---------- */
  // 对于并行模式，结果保存在 short_paths_ 中
  t1 = ros::Time::now();

  shortcutPaths();  // 对所有路径进行简化优化

  short_time = (ros::Time::now() - t1).toSec();

  /* ---------- 剪除等价路径 ---------- */
  t1 = ros::Time::now();

  filtered_paths = pruneEquivalent(short_paths_);  // 移除拓扑等价的路径

  prune_time = (ros::Time::now() - t1).toSec();

  /* ---------- 选择N条最短路径 ---------- */
  t1 = ros::Time::now();

  select_paths = selectShortPaths(filtered_paths, 1);  // 选择最短的若干条路径

  select_time = (ros::Time::now() - t1).toSec();

  final_paths_ = select_paths;  // 保存最终路径

  double total_time = graph_time + search_time + short_time + prune_time + select_time;

  // 输出各阶段耗时统计
  std::cout << "\n[Topo]: total time: " << total_time << ", graph: " << graph_time
            << ", search: " << search_time << ", short: " << short_time << ", prune: " << prune_time
            << ", select: " << select_time << std::endl;
}

/**
 * @brief 创建拓扑PRM图
 * @param start 起点位置
 * @param end 终点位置
 * @return 返回构建的拓扑图节点列表
 *
 * 核心算法流程：
 * 1. 初始化起点和终点作为Guard节点
 * 2. 计算采样区域的坐标变换（以起点终点连线为主轴）
 * 3. 在采样区域内随机生成点，检测碰撞
 * 4. 对每个有效采样点，查找可见的Guard节点
 * 5. 根据可见Guard数量决定节点类型：
 *    - 0个：该点成为新的Guard节点
 *    - 2个：检查是否需要在两个Guard间添加Connector节点
 * 6. 剪枝移除无用节点
 */
list<GraphNode::Ptr> TopologyPRM::createGraph(Eigen::Vector3d start, Eigen::Vector3d end) {

  /* 初始化起点、终点和采样区域 */
  graph_.clear();  // 清空图结构

  // 创建起点和终点Guard节点（ID分别为0和1）
  GraphNode::Ptr start_node = GraphNode::Ptr(new GraphNode(start, GraphNode::Guard, 0));
  GraphNode::Ptr end_node = GraphNode::Ptr(new GraphNode(end, GraphNode::Guard, 1));

  graph_.push_back(start_node);
  graph_.push_back(end_node);

  // 定义采样区域：以起点终点连线中点为中心的椭球体
  sample_r_(0) = 0.5 * (end - start).norm() + sample_inflate_(0);  // X方向半径（主轴方向）
  sample_r_(1) = sample_inflate_(1);  // Y方向半径
  sample_r_(2) = sample_inflate_(2);  // Z方向半径

  // 计算坐标变换：将采样坐标系对齐到起点-终点方向
  translation_ = 0.5 * (start + end);  // 平移向量为起点终点中点

  Eigen::Vector3d xtf, ytf, ztf, downward(0, 0, -1);
  xtf = (end - translation_).normalized();  // X轴指向终点方向
  ytf = xtf.cross(downward).normalized();   // Y轴垂直于X轴和重力方向
  ztf = xtf.cross(ytf);                     // Z轴由右手定则确定

  rotation_.col(0) = xtf;  // 旋转矩阵列向量
  rotation_.col(1) = ytf;
  rotation_.col(2) = ztf;

  int node_id = 1;  // 节点ID计数器，从2开始（0和1已被起点终点占用）

  /* ---------- 主采样循环 ---------- */
  int sample_num = 0;
  double sample_time = 0.0;
  Eigen::Vector3d pt;
  ros::Time t1, t2;
  while (sample_time < max_sample_time_ && sample_num < max_sample_num_) {
    t1 = ros::Time::now();

    pt = getSample();  // 在采样区域内随机生成一个点
    ++sample_num;
    double dist;
    Eigen::Vector3d grad;
    // 检查采样点与障碍物的距离
    dist = edt_environment_->evaluateCoarseEDT(pt, -1.0);
    if (dist <= clearance_) {  // 距离障碍物太近，跳过此采样点
      sample_time += (ros::Time::now() - t1).toSec();
      continue;
    }

    /* 查找该采样点可见的Guard节点 */
    vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt);
    if (visib_guards.size() == 0) {
      // 没有可见的Guard，该采样点成为新的Guard节点
      GraphNode::Ptr guard = GraphNode::Ptr(new GraphNode(pt, GraphNode::Guard, ++node_id));
      graph_.push_back(guard);
    } else if (visib_guards.size() == 2) {
      /* 尝试在两个Guard之间添加新连接 */
      bool need_connect = needConnection(visib_guards[0], visib_guards[1], pt);
      if (!need_connect) {  // 已存在拓扑等价的连接，跳过
        sample_time += (ros::Time::now() - t1).toSec();
        continue;
      }
      // 需要新的连接，添加Connector节点
      GraphNode::Ptr connector = GraphNode::Ptr(new GraphNode(pt, GraphNode::Connector, ++node_id));
      graph_.push_back(connector);

      // 建立双向连接关系
      visib_guards[0]->neighbors_.push_back(connector);
      visib_guards[1]->neighbors_.push_back(connector);

      connector->neighbors_.push_back(visib_guards[0]);
      connector->neighbors_.push_back(visib_guards[1]);
    }

    sample_time += (ros::Time::now() - t1).toSec();
  }

  /* 输出采样统计信息 */
  std::cout << "[Topo]: sample num: " << sample_num;

  pruneGraph();  // 剪枝：移除邻居数小于2的无用节点

  return graph_;
}

/**
 * @brief 查找从给定点可见的Guard节点
 * @param pt 查询点位置
 * @return 可见的Guard节点列表（最多2个）
 *
 * 只检查Guard类型的节点，忽略Connector节点
 * 使用射线投射检测可见性，最多返回2个可见Guard
 */
vector<GraphNode::Ptr> TopologyPRM::findVisibGuard(Eigen::Vector3d pt) {
  vector<GraphNode::Ptr> visib_guards;
  Eigen::Vector3d pc;  // 碰撞点位置

  int visib_num = 0;

  /* 遍历图中所有节点，查找可见的Guard */
  for (list<GraphNode::Ptr>::iterator iter = graph_.begin(); iter != graph_.end(); ++iter) {
    if ((*iter)->type_ == GraphNode::Connector) continue;  // 跳过Connector节点

    // 检测从pt到当前节点的连线是否可见（无碰撞）
    if (lineVisib(pt, (*iter)->pos_, resolution_, pc)) {
      visib_guards.push_back((*iter));
      ++visib_num;
      if (visib_num > 2) break;  // 最多找2个可见Guard
    }
  }

  return visib_guards;
}

/**
 * @brief 判断两个Guard节点之间是否需要添加新连接
 * @param g1 第一个Guard节点
 * @param g2 第二个Guard节点
 * @param pt 候选连接点位置
 * @return true表示需要新连接，false表示已存在等价连接
 *
 * 检查逻辑：
 * 1. 构造经过候选点pt的新路径path1: g1 -> pt -> g2
 * 2. 遍历g1和g2的公共邻居节点
 * 3. 对每个公共邻居，构造现有路径path2: g1 -> neighbor -> g2
 * 4. 判断path1和path2是否拓扑等价
 * 5. 若等价且path1更短，则更新邻居位置；若等价则返回false
 * 6. 若所有现有连接都不等价，返回true表示需要新连接
 */
bool TopologyPRM::needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt) {
  vector<Eigen::Vector3d> path1(3), path2(3);
  path1[0] = g1->pos_;  // 新路径起点
  path1[1] = pt;        // 新路径中间点
  path1[2] = g2->pos_;  // 新路径终点

  path2[0] = g1->pos_;  // 现有路径起点
  path2[2] = g2->pos_;  // 现有路径终点

  vector<Eigen::Vector3d> connect_pts;
  bool has_connect = false;
  // 遍历g1和g2的所有邻居组合，寻找公共邻居（即现有连接）
  for (int i = 0; i < g1->neighbors_.size(); ++i) {
    for (int j = 0; j < g2->neighbors_.size(); ++j) {
      if (g1->neighbors_[i]->id_ == g2->neighbors_[j]->id_) {  // 找到公共邻居
        path2[1] = g1->neighbors_[i]->pos_;  // 现有路径的中间点
        bool same_topo = sameTopoPath(path1, path2, 0.0);  // 判断拓扑等价性
        if (same_topo) {
          // 若拓扑等价，检查新连接是否更短
          if (pathLength(path1) < pathLength(path2)) {
            g1->neighbors_[i]->pos_ = pt;  // 用更短的连接点替换原有位置
          }
          return false;  // 已存在等价连接，不需要新连接
        }
      }
    }
  }
  return true;  // 没有找到拓扑等价的连接，需要添加新连接
}

/**
 * @brief 在采样区域内生成随机采样点
 * @return 世界坐标系下的随机采样点
 *
 * 采样流程：
 * 1. 在局部坐标系（椭球体内）生成随机点
 * 2. 通过旋转和平移变换转换到世界坐标系
 */
Eigen::Vector3d TopologyPRM::getSample() {
  /* 在局部坐标系中采样 */
  Eigen::Vector3d pt;
  pt(0) = rand_pos_(eng_) * sample_r_(0);  // X方向：[-sample_r_(0), sample_r_(0)]
  pt(1) = rand_pos_(eng_) * sample_r_(1);  // Y方向：[-sample_r_(1), sample_r_(1)]
  pt(2) = rand_pos_(eng_) * sample_r_(2);  // Z方向：[-sample_r_(2), sample_r_(2)]

  // 变换到世界坐标系：先旋转再平移
  pt = rotation_ * pt + translation_;

  return pt;
}

/**
 * @brief 检测两点之间的连线是否可见（无碰撞）
 * @param p1 起点位置
 * @param p2 终点位置
 * @param thresh 距离阈值，小于此值视为碰撞
 * @param pc 输出参数，碰撞点位置
 * @param caster_id 射线投射器ID，默认为0
 * @return true表示连线可见，false表示存在碰撞
 *
 * 使用射线投射算法沿直线逐步检查：
 * 1. 将起点终点转换为体素坐标
 * 2. 沿射线逐步采样，检查每个采样点的SDF距离
 * 3. 若任一点距离小于阈值，返回false并记录碰撞点
 */
bool TopologyPRM::lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh,
                            Eigen::Vector3d& pc, int caster_id) {
  Eigen::Vector3d ray_pt;  // 射线上的采样点
  Eigen::Vector3i pt_id;   // 体素索引
  double dist;             // SDF距离值

  // 设置射线投射器的起点和终点（转换为体素坐标）
  casters_[caster_id].setInput(p1 / resolution_, p2 / resolution_);
  // 沿射线逐步遍历
  while (casters_[caster_id].step(ray_pt)) {
    // 转换为SDF地图索引
    pt_id(0) = ray_pt(0) + offset_(0);
    pt_id(1) = ray_pt(1) + offset_(1);
    pt_id(2) = ray_pt(2) + offset_(2);
    // 查询该点的SDF距离值
    dist = edt_environment_->sdf_map_->getDistance(pt_id);
    if (dist <= thresh) {  // 距离小于阈值，发生碰撞
      edt_environment_->sdf_map_->indexToPos(pt_id, pc);  // 记录碰撞点位置
      return false;
    }
  }
  return true;  // 射线全程无碰撞，连线可见
}

/**
 * @brief 剪枝：移除拓扑图中的无用节点
 *
 * 剪枝规则：
 * 1. 保留起点和终点（id <= 1）
 * 2. 移除邻居数 <= 1 的节点（孤立节点或叶节点）
 * 3. 移除节点时，同时从其他节点的邻居列表中删除
 * 4. 重复迭代直到没有可剪枝节点
 *
 * 这样可以确保最终图中每个节点（除起点终点外）至少有2个连接
 */
void TopologyPRM::pruneGraph() {
  /* 剪枝无用节点 */
  if (graph_.size() > 2) {
    for (list<GraphNode::Ptr>::iterator iter1 = graph_.begin();
         iter1 != graph_.end() && graph_.size() > 2; ++iter1) {
      if ((*iter1)->id_ <= 1) continue;  // 跳过起点和终点

      /* 核心剪枝逻辑 */
      if ((*iter1)->neighbors_.size() <= 1) {  // 邻居数 <= 1，需要删除
        // 从其他节点的邻居列表中删除该节点
        for (list<GraphNode::Ptr>::iterator iter2 = graph_.begin(); iter2 != graph_.end(); ++iter2) {
          for (vector<GraphNode::Ptr>::iterator it_nb = (*iter2)->neighbors_.begin();
               it_nb != (*iter2)->neighbors_.end(); ++it_nb) {
            if ((*it_nb)->id_ == (*iter1)->id_) {
              (*iter2)->neighbors_.erase(it_nb);
              break;
            }
          }
        }

        // 从图中删除该节点，并重新开始检查
        graph_.erase(iter1);
        iter1 = graph_.begin();
      }
    }
  }
}

/**
 * @brief 剪除拓扑等价的路径
 * @param paths 输入的路径集合
 * @return 剪除等价路径后的路径集合
 *
 * 算法流程：
 * 1. 保留第一条路径作为基准
 * 2. 对每条新路径，与已保留的所有路径比较拓扑等价性
 * 3. 若与任一已保留路径等价，则丢弃该路径
 * 4. 若与所有已保留路径都不等价，则保留该路径
 * 5. 返回拓扑不同的路径集合
 */
vector<vector<Eigen::Vector3d>> TopologyPRM::pruneEquivalent(vector<vector<Eigen::Vector3d>>& paths) {
  vector<vector<Eigen::Vector3d>> pruned_paths;
  if (paths.size() < 1) return pruned_paths;

  /* ---------- 剪除拓扑等价路径 ---------- */
  // 输出: pruned_paths
  vector<int> exist_paths_id;
  exist_paths_id.push_back(0);  // 保留第一条路径

  for (int i = 1; i < paths.size(); ++i) {
    // 与已保留的路径比较
    bool new_path = true;

    for (int j = 0; j < exist_paths_id.size(); ++j) {
      // 与某一条已保留路径比较拓扑等价性
      bool same_topo = sameTopoPath(paths[i], paths[exist_paths_id[j]], 0.0);

      if (same_topo) {  // 拓扑等价，丢弃该路径
        new_path = false;
        break;
      }
    }

    if (new_path) {  // 是新的拓扑类型，保留
      exist_paths_id.push_back(i);
    }
  }

  // 保存剪除后的路径
  for (int i = 0; i < exist_paths_id.size(); ++i) {
    pruned_paths.push_back(paths[exist_paths_id[i]]);
  }

  std::cout << ", pruned path num: " << pruned_paths.size();

  return pruned_paths;
}

/**
 * @brief 选择最短的若干条路径
 * @param paths 输入的路径集合
 * @param step 步长参数（未使用）
 * @return 选择的最短路径集合
 *
 * 算法流程：
 * 1. 按长度排序选择最短路径
 * 2. 后续路径长度需小于 ratio_to_short_ * 最短路径长度
 * 3. 最多保留 reserve_num_ 条路径
 * 4. 合并起点和终点路径段
 * 5. 对合并后的路径进行简化优化
 * 6. 再次剪除等价路径
 */
vector<vector<Eigen::Vector3d>> TopologyPRM::selectShortPaths(vector<vector<Eigen::Vector3d>>& paths,
                                                              int step) {
  /* ---------- 只保留最短的若干条路径 ---------- */
  vector<vector<Eigen::Vector3d>> short_paths;
  vector<Eigen::Vector3d> short_path;
  double min_len;

  for (int i = 0; i < reserve_num_ && paths.size() > 0; ++i) {
    int path_id = shortestPath(paths);  // 找到当前最短路径
    if (i == 0) {
      short_paths.push_back(paths[path_id]);
      min_len = pathLength(paths[path_id]);  // 记录最短路径长度
      paths.erase(paths.begin() + path_id);
    } else {
      double rat = pathLength(paths[path_id]) / min_len;  // 计算与最短路径的长度比
      if (rat < ratio_to_short_) {  // 长度比满足要求
        short_paths.push_back(paths[path_id]);
        paths.erase(paths.begin() + path_id);
      } else {
        break;  // 后续路径都太长，终止选择
      }
    }
  }
  std::cout << ", select path num: " << short_paths.size();

  /* ---------- 合并起点和终点路径段 ---------- */
  for (int i = 0; i < short_paths.size(); ++i) {
    short_paths[i].insert(short_paths[i].begin(), start_pts_.begin(), start_pts_.end());  // 添加起点段
    short_paths[i].insert(short_paths[i].end(), end_pts_.begin(), end_pts_.end());        // 添加终点段
  }
  // 对合并后的路径进行简化（5次迭代）
  for (int i = 0; i < short_paths.size(); ++i) {
    shortcutPath(short_paths[i], i, 5);
    short_paths[i] = short_paths_[i];
  }

  // 再次剪除等价路径（合并后可能产生新的等价路径）
  short_paths = pruneEquivalent(short_paths);

  return short_paths;
}

/**
 * @brief 判断两条路径是否拓扑等价
 * @param path1 第一条路径
 * @param path2 第二条路径
 * @param thresh 可见性检测的距离阈值
 * @return true表示拓扑等价，false表示拓扑不同
 *
 * 拓扑等价性判断原理：
 * 1. 将两条路径等间隔离散化为相同数量的点
 * 2. 检查对应点之间的连线是否全部可见（无碰撞）
 * 3. 若所有对应点连线都可见，则两条路径拓扑等价
 * 4. 若存在任一连线被障碍物阻挡，则拓扑不同
 */
bool TopologyPRM::sameTopoPath(const vector<Eigen::Vector3d>& path1,
                               const vector<Eigen::Vector3d>& path2, double thresh) {
  // 计算两条路径的长度
  double len1 = pathLength(path1);
  double len2 = pathLength(path2);

  double max_len = max(len1, len2);  // 取较长的长度

  int pt_num = ceil(max_len / resolution_);  // 根据分辨率计算离散点数量

  // 将两条路径离散化为相同数量的点
  vector<Eigen::Vector3d> pts1 = discretizePath(path1, pt_num);
  vector<Eigen::Vector3d> pts2 = discretizePath(path2, pt_num);

  Eigen::Vector3d pc;  // 碰撞点
  // 检查对应点之间的连线可见性
  for (int i = 0; i < pt_num; ++i) {
    if (!lineVisib(pts1[i], pts2[i], thresh, pc)) {  // 连线被障碍物阻挡
      return false;  // 拓扑不同
    }
  }

  return true;  // 所有对应点连线都可见，拓扑等价
}

/**
 * @brief 找到路径集合中最短的路径
 * @param paths 路径集合
 * @return 最短路径的索引
 */
int TopologyPRM::shortestPath(vector<vector<Eigen::Vector3d>>& paths) {
  int short_id = -1;
  double min_len = 100000000;
  for (int i = 0; i < paths.size(); ++i) {
    double len = pathLength(paths[i]);
    if (len < min_len) {
      short_id = i;
      min_len = len;
    }
  }
  return short_id;
}

/**
 * @brief 计算路径的总长度
 * @param path 路径点序列
 * @return 路径总长度
 */
double TopologyPRM::pathLength(const vector<Eigen::Vector3d>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;

  // 累加所有相邻点之间的欧氏距离
  for (int i = 0; i < path.size() - 1; ++i) {
    length += (path[i + 1] - path[i]).norm();
  }
  return length;
}

/**
 * @brief 将路径等间隔离散化为指定数量的点
 * @param path 输入路径
 * @param pt_num 离散点数量
 * @return 离散化后的路径点序列
 *
 * 离散化算法：
 * 1. 计算路径上每个关键点的累积长度
 * 2. 将路径总长度等分为 pt_num 段
 * 3. 对每个等分点，在累积长度数组中查找所在区间
 * 4. 使用线性插值计算等分点的精确位置
 */
vector<Eigen::Vector3d> TopologyPRM::discretizePath(const vector<Eigen::Vector3d>& path, int pt_num) {
  vector<double> len_list;
  len_list.push_back(0.0);  // 起点累积长度为0

  // 计算每个路径点的累积长度
  for (int i = 0; i < path.size() - 1; ++i) {
    double inc_l = (path[i + 1] - path[i]).norm();  // 线段增量长度
    len_list.push_back(inc_l + len_list[i]);        // 累积长度
  }

  // 沿路径计算 pt_num 个等间隔点
  double len_total = len_list.back();              // 路径总长度
  double dl = len_total / double(pt_num - 1);      // 等间隔长度
  double cur_l;

  vector<Eigen::Vector3d> dis_path;
  for (int i = 0; i < pt_num; ++i) {
    cur_l = double(i) * dl;  // 当前目标长度

    // 查找当前长度所在的线段区间
    int idx = -1;
    for (int j = 0; j < len_list.size() - 1; ++j) {
      if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
        idx = j;
        break;
      }
    }

    // 计算插值系数并进行线性插值
    double lambda = (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
    Eigen::Vector3d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
    dis_path.push_back(inter_pt);
  }

  return dis_path;
}

/**
 * @brief 将路径转换为引导点序列
 * @param path 输入路径
 * @param pt_num 引导点数量
 * @return 引导点序列
 */
vector<Eigen::Vector3d> TopologyPRM::pathToGuidePts(vector<Eigen::Vector3d>& path, int pt_num) {
  return discretizePath(path, pt_num);
}

/**
 * @brief 对单条路径进行简化优化
 * @param path 输入路径
 * @param path_id 路径ID，用于索引结果数组
 * @param iter_num 迭代次数，默认为1
 *
 * 路径简化算法（贪心可见性简化）：
 * 1. 将路径离散化为密集点序列
 * 2. 从起点开始，尽可能跳过中间点直接连接可见的远点
 * 3. 若连线碰撞，则在碰撞点附近沿梯度方向推离障碍物
 * 4. 重复迭代直到路径不再变短或达到最大迭代次数
 */
void TopologyPRM::shortcutPath(vector<Eigen::Vector3d> path, int path_id, int iter_num) {
  vector<Eigen::Vector3d> short_path = path;
  vector<Eigen::Vector3d> last_path;

  for (int k = 0; k < iter_num; ++k) {
    last_path = short_path;

    vector<Eigen::Vector3d> dis_path = discretizePath(short_path);  // 离散化路径

    if (dis_path.size() < 2) {
      short_paths_[path_id] = dis_path;
      return;
    }

    /* 基于可见性的路径简化 */
    Eigen::Vector3d colli_pt, grad, dir, push_dir;
    double dist;
    short_path.clear();
    short_path.push_back(dis_path.front());  // 添加起点
    for (int i = 1; i < dis_path.size(); ++i) {
      // 检查从当前路径末点到第i个点是否可见
      if (lineVisib(short_path.back(), dis_path[i], resolution_, colli_pt, path_id)) continue;

      // 不可见，需要添加中间点；在碰撞点处沿梯度推离障碍物
      edt_environment_->evaluateEDTWithGrad(colli_pt, -1, dist, grad);
      if (grad.norm() > 1e-3) {
        grad.normalize();
        dir = (dis_path[i] - short_path.back()).normalized();
        push_dir = grad - grad.dot(dir) * dir;  // 梯度在垂直于前进方向的分量
        push_dir.normalize();
        colli_pt = colli_pt + resolution_ * push_dir;  // 沿推离方向偏移
      }
      short_path.push_back(colli_pt);
    }
    short_path.push_back(dis_path.back());  // 添加终点

    /* 若简化后路径更长，则终止迭代 */
    double len1 = pathLength(last_path);
    double len2 = pathLength(short_path);
    if (len2 > len1) {
      short_path = last_path;  // 恢复上一次结果
      break;
    }
  }

  short_paths_[path_id] = short_path;  // 保存简化后的路径
}

/**
 * @brief 对所有原始路径进行简化优化
 *
 * 支持两种模式：
 * 1. 并行模式：为每条路径创建独立线程进行简化，加快处理速度
 * 2. 串行模式：依次简化每条路径
 */
void TopologyPRM::shortcutPaths() {
  short_paths_.resize(raw_paths_.size());  // 预分配结果数组

  if (parallel_shortcut_) {
    // 并行简化：为每条路径创建一个线程
    vector<thread> short_threads;
    for (int i = 0; i < raw_paths_.size(); ++i) {
      short_threads.push_back(thread(&TopologyPRM::shortcutPath, this, raw_paths_[i], i, 1));
    }
    // 等待所有线程完成
    for (int i = 0; i < raw_paths_.size(); ++i) {
      short_threads[i].join();
    }
  } else {
    // 串行简化
    for (int i = 0; i < raw_paths_.size(); ++i) shortcutPath(raw_paths_[i], i);
  }
}

/**
 * @brief 将线段离散化为点序列
 * @param p1 起点
 * @param p2 终点
 * @return 离散化的点序列
 *
 * 根据分辨率将线段等间隔离散化，间隔为resolution_
 */
vector<Eigen::Vector3d> TopologyPRM::discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2) {
  Eigen::Vector3d dir = p2 - p1;
  double len = dir.norm();
  int seg_num = ceil(len / resolution_);  // 根据分辨率计算段数

  vector<Eigen::Vector3d> line_pts;
  if (seg_num <= 0) {
    return line_pts;
  }

  // 生成等间隔点序列
  for (int i = 0; i <= seg_num; ++i) line_pts.push_back(p1 + dir * double(i) / double(seg_num));

  return line_pts;
}

/**
 * @brief 将路径离散化为点序列（重载版本）
 * @param path 输入路径
 * @return 离散化的点序列
 *
 * 对路径的每条线段进行离散化，并合并为连续点序列
 */
vector<Eigen::Vector3d> TopologyPRM::discretizePath(vector<Eigen::Vector3d> path) {
  vector<Eigen::Vector3d> dis_path, segment;

  if (path.size() < 2) {
    ROS_ERROR("what path? ");
    return dis_path;
  }

  // 逐段离散化
  for (int i = 0; i < path.size() - 1; ++i) {
    segment = discretizeLine(path[i], path[i + 1]);

    if (segment.size() < 1) continue;

    dis_path.insert(dis_path.end(), segment.begin(), segment.end());
    if (i != path.size() - 2) dis_path.pop_back();  // 避免重复点（除了最后一段）
  }
  return dis_path;
}

/**
 * @brief 批量离散化多条路径
 * @param path 路径集合
 * @return 离散化后的路径集合
 */
vector<vector<Eigen::Vector3d>> TopologyPRM::discretizePaths(vector<vector<Eigen::Vector3d>>& path) {
  vector<vector<Eigen::Vector3d>> dis_paths;
  vector<Eigen::Vector3d> dis_path;

  for (int i = 0; i < path.size(); ++i) {
    dis_path = discretizePath(path[i]);

    if (dis_path.size() > 0) dis_paths.push_back(dis_path);
  }

  return dis_paths;
}

/**
 * @brief 获取路径的正交点（最垂直于起点终点连线的点）
 * @param path 输入路径
 * @return 正交点位置
 *
 * 算法流程：
 * 1. 计算起点终点连线方向
 * 2. 计算起点终点中点
 * 3. 遍历路径中间点，找到与连线方向最垂直的点
 * 4. 垂直性通过点向量与连线方向的余弦值最小来判断
 */
Eigen::Vector3d TopologyPRM::getOrthoPoint(const vector<Eigen::Vector3d>& path) {
  Eigen::Vector3d x1 = path.front();  // 起点
  Eigen::Vector3d x2 = path.back();   // 终点

  Eigen::Vector3d dir = (x2 - x1).normalized();  // 起点终点连线方向
  Eigen::Vector3d mid = 0.5 * (x1 + x2);         // 中点

  double min_cos = 1000.0;
  Eigen::Vector3d pdir;
  Eigen::Vector3d ortho_pt;

  // 遍历路径中间点，找到最垂直的点
  for (int i = 1; i < path.size() - 1; ++i) {
    pdir = (path[i] - mid).normalized();  // 从中点到当前点的方向
    double cos = fabs(pdir.dot(dir));     // 与连线方向的余弦值（越小越垂直）

    if (cos < min_cos) {
      min_cos = cos;
      ortho_pt = path[i];  // 更新正交点
    }
  }

  return ortho_pt;
}

/**
 * @brief 使用深度优先搜索在拓扑图中搜索所有路径
 * @return 从起点到终点的所有路径集合
 *
 * 算法流程：
 * 1. 使用DFS搜索所有从起点到终点的路径
 * 2. 按路径节点数排序（节点少的路径优先）
 * 3. 优先选择节点数少的路径，直到达到数量限制
 */
vector<vector<Eigen::Vector3d>> TopologyPRM::searchPaths() {
  raw_paths_.clear();

  vector<GraphNode::Ptr> visited;
  visited.push_back(graph_.front());  // 从起点开始

  depthFirstSearch(visited);  // DFS搜索所有路径

  // 按节点数对路径排序
  int min_node_num = 100000, max_node_num = 1;
  vector<vector<int>> path_list(100);  // path_list[节点数] = 该节点数的路径索引列表
  for (int i = 0; i < raw_paths_.size(); ++i) {
    if (int(raw_paths_[i].size()) > max_node_num) max_node_num = raw_paths_[i].size();
    if (int(raw_paths_[i].size()) < min_node_num) min_node_num = raw_paths_[i].size();
    path_list[int(raw_paths_[i].size())].push_back(i);
  }

  // 优先选择节点数少的路径
  vector<vector<Eigen::Vector3d>> filter_raw_paths;
  for (int i = min_node_num; i <= max_node_num; ++i) {
    bool reach_max = false;
    for (int j = 0; j < path_list[i].size(); ++j) {
      filter_raw_paths.push_back(raw_paths_[path_list[i][j]]);
      if (filter_raw_paths.size() >= max_raw_path2_) {  // 达到数量限制
        reach_max = true;
        break;
      }
    }
    if (reach_max) break;
  }
  std::cout << ", raw path num: " << raw_paths_.size() << ", " << filter_raw_paths.size();

  raw_paths_ = filter_raw_paths;

  return raw_paths_;
}

/**
 * @brief 深度优先搜索递归函数
 * @param vis 已访问节点序列（当前路径）
 *
 * DFS算法流程：
 * 1. 检查当前节点的邻居中是否有终点，若有则记录完整路径
 * 2. 遍历所有未访问的邻居节点
 * 3. 对每个邻居递归调用DFS
 * 4. 回溯：移除当前节点，尝试其他分支
 * 5. 当找到足够数量的路径时提前终止
 */
void TopologyPRM::depthFirstSearch(vector<GraphNode::Ptr>& vis) {
  GraphNode::Ptr cur = vis.back();  // 当前访问的节点

  // 第一步：检查是否可以到达终点
  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    // 检查是否到达目标节点（ID=1为终点）
    if (cur->neighbors_[i]->id_ == 1) {
      // 将当前路径添加到路径集合
      vector<Eigen::Vector3d> path;
      for (int j = 0; j < vis.size(); ++j) {
        path.push_back(vis[j]->pos_);
      }
      path.push_back(cur->neighbors_[i]->pos_);  // 添加终点

      raw_paths_.push_back(path);
      if (raw_paths_.size() >= max_raw_path_) return;  // 达到路径数量上限

      break;  // 找到终点后不再继续
    }
  }

  // 第二步：递归搜索其他邻居节点
  for (int i = 0; i < cur->neighbors_.size(); ++i) {
    // 跳过终点（终点在上面已处理）
    if (cur->neighbors_[i]->id_ == 1) continue;

    // 检查是否已访问过该节点（避免环路）
    bool revisit = false;
    for (int j = 0; j < vis.size(); ++j) {
      if (cur->neighbors_[i]->id_ == vis[j]->id_) {
        revisit = true;
        break;
      }
    }
    if (revisit) continue;

    // 递归搜索未访问的邻居
    vis.push_back(cur->neighbors_[i]);
    depthFirstSearch(vis);
    if (raw_paths_.size() >= max_raw_path_) return;  // 达到路径数量上限

    vis.pop_back();  // 回溯：移除当前节点，尝试其他分支
  }
}

/**
 * @brief 设置EDT环境指针
 * @param env EDT环境指针
 */
void TopologyPRM::setEnvironment(const EDTEnvironment::Ptr& env) { this->edt_environment_ = env; }

/**
 * @brief 检测三角形可见性（函数未完整实现）
 * @param pt 顶点
 * @param p1 底边起点
 * @param p2 底边终点
 * @return 可见性结果
 *
 * 原理：检测从pt到p1-p2线段上所有点的可见性
 * 注意：该函数实现不完整，缺少实际的可见性判断逻辑
 */
bool TopologyPRM::triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2) {
  // 获取沿p1-p2线段的遍历点
  vector<Eigen::Vector3d> pts;

  Eigen::Vector3d dir = p2 - p1;
  double length = dir.norm();
  int seg_num = ceil(length / resolution_);

  Eigen::Vector3d pt1;
  for (int i = 1; i < seg_num; ++i) {
    pt1 = p1 + dir * double(i) / double(seg_num);
    pts.push_back(pt1);
  }

  // 测试可见性（实现不完整）
  for (int i = 0; i < pts.size(); ++i) {
    {
      return false;  // 此处缺少实际判断逻辑
    }
  }

  return true;
}

}  // namespace fast_planner