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

#include <fast_path_searching/kinodynamic_astar.h>
#include <sstream>
#include <fast_plan_env/sdf_map.h>

using namespace std;
using namespace Eigen;

namespace fast_planner
{
/**
 * @brief 析构函数 - 释放预分配的路径节点内存池
 *
 * 遍历所有已分配的路径节点并释放内存,防止内存泄漏
 */
KinodynamicAstar::~KinodynamicAstar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];  // 释放每个预分配的节点
  }
}

/**
 * @brief Kinodynamic A*搜索算法主函数 - 考虑动力学约束的路径搜索
 *
 * @param start_pt 起始位置
 * @param start_v 起始速度
 * @param start_a 起始加速度
 * @param end_pt 目标位置
 * @param end_v 目标速度
 * @param init 是否为初始搜索(影响扩展策略)
 * @param dynamic 是否考虑时间维度(动态环境)
 * @param time_start 搜索起始时间
 * @return 搜索状态: REACH_END(到达终点), REACH_HORIZON(到达搜索边界), NEAR_END(接近终点), NO_PATH(无路径)
 */
int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
{
  start_vel_ = start_v;  // 保存起始速度用于后续轨迹生成
  start_acc_ = start_a;  // 保存起始加速度用于后续轨迹生成

  // 初始化起始节点
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pt;  // 状态的前3维为位置
  cur_node->state.tail(3) = start_v;   // 状态的后3维为速度
  cur_node->index = posToIndex(start_pt);  // 将连续位置转换为离散网格索引
  cur_node->g_score = 0.0;  // 起始节点的实际代价为0

  // 设置目标状态
  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;

  end_state.head(3) = end_pt;
  end_state.tail(3) = end_v;
  end_index = posToIndex(end_pt);
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);  // 计算启发式代价
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);  // 将起始节点加入开放列表(优先队列)
  use_node_num_ += 1;

  // 根据是否考虑动态环境来设置节点
  if (dynamic)
  {
    time_origin_ = time_start;  // 记录时间原点
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);  // 将时间离散化
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);  // 在时空哈希表中插入节点
    // cout << "time start: " << time_start << endl;
  }
  else
    expanded_nodes_.insert(cur_node->index, cur_node);  // 静态环境只需要空间索引

  PathNodePtr neighbor = NULL;
  PathNodePtr terminate_node = NULL;
  bool init_search = init;
  const int tolerance = ceil(1 / resolution_);  // 目标容差,单位为网格数

  // A*主循环 - 持续扩展节点直到开放列表为空
  while (!open_set_.empty())
  {
    cur_node = open_set_.top();  // 取出f_score最小的节点

    // 检查终止条件
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;  // 是否到达搜索边界
    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                    abs(cur_node->index(2) - end_index(2)) <= tolerance;  // 是否接近目标点

    if (reach_horizon || near_end)
    {
      terminate_node = cur_node;
      retrievePath(terminate_node);  // 回溯路径
      if (near_end)
      {
        // 检查是否可以直接连接到目标点(one-shot trajectory)
        estimateHeuristic(cur_node->state, end_state, time_to_goal);
        computeShotTraj(cur_node->state, end_state, time_to_goal);  // 计算直达轨迹
        if (init_search)
          ROS_ERROR("Shot in first search loop!");  // 初次搜索就能直达说明规划过于乐观
      }
    }

    // 处理到达搜索边界的情况
    if (reach_horizon)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;  // 到达边界且可以直达终点
      }
      else
      {
        std::cout << "reach horizon" << std::endl;
        return REACH_HORIZON;  // 仅到达边界,需要重新规划
      }
    }

    // 处理接近终点的情况
    if (near_end)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;  // 接近终点且可以直达
      }
      else if (cur_node->parent != NULL)
      {
        std::cout << "near end" << std::endl;
        return NEAR_END;  // 接近终点但无法直达,返回近似路径
      }
      else
      {
        std::cout << "no path" << std::endl;
        return NO_PATH;  // 起点就在终点附近但无法到达
      }
    }

    open_set_.pop();  // 从开放列表中移除当前节点
    cur_node->node_state = IN_CLOSE_SET;  // 标记为已扩展
    iter_num_ += 1;  // 迭代计数器

    // 设置采样分辨率
    double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;  // 传播后的状态
    vector<PathNodePtr> tmp_expand_nodes;  // 临时存储从当前节点扩展出的节点
    Eigen::Vector3d um;  // 控制输入(加速度)
    double pro_t;  // 传播后的时间
    vector<Eigen::Vector3d> inputs;  // 待尝试的控制输入集合
    vector<double> durations;  // 待尝试的持续时间集合

    // 根据是否为初始搜索采用不同的扩展策略
    if (init_search)
    {
      // 初始搜索: 仅使用起始加速度,采用更密集的时间采样
      inputs.push_back(start_acc_);
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
           tau += time_res_init * init_max_tau_)
        durations.push_back(tau);
      init_search = false;
    }
    else
    {
      // 正常搜索: 在加速度空间进行离散采样 (3x3x3=27个方向)
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
          for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
          {
            um << ax, ay, az;
            inputs.push_back(um);
          }
      // 时间采样
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        durations.push_back(tau);
    }

    // cout << "cur state:" << cur_state.head(3).transpose() << endl;
    // 遍历所有输入-时间组合,生成候选节点
    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j)
      {
        um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);  // 状态转移: 根据当前状态、控制输入和持续时间计算新状态
        pro_t = cur_node->time + tau;

        Eigen::Vector3d pro_pos = pro_state.head(3);

        // 检查1: 是否已在关闭列表中(已扩展过)
        Eigen::Vector3i pro_id = posToIndex(pro_pos);
        int pro_t_id = timeToIndex(pro_t);
        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          if (init_search)
            std::cout << "close" << std::endl;
          continue;  // 跳过已扩展的节点
        }

        // 检查2: 速度是否超过约束
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_)
        {
          if (init_search)
            std::cout << "vel" << std::endl;
          continue;  // 跳过速度超限的节点
        }

        // 检查3: 是否停留在同一体素内(无效扩展)
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          if (init_search)
            std::cout << "same" << std::endl;
          continue;  // 跳过未移动到新体素的节点
        }

        // 检查4: 碰撞检测 - 沿轨迹检查多个中间点
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;
        for (int k = 1; k <= check_num_; ++k)
        {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);  // 计算中间时刻的状态
          pos = xt.head(3);
          if (edt_environment_->sdf_map_->getInflateOccupancy(pos) == 1 )  // 检查是否在膨胀障碍物内
          {
            is_occ = true;
            break;
          }
        }
        if (is_occ)
        {
          if (init_search)
            std::cout << "safe" << std::endl;
          continue;  // 跳过碰撞的节点
        }

        // 计算代价函数
        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;  // g值: 控制能量+时间惩罚
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);  // f=g+h

        // 剪枝: 比较从同一父节点扩展到相同体素的多个节点,只保留代价最小的
        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;  // 找到到达相同体素的节点
            if (tmp_f_score < expand_node->f_score)
            {
              // 如果新节点代价更小,则更新该节点
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
              if (dynamic)
                expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }

        // 该节点到达的体素与其他节点不同,需要进一步处理
        if (!prune)
        {
          if (pro_node == NULL)
          {
            // 节点从未被访问过,创建新节点
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic)
            {
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);  // 加入开放列表

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            tmp_expand_nodes.push_back(pro_node);  // 记录为从当前节点扩展出的节点

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_)
            {
              cout << "run out of memory." << endl;
              return NO_PATH;  // 节点池耗尽
            }
          }
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            // 节点已在开放列表中,如果找到更优路径则更新
            if (tmp_g_score < pro_node->g_score)
            {
              // pro_node->index = pro_id;
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              if (dynamic)
                pro_node->time = cur_node->time + tau;
            }
          }
          else
          {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }
        }
      }
    // init_search = false;
  }

  // 开放列表为空,未找到路径
  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

/**
 * @brief 从ROS参数服务器加载搜索参数
 *
 * @param nh ROS节点句柄
 */
void KinodynamicAstar::setParam(ros::NodeHandle& nh)
{
  nh.param("search/max_tau", max_tau_, -1.0);  // 最大时间步长
  nh.param("search/init_max_tau", init_max_tau_, -1.0);  // 初始搜索的最大时间步长
  nh.param("search/max_vel", max_vel_, -1.0);  // 最大速度约束
  nh.param("search/max_acc", max_acc_, -1.0);  // 最大加速度约束
  nh.param("search/w_time", w_time_, -1.0);  // 时间权重
  nh.param("search/horizon", horizon_, -1.0);  // 搜索视野范围
  nh.param("search/resolution_astar", resolution_, -1.0);  // 空间分辨率
  nh.param("search/time_resolution", time_resolution_, -1.0);  // 时间分辨率
  nh.param("search/lambda_heu", lambda_heu_, -1.0);  // 启发式函数权重
  nh.param("search/allocate_num", allocate_num_, -1);  // 预分配的节点数量
  nh.param("search/check_num", check_num_, -1);  // 碰撞检测的中间点数量
  nh.param("search/optimistic", optimistic_, true);  // 是否使用乐观估计
  tie_breaker_ = 1.0 + 1.0 / 10000;  // 打破平局的小扰动

  double vel_margin;
  nh.param("search/vel_margin", vel_margin, 0.0);  // 速度余量
  max_vel_ += vel_margin;  // 在最大速度上增加余量
}

/**
 * @brief 从终点节点回溯路径到起点
 *
 * @param end_node 终点节点指针
 *
 * 通过父节点指针反向遍历,构建从起点到终点的路径
 */
void KinodynamicAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  // 沿父节点指针回溯到起点
  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());  // 反转为从起点到终点的顺序
}
/**
 * @brief 估计从状态x1到x2的启发式代价 - 基于最优时间的解析解
 *
 * @param x1 起始状态(位置+速度)
 * @param x2 目标状态(位置+速度)
 * @param optimal_time 输出参数,最优时间
 * @return 启发式代价值
 *
 * 通过求解四次方程找到最优时间,计算考虑加速度和时间权重的最小代价
 * 基于双端固定的边界值问题(BVP)的解析解
 */
double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{
  const Vector3d dp = x2.head(3) - x1.head(3);  // 位置差
  const Vector3d v0 = x1.segment(3, 3);  // 起始速度
  const Vector3d v1 = x2.segment(3, 3);  // 目标速度

  // 构建四次多项式的系数: c5*t^4 + c4*t^3 + c3*t^2 + c2*t + c1 = 0
  // 该多项式来源于对时间最优控制问题的一阶必要条件
  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);  // 求解四次方程得到候选时间

  // 计算时间下界: 基于最大速度的简单估计
  double v_max = max_vel_ * 0.5;
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;  // 无穷范数距离除以最大速度
  ts.push_back(t_bar);  // 将下界加入候选时间集合

  double cost = 100000000;
  double t_d = t_bar;

  // 遍历所有候选时间,找到代价最小的
  for (auto t : ts)
  {
    if (t < t_bar)
      continue;  // 跳过小于时间下界的解
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;  // 计算该时间下的代价
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;  // 添加打破平局的小扰动
}

/**
 * @brief 计算从state1到state2的直达轨迹(one-shot trajectory)
 *
 * @param state1 起始状态(位置+速度)
 * @param state2 目标状态(位置+速度)
 * @param time_to_goal 预计到达时间
 * @return 是否成功生成可行的直达轨迹
 *
 * 使用三次多项式插值生成满足边界条件的轨迹,并检查其是否满足动力学约束和避障要求
 */
bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  /* ---------- 计算多项式系数 ---------- */
  const Vector3d p0 = state1.head(3);  // 起始位置
  const Vector3d dp = state2.head(3) - p0;  // 位置差
  const Vector3d v0 = state1.segment(3, 3);  // 起始速度
  const Vector3d v1 = state2.segment(3, 3);  // 目标速度
  const Vector3d dv = v1 - v0;  // 速度差
  double t_d = time_to_goal;
  MatrixXd coef(3, 4);  // 3个维度,每个维度4个系数(a,b,c,d)
  end_vel_ = v1;

  // 求解三次多项式系数: p(t) = a*t^3 + b*t^2 + c*t + d
  // 满足边界条件: p(0)=p0, p(t_d)=p1, p'(0)=v0, p'(t_d)=v1
  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector3d c = v0;
  Vector3d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Vector3d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;

  // 构造求导矩阵: 对多项式系数左乘该矩阵相当于求导
  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- 前向检查轨迹可行性 ---------- */
  double t_delta = t_d / 10;  // 将轨迹分为10段进行检查
  for (double time = t_delta; time <= t_d; time += t_delta)
  {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t(j) = pow(time, j);  // [1, t, t^2, t^3]

    for (int dim = 0; dim < 3; dim++)
    {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);  // 计算位置
      vel(dim) = (Tm * poly1d).dot(t);  // 计算速度(一阶导数)
      acc(dim) = (Tm * Tm * poly1d).dot(t);  // 计算加速度(二阶导数)

      // 检查速度和加速度约束(注释掉了,仅做检查不返回)
      if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
      {
        // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
        // return false;
      }
    }

    // 检查是否超出地图边界
    if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
        coord(2) < origin_(2) || coord(2) >= map_size_3d_(2))
    {
      return false;
    }

    // if (edt_environment_->evaluateCoarseEDT(coord, -1.0) <= margin_) {
    //   return false;
    // }
    // 检查碰撞
    if (edt_environment_->sdf_map_->getInflateOccupancy(coord) == 1)
    {
      return false;  // 轨迹与障碍物碰撞
    }
  }

  // 所有检查通过,保存轨迹参数
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}

/**
 * @brief 求解三次方程 a*x^3 + b*x^2 + c*x + d = 0
 *
 * @param a 三次项系数
 * @param b 二次项系数
 * @param c 一次项系数
 * @param d 常数项
 * @return 所有实数解的向量
 *
 * 使用Cardano公式求解,根据判别式D的值分三种情况处理
 */
vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  // 归一化: 将方程转换为 x^3 + a2*x^2 + a1*x + a0 = 0
  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  // 计算Cardano公式中的中间变量
  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;  // 判别式

  if (D > 0)
  {
    // 判别式大于0: 一个实根,两个共轭复根
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    // 判别式等于0: 至少两个相等的实根
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);  // 单根
    dts.push_back(-a2 / 3 - S);  // 重根
    return dts;
  }
  else
  {
    // 判别式小于0: 三个不同的实根
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

/**
 * @brief 求解四次方程 a*x^4 + b*x^3 + c*x^2 + d*x + e = 0
 *
 * @param a 四次项系数
 * @param b 三次项系数
 * @param c 二次项系数
 * @param d 一次项系数
 * @param e 常数项
 * @return 所有实数解的向量
 *
 * 使用Ferrari方法求解: 将四次方程转化为三次预解方程,再分解为两个二次方程求解
 */
vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  // 归一化: 将方程转换为 x^4 + a3*x^3 + a2*x^2 + a1*x + a0 = 0
  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  // 求解三次预解方程(resolvent cubic)
  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;  // 无实数解

  double R = sqrt(r);
  double D, E;
  // 将四次方程分解为两个二次方程
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  // 收集所有实数根
  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

/**
 * @brief 初始化Kinodynamic A*搜索器
 *
 * 设置地图参数、预分配节点池、初始化状态转移矩阵
 */
void KinodynamicAstar::init()
{
  /* ---------- 地图参数 ---------- */
  this->inv_resolution_ = 1.0 / resolution_;  // 计算分辨率倒数用于快速索引转换
  inv_time_resolution_ = 1.0 / time_resolution_;
  edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);  // 获取地图的原点和尺寸

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  /* ---------- 预分配节点池 ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;  // 预先分配所有节点以避免运行时频繁内存分配
  }

  phi_ = Eigen::MatrixXd::Identity(6, 6);  // 状态转移矩阵(初始化为单位阵,后续会修改对角块)
  use_node_num_ = 0;  // 已使用节点数量
  iter_num_ = 0;  // 迭代次数
}

/**
 * @brief 设置环境指针
 *
 * @param env EDT(Euclidean Distance Transform)环境指针,用于碰撞检测
 */
void KinodynamicAstar::setEnvironment(const EDTEnvironment::Ptr& env)
{
  this->edt_environment_ = env;
}

/**
 * @brief 重置搜索器状态,为新的搜索做准备
 *
 * 清空所有搜索相关的数据结构,重置节点状态
 */
void KinodynamicAstar::reset()
{
  expanded_nodes_.clear();  // 清空已扩展节点哈希表
  path_nodes_.clear();  // 清空路径节点列表

  // 清空开放列表(优先队列)
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  // 重置所有已使用的节点
  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;  // 重置已使用节点计数
  iter_num_ = 0;  // 重置迭代计数
  is_shot_succ_ = false;  // 重置直达轨迹成功标志
  has_path_ = false;  // 重置路径存在标志
}

/**
 * @brief 获取完整的kinodynamic轨迹点序列
 *
 * @param delta_t 采样时间间隔
 * @return 位置点的向量
 *
 * 将搜索得到的路径段和可能的直达轨迹段组合,按时间间隔采样得到离散轨迹点
 */
std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t)
{
  vector<Vector3d> state_list;

  /* ---------- 获取搜索轨迹 ---------- */
  PathNodePtr node = path_nodes_.back();
  Matrix<double, 6, 1> x0, xt;

  // 从终点向起点反向采样
  while (node->parent != NULL)
  {
    Vector3d ut = node->input;  // 该段的控制输入
    double duration = node->duration;  // 该段的持续时间
    x0 = node->parent->state;  // 该段的起始状态

    // 在该段上反向采样
    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);  // 计算t时刻的状态
      state_list.push_back(xt.head(3));  // 保存位置
    }
    node = node->parent;
  }
  reverse(state_list.begin(), state_list.end());  // 反转为从起点到终点的顺序

  /* ---------- 获取直达轨迹(如果存在) ---------- */
  if (is_shot_succ_)
  {
    Vector3d coord;
    VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);  // [1, t, t^2, t^3]

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);  // 该维度的多项式系数
        coord(dim) = poly1d.dot(time);  // 计算该维度的位置
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}

/**
 * @brief 获取用于轨迹优化的采样点和边界导数
 *
 * @param ts 输入输出参数,采样时间间隔(会被自动调整)
 * @param point_set 输出参数,采样点集合
 * @param start_end_derivatives 输出参数,起点和终点的速度和加速度[v0, v1, a0, a1]
 *
 * 根据总路径时间自动调整采样点数量,确保至少有8个采样点
 */
void KinodynamicAstar::getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                                  vector<Eigen::Vector3d>& start_end_derivatives)
{
  /* ---------- 计算路径总时长 ---------- */
  double T_sum = 0.0;
  if (is_shot_succ_)
    T_sum += t_shot_;  // 直达轨迹的时间
  PathNodePtr node = path_nodes_.back();
  while (node->parent != NULL)
  {
    T_sum += node->duration;  // 累加搜索路径各段的时间
    node = node->parent;
  }
  // cout << "duration:" << T_sum << endl;

  // 计算终点速度和加速度
  Eigen::Vector3d end_vel, end_acc;
  double t;
  if (is_shot_succ_)
  {
    // 从直达轨迹计算终点状态
    t = t_shot_;
    end_vel = end_vel_;
    for (int dim = 0; dim < 3; ++dim)
    {
      Vector4d coe = coef_shot_.row(dim);
      end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;  // 二阶导数
    }
  }
  else
  {
    // 从搜索路径计算终点状态
    t = path_nodes_.back()->duration;
    end_vel = node->state.tail(3);
    end_acc = path_nodes_.back()->input;
  }

  // 获取采样点
  int seg_num = floor(T_sum / ts);
  seg_num = max(8, seg_num);  // 至少8个段
  ts = T_sum / double(seg_num);  // 重新计算时间间隔
  bool sample_shot_traj = is_shot_succ_;
  node = path_nodes_.back();

  // 从终点向起点反向采样
  for (double ti = T_sum; ti > -1e-5; ti -= ts)
  {
    if (sample_shot_traj)
    {
      // 在直达轨迹上采样
      Vector3d coord;
      Vector4d poly1d, time;

      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }

      point_set.push_back(coord);
      t -= ts;

      /* 该段结束 */
      if (t < -1e-5)
      {
        sample_shot_traj = false;
        if (node->parent != NULL)
          t += node->duration;  // 切换到搜索轨迹的最后一段
      }
    }
    else
    {
      // 在搜索轨迹上采样
      Eigen::Matrix<double, 6, 1> x0 = node->parent->state;
      Eigen::Matrix<double, 6, 1> xt;
      Vector3d ut = node->input;

      stateTransit(x0, xt, ut, t);  // 计算当前时刻的状态

      point_set.push_back(xt.head(3));
      t -= ts;

      // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
      if (t < -1e-5 && node->parent->parent != NULL)
      {
        node = node->parent;
        t += node->duration;  // 切换到前一段
      }
    }
  }
  reverse(point_set.begin(), point_set.end());  // 反转为从起点到终点的顺序

  // 计算起点加速度
  Eigen::Vector3d start_acc;
  if (path_nodes_.back()->parent == NULL)
  {
    // 没有搜索轨迹,仅有直达轨迹,从多项式计算
    start_acc = 2 * coef_shot_.col(2);
  }
  else
  {
    // 使用搜索轨迹第一段的输入
    start_acc = node->input;
  }

  // 保存边界导数: [起始速度, 终点速度, 起始加速度, 终点加速度]
  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(end_acc);
}

/**
 * @brief 获取所有已访问(扩展)的节点
 *
 * @return 已访问节点的向量
 *
 * 用于可视化搜索过程
 */
std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
{
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

/**
 * @brief 将连续位置转换为离散网格索引
 *
 * @param pt 连续的3D位置
 * @return 3D网格索引
 */
Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
{
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

  // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
  // origin_(1)) * inv_resolution_),
  //     floor((pt(2) - origin_(2)) * inv_resolution_);

  return idx;
}

/**
 * @brief 将连续时间转换为离散时间索引
 *
 * @param time 连续时间
 * @return 时间索引
 */
int KinodynamicAstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

/**
 * @brief 状态转移函数 - 根据当前状态、控制输入和时间计算新状态
 *
 * @param state0 初始状态[位置; 速度]
 * @param state1 输出参数,转移后的状态
 * @param um 控制输入(加速度)
 * @param tau 持续时间
 *
 * 使用恒定加速度模型: p1 = p0 + v0*tau + 0.5*a*tau^2, v1 = v0 + a*tau
 */
void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                                    Eigen::Vector3d um, double tau)
{
  for (int i = 0; i < 3; ++i)
    phi_(i, i + 3) = tau;  // 设置状态转移矩阵的非对角块

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;  // 位置的积分项: 0.5*a*tau^2
  integral.tail(3) = tau * um;  // 速度的积分项: a*tau

  state1 = phi_ * state0 + integral;  // 线性状态转移
}

}  // namespace fast_planner
