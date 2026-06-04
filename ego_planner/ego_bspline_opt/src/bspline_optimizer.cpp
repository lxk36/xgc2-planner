// ego_planner的B样条轨迹优化器实现文件
// 该文件实现了基于梯度的B样条轨迹优化算法，主要用于多旋翼无人机的轨迹规划
#include "ego_bspline_opt/bspline_optimizer.h"
#include "ego_bspline_opt/gradient_descent_optimizer.h"
// using namespace std;

namespace ego_planner
{

  /**
   * @brief 从ROS参数服务器加载优化器参数
   * @param nh ROS节点句柄
   *
   * 主要参数包括：
   * - lambda1_: 平滑性代价权重
   * - lambda2_: 碰撞代价权重
   * - lambda3_: 可行性代价权重（速度/加速度约束）
   * - lambda4_: 拟合度代价权重
   * - dist0_: 安全距离
   * - max_vel_: 最大速度
   * - max_acc_: 最大加速度
   * - order_: B样条阶数（默认为3，即三次B样条）
   */
  void BsplineOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/lambda_smooth", lambda1_, -1.0);
    nh.param("optimization/lambda_collision", lambda2_, -1.0);
    nh.param("optimization/lambda_feasibility", lambda3_, -1.0);
    nh.param("optimization/lambda_fitness", lambda4_, -1.0);

    nh.param("optimization/dist0", dist0_, -1.0);
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);

    nh.param("optimization/order", order_, 3);
  }

  /**
   * @brief 设置环境地图
   * @param env 栅格地图智能指针，用于碰撞检测和距离查询
   */
  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &env)
  {
    this->grid_map_ = env;
  }

  /**
   * @brief 设置B样条的控制点
   * @param points 控制点矩阵，每列代表一个3D控制点
   */
  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  /**
   * @brief 设置B样条的时间间隔
   * @param ts B样条的节点时间间隔
   */
  void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

  /**
   * @brief 初始化控制点并根据障碍物分段轨迹
   * @param init_points 初始控制点矩阵（会被修改）
   * @param flag_first_init 是否为首次初始化
   * @return 每个障碍物段对应的A*路径集合
   *
   * 此函数与check_collision_and_rebound()非常相似，
   * 分开编写是因为该版本自2020年3月以来一直稳定运行。
   * 未来可能会合并这两个函数。
   *
   * 主要功能：
   * 1. 根据障碍物将初始轨迹分段
   * 2. 为每个障碍物段调用A*搜索绕行路径
   * 3. 为每个控制点分配基准点和推力方向
   */
  std::vector<std::vector<Eigen::Vector3d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
  {

    if (flag_first_init)
    {
      cps_.clearance = dist0_;       // 设置安全间隙
      cps_.resize(init_points.cols()); // 重置控制点结构大小
      cps_.points = init_points;     // 复制初始控制点
    }

    /*** 根据障碍物对初始轨迹进行分段 ***/
    constexpr int ENOUGH_INTERVAL = 2; // 足够的间隔，用于判断占用状态变化是否稳定
    // 计算步长：基于地图分辨率和轨迹点间距
    double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 2;
    int in_id, out_id;  // 障碍物段的进入和退出ID
    vector<std::pair<int, int>> segment_ids; // 存储所有障碍物段的ID对
    int same_occ_state_times = ENOUGH_INTERVAL + 1; // 相同占用状态的持续次数
    bool occ, last_occ = false; // 当前和上一次的占用状态
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = (int)init_points.cols() - order_ - ((int)init_points.cols() - 2 * order_) / 3; // 只检查前2/3的点
    // 遍历控制点，检测障碍物段
    for (int i = order_; i <= i_end; ++i)
    {
      // 在相邻控制点之间进行线性插值检测
      for (double a = 1.0; a >= 0.0; a -= step_size)
      {
        // 检查插值点是否在膨胀后的障碍物中
        occ = grid_map_->getInflateOccupancy(a * init_points.col(i - 1) + (1 - a) * init_points.col(i));
        // cout << setprecision(5);
        // cout << (a * init_points.col(i-1) + (1-a) * init_points.col(i)).transpose() << " occ1=" << occ << endl;

        // 从自由空间进入障碍物
        if (occ && !last_occ)
        {
          // 确保状态变化是稳定的，或者是第一个检测点
          if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
          {
            in_id = i - 1;  // 记录进入障碍物前的控制点ID
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // 提前终止结束标志
        }
        // 从障碍物退出到自由空间
        else if (!occ && last_occ)
        {
          out_id = i;  // 记录退出障碍物后的控制点ID
          flag_got_end_maybe = true;  // 可能找到了出口
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;  // 保持相同状态的次数
        }

        // 确认找到了稳定的障碍物出口
        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;  // 更新上一次的占用状态

        // 成功识别了一个完整的障碍物段
        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }

    /*** A*搜索：为每个障碍物段寻找绕行路径 ***/
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      //cout << "in=" << in.transpose() << " out=" << out.transpose() << endl;
      // 获取障碍物段的进入点和退出点
      Eigen::Vector3d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
      // 调用A*算法搜索从in到out的无碰撞路径，分辨率为0.1
      if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
      {
        a_star_pathes.push_back(a_star_->getPath());  // 保存找到的路径
      }
      else
      {
        ROS_ERROR("a star error, force return!");
        return a_star_pathes;  // A*搜索失败，强制返回
      }
    }

    /*** 计算每个障碍物段的边界范围 ***/
    // 边界用于确定每个段可以调整的控制点范围
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // 第一个障碍物段
      {
        id_low_bound = order_;  // 下界从B样条阶数开始
        if (segment_ids.size() > 1)
        {
          // 上界取当前段出口和下一段入口的中点（向下取整）
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2);
        }
        else
        {
          id_up_bound = init_points.cols() - order_ - 1;  // 只有一个段时，上界到倒数第order个点
        }
      }
      else if (i == segment_ids.size() - 1) // 最后一个障碍物段，此时i != 0
      {
        // 下界取上一段出口和当前段入口的中点（向上取整）
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2);
        id_up_bound = init_points.cols() - order_ - 1;
      }
      else  // 中间的障碍物段
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // 向上取整
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // 向下取整
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    // cout << "+++++++++" << endl;
    // for ( int j=0; j<bounds.size(); ++j )
    // {
    //   cout << bounds[j].first << "  " << bounds[j].second << endl;
    // }

    /*** 调整段长度，确保每段有足够的控制点 ***/
    vector<std::pair<int, int>> final_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // 每段保证有足够的点以生成足够的推力（当前设为0，即不限制）
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** 调整段长度 ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;  // 计算当前段的点数
      //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
      if (num_points < minimum_points)  // 如果点数不足
      {
        // 计算需要在两端各添加多少点
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        // 在不超出边界的前提下扩展段的起始和结束索引
        final_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

        final_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
      }
      else  // 点数足够，直接使用原始段
      {
        final_segment_ids[i].first = segment_ids[i].first;
        final_segment_ids[i].second = segment_ids[i].second;
      }

      //cout << "final:" << "i = " << i << " first = " << final_segment_ids[i].first << " second = " << final_segment_ids[i].second << endl;
    }

    /*** 为每个段分配基准点和推力方向数据 ***/
    // 这是一个三步算法，用于为控制点确定避障的推力方向
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // 步骤1: 初始化临时标志，表示该点尚未找到交点
      for (int j = final_segment_ids[i].first; j <= final_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // 步骤2: 寻找A*路径与控制点法线的交点
      int got_intersection_id = -1;  // 记录找到交点的控制点索引
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        // ctrl_pts_law: 控制点的局部切线方向（相邻控制点连线）
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
        // 从A*路径中点开始搜索（使用中点而非最远点是为了减少计算量）
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id;
        // 计算A*路径点到控制点的向量在切线方向上的投影
        double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
        // 沿着A*路径搜索与控制点法线的交点
        while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
        {
          last_Astar_id = Astar_id;

          // 根据投影值的符号决定搜索方向
          if (val >= 0)
            --Astar_id;  // 向路径起点方向搜索
          else
            ++Astar_id;  // 向路径终点方向搜索

          val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

          // 检测符号变化，表示找到了交点（val和last_val异号）
          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // 不允许val = last_val = 0.0
          {
            // 通过线性插值计算精确的交点位置
            // 插值参数t由两点投影值的比例确定
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            //cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

            got_intersection_id = j;  // 记录找到交点的控制点索引
            break;
          }
        }

        // 如果找到了交点，计算基准点和推力方向
        if (got_intersection_id >= 0)
        {
          cps_.flag_temp[j] = true;  // 标记该点已找到交点
          double length = (intersection_point - cps_.points.col(j)).norm();  // 控制点到交点的距离
          if (length > 1e-5)  // 距离足够大才有意义
          {
            // 从交点向控制点方向回退，直到遇到障碍物边界
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              // 在控制点和交点之间进行线性插值
              occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

              // 找到障碍物边界或到达控制点
              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();  // 稍微退出障碍物
                // 设置基准点（障碍物边界点）
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                // 设置推力方向（从控制点指向交点）
                cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                break;
              }
            }
          }
        }
      }

      /* 特殊情况：段长度太短
       * 此时控制点可能在A*路径外侧，导致梯度方向相反
       * 因此需要特殊处理 */
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector3d ctrl_pts_law(cps_.points.col(segment_ids[i].second) - cps_.points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (cps_.points.col(segment_ids[i].second) + cps_.points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
        {
          last_Astar_id = Astar_id;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(cps_.points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //步骤3: 传播基准点和方向到段内其他控制点
      // 如果找到了交点，将其信息传播到同一段的其他控制点
      if (got_intersection_id >= 0)
      {
        // 向后传播（从交点到段末尾）
        for (int j = got_intersection_id + 1; j <= final_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])  // 如果该点还没有找到交点
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());  // 复制前一个点的基准点
            cps_.direction[j].push_back(cps_.direction[j - 1].back());    // 复制前一个点的方向
          }

        // 向前传播（从交点到段起始）
        for (int j = got_intersection_id - 1; j >= final_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])  // 如果该点还没有找到交点
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());  // 复制后一个点的基准点
            cps_.direction[j].push_back(cps_.direction[j + 1].back());    // 复制后一个点的方向
          }
      }
      else
      {
        // 没找到交点也没关系，可以忽略 ^_^
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    return a_star_pathes;
  }

  /**
   * @brief L-BFGS优化器的提前退出回调函数
   * @return 如果应该停止优化则返回非零值
   *
   * 当检测到错误或需要重新弹射（rebound）时提前终止优化
   */
  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // cout << "k=" << k << endl;
    // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /**
   * @brief Rebound阶段的代价函数（用于L-BFGS优化器）
   * @param func_data 优化器对象指针
   * @param x 当前优化变量（控制点坐标）
   * @param grad 梯度输出数组
   * @param n 变量维度
   * @return 总代价值
   */
  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRebound(x, grad, cost, n);  // 计算组合代价和梯度

    opt->iter_num_ += 1;  // 增加迭代计数
    return cost;
  }

  /**
   * @brief Refine阶段的代价函数（用于L-BFGS优化器）
   * @param func_data 优化器对象指针
   * @param x 当前优化变量（控制点坐标）
   * @param grad 梯度输出数组
   * @param n 变量维度
   * @return 总代价值
   */
  double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRefine(x, grad, cost, n);  // 计算组合代价和梯度

    opt->iter_num_ += 1;  // 增加迭代计数
    return cost;
  }

  /**
   * @brief 计算Rebound阶段的距离代价和梯度
   * @param q 控制点矩阵
   * @param cost 输出的代价值
   * @param gradient 输出的梯度矩阵
   * @param iter_num 当前迭代次数
   * @param smoothness_cost 平滑性代价（用于判断是否需要检查碰撞）
   *
   * 使用分段函数计算距离代价：
   * - 当距离误差 < 0（安全）：代价为0
   * - 当 0 <= 距离误差 < demarcation：三次函数（保证C2连续）
   * - 当距离误差 >= demarcation：二次函数（更强的惩罚）
   */
  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    double demarcation = cps_.clearance;  // 分界线，等于安全间隙
    // 二次函数段的系数，保证在分界点处与三次函数C2连续
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    force_stop_type_ = DONT_STOP;
    // 当迭代足够多且轨迹足够平滑时，检查碰撞并可能触发rebound
    if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1是实验值，表示轨迹足够平滑
    {
      check_collision_and_rebound();
    }

    /*** 计算距离代价和梯度 ***/
    for (auto i = order_; i < end_idx; ++i)
    {
      // 对每个控制点可能有多个障碍物约束
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        // 计算控制点到基准点在推力方向上的距离
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        double dist_err = cps_.clearance - dist;  // 距离误差（正值表示太近）
        Eigen::Vector3d dist_grad = cps_.direction[i][j];  // 梯度方向

        if (dist_err < 0)
        {
          /* 距离足够，无需惩罚 */
        }
        else if (dist_err < demarcation)
        {
          // 使用三次函数：cost = dist_err^3
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          // 使用二次函数：cost = a*dist_err^2 + b*dist_err + c
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
  }

  /**
   * @brief 计算拟合度代价和梯度
   * @param q 控制点矩阵
   * @param cost 输出的代价值
   * @param gradient 输出的梯度矩阵
   *
   * 拟合度代价衡量B样条轨迹与参考路径的偏离程度
   * 使用椭圆距离函数：f = |x·v|^2/a^2 + |x×v|^2/b^2
   * 其中：x是B样条点到参考点的向量，v是参考路径的切线方向
   * a^2=25允许沿路径方向较大偏差，b^2=1限制垂直方向偏差
   */
  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {

    cost = 0.0;

    int end_idx = q.cols() - order_;

    // 定义椭圆距离函数: f = |x·v|^2/a^2 + |x×v|^2/b^2
    double a2 = 25, b2 = 1;  // a^2=25允许沿路径更大偏差，b^2=1严格限制垂直偏差
    for (auto i = order_ - 1; i < end_idx + 1; ++i)
    {
      // 计算B样条上的点（使用均匀B样条的基函数系数1/6, 4/6, 1/6）
      Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
      // 参考路径的切线方向（归一化）
      Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

      double xdotv = x.dot(v);        // 沿路径方向的偏差
      Eigen::Vector3d xcrossv = x.cross(v);  // 垂直路径方向的偏差向量

      // 计算椭圆距离代价
      double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
      cost += f;

      // 计算梯度：df/dx = 2*xdotv/a^2 * v + 2/b^2 * [v]× * (x×v)
      // 其中 [v]× 是v的反对称矩阵
      Eigen::Matrix3d m;
      m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
      Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

      // 根据B样条基函数将梯度分配到相邻控制点
      gradient.col(i - 1) += df_dx / 6;
      gradient.col(i) += 4 * df_dx / 6;
      gradient.col(i + 1) += df_dx / 6;
    }
  }

  /**
   * @brief 计算平滑性代价和梯度
   * @param q 控制点矩阵
   * @param cost 输出的代价值
   * @param gradient 输出的梯度矩阵
   * @param falg_use_jerk 是否使用jerk（急动度）作为平滑性度量，否则使用加速度
   *
   * 平滑性代价最小化轨迹的高阶导数：
   * - jerk模式：最小化三阶导数的平方和（更平滑）
   * - acc模式：最小化二阶导数的平方和
   */
  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
  {

    cost = 0.0;

    if (falg_use_jerk)
    {
      Eigen::Vector3d jerk, temp_j;

      for (int i = 0; i < q.cols() - 3; i++)
      {
        /* 计算jerk（三阶差分近似三阶导数） */
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        cost += jerk.squaredNorm();  // 累加jerk的平方范数
        temp_j = 2.0 * jerk;
        /* jerk的梯度（对四个相关控制点的偏导数） */
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
      }
    }
    else
    {
      Eigen::Vector3d acc, temp_acc;

      for (int i = 0; i < q.cols() - 2; i++)
      {
        /* 计算加速度（二阶差分近似二阶导数） */
        acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
        cost += acc.squaredNorm();  // 累加加速度的平方范数
        temp_acc = 2.0 * acc;
        /* 加速度的梯度（对三个相关控制点的偏导数） */
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
      }
    }
  }

  /**
   * @brief 计算可行性代价和梯度（速度和加速度约束）
   * @param q 控制点矩阵
   * @param cost 输出的代价值
   * @param gradient 输出的梯度矩阵
   *
   * 可行性代价确保轨迹满足动力学约束：
   * - 速度约束：|v| <= max_vel_
   * - 加速度约束：|a| <= max_acc_
   *
   * 支持两种模式（通过宏SECOND_DERIVATIVE_CONTINOUS控制）：
   * 1. C2连续模式：使用三次和二次函数组合，保证二阶导数连续
   * 2. 简单模式：直接使用二次惩罚函数
   */
  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {

    //#define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS

    cost = 0.0;
    double demarcation = 1.0; // 分界值：1m/s 或 1m/s/s
    // 右侧（正向超速）的二次函数系数
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
    // 左侧（负向超速）的二次函数系数
    double al = ar, bl = -br, cl = cr;

    /* 缩写变量 */
    double ts, ts_inv2, ts_inv3;
    ts = bspline_interval_;      // B样条时间间隔
    ts_inv2 = 1 / ts / ts;       // 1/ts^2，用于加速度计算
    ts_inv3 = 1 / ts / ts / ts;  // 1/ts^3，用于速度归一化

    /* 速度可行性约束 */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      // 通过一阶差分近似速度
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      // 对x, y, z三个方向分别约束
      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_ + demarcation)  // 严重超过最大速度
        {
          double diff = vi(j) - max_vel_;
          cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

          double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) > max_vel_)
        {
          double diff = vi(j) - max_vel_;
          cost += pow(diff, 3) * ts_inv3;
          ;

          double grad = 3 * diff * diff / ts * ts_inv3;
          ;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -(max_vel_ + demarcation))
        {
          double diff = vi(j) + max_vel_;
          cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

          double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -max_vel_)
        {
          double diff = vi(j) + max_vel_;
          cost += -pow(diff, 3) * ts_inv3;

          double grad = -3 * diff * diff / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_ + demarcation)
        {
          double diff = ai(j) - max_acc_;
          cost += ar * diff * diff + br * diff + cr;

          double grad = (2.0 * ar * diff + br) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) > max_acc_)
        {
          double diff = ai(j) - max_acc_;
          cost += pow(diff, 3);

          double grad = 3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -(max_acc_ + demarcation))
        {
          double diff = ai(j) + max_acc_;
          cost += al * diff * diff + bl * diff + cl;

          double grad = (2.0 * al * diff + bl) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -max_acc_)
        {
          double diff = ai(j) + max_acc_;
          cost += -pow(diff, 3);

          double grad = -3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

#else
    // 简单模式：直接使用二次惩罚函数（当前使用的模式）

    cost = 0.0;
    /* 缩写变量 */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* 速度可行性约束 */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      // 通过一阶差分近似速度
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      //cout << "temp_v * vi=" ;
      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_)  // 正向超过最大速度
        {
          // cout << "fuck VEL" << endl;
          // cout << vi(j) << endl;
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // 乘以ts_inv2使速度和加速度代价量级相近

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)  // 负向超过最大速度
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* 在约束范围内，无惩罚 */
        }
      }
    }

    /* 加速度可行性约束 */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      // 通过二阶差分近似加速度
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      //cout << "temp_a * ai=" ;
      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_)  // 正向超过最大加速度
        {
          // cout << "fuck ACC" << endl;
          // cout << ai(j) << endl;
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)  // 负向超过最大加速度
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* 在约束范围内，无惩罚 */
        }
      }
      //cout << endl;
    }

#endif
  }

  /**
   * @brief 检查碰撞并触发弹射（rebound）机制
   * @return 如果检测到新的有效碰撞则返回true，否则返回false
   *
   * 该函数在优化过程中定期调用，用于：
   * 1. 检测当前轨迹是否与新障碍物碰撞
   * 2. 如果碰撞，重新运行A*搜索绕行路径
   * 3. 更新基准点和推力方向，准备下一轮优化
   */
  bool BsplineOptimizer::check_collision_and_rebound(void)
  {

    int end_idx = cps_.size - order_;

    /*** 根据障碍物检查并分段初始轨迹 ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;  // 是否发现新的有效碰撞
    int i_end = end_idx - (end_idx - order_) / 3;  // 只检查前2/3的轨迹
    for (int i = order_ - 1; i <= i_end; ++i)
    {

      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

      /*** 检查新碰撞是否有效 ***/
      // 如果控制点已经在已知障碍物的外侧（距离小于一个分辨率），则不算新碰撞
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          cout.precision(2);
          // 检查当前点是否在所有已知碰撞点的外侧
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution())
          {
            occ = false; // 不算真正的碰撞，仅为了便于理解
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          ROS_ERROR("ERROR! the drone is in obstacle. This should not happen.");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.size) // fail to get the obs free point
        {
          ROS_WARN("WARN! terminal point of the current trajectory is in obstacle, skip this planning.");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
        if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
        {
          a_star_pathes.push_back(a_star_->getPath());
        }
        else
        {
          ROS_ERROR("a star error");
          segment_ids.erase(segment_ids.begin() + i);
          i--;
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
          while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
          {
            last_Astar_id = Astar_id;

            if (val >= 0)
              --Astar_id;
            else
              ++Astar_id;

            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            // cout << val << endl;

            if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            cps_.flag_temp[j] = true;
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          ROS_WARN("Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  /**
   * @brief B样条轨迹优化的主入口函数（Rebound阶段）
   * @param optimal_points 输出的优化后控制点
   * @param ts B样条时间间隔
   * @return 优化是否成功
   *
   * Rebound优化阶段使用以下代价函数：
   * - 平滑性代价（lambda1）
   * - 距离代价（lambda2，避障）
   * - 可行性代价（lambda3，动力学约束）
   */
  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
  {
    setBsplineInterval(ts);

    bool flag_success = rebound_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  /**
   * @brief B样条轨迹优化的主入口函数（Refine阶段）
   * @param init_points 初始控制点
   * @param ts B样条时间间隔
   * @param optimal_points 输出的优化后控制点
   * @return 优化是否成功
   *
   * Refine优化阶段使用以下代价函数：
   * - 平滑性代价（lambda1）
   * - 拟合度代价（lambda4，跟踪参考路径）
   * - 可行性代价（lambda3，动力学约束）
   */
  bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
  {

    setControlPoints(init_points);
    setBsplineInterval(ts);

    bool flag_success = refine_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  /**
   * @brief Rebound优化的核心函数
   * @return 优化是否成功
   *
   * Rebound优化是一个迭代过程：
   * 1. 运行L-BFGS优化器优化控制点位置
   * 2. 检查优化后的轨迹是否有新的碰撞
   * 3. 如果有碰撞，增加碰撞权重并重新初始化（最多重启3次）
   * 4. 如果触发rebound（检测到新障碍物），更新推力方向并继续优化（最多20次）
   */
  bool BsplineOptimizer::rebound_optimize()
  {
    iter_num_ = 0;
    int start_id = order_;                      // 优化起始索引（跳过前order个固定点）
    int end_id = this->cps_.size - order_;      // 优化结束索引（跳过后order个固定点）
    variable_num_ = 3 * (end_id - start_id);    // 优化变量总数（3维坐标 × 可优化点数）
    double final_cost;

    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;    // 重启次数和弹射次数计数
    ;
    bool flag_force_return, flag_occ, success;
    new_lambda2_ = lambda2_;                    // 碰撞权重，可能会动态增加
    constexpr int MAX_RESART_NUMS_SET = 3;      // 最大重启次数
    do
    {
      /* ---------- 准备阶段 ---------- */
      min_cost_ = std::numeric_limits<double>::max();
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;

      // 将控制点数据复制到优化变量数组
      double q[variable_num_];
      memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

      // 配置L-BFGS优化器参数
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;            // 有限内存大小
      lbfgs_params.max_iterations = 200;     // 最大迭代次数
      lbfgs_params.g_epsilon = 0.01;         // 梯度收敛阈值

      /* ---------- 执行优化 ---------- */
      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;
      double total_time_ms = (t2 - t0).toSec() * 1000;

      /* ---------- 暂时成功，再次检查碰撞 ---------- */
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
        flag_force_return = false;

        // 构造B样条轨迹并沿轨迹检查碰撞
        UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
        double tm, tmp;
        traj.getTimeSpan(tm, tmp);
        // 步长设置为能通过每个网格的最大值
        double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution());
        for (double t = tm; t < tmp * 2 / 3; t += t_step) // 只检查前2/3的轨迹
        {
          flag_occ = grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t));
          if (flag_occ)
          {
            //cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

            if (t <= bspline_interval_) // 前3个控制点在障碍物中！
            {
              cout << cps_.points.col(1).transpose() << "\n"
                   << cps_.points.col(2).transpose() << "\n"
                   << cps_.points.col(3).transpose() << "\n"
                   << cps_.points.col(4).transpose() << endl;
              ROS_WARN("First 3 control points in obstacles! return false, t=%f", t);
              return false;  // 起始点在障碍物中，优化失败
            }

            break;
          }
        }

        if (!flag_occ)  // 没有碰撞，优化成功
        {
          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          success = true;
        }
        else // 有碰撞，需要重启优化
        {
          restart_nums++;
          initControlPoints(cps_.points, false);  // 重新初始化推力方向
          new_lambda2_ *= 2;  // 增加碰撞权重

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,keep optimizing\n\033[0m", iter_num_, time_ms);
        }
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)  // 被提前退出回调取消
      {
        flag_force_return = true;
        rebound_times++;
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
      }
      else  // 优化器出错
      {
        ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        // while (ros::ok());
      }

    } while ((flag_occ && restart_nums < MAX_RESART_NUMS_SET) ||  // 有碰撞且未超过重启次数
             (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));  // 触发rebound且未超过20次

    return success;
  }

  /**
   * @brief Refine优化的核心函数
   * @return 优化是否成功
   *
   * Refine优化用于精炼轨迹，使其更好地跟踪参考路径：
   * 1. 运行L-BFGS优化器，最小化拟合度、平滑性和可行性代价
   * 2. 检查优化后的轨迹是否安全
   * 3. 如果不安全，增加拟合度权重并重新优化（目前最多1次）
   */
  bool BsplineOptimizer::refine_optimize()
  {
    iter_num_ = 0;
    int start_id = order_;
    int end_id = this->cps_.points.cols() - order_;
    variable_num_ = 3 * (end_id - start_id);

    double q[variable_num_];
    double final_cost;

    // 将控制点数据复制到优化变量数组
    memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

    double origin_lambda4 = lambda4_;  // 保存原始拟合度权重
    bool flag_safe = true;             // 轨迹是否安全（无碰撞）
    int iter_count = 0;
    do
    {
      // 配置L-BFGS优化器参数
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.001;  // 更小的梯度阈值，要求更高精度

      // 执行优化
      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        //pass  // 优化成功或达到最大迭代次数
      }
      else
      {
        ROS_ERROR("Solver error in refining!, return = %d, %s", result, lbfgs::lbfgs_strerror(result));
      }

      // 检查优化后的轨迹是否安全
      UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
      double tm, tmp;
      traj.getTimeSpan(tm, tmp);
      // 步长定义为能通过每个网格的最大值
      double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution());
      for (double t = tm; t < tmp * 2 / 3; t += t_step)
      {
        if (grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t)))
        {
          // cout << "Refined traj hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

          Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
          for (size_t i = 0; i < ref_pts_.size(); i++)
          {
            ref_pts.row(i) = ref_pts_[i].transpose();
          }

          flag_safe = false;  // 发现碰撞
          break;
        }
      }

      // 如果不安全，增加拟合度权重（使轨迹更接近参考路径）
      if (!flag_safe)
        lambda4_ *= 2;

      iter_count++;
    } while (!flag_safe && iter_count <= 0);  // 目前最多重试1次（iter_count <= 0）

    lambda4_ = origin_lambda4;  // 恢复原始权重

    //cout << "iter_num_=" << iter_num_ << endl;

    return flag_safe;
  }

  /**
   * @brief 组合Rebound阶段的所有代价和梯度
   * @param x 优化变量（控制点坐标）
   * @param grad 输出的梯度
   * @param f_combine 输出的总代价
   * @param n 变量维度
   *
   * 组合三个代价项：
   * - 平滑性代价 × lambda1_
   * - 距离代价 × new_lambda2_（避障）
   * - 可行性代价 × lambda3_（动力学约束）
   */
  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {

    // 将优化变量复制回控制点矩阵（跳过前order个固定点）
    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- 计算代价和梯度 ---------- */
    double f_smoothness, f_distance, f_feasibility;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    // 加权组合所有代价项
    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility;
    //printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

    // 加权组合所有梯度
    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

  /**
   * @brief 组合Refine阶段的所有代价和梯度
   * @param x 优化变量（控制点坐标）
   * @param grad 输出的梯度
   * @param f_combine 输出的总代价
   * @param n 变量维度
   *
   * 组合三个代价项：
   * - 平滑性代价 × lambda1_
   * - 拟合度代价 × lambda4_（跟踪参考路径）
   * - 可行性代价 × lambda3_（动力学约束）
   */
  void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
  {

    // 将优化变量复制回控制点矩阵
    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- 计算代价和梯度 ---------- */
    double f_smoothness, f_fitness, f_feasibility;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    //time_satrt = ros::Time::now();

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcFitnessCost(cps_.points, f_fitness, g_fitness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    /* ---------- 转换为求解器格式 ---------- */
    // 加权组合所有代价项
    f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;
    // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);

    // 加权组合所有梯度
    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

} // namespace ego_planner