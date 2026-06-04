// 均匀B样条曲线实现文件
// 该文件实现了均匀B样条曲线的核心功能,包括曲线构造、求值、求导、可行性检查等
#include "ego_bspline_opt/uniform_bspline.h"
#include <ros/ros.h>

namespace ego_planner
{

  /**
   * @brief 均匀B样条构造函数
   * @param points 控制点矩阵,每列代表一个控制点
   * @param order B样条的阶数(次数+1),如3阶B样条是2次曲线
   * @param interval 节点向量的均匀间隔
   */
  UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order,
                                 const double &interval)
  {
    setUniformBspline(points, order, interval);
  }

  // 析构函数
  UniformBspline::~UniformBspline() {}

  /**
   * @brief 设置均匀B样条的参数
   * @param points 控制点矩阵(维度 × 控制点数量)
   * @param order B样条阶数
   * @param interval 节点间隔
   *
   * 该函数初始化B样条的所有必要参数,包括:
   * - 控制点
   * - 阶数和节点数
   * - 节点向量(knot vector)
   */
  void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order,
                                         const double &interval)
  {
    control_points_ = points;  // 存储控制点
    p_ = order;                // B样条阶数
    interval_ = interval;      // 节点间隔

    n_ = points.cols() - 1;    // 控制点索引的最大值(从0开始)
    m_ = n_ + p_ + 1;          // 节点向量索引的最大值

    // 初始化节点向量
    // 对于均匀B样条,节点按照固定间隔分布
    u_ = Eigen::VectorXd::Zero(m_ + 1);
    for (int i = 0; i <= m_; ++i)
    {
      // 前p_+1个节点(起始重复节点)
      if (i <= p_)
      {
        u_(i) = double(-p_ + i) * interval_;
      }
      // 中间的均匀节点
      else if (i > p_ && i <= m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
      // 后p_+1个节点(结束重复节点)
      else if (i > m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
    }
  }

  // 设置节点向量
  void UniformBspline::setKnot(const Eigen::VectorXd &knot) { this->u_ = knot; }

  // 获取节点向量
  Eigen::VectorXd UniformBspline::getKnot() { return this->u_; }

  /**
   * @brief 获取B样条的有效时间范围
   * @param um 起始时间(第p_个节点)
   * @param um_p 结束时间(第m_-p_个节点)
   * @return 是否成功获取时间范围
   *
   * B样条的有效参数范围在[u_p, u_{m-p}]之间
   */
  bool UniformBspline::getTimeSpan(double &um, double &um_p)
  {
    if (p_ > u_.rows() || m_ - p_ > u_.rows())
      return false;

    um = u_(p_);       // 有效起始节点
    um_p = u_(m_ - p_); // 有效结束节点

    return true;
  }

  // 获取控制点矩阵
  Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }

  /**
   * @brief 使用de Boor算法计算B样条在参数u处的值
   * @param u 参数值
   * @return 对应参数处的B样条曲线点
   *
   * de Boor算法是一种高效稳定的B样条求值算法,类似于贝塞尔曲线的de Casteljau算法
   * 算法复杂度: O(p^2),其中p是B样条阶数
   */
  Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u)
  {
    // 将参数u限制在有效范围内[u_p, u_{m-p}]
    double ub = min(max(u_(p_), u), u_(m_ - p_));

    // 确定参数u所在的节点区间[u_k, u_{k+1}]
    int k = p_;
    while (true)
    {
      if (u_(k + 1) >= ub)
        break;
      ++k;
    }

    /* de Boor算法 */
    // 初始化:选择p_+1个相关的控制点
    vector<Eigen::VectorXd> d;
    for (int i = 0; i <= p_; ++i)
    {
      d.push_back(control_points_.col(k - p_ + i));
      // cout << d[i].transpose() << endl;
    }

    // 递归计算:进行p_次迭代,每次减少一个点
    for (int r = 1; r <= p_; ++r)
    {
      for (int i = p_; i >= r; --i)
      {
        // 计算混合系数alpha
        double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
        // cout << "alpha: " << alpha << endl;
        // 线性插值更新控制点
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
      }
    }

    // 最后剩下的点即为所求的B样条值
    return d[p_];
  }

  // 注释掉的函数:使用相对时间t求值
  // Eigen::VectorXd UniformBspline::evaluateDeBoorT(const double& t) {
  //   return evaluateDeBoor(t + u_(p_));
  // }

  /**
   * @brief 计算B样条导数的控制点
   * @return 导数B样条的控制点矩阵
   *
   * B样条的导数仍然是一个B样条,其阶数降低为p_-1
   * 导数控制点的计算公式: Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
   * 其中Pi是原B样条的控制点,Qi是导数B样条的控制点
   */
  Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
  {
    // B样条的导数也是一个B样条,其阶数变为p_-1
    // 控制点公式: Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
    Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
    for (int i = 0; i < ctp.cols(); ++i)
    {
      ctp.col(i) =
          p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
    }
    return ctp;
  }

  /**
   * @brief 获取B样条的导数曲线
   * @return 导数B样条对象
   *
   * 通过计算导数控制点并构造新的B样条对象来获取导数曲线
   * 导数曲线的阶数比原曲线低1,节点向量需要裁剪首尾节点
   */
  UniformBspline UniformBspline::getDerivative()
  {
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    UniformBspline derivative(ctp, p_ - 1, interval_);

    /* 裁剪首尾节点 */
    // 导数B样条的节点向量比原始B样条少两个节点
    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);
    derivative.setKnot(knot);

    return derivative;
  }

  // 获取节点间隔
  double UniformBspline::getInterval() { return interval_; }

  /**
   * @brief 设置物理约束限制
   * @param vel 最大速度限制
   * @param acc 最大加速度限制
   * @param tolerance 可行性容忍度
   *
   * 这些限制用于检查轨迹的物理可行性
   */
  void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
  {
    limit_vel_ = vel;        // 速度限制
    limit_acc_ = acc;        // 加速度限制
    limit_ratio_ = 1.1;      // 限制比例(固定为1.1)
    feasibility_tolerance_ = tolerance;  // 可行性容忍度
  }

  /**
   * @brief 检查B样条轨迹的物理可行性
   * @param ratio 输出参数,表示需要的时间缩放比例以满足约束
   * @param show 是否显示不可行的点
   * @return 轨迹是否可行
   *
   * 该函数检查轨迹是否满足速度和加速度约束
   * 如果不可行,计算需要的时间缩放比例
   */
  bool UniformBspline::checkFeasibility(double &ratio, bool show)
  {
    bool fea = true;  // 可行性标志

    Eigen::MatrixXd P = control_points_;
    int dimension = control_points_.rows();  // 空间维度(通常是3:x,y,z)

    /* 检查速度可行性 */
    double max_vel = -1.0;
    // 扩大的速度限制(添加容忍度)
    double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 1; ++i)
    {
      // 通过相邻控制点的差分计算速度
      Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1));

      // 检查各个维度的速度是否超限
      if (fabs(vel(0)) > enlarged_vel_lim || fabs(vel(1)) > enlarged_vel_lim ||
          fabs(vel(2)) > enlarged_vel_lim)
      {
        if (show)
          cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
        fea = false;

        // 记录最大速度
        for (int j = 0; j < dimension; ++j)
        {
          max_vel = max(max_vel, fabs(vel(j)));
        }
      }
    }

    /* 检查加速度可行性 */
    double max_acc = -1.0;
    // 扩大的加速度限制(添加容忍度)
    double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 2; ++i)
    {
      // 通过二阶差分计算加速度
      // 加速度 = p*(p-1) * (速度差分) / 时间间隔
      Eigen::VectorXd acc = p_ * (p_ - 1) *
                            ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                             (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                            (u_(i + p_ + 1) - u_(i + 2));

      // 检查各个维度的加速度是否超限
      if (fabs(acc(0)) > enlarged_acc_lim || fabs(acc(1)) > enlarged_acc_lim ||
          fabs(acc(2)) > enlarged_acc_lim)
      {
        if (show)
          cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
        fea = false;

        // 记录最大加速度
        for (int j = 0; j < dimension; ++j)
        {
          max_acc = max(max_acc, fabs(acc(j)));
        }
      }
    }

    // 计算所需的时间缩放比例
    // 速度比例为线性关系,加速度比例为平方根关系(因为a=v/t)
    ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

    return fea;
  }

  /**
   * @brief 通过时间拉伸使轨迹满足物理约束
   * @param ratio 时间拉伸比例
   *
   * 该函数通过增加节点间隔来拉伸时间,从而降低速度和加速度
   * 只拉伸中间部分的节点,保持起始和结束部分不变
   */
  void UniformBspline::lengthenTime(const double &ratio)
  {
    int num1 = 5;  // 起始固定节点数
    int num2 = getKnot().rows() - 1 - 5;  // 结束固定节点索引

    // 计算总的时间增量
    double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
    // 计算每个节点的时间增量
    double t_inc = delta_t / double(num2 - num1);

    // 线性地增加中间节点的时间值
    for (int i = num1 + 1; i <= num2; ++i)
      u_(i) += double(i - num1) * t_inc;

    // 平移结束部分的所有节点
    for (int i = num2 + 1; i < u_.rows(); ++i)
      u_(i) += delta_t;
  }

  // 注释掉的函数:重新计算初始值
  // void UniformBspline::recomputeInit() {}

  /**
   * @brief 将点集参数化为B样条曲线
   * @param ts 时间步长
   * @param point_set 需要拟合的点集
   * @param start_end_derivative 起点和终点的速度和加速度约束[v_start, v_end, a_start, a_end]
   * @param ctrl_pts 输出的控制点矩阵
   *
   * 该函数通过求解线性系统将给定的点集拟合为B样条曲线
   * 同时满足起点和终点的速度、加速度约束
   */
  void UniformBspline::parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                             const vector<Eigen::Vector3d> &start_end_derivative,
                                             Eigen::MatrixXd &ctrl_pts)
  {
    // 输入参数检查
    if (ts <= 0)
    {
      cout << "[B-spline]:time step error." << endl;
      return;
    }

    if (point_set.size() <= 3)
    {
      cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
      return;
    }

    if (start_end_derivative.size() != 4)
    {
      cout << "[B-spline]:derivatives error." << endl;
    }

    int K = point_set.size();  // 点集数量

    // 构造系数矩阵A
    // 对于3阶B样条,基函数满足特定的系数关系
    Eigen::Vector3d prow(3), vrow(3), arow(3);
    prow << 1, 4, 1;    // 位置约束的基函数系数(归一化后为1/6)
    vrow << -1, 0, 1;   // 速度约束的基函数系数
    arow << 1, -2, 1;   // 加速度约束的基函数系数

    // 构造线性系统 A * ctrl_pts = b
    // 维度: (K+4) × (K+2), 其中K是点数,4是导数约束数,2是额外的控制点
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

    // 添加K个位置约束(每个点经过B样条曲线)
    for (int i = 0; i < K; ++i)
      A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

    // 添加起点和终点的速度约束
    A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

    // 添加起点和终点的加速度约束
    A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
    A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();

    //cout << "A" << endl << A << endl << endl;

    // 构造右侧向量b
    // 分别为x, y, z三个维度构造约束向量
    Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
    // 前K行: 位置约束
    for (int i = 0; i < K; ++i)
    {
      bx(i) = point_set[i](0);
      by(i) = point_set[i](1);
      bz(i) = point_set[i](2);
    }

    // 后4行: 起点和终点的速度、加速度约束
    // [v_start, v_end, a_start, a_end]
    for (int i = 0; i < 4; ++i)
    {
      bx(K + i) = start_end_derivative[i](0);
      by(K + i) = start_end_derivative[i](1);
      bz(K + i) = start_end_derivative[i](2);
    }

    // 求解线性系统 A*x = b
    // 使用列主元QR分解求解,这是一个数值稳定的方法
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

    // 将求解结果转换为控制点矩阵格式
    ctrl_pts.resize(3, K + 2);
    ctrl_pts.row(0) = px.transpose();
    ctrl_pts.row(1) = py.transpose();
    ctrl_pts.row(2) = pz.transpose();

    // cout << "[B-spline]: parameterization ok." << endl;
  }

  /**
   * @brief 获取B样条的总时间长度
   * @return 时间总长度,失败返回-1.0
   */
  double UniformBspline::getTimeSum()
  {
    double tm, tmp;
    if (getTimeSpan(tm, tmp))
      return tmp - tm;  // 返回有效时间范围
    else
      return -1.0;
  }

  /**
   * @brief 计算B样条曲线的长度
   * @param res 采样分辨率(时间步长)
   * @return 曲线总长度
   *
   * 通过在曲线上均匀采样并累加相邻点之间的距离来近似计算曲线长度
   */
  double UniformBspline::getLength(const double &res)
  {
    double length = 0.0;
    double dur = getTimeSum();  // 获取总时间
    Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;  // p_l:上一个点, p_n:当前点
    // 以res为步长采样曲线
    for (double t = res; t <= dur + 1e-4; t += res)
    {
      p_n = evaluateDeBoorT(t);
      length += (p_n - p_l).norm();  // 累加线段长度
      p_l = p_n;
    }
    return length;
  }

  /**
   * @brief 计算B样条轨迹的Jerk积分
   * @return Jerk的积分值(加加速度的平方积分)
   *
   * Jerk是加速度的导数(三阶导数),是衡量轨迹平滑性的重要指标
   * Jerk越小,轨迹越平滑
   */
  double UniformBspline::getJerk()
  {
    // 通过三次求导得到Jerk轨迹(位置->速度->加速度->Jerk)
    UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

    Eigen::VectorXd times = jerk_traj.getKnot();
    Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
    int dimension = ctrl_pts.rows();

    // 计算Jerk的积分: ∫(jerk²)dt
    // 对于B样条,可以通过控制点直接计算
    double jerk = 0.0;
    for (int i = 0; i < ctrl_pts.cols(); ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        // 每个节点区间的Jerk平方乘以时间间隔
        jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
      }
    }

    return jerk;
  }

  /**
   * @brief 计算轨迹的平均速度和最大速度
   * @param mean_v 输出参数:平均速度
   * @param max_v 输出参数:最大速度
   *
   * 通过在时间范围内采样速度曲线来统计速度信息
   */
  void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    UniformBspline vel = getDerivative();  // 获取速度曲线(一阶导数)
    double tm, tmp;
    vel.getTimeSpan(tm, tmp);  // 获取有效时间范围

    double max_vel = -1.0, mean_vel = 0.0;
    int num = 0;
    // 以0.01s为步长采样速度
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
      double vn = vxd.norm();  // 计算速度的模

      mean_vel += vn;
      ++num;
      if (vn > max_vel)
      {
        max_vel = vn;
      }
    }

    mean_vel = mean_vel / double(num);  // 计算平均值
    mean_v = mean_vel;
    max_v = max_vel;
  }

  /**
   * @brief 计算轨迹的平均加速度和最大加速度
   * @param mean_a 输出参数:平均加速度
   * @param max_a 输出参数:最大加速度
   *
   * 通过在时间范围内采样加速度曲线来统计加速度信息
   */
  void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    UniformBspline acc = getDerivative().getDerivative();  // 获取加速度曲线(二阶导数)
    double tm, tmp;
    acc.getTimeSpan(tm, tmp);  // 获取有效时间范围

    double max_acc = -1.0, mean_acc = 0.0;
    int num = 0;
    // 以0.01s为步长采样加速度
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd axd = acc.evaluateDeBoor(t);
      double an = axd.norm();  // 计算加速度的模

      mean_acc += an;
      ++num;
      if (an > max_acc)
      {
        max_acc = an;
      }
    }

    mean_acc = mean_acc / double(num);  // 计算平均值
    mean_a = mean_acc;
    max_a = max_acc;
  }
} // namespace ego_planner
