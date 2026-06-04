// 头文件保护宏，防止重复包含
#ifndef _POLYNOMIAL_TRAJ_H
#define _POLYNOMIAL_TRAJ_H

// Eigen库：用于线性代数运算，提供向量、矩阵等数学工具
#include <Eigen/Eigen>
// C++标准库：动态数组容器
#include <vector>

using std::vector;

/**
 * @brief 多项式轨迹类
 *
 * 该类实现了分段多项式轨迹的表示和操作，用于无人机或机器人的轨迹规划。
 * 轨迹由多个多项式段组成，每段在x、y、z三个轴上独立使用多项式表示。
 * 支持轨迹评估、速度/加速度计算、以及各种轨迹指标的统计。
 *
 * 核心功能：
 * 1. 轨迹表示：采用分段多项式表示，每段独立定义x/y/z轴的位置函数
 * 2. 轨迹评估：可计算任意时刻的位置、速度、加速度
 * 3. 轨迹生成：支持最小Snap轨迹生成(Minimum Snap Trajectory)
 * 4. 轨迹分析：计算轨迹长度、平均/最大速度、加速度、Jerk代价等指标
 *
 * 多项式表示说明：
 * - 每段轨迹在单个轴上的位置表示为：p(t) = c_(n-1)*t^(n-1) + ... + c_1*t + c_0
 * - 系数存储顺序：从高次到低次 [c_(n-1), c_(n-2), ..., c_1, c_0]
 * - 速度 v(t) = dp/dt，加速度 a(t) = d²p/dt²，jerk j(t) = d³p/dt³
 *
 * 使用流程：
 * 1. 创建对象：PolynomialTraj traj;
 * 2. 添加轨迹段：traj.addSegment(cx, cy, cz, t);
 * 3. 初始化：traj.init();
 * 4. 评估/分析：evaluate(), getLength(), getJerk() 等
 */
class PolynomialTraj
{
private:
  /* === 轨迹定义相关变量 === */
  vector<double> times;       // 每段轨迹的持续时间(单位:秒)
  vector<vector<double>> cxs; // 每段轨迹x轴的多项式系数，从高次到低次排列[c_(n-1), ..., c_0]
  vector<vector<double>> cys; // 每段轨迹y轴的多项式系数，从高次到低次排列[c_(n-1), ..., c_0]
  vector<vector<double>> czs; // 每段轨迹z轴的多项式系数，从高次到低次排列[c_(n-1), ..., c_0]

  double time_sum;  // 轨迹总时间(所有段时间之和)
  int num_seg;      // 轨迹段数

  /* === 轨迹评估缓存变量 === */
  vector<Eigen::Vector3d> traj_vec3d;  // 轨迹采样点序列(用于长度计算)
  double length;                        // 轨迹总长度(米)

public:
  /**
   * @brief 默认构造函数
   */
  PolynomialTraj(/* args */)
  {
  }

  /**
   * @brief 析构函数
   */
  ~PolynomialTraj()
  {
  }

  /**
   * @brief 重置轨迹数据
   *
   * 清空所有轨迹段的系数、时间等信息，将轨迹恢复到初始状态
   */
  void reset()
  {
    times.clear(), cxs.clear(), cys.clear(), czs.clear();
    time_sum = 0.0, num_seg = 0;
  }

  /**
   * @brief 添加一段轨迹
   *
   * @param cx x轴多项式系数向量
   * @param cy y轴多项式系数向量
   * @param cz z轴多项式系数向量
   * @param t 该段轨迹的持续时间
   */
  void addSegment(vector<double> cx, vector<double> cy, vector<double> cz, double t)
  {
    cxs.push_back(cx), cys.push_back(cy), czs.push_back(cz), times.push_back(t);
  }

  /**
   * @brief 初始化轨迹
   *
   * 计算轨迹段数和总时间，必须在添加所有轨迹段后调用
   */
  void init()
  {
    num_seg = times.size();
    time_sum = 0.0;
    for (int i = 0; i < times.size(); ++i)
    {
      time_sum += times[i];
    }
  }

  /**
   * @brief 获取所有轨迹段的时间向量
   *
   * @return vector<double> 时间向量
   */
  vector<double> getTimes()
  {
    return times;
  }

  /**
   * @brief 获取指定轴的多项式系数
   *
   * @param axis 轴索引(0:x轴, 1:y轴, 2:z轴)
   * @return vector<vector<double>> 该轴所有段的多项式系数
   *                                 如果轴索引非法，返回空向量并输出红色错误信息
   */
  vector<vector<double>> getCoef(int axis)
  {
    switch (axis)
    {
    case 0:
      return cxs;  // 返回x轴系数
    case 1:
      return cys;  // 返回y轴系数
    case 2:
      return czs;  // 返回z轴系数
    default:
      // 非法轴索引，输出红色错误信息 (\033[31m为红色，\033[0m为重置)
      std::cout << "\033[31mIllegal axis!\033[0m" << std::endl;
    }

    // 返回空向量(仅在axis非法时执行)
    vector<vector<double>> empty;
    return empty;
  }

  /**
   * @brief 在给定时刻评估轨迹位置
   *
   * 根据时间t计算对应的三维位置坐标。首先确定时间t所属的轨迹段，
   * 然后使用该段的多项式系数计算位置。
   *
   * 多项式表示：p(t) = c_(n-1)*t^(n-1) + c_(n-2)*t^(n-2) + ... + c_1*t + c_0
   * 其中系数向量cx,cy,cz按从高次到低次存储：[c_(n-1), c_(n-2), ..., c_1, c_0]
   *
   * @param t 时间(相对于轨迹起点)
   * @return Eigen::Vector3d 该时刻的三维位置
   */
  Eigen::Vector3d evaluate(double t)
  {
    /* 确定时间t所属的轨迹段索引 */
    int idx = 0;
    // 遍历各段时间，找到t所在的段，加1e-4是为了数值稳定性
    while (times[idx] + 1e-4 < t)
    {
      t -= times[idx];  // 减去前面段的时间，得到段内相对时间
      ++idx;
    }

    /* 评估该段轨迹在时刻t的位置 */
    int order = cxs[idx].size();  // 多项式阶数(系数个数)
    Eigen::VectorXd cx(order), cy(order), cz(order), tv(order);
    for (int i = 0; i < order; ++i)
    {
      // 提取该段三个轴的多项式系数
      cx(i) = cxs[idx][i], cy(i) = cys[idx][i], cz(i) = czs[idx][i];
      // 构造时间幂次向量，注意索引倒序：tv = [t^(n-1), t^(n-2), ..., t, 1]
      tv(order - 1 - i) = std::pow(t, double(i));
    }

    // 通过点积计算多项式的值：p = tv · c = c_(n-1)*t^(n-1) + ... + c_0
    Eigen::Vector3d pt;
    pt(0) = tv.dot(cx), pt(1) = tv.dot(cy), pt(2) = tv.dot(cz);
    return pt;
  }

  /**
   * @brief 在给定时刻评估轨迹速度
   *
   * 通过对位置多项式求一阶导数得到速度。速度是位置对时间的导数。
   *
   * @param t 时间(相对于轨迹起点)
   * @return Eigen::Vector3d 该时刻的三维速度
   */
  Eigen::Vector3d evaluateVel(double t)
  {
    /* 确定时间t所属的轨迹段索引 */
    int idx = 0;
    while (times[idx] + 1e-4 < t)
    {
      t -= times[idx];  // 减去前面段的时间，得到段内相对时间
      ++idx;
    }

    /* 计算速度多项式系数 */
    int order = cxs[idx].size();
    Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);

    /* 对位置多项式求一阶导数得到速度系数
     * 若位置为 p(t) = c_n*t^n + ... + c_1*t + c_0
     * 则速度为 v(t) = n*c_n*t^(n-1) + ... + c_1
     */
    for (int i = 0; i < order - 1; ++i)
    {
      vx(i) = double(i + 1) * cxs[idx][order - 2 - i];
      vy(i) = double(i + 1) * cys[idx][order - 2 - i];
      vz(i) = double(i + 1) * czs[idx][order - 2 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 1);
    for (int i = 0; i < order - 1; ++i)
      tv(i) = pow(ts, i);  // 构造时间幂次向量 [1, t, t^2, ..., t^(n-2)]

    Eigen::Vector3d vel;
    vel(0) = tv.dot(vx), vel(1) = tv.dot(vy), vel(2) = tv.dot(vz);  // 速度多项式求值
    return vel;
  }

  /**
   * @brief 在给定时刻评估轨迹加速度
   *
   * 通过对位置多项式求二阶导数得到加速度。加速度是位置对时间的二阶导数。
   *
   * @param t 时间(相对于轨迹起点)
   * @return Eigen::Vector3d 该时刻的三维加速度
   */
  Eigen::Vector3d evaluateAcc(double t)
  {
    /* 确定时间t所属的轨迹段索引 */
    int idx = 0;
    while (times[idx] + 1e-4 < t)
    {
      t -= times[idx];  // 减去前面段的时间，得到段内相对时间
      ++idx;
    }

    /* 计算加速度多项式系数 */
    int order = cxs[idx].size();
    Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

    /* 对位置多项式求二阶导数得到加速度系数
     * 若位置为 p(t) = c_n*t^n + ... + c_2*t^2 + c_1*t + c_0
     * 则加速度为 a(t) = n*(n-1)*c_n*t^(n-2) + ... + 2*c_2
     */
    for (int i = 0; i < order - 2; ++i)
    {
      ax(i) = double((i + 2) * (i + 1)) * cxs[idx][order - 3 - i];
      ay(i) = double((i + 2) * (i + 1)) * cys[idx][order - 3 - i];
      az(i) = double((i + 2) * (i + 1)) * czs[idx][order - 3 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 2);
    for (int i = 0; i < order - 2; ++i)
      tv(i) = pow(ts, i);  // 构造时间幂次向量 [1, t, t^2, ..., t^(n-3)]

    Eigen::Vector3d acc;
    acc(0) = tv.dot(ax), acc(1) = tv.dot(ay), acc(2) = tv.dot(az);  // 加速度多项式求值
    return acc;
  }

  /**
   * @brief 获取轨迹总时间
   *
   * @return double 轨迹总时间
   */
  double getTimeSum()
  {
    return this->time_sum;
  }

  /**
   * @brief 获取轨迹的采样点序列
   *
   * 以0.01秒为间隔对整条轨迹进行采样，生成位置点序列。
   * 注意：此函数会修改内部状态，必须按顺序调用(先getTraj，再getLength)
   *
   * @return vector<Eigen::Vector3d> 轨迹采样点序列
   */
  vector<Eigen::Vector3d> getTraj()
  {
    double eval_t = 0.0;
    traj_vec3d.clear();
    while (eval_t < time_sum)
    {
      Eigen::Vector3d pt = evaluate(eval_t);
      traj_vec3d.push_back(pt);
      eval_t += 0.01;  // 采样间隔0.01秒
    }
    return traj_vec3d;
  }

  /**
   * @brief 计算轨迹总长度
   *
   * 基于getTraj()生成的采样点计算轨迹长度。
   * 注意：必须先调用getTraj()才能调用此函数
   *
   * @return double 轨迹总长度(米)
   */
  double getLength()
  {
    length = 0.0;

    Eigen::Vector3d p_l = traj_vec3d[0], p_n;  // p_l:上一个点, p_n:当前点
    for (int i = 1; i < traj_vec3d.size(); ++i)
    {
      p_n = traj_vec3d[i];
      length += (p_n - p_l).norm();  // 累加相邻点之间的距离
      p_l = p_n;
    }
    return length;
  }

  /**
   * @brief 计算平均速度
   *
   * 通过轨迹长度除以总时间计算平均速度。
   *
   * !!!警告：此函数存在BUG，缺少return语句，会返回未定义值!!!
   * 应该在最后添加：return mean_vel;
   *
   * @return double 平均速度(米/秒) - 当前返回值未定义
   */
  double getMeanVel()
  {
    double mean_vel = length / time_sum;
    // BUG: 缺少 return mean_vel; 语句
  }

  /**
   * @brief 计算加速度代价
   *
   * 计算轨迹的加速度代价，用于评估轨迹的平滑性。
   * 代价基于加速度项的二次积分近似计算。
   *
   * 对于多项式 p(t) = c_(n-1)*t^(n-1) + ... + c_2*t^2 + c_1*t + c_0
   * 加速度 a(t) = (n-1)*(n-2)*c_(n-1)*t^(n-3) + ... + 2*c_2
   * 此处简化计算，只考虑常数项 2*c_2 的贡献
   * 代价 = ∫(a²)dt ≈ (2*c_2)² * T，其中T为时间段长度
   *
   * @return double 加速度代价
   */
  double getAccCost()
  {
    double cost = 0.0;
    int order = cxs[0].size();  // 多项式阶数

    for (int s = 0; s < times.size(); ++s)
    {
      Eigen::Vector3d um;
      // 提取二次项系数c_2(索引为order-3)，对应加速度的常数项
      // 由于 d²/dt²(c_2*t²) = 2*c_2，所以乘以2
      um(0) = 2 * cxs[s][order - 3], um(1) = 2 * cys[s][order - 3], um(2) = 2 * czs[s][order - 3];
      // 累加每段的加速度代价：||a||² * Δt
      cost += um.squaredNorm() * times[s];
    }

    return cost;
  }

  /**
   * @brief 计算Jerk代价
   *
   * 计算轨迹的jerk(急动度/加加速度)代价，jerk是加速度对时间的导数。
   * 通过对加速度求导并计算其二次积分来评估轨迹的平滑性。
   * Jerk越小，轨迹越平滑，对执行器的冲击越小。
   *
   * 数学原理：
   * 对于多项式 p(t) = Σ c_i * t^i (i从0到n-1)
   * Jerk j(t) = d³p/dt³ = Σ i*(i-1)*(i-2) * c_i * t^(i-3) (i从3到n-1)
   * Jerk代价 = ∫₀ᵀ j(t)² dt = cᵀ * Q * c
   * 其中Q矩阵的元素 Q(i,j) = i*(i-1)*(i-2)*j*(j-1)*(j-2) * T^(i+j-5) / (i+j-5)
   *
   * @return double Jerk代价(加加速度的平方积分)
   */
  double getJerk()
  {
    double jerk = 0.0;

    /* 计算每段轨迹的jerk代价 */
    for (int s = 0; s < times.size(); ++s)
    {
      Eigen::VectorXd cxv(cxs[s].size()), cyv(cys[s].size()), czv(czs[s].size());
      /* 转换系数顺序：从高次到低次 -> 从低次到高次
       * 原存储：[c_(n-1), c_(n-2), ..., c_1, c_0]
       * 转换后：[c_0, c_1, ..., c_(n-2), c_(n-1)]
       */
      int order = cxs[s].size();
      for (int j = 0; j < order; ++j)
      {
        cxv(j) = cxs[s][order - 1 - j], cyv(j) = cys[s][order - 1 - j], czv(j) = czs[s][order - 1 - j];
      }
      double ts = times[s];  // 该段轨迹的时间长度

      /* 构造jerk积分矩阵Q
       * Jerk = d³p/dt³，jerk代价为 ∫(jerk²)dt = cᵀQc
       * Q(i,j)对应t^i和t^j项在积分∫₀ᵀ t^(i+j-6) dt中的贡献
       */
      Eigen::MatrixXd mat_jerk(order, order);
      mat_jerk.setZero();
      // 只有i,j >= 3时才有jerk项(三阶导数后t^0,t^1,t^2项消失)
      for (int i = 3; i < order; i += 1)
        for (int j = 3; j < order; j += 1)
        {
          // 计算jerk的二次型积分矩阵元素
          // d³(c_i*t^i)/dt³ = i*(i-1)*(i-2)*c_i*t^(i-3)
          // ∫₀ᵀ [i*(i-1)*(i-2)*t^(i-3)] * [j*(j-1)*(j-2)*t^(j-3)] dt
          // = i*(i-1)*(i-2)*j*(j-1)*(j-2) * ∫₀ᵀ t^(i+j-6) dt
          // = i*(i-1)*(i-2)*j*(j-1)*(j-2) * T^(i+j-5) / (i+j-5)
          mat_jerk(i, j) =
              i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) * pow(ts, i + j - 5) / (i + j - 5);
        }

      // 累加三个轴的jerk代价：Jx + Jy + Jz
      // 每个轴的代价为 c^T * Q * c (向量的二次型)
      jerk += (cxv.transpose() * mat_jerk * cxv)(0, 0);
      jerk += (cyv.transpose() * mat_jerk * cyv)(0, 0);
      jerk += (czv.transpose() * mat_jerk * czv)(0, 0);
    }

    return jerk;
  }

  /**
   * @brief 计算平均速度和最大速度
   *
   * 通过对轨迹采样(间隔0.01秒)，统计所有采样点的速度，
   * 计算平均速度和最大速度。
   *
   * !!!注意：此函数存在BUG，在计算时间幂次时使用了ts而非eval_t!!!
   * 第407行应该是 tv(i) = pow(eval_t, i); 而不是 pow(ts, i);
   *
   * @param mean_v 输出参数：平均速度(米/秒)
   * @param max_v 输出参数：最大速度(米/秒)
   */
  void getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    int num = 0;  // 采样点数量
    mean_v = 0.0, max_v = -1.0;
    for (int s = 0; s < times.size(); ++s)
    {
      int order = cxs[s].size();
      Eigen::VectorXd vx(order - 1), vy(order - 1), vz(order - 1);

      /* 计算速度多项式系数(对位置多项式求一阶导数) */
      for (int i = 0; i < order - 1; ++i)
      {
        vx(i) = double(i + 1) * cxs[s][order - 2 - i];
        vy(i) = double(i + 1) * cys[s][order - 2 - i];
        vz(i) = double(i + 1) * czs[s][order - 2 - i];
      }
      double ts = times[s];  // 该段的总时间

      // 对该段轨迹进行采样
      double eval_t = 0.0;
      while (eval_t < ts)
      {
        Eigen::VectorXd tv(order - 1);
        for (int i = 0; i < order - 1; ++i)
          tv(i) = pow(ts, i);  // BUG: 应该是 pow(eval_t, i)，构造时间幂次向量 [1, t, t^2, ...]
        Eigen::Vector3d vel;
        vel(0) = tv.dot(vx), vel(1) = tv.dot(vy), vel(2) = tv.dot(vz);
        double vn = vel.norm();  // 速度模长
        mean_v += vn;            // 累加速度用于计算平均值
        if (vn > max_v)
          max_v = vn;  // 更新最大速度
        ++num;

        eval_t += 0.01;  // 采样间隔0.01秒
      }
    }

    mean_v = mean_v / double(num);  // 计算平均速度
  }

  /**
   * @brief 计算平均加速度和最大加速度
   *
   * 通过对轨迹采样(间隔0.01秒)，统计所有采样点的加速度，
   * 计算平均加速度和最大加速度。
   *
   * !!!注意：此函数存在BUG，在计算时间幂次时使用了ts而非eval_t!!!
   * 第494行应该是 tv(i) = pow(eval_t, i); 而不是 pow(ts, i);
   *
   * @param mean_a 输出参数：平均加速度(米/秒²)
   * @param max_a 输出参数：最大加速度(米/秒²)
   */
  void getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    int num = 0;  // 采样点数量
    mean_a = 0.0, max_a = -1.0;
    for (int s = 0; s < times.size(); ++s)
    {
      int order = cxs[s].size();
      Eigen::VectorXd ax(order - 2), ay(order - 2), az(order - 2);

      /* 计算加速度多项式系数(对位置多项式求二阶导数) */
      for (int i = 0; i < order - 2; ++i)
      {
        ax(i) = double((i + 2) * (i + 1)) * cxs[s][order - 3 - i];
        ay(i) = double((i + 2) * (i + 1)) * cys[s][order - 3 - i];
        az(i) = double((i + 2) * (i + 1)) * czs[s][order - 3 - i];
      }
      double ts = times[s];  // 该段的总时间

      // 对该段轨迹进行采样
      double eval_t = 0.0;
      while (eval_t < ts)
      {
        Eigen::VectorXd tv(order - 2);
        for (int i = 0; i < order - 2; ++i)
          tv(i) = pow(ts, i);  // BUG: 应该是 pow(eval_t, i)，构造时间幂次向量 [1, t, t^2, ...]
        Eigen::Vector3d acc;
        acc(0) = tv.dot(ax), acc(1) = tv.dot(ay), acc(2) = tv.dot(az);
        double an = acc.norm();  // 加速度模长
        mean_a += an;            // 累加加速度用于计算平均值
        if (an > max_a)
          max_a = an;  // 更新最大加速度
        ++num;

        eval_t += 0.01;  // 采样间隔0.01秒
      }
    }

    mean_a = mean_a / double(num);  // 计算平均加速度
  }

  /**
   * @brief 生成最小Snap轨迹(静态方法)
   *
   * 基于给定的路径点、起止速度和加速度，生成最小snap(加速度的导数)的多项式轨迹。
   * Minimum Snap轨迹优化是一种常用的平滑轨迹生成方法，通过最小化snap的积分
   * 来生成平滑、适合四旋翼等飞行器执行的轨迹。
   *
   * @param Pos 路径点矩阵，每列为一个三维路径点
   * @param start_vel 起始速度
   * @param end_vel 终止速度
   * @param start_acc 起始加速度
   * @param end_acc 终止加速度
   * @param Time 每段轨迹的时间向量
   * @return PolynomialTraj 生成的多项式轨迹
   */
  static PolynomialTraj minSnapTraj(const Eigen::MatrixXd &Pos, const Eigen::Vector3d &start_vel,
                                    const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                                    const Eigen::Vector3d &end_acc, const Eigen::VectorXd &Time);

  /**
   * @brief 生成单段轨迹(静态方法)
   *
   * 根据起点和终点的位置、速度、加速度，生成一段多项式轨迹。
   * 这是一个便捷函数，用于生成点到点的轨迹段。
   *
   * 该函数通过求解边界值问题，确定满足起点和终点约束的多项式系数。
   * 对于n阶多项式，需要n+1个约束条件(起点和终点的位置、速度、加速度等)。
   *
   * @param start_pt 起点位置
   * @param start_vel 起点速度
   * @param start_acc 起点加速度
   * @param end_pt 终点位置
   * @param end_vel 终点速度
   * @param end_acc 终点加速度
   * @param t 轨迹段的持续时间
   * @return PolynomialTraj 生成的单段多项式轨迹
   */
  static PolynomialTraj one_segment_traj_gen(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                             const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
                                             double t);
};

#endif  // _POLYNOMIAL_TRAJ_H 结束头文件保护
