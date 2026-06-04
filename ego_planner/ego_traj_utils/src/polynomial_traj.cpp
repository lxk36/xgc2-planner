#include <iostream>
#include <ego_traj_utils/polynomial_traj.h>

/**
 * @brief 生成最小Snap轨迹（Minimum Snap Trajectory Generation）
 *
 * 该函数通过闭式解（Closed-form Solution）生成通过多个路径点的最小snap轨迹。
 * Snap是加加速度（jerk）的导数，即位置的四阶导数。最小化snap可以生成平滑且动力学可行的轨迹。
 *
 * @param Pos 路径点位置矩阵 (3 x n+1)，每列表示一个路径点的xyz坐标
 * @param start_vel 起点速度 (3维向量)
 * @param end_vel 终点速度 (3维向量)
 * @param start_acc 起点加速度 (3维向量)
 * @param end_acc 终点加速度 (3维向量)
 * @param Time 每段轨迹的时间分配 (n维向量)
 * @return PolynomialTraj 生成的多项式轨迹对象
 */
PolynomialTraj PolynomialTraj::minSnapTraj(const Eigen::MatrixXd &Pos, const Eigen::Vector3d &start_vel,
                                           const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_acc, const Eigen::VectorXd &Time)
{
  int seg_num = Time.size();  // 轨迹段数量
  Eigen::MatrixXd poly_coeff(seg_num, 3 * 6);  // 多项式系数矩阵：每段3个维度(xyz) x 6个系数(5次多项式)
  Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);  // xyz三个维度的多项式系数向量

  int num_f, num_p; // num_f: 固定变量数量; num_p: 自由变量数量
  int num_d;        // num_d: 所有段的导数约束总数

  // Lambda函数：计算阶乘
  const static auto Factorial = [](int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
      fac = fac * i;
    return fac;
  };

  /* ---------- 端点导数约束（End Point Derivative Constraints） ---------- */
  // 初始化导数向量：每段有6个约束（起点位置、终点位置、起点速度、终点速度、起点加速度、终点加速度）
  Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);  // x方向的导数约束
  Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);  // y方向的导数约束
  Eigen::VectorXd Dz = Eigen::VectorXd::Zero(seg_num * 6);  // z方向的导数约束

  // 为每段轨迹设置导数约束
  for (int k = 0; k < seg_num; k++)
  {
    /* 位置约束：每段的起点和终点位置 */
    Dx(k * 6) = Pos(0, k);          // 第k段起点的x坐标
    Dx(k * 6 + 1) = Pos(0, k + 1);  // 第k段终点的x坐标
    Dy(k * 6) = Pos(1, k);          // 第k段起点的y坐标
    Dy(k * 6 + 1) = Pos(1, k + 1);  // 第k段终点的y坐标
    Dz(k * 6) = Pos(2, k);          // 第k段起点的z坐标
    Dz(k * 6 + 1) = Pos(2, k + 1);  // 第k段终点的z坐标

    // 第一段轨迹：设置起点的速度和加速度约束
    if (k == 0)
    {
      Dx(k * 6 + 2) = start_vel(0);  // 起点x方向速度
      Dy(k * 6 + 2) = start_vel(1);  // 起点y方向速度
      Dz(k * 6 + 2) = start_vel(2);  // 起点z方向速度

      Dx(k * 6 + 4) = start_acc(0);  // 起点x方向加速度
      Dy(k * 6 + 4) = start_acc(1);  // 起点y方向加速度
      Dz(k * 6 + 4) = start_acc(2);  // 起点z方向加速度
    }
    // 最后一段轨迹：设置终点的速度和加速度约束
    else if (k == seg_num - 1)
    {
      Dx(k * 6 + 3) = end_vel(0);  // 终点x方向速度
      Dy(k * 6 + 3) = end_vel(1);  // 终点y方向速度
      Dz(k * 6 + 3) = end_vel(2);  // 终点z方向速度

      Dx(k * 6 + 5) = end_acc(0);  // 终点x方向加速度
      Dy(k * 6 + 5) = end_acc(1);  // 终点y方向加速度
      Dz(k * 6 + 5) = end_acc(2);  // 终点z方向加速度
    }
  }

  /* ---------- 映射矩阵A（Mapping Matrix A）---------- */
  // 矩阵A将多项式系数映射到导数空间
  // 对于5次多项式 p(t) = c5*t^5 + c4*t^4 + c3*t^3 + c2*t^2 + c1*t + c0
  // 矩阵A建立了系数[c5,c4,c3,c2,c1,c0]与约束[p(0), p(T), p'(0), p'(T), p''(0), p''(T)]的关系
  Eigen::MatrixXd Ab;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);  // 分块对角矩阵

  for (int k = 0; k < seg_num; k++)
  {
    Ab = Eigen::MatrixXd::Zero(6, 6);  // 每段的局部映射矩阵
    // 构造映射矩阵：i=0对应位置，i=1对应速度，i=2对应加速度
    for (int i = 0; i < 3; i++)
    {
      // 起点约束（t=0时）：第i阶导数
      Ab(2 * i, i) = Factorial(i);
      // 终点约束（t=T时）：第i阶导数
      for (int j = i; j < 6; j++)
        Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);
    }
    // 将局部矩阵放入全局分块对角矩阵A中
    A.block(k * 6, k * 6, 6, 6) = Ab;
  }

  /* ---------- 选择矩阵C'（Selection Matrix C'）---------- */
  // 选择矩阵C将导数约束分为固定变量（fixed）和自由变量（free）
  // 固定变量：起点/终点的位置、速度、加速度
  // 自由变量：中间路径点的速度和加速度（通过优化求解）
  Eigen::MatrixXd Ct, C;

  num_f = 2 * seg_num + 4; // 固定变量数量：起点3个(p,v,a) + 终点3个(p,v,a) + 中间点位置(seg_num-1)*2 = 2m + 4
  num_p = 2 * seg_num - 2; // 自由变量数量：中间点的速度和加速度 (seg_num - 1) * 2 = 2m - 2
  num_d = 6 * seg_num;     // 总约束数量：每段6个约束
  Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);  // 选择矩阵转置

  // 第一段的起点约束
  Ct(0, 0) = 1;  // 起点位置
  Ct(2, 1) = 1;  // 起点速度
  Ct(4, 2) = 1;  // 起点加速度

  // 第一段的终点约束
  Ct(1, 3) = 1;               // 第一段终点位置（也是第二段起点位置）
  Ct(3, 2 * seg_num + 4) = 1; // 第一段终点速度（自由变量）
  Ct(5, 2 * seg_num + 5) = 1; // 第一段终点加速度（自由变量）

  // 最后一段的起点和终点约束
  Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1; // 最后一段起点位置
  Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1; // 最后一段终点位置（全局终点）
  Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1; // 最后一段起点速度（自由变量）
  Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1; // 最后一段终点速度（固定）
  Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1; // 最后一段起点加速度（自由变量）
  Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1; // 最后一段终点加速度（固定）

  // 中间段的约束（第2段到第seg_num-1段）
  for (int j = 2; j < seg_num; j++)
  {
    Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1; // 起点位置（固定）
    Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1; // 终点位置（固定）
    Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1; // 起点速度（自由变量）
    Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1; // 终点速度（自由变量）
    Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1; // 起点加速度（自由变量）
    Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1; // 终点加速度（自由变量）
  }

  C = Ct.transpose();  // 获得选择矩阵C

  // 将原始导数约束向量映射到固定-自由变量空间
  Eigen::VectorXd Dx1 = C * Dx;
  Eigen::VectorXd Dy1 = C * Dy;
  Eigen::VectorXd Dz1 = C * Dz;

  /* ---------- 最小Snap代价矩阵Q（Minimum Snap Cost Matrix）---------- */
  // Q矩阵定义了轨迹的平滑性代价，通过最小化snap（四阶导数的积分）来获得平滑轨迹
  // 对于5次多项式，snap = d^4p/dt^4，积分形式为 J = ∫(d^4p/dt^4)^2 dt
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    // 只有3阶及以上的系数对snap有贡献（因为snap是4阶导数）
    for (int i = 3; i < 6; i++)
    {
      for (int j = 3; j < 6; j++)
      {
        // 计算Hessian矩阵元素：Q(i,j) = ∫(d^4/dt^4(t^i) * d^4/dt^4(t^j)) dt
        // 公式：i!/(i-4)! * j!/(j-4)! * t^(i+j-7) / (i+j-7)
        Q(k * 6 + i, k * 6 + j) =
            i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
      }
    }
  }

  /* ---------- R矩阵（映射后的代价矩阵）---------- */
  // R矩阵将Q矩阵从多项式系数空间映射到导数约束空间
  // R = C * A^(-T) * Q * A^(-1) * C^T
  // 这一步将优化问题从系数空间转换到约束空间，便于后续的闭式求解
  Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;

  // 提取固定变量对应的导数约束
  Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4), Dzf(2 * seg_num + 4);

  Dxf = Dx1.segment(0, 2 * seg_num + 4);  // x方向的固定约束
  Dyf = Dy1.segment(0, 2 * seg_num + 4);  // y方向的固定约束
  Dzf = Dz1.segment(0, 2 * seg_num + 4);  // z方向的固定约束

  // 将R矩阵分块为四个子矩阵
  // R = [Rff, Rfp]  其中f表示fixed（固定），p表示free（自由）
  //     [Rpf, Rpp]
  Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);  // 固定-固定块
  Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);  // 固定-自由块
  Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);  // 自由-固定块
  Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);  // 自由-自由块

  Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
  Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
  Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
  Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

  /* ---------- 闭式解（Closed-form Solution）---------- */
  // 通过求解二次规划问题的KKT条件，得到自由变量的闭式解
  // 优化问题：min J = D^T * R * D, 其中D = [Df; Dp]
  // 解为：Dp* = -Rpp^(-1) * Rfp^T * Df

  Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2), Dzp(2 * seg_num - 2);
  // 计算自由变量的最优解（中间路径点的速度和加速度）
  Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
  Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
  Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

  // 将求解得到的自由变量填入完整的导数向量中
  Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
  Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
  Dz1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;

  // 从导数约束反推多项式系数：P = A^(-1) * C^T * D
  Px = (A.inverse() * Ct) * Dx1;
  Py = (A.inverse() * Ct) * Dy1;
  Pz = (A.inverse() * Ct) * Dz1;

  // 整理多项式系数矩阵：每行对应一段轨迹，包含xyz三个维度的6个系数
  for (int i = 0; i < seg_num; i++)
  {
    poly_coeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();   // x方向的6个系数
    poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();   // y方向的6个系数
    poly_coeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();  // z方向的6个系数
  }

  /* ---------- 构造多项式轨迹对象 ---------- */
  // 将计算得到的多项式系数转换为PolynomialTraj对象
  PolynomialTraj poly_traj;
  for (int i = 0; i < poly_coeff.rows(); ++i)
  {
    vector<double> cx(6), cy(6), cz(6);
    // 提取第i段轨迹的xyz三个维度的系数
    for (int j = 0; j < 6; ++j)
    {
      cx[j] = poly_coeff(i, j);       // x方向第j个系数
      cy[j] = poly_coeff(i, j + 6);   // y方向第j个系数
      cz[j] = poly_coeff(i, j + 12);  // z方向第j个系数
    }
    // 反转系数顺序：从[c0,c1,c2,c3,c4,c5]变为[c5,c4,c3,c2,c1,c0]
    // 这是因为多项式表示可能需要从高次项到低次项的顺序
    reverse(cx.begin(), cx.end());
    reverse(cy.begin(), cy.end());
    reverse(cz.begin(), cz.end());
    double ts = Time(i);  // 第i段的持续时间
    poly_traj.addSegment(cx, cy, cz, ts);  // 添加轨迹段
  }

  return poly_traj;
}

/**
 * @brief 生成单段轨迹（Single Segment Trajectory Generation）
 *
 * 该函数生成一段5次多项式轨迹，满足给定的起点和终点约束（位置、速度、加速度）
 * 通过求解线性方程组 C * coefficients = boundary_conditions 得到多项式系数
 *
 * @param start_pt 起点位置 (3维向量)
 * @param start_vel 起点速度 (3维向量)
 * @param start_acc 起点加速度 (3维向量)
 * @param end_pt 终点位置 (3维向量)
 * @param end_vel 终点速度 (3维向量)
 * @param end_acc 终点加速度 (3维向量)
 * @param t 轨迹持续时间
 * @return PolynomialTraj 生成的单段多项式轨迹对象
 */
PolynomialTraj PolynomialTraj::one_segment_traj_gen(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
                                                    double t)
{
  // 系数矩阵C：建立多项式系数与边界条件的关系
  // 对于5次多项式 p(τ) = c5*τ^5 + c4*τ^4 + c3*τ^3 + c2*τ^2 + c1*τ + c0
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6), Crow(1, 6);
  Eigen::VectorXd Bx(6), By(6), Bz(6);  // 边界条件向量

  // 构造系数矩阵C
  // 第0行：p(0) = c0，即起点位置约束
  C(0, 5) = 1;
  // 第1行：p'(0) = c1，即起点速度约束
  C(1, 4) = 1;
  // 第2行：p''(0) = 2*c2，即起点加速度约束
  C(2, 3) = 2;

  // 第3行：p(t) = c5*t^5 + c4*t^4 + c3*t^3 + c2*t^2 + c1*t + c0，即终点位置约束
  Crow << pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1;
  C.row(3) = Crow;

  // 第4行：p'(t) = 5*c5*t^4 + 4*c4*t^3 + 3*c3*t^2 + 2*c2*t + c1，即终点速度约束
  Crow << 5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0;
  C.row(4) = Crow;

  // 第5行：p''(t) = 20*c5*t^3 + 12*c4*t^2 + 6*c3*t + 2*c2，即终点加速度约束
  Crow << 20 * pow(t, 3), 12 * pow(t, 2), 6 * t, 2, 0, 0;
  C.row(5) = Crow;

  // 边界条件向量B：包含起点和终点的位置、速度、加速度
  Bx << start_pt(0), start_vel(0), start_acc(0), end_pt(0), end_vel(0), end_acc(0);
  By << start_pt(1), start_vel(1), start_acc(1), end_pt(1), end_vel(1), end_acc(1);
  Bz << start_pt(2), start_vel(2), start_acc(2), end_pt(2), end_vel(2), end_acc(2);

  // 求解线性方程组 C * coefficients = B，得到多项式系数
  // 使用列主元QR分解求解，数值稳定性好
  Eigen::VectorXd Cofx = C.colPivHouseholderQr().solve(Bx);
  Eigen::VectorXd Cofy = C.colPivHouseholderQr().solve(By);
  Eigen::VectorXd Cofz = C.colPivHouseholderQr().solve(Bz);

  // 将系数转换为std::vector格式
  vector<double> cx(6), cy(6), cz(6);
  for (int i = 0; i < 6; i++)
  {
    cx[i] = Cofx(i);  // x方向的多项式系数
    cy[i] = Cofy(i);  // y方向的多项式系数
    cz[i] = Cofz(i);  // z方向的多项式系数
  }

  // 构造并返回轨迹对象
  PolynomialTraj poly_traj;
  poly_traj.addSegment(cx, cy, cz, t);

  return poly_traj;
}
