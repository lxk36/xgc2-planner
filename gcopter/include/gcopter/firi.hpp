/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

/* This is an old version of FIRI for temporary usage here. */
/* 这是FIRI的旧版本，在此处临时使用 */

#ifndef FIRI_HPP
#define FIRI_HPP

#include "lbfgs.hpp"  // L-BFGS优化器
#include "sdlp.hpp"   // 单纯形线性规划求解器

#include <Eigen/Eigen>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cfloat>
#include <cmath>
#include <vector>

// FIRI: Fast Inscribed Radius Inflation（快速内切半径膨胀）算法命名空间
// 用于在给定边界和障碍物约束下，寻找最大体积的内切椭球
namespace firi
{

    /**
     * @brief 计算3x3对称正定矩阵的Cholesky分解
     *
     * @param A 输入的3x3对称正定矩阵，需要满足 A = L * L^T
     * @param L 输出的3x3下三角矩阵（Cholesky因子）
     *
     * @note 该函数使用显式公式直接计算Cholesky分解，避免通用算法的额外开销
     *       L是下三角矩阵，满足 A = L * L^T
     */
    inline void chol3d(const Eigen::Matrix3d &A,
                       Eigen::Matrix3d &L)
    {
        // 计算下三角矩阵L的各个元素
        L(0, 0) = sqrt(A(0, 0));                                              // L_00 = sqrt(A_00)
        L(0, 1) = 0.0;                                                        // 上三角部分为0
        L(0, 2) = 0.0;
        L(1, 0) = 0.5 * (A(0, 1) + A(1, 0)) / L(0, 0);                       // L_10 = A_01 / L_00
        L(1, 1) = sqrt(A(1, 1) - L(1, 0) * L(1, 0));                         // L_11 = sqrt(A_11 - L_10^2)
        L(1, 2) = 0.0;
        L(2, 0) = 0.5 * (A(0, 2) + A(2, 0)) / L(0, 0);                       // L_20 = A_02 / L_00
        L(2, 1) = (0.5 * (A(1, 2) + A(2, 1)) - L(2, 0) * L(1, 0)) / L(1, 1); // L_21 = (A_12 - L_20*L_10) / L_11
        L(2, 2) = sqrt(A(2, 2) - L(2, 0) * L(2, 0) - L(2, 1) * L(2, 1));     // L_22 = sqrt(A_22 - L_20^2 - L_21^2)
        return;
    }

    /**
     * @brief 平滑L1罚函数及其梯度计算
     *
     * @param mu 平滑参数，控制平滑区域的大小
     * @param x  输入变量（通常是约束违反量）
     * @param f  输出的函数值
     * @param df 输出的导数值
     * @return 如果x>=0返回true，否则返回false
     *
     * @note 该函数实现了一个平滑的L1罚函数：
     *       - 当 x < 0: 无效，返回false
     *       - 当 x > mu: f = x - 0.5*mu (线性区域)
     *       - 当 0 <= x <= mu: f = (mu - 0.5*x) * (x/mu)^3 (三次平滑过渡区域)
     *       这种平滑化避免了标准L1函数在0处的不可微性
     */
    inline bool smoothedL1(const double &mu,
                           const double &x,
                           double &f,
                           double &df)
    {
        if (x < 0.0)
        {
            return false;  // x必须非负
        }
        else if (x > mu)
        {
            // 线性区域：当违反量较大时
            f = x - 0.5 * mu;  // 函数值
            df = 1.0;          // 导数为常数1
            return true;
        }
        else
        {
            // 平滑过渡区域：0 <= x <= mu
            const double xdmu = x / mu;                           // x/mu
            const double sqrxdmu = xdmu * xdmu;                   // (x/mu)^2
            const double mumxd2 = mu - 0.5 * x;                   // mu - 0.5*x
            f = mumxd2 * sqrxdmu * xdmu;                          // (mu - 0.5*x) * (x/mu)^3
            df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);   // 导数
            return true;
        }
    }

    /**
     * @brief 最大体积内切椭球(MVIE)优化的代价函数及梯度计算
     *
     * @param data 优化参数数据指针，包含约束数量M、平滑参数、罚函数权重和约束矩阵A
     * @param x    优化变量向量 [p(3), rtd(3), cde(3)]，其中：
     *             - p: 椭球中心位置 (3D)
     *             - rtd: 椭球主轴长度的平方根 (3D)
     *             - cde: Cholesky分解的非对角元素 (3D)
     * @param grad 输出的梯度向量
     * @return 总代价值（罚函数项 - 体积对数项）
     *
     * @note 目标函数设计：
     *       1. 最大化椭球体积（等价于最小化 -log(det(L))，其中L是椭球矩阵的Cholesky因子）
     *       2. 最小化约束违反（使用平滑L1罚函数）
     *       约束形式: ||A_i * L||_2 + A_i * p <= 1，确保椭球在多面体内部
     */
    inline double costMVIE(void *data,
                           const Eigen::VectorXd &x,
                           Eigen::VectorXd &grad)
    {
        // 从data指针解析优化参数
        const int64_t *pM = (int64_t *)data;                  // 约束数量
        const double *pSmoothEps = (double *)(pM + 1);        // 平滑参数epsilon
        const double *pPenaltyWt = pSmoothEps + 1;            // 罚函数权重
        const double *pA = pPenaltyWt + 1;                    // 约束矩阵A的数据指针

        const int M = *pM;                                    // 约束数量
        const double smoothEps = *pSmoothEps;                 // 平滑L1函数的平滑参数
        const double penaltyWt = *pPenaltyWt;                 // 罚函数权重
        Eigen::Map<const Eigen::MatrixX3d> A(pA, M, 3);       // M×3约束矩阵

        // 将优化变量映射为3个向量
        Eigen::Map<const Eigen::Vector3d> p(x.data());        // 椭球中心位置
        Eigen::Map<const Eigen::Vector3d> rtd(x.data() + 3);  // 椭球主轴长度的平方根
        Eigen::Map<const Eigen::Vector3d> cde(x.data() + 6);  // Cholesky分解的非对角元素

        // 将梯度映射为3个向量
        Eigen::Map<Eigen::Vector3d> gdp(grad.data());         // 对p的梯度
        Eigen::Map<Eigen::Vector3d> gdrtd(grad.data() + 3);   // 对rtd的梯度
        Eigen::Map<Eigen::Vector3d> gdcde(grad.data() + 6);   // 对cde的梯度

        // 初始化代价和梯度
        double cost = 0;
        gdp.setZero();
        gdrtd.setZero();
        gdcde.setZero();

        // 构造Cholesky因子L（下三角矩阵）
        // L表示椭球的形状矩阵Q的Cholesky分解，Q = L * L^T
        Eigen::Matrix3d L;
        L(0, 0) = rtd(0) * rtd(0) + DBL_EPSILON;  // 对角元素的平方，加epsilon防止数值问题
        L(0, 1) = 0.0;                             // 上三角部分为0
        L(0, 2) = 0.0;
        L(1, 0) = cde(0);                          // 下三角非对角元素
        L(1, 1) = rtd(1) * rtd(1) + DBL_EPSILON;
        L(1, 2) = 0.0;
        L(2, 0) = cde(2);
        L(2, 1) = cde(1);
        L(2, 2) = rtd(2) * rtd(2) + DBL_EPSILON;

        // 计算约束相关量
        const Eigen::MatrixX3d AL = A * L;                    // A * L, 每行代表一个约束方向变换后的向量
        const Eigen::VectorXd normAL = AL.rowwise().norm();   // ||A_i * L||_2，每个约束的范数
        const Eigen::Matrix3Xd adjNormAL = (AL.array().colwise() / normAL.array()).transpose(); // 归一化后的转置
        const Eigen::VectorXd consViola = (normAL + A * p).array() - 1.0; // 约束违反量: ||A_i*L|| + A_i*p - 1

        // 累加所有约束的罚函数项及其梯度
        double c, dc;
        Eigen::Vector3d vec;
        for (int i = 0; i < M; ++i)
        {
            // 对每个约束使用平滑L1罚函数
            if (smoothedL1(smoothEps, consViola(i), c, dc))
            {
                cost += c;                                     // 累加代价
                vec = dc * A.row(i).transpose();               // dc * A_i^T
                gdp += vec;                                    // 对p的梯度累加
                gdrtd += adjNormAL.col(i).cwiseProduct(vec);   // 对rtd的梯度累加（逐元素乘法）
                // 对cde的梯度累加（根据L的结构）
                gdcde(0) += adjNormAL(0, i) * vec(1);
                gdcde(1) += adjNormAL(1, i) * vec(2);
                gdcde(2) += adjNormAL(0, i) * vec(2);
            }
        }
        // 应用罚函数权重
        cost *= penaltyWt;
        gdp *= penaltyWt;
        gdrtd *= penaltyWt;
        gdcde *= penaltyWt;

        // 减去体积最大化项（最大化体积 = 最小化负对数体积）
        // det(L) = L(0,0) * L(1,1) * L(2,2)（对角元素乘积）
        // -log(det(L)) = -(log(L(0,0)) + log(L(1,1)) + log(L(2,2)))
        cost -= log(L(0, 0)) + log(L(1, 1)) + log(L(2, 2));

        // 体积项对对角元素的梯度: d(-log(L_ii))/dL_ii = -1/L_ii
        gdrtd(0) -= 1.0 / L(0, 0);
        gdrtd(1) -= 1.0 / L(1, 1);
        gdrtd(2) -= 1.0 / L(2, 2);

        // 链式法则：L_ii = rtd_i^2，所以 dL_ii/d(rtd_i) = 2*rtd_i
        gdrtd(0) *= 2.0 * rtd(0);
        gdrtd(1) *= 2.0 * rtd(1);
        gdrtd(2) *= 2.0 * rtd(2);

        return cost;
    }

    /**
     * @brief 计算给定半空间约束下的最大体积内切椭球(MVIE)
     *
     * @param hPoly 半空间约束矩阵（M×4），每行定义一个半空间约束：
     *              h0*x + h1*y + h2*z + h3 <= 0
     * @param R     输入/输出：椭球的旋转矩阵（初始值作为初始猜测，假设为旋转矩阵）
     * @param p     输入/输出：椭球的中心位置（初始值作为初始猜测）
     * @param r     输入/输出：椭球的三个主轴半径（初始值作为初始猜测）
     * @return 优化成功返回true，失败返回false
     *
     * @note 算法流程：
     *       1. 通过线性规划找到多面体的最深内部点
     *       2. 将约束转换为相对于内部点的标准形式
     *       3. 使用L-BFGS优化求解最大体积内切椭球
     *       4. 通过SVD分解恢复椭球的旋转矩阵和主轴半径
     */
    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    // R, p, r are ALWAYS taken as the initial guess
    // R is also assumed to be a rotation matrix
    inline bool maxVolInsEllipsoid(const Eigen::MatrixX4d &hPoly,
                                   Eigen::Matrix3d &R,
                                   Eigen::Vector3d &p,
                                   Eigen::Vector3d &r)
    {
        // ========== 第一步：寻找多面体的最深内部点 ==========
        // 使用线性规划求解：max depth, s.t. A*[x; depth] <= b
        const int M = hPoly.rows();                                      // 约束数量
        Eigen::MatrixX4d Alp(M, 4);                                      // 线性规划约束矩阵
        Eigen::VectorXd blp(M);                                          // 线性规划约束右端
        Eigen::Vector4d clp, xlp;                                        // 目标函数系数和解
        const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm(); // 计算每个约束法向量的范数

        // 归一化约束：将每个半空间约束的法向量归一化
        Alp.leftCols<3>() = hPoly.leftCols<3>().array().colwise() / hNorm;
        Alp.rightCols<1>().setConstant(1.0);                             // 深度变量的系数
        blp = -hPoly.rightCols<1>().array() / hNorm;                     // 归一化后的常数项

        // 目标函数：最大化深度（即最小化 -depth）
        clp.setZero();
        clp(3) = -1.0;                                                   // min -depth

        // 求解线性规划问题
        const double maxdepth = -sdlp::linprog<4>(clp, Alp, blp, xlp);
        if (!(maxdepth > 0.0) || std::isinf(maxdepth))
        {
            return false;  // 如果深度<=0或无穷大，说明多面体为空或无界
        }
        const Eigen::Vector3d interior = xlp.head<3>();                  // 提取最深内部点的坐标

        // ========== 第二步：准备MVIE优化的数据 ==========
        // 分配内存存储优化参数：M(int64) + smoothEps(double) + penaltyWt(double) + A矩阵(M*3个double)
        uint8_t *optData = new uint8_t[sizeof(int64_t) + (2 + 3 * M) * sizeof(double)];
        int64_t *pM = (int64_t *)optData;
        double *pSmoothEps = (double *)(pM + 1);
        double *pPenaltyWt = pSmoothEps + 1;
        double *pA = pPenaltyWt + 1;

        *pM = M;
        Eigen::Map<Eigen::MatrixX3d> A(pA, M, 3);
        // 将约束转换为相对于内部点的标准形式
        // 原约束: h*x + d <= 0, 转换为: h*(L*y + interior) + d <= 0
        // 整理为: ||h*L|| + h*interior + d/||h|| <= 1 的形式
        A = Alp.leftCols<3>().array().colwise() /
            (blp - Alp.leftCols<3>() * interior).array();

        // 初始化优化变量
        Eigen::VectorXd x(9);  // [p(3), rtd(3), cde(3)]，共9个变量

        // 从初始猜测的R和r构造椭球矩阵Q，并计算其Cholesky分解
        const Eigen::Matrix3d Q = R * (r.cwiseProduct(r)).asDiagonal() * R.transpose();
        Eigen::Matrix3d L;
        chol3d(Q, L);  // Q = L * L^T

        // 设置初始优化变量（相对于内部点）
        x.head<3>() = p - interior;         // 椭球中心相对于内部点的位置
        x(3) = sqrt(L(0, 0));               // rtd: 对角元素的平方根
        x(4) = sqrt(L(1, 1));
        x(5) = sqrt(L(2, 2));
        x(6) = L(1, 0);                     // cde: 非对角元素
        x(7) = L(2, 1);
        x(8) = L(2, 0);

        // ========== 第三步：使用L-BFGS优化求解MVIE ==========
        double minCost;
        lbfgs::lbfgs_parameter_t paramsMVIE;
        paramsMVIE.mem_size = 18;           // L-BFGS历史记忆大小
        paramsMVIE.g_epsilon = 0.0;         // 梯度收敛阈值（0表示不使用）
        paramsMVIE.min_step = 1.0e-32;      // 最小步长
        paramsMVIE.past = 3;                // 用于检查收敛的历史迭代次数
        paramsMVIE.delta = 1.0e-7;          // 相对代价变化的收敛阈值
        *pSmoothEps = 1.0e-2;               // 平滑L1函数的epsilon参数
        *pPenaltyWt = 1.0e+3;               // 罚函数权重

        // 执行L-BFGS优化
        int ret = lbfgs::lbfgs_optimize(x,
                                        minCost,
                                        &costMVIE,        // 代价函数
                                        nullptr,          // 无进度回调
                                        nullptr,          // 无自定义参数
                                        optData,          // 优化数据
                                        paramsMVIE);      // 优化参数

        if (ret < 0)
        {
            printf("FIRI WARNING: %s\n", lbfgs::lbfgs_strerror(ret));
        }

        // ========== 第四步：从优化结果恢复椭球参数 ==========
        // 将椭球中心转换回全局坐标系
        p = x.head<3>() + interior;

        // 重构Cholesky因子L
        L(0, 0) = x(3) * x(3);
        L(0, 1) = 0.0;
        L(0, 2) = 0.0;
        L(1, 0) = x(6);
        L(1, 1) = x(4) * x(4);
        L(1, 2) = 0.0;
        L(2, 0) = x(8);
        L(2, 1) = x(7);
        L(2, 2) = x(5) * x(5);

        // 使用SVD分解L得到椭球的旋转矩阵和主轴半径
        // L = U * S * V^T，其中U是正交矩阵，S是奇异值（主轴半径）
        Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::FullPivHouseholderQRPreconditioner> svd(L, Eigen::ComputeFullU);
        const Eigen::Matrix3d U = svd.matrixU();    // 左奇异向量矩阵
        const Eigen::Vector3d S = svd.singularValues(); // 奇异值（主轴半径）

        // 确保U是旋转矩阵（行列式为+1）
        if (U.determinant() < 0.0)
        {
            // 如果行列式为负，交换前两列以得到旋转矩阵
            R.col(0) = U.col(1);
            R.col(1) = U.col(0);
            R.col(2) = U.col(2);
            r(0) = S(1);
            r(1) = S(0);
            r(2) = S(2);
        }
        else
        {
            R = U;
            r = S;
        }

        delete[] optData;  // 释放优化数据内存

        return ret >= 0;   // 返回优化是否成功
    }

    /**
     * @brief FIRI (Fast Inscribed Radius Inflation) 主算法
     *        在给定边界约束和障碍物点云之间，构造一个连接起点和终点的无碰撞凸多面体走廊
     *
     * @param bd         边界约束（M×4矩阵），每行定义一个半空间约束：h0*x + h1*y + h2*z + h3 <= 0
     * @param pc         障碍物点云（3×N矩阵），每列是一个3D障碍物点
     * @param a          起点位置（3D向量）
     * @param b          终点位置（3D向量）
     * @param hPoly      输出的凸多面体走廊（半空间表示）
     * @param iterations 迭代次数（默认4次），每次迭代都会膨胀内切椭球以扩大走廊
     * @param epsilon    数值容差（默认1e-6）
     * @return 成功返回true，失败返回false
     *
     * @note 算法原理：
     *       1. 验证起点和终点都在边界约束内
     *       2. 迭代过程：
     *          a) 将空间映射到椭球坐标系（椭球变为单位球）
     *          b) 计算边界约束和障碍物点的切平面/支撑平面
     *          c) 逐步选择距离最近的约束，构造最小的包络凸多面体
     *          d) 在新的凸多面体内计算最大体积内切椭球（MVIE）
     *          e) 用新椭球更新坐标变换，重复迭代
     *       3. 最终得到的hPoly定义了一个无碰撞的凸多面体走廊
     */
    inline bool firi(const Eigen::MatrixX4d &bd,
                     const Eigen::Matrix3Xd &pc,
                     const Eigen::Vector3d &a,
                     const Eigen::Vector3d &b,
                     Eigen::MatrixX4d &hPoly,
                     const int iterations = 4,
                     const double epsilon = 1.0e-6)
    {
        // 将起点和终点转换为齐次坐标
        const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
        const Eigen::Vector4d bh(b(0), b(1), b(2), 1.0);

        // 验证起点和终点是否在边界约束内
        // 对于每个约束 bd[i] * point <= 0
        if ((bd * ah).maxCoeff() > 0.0 ||
            (bd * bh).maxCoeff() > 0.0)
        {
            return false;  // 起点或终点在边界外，无法构造走廊
        }

        const int M = bd.rows();   // 边界约束数量
        const int N = pc.cols();   // 障碍物点数量

        // 初始化椭球参数
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // 旋转矩阵（初始为单位矩阵）
        Eigen::Vector3d p = 0.5 * (a + b);                // 椭球中心（初始为起点和终点的中点）
        Eigen::Vector3d r = Eigen::Vector3d::Ones();      // 椭球主轴半径（初始为单位球）
        Eigen::MatrixX4d forwardH(M + N, 4);              // 变换后的半空间约束
        int nH = 0;                                        // 有效约束数量

        // ========== 迭代优化过程 ==========
        for (int loop = 0; loop < iterations; ++loop)
        {
            // 定义椭球坐标变换
            // forward: 将原始空间映射到椭球坐标系（椭球 -> 单位球）
            // backward: 将椭球坐标系映射回原始空间（单位球 -> 椭球）
            const Eigen::Matrix3d forward = r.cwiseInverse().asDiagonal() * R.transpose();
            const Eigen::Matrix3d backward = R * r.asDiagonal();

            // 将边界约束变换到椭球坐标系
            const Eigen::MatrixX3d forwardB = bd.leftCols<3>() * backward;  // 法向量变换
            const Eigen::VectorXd forwardD = bd.rightCols<1>() + bd.leftCols<3>() * p; // 常数项调整

            // 将障碍物点云变换到椭球坐标系（相对于椭球中心）
            const Eigen::Matrix3Xd forwardPC = forward * (pc.colwise() - p);

            // 变换起点和终点
            const Eigen::Vector3d fwd_a = forward * (a - p);
            const Eigen::Vector3d fwd_b = forward * (b - p);

            // 计算边界约束到椭球的距离（在变换空间中）
            const Eigen::VectorXd distDs = forwardD.cwiseAbs().cwiseQuotient(forwardB.rowwise().norm());

            // 为每个障碍物点计算切平面
            Eigen::MatrixX4d tangents(N, 4);  // 切平面参数 [法向量(3), 常数项(1)]
            Eigen::VectorXd distRs(N);         // 障碍物点到椭球中心的距离

            for (int i = 0; i < N; i++)
            {
                // 初始切平面：从椭球中心指向障碍物点的方向
                distRs(i) = forwardPC.col(i).norm();
                tangents(i, 3) = -distRs(i);  // 切平面常数项
                tangents.block<1, 3>(i, 0) = forwardPC.col(i).transpose() / distRs(i); // 归一化法向量

                // 检查切平面是否违反起点约束
                if (tangents.block<1, 3>(i, 0).dot(fwd_a) + tangents(i, 3) > epsilon)
                {
                    // 调整切平面：投影fwd_a到垂直于(pc[i] - fwd_a)的平面上
                    const Eigen::Vector3d delta = forwardPC.col(i) - fwd_a;
                    tangents.block<1, 3>(i, 0) = fwd_a - (delta.dot(fwd_a) / delta.squaredNorm()) * delta;
                    distRs(i) = tangents.block<1, 3>(i, 0).norm();
                    tangents(i, 3) = -distRs(i);
                    tangents.block<1, 3>(i, 0) /= distRs(i);
                }

                // 检查切平面是否违反终点约束
                if (tangents.block<1, 3>(i, 0).dot(fwd_b) + tangents(i, 3) > epsilon)
                {
                    // 调整切平面：投影fwd_b到垂直于(pc[i] - fwd_b)的平面上
                    const Eigen::Vector3d delta = forwardPC.col(i) - fwd_b;
                    tangents.block<1, 3>(i, 0) = fwd_b - (delta.dot(fwd_b) / delta.squaredNorm()) * delta;
                    distRs(i) = tangents.block<1, 3>(i, 0).norm();
                    tangents(i, 3) = -distRs(i);
                    tangents.block<1, 3>(i, 0) /= distRs(i);
                }

                // 如果仍然违反起点约束，使用叉积构造切平面
                if (tangents.block<1, 3>(i, 0).dot(fwd_a) + tangents(i, 3) > epsilon)
                {
                    // 切平面法向量 = (fwd_a - pc[i]) × (fwd_b - pc[i])
                    tangents.block<1, 3>(i, 0) = (fwd_a - forwardPC.col(i)).cross(fwd_b - forwardPC.col(i)).normalized();
                    tangents(i, 3) = -tangents.block<1, 3>(i, 0).dot(fwd_a);
                    // 确保切平面的正确方向（法向量指向外侧）
                    tangents.row(i) *= tangents(i, 3) > 0.0 ? -1.0 : 1.0;
                }
            }

            // 标记哪些约束尚未被选择
            Eigen::Matrix<uint8_t, -1, 1> bdFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(M, 1);
            Eigen::Matrix<uint8_t, -1, 1> pcFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(N, 1);

            nH = 0;  // 重置有效约束计数

            // ========== 贪婪选择约束构造凸多面体 ==========
            bool completed = false;
            int bdMinId = 0, pcMinId = 0;
            double minSqrD = distDs.minCoeff(&bdMinId);  // 找到距离最近的边界约束
            double minSqrR = INFINITY;
            if (distRs.size() != 0)
            {
                minSqrR = distRs.minCoeff(&pcMinId);      // 找到距离最近的障碍物点
            }

            // 逐步选择最近的约束，直到所有障碍物点都被排除
            for (int i = 0; !completed && i < (M + N); ++i)
            {
                // 选择距离更近的约束（边界或障碍物）
                if (minSqrD < minSqrR)
                {
                    // 选择边界约束
                    forwardH.block<1, 3>(nH, 0) = forwardB.row(bdMinId);
                    forwardH(nH, 3) = forwardD(bdMinId);
                    bdFlags(bdMinId) = 0;  // 标记为已选择
                }
                else
                {
                    // 选择障碍物切平面
                    forwardH.row(nH) = tangents.row(pcMinId);
                    pcFlags(pcMinId) = 0;  // 标记为已选择
                }

                // 更新未选择的约束，寻找下一个最近的
                completed = true;
                minSqrD = INFINITY;
                for (int j = 0; j < M; ++j)
                {
                    if (bdFlags(j))
                    {
                        completed = false;
                        if (minSqrD > distDs(j))
                        {
                            bdMinId = j;
                            minSqrD = distDs(j);
                        }
                    }
                }

                // 检查障碍物点是否被当前半空间排除
                minSqrR = INFINITY;
                for (int j = 0; j < N; ++j)
                {
                    if (pcFlags(j))
                    {
                        // 如果障碍物点在当前半空间外侧，则标记为已排除
                        if (forwardH.block<1, 3>(nH, 0).dot(forwardPC.col(j)) + forwardH(nH, 3) > -epsilon)
                        {
                            pcFlags(j) = 0;
                        }
                        else
                        {
                            completed = false;
                            if (minSqrR > distRs(j))
                            {
                                pcMinId = j;
                                minSqrR = distRs(j);
                            }
                        }
                    }
                }
                ++nH;  // 增加有效约束数量
            }

            // 将变换后的半空间约束转换回原始坐标系
            hPoly.resize(nH, 4);
            for (int i = 0; i < nH; ++i)
            {
                hPoly.block<1, 3>(i, 0) = forwardH.block<1, 3>(i, 0) * forward;  // 逆变换法向量
                hPoly(i, 3) = forwardH(i, 3) - hPoly.block<1, 3>(i, 0).dot(p);   // 调整常数项
            }

            // 如果是最后一次迭代，直接退出
            if (loop == iterations - 1)
            {
                break;
            }

            // 计算新的最大体积内切椭球，为下一次迭代做准备
            maxVolInsEllipsoid(hPoly, R, p, r);
        }

        return true;
    }

}

#endif
