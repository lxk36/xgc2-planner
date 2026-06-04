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

#ifndef GCOPTER_HPP
#define GCOPTER_HPP

#include "gcopter/minco.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/lbfgs.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>

namespace gcopter
{
    // GCOPTER多面体安全飞行走廊优化器类
    // 实现基于MINCO轨迹表示的时空联合优化，考虑动力学约束和多面体走廊约束
    class GCOPTER_PolytopeSFC
    {
    public:
        // 类型定义
        typedef Eigen::Matrix3Xd PolyhedronV;  // V表示形式的多面体(顶点表示)，3xN矩阵，每列为一个顶点
        typedef Eigen::MatrixX4d PolyhedronH;  // H表示形式的多面体(半空间表示)，Nx4矩阵，每行为[normal, offset]
        typedef std::vector<PolyhedronV> PolyhedraV;  // 多个V表示多面体的集合
        typedef std::vector<PolyhedronH> PolyhedraH;  // 多个H表示多面体的集合

    private:
        // 核心优化模块
        minco::MINCO_S3NU minco;        // MINCO轨迹生成器(Snap连续)
        flatness::FlatnessMap flatmap;  // 微分平坦映射，用于从轨迹到姿态、推力的转换

        // 优化参数
        double rho;                      // 时间权重，控制时间最优性的重要程度
        Eigen::Matrix3d headPVA;         // 起点的位置-速度-加速度(Position-Velocity-Acceleration)
        Eigen::Matrix3d tailPVA;         // 终点的位置-速度-加速度

        // 走廊约束
        PolyhedraV vPolytopes;           // V表示的多面体集合(顶点表示)
        PolyhedraH hPolytopes;           // H表示的多面体集合(半空间表示)
        Eigen::Matrix3Xd shortPath;      // 走廊中的最短路径

        // 索引映射
        Eigen::VectorXi pieceIdx;        // 每个多面体对应的轨迹段数
        Eigen::VectorXi vPolyIdx;        // 每个中间点对应的V多面体索引
        Eigen::VectorXi hPolyIdx;        // 每个轨迹段对应的H多面体索引

        // 维度信息
        int polyN;                       // 多面体数量
        int pieceN;                      // 轨迹段数量

        int spatialDim;                  // 空间优化变量维度
        int temporalDim;                 // 时间优化变量维度

        // 惩罚函数参数
        double smoothEps;                // 平滑L1惩罚的平滑因子
        int integralRes;                 // 积分分辨率(每段的采样点数)
        Eigen::VectorXd magnitudeBd;     // 幅值边界 [v_max, omg_max, theta_max, thrust_min, thrust_max]
        Eigen::VectorXd penaltyWt;       // 惩罚权重 [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]
        Eigen::VectorXd physicalPm;      // 物理参数 [mass, gravity, h_drag, v_drag, p_drag, smooth_factor]
        double allocSpeed;               // 时间分配速度(用于初始化)

        // L-BFGS求解器参数
        lbfgs::lbfgs_parameter_t lbfgs_params;

        // 临时变量(避免重复分配内存)
        Eigen::Matrix3Xd points;         // 中间路径点
        Eigen::VectorXd times;           // 每段时间分配
        Eigen::Matrix3Xd gradByPoints;   // 代价对路径点的梯度
        Eigen::VectorXd gradByTimes;     // 代价对时间的梯度
        Eigen::MatrixX3d partialGradByCoeffs;   // 代价对MINCO系数的偏梯度
        Eigen::VectorXd partialGradByTimes;     // 代价对时间的偏梯度

    private:
        // 时间变换的前向映射: tau -> T
        // 使用软加映射保证T始终为正，避免优化过程中出现负时间
        // tau > 0: T = 0.5*tau^2 + tau + 1 (二次增长)
        // tau < 0: T = 1 / (0.5*tau^2 - tau + 1) (渐近趋于0)
        static inline void forwardT(const Eigen::VectorXd &tau,
                                    Eigen::VectorXd &T)
        {
            const int sizeTau = tau.size();
            T.resize(sizeTau);
            for (int i = 0; i < sizeTau; i++)
            {
                T(i) = tau(i) > 0.0
                           ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                           : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
            }
            return;
        }

        // 时间变换的反向映射: T -> tau
        // 是forwardT的逆运算，用于初始化优化变量
        // T > 1: tau = sqrt(2T - 1) - 1
        // T < 1: tau = 1 - sqrt(2/T - 1)
        template <typename EIGENVEC>
        static inline void backwardT(const Eigen::VectorXd &T,
                                     EIGENVEC &tau)
        {
            const int sizeT = T.size();
            tau.resize(sizeT);
            for (int i = 0; i < sizeT; i++)
            {
                tau(i) = T(i) > 1.0
                             ? (sqrt(2.0 * T(i) - 1.0) - 1.0)
                             : (1.0 - sqrt(2.0 / T(i) - 1.0));
            }

            return;
        }

        // 时间变换梯度的反向传播: 链式法则计算 dL/dtau = dL/dT * dT/dtau
        // tau > 0: dT/dtau = tau + 1
        // tau < 0: dT/dtau = (1 - tau) / T^2
        template <typename EIGENVEC>
        static inline void backwardGradT(const Eigen::VectorXd &tau,
                                         const Eigen::VectorXd &gradT,
                                         EIGENVEC &gradTau)
        {
            const int sizeTau = tau.size();
            gradTau.resize(sizeTau);
            double denSqrt;
            for (int i = 0; i < sizeTau; i++)
            {
                if (tau(i) > 0)
                {
                    // 正区间: dT/dtau = tau + 1
                    gradTau(i) = gradT(i) * (tau(i) + 1.0);
                }
                else
                {
                    // 负区间: dT/dtau = (1 - tau) / T^2
                    denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                    gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
                }
            }

            return;
        }

        // 空间点的前向映射: xi -> P
        // 将无约束优化变量xi映射到多面体内的点P
        // 使用原点偏移基(Origin-Offset Basis)表示: P = base + sum(offset_i * q_i^2)
        // 其中q是归一化后的xi向量，平方确保凸组合，归一化确保数值稳定性
        static inline void forwardP(const Eigen::VectorXd &xi,
                                    const Eigen::VectorXi &vIdx,
                                    const PolyhedraV &vPolys,
                                    Eigen::Matrix3Xd &P)
        {
            const int sizeP = vIdx.size();
            P.resize(3, sizeP);
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);                         // 获取多面体索引
                k = vPolys[l].cols();                // 多面体顶点数
                q = xi.segment(j, k).normalized().head(k - 1);  // 归一化并取前k-1个分量
                // P = 基点 + 偏移向量的加权和(权重为q的平方)
                P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) +
                           vPolys[l].col(0);
            }
            return;
        }

        // 微型非线性最小二乘问题的代价函数
        // 用于backwardP中求解最接近给定点的多面体内点
        // 目标: 最小化 ||P - target||^2，约束: P在多面体内
        // 同时惩罚归一化约束的违反: ||xi||^2 = 1
        static inline double costTinyNLS(void *ptr,
                                         const Eigen::VectorXd &xi,
                                         Eigen::VectorXd &gradXi)
        {
            const int n = xi.size();
            const Eigen::Matrix3Xd &ovPoly = *(Eigen::Matrix3Xd *)ptr;  // [target, base, offsets...]

            // 计算当前点与目标点的差
            const double sqrNormXi = xi.squaredNorm();
            const double invNormXi = 1.0 / sqrt(sqrNormXi);
            const Eigen::VectorXd unitXi = xi * invNormXi;  // 归一化xi
            const Eigen::VectorXd r = unitXi.head(n - 1);
            const Eigen::Vector3d delta = ovPoly.rightCols(n - 1) * r.cwiseProduct(r) +
                                          ovPoly.col(1) - ovPoly.col(0);  // 映射点 - 目标点

            // 主代价: ||delta||^2
            double cost = delta.squaredNorm();
            // 梯度计算(链式法则)
            gradXi.head(n - 1) = (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() *
                                 r.array() * 2.0;
            gradXi(n - 1) = 0.0;
            gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;  // 投影到单位球切空间

            // 添加归一化惩罚: ||xi||^2 = 1
            const double sqrNormViolation = sqrNormXi - 1.0;
            if (sqrNormViolation > 0.0)
            {
                double c = sqrNormViolation * sqrNormViolation;
                const double dc = 3.0 * c;
                c *= sqrNormViolation;  // 三次惩罚
                cost += c;
                gradXi += dc * 2.0 * xi;
            }

            return cost;
        }

        // 空间点的反向映射: P -> xi
        // 给定多面体内的点P，求解对应的优化变量xi
        // 这是forwardP的逆运算，通过求解优化问题实现
        template <typename EIGENVEC>
        static inline void backwardP(const Eigen::Matrix3Xd &P,
                                     const Eigen::VectorXi &vIdx,
                                     const PolyhedraV &vPolys,
                                     EIGENVEC &xi)
        {
            const int sizeP = P.cols();

            double minSqrD;
            // 配置微型NLS求解器参数
            lbfgs::lbfgs_parameter_t tiny_nls_params;
            tiny_nls_params.past = 0;
            tiny_nls_params.delta = 1.0e-5;
            tiny_nls_params.g_epsilon = FLT_EPSILON;
            tiny_nls_params.max_iterations = 128;

            Eigen::Matrix3Xd ovPoly;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();

                // 构造优化问题数据: [target_point, base, offsets...]
                ovPoly.resize(3, k + 1);
                ovPoly.col(0) = P.col(i);           // 目标点
                ovPoly.rightCols(k) = vPolys[l];    // 多面体顶点

                // 初始化为均匀分布(在单位球上)
                Eigen::VectorXd x(k);
                x.setConstant(sqrt(1.0 / k));

                // 求解最接近P的多面体内点对应的xi
                lbfgs::lbfgs_optimize(x,
                                      minSqrD,
                                      &GCOPTER_PolytopeSFC::costTinyNLS,
                                      nullptr,
                                      nullptr,
                                      &ovPoly,
                                      tiny_nls_params);

                xi.segment(j, k) = x;
            }

            return;
        }

        // 空间点梯度的反向传播: dL/dxi = dL/dP * dP/dxi
        // 使用链式法则计算梯度，考虑归一化操作的导数
        template <typename EIGENVEC>
        static inline void backwardGradP(const Eigen::VectorXd &xi,
                                         const Eigen::VectorXi &vIdx,
                                         const PolyhedraV &vPolys,
                                         const Eigen::Matrix3Xd &gradP,
                                         EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double normInv;
            Eigen::VectorXd q, gradQ, unitQ;
            for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k)
            {
                l = vIdx(i);
                k = vPolys[l].cols();
                q = xi.segment(j, k);
                normInv = 1.0 / q.norm();
                unitQ = q * normInv;  // 单位向量
                gradQ.resize(k);
                // 计算对q的梯度: dP/dq = offsets^T * diag(2*q)
                gradQ.head(k - 1) = (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                // 投影到正交空间: grad_xi = (grad_q - (q^T grad_q)q) / ||q||
                gradXi.segment(j, k) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
            }

            return;
        }

        // 范数限制层: 强制xi的范数为1
        // 当||xi||^2 > 1时添加三次惩罚: cost += (||xi||^2 - 1)^3
        // 这确保了优化过程中xi保持在单位球面附近，避免数值问题
        template <typename EIGENVEC>
        static inline void normRetrictionLayer(const Eigen::VectorXd &xi,
                                               const Eigen::VectorXi &vIdx,
                                               const PolyhedraV &vPolys,
                                               double &cost,
                                               EIGENVEC &gradXi)
        {
            const int sizeP = vIdx.size();
            gradXi.resize(xi.size());

            double sqrNormQ, sqrNormViolation, c, dc;
            Eigen::VectorXd q;
            for (int i = 0, j = 0, k; i < sizeP; i++, j += k)
            {
                k = vPolys[vIdx(i)].cols();  // 当前多面体的顶点数

                q = xi.segment(j, k);         // 提取对应的xi分段
                sqrNormQ = q.squaredNorm();   // 计算平方范数
                sqrNormViolation = sqrNormQ - 1.0;  // 违反量
                if (sqrNormViolation > 0.0)
                {
                    // 三次惩罚: c = (violation)^3
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;  // 导数: dc/dviolation = 3*violation^2
                    c *= sqrNormViolation;
                    cost += c;
                    // 链式法则: dc/dxi = dc/dviolation * dviolation/d(||xi||^2) * d(||xi||^2)/dxi
                    gradXi.segment(j, k) += dc * 2.0 * q;
                }
            }

            return;
        }

        // 平滑L1惩罚函数
        // 用于处理约束违反，提供可微的惩罚
        // x < 0: 无惩罚
        // 0 <= x <= mu: 平滑过渡区 f(x) = (mu - 0.5*x) * (x/mu)^3
        // x > mu: 线性增长区 f(x) = x - 0.5*mu
        // 参数:
        //   x: 约束违反量
        //   mu: 平滑因子(过渡区宽度)
        //   f: 输出惩罚值
        //   df: 输出惩罚对x的导数
        // 返回: true表示有惩罚，false表示无惩罚
        static inline bool smoothedL1(const double &x,
                                      const double &mu,
                                      double &f,
                                      double &df)
        {
            if (x < 0.0)
            {
                // 无违反，无惩罚
                return false;
            }
            else if (x > mu)
            {
                // 线性增长区
                f = x - 0.5 * mu;
                df = 1.0;
                return true;
            }
            else
            {
                // 平滑过渡区 [0, mu]
                const double xdmu = x / mu;          // 归一化x
                const double sqrxdmu = xdmu * xdmu;  // x^2/mu^2
                const double mumxd2 = mu - 0.5 * x;  // mu - x/2
                f = mumxd2 * sqrxdmu * xdmu;         // (mu - x/2) * (x/mu)^3
                // 导数计算
                df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
                return true;
            }
        }

        // 添加惩罚函数项
        // 通过数值积分计算轨迹的约束违反惩罚，包括:
        // 1. 位置约束: 不能超出多面体走廊
        // 2. 速度约束: ||v|| <= v_max
        // 3. 角速度约束: ||omega|| <= omg_max
        // 4. 倾角约束: theta <= theta_max
        // 5. 推力约束: thrust_min <= thrust <= thrust_max
        //
        // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
        //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
        static inline void attachPenaltyFunctional(const Eigen::VectorXd &T,
                                                   const Eigen::MatrixX3d &coeffs,
                                                   const Eigen::VectorXi &hIdx,
                                                   const PolyhedraH &hPolys,
                                                   const double &smoothFactor,
                                                   const int &integralResolution,
                                                   const Eigen::VectorXd &magnitudeBounds,
                                                   const Eigen::VectorXd &penaltyWeights,
                                                   flatness::FlatnessMap &flatMap,
                                                   double &cost,
                                                   Eigen::VectorXd &gradT,
                                                   Eigen::MatrixX3d &gradC)
        {
            // 提取约束边界
            const double velSqrMax = magnitudeBounds(0) * magnitudeBounds(0);  // 最大速度平方
            const double omgSqrMax = magnitudeBounds(1) * magnitudeBounds(1);  // 最大角速度平方
            const double thetaMax = magnitudeBounds(2);                        // 最大倾角
            const double thrustMean = 0.5 * (magnitudeBounds(3) + magnitudeBounds(4));  // 推力中心值
            const double thrustRadi = 0.5 * fabs(magnitudeBounds(4) - magnitudeBounds(3));  // 推力半径
            const double thrustSqrRadi = thrustRadi * thrustRadi;              // 推力半径平方

            // 提取惩罚权重
            const double weightPos = penaltyWeights(0);     // 位置惩罚权重
            const double weightVel = penaltyWeights(1);     // 速度惩罚权重
            const double weightOmg = penaltyWeights(2);     // 角速度惩罚权重
            const double weightTheta = penaltyWeights(3);   // 倾角惩罚权重
            const double weightThrust = penaltyWeights(4);  // 推力惩罚权重

            // 轨迹状态变量(位置、速度、加速度、jerk、snap)
            Eigen::Vector3d pos, vel, acc, jer, sna;
            // 总梯度(反向传播用)
            Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
            double totalGradPsi, totalGradPsiD;  // 偏航角及其导数的梯度
            // 微分平坦映射输出(推力、四元数、角速度)
            double thr, cos_theta;
            Eigen::Vector4d quat;
            Eigen::Vector3d omg;
            // 对平坦映射输出的梯度
            double gradThr;
            Eigen::Vector4d gradQuat;
            Eigen::Vector3d gradPos, gradVel, gradOmg;

            // 积分相关变量
            double step, alpha;                   // 积分步长和归一化时间
            double s1, s2, s3, s4, s5;            // 时间的幂次: t, t^2, t^3, t^4, t^5
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;  // 多项式基函数及其导数
            Eigen::Vector3d outerNormal;          // 多面体外法向量
            int K, L;                             // 多面体半空间数量和索引
            // 约束违反量
            double violaPos, violaVel, violaOmg, violaTheta, violaThrust;
            // 约束违反惩罚导数
            double violaPosPenaD, violaVelPenaD, violaOmgPenaD, violaThetaPenaD, violaThrustPenaD;
            // 约束违反惩罚值
            double violaPosPena, violaVelPena, violaOmgPena, violaThetaPena, violaThrustPena;
            double node, pena;  // 积分节点权重和总惩罚

            const int pieceNum = T.size();
            const double integralFrac = 1.0 / integralResolution;  // 积分分数步长
            // 遍历每段轨迹
            for (int i = 0; i < pieceNum; i++)
            {
                const Eigen::Matrix<double, 6, 3> &c = coeffs.block<6, 3>(i * 6, 0);  // 第i段的系数矩阵
                step = T(i) * integralFrac;  // 该段的实际时间步长
                // 数值积分: 使用梯形法则
                for (int j = 0; j <= integralResolution; j++)
                {
                    // 计算时间的各次幂
                    s1 = j * step;
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    // 构造多项式基函数: [1, t, t^2, t^3, t^4, t^5]^T
                    beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
                    // 一阶导数: [0, 1, 2t, 3t^2, 4t^3, 5t^4]^T
                    beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
                    // 二阶导数: [0, 0, 2, 6t, 12t^2, 20t^3]^T
                    beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3;
                    // 三阶导数(jerk): [0, 0, 0, 6, 24t, 60t^2]^T
                    beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2;
                    // 四阶导数(snap): [0, 0, 0, 0, 24, 120t]^T
                    beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1;
                    // 计算轨迹状态: p(t) = c^T * beta
                    pos = c.transpose() * beta0;
                    vel = c.transpose() * beta1;
                    acc = c.transpose() * beta2;
                    jer = c.transpose() * beta3;
                    sna = c.transpose() * beta4;

                    // 微分平坦映射: 从轨迹状态(vel, acc, jer)计算推力、姿态、角速度
                    flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);

                    // 计算各约束的违反量
                    violaVel = vel.squaredNorm() - velSqrMax;  // 速度约束违反
                    violaOmg = omg.squaredNorm() - omgSqrMax;  // 角速度约束违反
                    cos_theta = 1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2));  // 从四元数计算倾角余弦
                    violaTheta = acos(cos_theta) - thetaMax;   // 倾角约束违反
                    violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;  // 推力约束违反

                    // 初始化梯度累加器
                    gradThr = 0.0;
                    gradQuat.setZero();
                    gradPos.setZero(), gradVel.setZero(), gradOmg.setZero();
                    pena = 0.0;  // 当前积分点的总惩罚

                    // 检查位置是否在安全走廊内
                    L = hIdx(i);            // 当前段对应的多面体索引
                    K = hPolys[L].rows();   // 该多面体的半空间数量
                    for (int k = 0; k < K; k++)
                    {
                        outerNormal = hPolys[L].block<1, 3>(k, 0);  // 第k个半空间的外法向量
                        // 违反量 = n^T * p + d，大于0表示在半空间外侧(违反)
                        violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
                        if (smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD))
                        {
                            // 累加位置惩罚梯度
                            gradPos += weightPos * violaPosPenaD * outerNormal;
                            pena += weightPos * violaPosPena;
                        }
                    }

                    // 检查速度约束
                    if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD))
                    {
                        // d(||v||^2)/dv = 2v
                        gradVel += weightVel * violaVelPenaD * 2.0 * vel;
                        pena += weightVel * violaVelPena;
                    }

                    // 检查角速度约束
                    if (smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD))
                    {
                        // d(||omg||^2)/domg = 2*omg
                        gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
                        pena += weightOmg * violaOmgPena;
                    }

                    // 检查倾角约束
                    if (smoothedL1(violaTheta, smoothFactor, violaThetaPena, violaThetaPenaD))
                    {
                        // d(acos(cos_theta))/dquat，使用链式法则
                        // d(acos(x))/dx = -1/sqrt(1-x^2)
                        // d(cos_theta)/dquat = -4*[0, qx, qy, 0]^T
                        gradQuat += weightTheta * violaThetaPenaD /
                                    sqrt(1.0 - cos_theta * cos_theta) * 4.0 *
                                    Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
                        pena += weightTheta * violaThetaPena;
                    }

                    // 检查推力约束
                    if (smoothedL1(violaThrust, smoothFactor, violaThrustPena, violaThrustPenaD))
                    {
                        // d((thr-mean)^2)/dthr = 2*(thr-mean)
                        gradThr += weightThrust * violaThrustPenaD * 2.0 * (thr - thrustMean);
                        pena += weightThrust * violaThrustPena;
                    }

                    // 微分平坦映射的反向传播: 从(pos,vel,thr,quat,omg)的梯度计算(pos,vel,acc,jer)的梯度
                    flatMap.backward(gradPos, gradVel, gradThr, gradQuat, gradOmg,
                                     totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                                     totalGradPsi, totalGradPsiD);

                    // 梯形积分权重: 首尾点权重0.5，中间点权重1.0
                    node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
                    alpha = j * integralFrac;  // 归一化时间 [0, 1]

                    // 累加对系数矩阵的梯度
                    gradC.block<6, 3>(i * 6, 0) += (beta0 * totalGradPos.transpose() +
                                                    beta1 * totalGradVel.transpose() +
                                                    beta2 * totalGradAcc.transpose() +
                                                    beta3 * totalGradJer.transpose()) *
                                                   node * step;

                    // 累加对时间的梯度(包含两部分)
                    // 1. 通过轨迹导数变化影响代价
                    gradT(i) += (totalGradPos.dot(vel) +
                                 totalGradVel.dot(acc) +
                                 totalGradAcc.dot(jer) +
                                 totalGradJer.dot(sna)) *
                                    alpha * node * step +
                                // 2. 积分区域变化影响代价
                                node * integralFrac * pena;

                    // 累加总代价
                    cost += node * step * pena;
                }
            }

            return;
        }

        // 总代价函数(L-BFGS优化器的回调函数)
        // 代价 = Snap能量 + 约束惩罚 + 时间惩罚
        // 参数:
        //   ptr: GCOPTER对象指针
        //   x: 优化变量 [tau; xi]，其中tau是时间变换变量，xi是空间变换变量
        //   g: 输出梯度
        // 返回: 总代价值
        static inline double costFunctional(void *ptr,
                                            const Eigen::VectorXd &x,
                                            Eigen::VectorXd &g)
        {
            GCOPTER_PolytopeSFC &obj = *(GCOPTER_PolytopeSFC *)ptr;
            const int dimTau = obj.temporalDim;     // 时间变量维度
            const int dimXi = obj.spatialDim;       // 空间变量维度
            const double weightT = obj.rho;         // 时间权重

            // 使用Eigen::Map进行零拷贝访问
            Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
            Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
            Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
            Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);

            // 前向传播: 将优化变量映射到物理空间
            forwardT(tau, obj.times);       // tau -> T (时间)
            forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points);  // xi -> P (路径点)

            // 计算Snap能量代价及其梯度
            double cost;
            obj.minco.setParameters(obj.points, obj.times);
            obj.minco.getEnergy(cost);
            obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
            obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);

            // 添加约束惩罚项
            attachPenaltyFunctional(obj.times, obj.minco.getCoeffs(),
                                    obj.hPolyIdx, obj.hPolytopes,
                                    obj.smoothEps, obj.integralRes,
                                    obj.magnitudeBd, obj.penaltyWt, obj.flatmap,
                                    cost, obj.partialGradByTimes, obj.partialGradByCoeffs);

            // 将对(coeffs, times)的梯度传播到(points, times)
            obj.minco.propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                    obj.gradByPoints, obj.gradByTimes);

            // 添加时间惩罚项
            cost += weightT * obj.times.sum();
            obj.gradByTimes.array() += weightT;

            // 反向传播: 将梯度映射回优化变量空间
            backwardGradT(tau, obj.gradByTimes, gradTau);
            backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi);

            // 添加范数限制层的惩罚
            normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);

            return cost;
        }

        // 距离代价函数(用于计算走廊内最短路径)
        // 目标: 最小化从起点到终点经过重叠区域的总距离
        // 约束: 中间点必须在对应的重叠多面体内
        // 参数:
        //   ptr: 数据指针数组 [&smoothD, &ini, &fin, &vPolys]
        //   xi: 优化变量，编码中间点的位置
        //   gradXi: 输出梯度
        // 返回: 总路径长度(平滑化)
        static inline double costDistance(void *ptr,
                                          const Eigen::VectorXd &xi,
                                          Eigen::VectorXd &gradXi)
        {
            void **dataPtrs = (void **)ptr;
            const double &dEps = *((const double *)(dataPtrs[0]));            // 平滑因子
            const Eigen::Vector3d &ini = *((const Eigen::Vector3d *)(dataPtrs[1]));  // 起点
            const Eigen::Vector3d &fin = *((const Eigen::Vector3d *)(dataPtrs[2]));  // 终点
            const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));        // V表示的多面体

            double cost = 0.0;
            const int overlaps = vPolys.size() / 2;  // 重叠区域数量

            Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
            Eigen::Vector3d a, b, d;
            Eigen::VectorXd r;
            double smoothedDistance;
            // 计算路径长度: ini -> p1 -> p2 -> ... -> fin
            for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k)
            {
                // 起点
                a = i == 0 ? ini : b;
                // 终点
                if (i < overlaps)
                {
                    // 从xi中解码第i个中间点
                    k = vPolys[2 * i + 1].cols();
                    Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                    r = q.normalized().head(k - 1);
                    b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                        vPolys[2 * i + 1].col(0);
                }
                else
                {
                    b = fin;
                }

                // 计算平滑距离: sqrt(||d||^2 + eps)，避免导数在0处不连续
                d = b - a;
                smoothedDistance = sqrt(d.squaredNorm() + dEps);
                cost += smoothedDistance;

                // 计算对点的梯度: d(sqrt(||d||^2 + eps))/dd = d / sqrt(||d||^2 + eps)
                if (i < overlaps)
                {
                    gradP.col(i) += d / smoothedDistance;
                }
                if (i > 0)
                {
                    gradP.col(i - 1) -= d / smoothedDistance;
                }
            }

            // 将对点P的梯度反向传播到xi，并添加范数限制惩罚
            Eigen::VectorXd unitQ;
            double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);

                // 计算对xi的梯度(通过链式法则)
                sqrNormQ = q.squaredNorm();
                invNormQ = 1.0 / sqrt(sqrNormQ);
                unitQ = q * invNormQ;
                gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                                    unitQ.head(k - 1).array() * 2.0;
                gradQ(k - 1) = 0.0;
                // 投影到单位球切空间
                gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

                // 添加范数限制惩罚
                sqrNormViolation = sqrNormQ - 1.0;
                if (sqrNormViolation > 0.0)
                {
                    c = sqrNormViolation * sqrNormViolation;
                    dc = 3.0 * c;
                    c *= sqrNormViolation;
                    cost += c;
                    gradQ += dc * 2.0 * q;
                }
            }

            return cost;
        }

        // 计算安全走廊内的最短路径
        // 从起点到终点，经过所有重叠区域的最短路径
        // 使用L-BFGS优化求解
        // 参数:
        //   ini: 起点
        //   fin: 终点
        //   vPolys: V表示的多面体序列(偶数索引为单个多面体，奇数索引为重叠区域)
        //   smoothD: 距离平滑因子
        //   path: 输出路径，包含起点、中间点、终点
        static inline void getShortestPath(const Eigen::Vector3d &ini,
                                           const Eigen::Vector3d &fin,
                                           const PolyhedraV &vPolys,
                                           const double &smoothD,
                                           Eigen::Matrix3Xd &path)
        {
            const int overlaps = vPolys.size() / 2;  // 重叠区域数量
            // 统计每个重叠区域的顶点数
            Eigen::VectorXi vSizes(overlaps);
            for (int i = 0; i < overlaps; i++)
            {
                vSizes(i) = vPolys[2 * i + 1].cols();
            }

            // 初始化优化变量: 均匀分布在单位球上
            Eigen::VectorXd xi(vSizes.sum());
            for (int i = 0, j = 0; i < overlaps; i++)
            {
                xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
                j += vSizes(i);
            }

            // 配置L-BFGS参数
            double minDistance;
            void *dataPtrs[4];
            dataPtrs[0] = (void *)(&smoothD);
            dataPtrs[1] = (void *)(&ini);
            dataPtrs[2] = (void *)(&fin);
            dataPtrs[3] = (void *)(&vPolys);
            lbfgs::lbfgs_parameter_t shortest_path_params;
            shortest_path_params.past = 3;
            shortest_path_params.delta = 1.0e-3;
            shortest_path_params.g_epsilon = 1.0e-5;

            // 优化求解最短路径
            lbfgs::lbfgs_optimize(xi,
                                  minDistance,
                                  &GCOPTER_PolytopeSFC::costDistance,
                                  nullptr,
                                  nullptr,
                                  dataPtrs,
                                  shortest_path_params);

            // 解码优化结果，构造路径
            path.resize(3, overlaps + 2);
            path.leftCols<1>() = ini;      // 起点
            path.rightCols<1>() = fin;     // 终点
            Eigen::VectorXd r;
            for (int i = 0, j = 0, k; i < overlaps; i++, j += k)
            {
                k = vPolys[2 * i + 1].cols();
                Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
                r = q.normalized().head(k - 1);
                // 中间点: 从xi解码到3D坐标
                path.col(i + 1) = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) +
                                  vPolys[2 * i + 1].col(0);
            }

            return;
        }

        // 处理安全走廊: 从H表示转换为V表示
        // 生成序列: [poly0, overlap01, poly1, overlap12, poly2, ..., polyN]
        // 其中overlap为相邻多面体的交集
        // 参数:
        //   hPs: H表示(半空间)的多面体序列
        //   vPs: 输出V表示(顶点)的多面体序列，长度为2*N+1
        // 返回: 成功返回true，失败(如多面体退化)返回false
        static inline bool processCorridor(const PolyhedraH &hPs,
                                           PolyhedraV &vPs)
        {
            const int sizeCorridor = hPs.size() - 1;  // 相邻多面体对数

            vPs.clear();
            vPs.reserve(2 * sizeCorridor + 1);

            int nv;
            PolyhedronH curIH;
            PolyhedronV curIV, curIOB;
            for (int i = 0; i < sizeCorridor; i++)
            {
                // 1. 处理第i个多面体
                if (!geo_utils::enumerateVs(hPs[i], curIV))
                {
                    return false;  // 顶点枚举失败
                }
                nv = curIV.cols();
                // 转换为原点偏移基(Origin-Offset Basis)表示
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);  // 基点(选择第一个顶点)
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);  // 偏移向量
                vPs.push_back(curIOB);

                // 2. 处理第i和i+1个多面体的重叠区域(交集)
                curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);
                curIH.topRows(hPs[i].rows()) = hPs[i];      // 合并半空间
                curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
                if (!geo_utils::enumerateVs(curIH, curIV))
                {
                    return false;
                }
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                vPs.push_back(curIOB);
            }

            // 3. 处理最后一个多面体
            if (!geo_utils::enumerateVs(hPs.back(), curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);

            return true;
        }

        // 设置初始轨迹
        // 根据最短路径和期望速度，初始化内部路径点和时间分配
        // 参数:
        //   path: 最短路径(包含起点和终点)
        //   speed: 期望速度
        //   intervalNs: 每段路径分成的小段数
        //   innerPoints: 输出内部路径点(不包括起点和终点)
        //   timeAlloc: 输出每小段的时间分配
        static inline void setInitial(const Eigen::Matrix3Xd &path,
                                      const double &speed,
                                      const Eigen::VectorXi &intervalNs,
                                      Eigen::Matrix3Xd &innerPoints,
                                      Eigen::VectorXd &timeAlloc)
        {
            const int sizeM = intervalNs.size();   // 大段数(路径点数-1)
            const int sizeN = intervalNs.sum();    // 总小段数
            innerPoints.resize(3, sizeN - 1);      // 内部点(不包括起点)
            timeAlloc.resize(sizeN);               // 每小段的时间

            Eigen::Vector3d a, b, c;
            for (int i = 0, j = 0, k = 0, l; i < sizeM; i++)
            {
                l = intervalNs(i);          // 第i大段分成l个小段
                a = path.col(i);            // 起点
                b = path.col(i + 1);        // 终点
                c = (b - a) / l;            // 每小段的位移向量
                // 时间分配: t = distance / speed
                timeAlloc.segment(j, l).setConstant(c.norm() / speed);
                j += l;
                // 生成内部点
                for (int m = 0; m < l; m++)
                {
                    if (i > 0 || m > 0)  // 跳过起点
                    {
                        innerPoints.col(k++) = a + c * m;
                    }
                }
            }
        }

    public:
        // 设置优化问题
        // 配置边界条件、安全走廊、物理参数等
        // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
        //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
        // 参数:
        //   timeWeight: 时间权重rho
        //   initialPVA: 初始状态 [position, velocity, acceleration]
        //   terminalPVA: 终端状态 [position, velocity, acceleration]
        //   safeCorridor: H表示的安全走廊多面体序列
        //   lengthPerPiece: 每段轨迹的期望长度(用于分段)
        //   smoothingFactor: 平滑L1惩罚的平滑因子
        //   integralResolution: 积分分辨率(每段采样点数)
        //   magnitudeBounds: 幅值约束边界
        //   penaltyWeights: 惩罚权重
        //   physicalParams: 物理参数
        // 返回: 成功返回true，失败(如走廊处理失败)返回false
        inline bool setup(const double &timeWeight,
                          const Eigen::Matrix3d &initialPVA,
                          const Eigen::Matrix3d &terminalPVA,
                          const PolyhedraH &safeCorridor,
                          const double &lengthPerPiece,
                          const double &smoothingFactor,
                          const int &integralResolution,
                          const Eigen::VectorXd &magnitudeBounds,
                          const Eigen::VectorXd &penaltyWeights,
                          const Eigen::VectorXd &physicalParams)
        {
            // 保存优化参数
            rho = timeWeight;
            headPVA = initialPVA;
            tailPVA = terminalPVA;

            // 处理安全走廊
            hPolytopes = safeCorridor;
            // 归一化每个半空间的法向量
            for (size_t i = 0; i < hPolytopes.size(); i++)
            {
                const Eigen::ArrayXd norms =
                    hPolytopes[i].leftCols<3>().rowwise().norm();
                hPolytopes[i].array().colwise() /= norms;
            }
            // 转换为V表示
            if (!processCorridor(hPolytopes, vPolytopes))
            {
                return false;
            }

            // 保存配置参数
            polyN = hPolytopes.size();
            smoothEps = smoothingFactor;
            integralRes = integralResolution;
            magnitudeBd = magnitudeBounds;
            penaltyWt = penaltyWeights;
            physicalPm = physicalParams;
            allocSpeed = magnitudeBd(0) * 3.0;  // 初始时间分配速度(取最大速度的3倍，偏保守)

            // 计算走廊内最短路径
            getShortestPath(headPVA.col(0), tailPVA.col(0),
                            vPolytopes, smoothEps, shortPath);

            // 根据最短路径长度自适应分段
            const Eigen::Matrix3Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
            pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
            pieceIdx.array() += 1;  // 至少1段
            pieceN = pieceIdx.sum();

            // 设置优化变量维度
            temporalDim = pieceN;   // 时间变量数 = 轨迹段数
            spatialDim = 0;         // 空间变量数(稍后累加计算)
            vPolyIdx.resize(pieceN - 1);  // 每个内部点对应的V多面体索引
            hPolyIdx.resize(pieceN);      // 每个轨迹段对应的H多面体索引

            // 建立索引映射
            for (int i = 0, j = 0, k; i < polyN; i++)
            {
                k = pieceIdx(i);  // 第i个多面体对应k段轨迹
                for (int l = 0; l < k; l++, j++)
                {
                    // 内部点在多面体内部或重叠区域
                    if (l < k - 1)
                    {
                        // 在第i个多面体内部
                        vPolyIdx(j) = 2 * i;
                        spatialDim += vPolytopes[2 * i].cols();
                    }
                    else if (i < polyN - 1)
                    {
                        // 在第i和i+1个多面体的重叠区域
                        vPolyIdx(j) = 2 * i + 1;
                        spatialDim += vPolytopes[2 * i + 1].cols();
                    }
                    // 轨迹段在第i个多面体内
                    hPolyIdx(j) = i;
                }
            }

            // 初始化MINCO轨迹生成器、微分平坦映射
            minco.setConditions(headPVA, tailPVA, pieceN);
            // 配置微分平坦映射: (mass, gravity, h_drag, v_drag, p_drag, smooth_factor)
            flatmap.reset(physicalPm(0), physicalPm(1), physicalPm(2),
                          physicalPm(3), physicalPm(4), physicalPm(5));

            // 预分配临时变量(避免优化过程中重复分配内存)
            points.resize(3, pieceN - 1);               // 内部路径点
            times.resize(pieceN);                       // 时间分配
            gradByPoints.resize(3, pieceN - 1);         // 对路径点的梯度
            gradByTimes.resize(pieceN);                 // 对时间的梯度
            partialGradByCoeffs.resize(6 * pieceN, 3);  // 对系数的偏梯度
            partialGradByTimes.resize(pieceN);          // 对时间的偏梯度

            return true;
        }

        // 执行轨迹优化
        // 使用L-BFGS算法求解最优轨迹
        // 参数:
        //   traj: 输出优化后的轨迹(5阶多项式)
        //   relCostTol: 相对代价容差(收敛判据)
        // 返回: 最优代价值，失败时返回INFINITY
        inline double optimize(Trajectory<5> &traj,
                               const double &relCostTol)
        {
            // 初始化优化变量 x = [tau; xi]
            Eigen::VectorXd x(temporalDim + spatialDim);
            Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
            Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);

            // 设置初始猜测
            setInitial(shortPath, allocSpeed, pieceIdx, points, times);
            backwardT(times, tau);     // 将物理时间映射为优化变量
            backwardP(points, vPolyIdx, vPolytopes, xi);  // 将路径点映射为优化变量

            // 配置L-BFGS求解器参数
            double minCostFunctional;
            lbfgs_params.mem_size = 256;         // 历史信息存储量
            lbfgs_params.past = 3;               // 用于停机判据的历史窗口
            lbfgs_params.min_step = 1.0e-32;     // 最小步长
            lbfgs_params.g_epsilon = 0.0;        // 梯度容差(禁用)
            lbfgs_params.delta = relCostTol;     // 相对代价变化容差

            // 执行优化
            int ret = lbfgs::lbfgs_optimize(x,
                                            minCostFunctional,
                                            &GCOPTER_PolytopeSFC::costFunctional,
                                            nullptr,
                                            nullptr,
                                            this,
                                            lbfgs_params);

            // 解码优化结果
            if (ret >= 0)
            {
                // 优化成功
                forwardT(tau, times);     // 映射回物理时间
                forwardP(xi, vPolyIdx, vPolytopes, points);  // 映射回路径点
                minco.setParameters(points, times);
                minco.getTrajectory(traj);  // 生成最终轨迹
            }
            else
            {
                // 优化失败
                traj.clear();
                minCostFunctional = INFINITY;
                std::cout << "Optimization Failed: "
                          << lbfgs::lbfgs_strerror(ret)
                          << std::endl;
            }

            return minCostFunctional;
        }
    };

}

#endif
