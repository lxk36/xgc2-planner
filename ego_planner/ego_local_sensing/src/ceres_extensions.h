//
//  ceres_extensions.h
//  Bundle_Adjust_Test
//
//  Created by Lloyd Hughes on 2014/04/11.
//  Copyright (c) 2014 Lloyd Hughes. All rights reserved.
//  hughes.lloyd@gmail.com
//

#ifndef CERES_EXTENSIONS_H
#define CERES_EXTENSIONS_H

#include <Eigen/Core>
#include <ceres/local_parameterization.h>
#include <ceres/rotation.h>

/**
 * @brief Ceres扩展命名空间
 *
 * 本文件提供了用于Ceres优化库的四元数参数化扩展功能
 * 主要包含：
 * 1. Eigen四元数的局部参数化实现，用于Ceres优化中的四元数流形优化
 * 2. 四元数与旋转矩阵之间的转换函数
 * 3. 四元数旋转点的高效计算方法
 * 4. 四元数乘法运算
 *
 * 这些扩展支持自动微分和数值微分，适用于视觉SLAM、位姿优化等应用场景
 */
namespace ceres_ext {

    /**
     * @class EigenQuaternionParameterization
     * @brief Eigen四元数的局部参数化类
     *
     * 四元数局部参数化用于在流形上进行优化，避免冗余参数带来的问题
     *
     * 参数化公式：
     * Plus(x, delta) = [cos(|delta|), sin(|delta|) delta / |delta|] * x
     *
     * 其中：
     * - x 是当前四元数状态 (全局参数空间，4维)
     * - delta 是切空间增量 (局部参数空间，3维)
     * - * 是四元数乘法运算符
     * - 四元数格式：[x, y, z, w]，其中w是实部(cos theta)
     *
     * 这种参数化方式的优势：
     * 1. 避免四元数单位范数约束的显式处理
     * 2. 将4维优化问题降低到3维切空间
     * 3. 数值稳定性好，避免万向锁问题
     */
    class EigenQuaternionParameterization : public ceres::LocalParameterization
    {
    public:
        virtual ~EigenQuaternionParameterization() {}
        
        /**
         * @brief 四元数流形上的加法操作
         *
         * 实现流形上的更新操作：x_new = Plus(x, delta)
         * 使用指数映射将切空间的3维增量映射到四元数流形上
         *
         * @param x_raw 当前四元数 [x, y, z, w] (4维全局参数)
         * @param delta_raw 切空间增量向量 [dx, dy, dz] (3维局部参数)
         * @param x_plus_delta_raw 输出：更新后的四元数 (4维)
         * @return true 表示操作成功
         *
         * 算法流程：
         * 1. 计算增量的范数 |delta|
         * 2. 构造旋转四元数：q_delta = [cos(|delta|), sin(|delta|)/|delta| * delta]
         * 3. 四元数右乘：x_new = q_delta * x
         * 4. 特殊情况：当|delta|接近0时，直接返回x（避免除零）
         */
        virtual bool Plus(const double* x_raw, const double* delta_raw, double* x_plus_delta_raw) const
        {
            // 将原始指针映射为Eigen四元数和向量对象
            const Eigen::Map<const Eigen::Quaterniond> x(x_raw);        // 当前四元数
            const Eigen::Map<const Eigen::Vector3d > delta(delta_raw);  // 切空间增量

            Eigen::Map<Eigen::Quaterniond> x_plus_delta(x_plus_delta_raw); // 输出四元数

            // 计算增量向量的2范数
            const double delta_norm = delta.norm();

            // 如果增量非零，执行四元数更新
            if ( delta_norm > 0.0 )
            {
                // 计算 sin(|delta|) / |delta|，用于构造单位四元数
                const double sin_delta_by_delta = sin(delta_norm) / delta_norm;

                // 构造表示旋转的四元数：[w, x, y, z] = [cos(θ), sin(θ)*axis]
                Eigen::Quaterniond tmp( cos(delta_norm),
                                       sin_delta_by_delta*delta[0],
                                       sin_delta_by_delta*delta[1],
                                       sin_delta_by_delta*delta[2] );

                // 四元数右乘更新：q_new = q_delta * q_old
                x_plus_delta = tmp*x;
            }
            else
            {
                // 增量为零时，保持不变
                x_plus_delta = x;
            }
            return true;
        }
        
        /**
         * @brief 计算局部参数化的雅可比矩阵
         *
         * 计算Plus操作对局部参数delta的导数：J = d(Plus(x, delta))/d(delta) 在 delta=0 处
         * 雅可比矩阵大小为 4x3（全局参数维度 × 局部参数维度）
         *
         * @param x 当前四元数 [x, y, z, w]
         * @param jacobian 输出：4x3雅可比矩阵，按行优先存储（连续12个元素）
         * @return true 表示计算成功
         *
         * 矩阵形式：
         *     [  w   z  -y ]    <- d(qx)/d(delta)
         * J = [ -z   w   x ]    <- d(qy)/d(delta)
         *     [  y  -x   w ]    <- d(qz)/d(delta)
         *     [ -x  -y  -z ]    <- d(qw)/d(delta)
         *
         * 这个雅可比矩阵描述了四元数在当前状态下对切空间扰动的敏感度
         * Ceres使用此矩阵进行梯度下降和牛顿法等优化算法
         */
        virtual bool ComputeJacobian(const double* x, double* jacobian) const
        {
            // 第一行：d(qx)/d(delta) = [w, z, -y]
            jacobian[0] =  x[3]; jacobian[1]  =  x[2]; jacobian[2]   = -x[1];  // NOLINT x
            // 第二行：d(qy)/d(delta) = [-z, w, x]
            jacobian[3] = -x[2]; jacobian[4]  =  x[3]; jacobian[5]   =  x[0];  // NOLINT y
            // 第三行：d(qz)/d(delta) = [y, -x, w]
            jacobian[6] =  x[1]; jacobian[7]  = -x[0]; jacobian[8]   =  x[3];  // NOLINT z
            // 第四行：d(qw)/d(delta) = [-x, -y, -z]
            jacobian[9] = -x[0]; jacobian[10] = -x[1]; jacobian[11] = -x[2];  // NOLINT w
            return true;
        }

        /**
         * @brief 返回全局参数空间的维度
         * @return 4 (四元数有4个分量)
         */
        virtual int GlobalSize() const { return 4; }

        /**
         * @brief 返回局部参数空间的维度
         * @return 3 (切空间是3维的，对应旋转的3个自由度)
         */
        virtual int LocalSize() const { return 3; }

    };

    /**
     * @brief 将Eigen四元数转换为缩放旋转矩阵（数组形式）
     *
     * 这是一个包装函数，将普通数组转换为矩阵适配器后调用实际转换函数
     *
     * @tparam T 数据类型（支持double、float、Jet等，用于自动微分）
     * @param q 输入四元数 [x, y, z, w]，注意这里w是第4个元素
     * @param R 输出：3x3旋转矩阵（按行优先存储，9个元素）
     *
     * 注意：输出的旋转矩阵是"缩放"的，即 R_scaled = |q|^2 * R_normalized
     * 如果四元数未归一化，矩阵会包含范数的平方作为缩放因子
     */
    template <typename T> inline
    void EigenQuaternionToScaledRotation(const T q[4], T R[3 * 3]) {
        EigenQuaternionToScaledRotation(q, RowMajorAdapter3x3(R));
    }

    /**
     * @brief 将Eigen四元数转换为缩放旋转矩阵（模板化版本）
     *
     * 使用四元数公式直接计算旋转矩阵，不进行归一化
     * 支持Ceres的自动微分类型（Jet）
     *
     * @tparam T 数据类型
     * @tparam row_stride 矩阵行步长
     * @tparam col_stride 矩阵列步长
     * @param q 输入四元数 [x, y, z, w]
     * @param R 输出：矩阵适配器，接收计算得到的旋转矩阵
     *
     * 四元数到旋转矩阵的转换公式：
     * 令 q = [x, y, z, w] = [b, c, d, a]（为了简化书写）
     *
     *     [ a²+b²-c²-d²   2(bc-ad)      2(ac+bd)   ]
     * R = [  2(ad+bc)    a²-b²+c²-d²   2(cd-ab)   ]
     *     [  2(bd-ac)     2(ab+cd)     a²-b²-c²+d² ]
     *
     * 注意：此函数不检查四元数是否归一化，输出矩阵会缩放 |q|^2 倍
     */
    template <typename T, int row_stride, int col_stride> inline
    void EigenQuaternionToScaledRotation(const T q[4],
					 const ceres::MatrixAdapter<T, row_stride, col_stride>& R) {
        // 为四元数元素创建方便的别名
        // 四元数 q = [x, y, z, w] -> [b, c, d, a]
        T a = q[3];  // w分量（实部）
        T b = q[0];  // x分量（虚部）
        T c = q[1];  // y分量（虚部）
        T d = q[2];  // z分量（虚部）

        // 预计算所有需要的乘积项（优化性能，减少重复计算）
        T aa = a * a;
        T ab = a * b;
        T ac = a * c;
        T ad = a * d;
        T bb = b * b;
        T bc = b * c;
        T bd = b * d;
        T cc = c * c;
        T cd = c * d;
        T dd = d * d;

        // 填充旋转矩阵（每行对应一个旋转后的基向量）
        R(0, 0) = aa + bb - cc - dd; R(0, 1) = T(2) * (bc - ad);  R(0, 2) = T(2) * (ac + bd);  // NOLINT
        R(1, 0) = T(2) * (ad + bc);  R(1, 1) = aa - bb + cc - dd; R(1, 2) = T(2) * (cd - ab);  // NOLINT
        R(2, 0) = T(2) * (bd - ac);  R(2, 1) = T(2) * (ab + cd);  R(2, 2) = aa - bb - cc + dd; // NOLINT
    }

    /**
     * @brief 将Eigen四元数转换为归一化旋转矩阵（数组形式）
     *
     * 这是一个包装函数，将普通数组转换为矩阵适配器后调用实际转换函数
     *
     * @tparam T 数据类型（支持double、float、Jet等，用于自动微分）
     * @param q 输入四元数 [x, y, z, w]，可以是非单位四元数
     * @param R 输出：3x3标准正交旋转矩阵（按行优先存储，9个元素）
     *
     * 与EigenQuaternionToScaledRotation的区别：
     * - 此函数会对四元数进行归一化，确保输出是标准旋转矩阵（行列式=1）
     * - ScaledRotation版本不归一化，输出矩阵会包含四元数范数的平方作为缩放因子
     */
    template <typename T> inline
    void EigenQuaternionToRotation(const T q[4], T R[3 * 3]) {
        EigenQuaternionToRotation(q, RowMajorAdapter3x3(R));
    }

    /**
     * @brief 将Eigen四元数转换为归一化旋转矩阵（模板化版本）
     *
     * 先计算缩放旋转矩阵，然后除以四元数范数的平方，得到标准正交旋转矩阵
     *
     * @tparam T 数据类型
     * @tparam row_stride 矩阵行步长
     * @tparam col_stride 矩阵列步长
     * @param q 输入四元数 [x, y, z, w]，可以是非单位四元数
     * @param R 输出：矩阵适配器，接收归一化的旋转矩阵
     *
     * 算法步骤：
     * 1. 调用EigenQuaternionToScaledRotation得到 R_scaled = |q|^2 * R
     * 2. 计算归一化因子：normalizer = 1 / |q|^2
     * 3. 矩阵每个元素乘以归一化因子：R = R_scaled / |q|^2
     *
     * 注意：如果四元数范数为0，会触发CHECK断言失败
     */
    template <typename T, int row_stride, int col_stride> inline
    void EigenQuaternionToRotation(const T q[4],
				   const ceres::MatrixAdapter<T, row_stride, col_stride>& R) {
        // 先计算缩放旋转矩阵（未归一化）
        EigenQuaternionToScaledRotation(q, R);

        // 计算四元数的范数平方：|q|^2 = x^2 + y^2 + z^2 + w^2
        T normalizer = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];

        // 检查范数不为0（避免除零错误）
        CHECK_NE(normalizer, T(0));

        // 计算归一化因子：1 / |q|^2
        normalizer = T(1) / normalizer;

        // 对旋转矩阵的每个元素进行归一化
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R(i, j) *= normalizer;
            }
        }
    }

    /**
     * @brief 使用单位四元数旋转三维点（优化版本）
     *
     * 通过四元数乘法公式高效计算点的旋转：p' = q * p * q^(-1)
     * 此函数假设输入的四元数已经归一化（|q| = 1），不进行归一化检查
     *
     * @tparam T 数据类型（支持double、float、Jet等）
     * @param q 单位四元数 [x, y, z, w]，必须满足 x^2+y^2+z^2+w^2=1
     * @param pt 输入点 [x, y, z]
     * @param result 输出：旋转后的点 [x', y', z']
     *
     * 优化策略：
     * - 预计算所有需要的中间项，避免重复计算
     * - 使用优化的四元数-点旋转公式，比矩阵乘法更高效
     * - 公式来源：p' = p + 2w(q_vec × p) + 2(q_vec × (q_vec × p))
     *   其中 q = [q_vec, w] = [[x,y,z], w]
     */
    template <typename T> inline
    void EigenUnitQuaternionRotatePoint(const T q[4], const T pt[3], T result[3]) {
        // 预计算所有需要的乘积项（优化性能）
        // 令 q = [x, y, z, w] = [q[0], q[1], q[2], q[3]]
        const T t2 =  q[3] * q[0];   // w * x
        const T t3 =  q[3] * q[1];   // w * y
        const T t4 =  q[3] * q[2];   // w * z
        const T t5 = -q[0] * q[0];   // -x^2
        const T t6 =  q[0] * q[1];   // x * y
        const T t7 =  q[0] * q[2];   // x * z
        const T t8 = -q[1] * q[1];   // -y^2
        const T t9 =  q[1] * q[2];   // y * z
        const T t1 = -q[2] * q[2];   // -z^2

        // 应用旋转公式：result = R(q) * pt
        // 这里使用的是展开的四元数旋转公式，等价于 q * [0, pt] * q^(-1)
        result[0] = T(2) * ((t8 + t1) * pt[0] + (t6 - t4) * pt[1] + (t3 + t7) * pt[2]) + pt[0];  // NOLINT
        result[1] = T(2) * ((t4 + t6) * pt[0] + (t5 + t1) * pt[1] + (t9 - t2) * pt[2]) + pt[1];  // NOLINT
        result[2] = T(2) * ((t7 - t3) * pt[0] + (t2 + t9) * pt[1] + (t5 + t8) * pt[2]) + pt[2];  // NOLINT
    }

    /**
     * @brief 使用任意四元数旋转三维点（带归一化）
     *
     * 先将输入四元数归一化为单位四元数，再执行旋转操作
     * 适用于未归一化的四元数，确保数值稳定性
     *
     * @tparam T 数据类型（支持double、float、Jet等，用于自动微分）
     * @param q 输入四元数 [x, y, z, w]，可以是非单位四元数
     * @param pt 输入点 [x, y, z]
     * @param result 输出：旋转后的点 [x', y', z']
     *
     * 算法步骤：
     * 1. 计算四元数的范数：|q| = sqrt(x^2 + y^2 + z^2 + w^2)
     * 2. 归一化四元数：q_unit = q / |q|
     * 3. 调用EigenUnitQuaternionRotatePoint执行实际旋转
     *
     * 注意：此函数比EigenUnitQuaternionRotatePoint多一次归一化开销
     * 如果确定输入四元数已归一化，应直接使用EigenUnitQuaternionRotatePoint
     */
    template <typename T> inline
    void EigenQuaternionRotatePoint(const T q[4], const T pt[3], T result[3]) {
        // 计算归一化缩放因子：scale = 1 / |q|
        const T scale = T(1) / sqrt(q[0] * q[0] +
                                    q[1] * q[1] +
                                    q[2] * q[2] +
                                    q[3] * q[3]);

        // 创建单位四元数：unit = q / |q|
        const T unit[4] = {
            scale * q[0],  // 归一化后的 x 分量
            scale * q[1],  // 归一化后的 y 分量
            scale * q[2],  // 归一化后的 z 分量
            scale * q[3],  // 归一化后的 w 分量
        };

        // 使用单位四元数执行旋转
        EigenUnitQuaternionRotatePoint(unit, pt, result);
    }

    /**
     * @brief 计算两个Eigen四元数的乘积
     *
     * 实现四元数乘法：zw = z * w
     * 四元数乘法不满足交换律，即 z*w ≠ w*z
     *
     * @tparam T 数据类型（支持double、float、Jet等，用于自动微分）
     * @param z 左侧四元数 [x1, y1, z1, w1]
     * @param w 右侧四元数 [x2, y2, z2, w2]
     * @param zw 输出：四元数乘积 z*w = [x', y', z', w']
     *
     * 四元数乘法公式（Hamilton约定）：
     * 令 z = [z_vec, z_w] = [[x1, y1, z1], w1]
     *    w = [w_vec, w_w] = [[x2, y2, z2], w2]
     *
     * 则 z*w = [z_w*w_vec + w_w*z_vec + z_vec×w_vec, z_w*w_w - z_vec·w_vec]
     *
     * 展开为分量形式：
     *   x' =  x1*w2 + y1*z2 - z1*y2 + w1*x2
     *   y' = -x1*z2 + y1*w2 + z1*x2 + w1*y2
     *   z' =  x1*y2 - y1*x2 + z1*w2 + w1*z2
     *   w' = -x1*x2 - y1*y2 - z1*z2 + w1*w2
     *
     * 应用场景：
     * - 组合旋转：先旋转q1再旋转q2 = q2 * q1（注意顺序）
     * - 更新四元数状态：q_new = q_delta * q_old
     */
    template<typename T> inline
    void EigenQuaternionProduct(const T z[4], const T w[4], T zw[4]) {
        // x分量：x1*w2 + y1*z2 - z1*y2 + w1*x2
        zw[0] =   z[0] * w[3] + z[1] * w[2] - z[2] * w[1] + z[3] * w[0];
        // y分量：-x1*z2 + y1*w2 + z1*x2 + w1*y2
        zw[1] = - z[0] * w[2] + z[1] * w[3] + z[2] * w[0] + z[3] * w[1];
        // z分量：x1*y2 - y1*x2 + z1*w2 + w1*z2
        zw[2] =   z[0] * w[1] - z[1] * w[0] + z[2] * w[3] + z[3] * w[2];
        // w分量（实部）：-x1*x2 - y1*y2 - z1*z2 + w1*w2
        zw[3] = - z[0] * w[0] - z[1] * w[1] - z[2] * w[2] + z[3] * w[3];
    }

} // namespace ceres_ext

#endif // CERES_EXTENSIONS_H