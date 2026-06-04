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
 * @namespace ceres_ext
 * @brief Ceres优化库的扩展命名空间，提供四元数相关的参数化和旋转操作
 *
 * 该命名空间包含了用于Ceres优化的四元数局部参数化类以及相关的四元数操作函数。
 * 主要用于在优化过程中正确处理四元数的流形结构。
 */
namespace ceres_ext {

    /**
     * @class EigenQuaternionParameterization
     * @brief Eigen四元数的局部参数化类，用于Ceres优化
     *
     * Plus(x, delta) = [cos(|delta|), sin(|delta|) delta / |delta|] * x
     * 其中 * 是四元数乘法运算符。这里假设四元数向量的第一个元素是实部（cos theta）。
     *
     * 该类实现了四元数的局部参数化，将4维四元数空间映射到3维切空间，
     * 避免了冗余参数并保持了四元数的单位范数约束。
     */
    class EigenQuaternionParameterization : public ceres::LocalParameterization
    {
    public:
        /**
         * @brief 虚析构函数
         */
        virtual ~EigenQuaternionParameterization() {}

        /**
         * @brief 四元数加法操作：x_plus_delta = Plus(x, delta)
         *
         * 将3维增量向量delta应用到4维四元数x上，得到新的四元数x_plus_delta。
         * 该操作通过将delta转换为四元数形式，然后与x进行四元数乘法实现。
         *
         * @param x_raw 输入四元数 (4维: [x, y, z, w])
         * @param delta_raw 增量向量 (3维: [dx, dy, dz])
         * @param x_plus_delta_raw 输出四元数 (4维)
         * @return true 表示操作成功
         */
        virtual bool Plus(const double* x_raw, const double* delta_raw, double* x_plus_delta_raw) const
        {
            // 将原始指针映射为Eigen四元数和向量
            const Eigen::Map<const Eigen::Quaterniond> x(x_raw);
            const Eigen::Map<const Eigen::Vector3d > delta(delta_raw);

            Eigen::Map<Eigen::Quaterniond> x_plus_delta(x_plus_delta_raw);

            // 计算增量向量的范数
            const double delta_norm = delta.norm();
            if ( delta_norm > 0.0 )
            {
                // 计算sin(|delta|) / |delta|，用于构造增量四元数
                const double sin_delta_by_delta = sin(delta_norm) / delta_norm;
                // 构造增量四元数: [cos(|delta|), sin(|delta|)/|delta| * delta]
                Eigen::Quaterniond tmp( cos(delta_norm), sin_delta_by_delta*delta[0], sin_delta_by_delta*delta[1], sin_delta_by_delta*delta[2] );

                // 四元数乘法：x_plus_delta = tmp * x
                x_plus_delta = tmp*x;
            }
            else
            {
                // 增量为零时，直接返回原四元数
                x_plus_delta = x;
            }
            return true;
        }

        /**
         * @brief 计算Plus操作的雅可比矩阵
         *
         * 计算 d(Plus(x, delta)) / d(delta) 在 delta = 0 处的雅可比矩阵。
         * 雅可比矩阵为4x3，将3维切空间映射到4维四元数空间。
         *
         * @param x 输入四元数 (4维: [x, y, z, w])
         * @param jacobian 输出雅可比矩阵 (4x3 = 12个元素，行主序存储)
         * @return true 表示计算成功
         */
        virtual bool ComputeJacobian(const double* x, double* jacobian) const
        {
            // 雅可比矩阵 J 的每一行对应四元数的一个分量 [x, y, z, w]
            jacobian[0] =  x[3]; jacobian[1]  =  x[2]; jacobian[2]   = -x[1];  // NOLINT x分量的导数
            jacobian[3] = -x[2]; jacobian[4]  =  x[3]; jacobian[5]   =  x[0];  // NOLINT y分量的导数
            jacobian[6] =  x[1]; jacobian[7]  = -x[0]; jacobian[8]   =  x[3];  // NOLINT z分量的导数
            jacobian[9] = -x[0]; jacobian[10] = -x[1]; jacobian[11] = -x[2];  // NOLINT w分量的导数
                return true;
        }

        /**
         * @brief 返回全局参数化的维度（四元数的维度）
         * @return 4 (四元数有4个分量)
         */
        virtual int GlobalSize() const { return 4; }

        /**
         * @brief 返回局部参数化的维度（切空间的维度）
         * @return 3 (3维旋转自由度)
         */
        virtual int LocalSize() const { return 3; }

    };

    /**
     * @brief 将Eigen四元数转换为缩放旋转矩阵（数组形式）
     *
     * @tparam T 数据类型（支持自动微分）
     * @param q 输入四元数 (4维: [x, y, z, w])
     * @param R 输出旋转矩阵 (3x3 = 9个元素，行主序)
     *
     * @note 该函数返回的旋转矩阵是缩放的，即 R * norm(q)^2，需要归一化才能得到标准旋转矩阵
     */
    template <typename T> inline
    void EigenQuaternionToScaledRotation(const T q[4], T R[3 * 3]) {
        EigenQuaternionToScaledRotation(q, RowMajorAdapter3x3(R));
    }

    /**
     * @brief 将Eigen四元数转换为缩放旋转矩阵（矩阵适配器形式）
     *
     * 使用四元数公式将四元数转换为旋转矩阵。为了提高效率，该函数返回的是
     * 缩放后的旋转矩阵（缩放因子为 norm(q)^2），避免了除法运算。
     *
     * @tparam T 数据类型（支持自动微分）
     * @tparam row_stride 行跨度
     * @tparam col_stride 列跨度
     * @param q 输入四元数 (4维: [x, y, z, w])
     * @param R 输出旋转矩阵的矩阵适配器
     */
    template <typename T, int row_stride, int col_stride> inline
    void EigenQuaternionToScaledRotation(const T q[4],
					 const ceres::MatrixAdapter<T, row_stride, col_stride>& R) {
        // 为四元数元素创建方便的命名
        T a = q[3];  // w分量（实部）
        T b = q[0];  // x分量（虚部）
        T c = q[1];  // y分量（虚部）
        T d = q[2];  // z分量（虚部）

        // 预计算四元数元素的平方和乘积
        // 这样做不是为了消除公共子表达式，而是为了使代码行更短以适应80列宽度
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

        // 使用四元数到旋转矩阵的标准公式
        // 旋转矩阵 R = (缩放因子) * 标准旋转矩阵
        R(0, 0) = aa + bb - cc - dd; R(0, 1) = T(2) * (bc - ad);  R(0, 2) = T(2) * (ac + bd);  // NOLINT
        R(1, 0) = T(2) * (ad + bc);  R(1, 1) = aa - bb + cc - dd; R(1, 2) = T(2) * (cd - ab);  // NOLINT
        R(2, 0) = T(2) * (bd - ac);  R(2, 1) = T(2) * (ab + cd);  R(2, 2) = aa - bb - cc + dd; // NOLINT
    }

    /**
     * @brief 将Eigen四元数转换为归一化旋转矩阵（数组形式）
     *
     * @tparam T 数据类型（支持自动微分）
     * @param q 输入四元数 (4维: [x, y, z, w])
     * @param R 输出旋转矩阵 (3x3 = 9个元素，行主序)
     *
     * @note 该函数返回标准的归一化旋转矩阵，即使输入四元数不是单位四元数也能正确处理
     */
    template <typename T> inline
    void EigenQuaternionToRotation(const T q[4], T R[3 * 3]) {
        EigenQuaternionToRotation(q, RowMajorAdapter3x3(R));
    }

    /**
     * @brief 将Eigen四元数转换为归一化旋转矩阵（矩阵适配器形式）
     *
     * 先调用缩放旋转矩阵函数，然后通过归一化因子将其转换为标准旋转矩阵。
     * 即使输入四元数不是单位四元数，该函数也能正确处理。
     *
     * @tparam T 数据类型（支持自动微分）
     * @tparam row_stride 行跨度
     * @tparam col_stride 列跨度
     * @param q 输入四元数 (4维: [x, y, z, w])
     * @param R 输出旋转矩阵的矩阵适配器
     */
    template <typename T, int row_stride, int col_stride> inline
    void EigenQuaternionToRotation(const T q[4],
				   const ceres::MatrixAdapter<T, row_stride, col_stride>& R) {
        // 先计算缩放旋转矩阵
        EigenQuaternionToScaledRotation(q, R);

        // 计算归一化因子：1 / (四元数的模的平方)
        T normalizer = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
        CHECK_NE(normalizer, T(0));  // 确保四元数不为零
        normalizer = T(1) / normalizer;

        // 对旋转矩阵的每个元素进行归一化
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R(i, j) *= normalizer;
            }
        }
    }

    /**
     * @brief 使用单位四元数旋转一个三维点
     *
     * 使用单位四元数 q 对三维点 pt 进行旋转变换。该函数假设输入的四元数是单位四元数。
     * 使用优化的公式直接计算旋转后的点，避免了显式构造旋转矩阵。
     *
     * @tparam T 数据类型（支持自动微分）
     * @param q 输入单位四元数 (4维: [x, y, z, w])，必须满足 norm(q) = 1
     * @param pt 输入三维点 (3维: [x, y, z])
     * @param result 输出旋转后的三维点 (3维: [x, y, z])
     */
    template <typename T> inline
    void EigenUnitQuaternionRotatePoint(const T q[4], const T pt[3], T result[3]) {
        // 预计算四元数各分量的乘积，用于构造旋转公式
        const T t2 =  q[3] * q[0];  // w * x
        const T t3 =  q[3] * q[1];  // w * y
        const T t4 =  q[3] * q[2];  // w * z
        const T t5 = -q[0] * q[0];  // -x^2
        const T t6 =  q[0] * q[1];  // x * y
        const T t7 =  q[0] * q[2];  // x * z
        const T t8 = -q[1] * q[1];  // -y^2
        const T t9 =  q[1] * q[2];  // y * z
        const T t1 = -q[2] * q[2];  // -z^2

        // 使用四元数旋转公式：result = q * pt * q^(-1)
        // 通过优化的矩阵形式直接计算，避免显式构造旋转矩阵
        result[0] = T(2) * ((t8 + t1) * pt[0] + (t6 - t4) * pt[1] + (t3 + t7) * pt[2]) + pt[0];  // NOLINT
        result[1] = T(2) * ((t4 + t6) * pt[0] + (t5 + t1) * pt[1] + (t9 - t2) * pt[2]) + pt[1];  // NOLINT
        result[2] = T(2) * ((t7 - t3) * pt[0] + (t2 + t9) * pt[1] + (t5 + t8) * pt[2]) + pt[2];  // NOLINT
    }

    /**
     * @brief 使用任意四元数旋转一个三维点
     *
     * 使用四元数 q（可以是非单位四元数）对三维点 pt 进行旋转变换。
     * 该函数首先将输入四元数归一化为单位四元数，然后执行旋转操作。
     *
     * @tparam T 数据类型（支持自动微分）
     * @param q 输入四元数 (4维: [x, y, z, w])，可以是非单位四元数
     * @param pt 输入三维点 (3维: [x, y, z])
     * @param result 输出旋转后的三维点 (3维: [x, y, z])
     */
    template <typename T> inline
    void EigenQuaternionRotatePoint(const T q[4], const T pt[3], T result[3]) {
        // 计算缩放因子：scale = 1 / norm(q)
        const T scale = T(1) / sqrt(q[0] * q[0] +
                                    q[1] * q[1] +
                                    q[2] * q[2] +
                                    q[3] * q[3]);

        // 将四元数归一化为单位四元数
        const T unit[4] = {
            scale * q[0],
            scale * q[1],
            scale * q[2],
            scale * q[3],
        };

        // 使用单位四元数进行旋转
        EigenUnitQuaternionRotatePoint(unit, pt, result);
    }

    /**
     * @brief 计算两个Eigen四元数的乘积
     *
     * 计算四元数乘法 zw = z * w。四元数乘法是非交换的，即 z * w ≠ w * z。
     * 该函数使用标准的四元数乘法公式进行计算。
     *
     * 四元数表示为 [x, y, z, w]，其中 w 是实部，(x, y, z) 是虚部。
     *
     * @tparam T 数据类型（支持自动微分）
     * @param z 第一个四元数 (4维: [x, y, z, w])
     * @param w 第二个四元数 (4维: [x, y, z, w])
     * @param zw 输出乘积四元数 zw = z * w (4维: [x, y, z, w])
     */
    template<typename T> inline
    void EigenQuaternionProduct(const T z[4], const T w[4], T zw[4]) {
        // 四元数乘法公式：
        // 如果 z = (z.x, z.y, z.z, z.w) 和 w = (w.x, w.y, w.z, w.w)
        // 则 zw = z * w 的各分量为：
        zw[0] =   z[0] * w[3] + z[1] * w[2] - z[2] * w[1] + z[3] * w[0];  // x分量
        zw[1] = - z[0] * w[2] + z[1] * w[3] + z[2] * w[0] + z[3] * w[1];  // y分量
        zw[2] =   z[0] * w[1] - z[1] * w[0] + z[2] * w[3] + z[3] * w[2];  // z分量
        zw[3] = - z[0] * w[0] - z[1] * w[1] - z[2] * w[2] + z[3] * w[3];  // w分量（实部）
    }

}  // namespace ceres_ext

#endif  // CERES_EXTENSIONS_H