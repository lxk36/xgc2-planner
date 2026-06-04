// Eigen库，用于矩阵和向量运算
#include <Eigen/Eigen>
// Ceres Solver库，用于非线性最小二乘优化
#include <ceres/ceres.h>

/**
 * @brief 传感器对齐误差结构体
 *
 * 该结构体用于计算相机和激光雷达(Velodyne)之间的对齐误差，
 * 通过Ceres优化器进行传感器外参标定。
 *
 * 优化目标：找到世界坐标系到相机/激光雷达的变换，以及相机到激光雷达的外参变换，
 * 使得两个传感器的位姿在同一坐标系下对齐。
 */
struct AlignError {
    /**
     * @brief 构造函数，初始化相机和激光雷达的观测位姿
     *
     * @param camera_pose 相机在某参考系下的旋转姿态（四元数）
     * @param camera_trans 相机在某参考系下的平移向量
     * @param velodyne_pose 激光雷达在某参考系下的旋转姿态（四元数）
     * @param velodyne_trans 激光雷达在某参考系下的平移向量
     */
    AlignError(  const Eigen::Quaterniond camera_pose, const Eigen::Vector3d camera_trans,
                 const Eigen::Quaterniond velodyne_pose, const Eigen::Vector3d velodyne_trans)
    {
      // 存储相机姿态的四元数分量 (x, y, z, w)
      camera_q[0] = camera_pose.x();
      camera_q[1] = camera_pose.y();
      camera_q[2] = camera_pose.z();
      camera_q[3] = camera_pose.w();
      // 存储相机位置的平移分量 (x, y, z)
      camera_t[0] = camera_trans.x();
      camera_t[1] = camera_trans.y();
      camera_t[2] = camera_trans.z();

      // 存储激光雷达姿态的四元数分量 (x, y, z, w)
      velodyne_q[0] = velodyne_pose.x();
      velodyne_q[1] = velodyne_pose.y();
      velodyne_q[2] = velodyne_pose.z();
      velodyne_q[3] = velodyne_pose.w();
      // 存储激光雷达位置的平移分量 (x, y, z)
      velodyne_t[0] = velodyne_trans.x();
      velodyne_t[1] = velodyne_trans.y();
      velodyne_t[2] = velodyne_trans.z();
    }
 
    /**
     * @brief 工厂方法，创建Ceres代价函数对象
     *
     * 使用自动微分(AutoDiff)创建代价函数，封装AlignError对象的构造过程。
     *
     * @param camera_pose 相机观测到的旋转姿态
     * @param camera_trans 相机观测到的平移向量
     * @param velodyne_pose 激光雷达观测到的旋转姿态
     * @param velodyne_trans 激光雷达观测到的平移向量
     * @return Ceres代价函数指针
     *
     * 模板参数说明：
     * - AlignError: 代价函数类型
     * - 6: 残差维度（3维旋转 + 3维平移）
     * - 4: 第一个参数维度（世界坐标系旋转四元数）
     * - 3: 第二个参数维度（世界坐标系平移）
     * - 4: 第三个参数维度（相机到激光雷达旋转四元数）
     * - 3: 第四个参数维度（相机到激光雷达平移）
     */
    static ceres::CostFunction* Create( const Eigen::Quaterniond camera_pose, const Eigen::Vector3d camera_trans,
                                        const Eigen::Quaterniond velodyne_pose, const Eigen::Vector3d velodyne_trans)
    {
        return (new ceres::AutoDiffCostFunction<AlignError, 6, 4, 3, 4, 3>(
            new AlignError(camera_pose, camera_trans, velodyne_pose, velodyne_trans)));
    }
 
    /**
     * @brief 重载函数调用运算符，计算对齐残差
     *
     * 该函数是Ceres自动微分的核心，计算传感器对齐的残差向量。
     *
     * 坐标变换链：
     * 世界坐标系 -> 相机坐标系 -> 激光雷达坐标系
     * 通过优化world变换和v2c(velodyne to camera)外参，使两个传感器对齐。
     *
     * @tparam T 模板类型，支持double或Jet类型（用于自动微分）
     * @param world_rotation 世界坐标系的旋转四元数（待优化）
     * @param world_translation 世界坐标系的平移向量（待优化）
     * @param v2c_rotation 激光雷达到相机的旋转四元数（待优化）
     * @param v2c_translation 激光雷达到相机的平移向量（待优化）
     * @param residuals 输出的残差向量，维度为6（3旋转+3平移）
     * @return 始终返回true，表示计算成功
     */
    template <typename T>
    bool operator()(const T* const world_rotation, const T* const world_translation,
                    const T* const v2c_rotation, const T* const v2c_translation,
                    T* residuals) const
    {
        // 从数组映射为Eigen四元数和向量类型（世界坐标系变换）
        Eigen::Quaternion<T> q_world = Eigen::Map< const Eigen::Quaternion<T> >(world_rotation);
        Eigen::Matrix<T,3,1> t_world = Eigen::Map< const Eigen::Matrix<T,3,1> >(world_translation);

        // 从数组映射为Eigen四元数和向量类型（激光雷达到相机的外参变换）
        Eigen::Quaternion<T> q_v2c = Eigen::Map< const Eigen::Quaternion<T> >(v2c_rotation);
        Eigen::Matrix<T,3,1> t_v2c = Eigen::Map< const Eigen::Matrix<T,3,1> >(v2c_translation);

        // 构造相机的观测位姿（固定值，从构造函数传入）
        Eigen::Quaternion<T> q_c;
        Eigen::Matrix<T,3,1> t_c;
        q_c.x() = T(camera_q[0]);
        q_c.y() = T(camera_q[1]);
        q_c.z() = T(camera_q[2]);
        q_c.w() = T(camera_q[3]);
        t_c << T(camera_t[0]), T(camera_t[1]), T(camera_t[2]);

        // 构造激光雷达的观测位姿（固定值，从构造函数传入）
        Eigen::Quaternion<T> q_v;
        Eigen::Matrix<T,3,1> t_v;
        q_v.x() = T(velodyne_q[0]);
        q_v.y() = T(velodyne_q[1]);
        q_v.z() = T(velodyne_q[2]);
        q_v.w() = T(velodyne_q[3]);
        t_v << T(velodyne_t[0]), T(velodyne_t[1]), T(velodyne_t[2]);

        // 计算左侧的复合变换：世界坐标系 -> 相机 -> 激光雷达
        Eigen::Quaternion<T> q_left;
        Eigen::Matrix<T,3,1> t_left;

        // 旋转复合：q_left = q_world * q_c * q_v2c
        q_left = q_world * q_c * q_v2c;
        // 平移复合：t_left = q_world * (q_c * t_v2c + t_c) + t_world
        t_left = q_world * q_c * t_v2c + q_world * t_c + t_world;

        // 计算与激光雷达观测位姿的差异（残差）
        Eigen::Quaternion<T> q_diff;
        Eigen::Matrix<T,3,1> t_diff;
        // 旋转差异：将左侧变换与激光雷达姿态对比
        q_diff = q_left * q_v.inverse();
        // 平移差异：简化版本，直接计算平移差
        // 注释掉的版本：t_diff = t_left - q_left * q_v.inverse() * t_v;
        t_diff = t_left - t_v;

        // 填充残差向量：前3维为旋转残差（四元数虚部），后3维为平移残差
        residuals[0] = q_diff.x();
        residuals[1] = q_diff.y();
        residuals[2] = q_diff.z();
        residuals[3] = t_diff(0);
        residuals[4] = t_diff(1);
        residuals[5] = t_diff(2);

        return true;
    }

    // 成员变量：存储观测数据
    double camera_q[4];      // 相机姿态四元数 [x, y, z, w]
    double camera_t[3];      // 相机位置平移向量 [x, y, z]
    double velodyne_q[4];    // 激光雷达姿态四元数 [x, y, z, w]
    double velodyne_t[3];    // 激光雷达位置平移向量 [x, y, z]
};
