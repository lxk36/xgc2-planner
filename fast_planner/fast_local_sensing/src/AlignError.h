// 包含Eigen线性代数库，用于矩阵和向量运算
#include <Eigen/Eigen>
// 包含Ceres优化库，用于非线性最小二乘优化
#include <ceres/ceres.h>

/**
 * @brief 传感器对齐误差代价函数结构体
 *
 * 用于Ceres优化器的代价函数，计算相机和激光雷达(Velodyne)之间的对齐误差。
 * 该结构体用于多传感器标定，通过最小化对齐误差来求解传感器之间的外参标定参数。
 */
struct AlignError {
    /**
     * @brief 构造函数
     *
     * @param camera_pose 相机的旋转姿态（四元数表示）
     * @param camera_trans 相机的平移向量
     * @param velodyne_pose 激光雷达的旋转姿态（四元数表示）
     * @param velodyne_trans 激光雷达的平移向量
     *
     * 该构造函数将相机和激光雷达的位姿存储为成员变量，用于后续误差计算
     */
    AlignError(  const Eigen::Quaterniond camera_pose, const Eigen::Vector3d camera_trans,
                 const Eigen::Quaterniond velodyne_pose, const Eigen::Vector3d velodyne_trans)
    {
      // 存储相机的四元数旋转 (x, y, z, w)
      camera_q[0] = camera_pose.x();
      camera_q[1] = camera_pose.y();
      camera_q[2] = camera_pose.z();
      camera_q[3] = camera_pose.w();

      // 存储相机的平移向量 (x, y, z)
      camera_t[0] = camera_trans.x();
      camera_t[1] = camera_trans.y();
      camera_t[2] = camera_trans.z();

      // 存储激光雷达的四元数旋转 (x, y, z, w)
      velodyne_q[0] = velodyne_pose.x();
      velodyne_q[1] = velodyne_pose.y();
      velodyne_q[2] = velodyne_pose.z();
      velodyne_q[3] = velodyne_pose.w();

      // 存储激光雷达的平移向量 (x, y, z)
      velodyne_t[0] = velodyne_trans.x();
      velodyne_t[1] = velodyne_trans.y();
      velodyne_t[2] = velodyne_trans.z();
    }
 
    /**
     * @brief 工厂方法：创建代价函数对象
     *
     * @param camera_pose 相机的旋转姿态（四元数表示）
     * @param camera_trans 相机的平移向量
     * @param velodyne_pose 激光雷达的旋转姿态（四元数表示）
     * @param velodyne_trans 激光雷达的平移向量
     * @return 返回Ceres代价函数指针
     *
     * 工厂方法隐藏了代价函数对象的构造细节，向客户端代码提供简洁的接口。
     * 使用Ceres的自动微分功能，模板参数说明：
     * - AlignError: 代价函数类型
     * - 6: 残差维度（3个旋转残差 + 3个平移残差）
     * - 4: 第一个优化参数维度（世界坐标系旋转，四元数4个分量）
     * - 3: 第二个优化参数维度（世界坐标系平移，3D向量）
     * - 4: 第三个优化参数维度（激光雷达到相机的旋转，四元数4个分量）
     * - 3: 第四个优化参数维度（激光雷达到相机的平移，3D向量）
     */
    static ceres::CostFunction* Create( const Eigen::Quaterniond camera_pose, const Eigen::Vector3d camera_trans,
                                        const Eigen::Quaterniond velodyne_pose, const Eigen::Vector3d velodyne_trans)
    {
        return (new ceres::AutoDiffCostFunction<AlignError, 6, 4, 3, 4, 3>(
            new AlignError(camera_pose, camera_trans, velodyne_pose, velodyne_trans)));
    }
 
    /**
     * @brief 重载的函数调用运算符：计算对齐残差
     *
     * @param world_rotation 世界坐标系的旋转（待优化参数）
     * @param world_translation 世界坐标系的平移（待优化参数）
     * @param v2c_rotation 激光雷达到相机的旋转外参（待优化参数）
     * @param v2c_translation 激光雷达到相机的平移外参（待优化参数）
     * @param residuals 输出的残差向量（6维：3个旋转残差 + 3个平移残差）
     * @return 返回true表示计算成功
     *
     * 该函数实现了传感器对齐误差的计算逻辑：
     * 通过计算相机和激光雷达在世界坐标系下的位姿差异，得到对齐残差。
     * 使用模板参数T支持Ceres的自动微分功能。
     */
    template <typename T>
    bool operator()(const T* const world_rotation, const T* const world_translation,
                    const T* const v2c_rotation, const T* const v2c_translation,
                    T* residuals) const
    {
        // 将输入的世界坐标系旋转参数映射为Eigen四元数
        Eigen::Quaternion<T> q_world = Eigen::Map< const Eigen::Quaternion<T> >(world_rotation);
        // 将输入的世界坐标系平移参数映射为Eigen向量
        Eigen::Matrix<T,3,1> t_world = Eigen::Map< const Eigen::Matrix<T,3,1> >(world_translation);

        // 将激光雷达到相机的旋转外参映射为Eigen四元数
        Eigen::Quaternion<T> q_v2c = Eigen::Map< const Eigen::Quaternion<T> >(v2c_rotation);
        // 将激光雷达到相机的平移外参映射为Eigen向量
        Eigen::Matrix<T,3,1> t_v2c = Eigen::Map< const Eigen::Matrix<T,3,1> >(v2c_translation);

        // 相机的位姿（观测值，存储在成员变量中）
        Eigen::Quaternion<T> q_c;
        Eigen::Matrix<T,3,1> t_c;
        q_c.x() = T(camera_q[0]);
        q_c.y() = T(camera_q[1]);
        q_c.z() = T(camera_q[2]);
        q_c.w() = T(camera_q[3]);
        t_c << T(camera_t[0]), T(camera_t[1]), T(camera_t[2]);

        // 激光雷达的位姿（观测值，存储在成员变量中）
        Eigen::Quaternion<T> q_v;
        Eigen::Matrix<T,3,1> t_v;
        q_v.x() = T(velodyne_q[0]);
        q_v.y() = T(velodyne_q[1]);
        q_v.z() = T(velodyne_q[2]);
        q_v.w() = T(velodyne_q[3]);
        t_v << T(velodyne_t[0]), T(velodyne_t[1]), T(velodyne_t[2]);

        // 通过相机观测推算的激光雷达位姿（左边计算路径）
        Eigen::Quaternion<T> q_left;
        Eigen::Matrix<T,3,1> t_left;

        // 计算旋转部分：世界坐标系 -> 相机 -> 激光雷达
        // 公式：R_left = R_world * R_camera * R_v2c（激光雷达到相机的外参）
        q_left = q_world * q_c * q_v2c;

        // 计算平移部分：先将激光雷达到相机的平移转换到相机坐标系，
        // 再转换到世界坐标系，最后加上相机在世界坐标系的位置
        // 公式：t_left = R_world * R_camera * t_v2c + R_world * t_camera + t_world
        t_left = q_world * q_c * t_v2c + q_world * t_c + t_world;

        // 计算位姿差异（对齐误差）
        Eigen::Quaternion<T> q_diff;
        Eigen::Matrix<T,3,1> t_diff;

        // 旋转误差：计算推算位姿与实际激光雷达位姿的旋转差异
        q_diff = q_left * q_v.inverse();

        // 平移误差：计算推算位置与实际激光雷达位置的平移差异
        // 注：下面这行是更精确的计算方式，但被简化为直接相减
        // t_diff = t_left - q_left * q_v.inverse() * t_v;
        t_diff = t_left - t_v;

        // 将旋转误差的四元数虚部（x, y, z）作为残差的前3维
        residuals[0] = q_diff.x();
        residuals[1] = q_diff.y();
        residuals[2] = q_diff.z();

        // 将平移误差的三个分量（x, y, z）作为残差的后3维
        residuals[3] = t_diff(0);
        residuals[4] = t_diff(1);
        residuals[5] = t_diff(2);

        // 返回true表示残差计算成功
        return true;
    }

    // 成员变量：存储相机位姿
    double camera_q[4];     // 相机的四元数旋转 [x, y, z, w]
    double camera_t[3];     // 相机的平移向量 [x, y, z]

    // 成员变量：存储激光雷达位姿
    double velodyne_q[4];   // 激光雷达的四元数旋转 [x, y, z, w]
    double velodyne_t[3];   // 激光雷达的平移向量 [x, y, z]
};
