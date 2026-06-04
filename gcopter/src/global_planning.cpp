// GCOPTER相关头文件
#include "misc/visualizer.hpp"         // 可视化工具类
#include "gcopter/trajectory.hpp"      // 轨迹表示类
#include "gcopter/gcopter.hpp"         // GCOPTER优化器核心算法
#include "gcopter/firi.hpp"            // 有限脉冲响应相关
#include "gcopter/flatness.hpp"        // 微分平坦性映射
#include "gcopter/voxel_map.hpp"       // 体素地图类
#include "gcopter/sfc_gen.hpp"         // 安全飞行走廊生成器

// ROS相关头文件
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

// C++标准库头文件
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

/**
 * @brief 配置参数结构体
 * 存储全局规划器所需的所有配置参数，包括地图、车辆动力学、优化器参数等
 */
struct Config
{
    // 话题相关参数
    std::string mapTopic;              // 地图点云话题名称
    std::string targetTopic;           // 目标位置话题名称

    // 地图相关参数
    double dilateRadius;               // 障碍物膨胀半径（用于安全距离）
    double voxelWidth;                 // 体素网格宽度
    std::vector<double> mapBound;      // 地图边界 [x_min, x_max, y_min, y_max, z_min, z_max]
    double timeoutRRT;                 // RRT路径搜索超时时间

    // 车辆运动约束参数
    double maxVelMag;                  // 最大速度模值
    double maxBdrMag;                  // 最大机体角速度模值
    double maxTiltAngle;               // 最大倾斜角
    double minThrust;                  // 最小推力
    double maxThrust;                  // 最大推力

    // 车辆物理参数
    double vehicleMass;                // 车辆质量
    double gravAcc;                    // 重力加速度
    double horizDrag;                  // 水平方向阻力系数
    double vertDrag;                   // 垂直方向阻力系数
    double parasDrag;                  // 寄生阻力系数
    double speedEps;                   // 速度平滑因子

    // 优化器参数
    double weightT;                    // 时间权重
    std::vector<double> chiVec;        // 惩罚权重向量 [位置, 速度, 角速度, 倾斜角, 推力]
    double smoothingEps;               // 平滑参数
    int integralIntervs;               // 数值积分区间数
    double relCostTol;                 // 相对代价容差

    /**
     * @brief 构造函数，从ROS参数服务器加载所有配置参数
     * @param nh_priv 私有命名空间的节点句柄
     */
    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
        nh_priv.getParam("TimeoutRRT", timeoutRRT);
        nh_priv.getParam("MaxVelMag", maxVelMag);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        nh_priv.getParam("MinThrust", minThrust);
        nh_priv.getParam("MaxThrust", maxThrust);
        nh_priv.getParam("VehicleMass", vehicleMass);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("HorizDrag", horizDrag);
        nh_priv.getParam("VertDrag", vertDrag);
        nh_priv.getParam("ParasDrag", parasDrag);
        nh_priv.getParam("SpeedEps", speedEps);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("RelCostTol", relCostTol);
    }
};

/**
 * @brief 全局路径规划器类
 * 负责接收地图和目标点，生成时间最优且满足动力学约束的轨迹
 * 主要功能：
 * 1. 接收点云地图并构建体素地图
 * 2. 接收目标点并生成安全飞行走廊
 * 3. 使用GCOPTER算法进行轨迹优化
 * 4. 实时可视化轨迹和车辆状态
 */
class GlobalPlanner
{
private:
    Config config;                              // 配置参数

    // ROS通信相关
    ros::NodeHandle nh;                         // ROS节点句柄
    ros::Subscriber mapSub;                     // 地图订阅器
    ros::Subscriber targetSub;                  // 目标点订阅器

    // 地图与环境
    bool mapInitialized;                        // 地图是否已初始化标志
    voxel_map::VoxelMap voxelMap;               // 体素地图对象
    Visualizer visualizer;                      // 可视化工具对象
    std::vector<Eigen::Vector3d> startGoal;     // 起点和终点列表

    // 轨迹相关
    Trajectory<5> traj;                         // 5阶多项式轨迹（位置、速度、加速度、急动度、Snap）
    double trajStamp;                           // 轨迹生成时间戳

public:
    /**
     * @brief 全局规划器构造函数
     * @param conf 配置参数结构体
     * @param nh_ ROS节点句柄
     *
     * 功能：
     * 1. 初始化成员变量
     * 2. 根据地图边界和体素宽度创建体素地图
     * 3. 订阅地图点云和目标点话题
     */
    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          visualizer(nh)
    {
        // 计算体素地图的xyz维度（单位：体素数）
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        // 地图原点偏移量（地图的最小边界点）
        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        // 创建体素地图对象
        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        // 订阅地图点云话题，使用tcpNoDelay提高实时性
        mapSub = nh.subscribe(config.mapTopic, 1, &GlobalPlanner::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());

        // 订阅目标点话题
        targetSub = nh.subscribe(config.targetTopic, 1, &GlobalPlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
    }

    /**
     * @brief 地图点云回调函数
     * @param msg PointCloud2类型的点云消息
     *
     * 功能：
     * 1. 将接收到的点云数据填充到体素地图中
     * 2. 对障碍物进行膨胀处理以确保安全距离
     * 3. 仅在第一次接收时处理（通过mapInitialized标志）
     */
    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (!mapInitialized)
        {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;  // 点云中点的总数
            float *fdata = (float *)(&msg->data[0]);                  // 指向点云数据的指针

            // 遍历所有点云数据
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;  // 计算当前点的数据起始位置

                // 跳过无效点（NaN或Inf）
                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
                // 将有效点标记为占据
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }

            // 对障碍物进行膨胀处理，膨胀距离为配置的dilateRadius
            // 转换为体素单位（向上取整）
            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));

            mapInitialized = true;  // 标记地图已初始化
        }
    }

    /**
     * @brief 核心路径规划函数
     *
     * 完整的规划流程：
     * 1. 使用采样法规划从起点到终点的初始路径
     * 2. 基于初始路径生成安全飞行走廊（凸多面体序列）
     * 3. 对安全飞行走廊进行短切优化
     * 4. 设置GCOPTER优化器的约束参数
     * 5. 执行轨迹优化得到时间最优且满足动力学约束的轨迹
     * 6. 可视化结果
     */
    inline void plan()
    {
        if (startGoal.size() == 2)  // 确保已设置起点和终点
        {
            // 步骤1: 使用基于采样的路径规划算法生成初始路径
            std::vector<Eigen::Vector3d> route;
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],      // 起点
                                                   startGoal[1],      // 终点
                                                   voxelMap.getOrigin(),  // 地图原点
                                                   voxelMap.getCorner(),  // 地图角点
                                                   &voxelMap, 0.01,   // 体素地图和分辨率
                                                   route);            // 输出路径

            // 步骤2: 生成安全飞行走廊（SFC）
            std::vector<Eigen::MatrixX4d> hPolys;  // 半平面表示的凸多面体序列
            std::vector<Eigen::Vector3d> pc;       // 点云数据
            voxelMap.getSurf(pc);                  // 获取地图表面点

            // 基于路径和障碍物点云生成凸包覆盖（安全飞行走廊）
            sfc_gen::convexCover(route,                    // 输入路径
                                 pc,                       // 障碍物点云
                                 voxelMap.getOrigin(),     // 地图原点
                                 voxelMap.getCorner(),     // 地图角点
                                 7.0,                      // 最大凸包边长
                                 3.0,                      // 最小凸包边长
                                 hPolys);                  // 输出凸多面体序列

            // 步骤3: 对安全飞行走廊进行短切优化，减少不必要的中间走廊
            sfc_gen::shortCut(hPolys);

            // 路径有效（至少包含起点和终点）
            if (route.size() > 1)
            {
                // 可视化生成的安全飞行走廊
                visualizer.visualizePolytope(hPolys);

                // 步骤4: 设置初始状态和终止状态
                // 每个状态包含：位置、速度、加速度（均为3维向量）
                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();  // 起点，零速度，零加速度
                finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();   // 终点，零速度，零加速度

                // 创建GCOPTER优化器对象
                gcopter::GCOPTER_PolytopeSFC gcopter;

                // 步骤5: 初始化优化器约束参数
                // magnitudeBounds = [最大速度, 最大角速度, 最大倾斜角, 最小推力, 最大推力]^T
                // penaltyWeights = [位置权重, 速度权重, 角速度权重, 倾斜角权重, 推力权重]^T
                // physicalParams = [车辆质量, 重力加速度, 水平阻力系数,
                //                   垂直阻力系数, 寄生阻力系数, 速度平滑因子]^T

                // 运动幅值边界约束
                Eigen::VectorXd magnitudeBounds(5);
                magnitudeBounds(0) = config.maxVelMag;      // 最大速度模值
                magnitudeBounds(1) = config.maxBdrMag;      // 最大机体角速度模值
                magnitudeBounds(2) = config.maxTiltAngle;   // 最大倾斜角
                magnitudeBounds(3) = config.minThrust;      // 最小推力
                magnitudeBounds(4) = config.maxThrust;      // 最大推力

                // 惩罚权重（用于软约束）
                Eigen::VectorXd penaltyWeights(5);
                penaltyWeights(0) = (config.chiVec)[0];     // 位置惩罚权重
                penaltyWeights(1) = (config.chiVec)[1];     // 速度惩罚权重
                penaltyWeights(2) = (config.chiVec)[2];     // 角速度惩罚权重
                penaltyWeights(3) = (config.chiVec)[3];     // 倾斜角惩罚权重
                penaltyWeights(4) = (config.chiVec)[4];     // 推力惩罚权重

                // 车辆物理参数
                Eigen::VectorXd physicalParams(6);
                physicalParams(0) = config.vehicleMass;     // 车辆质量
                physicalParams(1) = config.gravAcc;         // 重力加速度
                physicalParams(2) = config.horizDrag;       // 水平阻力系数
                physicalParams(3) = config.vertDrag;        // 垂直阻力系数
                physicalParams(4) = config.parasDrag;       // 寄生阻力系数
                physicalParams(5) = config.speedEps;        // 速度平滑因子

                const int quadratureRes = config.integralIntervs;  // 积分分辨率（数值积分的区间数）

                // 清空之前的轨迹
                traj.clear();

                // 步骤6: 设置GCOPTER优化问题
                if (!gcopter.setup(config.weightT,        // 时间权重
                                   iniState, finState,    // 初始和终止状态
                                   hPolys,                // 安全飞行走廊（凸多面体序列）
                                   INFINITY,              // 最大运行时间限制
                                   config.smoothingEps,   // 平滑参数
                                   quadratureRes,         // 积分分辨率
                                   magnitudeBounds,       // 幅值边界约束
                                   penaltyWeights,        // 惩罚权重
                                   physicalParams))       // 物理参数
                {
                    return;  // 设置失败，返回
                }

                // 步骤7: 执行优化求解
                if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
                {
                    return;  // 优化失败（代价为无穷大），返回
                }

                // 步骤8: 如果轨迹生成成功，保存时间戳并可视化
                if (traj.getPieceNum() > 0)
                {
                    trajStamp = ros::Time::now().toSec();  // 记录轨迹生成时间戳
                    visualizer.visualize(traj, route);     // 可视化轨迹和路径
                }
            }
        }
    }

    /**
     * @brief 目标点回调函数
     * @param msg PoseStamped类型的目标位姿消息
     *
     * 功能：
     * 1. 接收用户通过RViz设置的目标点
     * 2. 根据姿态的z分量计算目标点的高度
     * 3. 检查目标点是否在自由空间中
     * 4. 收集起点和终点后触发路径规划
     */
    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized)
        {
            // 如果已经有起点和终点，清空以接收新的目标点对
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }

            // 根据姿态的z分量计算目标点的高度
            // z分量的绝对值用于在地图z范围内插值高度
            const double zGoal = config.mapBound[4] + config.dilateRadius +
                                 fabs(msg->pose.orientation.z) *
                                     (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);

            // 构造3D目标点
            const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);

            // 检查目标点是否在自由空间中（未被占据）
            if (voxelMap.query(goal) == 0)
            {
                // 可视化目标点，大小为0.5，颜色根据是起点还是终点而定
                visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());
                startGoal.emplace_back(goal);  // 添加到起点终点列表
            }
            else
            {
                ROS_WARN("Infeasible Position Selected !!!\n");  // 目标点在障碍物中，不可行
            }

            // 当收集到起点和终点后，触发路径规划
            plan();
        }
        return;
    }

    /**
     * @brief 实时处理函数（主循环调用）
     *
     * 功能：
     * 1. 使用微分平坦性将轨迹转换为控制指令（推力、姿态、角速度）
     * 2. 发布实时的飞行状态数据用于监控和可视化
     * 3. 可视化当前位置
     *
     * 在1000Hz的频率下被主循环调用
     */
    inline void process()
    {
        // 配置物理参数用于微分平坦性映射
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;    // 车辆质量
        physicalParams(1) = config.gravAcc;        // 重力加速度
        physicalParams(2) = config.horizDrag;      // 水平阻力系数
        physicalParams(3) = config.vertDrag;       // 垂直阻力系数
        physicalParams(4) = config.parasDrag;      // 寄生阻力系数
        physicalParams(5) = config.speedEps;       // 速度平滑因子

        // 创建并初始化微分平坦性映射对象
        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));

        // 如果已生成轨迹
        if (traj.getPieceNum() > 0)
        {
            // 计算从轨迹开始到现在的时间差
            const double delta = ros::Time::now().toSec() - trajStamp;

            // 如果在轨迹有效时间范围内
            if (delta > 0.0 && delta < traj.getTotalDuration())
            {
                double thr;                  // 推力
                Eigen::Vector4d quat;        // 四元数姿态
                Eigen::Vector3d omg;         // 机体角速度

                // 通过微分平坦性正向映射：从轨迹状态（速度、加速度、急动度）
                // 映射到控制输入（推力、姿态、角速度）
                flatmap.forward(traj.getVel(delta),   // 当前速度
                                traj.getAcc(delta),   // 当前加速度
                                traj.getJer(delta),   // 当前急动度
                                0.0, 0.0,             // 偏航角和偏航角速度（设为0）
                                thr, quat, omg);      // 输出：推力、姿态、角速度

                // 计算飞行状态指标
                double speed = traj.getVel(delta).norm();          // 速度模值
                double bodyratemag = omg.norm();                   // 机体角速度模值
                // 从四元数计算倾斜角（俯仰和横滚的组合角度）
                double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));

                // 发布飞行状态数据用于实时监控和绘图
                std_msgs::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
                speedMsg.data = speed;           // 速度
                thrMsg.data = thr;               // 推力
                tiltMsg.data = tiltangle;        // 倾斜角
                bdrMsg.data = bodyratemag;       // 机体角速度
                visualizer.speedPub.publish(speedMsg);
                visualizer.thrPub.publish(thrMsg);
                visualizer.tiltPub.publish(tiltMsg);
                visualizer.bdrPub.publish(bdrMsg);

                // 可视化当前位置（用球体表示，半径为dilateRadius）
                visualizer.visualizeSphere(traj.getPos(delta),
                                           config.dilateRadius);
            }
        }
    }
};

/**
 * @brief 主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出码
 *
 * 功能：
 * 1. 初始化ROS节点
 * 2. 创建全局规划器对象并加载配置参数
 * 3. 以1000Hz的频率运行主循环
 * 4. 在主循环中处理轨迹跟踪和状态发布
 */
int main(int argc, char **argv)
{
    // 初始化ROS节点，节点名称为"global_planning_node"
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh_;

    // 创建全局规划器对象
    // Config(ros::NodeHandle("~")) 从私有命名空间加载参数
    GlobalPlanner global_planner(Config(ros::NodeHandle("~")), nh_);

    // 设置循环频率为1000Hz（1ms周期）
    ros::Rate lr(1000);
    while (ros::ok())
    {
        // 处理轨迹跟踪、状态计算和可视化
        global_planner.process();

        // 处理ROS回调函数（地图和目标点回调）
        ros::spinOnce();

        // 休眠以维持1000Hz的循环频率
        lr.sleep();
    }

    return 0;
}
