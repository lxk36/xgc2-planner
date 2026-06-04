#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

// GCOPTER轨迹表示和几何工具
#include "gcopter/trajectory.hpp"
#include "gcopter/quickhull.hpp"
#include "gcopter/geo_utils.hpp"

// 标准库头文件
#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>

// ROS相关头文件
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/**
 * @brief 轨迹规划器可视化类
 *
 * 该类负责将轨迹规划的各种元素在RViz中进行可视化显示，包括：
 * - 前端规划路径
 * - 轨迹路径点
 * - 优化后的完整轨迹
 * - 自由空间多面体（corridor）
 * - 安全半径球体
 * - 飞行器状态参数（速度、推力、倾斜角、机体角速率）
 */
class Visualizer
{
private:
    // ROS节点句柄，用于创建发布器和管理ROS相关操作
    ros::NodeHandle nh;

    // 各类可视化元素的ROS发布器
    ros::Publisher routePub;        // 前端规划路径发布器（路径搜索算法得到的初始路径）
    ros::Publisher wayPointsPub;    // 轨迹路径点发布器（优化后轨迹的关键控制点）
    ros::Publisher trajectoryPub;   // 完整轨迹发布器（密集采样的轨迹点）
    ros::Publisher meshPub;         // 自由空间多面体网格发布器（corridor的三角网格面）
    ros::Publisher edgePub;         // 多面体边缘发布器（corridor的边界线）
    ros::Publisher spherePub;       // 安全球体发布器（起点、终点或安全半径球）

public:
    // 飞行器动态参数发布器（用于实时监控和调试）
    ros::Publisher speedPub;        // 速度发布器（飞行器速度大小）
    ros::Publisher thrPub;          // 总推力发布器（飞行器所需总推力）
    ros::Publisher tiltPub;         // 倾斜角发布器（飞行器机体倾斜角度）
    ros::Publisher bdrPub;          // 机体角速率发布器（飞行器旋转角速度）

public:
    /**
     * @brief 可视化器构造函数
     * @param nh_ ROS节点句柄引用
     *
     * 初始化所有ROS发布器，设置各自的话题名称和队列大小
     * 话题名称采用统一的命名空间 "/visualizer/" 前缀
     */
    Visualizer(ros::NodeHandle &nh_)
        : nh(nh_)
    {
        // 初始化各类可视化发布器
        routePub = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
        wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);
        trajectoryPub = nh.advertise<visualization_msgs::Marker>("/visualizer/trajectory", 10);
        meshPub = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
        edgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);
        spherePub = nh.advertise<visualization_msgs::Marker>("/visualizer/spheres", 1000);

        // 初始化飞行器状态参数发布器
        speedPub = nh.advertise<std_msgs::Float64>("/visualizer/speed", 1000);
        thrPub = nh.advertise<std_msgs::Float64>("/visualizer/total_thrust", 1000);
        tiltPub = nh.advertise<std_msgs::Float64>("/visualizer/tilt_angle", 1000);
        bdrPub = nh.advertise<std_msgs::Float64>("/visualizer/body_rate", 1000);
    }

    /**
     * @brief 可视化轨迹和前端规划路径
     * @tparam D 轨迹的维度（通常为3）
     * @param traj 优化后的轨迹对象
     * @param route 前端路径搜索算法得到的初始路径点集合
     *
     * 该函数在RViz中同时显示三种元素：
     * 1. 前端规划路径（红色线段）- 路径搜索算法的输出
     * 2. 轨迹路径点（红色球体）- 优化后轨迹的关键控制点
     * 3. 完整轨迹（蓝色线段）- 密集采样的轨迹曲线
     */
    template <int D>
    inline void visualize(const Trajectory<D> &traj,
                          const std::vector<Eigen::Vector3d> &route)
    {
        // 创建三种可视化标记
        visualization_msgs::Marker routeMarker, wayPointsMarker, trajMarker;

        // 配置前端路径标记（红色线段）
        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;  // 线段列表类型
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = "odom";  // 里程计坐标系
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";  // 命名空间
        routeMarker.color.r = 1.00;  // 红色
        routeMarker.color.g = 0.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;  // 不透明
        routeMarker.scale.x = 0.1;   // 线宽

        // 配置路径点标记（红色球体）
        wayPointsMarker = routeMarker;  // 复制基础配置
        wayPointsMarker.id = -wayPointsMarker.id - 1;  // 不同的ID
        wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;  // 球体列表类型
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.00;  // 红色
        wayPointsMarker.color.g = 0.00;
        wayPointsMarker.color.b = 0.00;
        wayPointsMarker.scale.x = 0.35;  // 球体直径
        wayPointsMarker.scale.y = 0.35;
        wayPointsMarker.scale.z = 0.35;

        // 配置完整轨迹标记（蓝色线段）
        trajMarker = routeMarker;  // 复制基础配置
        trajMarker.header.frame_id = "odom";
        trajMarker.id = 0;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.00;  // 蓝色
        trajMarker.color.g = 0.50;
        trajMarker.color.b = 1.00;
        trajMarker.scale.x = 0.30;  // 线宽

        // 可视化前端规划路径（如果存在）
        if (route.size() > 0)
        {
            bool first = true;
            Eigen::Vector3d last;
            // 遍历路径点，创建连续的线段
            for (auto it : route)
            {
                if (first)
                {
                    // 第一个点仅记录，不绘制
                    first = false;
                    last = it;
                    continue;
                }
                geometry_msgs::Point point;

                // 添加线段的起点（上一个路径点）
                point.x = last(0);
                point.y = last(1);
                point.z = last(2);
                routeMarker.points.push_back(point);

                // 添加线段的终点（当前路径点）
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                routeMarker.points.push_back(point);

                last = it;
            }

            // 发布前端路径标记
            routePub.publish(routeMarker);
        }

        // 可视化轨迹路径点（如果轨迹存在）
        if (traj.getPieceNum() > 0)
        {
            // 获取轨迹的所有位置控制点
            Eigen::MatrixXd wps = traj.getPositions();

            // 遍历所有路径点，创建球体标记
            for (int i = 0; i < wps.cols(); i++)
            {
                geometry_msgs::Point point;
                point.x = wps.col(i)(0);
                point.y = wps.col(i)(1);
                point.z = wps.col(i)(2);
                wayPointsMarker.points.push_back(point);
            }

            // 发布路径点标记
            wayPointsPub.publish(wayPointsMarker);
        }

        // 可视化完整轨迹曲线（如果轨迹存在）
        if (traj.getPieceNum() > 0)
        {
            // 时间采样间隔，0.01秒（100Hz）
            double T = 0.01;
            Eigen::Vector3d lastX = traj.getPos(0.0);

            // 沿轨迹进行时间采样，生成密集的轨迹点
            for (double t = T; t < traj.getTotalDuration(); t += T)
            {
                geometry_msgs::Point point;
                Eigen::Vector3d X = traj.getPos(t);

                // 添加线段的起点（上一个采样点）
                point.x = lastX(0);
                point.y = lastX(1);
                point.z = lastX(2);
                trajMarker.points.push_back(point);

                // 添加线段的终点（当前采样点）
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                trajMarker.points.push_back(point);

                lastX = X;
            }

            // 发布轨迹标记
            trajectoryPub.publish(trajMarker);
        }
    }

    /**
     * @brief 可视化自由空间多面体（corridor）
     * @param hPolys H-表示的多面体向量，每个多面体由半平面约束矩阵表示
     *
     * 该函数将H-表示的多面体转换为可视化的三角网格：
     * 1. 对每个H-表示多面体进行顶点枚举，得到V-表示
     * 2. 使用QuickHull算法计算凸包，生成三角网格
     * 3. 在RViz中显示网格面和边界线
     *
     * 注意：H-表示（半平面交集）无法直接可视化，需要转换为V-表示（顶点集合）
     */
    inline void visualizePolytope(const std::vector<Eigen::MatrixX4d> &hPolys)
    {
        // 存储所有多面体的三角网格
        // mesh: 最终的网格顶点集合
        // curTris: 当前多面体的三角网格
        // oldTris: 已处理的三角网格
        Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);

        // 遍历每个多面体，生成三角网格
        for (size_t id = 0; id < hPolys.size(); id++)
        {
            oldTris = mesh;  // 保存已有的网格

            // 步骤1: 从H-表示枚举顶点，得到V-表示
            Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
            geo_utils::enumerateVs(hPolys[id], vPoly);

            // 步骤2: 使用QuickHull算法计算凸包，生成三角网格
            quickhull::QuickHull<double> tinyQH;
            const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
            const auto &idxBuffer = polyHull.getIndexBuffer();  // 三角形顶点索引
            int hNum = idxBuffer.size() / 3;  // 三角形数量

            // 步骤3: 根据索引提取三角形顶点
            curTris.resize(3, hNum * 3);
            for (int i = 0; i < hNum * 3; i++)
            {
                curTris.col(i) = vPoly.col(idxBuffer[i]);
            }

            // 步骤4: 合并当前多面体的网格到总网格中
            mesh.resize(3, oldTris.cols() + curTris.cols());
            mesh.leftCols(oldTris.cols()) = oldTris;      // 已有网格
            mesh.rightCols(curTris.cols()) = curTris;     // 新网格
        }

        // 创建RViz可视化标记（三角网格和边界线）
        visualization_msgs::Marker meshMarker, edgeMarker;

        // 配置三角网格标记（蓝色半透明）
        meshMarker.id = 0;
        meshMarker.header.stamp = ros::Time::now();
        meshMarker.header.frame_id = "odom";
        meshMarker.pose.orientation.w = 1.00;
        meshMarker.action = visualization_msgs::Marker::ADD;
        meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;  // 三角形列表类型
        meshMarker.ns = "mesh";
        meshMarker.color.r = 0.00;
        meshMarker.color.g = 0.00;
        meshMarker.color.b = 1.00;  // 蓝色
        meshMarker.color.a = 0.15;  // 半透明（15%不透明度）
        meshMarker.scale.x = 1.0;
        meshMarker.scale.y = 1.0;
        meshMarker.scale.z = 1.0;

        // 配置边界线标记（青色）
        edgeMarker = meshMarker;
        edgeMarker.type = visualization_msgs::Marker::LINE_LIST;  // 线段列表类型
        edgeMarker.ns = "edge";
        edgeMarker.color.r = 0.00;
        edgeMarker.color.g = 1.00;  // 青色
        edgeMarker.color.b = 1.00;
        edgeMarker.color.a = 1.00;  // 不透明
        edgeMarker.scale.x = 0.02;  // 线宽

        geometry_msgs::Point point;

        int ptnum = mesh.cols();  // 网格顶点总数

        // 添加所有三角形顶点到网格标记
        for (int i = 0; i < ptnum; i++)
        {
            point.x = mesh(0, i);
            point.y = mesh(1, i);
            point.z = mesh(2, i);
            meshMarker.points.push_back(point);
        }

        // 为每个三角形生成边界线
        // 每个三角形有3条边，每条边由2个顶点定义
        for (int i = 0; i < ptnum / 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                // 当前边的第一个顶点
                point.x = mesh(0, 3 * i + j);
                point.y = mesh(1, 3 * i + j);
                point.z = mesh(2, 3 * i + j);
                edgeMarker.points.push_back(point);

                // 当前边的第二个顶点（下一个顶点，循环回第一个）
                point.x = mesh(0, 3 * i + (j + 1) % 3);
                point.y = mesh(1, 3 * i + (j + 1) % 3);
                point.z = mesh(2, 3 * i + (j + 1) % 3);
                edgeMarker.points.push_back(point);
            }
        }

        // 发布多面体网格和边界
        meshPub.publish(meshMarker);
        edgePub.publish(edgeMarker);

        return;
    }

    /**
     * @brief 可视化单个球体
     * @param center 球体中心位置
     * @param radius 球体半径
     *
     * 该函数在RViz中显示一个蓝色球体，通常用于表示安全半径
     * 先删除旧的球体标记，再添加新的球体
     */
    inline void visualizeSphere(const Eigen::Vector3d &center,
                                const double &radius)
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        // 配置球体标记（蓝色）
        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;  // 球体列表类型
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 0.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 1.00;  // 蓝色
        sphereMarkers.color.a = 1.00;  // 不透明
        sphereMarkers.scale.x = radius * 2.0;  // 直径 = 半径 × 2
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        // 创建删除标记，用于清除旧球体
        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETE;

        // 设置球体中心位置
        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        // 先删除旧球体，再显示新球体
        spherePub.publish(sphereDeleter);
        spherePub.publish(sphereMarkers);
    }

    /**
     * @brief 可视化起点或终点球体
     * @param center 球体中心位置
     * @param radius 球体半径
     * @param sg 标识符，0表示起点，非0表示终点
     *
     * 该函数在RViz中显示红色球体标记起点或终点：
     * - sg=0时：清除所有旧标记，然后显示起点
     * - sg≠0时：仅添加终点标记
     * 使用不同的ID区分起点和终点，便于单独管理
     */
    inline void visualizeStartGoal(const Eigen::Vector3d &center,
                                   const double &radius,
                                   const int sg)
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        // 配置起点/终点球体标记（红色）
        sphereMarkers.id = sg;  // 使用sg作为标记ID
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;  // 球体列表类型
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "StartGoal";  // 起点终点命名空间
        sphereMarkers.color.r = 1.00;  // 红色
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 0.00;
        sphereMarkers.color.a = 1.00;  // 不透明
        sphereMarkers.scale.x = radius * 2.0;  // 直径 = 半径 × 2
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        // 创建删除所有标记的消息
        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETEALL;

        // 设置球体中心位置
        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        // 如果是起点（sg=0），先清除所有旧的起点终点标记
        if (sg == 0)
        {
            spherePub.publish(sphereDeleter);  // 删除所有旧标记
            ros::Duration(1.0e-9).sleep();     // 短暂延迟确保删除完成
            sphereMarkers.header.stamp = ros::Time::now();  // 更新时间戳
        }

        // 发布新的起点或终点标记
        spherePub.publish(sphereMarkers);
    }
};

#endif