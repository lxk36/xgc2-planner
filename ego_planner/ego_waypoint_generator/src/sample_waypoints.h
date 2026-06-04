/**
 * @file sample_waypoints.h
 * @brief 航点轨迹生成器 - 预定义轨迹模板
 *
 * 功能说明：
 * 本文件提供了一系列预定义的航点轨迹生成函数，用于无人机路径规划的测试和演示。
 * 主要包含三种标准轨迹模式：
 * 1. point() - 不规则路径轨迹，用于测试路径跟随能力
 * 2. circle() - 圆形轨迹，用于测试循环飞行和稳定性
 * 3. eight() - 8字形轨迹，用于测试复杂机动和动态性能
 *
 * 轨迹特点：
 * - 所有航点都包含完整的位姿信息（位置+方向）
 * - 使用ROS标准的nav_msgs::Path消息类型
 * - 轨迹通过缩放参数可调节尺寸
 * - 适用于EGO-Planner等轨迹规划器的输入
 *
 * 使用场景：
 * - 仿真环境中的无人机轨迹测试
 * - 轨迹跟踪算法的性能评估
 * - 运动规划算法的基准测试
 */

#ifndef SAMPLE_WAYPOINTS_H
#define SAMPLE_WAYPOINTS_H

#include <ros/ros.h>           // ROS核心功能
#include <tf/tf.h>              // TF坐标变换库，用于四元数转换
#include <nav_msgs/Path.h>      // ROS路径消息类型

/**
 * @brief 生成不规则路径轨迹
 *
 * 功能：生成一条包含8个航点的不规则路径，用于测试无人机的路径跟随能力
 *
 * 轨迹描述：
 * - 轨迹形状：从左向右的弯曲路径，包含转弯和直线段
 * - 航点数量：8个关键点
 * - 飞行高度：恒定高度（h = 1.0m）
 * - 轨迹范围：X方向 0~35m，Y方向 0~7m
 *
 * 坐标系说明：
 * - X轴：向前（主要飞行方向）
 * - Y轴：向左
 * - Z轴：向上（高度）
 *
 * @return nav_msgs::Path 包含所有航点的路径消息
 */
nav_msgs::Path point()
{
    // 初始化路径消息和位姿变量
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);  // 设置偏航角为0（朝向正前方）

    double h = 1.0;      // 飞行高度（米）
    double scale = 7.0;  // 轨迹缩放因子，用于调整轨迹尺寸

    // 航点1：起始点 (14.0, 0.0, 1.0)
    pt.pose.position.y =  scale * 0.0;
    pt.pose.position.x =  scale * 2.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点2：直线前进 (28.0, 0.0, 1.0)
    pt.pose.position.y =  scale * 0.0;
    pt.pose.position.x =  scale * 4.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点3：开始向左转弯 (35.0, 1.75, 1.0)
    pt.pose.position.y =  scale * 0.25;
    pt.pose.position.x =  scale * 5.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点4：轨迹最右侧点 (37.1, 3.5, 1.0)
    pt.pose.position.y =  scale * 0.5;
    pt.pose.position.x =  scale * 5.3;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点5：转弯中点 (35.0, 5.25, 1.0)
    pt.pose.position.y =  scale * 0.75;
    pt.pose.position.x =  scale * 5.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点6：完成转弯，向后飞行 (28.0, 7.0, 1.0)
    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 4.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点7：继续向后 (14.0, 7.0, 1.0)
    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 2.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点8：终点 (0.0, 7.0, 1.0)
    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 0.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 返回完整的航点路径
    return waypoints;
}

/**
 * @brief 生成圆形循环轨迹
 *
 * 功能：生成一条重复的闭合圆形轨迹，用于测试无人机的循环飞行和稳定性
 *
 * 轨迹描述：
 * - 轨迹形状：类似三角形的闭合循环路径，重复2次
 * - 航点数量：13个关键点（包含重复点）
 * - 飞行高度：恒定高度（h = 1.0m）
 * - 轨迹范围：X方向 0~25m，Y方向 -12~0m
 *
 * 轨迹特点：
 * - 形成一个闭环，起点和终点重合
 * - 包含3个主要转折点，形成三角形飞行路径
 * - 轨迹重复两次，用于测试长时间循环飞行
 * - 适合测试轨迹跟踪的稳定性和重复性
 *
 * @return nav_msgs::Path 包含所有航点的循环路径消息
 */
nav_msgs::Path circle()
{
    double h = 1.0;      // 飞行高度（米）
    double scale = 5.0;  // 轨迹缩放因子
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);  // 设置偏航角为0

    // ========== 第一次循环 ==========
    // 航点1：循环起始点/中心点 (12.5, -6.0, 1.0)
    pt.pose.position.y = -1.2 * scale;
    pt.pose.position.x =  2.5 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点2：三角形第一个顶点（右下） (25.0, -12.0, 1.0)
    pt.pose.position.y = -2.4 * scale;
    pt.pose.position.x =  5.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点3：三角形第二个顶点（右上） (25.0, 0.0, 1.0)
    pt.pose.position.y =  0.0 * scale;
    pt.pose.position.x =  5.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点4：返回中心点 (12.5, -6.0, 1.0)
    pt.pose.position.y = -1.2 * scale;
    pt.pose.position.x =  2.5 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点5：三角形第三个顶点（左下） (0.0, -12.0, 1.0)
    pt.pose.position.y = -2.4 * scale;
    pt.pose.position.x =  0. * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点6：三角形第四个顶点（左上） (0.0, 0.0, 1.0)
    pt.pose.position.y =  0.0 * scale;
    pt.pose.position.x =  0.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // ========== 第二次循环（重复相同路径） ==========
    // 航点7：返回中心点 (12.5, -6.0, 1.0)
    pt.pose.position.y = -1.2 * scale;
    pt.pose.position.x =  2.5 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点8：右下顶点 (25.0, -12.0, 1.0)
    pt.pose.position.y = -2.4 * scale;
    pt.pose.position.x =  5.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点9：右上顶点 (25.0, 0.0, 1.0)
    pt.pose.position.y =  0.0 * scale;
    pt.pose.position.x =  5.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点10：中心点 (12.5, -6.0, 1.0)
    pt.pose.position.y = -1.2 * scale;
    pt.pose.position.x =  2.5 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点11：左下顶点 (0.0, -12.0, 1.0)
    pt.pose.position.y = -2.4 * scale;
    pt.pose.position.x =  0. * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点12：左上顶点（终点） (0.0, 0.0, 1.0)
    pt.pose.position.y =  0.0 * scale;
    pt.pose.position.x =  0.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 返回完整的循环路径
    return waypoints;
}

/**
 * @brief 生成8字形轨迹
 *
 * 功能：生成一个标准的8字形（∞形）飞行轨迹，用于测试无人机的复杂机动能力
 *
 * 轨迹描述：
 * - 轨迹形状：水平放置的8字形（无穷大符号∞）
 * - 由两个对称的圆环组成，在中心点相交
 * - 航点数量：16个关键点（每个循环8个点）
 * - 飞行高度：在h/2到h之间变化（1.0m ~ 2.0m）
 * - 轨迹范围：X方向 0~40m，Y方向 -10~10m
 *
 * 轨迹特点：
 * - 左右两个圆环形成对称的8字形
 * - 高度随轨迹变化，形成3D立体轨迹
 * - 在两个环的连接处（中心点）高度最高
 * - 在每个环的左右两端高度较低
 * - 适合测试无人机的敏捷性和动态跟踪能力
 *
 * 参数说明：
 * - r: 每个圆环的半径
 * - h: 最大飞行高度
 * - offset_x/y: 轨迹的偏移量（当前为0，轨迹以原点为基准）
 *
 * @return nav_msgs::Path 包含所有航点的8字形路径消息
 */
nav_msgs::Path eight()
{
    // 轨迹参数定义
    double offset_x = 0.0;  // X方向偏移量（米）
    double offset_y = 0.0;  // Y方向偏移量（米）
    double r = 10.0;        // 圆环半径（米）
    double h = 2.0;         // 最大飞行高度（米）
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);  // 设置偏航角为0

    // 循环控制：当前设置为1次完整的8字形轨迹
    for(int i=0; i< 1; ++i)
    {
        // ========== 第一个循环：右侧圆环（顺时针） ==========
        // 航点1：右环下端点 (10.0, -10.0, 1.0)
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y = -r + offset_y;
        pt.pose.position.z =  h/2;
        waypoints.poses.push_back(pt);

        // 航点2：右环与左环连接点（8字中心） (20.0, 0.0, 2.0) - 最高点
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);

        // 航点3：右环上端点 (30.0, 10.0, 1.0)
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y =  r ;
        pt.pose.position.z =  h/2;
        waypoints.poses.push_back(pt);

        // 航点4：右环最右侧点 (40.0, 0.0, 2.0) - 最高点
        pt.pose.position.x =  r*4 + offset_x * 4;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);

        // 航点5：右环下端点（返回） (30.0, -10.0, 1.0)
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y = -r ;
        pt.pose.position.z =  h/2;
        waypoints.poses.push_back(pt);

        // 航点6：回到8字中心 (20.0, 0.0, 2.0) - 最高点
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);

        // ========== 切换到左侧圆环（逆时针） ==========
        // 航点7：左环上端点 (10.0, 10.0, 1.0)
        pt.pose.position.x =  r + offset_x * 2;
        pt.pose.position.y =  r ;
        pt.pose.position.z =  h/2;
        waypoints.poses.push_back(pt);

        // 航点8：左环最左侧点 (0.0, 0.0, 2.0) - 最高点
        pt.pose.position.x =  0  + offset_x;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);

        // ========== 第二个循环：右侧圆环（重复，高度稍高） ==========
        // 航点9：右环下端点 (10.0, -10.0, 3.0) - 高度变化
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y = -r;
        pt.pose.position.z =  h / 2 * 3;
        waypoints.poses.push_back(pt);

        // 航点10：8字中心 (20.0, 0.0, 2.0)
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);

        // 航点11：右环上端点 (30.0, 10.0, 3.0) - 高度变化
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y =  r;
        pt.pose.position.z =  h / 2 * 3;
        waypoints.poses.push_back(pt);

        // 航点12：右环最右侧 (40.0, 0.0, 2.0)
        pt.pose.position.x =  r*4 + offset_x * 4;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);

        // 航点13：右环下端点 (30.0, -10.0, 3.0) - 高度变化
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y = -r;
        pt.pose.position.z =  h / 2 * 3;
        waypoints.poses.push_back(pt);

        // 航点14：8字中心 (20.0, 0.0, 2.0)
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);

        // ========== 第二个循环：左侧圆环 ==========
        // 航点15：左环上端点 (10.0, 10.0, 3.0) - 高度变化
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y =  r + offset_y;
        pt.pose.position.z =  h / 2 * 3;
        waypoints.poses.push_back(pt);

        // 航点16：左环最左侧（终点） (0.0, 0.0, 2.0)
        pt.pose.position.x =  0;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);
    }

    // 返回完整的8字形轨迹
    return waypoints;
}
#endif  // SAMPLE_WAYPOINTS_H