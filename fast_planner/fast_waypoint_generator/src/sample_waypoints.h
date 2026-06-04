/**
 * @file sample_waypoints.h
 * @brief 无人机航点生成器的示例轨迹定义文件
 *
 * 该文件提供了三种预定义的航点轨迹生成函数，用于无人机路径规划测试：
 * 1. point() - 生成折线型航点序列
 * 2. circle() - 生成圆形循环航点序列
 * 3. eight() - 生成8字形航点序列
 *
 * 所有函数返回nav_msgs::Path类型，包含位姿序列供路径规划器使用
 */

#ifndef SAMPLE_WAYPOINTS_H
#define SAMPLE_WAYPOINTS_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

/**
 * @brief 生成折线型航点序列
 *
 * 该函数创建一个沿着近似弧形的折线路径，包含8个航点。
 * 航点序列从右向左再向上形成一个弧形轨迹，适合测试基本的路径跟踪能力。
 *
 * 轨迹特点：
 * - 高度固定：1.0米
 * - 缩放因子：7.0（所有坐标乘以该因子）
 * - 航点数量：8个
 * - 轨迹形状：从(14,0)开始，经过弧形路径到达(0,7)
 *
 * @return nav_msgs::Path 包含8个位姿的航点路径
 */
nav_msgs::Path point()
{
    // 轨迹参数
    nav_msgs::Path waypoints;              // 航点路径容器
    geometry_msgs::PoseStamped pt;         // 单个航点位姿
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);  // 设置航向角为0（朝向x轴正方向）

    double h = 1.0;        // 飞行高度（米）
    double scale = 7.0;    // 坐标缩放因子

    // 航点1：起始点 (14.0, 0.0, 1.0)
    pt.pose.position.y =  scale * 0.0;
    pt.pose.position.x =  scale * 2.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点2：向右移动 (28.0, 0.0, 1.0)
    pt.pose.position.y =  scale * 0.0;
    pt.pose.position.x =  scale * 4.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点3：开始向上弯曲 (35.0, 1.75, 1.0)
    pt.pose.position.y =  scale * 0.25;
    pt.pose.position.x =  scale * 5.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点4：弧线顶点 (37.1, 3.5, 1.0)
    pt.pose.position.y =  scale * 0.5;
    pt.pose.position.x =  scale * 5.3;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点5：开始向左弯曲 (35.0, 5.25, 1.0)
    pt.pose.position.y =  scale * 0.75;
    pt.pose.position.x =  scale * 5.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点6：继续向左 (28.0, 7.0, 1.0)
    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 4.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点7：进一步向左 (14.0, 7.0, 1.0)
    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 2.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点8：终点 (0.0, 7.0, 1.0)
    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 0.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 返回航点路径
    return waypoints;
}

/**
 * @brief 生成圆形循环航点序列
 *
 * 该函数创建一个往返循环的三角形路径，包含13个航点。
 * 路径在两个三角形之间来回移动，形成连续的循环轨迹。
 *
 * 轨迹特点：
 * - 高度固定：1.0米
 * - 缩放因子：5.0
 * - 航点数量：13个
 * - 轨迹形状：两个三角形之间的往返路径
 * - 适用场景：测试循环路径跟踪和转向性能
 *
 * @return nav_msgs::Path 包含13个位姿的循环航点路径
 */
nav_msgs::Path circle()
{
    double h = 1.0;         // 飞行高度（米）
    double scale = 5.0;     // 坐标缩放因子
    nav_msgs::Path waypoints;              // 航点路径容器
    geometry_msgs::PoseStamped pt;         // 单个航点位姿
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);  // 设置航向角为0

    // 第一个三角形循环：右侧三角形
    // 航点1：中间点 (12.5, -6.0, 1.0)
    pt.pose.position.y = -1.2 * scale;
    pt.pose.position.x =  2.5 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点2：右下角 (25.0, -12.0, 1.0)
    pt.pose.position.y = -2.4 * scale;
    pt.pose.position.x =  5.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点3：右上角 (25.0, 0.0, 1.0)
    pt.pose.position.y =  0.0 * scale;
    pt.pose.position.x =  5.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点4：返回中间点 (12.5, -6.0, 1.0)
    pt.pose.position.y = -1.2 * scale;
    pt.pose.position.x =  2.5 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 第二个三角形循环：左侧三角形
    // 航点5：左下角 (0.0, -12.0, 1.0)
    pt.pose.position.y = -2.4 * scale;
    pt.pose.position.x =  0. * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点6：左上角（原点） (0.0, 0.0, 1.0)
    pt.pose.position.y =  0.0 * scale;
    pt.pose.position.x =  0.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点7：返回中间点 (12.5, -6.0, 1.0)
    pt.pose.position.y = -1.2 * scale;
    pt.pose.position.x =  2.5 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 重复循环：再次遍历右侧三角形
    // 航点8：右下角 (25.0, -12.0, 1.0)
    pt.pose.position.y = -2.4 * scale;
    pt.pose.position.x =  5.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点9：右上角 (25.0, 0.0, 1.0)
    pt.pose.position.y =  0.0 * scale;
    pt.pose.position.x =  5.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点10：返回中间点 (12.5, -6.0, 1.0)
    pt.pose.position.y = -1.2 * scale;
    pt.pose.position.x =  2.5 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 重复循环：再次遍历左侧三角形
    // 航点11：左下角 (0.0, -12.0, 1.0)
    pt.pose.position.y = -2.4 * scale;
    pt.pose.position.x =  0. * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 航点12：左上角（原点） (0.0, 0.0, 1.0)
    pt.pose.position.y =  0.0 * scale;
    pt.pose.position.x =  0.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);

    // 返回循环航点路径
    return waypoints;
}

/**
 * @brief 生成8字形（Figure-8）航点序列
 *
 * 该函数创建一个3D立体8字形轨迹，包含高度变化的环形路径。
 * 轨迹由两个相连的环形组成，在空间中形成立体8字形状。
 *
 * 轨迹特点：
 * - 基准高度：2.0米
 * - 高度变化：在1.0米到2.0米之间变化
 * - 环形半径：10.0米
 * - 航点数量：16个（每个循环8个航点）
 * - 轨迹形状：两个竖直的环形连接成8字形
 * - 适用场景：测试复杂的3D机动能力和高度变化跟踪
 *
 * @return nav_msgs::Path 包含16个位姿的8字形航点路径
 */
nav_msgs::Path eight()
{
    // 8字形轨迹参数
    double offset_x = 0.0;  // X方向偏移量（用于扩展轨迹，当前未使用）
    double offset_y = 0.0;  // Y方向偏移量（用于扩展轨迹，当前未使用）
    double r = 10.0;        // 环形半径（米）
    double h = 2.0;         // 基准飞行高度（米）
    nav_msgs::Path waypoints;              // 航点路径容器
    geometry_msgs::PoseStamped pt;         // 单个航点位姿
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);  // 设置航向角为0

    for(int i=0; i< 1; ++i)  // 循环次数（当前为1次完整的8字形）
    {
        // === 第一个环形（右侧环） ===
        // 航点1：右环下部 (10.0, -10.0, 1.0) - 高度降低
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y = -r + offset_y;
        pt.pose.position.z =  h/2;
        waypoints.poses.push_back(pt);

        // 航点2：右环中点 (20.0, 0.0, 2.0) - 恢复到基准高度
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);

        // 航点3：右环上部 (30.0, 10.0, 1.0) - 高度降低
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y =  r ;
        pt.pose.position.z =  h/2;
        waypoints.poses.push_back(pt);

        // 航点4：右环顶点 (40.0, 0.0, 2.0) - 恢复到基准高度
        pt.pose.position.x =  r*4 + offset_x * 4;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);

        // 航点5：右环下部回程 (30.0, -10.0, 1.0) - 高度降低
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y = -r ;
        pt.pose.position.z =  h/2;
        waypoints.poses.push_back(pt);

        // 航点6：右环中点回程 (20.0, 0.0, 2.0) - 恢复到基准高度
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);

        // 航点7：左环上部 (10.0, 10.0, 1.0) - 高度降低，过渡到左环
        pt.pose.position.x =  r + offset_x * 2;
        pt.pose.position.y =  r ;
        pt.pose.position.z =  h/2;
        waypoints.poses.push_back(pt);

        // 航点8：8字形交叉点 (0.0, 0.0, 2.0) - 恢复到基准高度
        pt.pose.position.x =  0  + offset_x;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);

        // === 第二个环形（左侧环，高度变化更大） ===
        // 航点9：左环下部 (10.0, -10.0, 3.0) - 高度升高
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y = -r;
        pt.pose.position.z =  h / 2 * 3;
        waypoints.poses.push_back(pt);

        // 航点10：左环中点 (20.0, 0.0, 2.0) - 降到基准高度
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);

        // 航点11：左环上部 (30.0, 10.0, 3.0) - 高度升高
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y =  r;
        pt.pose.position.z =  h / 2 * 3;
        waypoints.poses.push_back(pt);

        // 航点12：左环顶点 (40.0, 0.0, 2.0) - 降到基准高度
        pt.pose.position.x =  r*4 + offset_x * 4;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);

        // 航点13：左环下部回程 (30.0, -10.0, 3.0) - 高度升高
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y = -r;
        pt.pose.position.z =  h / 2 * 3;
        waypoints.poses.push_back(pt);

        // 航点14：左环中点回程 (20.0, 0.0, 2.0) - 降到基准高度
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);

        // 航点15：返回上部 (10.0, 10.0, 3.0) - 高度升高
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y =  r + offset_y;
        pt.pose.position.z =  h / 2 * 3;
        waypoints.poses.push_back(pt);

        // 航点16：回到8字形交叉点 (0.0, 0.0, 2.0) - 结束点
        pt.pose.position.x =  0;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);
    }
    // 返回8字形航点路径
    return waypoints;
}
#endif