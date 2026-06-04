/**
 * @file waypoint_generator.cpp
 * @brief 航点生成器节点 - 用于生成和发布无人机导航航点
 *
 * 该节点支持多种航点生成模式：
 * - manual: 手动模式，通过RViz交互式设置航点
 * - circle: 圆形轨迹
 * - eight: 8字形轨迹
 * - point: 点轨迹
 * - series: 序列航点（从参数服务器加载）
 * - manual-lonely-waypoint: 单个独立航点模式
 * - noyaw: 保持当前偏航角的手动模式
 */

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include "sample_waypoints.h"
#include <vector>
#include <deque>
#include <boost/format.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using bfmt = boost::format;

// ROS发布器定义
ros::Publisher pub1;  // 航点路径发布器
ros::Publisher pub2;  // 航点可视化发布器
ros::Publisher pub3;  // 保留的发布器

// 全局变量
string waypoint_type = string("manual");  // 航点类型：manual/circle/eight/point/series等
bool is_odom_ready;                        // 里程计数据就绪标志
nav_msgs::Odometry odom;                   // 当前里程计信息
nav_msgs::Path waypoints;                  // 当前航点路径

// 序列航点相关变量
std::deque<nav_msgs::Path> waypointSegments;  // 航点段队列，用于存储多段轨迹
ros::Time trigged_time;                        // 触发时间戳

/**
 * @brief 加载单个航点段
 * @param nh ROS节点句柄
 * @param segid 段ID编号
 * @param time_base 时间基准，用于计算绝对时间戳
 *
 * 功能说明：
 * 1. 从参数服务器读取指定ID的航点段配置
 * 2. 将相对坐标转换为世界坐标系
 * 3. 应用偏航角旋转变换
 * 4. 将航点段添加到队列中
 */
void load_seg(ros::NodeHandle& nh, int segid, const ros::Time& time_base) {
    // 构造参数路径前缀，例如 "seg0/"
    std::string seg_str = boost::str(bfmt("seg%d/") % segid);
    double yaw;                // 偏航角
    double time_from_start;    // 相对起始时间

    ROS_INFO("Getting segment %d", segid);

    // 从参数服务器读取航点段配置
    ROS_ASSERT(nh.getParam(seg_str + "yaw", yaw));
    ROS_ASSERT_MSG((yaw > -3.1499999) && (yaw < 3.14999999), "yaw=%.3f", yaw);  // 确保yaw在[-π, π]范围内
    ROS_ASSERT(nh.getParam(seg_str + "time_from_start", time_from_start));
    ROS_ASSERT(time_from_start >= 0.0);  // 时间必须非负

    // 读取航点坐标数组
    std::vector<double> ptx;  // X坐标数组
    std::vector<double> pty;  // Y坐标数组
    std::vector<double> ptz;  // Z坐标数组

    ROS_ASSERT(nh.getParam(seg_str + "x", ptx));
    ROS_ASSERT(nh.getParam(seg_str + "y", pty));
    ROS_ASSERT(nh.getParam(seg_str + "z", ptz));

    // 验证数据有效性：所有坐标数组必须等长且非空
    ROS_ASSERT(ptx.size());
    ROS_ASSERT(ptx.size() == pty.size() && ptx.size() == ptz.size());

    nav_msgs::Path path_msg;

    // 设置时间戳：基准时间 + 相对时间
    path_msg.header.stamp = time_base + ros::Duration(time_from_start);

    // 获取当前无人机的偏航角（基准偏航角）
    double baseyaw = tf::getYaw(odom.pose.pose.orientation);

    // 遍历所有航点，进行坐标变换
    for (size_t k = 0; k < ptx.size(); ++k) {
        geometry_msgs::PoseStamped pt;

        // 设置航点的朝向：基准偏航角 + 相对偏航角
        pt.pose.orientation = tf::createQuaternionMsgFromYaw(baseyaw + yaw);

        // 坐标变换：将相对坐标转换为世界坐标
        Eigen::Vector2d dp(ptx.at(k), pty.at(k));  // 相对坐标
        Eigen::Vector2d rdp;                        // 旋转后的坐标

        // 应用旋转矩阵：R(-baseyaw-yaw) * [x, y]
        rdp.x() = std::cos(-baseyaw-yaw)*dp.x() + std::sin(-baseyaw-yaw)*dp.y();
        rdp.y() =-std::sin(-baseyaw-yaw)*dp.x() + std::cos(-baseyaw-yaw)*dp.y();

        // 加上当前位置偏移，得到世界坐标系下的绝对位置
        pt.pose.position.x = rdp.x() + odom.pose.pose.position.x;
        pt.pose.position.y = rdp.y() + odom.pose.pose.position.y;
        pt.pose.position.z = ptz.at(k) + odom.pose.pose.position.z;

        path_msg.poses.push_back(pt);
    }

    // 将构造好的航点段加入队列
    waypointSegments.push_back(path_msg);
}

/**
 * @brief 加载所有航点段
 * @param nh ROS节点句柄
 * @param time_base 时间基准
 *
 * 功能说明：
 * 1. 从参数服务器读取航点段总数
 * 2. 依次加载每个航点段
 * 3. 验证各段时间戳的单调性（后续段的时间戳必须大于前面的段）
 */
void load_waypoints(ros::NodeHandle& nh, const ros::Time& time_base) {
    int seg_cnt = 0;
    waypointSegments.clear();  // 清空之前的航点段

    // 读取航点段数量
    ROS_ASSERT(nh.getParam("segment_cnt", seg_cnt));

    // 依次加载每个航点段
    for (int i = 0; i < seg_cnt; ++i) {
        load_seg(nh, i, time_base);
        // 确保时间戳严格递增
        if (i > 0) {
            ROS_ASSERT(waypointSegments[i - 1].header.stamp < waypointSegments[i].header.stamp);
        }
    }
    ROS_INFO("Overall load %zu segments", waypointSegments.size());
}

/**
 * @brief 发布航点路径
 *
 * 功能说明：
 * 1. 设置航点路径的frame_id和时间戳
 * 2. 将当前位置作为起始点插入航点序列
 * 3. 发布航点路径消息
 * 4. 清空航点缓存
 */
void publish_waypoints() {
    // 设置航点路径的坐标系为世界坐标系
    waypoints.header.frame_id = std::string("world");
    waypoints.header.stamp = ros::Time::now();

    // 发布航点路径消息
    pub1.publish(waypoints);

    // 将当前位置作为起始点插入到航点序列开头
    geometry_msgs::PoseStamped init_pose;
    init_pose.header = odom.header;
    init_pose.pose = odom.pose.pose;
    waypoints.poses.insert(waypoints.poses.begin(), init_pose);

    // pub2.publish(waypoints);  // 保留的备用发布代码

    // 清空航点缓存，准备接收下一批航点
    waypoints.poses.clear();
}

/**
 * @brief 发布航点可视化消息
 *
 * 功能说明：
 * 1. 构造PoseArray消息用于RViz可视化
 * 2. 将当前位置和所有航点添加到可视化数组
 * 3. 发布可视化消息供RViz显示
 */
void publish_waypoints_vis() {
    nav_msgs::Path wp_vis = waypoints;
    geometry_msgs::PoseArray poseArray;

    // 设置可视化消息的坐标系和时间戳
    poseArray.header.frame_id = std::string("world");
    poseArray.header.stamp = ros::Time::now();

    // 添加当前位置作为起点
    {
        geometry_msgs::Pose init_pose;
        init_pose = odom.pose.pose;
        poseArray.poses.push_back(init_pose);
    }

    // 依次添加所有航点到可视化数组
    for (auto it = waypoints.poses.begin(); it != waypoints.poses.end(); ++it) {
        geometry_msgs::Pose p;
        p = it->pose;
        poseArray.poses.push_back(p);
    }

    // 发布航点可视化消息
    pub2.publish(poseArray);
}

/**
 * @brief 里程计回调函数
 * @param msg 里程计消息
 *
 * 功能说明：
 * 1. 更新当前里程计信息和就绪标志
 * 2. 检查航点段队列，判断是否到达下一段航点的发布时间
 * 3. 如果到达时间，则发布相应的航点段并从队列中移除
 */
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 更新里程计数据和就绪标志
    is_odom_ready = true;
    odom = *msg;

    // 检查是否有待发布的航点段
    if (waypointSegments.size()) {
        // 获取下一段航点的期望发布时间
        ros::Time expected_time = waypointSegments.front().header.stamp;

        // 判断是否到达发布时间
        if (odom.header.stamp >= expected_time) {
            // 取出队列首部的航点段
            waypoints = waypointSegments.front();

            // 构造日志信息，输出航点详情
            std::stringstream ss;
            ss << bfmt("Series send %.3f from start:\n") % trigged_time.toSec();
            for (auto& pose_stamped : waypoints.poses) {
                ss << bfmt("P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f)") %
                          pose_stamped.pose.position.x % pose_stamped.pose.position.y %
                          pose_stamped.pose.position.z % pose_stamped.pose.orientation.w %
                          pose_stamped.pose.orientation.x % pose_stamped.pose.orientation.y %
                          pose_stamped.pose.orientation.z << std::endl;
            }
            ROS_INFO_STREAM(ss.str());

            // 发布航点可视化和实际航点
            publish_waypoints_vis();
            publish_waypoints();

            // 从队列中移除已发布的航点段
            waypointSegments.pop_front();
        }
    }
}

/**
 * @brief 目标点回调函数（手动模式）
 * @param msg 目标点位姿消息（通常来自RViz的2D Nav Goal工具）
 *
 * 功能说明：
 * 根据不同的waypoint_type参数，执行不同的航点生成策略：
 * - circle/eight/point: 生成预定义的轨迹模式
 * - series: 加载序列航点
 * - manual-lonely-waypoint: 单个独立航点模式
 * - manual/noyaw: 手动添加航点模式
 *   - z > 0: 添加新航点
 *   - -1 < z < 0: 删除最后一个航点
 *   - z < -1: 结束输入并发布航点
 */
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
/*    if (!is_odom_ready) {
        ROS_ERROR("[waypoint_generator] No odom!");
        return;
    }*/

    // 记录触发时间
    trigged_time = ros::Time::now(); //odom.header.stamp;
    //ROS_ASSERT(trigged_time > ros::Time(0));

    // 读取航点类型参数
    ros::NodeHandle n("~");
    n.param("waypoint_type", waypoint_type, string("manual"));

    // 根据航点类型执行相应操作
    if (waypoint_type == string("circle")) {
        // 圆形轨迹模式
        waypoints = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("eight")) {
        // 8字形轨迹模式
        waypoints = eight();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("point")) {
        // 点轨迹模式
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("series")) {
        // 序列航点模式：从参数服务器加载
        load_waypoints(n, trigged_time);
    } else if (waypoint_type == string("manual-lonely-waypoint")) {
        // 单个独立航点模式
        if (msg->pose.position.z > -0.1) {
            // 高度 > -0.1 为有效目标点
            geometry_msgs::PoseStamped pt = *msg;
            waypoints.poses.clear();
            waypoints.poses.push_back(pt);
            publish_waypoints_vis();
            publish_waypoints();
        } else {
            ROS_WARN("[waypoint_generator] invalid goal in manual-lonely-waypoint mode.");
        }
    } else {
        // 手动模式（默认）：通过高度值控制航点操作
        if (msg->pose.position.z > 0) {
            // 高度 > 0：添加新航点
            geometry_msgs::PoseStamped pt = *msg;
            if (waypoint_type == string("noyaw")) {
                // noyaw模式：保持当前偏航角
                double yaw = tf::getYaw(odom.pose.pose.orientation);
                pt.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            }
            waypoints.poses.push_back(pt);
            publish_waypoints_vis();
        } else if (msg->pose.position.z > -1.0) {
            // -1.0 < 高度 < 0：删除最后一个航点
            if (waypoints.poses.size() >= 1) {
                waypoints.poses.erase(std::prev(waypoints.poses.end()));
            }
            publish_waypoints_vis();
        } else {
            // 高度 < -1.0：结束输入，发布所有航点
            if (waypoints.poses.size() >= 1) {
                publish_waypoints_vis();
                publish_waypoints();
            }
        }
    }
}

/**
 * @brief 轨迹启动触发回调函数
 * @param msg 触发消息
 *
 * 功能说明：
 * 1. 验证里程计数据是否就绪
 * 2. 根据waypoint_type参数生成相应的轨迹模式
 * 3. 发布生成的航点
 *
 * 支持的模式：free/point/circle/eight/series
 */
void traj_start_trigger_callback(const geometry_msgs::PoseStamped& msg) {
    // 检查里程计数据是否就绪
    if (!is_odom_ready) {
        ROS_ERROR("[waypoint_generator] No odom!");
        return;
    }

    ROS_WARN("[waypoint_generator] Trigger!");

    // 记录触发时间为里程计时间戳
    trigged_time = odom.header.stamp;
    ROS_ASSERT(trigged_time > ros::Time(0));

    // 读取航点类型参数
    ros::NodeHandle n("~");
    n.param("waypoint_type", waypoint_type, string("manual"));

    ROS_ERROR_STREAM("Pattern " << waypoint_type << " generated!");

    // 根据类型生成相应的轨迹模式
    if (waypoint_type == string("free")) {
        // 自由模式（等同于point模式）
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("circle")) {
        // 圆形轨迹
        waypoints = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("eight")) {
        // 8字形轨迹
        waypoints = eight();
        publish_waypoints_vis();
        publish_waypoints();
   } else if (waypoint_type == string("point")) {
        // 点轨迹
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("series")) {
        // 序列航点模式
        load_waypoints(n, trigged_time);
    }
}

/**
 * @brief 主函数
 *
 * 功能说明：
 * 1. 初始化ROS节点
 * 2. 读取航点类型参数
 * 3. 创建订阅器：
 *    - odom: 订阅里程计信息
 *    - goal: 订阅目标点（来自RViz）
 *    - traj_start_trigger: 订阅轨迹启动触发信号
 * 4. 创建发布器：
 *    - waypoints: 发布航点路径
 *    - waypoints_vis: 发布航点可视化信息
 * 5. 进入ROS事件循环
 */
int main(int argc, char** argv) {
    // 初始化ROS节点，节点名为"waypoint_generator"
    ros::init(argc, argv, "waypoint_generator");

    // 创建私有命名空间的节点句柄
    ros::NodeHandle n("~");

    // 从参数服务器读取航点类型，默认为"manual"
    n.param("waypoint_type", waypoint_type, string("manual"));

    // 创建订阅器
    ros::Subscriber sub1 = n.subscribe("odom", 10, odom_callback);                         // 订阅里程计话题
    ros::Subscriber sub2 = n.subscribe("goal", 10, goal_callback);                         // 订阅目标点话题（RViz的2D Nav Goal）
    ros::Subscriber sub3 = n.subscribe("traj_start_trigger", 10, traj_start_trigger_callback);  // 订阅轨迹启动触发话题

    // 创建发布器
    pub1 = n.advertise<nav_msgs::Path>("waypoints", 50);                   // 发布航点路径，队列大小50
    pub2 = n.advertise<geometry_msgs::PoseArray>("waypoints_vis", 10);     // 发布航点可视化，队列大小10

    // 初始化触发时间为0
    trigged_time = ros::Time(0);

    // 进入ROS事件循环，等待回调函数被触发
    ros::spin();

    return 0;
}
