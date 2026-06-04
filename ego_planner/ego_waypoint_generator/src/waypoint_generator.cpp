/**
 * @file waypoint_generator.cpp
 * @brief 航点生成器节点实现
 *
 * 功能描述：
 * 该节点负责为无人机轨迹规划提供航点序列。支持多种航点生成模式：
 * 1. manual: 手动模式，通过RViz的2D Nav Goal工具交互式设置航点
 * 2. manual-lonely-waypoint: 手动单航点模式，每次只发送一个航点
 * 3. noyaw: 无偏航角模式，保持当前偏航角不变
 * 4. circle/eight/point: 预定义轨迹模式（圆形、8字形、点序列）
 * 5. series: 分段航点序列模式，从参数服务器加载多段带时间戳的航点
 *
 * 主要功能：
 * - 接收RViz目标点输入并转换为航点序列
 * - 支持相对于当前位置和朝向的坐标变换
 * - 时序航点管理，支持按时间触发的分段航点发布
 * - 航点可视化（PoseArray和Path消息）
 * - 与里程计信息同步，确保航点基于当前位置生成
 *
 * 输入话题：
 * - odom: 无人机里程计信息
 * - goal: RViz发布的目标点（geometry_msgs/PoseStamped）
 * - traj_start_trigger: 轨迹开始触发信号
 *
 * 输出话题：
 * - waypoints: 航点路径（nav_msgs/Path）
 * - waypoints_vis: 航点可视化（geometry_msgs/PoseArray）
 *
 * @author Unknown
 * @date Unknown
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

// ==================== 全局变量定义 ====================

ros::Publisher pub1;  // 航点路径发布器（nav_msgs::Path）
ros::Publisher pub2;  // 航点可视化发布器（geometry_msgs::PoseArray）
ros::Publisher pub3;  // 保留的发布器（未使用）

string waypoint_type = string("manual");  // 航点生成类型，默认为手动模式
bool is_odom_ready;                       // 里程计数据是否就绪的标志位
nav_msgs::Odometry odom;                  // 当前里程计信息，包含位置、姿态、速度等
nav_msgs::Path waypoints;                 // 当前航点路径序列

// series模式所需的分段航点数据
std::deque<nav_msgs::Path> waypointSegments;  // 分段航点队列，每个元素是一段带时间戳的航点序列
ros::Time trigged_time;                       // 航点序列触发时间，用于计算分段航点的发送时刻

/**
 * @brief 从参数服务器加载单个航点分段
 *
 * 该函数用于series模式，从ROS参数服务器读取一个航点分段的配置信息，
 * 并将其转换为世界坐标系下的航点序列。支持相对于当前位置和朝向的坐标变换。
 *
 * 参数格式：
 * - seg<id>/yaw: 该分段的偏航角（相对于当前朝向）
 * - seg<id>/time_of_start: 该分段的触发时间（相对于trigged_time）
 * - seg<id>/x, y, z: 航点坐标数组（相对于当前位置和朝向）
 *
 * 坐标变换过程：
 * 1. 读取相对坐标(ptx, pty, ptz)和相对偏航角yaw
 * 2. 获取当前朝向baseyaw
 * 3. 应用旋转变换将相对坐标转换到世界坐标系
 * 4. 叠加当前位置得到最终世界坐标
 *
 * @param nh 节点句柄，用于读取参数
 * @param segid 分段ID，用于构造参数名称
 * @param time_base 基准时间（通常是trigged_time），用于计算绝对触发时间
 */
void load_seg(ros::NodeHandle& nh, int segid, const ros::Time& time_base) {
    // 构造参数前缀，例如 "seg0/", "seg1/" 等
    std::string seg_str = boost::str(bfmt("seg%d/") % segid);
    double yaw;                  // 相对于当前朝向的偏航角
    double time_of_start = 0.0;  // 相对于触发时间的起始时间

    ROS_INFO("Getting segment %d", segid);

    // 读取并验证偏航角（必须在±π范围内）
    ROS_ASSERT(nh.getParam(seg_str + "yaw", yaw));
    ROS_ASSERT_MSG((yaw > -3.1499999) && (yaw < 3.14999999), "yaw=%.3f", yaw);

    // 读取并验证起始时间（必须非负）
    ROS_ASSERT(nh.getParam(seg_str + "time_of_start", time_of_start));
    ROS_ASSERT(time_of_start >= 0.0);

    // 读取航点坐标数组（相对坐标）
    std::vector<double> ptx;
    std::vector<double> pty;
    std::vector<double> ptz;

    ROS_ASSERT(nh.getParam(seg_str + "x", ptx));
    ROS_ASSERT(nh.getParam(seg_str + "y", pty));
    ROS_ASSERT(nh.getParam(seg_str + "z", ptz));

    // 验证坐标数组：非空且维度一致
    ROS_ASSERT(ptx.size());
    ROS_ASSERT(ptx.size() == pty.size() && ptx.size() == ptz.size());

    nav_msgs::Path path_msg;

    // 设置该分段的触发时间戳（绝对时间）
    path_msg.header.stamp = time_base + ros::Duration(time_of_start);

    // 获取当前的偏航角（世界坐标系）
    double baseyaw = tf::getYaw(odom.pose.pose.orientation);

    // 遍历所有航点，进行坐标变换
    for (size_t k = 0; k < ptx.size(); ++k) {
        geometry_msgs::PoseStamped pt;

        // 设置航点的朝向（当前朝向 + 相对偏航角）
        pt.pose.orientation = tf::createQuaternionMsgFromYaw(baseyaw + yaw);

        // 相对坐标（机体坐标系）
        Eigen::Vector2d dp(ptx.at(k), pty.at(k));
        Eigen::Vector2d rdp;

        // 应用旋转矩阵将相对坐标转换到世界坐标系
        // 旋转角度为 -(baseyaw + yaw)，即从世界系到机体系的逆变换
        rdp.x() = std::cos(-baseyaw-yaw)*dp.x() + std::sin(-baseyaw-yaw)*dp.y();
        rdp.y() =-std::sin(-baseyaw-yaw)*dp.x() + std::cos(-baseyaw-yaw)*dp.y();

        // 世界坐标 = 旋转后的相对坐标 + 当前位置
        pt.pose.position.x = rdp.x() + odom.pose.pose.position.x;
        pt.pose.position.y = rdp.y() + odom.pose.pose.position.y;
        pt.pose.position.z = ptz.at(k) + odom.pose.pose.position.z;  // Z轴直接叠加

        path_msg.poses.push_back(pt);
    }

    // 将该分段加入队列
    waypointSegments.push_back(path_msg);
}

/**
 * @brief 从参数服务器加载所有航点分段
 *
 * 该函数用于series模式，批量加载所有配置的航点分段。
 * 会验证分段的时序性，确保各分段按时间顺序排列。
 *
 * 参数格式：
 * - segment_cnt: 分段总数
 * - seg0/, seg1/, seg2/, ... : 各个分段的配置
 *
 * @param nh 节点句柄，用于读取参数
 * @param time_base 基准时间（trigged_time），用于计算每段的绝对触发时间
 */
void load_waypoints(ros::NodeHandle& nh, const ros::Time& time_base) {
    int seg_cnt = 0;
    waypointSegments.clear();  // 清空之前的分段数据

    // 读取分段总数
    ROS_ASSERT(nh.getParam("segment_cnt", seg_cnt));

    // 依次加载每个分段
    for (int i = 0; i < seg_cnt; ++i) {
        load_seg(nh, i, time_base);

        // 验证时序性：确保当前分段的触发时间晚于前一分段
        if (i > 0) {
            ROS_ASSERT(waypointSegments[i - 1].header.stamp < waypointSegments[i].header.stamp);
        }
    }

    ROS_INFO("Overall load %zu segments", waypointSegments.size());
}

/**
 * @brief 发布航点路径消息
 *
 * 该函数将当前的航点序列发布到waypoints话题，供轨迹规划模块使用。
 * 发布后会在航点序列前插入当前位置作为起点，然后清空航点缓存。
 *
 * 发布流程：
 * 1. 设置消息头（world坐标系，当前时间戳）
 * 2. 发布航点序列（不包含起点）
 * 3. 在序列开头插入当前位置作为起点
 * 4. 清空航点缓存，准备接收新的航点
 */
void publish_waypoints() {
    // 设置航点路径的坐标系和时间戳
    waypoints.header.frame_id = std::string("world");
    waypoints.header.stamp = ros::Time::now();

    // 发布航点序列（不包含起点）
    pub1.publish(waypoints);

    // 构造起点（当前位置）
    geometry_msgs::PoseStamped init_pose;
    init_pose.header = odom.header;
    init_pose.pose = odom.pose.pose;

    // 将起点插入航点序列开头
    waypoints.poses.insert(waypoints.poses.begin(), init_pose);

    // pub2.publish(waypoints);  // 备用发布器（已注释）

    // 清空航点缓存
    waypoints.poses.clear();
}

/**
 * @brief 发布航点可视化消息
 *
 * 该函数将航点序列转换为PoseArray格式并发布，用于在RViz中可视化显示。
 * PoseArray会包含当前位置（起点）和所有待访问的航点。
 *
 * 可视化内容：
 * - 箭头显示每个航点的位置和朝向
 * - 第一个箭头为当前位置
 * - 后续箭头为待访问的航点
 */
void publish_waypoints_vis() {
    nav_msgs::Path wp_vis = waypoints;  // 备份（未使用）
    geometry_msgs::PoseArray poseArray;

    // 设置消息头
    poseArray.header.frame_id = std::string("world");
    poseArray.header.stamp = ros::Time::now();

    // 添加当前位置作为第一个姿态
    {
        geometry_msgs::Pose init_pose;
        init_pose = odom.pose.pose;
        poseArray.poses.push_back(init_pose);
    }

    // 添加所有航点姿态
    for (auto it = waypoints.poses.begin(); it != waypoints.poses.end(); ++it) {
        geometry_msgs::Pose p;
        p = it->pose;
        poseArray.poses.push_back(p);
    }

    // 发布可视化消息
    pub2.publish(poseArray);
}

/**
 * @brief 里程计回调函数
 *
 * 该函数处理无人机的里程计更新，主要用于两个目的：
 * 1. 保存最新的位置和姿态信息，供航点坐标变换使用
 * 2. 在series模式下，根据时间戳触发分段航点的发送
 *
 * series模式的工作流程：
 * - 检查分段队列是否有待发送的分段
 * - 比较当前时间与队首分段的触发时间
 * - 时间到达时，发布该分段航点并从队列中移除
 * - 循环处理直到所有分段发送完毕
 *
 * @param msg 里程计消息，包含位置、姿态、速度等信息
 */
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 标记里程计数据已就绪
    is_odom_ready = true;
    // 保存最新的里程计信息
    odom = *msg;

    // series模式：处理分段航点的时序发送
    if (waypointSegments.size()) {
        // 获取队首分段的预期触发时间
        ros::Time expected_time = waypointSegments.front().header.stamp;

        // 检查是否到达触发时间
        if (odom.header.stamp >= expected_time) {
            // 取出队首分段作为当前航点
            waypoints = waypointSegments.front();

            // 构造日志信息，显示发送的航点详情
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

            // 发布可视化和航点消息
            publish_waypoints_vis();
            publish_waypoints();

            // 从队列中移除已发送的分段
            waypointSegments.pop_front();
        }
    }
}

/**
 * @brief 目标点回调函数
 *
 * 该函数处理从RViz的2D Nav Goal工具接收的目标点，根据不同的模式生成航点。
 * 这是用户交互的主要入口，支持多种航点生成策略。
 *
 * 支持的模式：
 * 1. circle: 生成圆形轨迹
 * 2. eight: 生成8字形轨迹
 * 3. points: 生成点序列轨迹
 * 4. series: 从参数服务器加载分段航点序列
 * 5. manual-lonely-waypoint: 单航点模式，每次只发送一个目标点
 * 6. manual/noyaw: 手动模式，支持交互式添加/删除航点
 *
 * manual模式的高度控制规则：
 * - z > 0: 添加新航点（保持或设置朝向）
 * - -1.0 < z < 0: 删除最后一个航点
 * - z < -1.0: 结束输入，发布航点序列
 *
 * @param msg 目标点消息（从RViz发布）
 */
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // 里程计检查（已注释，允许在无里程计时也能工作）
/*    if (!is_odom_ready) {
        ROS_ERROR("[waypoint_generator] No odom!");
        return;
    }*/

    // 记录触发时间（用于series模式的时序控制）
    trigged_time = ros::Time::now(); //odom.header.stamp;
    //ROS_ASSERT(trigged_time > ros::Time(0));

    // 读取航点类型参数
    ros::NodeHandle n("~");
    n.param("waypoint_type", waypoint_type, string("manual"));

    // 根据不同模式生成航点
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
    } else if (waypoint_type == string("points")) {
        // 点序列轨迹模式
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("series")) {
        // 分段航点序列模式（从参数服务器加载）
        load_waypoints(n, trigged_time);
    } else if (waypoint_type == string("manual-lonely-waypoint")) {
        // 单航点模式：每次只发送一个目标点
        if (msg->pose.position.z > -0.1) {
            // 高度 > -0.1 视为有效目标点
            geometry_msgs::PoseStamped pt = *msg;
            waypoints.poses.clear();  // 清空之前的航点
            waypoints.poses.push_back(pt);
            publish_waypoints_vis();
            publish_waypoints();
        } else {
            ROS_WARN("[waypoint_generator] invalid goal in manual-lonely-waypoint mode.");
        }
    } else {
        // 手动模式（manual/noyaw）：交互式添加/删除航点
        if (msg->pose.position.z > 0) {
            // 情况1: 高度 > 0，添加新航点
            geometry_msgs::PoseStamped pt = *msg;

            // noyaw模式：保持当前偏航角不变
            if (waypoint_type == string("noyaw")) {
                double yaw = tf::getYaw(odom.pose.pose.orientation);
                pt.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            }

            waypoints.poses.push_back(pt);
            publish_waypoints_vis();  // 只更新可视化，不发布航点
        } else if (msg->pose.position.z > -1.0) {
            // 情况2: 0 > 高度 > -1.0，删除最后一个航点
            if (waypoints.poses.size() >= 1) {
                waypoints.poses.erase(std::prev(waypoints.poses.end()));
            }
            publish_waypoints_vis();
        } else {
            // 情况3: 高度 < -1.0，结束输入并发布航点序列
            if (waypoints.poses.size() >= 1) {
                publish_waypoints_vis();
                publish_waypoints();  // 发布航点给规划模块
            }
        }
    }
}

/**
 * @brief 轨迹开始触发回调函数
 *
 * 该函数响应外部触发信号，用于启动预定义的轨迹模式。
 * 与goal_callback不同，这是通过话题消息触发，而非RViz交互。
 * 主要用于自动化测试和程序化控制。
 *
 * 支持的模式：
 * - free/point: 点序列轨迹
 * - circle: 圆形轨迹
 * - eight: 8字形轨迹
 * - series: 分段航点序列
 *
 * @param msg 触发消息（内容未使用，仅作为触发信号）
 */
void traj_start_trigger_callback(const geometry_msgs::PoseStamped& msg) {
    // 检查里程计是否就绪
    if (!is_odom_ready) {
        ROS_ERROR("[waypoint_generator] No odom!");
        return;
    }

    ROS_WARN("[waypoint_generator] Trigger!");

    // 记录触发时间（使用里程计时间戳）
    trigged_time = odom.header.stamp;
    ROS_ASSERT(trigged_time > ros::Time(0));

    // 读取航点类型参数
    ros::NodeHandle n("~");
    n.param("waypoint_type", waypoint_type, string("manual"));

    ROS_ERROR_STREAM("Pattern " << waypoint_type << " generated!");

    // 根据模式生成并发布航点
    if (waypoint_type == string("free")) {
        // 自由点序列模式
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("circle")) {
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
        // 点序列轨迹模式
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("series")) {
        // 分段航点序列模式
        load_waypoints(n, trigged_time);
    }
}

/**
 * @brief 主函数
 *
 * 初始化ROS节点和通信接口，设置航点生成器的基本参数。
 *
 * 节点配置：
 * - 节点名称: waypoint_generator
 * - 默认航点类型: manual（可通过参数服务器修改）
 *
 * 订阅话题：
 * - odom: 里程计信息（队列长度10）
 * - goal: RViz目标点（队列长度10）
 * - traj_start_trigger: 轨迹开始触发信号（队列长度10）
 *
 * 发布话题：
 * - waypoints: 航点路径序列（队列长度50）
 * - waypoints_vis: 航点可视化（队列长度10）
 *
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 */
int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "waypoint_generator");
    ros::NodeHandle n("~");

    // 读取航点类型参数（默认为manual模式）
    n.param("waypoint_type", waypoint_type, string("manual"));

    // 订阅话题
    ros::Subscriber sub1 = n.subscribe("odom", 10, odom_callback);
    ros::Subscriber sub2 = n.subscribe("goal", 10, goal_callback);
    ros::Subscriber sub3 = n.subscribe("traj_start_trigger", 10, traj_start_trigger_callback);

    // 创建发布器
    pub1 = n.advertise<nav_msgs::Path>("waypoints", 50);
    pub2 = n.advertise<geometry_msgs::PoseArray>("waypoints_vis", 10);

    // 初始化触发时间
    trigged_time = ros::Time(0);

    // 进入ROS事件循环
    ros::spin();

    return 0;
}
