/**
 * @file ego_planner_node.cpp
 * @brief EGO-Planner主节点程序
 *
 * 该文件是EGO-Planner轨迹规划器的ROS节点入口程序。
 * EGO-Planner是一个基于梯度的局部轨迹优化规划器，用于无人机在未知环境中的自主导航。
 * 主要特点：
 * - 使用B样条曲线表示轨迹
 * - 基于梯度的优化方法
 * - 支持动态障碍物环境
 * - 实时重规划能力
 */

#include <ros/ros.h>                          // ROS核心库
#include <visualization_msgs/Marker.h>        // ROS可视化消息类型

#include <ego_plan_manage/ego_replan_fsm.h>       // EGO-Planner重规划有限状态机

using namespace ego_planner;  // 使用ego_planner命名空间

/**
 * @brief EGO-Planner节点主函数
 *
 * 该函数完成以下任务：
 * 1. 初始化ROS节点
 * 2. 创建EGO重规划有限状态机对象
 * 3. 初始化状态机（加载参数、订阅话题、发布器等）
 * 4. 进入ROS事件循环，处理回调函数
 *
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码，0表示正常退出
 */
int main(int argc, char **argv)
{
  // 初始化ROS节点，节点名称为"ego_planner_node"
  ros::init(argc, argv, "ego_planner_node");

  // 创建私有命名空间的节点句柄
  // 使用"~"表示私有命名空间，用于读取该节点专属的参数
  ros::NodeHandle nh("~");

  // 创建EGO重规划有限状态机对象
  // 该对象负责管理整个规划流程，包括状态转换、轨迹优化、碰撞检测等
  EGOReplanFSM rebo_replan;

  // 初始化重规划状态机
  // 完成参数加载、话题订阅、发布器初始化、定时器设置等工作
  rebo_replan.init(nh);

  // 延迟1秒，等待所有ROS组件完全初始化
  // 确保所有订阅者和发布者都已准备就绪
  ros::Duration(1.0).sleep();

  // 进入ROS事件循环，开始处理回调函数
  // 程序将持续运行直到收到关闭信号（如Ctrl+C）
  ros::spin();

  // 正常退出程序
  return 0;
}
