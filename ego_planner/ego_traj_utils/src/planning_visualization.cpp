/*
 * 文件: planning_visualization.cpp
 * 功能: EGO-Planner路径规划可视化模块实现
 * 描述: 提供RViz可视化功能，用于显示目标点、全局路径、初始路径、优化后路径和A*搜索路径等
 *       支持多种可视化形式，包括球体标记、线条、箭头等
 */

#include <ego_traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace ego_planner
{
  /**
   * @brief 构造函数 - 初始化规划可视化对象
   * @param nh ROS节点句柄，用于创建发布器
   *
   * 功能说明:
   * - 初始化5个可视化发布器，分别用于不同类型的路径和点的可视化
   * - goal_point_pub: 发布目标点可视化消息
   * - global_list_pub: 发布全局路径点列表
   * - init_list_pub: 发布初始路径点列表
   * - optimal_list_pub: 发布优化后的路径点列表
   * - a_star_list_pub: 发布A*搜索得到的路径（可能有多条）
   */
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh)
  {
    node = nh;

    // 创建目标点可视化发布器，队列大小为2
    goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
    // 创建全局路径可视化发布器
    global_list_pub = nh.advertise<visualization_msgs::Marker>("global_list", 2);
    // 创建初始路径可视化发布器
    init_list_pub = nh.advertise<visualization_msgs::Marker>("init_list", 2);
    // 创建优化路径可视化发布器
    optimal_list_pub = nh.advertise<visualization_msgs::Marker>("optimal_list", 2);
    // 创建A*路径可视化发布器，队列大小为20以支持多条路径同时显示
    a_star_list_pub = nh.advertise<visualization_msgs::Marker>("a_star_list", 20);
  }

  /**
   * @brief 在RViz中显示标记点列表（球体+连线形式）
   * @param pub ROS发布器，用于发布可视化消息
   * @param list 要显示的3D点列表
   * @param scale 球体的尺寸大小
   * @param color RGBA颜色向量 (R, G, B, A)
   * @param id 标记ID，用于区分不同的可视化对象
   *
   * 实际使用的ID: {id, id+1000}
   * - id: 用于球体列表
   * - id+1000: 用于连线
   *
   * 可视化效果:
   * - 在每个路径点处显示一个球体
   * - 用线条连接所有路径点，形成连续路径
   */
  void PlanningVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id)
  {
    // 创建两个标记：球体列表和线条
    visualization_msgs::Marker sphere, line_strip;

    // 设置坐标系为"world"，时间戳为当前时间
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();

    // 设置标记类型：球体列表和连续线条
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // 设置操作为添加
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;

    // 设置ID，线条ID比球体ID大1000以避免冲突
    sphere.id = id;
    line_strip.id = id + 1000;

    // 设置姿态四元数（无旋转）
    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    // 设置颜色（RGB）
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    // 设置透明度，如果color(3)太小则设为1.0（完全不透明）
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;

    // 设置球体尺寸（xyz三个方向相同）
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    // 设置线条宽度为球体尺寸的一半
    line_strip.scale.x = scale / 2;

    // 遍历所有点，添加到球体列表和线条中
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);      // 添加到球体列表
      line_strip.points.push_back(pt);   // 添加到线条点列表
    }

    // 发布球体和线条标记
    pub.publish(sphere);
    pub.publish(line_strip);
  }

  /**
   * @brief 生成路径显示数组（用于批量可视化）
   * @param array MarkerArray引用，用于存储生成的标记
   * @param list 要显示的3D点列表
   * @param scale 球体的尺寸大小
   * @param color RGBA颜色向量 (R, G, B, A)
   * @param id 标记ID基础值
   *
   * 实际使用的ID: {id, id+1}
   * - id: 用于球体列表
   * - id+1: 用于连线
   *
   * 注意: 此函数使用"map"坐标系，与displayMarkerList的"world"坐标系不同
   * 线条宽度为scale/3，比displayMarkerList中的scale/2更细
   */
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    // 创建球体和线条标记
    visualization_msgs::Marker sphere, line_strip;

    // 设置坐标系为"map"（注意与displayMarkerList不同）
    sphere.header.frame_id = line_strip.header.frame_id = "map";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();

    // 设置标记类型
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // 设置操作为添加
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;

    // 设置ID，线条ID为id+1
    sphere.id = id;
    line_strip.id = id + 1;

    // 设置姿态和颜色
    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;

    // 设置尺寸
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    // 线条宽度为scale/3（比displayMarkerList中的scale/2更细）
    line_strip.scale.x = scale / 3;

    // 添加所有点到标记中
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }

    // 将球体和线条添加到标记数组中
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  /**
   * @brief 生成箭头显示数组（用于显示方向信息）
   * @param array MarkerArray引用，用于存储生成的箭头标记
   * @param list 3D点列表，每两个点形成一个箭头（起点和终点）
   * @param scale 箭头轴的直径
   * @param color RGBA颜色向量 (R, G, B, A)
   * @param id 标记ID基础值
   *
   * 实际使用的ID范围: {1000*id ~ (箭头数量)+1000*id}
   * 例如：如果id=2且有5个箭头，则ID范围为[2000, 2004]
   *
   * 数据格式:
   * - list中每两个连续的点定义一个箭头
   * - list[2*i] 为第i个箭头的起点
   * - list[2*i+1] 为第i个箭头的终点
   *
   * 箭头尺寸:
   * - scale.x: 箭头轴的直径
   * - scale.y: 箭头头部的直径（是轴直径的2倍）
   * - scale.z: 箭头头部的长度（是轴直径的2倍）
   */
  void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    // 创建箭头标记模板
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // 设置箭头颜色
    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;

    // 设置箭头尺寸
    arrow.scale.x = scale;        // 箭头轴的直径
    arrow.scale.y = 2 * scale;    // 箭头头部的直径
    arrow.scale.z = 2 * scale;    // 箭头头部的长度

    // 遍历点列表，每两个点创建一个箭头
    geometry_msgs::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // 注释掉的代码：可以让箭头颜色随索引渐变
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      // 设置箭头起点（第2*i个点）
      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);

      // 设置箭头终点（第2*i+1个点）
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);

      // 清空并设置箭头的两个点（起点和终点）
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);

      // 设置唯一ID，使用i + id*1000的方式避免冲突
      arrow.id = i + id * 1000;

      // 添加到标记数组
      array.markers.push_back(arrow);
    }
  }

  /**
   * @brief 显示目标点（单个球体标记）
   * @param goal_point 目标点的3D坐标
   * @param color RGBA颜色向量 (R, G, B, A)
   * @param scale 球体尺寸
   * @param id 标记ID
   *
   * 用途: 在RViz中以球体形式高亮显示规划的目标位置
   */
  void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    // 创建球体标记
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    // 设置姿态（无旋转）
    sphere.pose.orientation.w = 1.0;

    // 设置颜色（使用传入的color参数的alpha值）
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);

    // 设置球体尺寸
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;

    // 设置球体位置为目标点坐标
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    // 发布目标点标记
    goal_point_pub.publish(sphere);
  }

  /**
   * @brief 显示全局路径点列表
   * @param init_pts 全局路径的3D点列表
   * @param scale 显示球体的尺寸
   * @param id 标记ID
   *
   * 颜色: 青色 (R=0, G=0.5, B=0.5, A=1.0)
   * 优化: 如果没有订阅者，则跳过发布以节省计算资源
   */
  void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {
    // 检查是否有订阅者，没有则直接返回（避免无用计算）
    if (global_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    // 设置青色 (cyan)
    Eigen::Vector4d color(0, 0.5, 0.5, 1);
    // 调用通用的标记列表显示函数
    displayMarkerList(global_list_pub, init_pts, scale, color, id);
  }

  /**
   * @brief 显示初始路径点列表
   * @param init_pts 初始路径的3D点列表
   * @param scale 显示球体的尺寸
   * @param id 标记ID
   *
   * 颜色: 蓝色 (R=0, G=0, B=1, A=1.0)
   * 优化: 如果没有订阅者，则跳过发布以节省计算资源
   */
  void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {
    // 检查是否有订阅者，没有则直接返回（避免无用计算）
    if (init_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    // 设置蓝色 (blue)
    Eigen::Vector4d color(0, 0, 1, 1);
    // 调用通用的标记列表显示函数
    displayMarkerList(init_list_pub, init_pts, scale, color, id);
  }

  /**
   * @brief 显示优化后的路径点列表
   * @param optimal_pts 优化后的路径点矩阵，每一列代表一个3D点
   * @param id 标记ID
   *
   * 数据格式:
   * - optimal_pts: 3xN矩阵，其中N是路径点数量
   * - 每一列包含一个点的 [x, y, z] 坐标
   *
   * 颜色: 红色 (R=1, G=0, B=0, A=1.0)
   * 尺寸: 固定为0.15
   * 优化: 如果没有订阅者，则跳过发布以节省计算资源
   */
  void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
  {
    // 检查是否有订阅者，没有则直接返回（避免无用计算）
    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    // 将矩阵形式的点转换为vector<Eigen::Vector3d>格式
    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
      // 提取第i列并转置为3D向量
      Eigen::Vector3d pt = optimal_pts.col(i).transpose();
      list.push_back(pt);
    }

    // 设置红色 (red)，用于突出显示优化后的路径
    Eigen::Vector4d color(1, 0, 0, 1);
    // 调用通用的标记列表显示函数，使用固定尺寸0.15
    displayMarkerList(optimal_list_pub, list, 0.15, color, id);
  }

  /**
   * @brief 显示A*搜索得到的多条路径
   * @param a_star_paths A*路径集合，每条路径是一个3D点列表
   * @param id 标记ID基础值
   *
   * 数据格式:
   * - a_star_paths: 二维向量，第一层表示不同的路径，第二层表示每条路径上的点
   * - 例如: a_star_paths[i] 代表第i条A*路径
   *
   * 实际使用的ID范围: [id ~ id+a_star_paths.size()-1]
   *
   * 可视化特性:
   * - 每次调用时随机生成颜色（黄绿色系），使不同次的A*路径有所区别
   * - R分量: 0.5 ~ 1.0（随机）
   * - G分量: 0.5 ~ 1.0（随机）
   * - B分量: 0（固定）
   * - 尺寸: 0.05 ~ 0.15（随机）
   *
   * 优化: 如果没有订阅者，则跳过发布以节省计算资源
   */
  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {
    // 检查是否有订阅者，没有则直接返回（避免无用计算）
    if (a_star_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    int i = 0;
    vector<Eigen::Vector3d> list;

    // 随机生成颜色（黄绿色系，R和G在0.5~1.0之间，B为0）
    // 这样每次调用时A*路径的颜色会有所不同
    Eigen::Vector4d color = Eigen::Vector4d(
        0.5 + ((double)rand() / RAND_MAX / 2),  // R: 0.5 ~ 1.0
        0.5 + ((double)rand() / RAND_MAX / 2),  // G: 0.5 ~ 1.0
        0,                                       // B: 0
        1);                                      // A: 1.0 (完全不透明)

    // 随机生成尺寸（0.05 ~ 0.15）
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    // 遍历所有A*路径
    for (auto block : a_star_paths)
    {
      list.clear();
      // 将当前路径的所有点添加到list中
      for (auto pt : block)
      {
        list.push_back(pt);
      }

      // 显示当前路径，使用id+i作为标记ID
      // 实际使用的ID范围: [id ~ id+a_star_paths.size()-1]
      displayMarkerList(a_star_list_pub, list, scale, color, id + i);
      i++;
    }
  }

  /**
   * @brief 显示箭头列表
   * @param pub ROS发布器，用于发布箭头标记数组
   * @param list 3D点列表，每两个点定义一个箭头（起点和终点）
   * @param scale 箭头轴的直径
   * @param color RGBA颜色向量 (R, G, B, A)
   * @param id 标记ID基础值
   *
   * 工作流程:
   * 1. 先发布空数组以清除之前的箭头
   * 2. 生成新的箭头数组
   * 3. 发布新的箭头数组
   *
   * 用途: 通常用于显示速度、加速度等矢量信息
   */
  void PlanningVisualization::displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::MarkerArray array;

    // 先发布空数组以清除之前的所有箭头标记
    pub.publish(array);

    // 生成新的箭头显示数组
    generateArrowDisplayArray(array, list, scale, color, id);

    // 发布新的箭头数组
    pub.publish(array);
  }

} // namespace ego_planner