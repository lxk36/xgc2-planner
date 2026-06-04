/**
 * @file euroc.cpp
 * @brief EUROC数据集处理节点 - 用于同步相机图像和位姿数据，并渲染深度图像
 *
 * 该节点主要功能：
 * 1. 订阅EUROC数据集的相机图像和位姿数据
 * 2. 使用GPU加速的深度渲染器生成深度图像
 * 3. 支持图像去畸变处理
 * 4. 提供PnP求解用于相机标定
 * 5. 发布深度图和彩色深度图用于可视化
 */

#include <iostream>
#include <fstream>
#include <vector>

// ROS相关头文件
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// 动态参数配置
#include <dynamic_reconfigure/server.h>
#include <cloud_banchmark/cloud_banchmarkConfig.h>

// 点云处理库
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

// 数学和图像处理库
#include <eigen3/Eigen/Dense>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

// 自定义的GPU深度渲染器
#include "depth_render.cuh"

using namespace cv;
using namespace std;
using namespace Eigen;

// 定义近似时间同步策略类型：用于同步图像和位姿数据
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::TransformStamped> approx_policy;

// ===== 深度图相关变量 =====
int *depth_hostptr;         // 深度数据的主机端指针（用于从GPU拷贝数据）
cv::Mat depth_mat;          // 深度图矩阵（单位：米）

// ===== 相机内参变量 =====
int width, height;          // 图像宽度和高度
cv::Mat cv_K, cv_D;         // 相机内参矩阵K和畸变系数D
double fx, fy, cx, cy;      // 相机焦距(fx, fy)和主点(cx, cy)
cv::Mat undist_map1, undist_map2;  // 去畸变映射表
bool is_distorted(false);   // 标志位：图像是否存在畸变

// ===== GPU深度渲染器和发布器 =====
DepthRender depthrender;    // GPU深度渲染器对象
ros::Publisher pub_depth;   // 深度图发布器
ros::Publisher pub_color;   // 彩色深度图发布器
ros::Publisher pub_posedimage;  // 位姿图像发布器

// ===== 坐标系变换矩阵 =====
Matrix4d vicon2body;        // Vicon坐标系到机体坐标系的变换
Matrix4d cam02body;         // 相机坐标系到机体坐标系的变换
Matrix4d cam2world;         // 相机坐标系到世界坐标系的变换
Matrix4d vicon2leica;       // Vicon坐标系到Leica坐标系的变换（通过PnP求解得到）

// ===== 时间戳和图像数据 =====
ros::Time receive_stamp;    // 接收到的消息时间戳
cv::Mat undistorted_image;  // 去畸变后的图像

/**
 * @struct PoseInfo
 * @brief 位姿信息结构体 - 存储时间戳和对应的4x4变换矩阵
 */
struct PoseInfo
{
  Matrix4d pose;            // 4x4位姿变换矩阵
  double time;              // 时间戳
};

vector<PoseInfo> gt_pose_vect;  // 存储所有ground truth位姿的向量

// 函数前向声明
void render_currentpose();

/**
 * @brief 从文件中读取ground truth位姿数据
 *
 * 文件格式：每行8个数据
 * timestamp tx ty tz qw qx qy qz
 * 其中：timestamp为时间戳，(tx,ty,tz)为位置，(qw,qx,qy,qz)为四元数姿态
 *
 * @param file 输入文件流
 * @return 总是返回true
 */
bool read_pose(fstream &file)
{
  int count = 0;          // 位姿数量计数器
  bool good = true;       // 文件读取状态标志
  double para[16];        // 临时存储读取的参数

  // 循环读取文件中的每一行位姿数据
  while(good)
  {
    count ++;
    // 每行读取8个参数：时间戳 + 3个位置 + 4个四元数
    for(int i = 0; i < 8 && good; i++)
    {
      good = good && file >> para[i];
    }

    if(good)
    {
      // 解析位置信息
      Eigen::Vector3d request_position;
      Eigen::Quaterniond request_pose;
      request_position.x() = para[1];  // x坐标
      request_position.y() = para[2];  // y坐标
      request_position.z() = para[3];  // z坐标

      // 解析四元数姿态（注意四元数顺序：w, x, y, z）
      request_pose.w() = para[4];
      request_pose.x() = para[5];
      request_pose.y() = para[6];
      request_pose.z() = para[7];

      // 构建位姿信息并添加到向量中
      PoseInfo info;
      info.time = para[0];  // 时间戳
      info.pose = Matrix4d::Identity();  // 初始化为单位矩阵
      info.pose.block<3,3>(0,0) = request_pose.toRotationMatrix();  // 设置旋转部分
      info.pose(0,3) = para[1];  // 设置平移部分x
      info.pose(1,3) = para[2];  // 设置平移部分y
      info.pose(2,3) = para[3];  // 设置平移部分z
      gt_pose_vect.push_back(info);
    }
  }
  printf("we have %d poses.\n", count);
  return true;
}

// ===== PnP求解相关的点对 =====
vector<cv::Point3f> pts_3;  // 3D空间点集合（相机坐标系）
vector<cv::Point2f> pts_2;  // 2D图像点集合（像素坐标）

/**
 * @brief 图像窗口的鼠标回调函数
 *
 * 当用户在图像窗口中点击时，记录该像素坐标作为2D点
 * 用于后续的PnP求解（配合深度图中的3D点）
 *
 * @param event 鼠标事件类型
 * @param x 鼠标点击的x坐标
 * @param y 鼠标点击的y坐标
 * @param flags 额外标志
 * @param userdata 用户数据指针
 */
void imageBackFunc(int event, int x, int y, int flags, void* userdata)
{
  if( event == EVENT_LBUTTONDOWN )  // 左键点击事件
  {
    cout << "image is clicked - position (" << x << ", " << y << ")" << endl;
    pts_2.push_back(cv::Point2f(x,y));  // 记录2D像素坐标
  }
}

/**
 * @brief 深度图窗口的鼠标回调函数
 *
 * 当用户在深度图窗口中点击时，根据深度值反投影到3D空间
 * 生成相机坐标系下的3D点，用于PnP求解
 *
 * @param event 鼠标事件类型
 * @param x 鼠标点击的x坐标
 * @param y 鼠标点击的y坐标
 * @param flags 额外标志
 * @param userdata 用户数据指针
 */
void depthBackFunc(int event, int x, int y, int flags, void* userdata)
{
  if( event == EVENT_LBUTTONDOWN )  // 左键点击事件
  {
    cout << "depth is clicked - position (" << x << ", " << y << ")" << endl;
    double depth = depth_mat.at<float>(y,x);  // 获取该点的深度值

    // 根据针孔相机模型反投影到3D空间（相机坐标系）
    // 公式：X = (u - cx) * Z / fx, Y = (v - cy) * Z / fy, Z = depth
    double space_x = (x - cx) * depth / fx;
    double space_y = (y - cy) * depth / fy;
    double space_z = depth;
    pts_3.push_back(cv::Point3f(space_x, space_y, space_z));  // 记录3D点
  }
}

/**
 * @brief 使用PnP算法求解Vicon到Leica坐标系的变换
 *
 * PnP (Perspective-n-Point)算法：已知n个3D-2D点对和相机内参，求解相机位姿
 * 这里用于标定Vicon和Leica两个坐标系之间的变换关系
 *
 * 算法流程：
 * 1. 检查点对数量是否足够（至少需要5对点）
 * 2. 使用OpenCV的solvePnP求解相机位姿
 * 3. 将结果转换为4x4变换矩阵
 * 4. 取逆得到vicon2leica变换
 */
void solve_pnp()
{
  // 注释中是一个参考变换矩阵示例
  //   translation :
  //   0.994976 -0.0431638  0.0903361 -0.0338185
  //  0.0444475   0.998937  -0.012246  0.0541652
  // -0.0897114  0.0161996   0.995836  0.0384018
  //          0          0          0          1

  printf("we have %d pair points.\n", pts_3.size());

  // 检查点对数量：PnP至少需要4对点，这里要求至少5对以提高精度
  if(pts_3.size() < 5 || pts_2.size() < 5)
  {
    return;
  }

  // 检查3D点和2D点数量是否匹配
  if(pts_3.size() != pts_2.size())
  {
    printf("error, not equal!\n");
    return;
  }

  // 使用OpenCV的PnP求解器
  cv::Mat r, rvec, t;
  // solvePnP输入：3D点、2D点、相机内参、畸变系数（这里设为0），输出：旋转向量、平移向量
  cv::solvePnP(pts_3, pts_2, cv_K, cv::Mat::zeros(4,1,CV_32FC1), rvec, t);

  // 将旋转向量转换为旋转矩阵（罗德里格斯公式）
  cv::Rodrigues(rvec, r);

  // 将OpenCV的Mat格式转换为Eigen的Matrix格式
  Matrix3d R_ref;
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
      {
          R_ref(i,j) = r.at<double>(i, j);
      }

  // 构建完整的4x4变换矩阵
  Matrix4d pnp_result = Matrix4d::Identity();
  pnp_result.block<3,3>(0,0) = R_ref;  // 设置旋转部分
  pnp_result(0,3) = t.at<double>(0, 0);  // 设置平移部分
  pnp_result(1,3) = t.at<double>(1, 0);
  pnp_result(2,3) = t.at<double>(2, 0);

  // 取逆得到vicon到leica的变换（PnP求解的是leica到vicon）
  vicon2leica = pnp_result.inverse();
  cout << "translation : " << endl << pnp_result << endl;
}

/**
 * @brief 图像和位姿同步回调函数
 *
 * 该函数是核心回调函数，当图像和位姿数据同步到达时被调用
 *
 * 主要步骤：
 * 1. 检查图像和位姿的时间差
 * 2. 从ground truth数据中查找最接近的位姿
 * 3. 进行坐标系变换：ground truth -> body -> camera -> world
 * 4. 对图像进行去畸变处理
 * 5. 调用render_currentpose()渲染深度图
 *
 * @param image_input 输入的图像消息（单目灰度图）
 * @param pose_input 输入的位姿消息（Transform）
 */
void image_pose_callback(
    const sensor_msgs::ImageConstPtr &image_input,
    const geometry_msgs::TransformStampedConstPtr &pose_input)
{
  // ===== 计算图像和位姿的时间差 =====
  double time_diff = fabs(image_input->header.stamp.toSec() - pose_input->header.stamp.toSec()) * 1000.0;
  printf("time diff is %lf ms.\n", time_diff);

  // ===== 初始化接收到的位姿矩阵 =====
  Matrix4d Pose_receive = Matrix4d::Identity();

  // ===== 方法1：使用Vicon实时位姿（已注释） =====
  // 如果使用Vicon系统提供的实时位姿数据，可以使用以下代码
  // Eigen::Vector3d request_position;
  // Eigen::Quaterniond request_pose;
  // request_position.x() = pose_input->transform.translation.x;
  // request_position.y() = pose_input->transform.translation.y;
  // request_position.z() = pose_input->transform.translation.z;
  // request_pose.x() = pose_input->transform.rotation.x;
  // request_pose.y() = pose_input->transform.rotation.y;
  // request_pose.z() = pose_input->transform.rotation.z;
  // request_pose.w() = pose_input->transform.rotation.w;
  // Pose_receive.block<3,3>(0,0) = request_pose.toRotationMatrix();
  // Pose_receive(0,3) = request_position(0);
  // Pose_receive(1,3) = request_position(1);
  // Pose_receive(2,3) = request_position(2);

  // ===== 方法2：使用ground truth位姿数据（当前使用） =====
  // 通过时间戳匹配，从预加载的ground truth数据中找到最接近的位姿
  double image_time = image_input->header.stamp.toSec();
  double min_time_diff = 999.9;  // 初始化为一个大值
  int min_time_index = 0;

  // 遍历所有ground truth位姿，找到时间戳最接近的一个
  for(int i = 1; i < gt_pose_vect.size(); i++)
  {
    double time_diff = fabs(image_time - gt_pose_vect[i].time);
    if(time_diff < min_time_diff)
    {
      min_time_diff = time_diff;
      min_time_index = i;
    }
  }
  printf("min time diff index %d, with diff time %lf ms.\n", min_time_index, min_time_diff*1000.0f);
  Pose_receive = gt_pose_vect[min_time_index].pose;

  // ===== 坐标系转换：ground truth -> body pose =====
  // 如果需要考虑Vicon到机体的变换，可以使用以下代码
  // Matrix4d body_pose = Pose_receive * vicon2body.inverse();
  Matrix4d body_pose = Pose_receive;  // 这里直接使用ground truth作为body pose

  // ===== 坐标系转换：body -> camera -> world =====
  // cam2world = body * cam02body * vicon2leica
  // 其中：cam02body是相机到机体的外参，vicon2leica是通过PnP标定得到的变换
  cam2world = body_pose * cam02body * vicon2leica;

  // 保存时间戳用于后续发布
  receive_stamp = pose_input->header.stamp;

  // ===== 图像处理：去畸变 =====
  cv_bridge::CvImageConstPtr cv_img_ptr = cv_bridge::toCvShare(image_input, sensor_msgs::image_encodings::MONO8);
  cv::Mat img_8uC1 = cv_img_ptr->image;
  undistorted_image.create(height, width, CV_8UC1);

  if(is_distorted)
  {
    // 如果图像有畸变，使用预计算的映射表进行重映射去畸变
    cv::remap(img_8uC1, undistorted_image, undist_map1, undist_map2, CV_INTER_LINEAR);
  }
  else
    undistorted_image = img_8uC1;  // 无畸变，直接使用原图

  // ===== 渲染当前位姿的深度图 =====
  render_currentpose();
}

/**
 * @brief 渲染当前相机位姿的深度图
 *
 * 该函数执行以下主要操作：
 * 1. 执行PnP求解（如果有足够的点对）
 * 2. 使用GPU深度渲染器渲染深度图
 * 3. 将深度数据从GPU拷贝到CPU并进行格式转换
 * 4. 发布原始深度图和彩色可视化深度图
 * 5. 在窗口中显示图像和深度图用于交互式标定
 */
void render_currentpose()
{
  // 尝试执行PnP求解（如果用户已收集足够的点对）
  solve_pnp();

  // 记录渲染开始时间，用于性能统计
  double this_time = ros::Time::now().toSec();

  // ===== GPU深度渲染 =====
  // 将相机位姿从world坐标系转换为相机坐标系（取逆）
  Matrix4d cam_pose = cam2world.inverse();

  // 调用GPU渲染器渲染深度图，结果存储在depth_hostptr中
  depthrender.render_pose(cam_pose, depth_hostptr);

  // ===== 深度数据处理 =====
  // 创建深度矩阵，存储浮点型深度值（单位：米）
  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);
  double min = 0.5;   // 最小深度值（用于可视化归一化）
  double max = 1.0f;  // 最大深度值（动态更新）

  // 遍历所有像素，将深度数据从GPU缓冲区拷贝到OpenCV矩阵
  for(int i = 0; i < height; i++)
  	for(int j = 0; j < width; j++)
  	{
  		// GPU渲染器输出的深度单位是毫米，转换为米
  		float depth = (float)depth_hostptr[i * width + j] / 1000.0f;
  		// 过滤掉过大的深度值（可能是无效点）
  		depth = depth < 500.0f ? depth : 0;
  		// 更新最大深度值
  		max = depth > max ? depth : max;
  		depth_mat.at<float>(i,j) = depth;
  	}

  // 输出渲染性能统计
  ROS_INFO("render cost %lf ms.", (ros::Time::now().toSec() - this_time) * 1000.0f);
  printf("max_depth %lf.\n", max);

  // ===== 发布原始深度图 =====
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = receive_stamp;  // 使用原始消息的时间戳
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;  // 32位浮点单通道
  out_msg.image = depth_mat.clone();
  pub_depth.publish(out_msg.toImageMsg());

  // ===== 生成彩色可视化深度图 =====
  // 将深度图归一化到0-255范围
  cv::Mat adjMap;
  depth_mat.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);

  // 应用彩虹色彩映射（近处蓝色，远处红色）
  cv::Mat falseColorsMap;
  cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);

  // 将原始灰度图转换为BGR格式
  cv::Mat bgr_image;
  cv::cvtColor(undistorted_image, bgr_image, cv::COLOR_GRAY2BGR);

  // 将彩色深度图叠加到原始图像上（深度图权重0.8，原图权重0.2）
  cv::addWeighted(bgr_image, 0.2, falseColorsMap, 0.8, 0.0, falseColorsMap);

  // ===== 发布彩色深度图 =====
  cv_bridge::CvImage cv_image_colored;
  cv_image_colored.header.frame_id = "depthmap";
  cv_image_colored.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image_colored.image = falseColorsMap;
  pub_color.publish(cv_image_colored.toImageMsg());

  // ===== 在窗口中显示图像（用于交互式PnP标定） =====
  cv::imshow("bluefox_image", bgr_image);     // 显示原始图像（用户可在此点击选择2D点）
  cv::imshow("depth_image", adjMap);          // 显示深度图（用户可在此点击选择3D点）
}

/**
 * @brief 主函数 - EUROC数据处理节点的入口
 *
 * 主要流程：
 * 1. 初始化ROS节点
 * 2. 加载相机内参和畸变参数
 * 3. 配置GPU深度渲染器
 * 4. 加载点云数据和ground truth位姿
 * 5. 设置消息同步和订阅
 * 6. 启动主循环
 */
int main(int argc, char **argv)
{
  // ===== 初始化ROS节点 =====
  ros::init(argc, argv, "cloud_banchmark");
  ros::NodeHandle nh("~");  // 使用私有命名空间

  // ===== 加载相机内参 =====
  nh.getParam("cam_width", width);    // 图像宽度
  nh.getParam("cam_height", height);  // 图像高度
  nh.getParam("cam_fx", fx);          // x方向焦距
  nh.getParam("cam_fy", fy);          // y方向焦距
  nh.getParam("cam_cx", cx);          // 主点x坐标
  nh.getParam("cam_cy", cy);          // 主点y坐标

  // 设置深度渲染器的相机参数
  depthrender.set_para(fx, fy, cx, cy, width, height);

  // ===== 构建相机内参矩阵K =====
  // K = [fx  0  cx]
  //     [ 0 fy  cy]
  //     [ 0  0   1]
  cv_K = (cv::Mat_<float>(3, 3) << fx, 0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f);

  // ===== 加载相机畸变参数（可选） =====
  // 如果参数服务器中存在畸变系数，则进行去畸变处理
  if(nh.hasParam("cam_k1") &&
     nh.hasParam("cam_k2") &&
     nh.hasParam("cam_r1") &&
     nh.hasParam("cam_r2") )
  {
    float k1, k2, r1, r2;
    nh.getParam("cam_k1", k1);  // 径向畸变系数k1
    nh.getParam("cam_k2", k2);  // 径向畸变系数k2
    nh.getParam("cam_r1", r1);  // 切向畸变系数p1（参数名为r1）
    nh.getParam("cam_r2", r2);  // 切向畸变系数p2（参数名为r2）

    // 畸变系数向量 D = [k1, k2, p1, p2]
    cv_D = (cv::Mat_<float>(1, 4) << k1, k2, r1, r2);

    // 初始化去畸变映射表（使用快速重映射，避免每帧重复计算）
    cv::initUndistortRectifyMap(
        cv_K,                         // 输入：相机内参矩阵
        cv_D,                         // 输入：畸变系数
        cv::Mat_<double>::eye(3,3),   // 输入：矫正变换矩阵（单位矩阵表示不做额外旋转）
        cv_K,                         // 输入：新的相机内参矩阵（保持不变）
        cv::Size(width, height),      // 输入：图像尺寸
        CV_16SC2,                     // 输出：映射表类型（带符号16位整数）
        undist_map1, undist_map2);    // 输出：x和y方向的映射表
    is_distorted = true;
  }

  // 输出畸变处理状态
  if(is_distorted)
    printf("need to rectify.\n");
  else
    printf("do not need to rectify.\n");

  // ===== 初始化坐标系变换矩阵 =====
  // Vicon到机体坐标系的变换（外参标定结果）
  vicon2body << 0.33638, -0.01749,  0.94156,  0.06901,
                -0.02078, -0.99972, -0.01114, -0.02781,
                0.94150, -0.01582, -0.33665, -0.12395,
                0.0,      0.0,      0.0,      1.0;

  // 相机到机体坐标系的变换（相机外参，通常由相机标定得到）
  cam02body <<  0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
                0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
                -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
                0.0, 0.0, 0.0, 1.0;

  // 相机到世界坐标系的变换（初始化为单位矩阵，后续在回调中更新）
  cam2world = Matrix4d::Identity();

  // ===== 加载3D点云数据 =====
  string cloud_path;
  nh.getParam("cloud_path", cloud_path);
  printf("cloud file %s\n", cloud_path.c_str());

  std::fstream data_file;
  data_file.open(cloud_path.c_str(), ios::in);
  vector<float> cloud_data;  // 存储点云坐标的向量
  double x, y, z, i, r, g, b;

  // 读取点云文件，格式：x y z intensity r g b
  // 只保存xyz坐标，忽略intensity和颜色信息
  while(data_file >> x >> y >> z >> i >> r >> g >> b)
  {
    cloud_data.push_back(x);
    cloud_data.push_back(y);
    cloud_data.push_back(z);
  }
  data_file.close();
  printf("has points %d.\n", cloud_data.size() / 3 );

  // ===== 加载ground truth位姿数据 =====
  string groundtruth_path = string("/home/denny/Downloads/wkx_bag/data.txt");
  std::fstream gt_file;
  gt_file.open(groundtruth_path.c_str(), ios::in);
  read_pose(gt_file);  // 读取所有位姿数据到gt_pose_vect向量中
  gt_file.close();

  // ===== 配置GPU深度渲染器 =====
  // 将点云数据传递给深度渲染器（拷贝到GPU显存）
  depthrender.set_data(cloud_data);
  // 分配主机端内存用于存储从GPU返回的深度数据
  depth_hostptr = (int*) malloc(width * height * sizeof(int));

  // ===== 设置消息订阅和同步 =====
  // 订阅相机图像话题
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/cam0/image_raw", 30);
  // 订阅Vicon位姿话题
  message_filters::Subscriber<geometry_msgs::TransformStamped> pose_sub(nh, "/vicon/firefly_sbx/firefly_sbx", 30);
  // 创建近似时间同步器，允许最多100ms的时间差
  message_filters::Synchronizer<approx_policy> sync2(approx_policy(100), image_sub, pose_sub);
  // 注册回调函数，当图像和位姿消息同步到达时调用
  sync2.registerCallback(boost::bind(image_pose_callback, _1, _2));

  // ===== 设置发布器 =====
  // 发布原始深度图（32位浮点型）
  pub_depth = nh.advertise<sensor_msgs::Image>("depth",1000);
  // 发布彩色可视化深度图（8位RGB）
  pub_color = nh.advertise<sensor_msgs::Image>("colordepth",1000);
  // pub_posedimage = nh.advertise<sensor_msgs::Image>("posedimage",1000);  // 未使用

  // ===== 初始化图像缓冲区 =====
  undistorted_image.create(height, width, CV_8UC1);

  // ===== 创建OpenCV窗口和鼠标回调 =====
  // 创建两个窗口用于显示图像和深度图
  cv::namedWindow("bluefox_image",1);
  cv::namedWindow("depth_image",1);
  // 设置鼠标回调函数，用于交互式PnP标定（用户可点击图像选择对应点）
  setMouseCallback("bluefox_image", imageBackFunc, NULL);  // 在图像窗口中点击选择2D点
  setMouseCallback("depth_image", depthBackFunc, NULL);    // 在深度图窗口中点击选择3D点

  // 初始化vicon到leica的变换为单位矩阵（后续通过PnP求解更新）
  vicon2leica = Matrix4d::Identity();

  // ===== 主循环 =====
  while(ros::ok())
  {
    ros::spinOnce();   // 处理ROS消息回调
    cv::waitKey(30);   // 等待30ms并处理OpenCV窗口事件（必须有此调用才能显示窗口）
  }
}