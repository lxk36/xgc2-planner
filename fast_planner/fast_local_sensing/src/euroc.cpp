/**
 * @file euroc.cpp
 * @brief EUROC数据集处理节点 - 用于同步相机图像和位姿数据，并渲染深度图
 *
 * 该文件实现了一个ROS节点，用于处理EUROC数据集的图像和位姿信息。
 * 主要功能包括：
 * 1. 同步相机图像和位姿数据
 * 2. 使用CUDA加速的深度渲染
 * 3. 图像去畸变处理
 * 4. PnP求解相机外参
 * 5. 发布深度图和彩色深度图
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

// 动态重配置相关
#include <dynamic_reconfigure/server.h>
#include <cloud_banchmark/cloud_banchmarkConfig.h>

// 点云处理相关
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

// Eigen和OpenCV相关
#include <eigen3/Eigen/Dense>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

// 深度渲染CUDA头文件
#include "depth_render.cuh"

using namespace cv;
using namespace std;
using namespace Eigen;

// 定义近似时间同步策略的类型别名，用于同步图像和位姿数据
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::TransformStamped> approx_policy;

// 深度图像相关变量
int *depth_hostptr;        // 指向主机端深度数据的指针
cv::Mat depth_mat;         // 深度图像矩阵

// 相机内参相关变量
int width, height;         // 图像的宽度和高度
cv::Mat cv_K, cv_D;        // 相机内参矩阵K和畸变系数D
double fx,fy,cx,cy;        // 焦距fx,fy和光心cx,cy
cv::Mat undist_map1, undist_map2;  // 去畸变映射表
bool is_distorted(false);  // 标识图像是否需要去畸变

// 深度渲染器对象
DepthRender depthrender;

// ROS发布器
ros::Publisher pub_depth;       // 发布深度图像
ros::Publisher pub_color;       // 发布彩色深度图像
ros::Publisher pub_posedimage;  // 发布带位姿的图像

// 坐标系变换矩阵
Matrix4d vicon2body;   // Vicon坐标系到机体坐标系的变换
Matrix4d cam02body;    // 相机坐标系到机体坐标系的变换
Matrix4d cam2world;    // 相机坐标系到世界坐标系的变换
Matrix4d vicon2leica;  // Vicon坐标系到Leica坐标系的变换

// 时间戳
ros::Time receive_stamp;

// 去畸变后的图像
cv::Mat undistorted_image;

/**
 * @brief 位姿信息结构体
 * 用于存储单个时刻的位姿信息
 */
struct PoseInfo
{
  Matrix4d pose;  // 4x4位姿变换矩阵
  double time;    // 时间戳
};

// 真值位姿向量，存储所有时刻的位姿信息
vector<PoseInfo> gt_pose_vect;

// 函数声明：渲染当前位姿对应的深度图
void render_currentpose();

/**
 * @brief 从文件读取位姿数据
 * @param file 输入文件流引用
 * @return 总是返回true
 *
 * 文件格式：每行8个数据
 * timestamp px py pz qw qx qy qz
 * 其中timestamp为时间戳，px/py/pz为位置，qw/qx/qy/qz为四元数姿态
 */
bool read_pose(fstream &file)
{
  int count = 0;
  bool good = true;
  double para[16];  // 用于存储每行读取的参数

  while(good)
  {
    count ++;
    // 读取8个参数：时间戳、位置(x,y,z)、四元数(w,x,y,z)
    for(int i = 0; i < 8 && good; i++)
    {
      good = good && file >> para[i];
    }

    if(good)
    {
      // 提取位置信息
      Eigen::Vector3d request_position;
      Eigen::Quaterniond request_pose;
      request_position.x() = para[1];
      request_position.y() = para[2];
      request_position.z() = para[3];

      // 提取姿态四元数信息
      request_pose.w() = para[4];
      request_pose.x() = para[5];
      request_pose.y() = para[6];
      request_pose.z() = para[7];

      // 构建位姿信息结构体
      PoseInfo info;
      info.time = para[0];
      info.pose = Matrix4d::Identity();
      info.pose.block<3,3>(0,0) = request_pose.toRotationMatrix();  // 设置旋转矩阵部分
      info.pose(0,3) = para[1];  // 设置平移向量x分量
      info.pose(1,3) = para[2];  // 设置平移向量y分量
      info.pose(2,3) = para[3];  // 设置平移向量z分量
      gt_pose_vect.push_back(info);
    }
  }
  printf("we have %d poses.\n", count);
  return true;
}

// PnP求解用的点对
vector<cv::Point3f> pts_3;  // 3D点（相机坐标系）
vector<cv::Point2f> pts_2;  // 2D点（图像坐标系）

/**
 * @brief 图像窗口鼠标回调函数
 * @param event 鼠标事件类型
 * @param x 鼠标点击的x坐标
 * @param y 鼠标点击的y坐标
 * @param flags 事件标志
 * @param userdata 用户数据
 *
 * 当用户在图像窗口点击时，记录2D像素坐标
 */
void imageBackFunc(int event, int x, int y, int flags, void* userdata)
{
  if( event == EVENT_LBUTTONDOWN )
  {
    cout << "image is clicked - position (" << x << ", " << y << ")" << endl;
    pts_2.push_back(cv::Point2f(x,y));
  }
}

/**
 * @brief 深度图窗口鼠标回调函数
 * @param event 鼠标事件类型
 * @param x 鼠标点击的x坐标
 * @param y 鼠标点击的y坐标
 * @param flags 事件标志
 * @param userdata 用户数据
 *
 * 当用户在深度图窗口点击时，根据深度值计算3D坐标
 */
void depthBackFunc(int event, int x, int y, int flags, void* userdata)
{
  if( event == EVENT_LBUTTONDOWN )
  {
    cout << "depth is clicked - position (" << x << ", " << y << ")" << endl;
    // 获取深度值
    double depth = depth_mat.at<float>(y,x);
    // 通过相机内参将像素坐标和深度转换为3D坐标
    double space_x = (x - cx) * depth / fx;
    double space_y = (y - cy) * depth / fy;
    double space_z = depth;
    pts_3.push_back(cv::Point3f(space_x, space_y, space_z));
  }
}

/**
 * @brief 使用PnP算法求解相机外参
 *
 * 通过用户手动点击的2D-3D点对，使用PnP算法求解相机外参标定
 * 求解Vicon坐标系到Leica坐标系的变换矩阵
 */
void solve_pnp()
{
  // 参考变换矩阵（注释中的参考值）
  //   translation :
  //   0.994976 -0.0431638  0.0903361 -0.0338185
  //  0.0444475   0.998937  -0.012246  0.0541652
  // -0.0897114  0.0161996   0.995836  0.0384018
  //          0          0          0          1

  printf("we have %d pair points.\n", pts_3.size());

  // 检查点对数量是否足够（至少需要5对点）
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
  // 使用零畸变模型进行PnP求解
  cv::solvePnP(pts_3, pts_2, cv_K, cv::Mat::zeros(4,1,CV_32FC1), rvec, t);
  // 将旋转向量转换为旋转矩阵
  cv::Rodrigues(rvec, r);

  // 将OpenCV矩阵转换为Eigen矩阵
  Matrix3d R_ref;
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
      {
          R_ref(i,j) = r.at<double>(i, j);
      }

  // 构建4x4变换矩阵
  Matrix4d pnp_result = Matrix4d::Identity();
  pnp_result.block<3,3>(0,0) = R_ref;     // 设置旋转部分
  pnp_result(0,3) = t.at<double>(0, 0);   // 设置平移x
  pnp_result(1,3) = t.at<double>(1, 0);   // 设置平移y
  pnp_result(2,3) = t.at<double>(2, 0);   // 设置平移z

  // 计算vicon到leica的变换矩阵（取逆）
  vicon2leica = pnp_result.inverse();
  cout << "translation : " << endl << pnp_result << endl;
}

/**
 * @brief 图像和位姿同步回调函数
 * @param image_input 输入图像消息
 * @param pose_input 输入位姿消息
 *
 * 该回调函数在接收到同步的图像和位姿数据时被调用
 * 主要功能：
 * 1. 计算图像和位姿的时间差
 * 2. 从真值位姿数据中找到时间最接近的位姿
 * 3. 计算相机到世界坐标系的变换
 * 4. 对图像进行去畸变处理
 * 5. 触发深度图渲染
 */
void image_pose_callback(
    const sensor_msgs::ImageConstPtr &image_input,
    const geometry_msgs::TransformStampedConstPtr &pose_input)
{
  // 计算图像和位姿的时间差
  double time_diff = fabs(image_input->header.stamp.toSec() - pose_input->header.stamp.toSec()) * 1000.0;
  printf("time diff is %lf ms.\n", time_diff);

  // 初始化接收到的位姿矩阵
  Matrix4d Pose_receive = Matrix4d::Identity();

  // 方案1：使用Vicon位姿数据（已注释）
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

  // 方案2：使用真值位姿数据（当前使用的方案）
  double image_time = image_input->header.stamp.toSec();
  double min_time_diff = 999.9;
  int min_time_index = 0;

  // 在真值位姿向量中查找时间最接近的位姿
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

  // 转换到机体坐标系位姿
  // Matrix4d body_pose = Pose_receive * vicon2body.inverse();
  Matrix4d body_pose = Pose_receive;

  // 计算相机到世界坐标系的变换
  cam2world = body_pose * cam02body * vicon2leica;

  // 保存接收时间戳
  receive_stamp = pose_input->header.stamp;

  // 处理图像：将ROS图像消息转换为OpenCV格式
  cv_bridge::CvImageConstPtr cv_img_ptr = cv_bridge::toCvShare(image_input, sensor_msgs::image_encodings::MONO8);
  cv::Mat img_8uC1 = cv_img_ptr->image;
  undistorted_image.create(height, width, CV_8UC1);

  // 如果图像有畸变，进行去畸变处理
  if(is_distorted)
  {
    cv::remap(img_8uC1, undistorted_image, undist_map1, undist_map2, CV_INTER_LINEAR);
  }
  else
    undistorted_image = img_8uC1;

  // 渲染当前位姿对应的深度图
  render_currentpose();
}

/**
 * @brief 渲染当前位姿对应的深度图
 *
 * 主要功能：
 * 1. 调用PnP求解更新相机外参
 * 2. 使用CUDA深度渲染器生成深度图
 * 3. 将深度数据转换为OpenCV矩阵格式
 * 4. 发布原始深度图
 * 5. 生成并发布彩色深度图用于可视化
 * 6. 显示图像和深度图窗口
 */
void render_currentpose()
{
  // 更新相机外参标定
  solve_pnp();

  double this_time = ros::Time::now().toSec();

  // 计算相机位姿（世界坐标系到相机坐标系）
  Matrix4d cam_pose = cam2world.inverse();

  // 使用CUDA深度渲染器渲染深度图
  depthrender.render_pose(cam_pose, depth_hostptr);

  // 初始化深度图矩阵
  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);
  double min = 0.5;
  double max = 1.0f;

  // 将深度数据从整型转换为浮点型，并找到最大深度值
  for(int i = 0; i < height; i++)
  	for(int j = 0; j < width; j++)
  	{
  		float depth = (float)depth_hostptr[i * width + j] / 1000.0f;  // 毫米转米
  		depth = depth < 500.0f ? depth : 0;  // 过滤超过500米的深度值
  		max = depth > max ? depth : max;     // 更新最大深度
  		depth_mat.at<float>(i,j) = depth;
  	}
  ROS_INFO("render cost %lf ms.", (ros::Time::now().toSec() - this_time) * 1000.0f);
  printf("max_depth %lf.\n", max);

  // 发布原始深度图
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = receive_stamp;
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_mat.clone();
  pub_depth.publish(out_msg.toImageMsg());

  // 生成归一化的深度图用于可视化
  cv::Mat adjMap;
  depth_mat.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);

  // 应用彩虹色彩映射
  cv::Mat falseColorsMap;
  cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);

  // 将深度图与原始图像混合显示
  cv::Mat bgr_image;
  cv::cvtColor(undistorted_image, bgr_image, cv::COLOR_GRAY2BGR);
  cv::addWeighted(bgr_image, 0.2, falseColorsMap, 0.8, 0.0, falseColorsMap);

  // 发布彩色深度图
  cv_bridge::CvImage cv_image_colored;
  cv_image_colored.header.frame_id = "depthmap";
  cv_image_colored.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image_colored.image = falseColorsMap;
  pub_color.publish(cv_image_colored.toImageMsg());

  // 显示图像窗口
  cv::imshow("bluefox_image", bgr_image);
  cv::imshow("depth_image", adjMap);
}

/**
 * @brief 主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 *
 * 主要功能：
 * 1. 初始化ROS节点
 * 2. 读取相机内参参数
 * 3. 初始化深度渲染器
 * 4. 配置图像去畸变参数
 * 5. 设置坐标系变换矩阵
 * 6. 读取点云数据和真值位姿
 * 7. 创建消息同步器订阅图像和位姿
 * 8. 创建深度图发布器
 * 9. 启动ROS主循环
 */
int main(int argc, char **argv)
{
  // 初始化ROS节点，节点名称为"cloud_banchmark"
  ros::init(argc, argv, "cloud_banchmark");
  ros::NodeHandle nh("~");  // 创建私有命名空间节点句柄

  // 从参数服务器读取相机参数
  nh.getParam("cam_width", width);    // 图像宽度
  nh.getParam("cam_height", height);  // 图像高度
  nh.getParam("cam_fx", fx);          // 焦距x
  nh.getParam("cam_fy", fy);          // 焦距y
  nh.getParam("cam_cx", cx);          // 光心x
  nh.getParam("cam_cy", cy);          // 光心y

  // 使用读取的参数初始化深度渲染器
  depthrender.set_para(fx, fy, cx, cy, width, height);

  // 构建相机内参矩阵K (3x3)
  // K = [fx  0  cx]
  //     [ 0 fy  cy]
  //     [ 0  0   1]
  cv_K = (cv::Mat_<float>(3, 3) << fx, 0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f);

  // 检查是否存在畸变参数
  if(nh.hasParam("cam_k1") &&
     nh.hasParam("cam_k2") &&
     nh.hasParam("cam_r1") &&
     nh.hasParam("cam_r2") )
  {
    // 读取畸变系数：k1, k2为径向畸变系数，r1, r2为切向畸变系数
    float k1, k2, r1, r2;
    nh.getParam("cam_k1", k1);
    nh.getParam("cam_k2", k2);
    nh.getParam("cam_r1", r1);
    nh.getParam("cam_r2", r2);

    // 构建畸变系数向量 (1x4)
    cv_D = (cv::Mat_<float>(1, 4) << k1, k2, r1, r2);

    // 初始化去畸变映射表
    // 该函数预先计算每个像素在去畸变后的映射关系，提高实时性能
    cv::initUndistortRectifyMap(
        cv_K,                         // 输入相机内参矩阵
        cv_D,                         // 输入畸变系数
        cv::Mat_<double>::eye(3,3),   // 可选的校正变换矩阵（单位矩阵表示不进行额外校正）
        cv_K,                         // 输出相机内参矩阵（与输入相同）
        cv::Size(width, height),      // 输出图像尺寸
        CV_16SC2,                     // 映射表数据类型
        undist_map1, undist_map2);    // 输出映射表
    is_distorted = true;  // 标记需要去畸变
  }

  // 打印畸变状态
  if(is_distorted)
    printf("need to rectify.\n");
  else
    printf("do not need to rectify.\n");

  // 初始化Vicon坐标系到机体坐标系的变换矩阵（预先标定好的外参）
  // 这是一个4x4的齐次变换矩阵，包含旋转和平移
  vicon2body << 0.33638, -0.01749,  0.94156,  0.06901,
                -0.02078, -0.99972, -0.01114, -0.02781,
                0.94150, -0.01582, -0.33665, -0.12395,
                0.0,      0.0,      0.0,      1.0;

  // 初始化相机坐标系到机体坐标系的变换矩阵（预先标定好的外参）
  cam02body <<  0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
                0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
                -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
                0.0, 0.0, 0.0, 1.0;

  // 初始化相机到世界坐标系的变换矩阵为单位矩阵（会在运行时更新）
  cam2world = Matrix4d::Identity();

  // 读取点云文件路径参数
  string cloud_path;
  nh.getParam("cloud_path", cloud_path);
  printf("cloud file %s\n", cloud_path.c_str());

  // 打开点云文件并读取数据
  std::fstream data_file;
  data_file.open(cloud_path.c_str(), ios::in);
  vector<float> cloud_data;  // 存储点云数据的向量
  double x, y, z, i, r, g, b;  // 点云的7个属性：位置(x,y,z)，强度(i)，颜色(r,g,b)

  // 逐行读取点云数据，只保存xyz坐标
	while(data_file >> x >> y >> z >> i >> r >> g >> b)
	{
		cloud_data.push_back(x);
		cloud_data.push_back(y);
		cloud_data.push_back(z);
	}
  data_file.close();
  printf("has points %d.\n", cloud_data.size() / 3 );  // 打印点云数量

  // 读取真值位姿文件（硬编码路径）
  string groundtruth_path = string("/home/denny/Downloads/wkx_bag/data.txt");
  std::fstream gt_file;
  gt_file.open(groundtruth_path.c_str(), ios::in);
  read_pose(gt_file);  // 调用函数读取所有位姿数据
  gt_file.close();

  // 将点云数据传递给深度渲染器
  depthrender.set_data(cloud_data);

  // 为深度图数据分配主机端内存
  depth_hostptr = (int*) malloc(width * height * sizeof(int));

  // 创建图像和位姿的订阅器
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/cam0/image_raw", 30);
  message_filters::Subscriber<geometry_msgs::TransformStamped> pose_sub(nh, "/vicon/firefly_sbx/firefly_sbx", 30);

  // 创建时间同步器，确保图像和位姿数据在时间上对齐
  // 使用近似时间同步策略，队列大小为100
  message_filters::Synchronizer<approx_policy> sync2(approx_policy(100), image_sub, pose_sub);
  sync2.registerCallback(boost::bind(image_pose_callback, _1, _2));  // 注册同步回调函数

  // 创建发布器：发布深度图像和彩色深度图像
  pub_depth = nh.advertise<sensor_msgs::Image>("depth",1000);
  pub_color = nh.advertise<sensor_msgs::Image>("colordepth",1000);
  // pub_posedimage = nh.advertise<sensor_msgs::Image>("posedimage",1000);  // 已注释的发布器

  // 为去畸变图像分配内存
  undistorted_image.create(height, width, CV_8UC1);

  // 创建OpenCV显示窗口
  cv::namedWindow("bluefox_image",1);  // 显示原始图像的窗口
  cv::namedWindow("depth_image",1);    // 显示深度图像的窗口

  // 设置鼠标回调函数，用于PnP标定时的点选择
  setMouseCallback("bluefox_image", imageBackFunc, NULL);  // 在图像上选择2D点
  setMouseCallback("depth_image", depthBackFunc, NULL);    // 在深度图上选择3D点

  // 初始化Vicon到Leica坐标系的变换矩阵为单位矩阵（会通过PnP求解更新）
  vicon2leica = Matrix4d::Identity();

  // ROS主循环
  while(ros::ok())
  {
    ros::spinOnce();  // 处理回调函数
    cv::waitKey(30);  // 等待30ms，用于OpenCV窗口刷新和响应键盘事件
  }
}