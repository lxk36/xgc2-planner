/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include <fast_plan_env/obj_predictor.h>
#include <string>

namespace fast_planner {
/* ============================== obj history_ ============================== */

// 静态成员变量定义
int ObjHistory::queue_size_;           // 历史队列大小
int ObjHistory::skip_num_;             // 跳过的帧数
ros::Time ObjHistory::global_start_time_;  // 全局起始时间

/**
 * @brief 初始化对象历史记录器
 * @param id 对象索引ID
 */
void ObjHistory::init(int id) {
  clear();         // 清空历史记录
  skip_ = 0;       // 重置跳帧计数器
  obj_idx_ = id;   // 设置对象索引
}

/**
 * @brief 位姿回调函数，记录动态障碍物的位姿历史
 * @param msg 包含位姿信息的消息指针
 *
 * 功能说明：
 * 1. 通过跳帧机制降低数据采样率
 * 2. 记录障碍物的3D位置和时间戳
 * 3. 维护固定大小的历史队列
 */
void ObjHistory::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  ++skip_;  // 跳帧计数器递增
  if (skip_ < ObjHistory::skip_num_) return;  // 未达到跳帧数则返回

  // 构建包含位置和时间戳的4维向量 [x, y, z, t]
  Eigen::Vector4d pos_t;
  pos_t(0) = msg->pose.position.x, pos_t(1) = msg->pose.position.y, pos_t(2) = msg->pose.position.z;
  pos_t(3) = (ros::Time::now() - ObjHistory::global_start_time_).toSec();  // 相对于起始时间的时间戳

  history_.push_back(pos_t);  // 将新数据加入历史队列
  // cout << "idx: " << obj_idx_ << "pos_t: " << pos_t.transpose() << endl;

  if (history_.size() > queue_size_) history_.pop_front();  // 保持队列大小固定，移除最旧数据

  skip_ = 0;  // 重置跳帧计数器
}

// ObjHistory::
/* ============================== obj predictor ==============================
 */

/**
 * @brief 默认构造函数
 */
ObjPredictor::ObjPredictor(/* args */) {
}

/**
 * @brief 带ROS节点句柄的构造函数
 * @param node ROS节点句柄
 */
ObjPredictor::ObjPredictor(ros::NodeHandle& node) {
  this->node_handle_ = node;
}

/**
 * @brief 析构函数
 */
ObjPredictor::~ObjPredictor() {
}

/**
 * @brief 初始化对象预测器
 *
 * 功能说明：
 * 1. 从参数服务器读取配置参数
 * 2. 初始化预测轨迹和对象尺寸容器
 * 3. 为每个动态障碍物订阅位姿话题
 * 4. 订阅障碍物可视化标记以获取尺寸信息
 * 5. 创建定时器定期更新预测
 */
void ObjPredictor::init() {
  /* get param */
  // 获取ROS参数：障碍物数量、正则化参数lambda、预测更新频率
  node_handle_.param("prediction/obj_num", obj_num_, 5);
  node_handle_.param("prediction/lambda", lambda_, 1.0);
  node_handle_.param("prediction/predict_rate", predict_rate_, 1.0);

  // 初始化预测轨迹容器，为每个障碍物分配空间
  predict_trajs_.reset(new vector<PolynomialPrediction>);
  predict_trajs_->resize(obj_num_);

  // 初始化障碍物尺寸容器
  obj_scale_.reset(new vector<Eigen::Vector3d>);
  obj_scale_->resize(obj_num_);
  scale_init_.resize(obj_num_);
  for (int i = 0; i < obj_num_; i++)
    scale_init_[i] = false;  // 标记尺寸未初始化

  /* subscribe to pose */
  // 为每个动态障碍物创建历史记录器并订阅其位姿话题
  for (int i = 0; i < obj_num_; i++) {
    shared_ptr<ObjHistory> obj_his(new ObjHistory);

    obj_his->init(i);  // 初始化历史记录器
    obj_histories_.push_back(obj_his);

    // 订阅话题 /dynamic/pose_i 获取第i个障碍物的位姿
    ros::Subscriber pose_sub = node_handle_.subscribe<geometry_msgs::PoseStamped>(
        "/dynamic/pose_" + std::to_string(i), 10, &ObjHistory::poseCallback, obj_his.get());

    pose_subs_.push_back(pose_sub);
  }

  // 订阅障碍物可视化标记话题以获取障碍物尺寸信息
  marker_sub_ = node_handle_.subscribe<visualization_msgs::Marker>("/dynamic/obj", 10,
                                                                   &ObjPredictor::markerCallback, this);

  /* update prediction */
  // 创建定时器，按指定频率定期调用预测回调函数
  predict_timer_ =
      node_handle_.createTimer(ros::Duration(1 / predict_rate_), &ObjPredictor::predictCallback, this);
}

/**
 * @brief 获取预测轨迹
 * @return 返回所有障碍物的预测轨迹
 */
ObjPrediction ObjPredictor::getPredictionTraj() {
  return this->predict_trajs_;
}

/**
 * @brief 获取障碍物尺寸信息
 * @return 返回所有障碍物的尺寸向量
 */
ObjScale ObjPredictor::getObjScale() {
  return this->obj_scale_;
}

/**
 * @brief 使用多项式拟合方法预测障碍物轨迹
 *
 * 算法说明：
 * 1. 使用5阶多项式拟合历史位置数据: p(t) = c0 + c1*t + c2*t^2 + ... + c5*t^5
 * 2. 优化目标包含两部分：
 *    a) 最小化拟合误差（位置误差）
 *    b) 最小化加速度变化（正则化项，通过lambda参数控制）
 * 3. 分别对x、y、z三个维度独立求解
 * 4. 使用最小二乘法求解超定方程组 A*p = b
 */
void ObjPredictor::predictPolyFit() {
  /* iterate all obj */
  // 遍历所有动态障碍物
  for (int i = 0; i < obj_num_; i++) {
    /* ---------- write A and b ---------- */
    // 构建最小二乘问题的系数矩阵A和右端项b
    Eigen::Matrix<double, 6, 6> A;      // 6x6系数矩阵
    Eigen::Matrix<double, 6, 1> temp;   // 临时向量
    Eigen::Matrix<double, 6, 1> bm[3];  // 右端项向量（x、y、z三个维度）
    vector<Eigen::Matrix<double, 6, 1>> pm(3);  // 多项式系数（x、y、z三个维度）

    A.setZero();
    for (int i = 0; i < 3; ++i)
      bm[i].setZero();

    /* ---------- estimation error ---------- */
    // 第一部分：最小化拟合误差项
    // 误差函数: E = sum((p(ti) - qi)^2) 其中qi是历史观测位置，ti是对应时间
    list<Eigen::Vector4d> his;
    obj_histories_[i]->getHistory(his);  // 获取该障碍物的历史位姿数据
    for (list<Eigen::Vector4d>::iterator it = his.begin(); it != his.end(); ++it) {
      Eigen::Vector3d qi = (*it).head(3);  // 历史位置 [x, y, z]
      double ti = (*it)(3);                // 时间戳

      /* A */
      // 构建多项式基函数向量: [1, t, t^2, t^3, t^4, t^5]
      temp << 1.0, ti, pow(ti, 2), pow(ti, 3), pow(ti, 4), pow(ti, 5);
      // 累加到系数矩阵A: A += 2 * [1, t, t^2, ..., t^5]^T * [1, t, t^2, ..., t^5]
      for (int j = 0; j < 6; ++j)
        A.row(j) += 2.0 * pow(ti, j) * temp.transpose();

      /* b */
      // 累加到右端项: b += 2 * qi * [1, t, t^2, ..., t^5]
      for (int dim = 0; dim < 3; ++dim)
        bm[dim] += 2.0 * qi(dim) * temp;
    }

    /* ---------- acceleration regulator ---------- */
    // 第二部分：加速度正则化项（平滑性约束）
    // 目标：最小化加速度的积分，使轨迹更平滑
    // 加速度: a(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + 20*c5*t^3
    // 最小化: lambda * integral(a(t)^2, t1, t2)
    double t1 = his.front()(3);  // 起始时间
    double t2 = his.back()(3);   // 结束时间

    // 加速度对多项式系数c2的积分贡献
    // d(a)/d(c2) = 2, 积分后对系数矩阵的贡献
    temp << 0.0, 0.0, 2 * t1 - 2 * t2, 3 * pow(t1, 2) - 3 * pow(t2, 2), 4 * pow(t1, 3) - 4 * pow(t2, 3),
        5 * pow(t1, 4) - 5 * pow(t2, 4);
    A.row(2) += -4 * lambda_ * temp.transpose();

    // 加速度对多项式系数c3的积分贡献
    // d(a)/d(c3) = 6*t
    temp << 0.0, 0.0, pow(t1, 2) - pow(t2, 2), 2 * pow(t1, 3) - 2 * pow(t2, 3),
        3 * pow(t1, 4) - 3 * pow(t2, 4), 4 * pow(t1, 5) - 4 * pow(t2, 5);
    A.row(3) += -12 * lambda_ * temp.transpose();

    // 加速度对多项式系数c4的积分贡献
    // d(a)/d(c4) = 12*t^2
    temp << 0.0, 0.0, 20 * pow(t1, 3) - 20 * pow(t2, 3), 45 * pow(t1, 4) - 45 * pow(t2, 4),
        72 * pow(t1, 5) - 72 * pow(t2, 5), 100 * pow(t1, 6) - 100 * pow(t2, 6);
    A.row(4) += -4.0 / 5.0 * lambda_ * temp.transpose();

    // 加速度对多项式系数c5的积分贡献
    // d(a)/d(c5) = 20*t^3
    temp << 0.0, 0.0, 35 * pow(t1, 4) - 35 * pow(t2, 4), 84 * pow(t1, 5) - 84 * pow(t2, 5),
        140 * pow(t1, 6) - 140 * pow(t2, 6), 200 * pow(t1, 7) - 200 * pow(t2, 7);
    A.row(5) += -4.0 / 7.0 * lambda_ * temp.transpose();

    /* ---------- solve ---------- */
    // 使用列主元QR分解求解线性方程组 A*pm = bm
    // 分别求解x、y、z三个维度的多项式系数
    for (int j = 0; j < 3; j++) {
      pm[j] = A.colPivHouseholderQr().solve(bm[j]);
    }

    /* ---------- update prediction container ---------- */
    // 更新预测轨迹容器，存储多项式系数和时间范围
    predict_trajs_->at(i).setPolynomial(pm);  // 设置多项式系数
    predict_trajs_->at(i).setTime(t1, t2);    // 设置有效时间范围
  }
}

/**
 * @brief 预测定时器回调函数
 * @param e 定时器事件
 *
 * 功能说明：
 * 定期调用预测算法更新障碍物未来轨迹
 * 当前使用恒定速度模型（predictConstVel），也可切换为多项式拟合模型（predictPolyFit）
 */
void ObjPredictor::predictCallback(const ros::TimerEvent& e) {
  // predictPolyFit();  // 可选：使用多项式拟合方法预测
  predictConstVel();    // 使用恒定速度模型预测
}

/**
 * @brief 可视化标记回调函数，用于获取障碍物尺寸信息
 * @param msg 可视化标记消息指针
 *
 * 功能说明：
 * 1. 从RViz的Marker消息中提取障碍物尺寸（长、宽、高）
 * 2. 当所有障碍物的尺寸都已接收后，自动停止订阅以节省资源
 */
void ObjPredictor::markerCallback(const visualization_msgs::MarkerConstPtr& msg) {
  int idx = msg->id;  // 障碍物索引
  // 提取障碍物的三维尺寸
  (*obj_scale_)[idx](0) = msg->scale.x;  // 长度
  (*obj_scale_)[idx](1) = msg->scale.y;  // 宽度
  (*obj_scale_)[idx](2) = msg->scale.z;  // 高度

  scale_init_[idx] = true;  // 标记该障碍物尺寸已初始化

  // 统计已完成初始化的障碍物数量
  int finish_num = 0;
  for (int i = 0; i < obj_num_; i++) {
    if (scale_init_[i]) finish_num++;
  }

  // 所有障碍物尺寸都已获取，关闭订阅以节省资源
  if (finish_num == obj_num_) {
    marker_sub_.shutdown();
  }
}

/**
 * @brief 使用恒定速度模型预测障碍物轨迹
 *
 * 算法说明：
 * 1. 假设障碍物以恒定速度运动
 * 2. 使用最近的两个历史位置点计算速度
 * 3. 通过线性插值/外推预测未来轨迹
 * 4. 预测模型: p(t) = p0 + v*t，其中v = (q2-q1)/(t2-t1)
 * 5. 使用一阶多项式表示: p(t) = c0 + c1*t
 */
void ObjPredictor::predictConstVel() {
  // 遍历所有动态障碍物
  for (int i = 0; i < obj_num_; i++) {
    /* ---------- get the last two point ---------- */
    // 获取历史位姿数据
    list<Eigen::Vector4d> his;
    obj_histories_[i]->getHistory(his);
    list<Eigen::Vector4d>::iterator list_it = his.end();

    /* ---------- test iteration ---------- */
    // cout << "----------------------------" << endl;
    // for (auto v4d : his)
    //   cout << "v4d: " << v4d.transpose() << endl;

    Eigen::Vector3d q1, q2;  // 最近的两个历史位置
    double t1, t2;           // 对应的时间戳

    // 获取最后一个位置点（最新）
    --list_it;
    q2 = (*list_it).head(3);  // 位置2
    t2 = (*list_it)(3);       // 时间2

    // 获取倒数第二个位置点
    --list_it;
    q1 = (*list_it).head(3);  // 位置1
    t1 = (*list_it)(3);       // 时间1

    // 构建线性方程组求解一阶多项式系数
    // 对于一阶多项式 p(t) = c0 + c1*t
    // 已知条件: p(t1) = q1, p(t2) = q2
    // 转化为矩阵形式: [1 t1] [c0]   [q1]
    //               [1 t2] [c1] = [q2]
    Eigen::Matrix<double, 2, 3> p01, q12;
    q12.row(0) = q1.transpose();  // 第一个观测位置
    q12.row(1) = q2.transpose();  // 第二个观测位置

    // 时间矩阵
    Eigen::Matrix<double, 2, 2> At12;
    At12 << 1, t1, 1, t2;

    // 求解多项式系数: p01 = At12^(-1) * q12
    // p01的第一行是c0（常数项），第二行是c1（速度项）
    p01 = At12.inverse() * q12;

    // 构建6维多项式向量（高阶项为0）
    vector<Eigen::Matrix<double, 6, 1>> polys(3);
    for (int j = 0; j < 3; ++j) {
      polys[j].setZero();        // 初始化为零
      polys[j].head(2) = p01.col(j);  // 只设置c0和c1，其余高阶系数为0
    }

    // 更新预测轨迹容器
    predict_trajs_->at(i).setPolynomial(polys);  // 设置多项式系数
    predict_trajs_->at(i).setTime(t1, t2);       // 设置时间范围
  }
}

// ObjPredictor::
}  // namespace fast_planner