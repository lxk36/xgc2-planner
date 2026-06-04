/**
 * @file decode_msgs.cpp
 * @brief 四旋翼消息解码实现文件
 *
 * 该文件实现了将原始字节数据解码为四旋翼ROS消息的功能。
 * 主要用于解析来自飞行控制器的二进制数据流，包括：
 * - 输出数据（姿态、角速度、加速度、磁场、遥控器通道等）
 * - 状态数据（循环频率、电压等）
 * - PPR输出数据（期望值、估计值、PWM信号等）
 */

#include "quadrotor_msgs/decode_msgs.h"
#include <quadrotor_msgs/comm_types.h>
#include <Eigen/Geometry>

namespace quadrotor_msgs
{

/**
 * @brief 解码输出数据
 *
 * 将原始字节数组解码为四旋翼输出数据消息，包括姿态、角速度、加速度、
 * 气压高度、磁场和遥控器通道信息。
 *
 * @param data 输入的原始字节数据向量
 * @param output 输出的解码后的OutputData消息（通过引用返回）
 * @return bool 解码成功返回true，失败返回false
 *
 * @note 该函数假设输入数据遵循OUTPUT_DATA结构体格式
 * @note 使用Asctec 2012固件的Z-Y-X欧拉角约定
 */
bool decodeOutputData(const std::vector<uint8_t> &data,
                      quadrotor_msgs::OutputData &output)
{
  struct OUTPUT_DATA output_data;

  // 检查数据大小是否匹配结构体大小
  if(data.size() != sizeof(output_data))
    return false;

  // 将字节数组复制到结构体中
  memcpy(&output_data, &data[0], sizeof(output_data));

  // 解码循环频率
  output.loop_rate = output_data.loop_rate;

  // 解码电压（从整数转换为伏特，除以1000）
  output.voltage = output_data.voltage/1e3;

  // 解码欧拉角（从整数转换为弧度）
  // 先除以100恢复为度数，再转换为弧度
  const double roll = output_data.roll/1e2 * M_PI/180;   // 滚转角（绕X轴旋转）
  const double pitch = output_data.pitch/1e2 * M_PI/180; // 俯仰角（绕Y轴旋转）
  const double yaw = output_data.yaw/1e2 * M_PI/180;     // 偏航角（绕Z轴旋转）

  // Asctec（2012固件）使用Z-Y-X欧拉角约定
  // 将欧拉角转换为四元数表示
  Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  // 填充输出消息的姿态四元数
  output.orientation.w = q.w();
  output.orientation.x = q.x();
  output.orientation.y = q.y();
  output.orientation.z = q.z();

  // 解码角速度（单位：rad/s）
  // 乘以0.0154然后转换为弧度
  output.angular_velocity.x = output_data.ang_vel[0]*0.0154*M_PI/180;
  output.angular_velocity.y = output_data.ang_vel[1]*0.0154*M_PI/180;
  output.angular_velocity.z = output_data.ang_vel[2]*0.0154*M_PI/180;

  // 解码线性加速度（单位：m/s²）
  // 先除以1000，再乘以重力加速度9.81
  output.linear_acceleration.x = output_data.acc[0]/1e3 * 9.81;
  output.linear_acceleration.y = output_data.acc[1]/1e3 * 9.81;
  output.linear_acceleration.z = output_data.acc[2]/1e3 * 9.81;

  // 解码气压高度相关数据（单位：米）
  output.pressure_dheight = output_data.dheight/1e3; // 高度变化率
  output.pressure_height = output_data.height/1e3;   // 绝对高度

  // 解码磁场数据（归一化）
  // 除以2500.0进行归一化处理
  output.magnetic_field.x = output_data.mag[0]/2500.0;
  output.magnetic_field.y = output_data.mag[1]/2500.0;
  output.magnetic_field.z = output_data.mag[2]/2500.0;

  // 解码遥控器通道数据
  // 遍历8个遥控器通道
  for(int i = 0; i < 8; i++)
  {
    output.radio_channel[i] = output_data.radio[i];
  }

  // 电机转速解码（已注释）
  // 如需要可以取消注释以解码4个电机的转速数据
  //for(int i = 0; i < 4; i++)
  //  output.motor_rpm[i] = output_data.rpm[i];

  // 解码序列号
  output.seq = output_data.seq;

  return true;
}

/**
 * @brief 解码状态数据
 *
 * 将原始字节数组解码为四旋翼状态数据消息，包括循环频率、电压和序列号。
 * 这是一个精简版的解码函数，只包含基本的状态信息。
 *
 * @param data 输入的原始字节数据向量
 * @param status 输出的解码后的StatusData消息（通过引用返回）
 * @return bool 解码成功返回true，失败返回false
 *
 * @note 该函数假设输入数据遵循STATUS_DATA结构体格式
 */
bool decodeStatusData(const std::vector<uint8_t> &data,
                      quadrotor_msgs::StatusData &status)
{
  struct STATUS_DATA status_data;

  // 检查数据大小是否匹配结构体大小
  if(data.size() != sizeof(status_data))
    return false;

  // 将字节数组复制到结构体中
  memcpy(&status_data, &data[0], sizeof(status_data));

  // 解码循环频率（Hz）
  status.loop_rate = status_data.loop_rate;

  // 解码电压（从整数转换为伏特，除以1000）
  status.voltage = status_data.voltage/1e3;

  // 解码序列号
  status.seq = status_data.seq;

  return true;
}

/**
 * @brief 解码PPR输出数据
 *
 * 将原始字节数组解码为PPR（Position-Pitch-Roll）输出数据消息。
 * PPR输出数据包含期望值、估计值和PWM信号，用于飞行控制器的调试和监控。
 *
 * @param data 输入的原始字节数据向量
 * @param output 输出的解码后的PPROutputData消息（通过引用返回）
 * @return bool 解码成功返回true，失败返回false
 *
 * @note 该函数假设输入数据遵循PPR_OUTPUT_DATA结构体格式
 * @note 各参数通过不同的缩放因子进行单位转换
 */
bool decodePPROutputData(const std::vector<uint8_t> &data,
                         quadrotor_msgs::PPROutputData &output)
{
  struct PPR_OUTPUT_DATA output_data;

  // 检查数据大小是否匹配结构体大小
  if(data.size() != sizeof(output_data))
    return false;

  // 将字节数组复制到结构体中
  memcpy(&output_data, &data[0], sizeof(output_data));

  // 解码时间戳
  output.quad_time = output_data.time;

  // 解码期望的控制量（除以10000进行单位转换）
  output.des_thrust = output_data.des_thrust*1e-4; // 期望推力
  output.des_roll = output_data.des_roll*1e-4;     // 期望滚转角
  output.des_pitch = output_data.des_pitch*1e-4;   // 期望俯仰角
  output.des_yaw = output_data.des_yaw*1e-4;       // 期望偏航角

  // 解码估计的姿态角（除以10000进行单位转换）
  output.est_roll = output_data.est_roll*1e-4;     // 估计滚转角
  output.est_pitch = output_data.est_pitch*1e-4;   // 估计俯仰角
  output.est_yaw = output_data.est_yaw*1e-4;       // 估计偏航角

  // 解码估计的角速度（除以1000进行单位转换，单位：rad/s）
  output.est_angvel_x = output_data.est_angvel_x*1e-3; // X轴角速度
  output.est_angvel_y = output_data.est_angvel_y*1e-3; // Y轴角速度
  output.est_angvel_z = output_data.est_angvel_z*1e-3; // Z轴角速度

  // 解码估计的加速度（除以10000进行单位转换，单位：m/s²）
  output.est_acc_x = output_data.est_acc_x*1e-4; // X轴加速度
  output.est_acc_y = output_data.est_acc_y*1e-4; // Y轴加速度
  output.est_acc_z = output_data.est_acc_z*1e-4; // Z轴加速度

  // 解码PWM信号（4个电机的PWM值）
  output.pwm[0] = output_data.pwm1; // 电机1 PWM
  output.pwm[1] = output_data.pwm2; // 电机2 PWM
  output.pwm[2] = output_data.pwm3; // 电机3 PWM
  output.pwm[3] = output_data.pwm4; // 电机4 PWM

  return true;
}

} // namespace quadrotor_msgs
