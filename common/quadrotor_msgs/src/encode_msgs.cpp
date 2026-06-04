/**
 * @file encode_msgs.cpp
 * @brief 四旋翼消息编码实现文件
 * @details 提供将ROS消息类型编码为字节流的功能，用于四旋翼控制器的底层通信
 */

#include "quadrotor_msgs/encode_msgs.h"
#include <quadrotor_msgs/comm_types.h>

namespace quadrotor_msgs
{

/**
 * @brief 编码SO3控制命令为字节流
 * @details 将高层次的SO3（三维旋转群）控制命令编码为紧凑的字节数组，用于底层通信。
 *          SO3命令包含力、姿态四元数、增益参数等控制信息，通过缩放因子压缩数据范围
 * @param so3_command 输入的SO3控制命令，包含期望力、姿态、增益等控制参数
 * @param output 输出的字节流，编码后的二进制数据存储在此向量中
 *
 * 编码细节：
 * - 力向量：缩放系数为500
 * - 姿态四元数：缩放系数为125
 * - 旋转增益kR：缩放系数为50
 * - 角速度增益kOm：缩放系数为100
 * - 当前偏航角：缩放系数为1e4
 * - 推力修正系数kf_correction：缩放系数为1e11
 * - 角度修正：缩放系数为2500
 */
void encodeSO3Command(const quadrotor_msgs::SO3Command &so3_command,
                      std::vector<uint8_t> &output)
{
  // 创建SO3命令输入结构体，用于存储编码后的数据
  struct SO3_CMD_INPUT so3_cmd_input;

  // 编码力向量（牛顿），缩放500倍以保持精度同时减小数据范围
  so3_cmd_input.force[0] = so3_command.force.x*500;  // X轴方向的力
  so3_cmd_input.force[1] = so3_command.force.y*500;  // Y轴方向的力
  so3_cmd_input.force[2] = so3_command.force.z*500;  // Z轴方向的力

  // 编码期望姿态四元数，缩放125倍（四元数分量范围为[-1,1]）
  so3_cmd_input.des_qx = so3_command.orientation.x*125;  // 四元数X分量
  so3_cmd_input.des_qy = so3_command.orientation.y*125;  // 四元数Y分量
  so3_cmd_input.des_qz = so3_command.orientation.z*125;  // 四元数Z分量
  so3_cmd_input.des_qw = so3_command.orientation.w*125;  // 四元数W分量（实部）

  // 编码旋转增益kR（姿态误差的比例增益），缩放50倍
  so3_cmd_input.kR[0] = so3_command.kR[0]*50;  // X轴旋转增益
  so3_cmd_input.kR[1] = so3_command.kR[1]*50;  // Y轴旋转增益
  so3_cmd_input.kR[2] = so3_command.kR[2]*50;  // Z轴旋转增益

  // 编码角速度增益kOm（角速度误差的微分增益），缩放100倍
  so3_cmd_input.kOm[0] = so3_command.kOm[0]*100;  // X轴角速度增益
  so3_cmd_input.kOm[1] = so3_command.kOm[1]*100;  // Y轴角速度增益
  so3_cmd_input.kOm[2] = so3_command.kOm[2]*100;  // Z轴角速度增益

  // 编码当前偏航角（弧度），缩放1e4倍以保持高精度
  so3_cmd_input.cur_yaw = so3_command.aux.current_yaw*1e4;

  // 编码推力修正系数，缩放1e11倍（该系数通常非常小）
  so3_cmd_input.kf_correction = so3_command.aux.kf_correction*1e11;
  // 编码角度修正值（弧度），缩放2500倍
  so3_cmd_input.angle_corrections[0] = so3_command.aux.angle_corrections[0]*2500;  // Roll修正
  so3_cmd_input.angle_corrections[1] = so3_command.aux.angle_corrections[1]*2500;  // Pitch修正

  // 复制布尔标志位（无需缩放）
  so3_cmd_input.enable_motors = so3_command.aux.enable_motors;          // 电机使能标志
  so3_cmd_input.use_external_yaw = so3_command.aux.use_external_yaw;    // 使用外部偏航角标志

  // 编码序列号，对255取模以适应8位整数范围
  so3_cmd_input.seq = so3_command.header.seq % 255;

  // 调整输出向量大小以容纳编码后的结构体
  output.resize(sizeof(so3_cmd_input));
  // 将结构体数据拷贝到字节流中
  memcpy(&output[0], &so3_cmd_input, sizeof(so3_cmd_input));
}

/**
 * @brief 编码TRPY控制命令为字节流
 * @details 将推力-横滚-俯仰-偏航（Thrust-Roll-Pitch-Yaw）控制命令编码为紧凑的字节数组。
 *          TRPY命令是一种更直观的控制方式，直接指定推力和三个欧拉角
 * @param trpy_command 输入的TRPY控制命令，包含推力和姿态角信息
 * @param output 输出的字节流，编码后的二进制数据存储在此向量中
 *
 * 编码细节：
 * - 推力值：缩放系数为1e4
 * - 横滚角（Roll）：缩放系数为1e4
 * - 俯仰角（Pitch）：缩放系数为1e4
 * - 偏航角（Yaw）：缩放系数为1e4
 * - 当前偏航角：缩放系数为1e4
 */
void encodeTRPYCommand(const quadrotor_msgs::TRPYCommand &trpy_command,
                        std::vector<uint8_t> &output)
{
  // 创建TRPY命令输入结构体，用于存储编码后的数据
  struct TRPY_CMD trpy_cmd_input;

  // 编码控制量，所有值统一缩放1e4倍以保持精度
  trpy_cmd_input.thrust = trpy_command.thrust*1e4;              // 推力值（通常已归一化）
  trpy_cmd_input.roll = trpy_command.roll*1e4;                  // 横滚角（弧度）
  trpy_cmd_input.pitch = trpy_command.pitch*1e4;                // 俯仰角（弧度）
  trpy_cmd_input.yaw = trpy_command.yaw*1e4;                    // 偏航角（弧度）
  trpy_cmd_input.current_yaw = trpy_command.aux.current_yaw*1e4;// 当前偏航角（弧度）

  // 复制辅助控制标志位
  trpy_cmd_input.enable_motors = trpy_command.aux.enable_motors;        // 电机使能标志
  trpy_cmd_input.use_external_yaw = trpy_command.aux.use_external_yaw;  // 使用外部偏航角标志

  // 调整输出向量大小以容纳编码后的结构体
  output.resize(sizeof(trpy_cmd_input));
  // 将结构体数据拷贝到字节流中
  memcpy(&output[0], &trpy_cmd_input, sizeof(trpy_cmd_input));
}

/**
 * @brief 编码位置-姿态-横滚增益参数为字节流
 * @details 将PD控制器的增益参数编码为字节数组，用于配置底层控制器。
 *          PPR代表Position-Pitch-Roll（位置-俯仰-横滚）控制增益
 * @param gains 输入的增益参数结构体，包含比例和微分增益
 * @param output 输出的字节流，编码后的二进制数据存储在此向量中
 *
 * 编码细节：
 * - Kp：位置/姿态的比例增益（无缩放）
 * - Kd：位置/姿态的微分增益（无缩放）
 * - Kp_yaw：偏航角的比例增益（无缩放）
 * - Kd_yaw：偏航角的微分增益（无缩放）
 */
void encodePPRGains(const quadrotor_msgs::Gains &gains,
                    std::vector<uint8_t> &output)
{
  // 创建PPR增益结构体，用于存储编码后的数据
  struct PPR_GAINS ppr_gains;

  // 直接复制增益参数（浮点数，无需缩放）
  ppr_gains.Kp = gains.Kp;          // 位置/姿态的比例增益
  ppr_gains.Kd = gains.Kd;          // 位置/姿态的微分增益
  ppr_gains.Kp_yaw = gains.Kp_yaw;  // 偏航角的比例增益
  ppr_gains.Kd_yaw = gains.Kd_yaw;  // 偏航角的微分增益

  // 调整输出向量大小以容纳编码后的结构体
  output.resize(sizeof(ppr_gains));
  // 将结构体数据拷贝到字节流中
  memcpy(&output[0], &ppr_gains, sizeof(ppr_gains));
}

} // namespace quadrotor_msgs
