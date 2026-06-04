/**
 * @file encode_msgs.h
 * @brief 四旋翼消息编码工具头文件
 *
 * 本文件定义了将四旋翼控制消息编码为字节流的函数接口。
 * 这些函数用于将ROS消息格式转换为底层通信所需的字节数据。
 */

#ifndef __QUADROTOR_MSGS_QUADROTOR_MSGS_H__
#define __QUADROTOR_MSGS_QUADROTOR_MSGS_H__

#include <stdint.h>           // 标准整数类型定义
#include <vector>              // STL向量容器
#include <quadrotor_msgs/SO3Command.h>    // SO3控制指令消息
#include <quadrotor_msgs/TRPYCommand.h>   // TRPY控制指令消息
#include <quadrotor_msgs/Gains.h>         // 增益参数消息

namespace quadrotor_msgs
{

/**
 * @brief 编码SO3控制指令
 *
 * 将SO3（Special Orthogonal Group 3）旋转矩阵表示的控制指令编码为字节流。
 * SO3表示法直接使用旋转矩阵来描述四旋翼的姿态控制指令，相比欧拉角更加稳定。
 *
 * @param so3_command SO3控制指令消息，包含期望的姿态旋转矩阵和力矢量
 * @param output 输出的字节向量，存储编码后的二进制数据
 */
void encodeSO3Command(const quadrotor_msgs::SO3Command &so3_command,
                      std::vector<uint8_t> &output);

/**
 * @brief 编码TRPY控制指令
 *
 * 将TRPY（Thrust-Roll-Pitch-Yaw）控制指令编码为字节流。
 * TRPY是四旋翼的常用控制方式，直接指定推力和三个姿态角。
 *
 * @param trpy_command TRPY控制指令消息，包含推力(Thrust)、横滚角(Roll)、俯仰角(Pitch)、偏航角(Yaw)
 * @param output 输出的字节向量，存储编码后的二进制数据
 */
void encodeTRPYCommand(const quadrotor_msgs::TRPYCommand &trpy_command,
                       std::vector<uint8_t> &output);

/**
 * @brief 编码PPR增益参数
 *
 * 将位置-速度-姿态控制器的增益参数编码为字节流。
 * PPR通常指Position-Velocity-Attitude控制回路的PID增益参数。
 *
 * @param gains 增益参数消息，包含控制器的各项增益系数
 * @param output 输出的字节向量，存储编码后的二进制数据
 */
void encodePPRGains(const quadrotor_msgs::Gains &gains,
                    std::vector<uint8_t> &output);
}

#endif
