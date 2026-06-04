// 头文件保护宏，防止重复包含
#ifndef __QUADROTOR_MSGS_QUADROTOR_MSGS_H__
#define __QUADROTOR_MSGS_QUADROTOR_MSGS_H__

// 标准整数类型定义
#include <stdint.h>
// 标准向量容器
#include <vector>
// 四旋翼输出数据消息类型
#include <quadrotor_msgs/OutputData.h>
// 四旋翼状态数据消息类型
#include <quadrotor_msgs/StatusData.h>
// 四旋翼PPR输出数据消息类型
#include <quadrotor_msgs/PPROutputData.h>

// 四旋翼消息命名空间，包含消息解码相关函数
namespace quadrotor_msgs
{

/**
 * @brief 解码输出数据
 *
 * 将原始字节数据解码为四旋翼输出数据结构。此函数用于解析从硬件或通信接口
 * 接收到的二进制数据，转换为结构化的OutputData消息格式。
 *
 * @param data 输入参数：包含原始二进制数据的uint8_t向量
 * @param output 输出参数：解码后的OutputData消息对象引用，函数会将解析结果填充到此对象中
 * @return bool 返回true表示解码成功，false表示解码失败（数据格式错误或长度不匹配）
 */
bool decodeOutputData(const std::vector<uint8_t> &data,
                      quadrotor_msgs::OutputData &output);

/**
 * @brief 解码状态数据
 *
 * 将原始字节数据解码为四旋翼状态数据结构。此函数用于解析从飞控系统接收到的
 * 二进制状态信息，转换为结构化的StatusData消息格式，通常包含飞行器的实时状态信息。
 *
 * @param data 输入参数：包含原始二进制数据的uint8_t向量
 * @param status 输出参数：解码后的StatusData消息对象引用，函数会将解析结果填充到此对象中
 * @return bool 返回true表示解码成功，false表示解码失败（数据格式错误或长度不匹配）
 */
bool decodeStatusData(const std::vector<uint8_t> &data,
                      quadrotor_msgs::StatusData &status);

/**
 * @brief 解码PPR输出数据
 *
 * 将原始字节数据解码为四旋翼PPR（可能是Position-Pose-Rate或其他特定协议）输出数据结构。
 * 此函数用于解析特定格式的二进制数据，转换为结构化的PPROutputData消息格式。
 *
 * @param data 输入参数：包含原始二进制数据的uint8_t向量
 * @param output 输出参数：解码后的PPROutputData消息对象引用，函数会将解析结果填充到此对象中
 * @return bool 返回true表示解码成功，false表示解码失败（数据格式错误或长度不匹配）
 */
bool decodePPROutputData(const std::vector<uint8_t> &data,
                         quadrotor_msgs::PPROutputData &output);
} // namespace quadrotor_msgs

// 结束头文件保护宏
#endif
