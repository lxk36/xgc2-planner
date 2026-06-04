/**
 * @file comm_types.h
 * @brief 四旋翼通信数据类型定义
 * @details 定义了四旋翼飞行器与地面站之间通信使用的各种数据结构，
 *          包括控制指令、状态数据、输出数据等。所有数据都使用定点数表示以节省带宽。
 */

#ifndef __QUADROTOR_MSGS_COMM_TYPES_H__
#define __QUADROTOR_MSGS_COMM_TYPES_H__

// SO3控制指令类型标识符
#define TYPE_SO3_CMD 's'

/**
 * @struct SO3_CMD_INPUT
 * @brief SO3(特殊正交群)控制指令输入结构体
 * @details 使用SO3姿态表示的四旋翼控制指令，采用四元数描述期望姿态，
 *          并包含姿态控制增益和电机使能等信息。所有数值都经过缩放以使用整数类型传输。
 */
struct SO3_CMD_INPUT
{
  // 解码时的缩放因子说明
  int16_t force[3];      ///< 三轴期望力 [N]，解码时除以500恢复实际值
  int8_t des_qx, des_qy, des_qz, des_qw; ///< 期望姿态四元数 (x,y,z,w)，解码时除以125恢复实际值
  uint8_t kR[3];         ///< 姿态控制增益 (roll, pitch, yaw)，解码时除以50恢复实际值
  uint8_t kOm[3];        ///< 角速度控制增益 (roll, pitch, yaw)，解码时除以100恢复实际值
  int16_t cur_yaw;       ///< 当前偏航角 [rad]，解码时除以10000恢复实际值
  int16_t kf_correction; ///< 推力系数修正值，解码时除以1e11恢复实际值
  uint8_t angle_corrections[2]; ///< 角度修正值 [roll, pitch]，解码时除以2500恢复实际值
  uint8_t enable_motors:1;      ///< 电机使能标志位，1表示使能，0表示失能
  uint8_t use_external_yaw:1;   ///< 使用外部偏航角标志位，1表示使用外部提供的偏航角
  uint8_t seq;           ///< 序列号，用于检测数据包丢失和顺序
};

// 状态数据类型标识符
#define TYPE_STATUS_DATA 'c'

/**
 * @struct STATUS_DATA
 * @brief 飞行器状态数据结构体
 * @details 包含飞行器的基本运行状态信息，如控制循环频率、电池电压等
 */
struct STATUS_DATA
{
  uint16_t loop_rate; ///< 控制循环频率 [Hz]，表示飞控系统的运行频率
  uint16_t voltage;   ///< 电池电压 [mV]，用于监测电池电量
  uint8_t seq;        ///< 序列号，用于检测数据包丢失和顺序
};

// 输出数据类型标识符
#define TYPE_OUTPUT_DATA 'd'

/**
 * @struct OUTPUT_DATA
 * @brief 飞行器传感器输出数据结构体
 * @details 包含飞行器所有传感器的测量数据，包括IMU、磁力计、高度计、
 *          遥控器通道值等，用于状态估计和监控
 */
struct OUTPUT_DATA
{
  uint16_t loop_rate;       ///< 控制循环频率 [Hz]
  uint16_t voltage;         ///< 电池电压 [mV]
  int16_t roll, pitch, yaw; ///< 姿态角：横滚、俯仰、偏航 [0.01 rad]
  int16_t ang_vel[3];       ///< 三轴角速度 [x, y, z]，单位为 [0.01 rad/s]
  int16_t acc[3];           ///< 三轴加速度 [x, y, z]，单位为 [0.01 m/s²]
  int16_t dheight;          ///< 高度变化率（垂直速度） [mm/s]
  int32_t height;           ///< 绝对高度 [mm]，由气压计或其他高度传感器测量
  int16_t mag[3];           ///< 三轴磁力计读数 [x, y, z]，用于航向估计
  uint8_t radio[8];         ///< 遥控器8个通道的PWM值
  //uint8_t rpm[4];         ///< 四个电机的转速（已注释）
  uint8_t seq;              ///< 序列号，用于检测数据包丢失和顺序
};

// TRPY控制指令类型标识符
#define TYPE_TRPY_CMD 'p'

/**
 * @struct TRPY_CMD
 * @brief TRPY (Thrust-Roll-Pitch-Yaw) 控制指令结构体
 * @details 使用推力和欧拉角表示的四旋翼控制指令，这是一种常见的四旋翼控制方式，
 *          直接指定推力和三个姿态角的期望值
 */
struct TRPY_CMD
{
  int16_t thrust;       ///< 期望推力值，控制飞行器的垂直升力
  int16_t roll;         ///< 期望横滚角 [0.01 rad]，控制左右倾斜
  int16_t pitch;        ///< 期望俯仰角 [0.01 rad]，控制前后倾斜
  int16_t yaw;          ///< 期望偏航角 [0.01 rad]，控制水平旋转方向
  int16_t current_yaw;  ///< 当前偏航角 [0.01 rad]，用于偏航角控制的反馈
  uint8_t enable_motors:1;      ///< 电机使能标志位，1表示使能，0表示失能
  uint8_t use_external_yaw:1;   ///< 使用外部偏航角标志位，1表示使用外部提供的偏航角
};

// PPR输出数据类型标识符
#define TYPE_PPR_OUTPUT_DATA 't'

/**
 * @struct PPR_OUTPUT_DATA
 * @brief PPR (Proportional-Proportional-Rate) 控制器输出数据结构体
 * @details 包含PPR控制器的期望值、估计值和PWM输出，用于调试和监控控制性能。
 *          该结构体同时记录了期望状态、估计状态和实际输出，便于分析控制效果
 */
struct PPR_OUTPUT_DATA
{
  uint16_t time;        ///< 时间戳 [ms]，用于数据同步和时序分析
  int16_t des_thrust;   ///< 期望推力值
  int16_t des_roll;     ///< 期望横滚角 [0.01 rad]
  int16_t des_pitch;    ///< 期望俯仰角 [0.01 rad]
  int16_t des_yaw;      ///< 期望偏航角 [0.01 rad]
  int16_t est_roll;     ///< 估计的横滚角 [0.01 rad]，由状态估计器提供
  int16_t est_pitch;    ///< 估计的俯仰角 [0.01 rad]，由状态估计器提供
  int16_t est_yaw;      ///< 估计的偏航角 [0.01 rad]，由状态估计器提供
  int16_t est_angvel_x; ///< 估计的X轴角速度 [0.01 rad/s]
  int16_t est_angvel_y; ///< 估计的Y轴角速度 [0.01 rad/s]
  int16_t est_angvel_z; ///< 估计的Z轴角速度 [0.01 rad/s]
  int16_t est_acc_x;    ///< 估计的X轴加速度 [0.01 m/s²]
  int16_t est_acc_y;    ///< 估计的Y轴加速度 [0.01 m/s²]
  int16_t est_acc_z;    ///< 估计的Z轴加速度 [0.01 m/s²]
  uint16_t pwm1;        ///< 电机1的PWM输出值 [μs]，通常范围为1000-2000
  uint16_t pwm2;        ///< 电机2的PWM输出值 [μs]
  uint16_t pwm3;        ///< 电机3的PWM输出值 [μs]
  uint16_t pwm4;        ///< 电机4的PWM输出值 [μs]
};

// PPR增益参数类型标识符
#define TYPE_PPR_GAINS 'g'

/**
 * @struct PPR_GAINS
 * @brief PPR控制器增益参数结构体
 * @details 定义PPR控制器的比例(P)和微分(D)增益参数，用于姿态控制和偏航控制。
 *          这些增益参数决定了控制器的响应速度和稳定性
 */
struct PPR_GAINS
{
  int16_t Kp;      ///< 姿态角(横滚/俯仰)的比例增益，控制角度误差的响应强度
  int16_t Kd;      ///< 姿态角速度(横滚/俯仰)的微分增益，控制角速度误差的响应强度
  int16_t Kp_yaw;  ///< 偏航角的比例增益，控制偏航角误差的响应强度
  int16_t Kd_yaw;  ///< 偏航角速度的微分增益，控制偏航角速度误差的响应强度
};

#endif // __QUADROTOR_MSGS_COMM_TYPES_H__
