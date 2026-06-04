/**
 * Copyright 1993-2013 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 *
 */

/*
 *  This file implements common mathematical operations on vector types
 *  (float3, float4 etc.) since these are not provided as standard by CUDA.
 *
 *  The syntax is modeled on the Cg standard library.
 *
 *  This is part of the Helper library includes
 *
 *    Thanks to Linh Hah for additions and fixes.
 */

/*
 * CUDA数学辅助库头文件
 *
 * 功能概述：
 * 本文件为CUDA向量类型(float2, float3, float4, int2, int3, int4等)提供了完整的数学运算支持。
 * CUDA标准库不提供这些向量类型的运算符重载，因此本库实现了以下功能：
 *
 * 1. 向量构造函数 - 从不同类型和维度创建向量
 * 2. 一元运算符 - 取负
 * 3. 二元运算符 - 加、减、乘、除及其复合赋值形式
 * 4. 数学函数 - min, max, clamp, lerp等
 * 5. 向量运算 - 点积、叉积、长度、归一化等
 * 6. 反射和插值 - reflect, smoothstep等高级函数
 *
 * 支持的向量类型：
 * - float2, float3, float4 (浮点型向量)
 * - int2, int3, int4 (整型向量)
 * - uint2, uint3, uint4 (无符号整型向量)
 *
 * 注意：
 * - __host__ __device__ 修饰符表示函数可在CPU和GPU上运行
 * - inline关键字确保函数被内联以提高性能
 */

#ifndef HELPER_MATH_H
#define HELPER_MATH_H

#include "cuda_runtime.h"

// 类型定义：为了代码简洁性定义无符号类型别名
typedef unsigned int uint;     // 无符号整型
typedef unsigned short ushort;  // 无符号短整型

#ifndef EXIT_WAIVED
#define EXIT_WAIVED 2  // 退出代码：表示测试被豁免
#endif

#ifndef __CUDACC__
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
// host implementations of CUDA functions
// CPU端CUDA函数实现
//
// 说明：当不使用CUDA编译器时，需要为某些CUDA特有函数提供CPU版本的实现
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 返回两个浮点数中的最小值
 * @param a 第一个浮点数
 * @param b 第二个浮点数
 * @return 较小的值
 */
inline float fminf(float a, float b)
{
    return a < b ? a : b;
}

/**
 * @brief 返回两个浮点数中的最大值
 * @param a 第一个浮点数
 * @param b 第二个浮点数
 * @return 较大的值
 */
inline float fmaxf(float a, float b)
{
    return a > b ? a : b;
}

/**
 * @brief 返回两个整数中的最大值
 * @param a 第一个整数
 * @param b 第二个整数
 * @return 较大的值
 */
inline int max(int a, int b)
{
    return a > b ? a : b;
}

/**
 * @brief 返回两个整数中的最小值
 * @param a 第一个整数
 * @param b 第二个整数
 * @return 较小的值
 */
inline int min(int a, int b)
{
    return a < b ? a : b;
}

/**
 * @brief 计算平方根的倒数（快速倒数平方根）
 * @param x 输入值
 * @return 1/sqrt(x)
 * @note 这个函数在向量归一化中非常有用
 */
inline float rsqrtf(float x)
{
    return 1.0f / sqrtf(x);
}
#endif

////////////////////////////////////////////////////////////////////////////////
// constructors
// 向量构造函数
//
// 说明：提供多种方式创建向量，支持类型转换和维度变换
// 这些构造函数允许从单个值、其他向量类型或不同维度的向量创建新向量
////////////////////////////////////////////////////////////////////////////////

// float2 构造函数组

/**
 * @brief 从单个浮点数创建float2向量，两个分量值相同
 * @param s 标量值
 * @return float2(s, s)
 */
inline __host__ __device__ float2 make_float2(float s)
{
    return make_float2(s, s);
}

/**
 * @brief 从float3向量创建float2向量，取前两个分量
 * @param a float3向量
 * @return float2(a.x, a.y)
 */
inline __host__ __device__ float2 make_float2(float3 a)
{
    return make_float2(a.x, a.y);
}

/**
 * @brief 从int2向量创建float2向量，进行类型转换
 * @param a int2向量
 * @return float2(float(a.x), float(a.y))
 */
inline __host__ __device__ float2 make_float2(int2 a)
{
    return make_float2(float(a.x), float(a.y));
}

/**
 * @brief 从uint2向量创建float2向量，进行类型转换
 * @param a uint2向量
 * @return float2(float(a.x), float(a.y))
 */
inline __host__ __device__ float2 make_float2(uint2 a)
{
    return make_float2(float(a.x), float(a.y));
}

// int2 构造函数组

/**
 * @brief 从单个整数创建int2向量，两个分量值相同
 * @param s 标量值
 * @return int2(s, s)
 */
inline __host__ __device__ int2 make_int2(int s)
{
    return make_int2(s, s);
}

/**
 * @brief 从int3向量创建int2向量，取前两个分量
 */
inline __host__ __device__ int2 make_int2(int3 a)
{
    return make_int2(a.x, a.y);
}

/**
 * @brief 从uint2向量创建int2向量，进行类型转换
 */
inline __host__ __device__ int2 make_int2(uint2 a)
{
    return make_int2(int(a.x), int(a.y));
}

/**
 * @brief 从float2向量创建int2向量，进行类型转换（截断小数部分）
 */
inline __host__ __device__ int2 make_int2(float2 a)
{
    return make_int2(int(a.x), int(a.y));
}

// uint2 构造函数组

/**
 * @brief 从单个无符号整数创建uint2向量，两个分量值相同
 * @param s 标量值
 * @return uint2(s, s)
 */
inline __host__ __device__ uint2 make_uint2(uint s)
{
    return make_uint2(s, s);
}

/**
 * @brief 从uint3向量创建uint2向量，取前两个分量
 */
inline __host__ __device__ uint2 make_uint2(uint3 a)
{
    return make_uint2(a.x, a.y);
}

/**
 * @brief 从int2向量创建uint2向量，进行类型转换
 */
inline __host__ __device__ uint2 make_uint2(int2 a)
{
    return make_uint2(uint(a.x), uint(a.y));
}

// float3 构造函数组（三维浮点向量）

/**
 * @brief 从单个浮点数创建float3向量，三个分量值相同
 * @param s 标量值
 * @return float3(s, s, s)
 */
inline __host__ __device__ float3 make_float3(float s)
{
    return make_float3(s, s, s);
}

/**
 * @brief 从float2向量创建float3向量，z分量设为0
 * @param a float2向量
 * @return float3(a.x, a.y, 0.0f)
 */
inline __host__ __device__ float3 make_float3(float2 a)
{
    return make_float3(a.x, a.y, 0.0f);
}

/**
 * @brief 从float2向量和一个标量创建float3向量
 * @param a float2向量，提供x和y分量
 * @param s 标量值，作为z分量
 * @return float3(a.x, a.y, s)
 */
inline __host__ __device__ float3 make_float3(float2 a, float s)
{
    return make_float3(a.x, a.y, s);
}

/**
 * @brief 从float4向量创建float3向量，取前三个分量
 * @param a float4向量
 * @return float3(a.x, a.y, a.z)
 */
inline __host__ __device__ float3 make_float3(float4 a)
{
    return make_float3(a.x, a.y, a.z);
}

/**
 * @brief 从int3向量创建float3向量，进行类型转换
 */
inline __host__ __device__ float3 make_float3(int3 a)
{
    return make_float3(float(a.x), float(a.y), float(a.z));
}

/**
 * @brief 从uint3向量创建float3向量，进行类型转换
 */
inline __host__ __device__ float3 make_float3(uint3 a)
{
    return make_float3(float(a.x), float(a.y), float(a.z));
}

// int3 构造函数组（三维整型向量）

/** @brief 从单个整数创建int3向量 */
inline __host__ __device__ int3 make_int3(int s)
{
    return make_int3(s, s, s);
}

/** @brief 从int2向量创建int3向量，z分量设为0 */
inline __host__ __device__ int3 make_int3(int2 a)
{
    return make_int3(a.x, a.y, 0);
}

/** @brief 从int2向量和标量创建int3向量 */
inline __host__ __device__ int3 make_int3(int2 a, int s)
{
    return make_int3(a.x, a.y, s);
}

/** @brief 从uint3向量创建int3向量，类型转换 */
inline __host__ __device__ int3 make_int3(uint3 a)
{
    return make_int3(int(a.x), int(a.y), int(a.z));
}

/** @brief 从float3向量创建int3向量，类型转换 */
inline __host__ __device__ int3 make_int3(float3 a)
{
    return make_int3(int(a.x), int(a.y), int(a.z));
}

// uint3 构造函数组（三维无符号整型向量）

/** @brief 从单个无符号整数创建uint3向量 */
inline __host__ __device__ uint3 make_uint3(uint s)
{
    return make_uint3(s, s, s);
}

/** @brief 从uint2向量创建uint3向量，z分量设为0 */
inline __host__ __device__ uint3 make_uint3(uint2 a)
{
    return make_uint3(a.x, a.y, 0);
}

/** @brief 从uint2向量和标量创建uint3向量 */
inline __host__ __device__ uint3 make_uint3(uint2 a, uint s)
{
    return make_uint3(a.x, a.y, s);
}

/** @brief 从uint4向量创建uint3向量，取前三个分量 */
inline __host__ __device__ uint3 make_uint3(uint4 a)
{
    return make_uint3(a.x, a.y, a.z);
}

/** @brief 从int3向量创建uint3向量，类型转换 */
inline __host__ __device__ uint3 make_uint3(int3 a)
{
    return make_uint3(uint(a.x), uint(a.y), uint(a.z));
}

// float4 构造函数组（四维浮点向量，常用于齐次坐标或RGBA颜色）

/** @brief 从单个浮点数创建float4向量 */
inline __host__ __device__ float4 make_float4(float s)
{
    return make_float4(s, s, s, s);
}

/** @brief 从float3向量创建float4向量，w分量设为0 */
inline __host__ __device__ float4 make_float4(float3 a)
{
    return make_float4(a.x, a.y, a.z, 0.0f);
}

/** @brief 从float3向量和标量创建float4向量 */
inline __host__ __device__ float4 make_float4(float3 a, float w)
{
    return make_float4(a.x, a.y, a.z, w);
}

/** @brief 从int4向量创建float4向量，类型转换 */
inline __host__ __device__ float4 make_float4(int4 a)
{
    return make_float4(float(a.x), float(a.y), float(a.z), float(a.w));
}

/** @brief 从uint4向量创建float4向量，类型转换 */
inline __host__ __device__ float4 make_float4(uint4 a)
{
    return make_float4(float(a.x), float(a.y), float(a.z), float(a.w));
}

// int4 构造函数组（四维整型向量）

/** @brief 从单个整数创建int4向量 */
inline __host__ __device__ int4 make_int4(int s)
{
    return make_int4(s, s, s, s);
}

/** @brief 从int3向量创建int4向量，w分量设为0 */
inline __host__ __device__ int4 make_int4(int3 a)
{
    return make_int4(a.x, a.y, a.z, 0);
}

/** @brief 从int3向量和标量创建int4向量 */
inline __host__ __device__ int4 make_int4(int3 a, int w)
{
    return make_int4(a.x, a.y, a.z, w);
}

/** @brief 从uint4向量创建int4向量，类型转换 */
inline __host__ __device__ int4 make_int4(uint4 a)
{
    return make_int4(int(a.x), int(a.y), int(a.z), int(a.w));
}

/** @brief 从float4向量创建int4向量，类型转换 */
inline __host__ __device__ int4 make_int4(float4 a)
{
    return make_int4(int(a.x), int(a.y), int(a.z), int(a.w));
}

// uint4 构造函数组（四维无符号整型向量）

/** @brief 从单个无符号整数创建uint4向量 */
inline __host__ __device__ uint4 make_uint4(uint s)
{
    return make_uint4(s, s, s, s);
}

/** @brief 从uint3向量创建uint4向量，w分量设为0 */
inline __host__ __device__ uint4 make_uint4(uint3 a)
{
    return make_uint4(a.x, a.y, a.z, 0);
}

/** @brief 从uint3向量和标量创建uint4向量 */
inline __host__ __device__ uint4 make_uint4(uint3 a, uint w)
{
    return make_uint4(a.x, a.y, a.z, w);
}

/** @brief 从int4向量创建uint4向量，类型转换 */
inline __host__ __device__ uint4 make_uint4(int4 a)
{
    return make_uint4(uint(a.x), uint(a.y), uint(a.z), uint(a.w));
}

////////////////////////////////////////////////////////////////////////////////
// negate
// 取负运算符
//
// 说明：为向量类型重载一元负号运算符，返回各分量取负后的向量
// 用法：float3 negVec = -vec;
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief float2向量取负运算
 * @param a 输入向量
 * @return 各分量取负后的向量
 */
inline __host__ __device__ float2 operator-(float2 &a)
{
    return make_float2(-a.x, -a.y);
}

/**
 * @brief int2向量取负运算
 */
inline __host__ __device__ int2 operator-(int2 &a)
{
    return make_int2(-a.x, -a.y);
}

/**
 * @brief float3向量取负运算
 */
inline __host__ __device__ float3 operator-(float3 &a)
{
    return make_float3(-a.x, -a.y, -a.z);
}

/**
 * @brief int3向量取负运算
 */
inline __host__ __device__ int3 operator-(int3 &a)
{
    return make_int3(-a.x, -a.y, -a.z);
}

/**
 * @brief float4向量取负运算
 */
inline __host__ __device__ float4 operator-(float4 &a)
{
    return make_float4(-a.x, -a.y, -a.z, -a.w);
}

/**
 * @brief int4向量取负运算
 */
inline __host__ __device__ int4 operator-(int4 &a)
{
    return make_int4(-a.x, -a.y, -a.z, -a.w);
}

////////////////////////////////////////////////////////////////////////////////
// addition
// 加法运算符
//
// 说明：为向量类型重载加法运算符，支持：
// 1. 向量与向量相加（对应分量相加）
// 2. 向量与标量相加（每个分量加上标量）
// 3. 复合赋值运算符 +=
////////////////////////////////////////////////////////////////////////////////

// float2 加法运算符组

/**
 * @brief float2向量加法：v1 + v2
 * @param a 第一个向量
 * @param b 第二个向量
 * @return 对应分量相加的结果向量
 */
inline __host__ __device__ float2 operator+(float2 a, float2 b)
{
    return make_float2(a.x + b.x, a.y + b.y);
}

/**
 * @brief float2向量复合赋值加法：v1 += v2
 * @param a 被修改的向量（引用）
 * @param b 加数向量
 */
inline __host__ __device__ void operator+=(float2 &a, float2 b)
{
    a.x += b.x;
    a.y += b.y;
}

/**
 * @brief float2向量与标量相加：v + s
 * @param a 向量
 * @param b 标量
 * @return 向量每个分量都加上标量的结果
 */
inline __host__ __device__ float2 operator+(float2 a, float b)
{
    return make_float2(a.x + b, a.y + b);
}
inline __host__ __device__ float2 operator+(float b, float2 a)
{
    return make_float2(a.x + b, a.y + b);
}
inline __host__ __device__ void operator+=(float2 &a, float b)
{
    a.x += b;
    a.y += b;
}

inline __host__ __device__ int2 operator+(int2 a, int2 b)
{
    return make_int2(a.x + b.x, a.y + b.y);
}
inline __host__ __device__ void operator+=(int2 &a, int2 b)
{
    a.x += b.x;
    a.y += b.y;
}
inline __host__ __device__ int2 operator+(int2 a, int b)
{
    return make_int2(a.x + b, a.y + b);
}
inline __host__ __device__ int2 operator+(int b, int2 a)
{
    return make_int2(a.x + b, a.y + b);
}
inline __host__ __device__ void operator+=(int2 &a, int b)
{
    a.x += b;
    a.y += b;
}

inline __host__ __device__ uint2 operator+(uint2 a, uint2 b)
{
    return make_uint2(a.x + b.x, a.y + b.y);
}
inline __host__ __device__ void operator+=(uint2 &a, uint2 b)
{
    a.x += b.x;
    a.y += b.y;
}
inline __host__ __device__ uint2 operator+(uint2 a, uint b)
{
    return make_uint2(a.x + b, a.y + b);
}
inline __host__ __device__ uint2 operator+(uint b, uint2 a)
{
    return make_uint2(a.x + b, a.y + b);
}
inline __host__ __device__ void operator+=(uint2 &a, uint b)
{
    a.x += b;
    a.y += b;
}


inline __host__ __device__ float3 operator+(float3 a, float3 b)
{
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline __host__ __device__ void operator+=(float3 &a, float3 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
}
inline __host__ __device__ float3 operator+(float3 a, float b)
{
    return make_float3(a.x + b, a.y + b, a.z + b);
}
inline __host__ __device__ void operator+=(float3 &a, float b)
{
    a.x += b;
    a.y += b;
    a.z += b;
}

inline __host__ __device__ int3 operator+(int3 a, int3 b)
{
    return make_int3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline __host__ __device__ void operator+=(int3 &a, int3 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
}
inline __host__ __device__ int3 operator+(int3 a, int b)
{
    return make_int3(a.x + b, a.y + b, a.z + b);
}
inline __host__ __device__ void operator+=(int3 &a, int b)
{
    a.x += b;
    a.y += b;
    a.z += b;
}

inline __host__ __device__ uint3 operator+(uint3 a, uint3 b)
{
    return make_uint3(a.x + b.x, a.y + b.y, a.z + b.z);
}
inline __host__ __device__ void operator+=(uint3 &a, uint3 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
}
inline __host__ __device__ uint3 operator+(uint3 a, uint b)
{
    return make_uint3(a.x + b, a.y + b, a.z + b);
}
inline __host__ __device__ void operator+=(uint3 &a, uint b)
{
    a.x += b;
    a.y += b;
    a.z += b;
}

inline __host__ __device__ int3 operator+(int b, int3 a)
{
    return make_int3(a.x + b, a.y + b, a.z + b);
}
inline __host__ __device__ uint3 operator+(uint b, uint3 a)
{
    return make_uint3(a.x + b, a.y + b, a.z + b);
}
inline __host__ __device__ float3 operator+(float b, float3 a)
{
    return make_float3(a.x + b, a.y + b, a.z + b);
}

inline __host__ __device__ float4 operator+(float4 a, float4 b)
{
    return make_float4(a.x + b.x, a.y + b.y, a.z + b.z,  a.w + b.w);
}
inline __host__ __device__ void operator+=(float4 &a, float4 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
}
inline __host__ __device__ float4 operator+(float4 a, float b)
{
    return make_float4(a.x + b, a.y + b, a.z + b, a.w + b);
}
inline __host__ __device__ float4 operator+(float b, float4 a)
{
    return make_float4(a.x + b, a.y + b, a.z + b, a.w + b);
}
inline __host__ __device__ void operator+=(float4 &a, float b)
{
    a.x += b;
    a.y += b;
    a.z += b;
    a.w += b;
}

inline __host__ __device__ int4 operator+(int4 a, int4 b)
{
    return make_int4(a.x + b.x, a.y + b.y, a.z + b.z,  a.w + b.w);
}
inline __host__ __device__ void operator+=(int4 &a, int4 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
}
inline __host__ __device__ int4 operator+(int4 a, int b)
{
    return make_int4(a.x + b, a.y + b, a.z + b,  a.w + b);
}
inline __host__ __device__ int4 operator+(int b, int4 a)
{
    return make_int4(a.x + b, a.y + b, a.z + b,  a.w + b);
}
inline __host__ __device__ void operator+=(int4 &a, int b)
{
    a.x += b;
    a.y += b;
    a.z += b;
    a.w += b;
}

inline __host__ __device__ uint4 operator+(uint4 a, uint4 b)
{
    return make_uint4(a.x + b.x, a.y + b.y, a.z + b.z,  a.w + b.w);
}
inline __host__ __device__ void operator+=(uint4 &a, uint4 b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    a.w += b.w;
}
inline __host__ __device__ uint4 operator+(uint4 a, uint b)
{
    return make_uint4(a.x + b, a.y + b, a.z + b,  a.w + b);
}
inline __host__ __device__ uint4 operator+(uint b, uint4 a)
{
    return make_uint4(a.x + b, a.y + b, a.z + b,  a.w + b);
}
inline __host__ __device__ void operator+=(uint4 &a, uint b)
{
    a.x += b;
    a.y += b;
    a.z += b;
    a.w += b;
}

////////////////////////////////////////////////////////////////////////////////
// subtract
// 减法运算符
//
// 说明：为向量类型重载减法运算符，支持：
// 1. 向量与向量相减（对应分量相减）
// 2. 向量与标量相减 / 标量与向量相减
// 3. 复合赋值运算符 -=
////////////////////////////////////////////////////////////////////////////////

// float2 减法运算符组
inline __host__ __device__ float2 operator-(float2 a, float2 b)
{
    return make_float2(a.x - b.x, a.y - b.y);
}
inline __host__ __device__ void operator-=(float2 &a, float2 b)
{
    a.x -= b.x;
    a.y -= b.y;
}
inline __host__ __device__ float2 operator-(float2 a, float b)
{
    return make_float2(a.x - b, a.y - b);
}
inline __host__ __device__ float2 operator-(float b, float2 a)
{
    return make_float2(b - a.x, b - a.y);
}
inline __host__ __device__ void operator-=(float2 &a, float b)
{
    a.x -= b;
    a.y -= b;
}

inline __host__ __device__ int2 operator-(int2 a, int2 b)
{
    return make_int2(a.x - b.x, a.y - b.y);
}
inline __host__ __device__ void operator-=(int2 &a, int2 b)
{
    a.x -= b.x;
    a.y -= b.y;
}
inline __host__ __device__ int2 operator-(int2 a, int b)
{
    return make_int2(a.x - b, a.y - b);
}
inline __host__ __device__ int2 operator-(int b, int2 a)
{
    return make_int2(b - a.x, b - a.y);
}
inline __host__ __device__ void operator-=(int2 &a, int b)
{
    a.x -= b;
    a.y -= b;
}

inline __host__ __device__ uint2 operator-(uint2 a, uint2 b)
{
    return make_uint2(a.x - b.x, a.y - b.y);
}
inline __host__ __device__ void operator-=(uint2 &a, uint2 b)
{
    a.x -= b.x;
    a.y -= b.y;
}
inline __host__ __device__ uint2 operator-(uint2 a, uint b)
{
    return make_uint2(a.x - b, a.y - b);
}
inline __host__ __device__ uint2 operator-(uint b, uint2 a)
{
    return make_uint2(b - a.x, b - a.y);
}
inline __host__ __device__ void operator-=(uint2 &a, uint b)
{
    a.x -= b;
    a.y -= b;
}

inline __host__ __device__ float3 operator-(float3 a, float3 b)
{
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline __host__ __device__ void operator-=(float3 &a, float3 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
}
inline __host__ __device__ float3 operator-(float3 a, float b)
{
    return make_float3(a.x - b, a.y - b, a.z - b);
}
inline __host__ __device__ float3 operator-(float b, float3 a)
{
    return make_float3(b - a.x, b - a.y, b - a.z);
}
inline __host__ __device__ void operator-=(float3 &a, float b)
{
    a.x -= b;
    a.y -= b;
    a.z -= b;
}

inline __host__ __device__ int3 operator-(int3 a, int3 b)
{
    return make_int3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline __host__ __device__ void operator-=(int3 &a, int3 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
}
inline __host__ __device__ int3 operator-(int3 a, int b)
{
    return make_int3(a.x - b, a.y - b, a.z - b);
}
inline __host__ __device__ int3 operator-(int b, int3 a)
{
    return make_int3(b - a.x, b - a.y, b - a.z);
}
inline __host__ __device__ void operator-=(int3 &a, int b)
{
    a.x -= b;
    a.y -= b;
    a.z -= b;
}

inline __host__ __device__ uint3 operator-(uint3 a, uint3 b)
{
    return make_uint3(a.x - b.x, a.y - b.y, a.z - b.z);
}
inline __host__ __device__ void operator-=(uint3 &a, uint3 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
}
inline __host__ __device__ uint3 operator-(uint3 a, uint b)
{
    return make_uint3(a.x - b, a.y - b, a.z - b);
}
inline __host__ __device__ uint3 operator-(uint b, uint3 a)
{
    return make_uint3(b - a.x, b - a.y, b - a.z);
}
inline __host__ __device__ void operator-=(uint3 &a, uint b)
{
    a.x -= b;
    a.y -= b;
    a.z -= b;
}

inline __host__ __device__ float4 operator-(float4 a, float4 b)
{
    return make_float4(a.x - b.x, a.y - b.y, a.z - b.z,  a.w - b.w);
}
inline __host__ __device__ void operator-=(float4 &a, float4 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
}
inline __host__ __device__ float4 operator-(float4 a, float b)
{
    return make_float4(a.x - b, a.y - b, a.z - b,  a.w - b);
}
inline __host__ __device__ void operator-=(float4 &a, float b)
{
    a.x -= b;
    a.y -= b;
    a.z -= b;
    a.w -= b;
}

inline __host__ __device__ int4 operator-(int4 a, int4 b)
{
    return make_int4(a.x - b.x, a.y - b.y, a.z - b.z,  a.w - b.w);
}
inline __host__ __device__ void operator-=(int4 &a, int4 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
}
inline __host__ __device__ int4 operator-(int4 a, int b)
{
    return make_int4(a.x - b, a.y - b, a.z - b,  a.w - b);
}
inline __host__ __device__ int4 operator-(int b, int4 a)
{
    return make_int4(b - a.x, b - a.y, b - a.z, b - a.w);
}
inline __host__ __device__ void operator-=(int4 &a, int b)
{
    a.x -= b;
    a.y -= b;
    a.z -= b;
    a.w -= b;
}

inline __host__ __device__ uint4 operator-(uint4 a, uint4 b)
{
    return make_uint4(a.x - b.x, a.y - b.y, a.z - b.z,  a.w - b.w);
}
inline __host__ __device__ void operator-=(uint4 &a, uint4 b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    a.w -= b.w;
}
inline __host__ __device__ uint4 operator-(uint4 a, uint b)
{
    return make_uint4(a.x - b, a.y - b, a.z - b,  a.w - b);
}
inline __host__ __device__ uint4 operator-(uint b, uint4 a)
{
    return make_uint4(b - a.x, b - a.y, b - a.z, b - a.w);
}
inline __host__ __device__ void operator-=(uint4 &a, uint b)
{
    a.x -= b;
    a.y -= b;
    a.z -= b;
    a.w -= b;
}

////////////////////////////////////////////////////////////////////////////////
// multiply
// 乘法运算符
//
// 说明：为向量类型重载乘法运算符，支持：
// 1. 向量与向量相乘（对应分量相乘，非点积）
// 2. 向量与标量相乘（缩放）
// 3. 复合赋值运算符 *=
// 注意：这是分量乘法，不是点积或叉积
////////////////////////////////////////////////////////////////////////////////

// float2 乘法运算符组
inline __host__ __device__ float2 operator*(float2 a, float2 b)
{
    return make_float2(a.x * b.x, a.y * b.y);
}
inline __host__ __device__ void operator*=(float2 &a, float2 b)
{
    a.x *= b.x;
    a.y *= b.y;
}
inline __host__ __device__ float2 operator*(float2 a, float b)
{
    return make_float2(a.x * b, a.y * b);
}
inline __host__ __device__ float2 operator*(float b, float2 a)
{
    return make_float2(b * a.x, b * a.y);
}
inline __host__ __device__ void operator*=(float2 &a, float b)
{
    a.x *= b;
    a.y *= b;
}

inline __host__ __device__ int2 operator*(int2 a, int2 b)
{
    return make_int2(a.x * b.x, a.y * b.y);
}
inline __host__ __device__ void operator*=(int2 &a, int2 b)
{
    a.x *= b.x;
    a.y *= b.y;
}
inline __host__ __device__ int2 operator*(int2 a, int b)
{
    return make_int2(a.x * b, a.y * b);
}
inline __host__ __device__ int2 operator*(int b, int2 a)
{
    return make_int2(b * a.x, b * a.y);
}
inline __host__ __device__ void operator*=(int2 &a, int b)
{
    a.x *= b;
    a.y *= b;
}

inline __host__ __device__ uint2 operator*(uint2 a, uint2 b)
{
    return make_uint2(a.x * b.x, a.y * b.y);
}
inline __host__ __device__ void operator*=(uint2 &a, uint2 b)
{
    a.x *= b.x;
    a.y *= b.y;
}
inline __host__ __device__ uint2 operator*(uint2 a, uint b)
{
    return make_uint2(a.x * b, a.y * b);
}
inline __host__ __device__ uint2 operator*(uint b, uint2 a)
{
    return make_uint2(b * a.x, b * a.y);
}
inline __host__ __device__ void operator*=(uint2 &a, uint b)
{
    a.x *= b;
    a.y *= b;
}

inline __host__ __device__ float3 operator*(float3 a, float3 b)
{
    return make_float3(a.x * b.x, a.y * b.y, a.z * b.z);
}
inline __host__ __device__ void operator*=(float3 &a, float3 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
}
inline __host__ __device__ float3 operator*(float3 a, float b)
{
    return make_float3(a.x * b, a.y * b, a.z * b);
}
inline __host__ __device__ float3 operator*(float b, float3 a)
{
    return make_float3(b * a.x, b * a.y, b * a.z);
}
inline __host__ __device__ void operator*=(float3 &a, float b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
}

inline __host__ __device__ int3 operator*(int3 a, int3 b)
{
    return make_int3(a.x * b.x, a.y * b.y, a.z * b.z);
}
inline __host__ __device__ void operator*=(int3 &a, int3 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
}
inline __host__ __device__ int3 operator*(int3 a, int b)
{
    return make_int3(a.x * b, a.y * b, a.z * b);
}
inline __host__ __device__ int3 operator*(int b, int3 a)
{
    return make_int3(b * a.x, b * a.y, b * a.z);
}
inline __host__ __device__ void operator*=(int3 &a, int b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
}

inline __host__ __device__ uint3 operator*(uint3 a, uint3 b)
{
    return make_uint3(a.x * b.x, a.y * b.y, a.z * b.z);
}
inline __host__ __device__ void operator*=(uint3 &a, uint3 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
}
inline __host__ __device__ uint3 operator*(uint3 a, uint b)
{
    return make_uint3(a.x * b, a.y * b, a.z * b);
}
inline __host__ __device__ uint3 operator*(uint b, uint3 a)
{
    return make_uint3(b * a.x, b * a.y, b * a.z);
}
inline __host__ __device__ void operator*=(uint3 &a, uint b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
}

inline __host__ __device__ float4 operator*(float4 a, float4 b)
{
    return make_float4(a.x * b.x, a.y * b.y, a.z * b.z,  a.w * b.w);
}
inline __host__ __device__ void operator*=(float4 &a, float4 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
    a.w *= b.w;
}
inline __host__ __device__ float4 operator*(float4 a, float b)
{
    return make_float4(a.x * b, a.y * b, a.z * b,  a.w * b);
}
inline __host__ __device__ float4 operator*(float b, float4 a)
{
    return make_float4(b * a.x, b * a.y, b * a.z, b * a.w);
}
inline __host__ __device__ void operator*=(float4 &a, float b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
}

inline __host__ __device__ int4 operator*(int4 a, int4 b)
{
    return make_int4(a.x * b.x, a.y * b.y, a.z * b.z,  a.w * b.w);
}
inline __host__ __device__ void operator*=(int4 &a, int4 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
    a.w *= b.w;
}
inline __host__ __device__ int4 operator*(int4 a, int b)
{
    return make_int4(a.x * b, a.y * b, a.z * b,  a.w * b);
}
inline __host__ __device__ int4 operator*(int b, int4 a)
{
    return make_int4(b * a.x, b * a.y, b * a.z, b * a.w);
}
inline __host__ __device__ void operator*=(int4 &a, int b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
}

inline __host__ __device__ uint4 operator*(uint4 a, uint4 b)
{
    return make_uint4(a.x * b.x, a.y * b.y, a.z * b.z,  a.w * b.w);
}
inline __host__ __device__ void operator*=(uint4 &a, uint4 b)
{
    a.x *= b.x;
    a.y *= b.y;
    a.z *= b.z;
    a.w *= b.w;
}
inline __host__ __device__ uint4 operator*(uint4 a, uint b)
{
    return make_uint4(a.x * b, a.y * b, a.z * b,  a.w * b);
}
inline __host__ __device__ uint4 operator*(uint b, uint4 a)
{
    return make_uint4(b * a.x, b * a.y, b * a.z, b * a.w);
}
inline __host__ __device__ void operator*=(uint4 &a, uint b)
{
    a.x *= b;
    a.y *= b;
    a.z *= b;
    a.w *= b;
}

////////////////////////////////////////////////////////////////////////////////
// divide
// 除法运算符
//
// 说明：为浮点向量类型重载除法运算符，支持：
// 1. 向量与向量相除（对应分量相除）
// 2. 向量与标量相除 / 标量与向量相除
// 3. 复合赋值运算符 /=
// 注意：整数向量不支持除法运算
////////////////////////////////////////////////////////////////////////////////

// float2 除法运算符组
inline __host__ __device__ float2 operator/(float2 a, float2 b)
{
    return make_float2(a.x / b.x, a.y / b.y);
}
inline __host__ __device__ void operator/=(float2 &a, float2 b)
{
    a.x /= b.x;
    a.y /= b.y;
}
inline __host__ __device__ float2 operator/(float2 a, float b)
{
    return make_float2(a.x / b, a.y / b);
}
inline __host__ __device__ void operator/=(float2 &a, float b)
{
    a.x /= b;
    a.y /= b;
}
inline __host__ __device__ float2 operator/(float b, float2 a)
{
    return make_float2(b / a.x, b / a.y);
}

inline __host__ __device__ float3 operator/(float3 a, float3 b)
{
    return make_float3(a.x / b.x, a.y / b.y, a.z / b.z);
}
inline __host__ __device__ void operator/=(float3 &a, float3 b)
{
    a.x /= b.x;
    a.y /= b.y;
    a.z /= b.z;
}
inline __host__ __device__ float3 operator/(float3 a, float b)
{
    return make_float3(a.x / b, a.y / b, a.z / b);
}
inline __host__ __device__ void operator/=(float3 &a, float b)
{
    a.x /= b;
    a.y /= b;
    a.z /= b;
}
inline __host__ __device__ float3 operator/(float b, float3 a)
{
    return make_float3(b / a.x, b / a.y, b / a.z);
}

inline __host__ __device__ float4 operator/(float4 a, float4 b)
{
    return make_float4(a.x / b.x, a.y / b.y, a.z / b.z,  a.w / b.w);
}
inline __host__ __device__ void operator/=(float4 &a, float4 b)
{
    a.x /= b.x;
    a.y /= b.y;
    a.z /= b.z;
    a.w /= b.w;
}
inline __host__ __device__ float4 operator/(float4 a, float b)
{
    return make_float4(a.x / b, a.y / b, a.z / b,  a.w / b);
}
inline __host__ __device__ void operator/=(float4 &a, float b)
{
    a.x /= b;
    a.y /= b;
    a.z /= b;
    a.w /= b;
}
inline __host__ __device__ float4 operator/(float b, float4 a)
{
    return make_float4(b / a.x, b / a.y, b / a.z, b / a.w);
}

////////////////////////////////////////////////////////////////////////////////
// min
// 最小值函数
//
// 说明：返回两个向量对应分量的最小值组成的新向量
// 对于浮点型使用fminf，对于整型使用min
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 返回两个float2向量的分量最小值
 * @param a 第一个向量
 * @param b 第二个向量
 * @return 每个分量为min(a[i], b[i])的新向量
 */
inline  __host__ __device__ float2 fminf(float2 a, float2 b)
{
    return make_float2(fminf(a.x,b.x), fminf(a.y,b.y));
}
inline __host__ __device__ float3 fminf(float3 a, float3 b)
{
    return make_float3(fminf(a.x,b.x), fminf(a.y,b.y), fminf(a.z,b.z));
}
inline  __host__ __device__ float4 fminf(float4 a, float4 b)
{
    return make_float4(fminf(a.x,b.x), fminf(a.y,b.y), fminf(a.z,b.z), fminf(a.w,b.w));
}

inline __host__ __device__ int2 min(int2 a, int2 b)
{
    return make_int2(min(a.x,b.x), min(a.y,b.y));
}
inline __host__ __device__ int3 min(int3 a, int3 b)
{
    return make_int3(min(a.x,b.x), min(a.y,b.y), min(a.z,b.z));
}
inline __host__ __device__ int4 min(int4 a, int4 b)
{
    return make_int4(min(a.x,b.x), min(a.y,b.y), min(a.z,b.z), min(a.w,b.w));
}

inline __host__ __device__ uint2 min(uint2 a, uint2 b)
{
    return make_uint2(min(a.x,b.x), min(a.y,b.y));
}
inline __host__ __device__ uint3 min(uint3 a, uint3 b)
{
    return make_uint3(min(a.x,b.x), min(a.y,b.y), min(a.z,b.z));
}
inline __host__ __device__ uint4 min(uint4 a, uint4 b)
{
    return make_uint4(min(a.x,b.x), min(a.y,b.y), min(a.z,b.z), min(a.w,b.w));
}

////////////////////////////////////////////////////////////////////////////////
// max
// 最大值函数
//
// 说明：返回两个向量对应分量的最大值组成的新向量
// 对于浮点型使用fmaxf，对于整型使用max
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 返回两个float2向量的分量最大值
 */
inline __host__ __device__ float2 fmaxf(float2 a, float2 b)
{
    return make_float2(fmaxf(a.x,b.x), fmaxf(a.y,b.y));
}
inline __host__ __device__ float3 fmaxf(float3 a, float3 b)
{
    return make_float3(fmaxf(a.x,b.x), fmaxf(a.y,b.y), fmaxf(a.z,b.z));
}
inline __host__ __device__ float4 fmaxf(float4 a, float4 b)
{
    return make_float4(fmaxf(a.x,b.x), fmaxf(a.y,b.y), fmaxf(a.z,b.z), fmaxf(a.w,b.w));
}

inline __host__ __device__ int2 max(int2 a, int2 b)
{
    return make_int2(max(a.x,b.x), max(a.y,b.y));
}
inline __host__ __device__ int3 max(int3 a, int3 b)
{
    return make_int3(max(a.x,b.x), max(a.y,b.y), max(a.z,b.z));
}
inline __host__ __device__ int4 max(int4 a, int4 b)
{
    return make_int4(max(a.x,b.x), max(a.y,b.y), max(a.z,b.z), max(a.w,b.w));
}

inline __host__ __device__ uint2 max(uint2 a, uint2 b)
{
    return make_uint2(max(a.x,b.x), max(a.y,b.y));
}
inline __host__ __device__ uint3 max(uint3 a, uint3 b)
{
    return make_uint3(max(a.x,b.x), max(a.y,b.y), max(a.z,b.z));
}
inline __host__ __device__ uint4 max(uint4 a, uint4 b)
{
    return make_uint4(max(a.x,b.x), max(a.y,b.y), max(a.z,b.z), max(a.w,b.w));
}

////////////////////////////////////////////////////////////////////////////////
// lerp
// 线性插值函数
//
// 说明：在两个值之间进行线性插值
// 公式：result = a + t*(b-a) = (1-t)*a + t*b
// - 当 t = 0 时，返回 a
// - 当 t = 1 时，返回 b
// - 当 0 < t < 1 时，返回 a 和 b 之间的插值
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 标量线性插值
 * @param a 起始值
 * @param b 结束值
 * @param t 插值参数，通常在[0,1]范围内
 * @return a和b之间的插值结果
 */
inline __device__ __host__ float lerp(float a, float b, float t)
{
    return a + t*(b-a);
}
inline __device__ __host__ float2 lerp(float2 a, float2 b, float t)
{
    return a + t*(b-a);
}
inline __device__ __host__ float3 lerp(float3 a, float3 b, float t)
{
    return a + t*(b-a);
}
inline __device__ __host__ float4 lerp(float4 a, float4 b, float t)
{
    return a + t*(b-a);
}

////////////////////////////////////////////////////////////////////////////////
// clamp
// 值域限制函数
//
// 说明：将值限制在指定范围内
// - 如果 v < a，返回 a
// - 如果 v > b，返回 b
// - 否则返回 v
// 应用：防止数值越界、颜色值归一化等
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 将浮点数限制在[a, b]范围内
 * @param f 待限制的值
 * @param a 下界
 * @param b 上界
 * @return 限制后的值
 */
inline __device__ __host__ float clamp(float f, float a, float b)
{
    return fmaxf(a, fminf(f, b));
}
inline __device__ __host__ int clamp(int f, int a, int b)
{
    return max(a, min(f, b));
}
inline __device__ __host__ uint clamp(uint f, uint a, uint b)
{
    return max(a, min(f, b));
}

inline __device__ __host__ float2 clamp(float2 v, float a, float b)
{
    return make_float2(clamp(v.x, a, b), clamp(v.y, a, b));
}
inline __device__ __host__ float2 clamp(float2 v, float2 a, float2 b)
{
    return make_float2(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y));
}
inline __device__ __host__ float3 clamp(float3 v, float a, float b)
{
    return make_float3(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b));
}
inline __device__ __host__ float3 clamp(float3 v, float3 a, float3 b)
{
    return make_float3(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z));
}
inline __device__ __host__ float4 clamp(float4 v, float a, float b)
{
    return make_float4(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b), clamp(v.w, a, b));
}
inline __device__ __host__ float4 clamp(float4 v, float4 a, float4 b)
{
    return make_float4(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z), clamp(v.w, a.w, b.w));
}

inline __device__ __host__ int2 clamp(int2 v, int a, int b)
{
    return make_int2(clamp(v.x, a, b), clamp(v.y, a, b));
}
inline __device__ __host__ int2 clamp(int2 v, int2 a, int2 b)
{
    return make_int2(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y));
}
inline __device__ __host__ int3 clamp(int3 v, int a, int b)
{
    return make_int3(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b));
}
inline __device__ __host__ int3 clamp(int3 v, int3 a, int3 b)
{
    return make_int3(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z));
}
inline __device__ __host__ int4 clamp(int4 v, int a, int b)
{
    return make_int4(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b), clamp(v.w, a, b));
}
inline __device__ __host__ int4 clamp(int4 v, int4 a, int4 b)
{
    return make_int4(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z), clamp(v.w, a.w, b.w));
}

inline __device__ __host__ uint2 clamp(uint2 v, uint a, uint b)
{
    return make_uint2(clamp(v.x, a, b), clamp(v.y, a, b));
}
inline __device__ __host__ uint2 clamp(uint2 v, uint2 a, uint2 b)
{
    return make_uint2(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y));
}
inline __device__ __host__ uint3 clamp(uint3 v, uint a, uint b)
{
    return make_uint3(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b));
}
inline __device__ __host__ uint3 clamp(uint3 v, uint3 a, uint3 b)
{
    return make_uint3(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z));
}
inline __device__ __host__ uint4 clamp(uint4 v, uint a, uint b)
{
    return make_uint4(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b), clamp(v.w, a, b));
}
inline __device__ __host__ uint4 clamp(uint4 v, uint4 a, uint4 b)
{
    return make_uint4(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z), clamp(v.w, a.w, b.w));
}

////////////////////////////////////////////////////////////////////////////////
// dot product
// 点积（内积）运算
//
// 说明：计算两个向量的点积
// 点积定义：dot(a,b) = a.x*b.x + a.y*b.y + ...
// 几何意义：dot(a,b) = |a|*|b|*cos(θ)，其中θ是两向量夹角
// 应用：
// - 计算向量夹角
// - 判断向量正交性（点积为0则正交）
// - 计算投影长度
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 计算两个float2向量的点积
 * @param a 第一个向量
 * @param b 第二个向量
 * @return 点积值
 */
inline __host__ __device__ float dot(float2 a, float2 b)
{
    return a.x * b.x + a.y * b.y;
}

/**
 * @brief 计算两个float3向量的点积
 */
inline __host__ __device__ float dot(float3 a, float3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/**
 * @brief 计算两个float4向量的点积
 */
inline __host__ __device__ float dot(float4 a, float4 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline __host__ __device__ int dot(int2 a, int2 b)
{
    return a.x * b.x + a.y * b.y;
}
inline __host__ __device__ int dot(int3 a, int3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline __host__ __device__ int dot(int4 a, int4 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline __host__ __device__ uint dot(uint2 a, uint2 b)
{
    return a.x * b.x + a.y * b.y;
}
inline __host__ __device__ uint dot(uint3 a, uint3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline __host__ __device__ uint dot(uint4 a, uint4 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

////////////////////////////////////////////////////////////////////////////////
// length
// 向量长度（模）计算
//
// 说明：计算向量的欧几里得长度
// 公式：length(v) = sqrt(v.x² + v.y² + ...)
// 实现：通过 sqrt(dot(v,v)) 计算
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 计算float2向量的长度
 * @param v 输入向量
 * @return 向量的长度（模）
 */
inline __host__ __device__ float length(float2 v)
{
    return sqrtf(dot(v, v));
}

/**
 * @brief 计算float3向量的长度
 */
inline __host__ __device__ float length(float3 v)
{
    return sqrtf(dot(v, v));
}

/**
 * @brief 计算float4向量的长度
 */
inline __host__ __device__ float length(float4 v)
{
    return sqrtf(dot(v, v));
}

////////////////////////////////////////////////////////////////////////////////
// normalize
// 向量归一化
//
// 说明：将向量转换为单位向量（长度为1的向量）
// 算法：v_normalized = v / |v| = v * (1/|v|)
// 实现：使用rsqrtf(快速倒数平方根)提高性能
// 应用：获取方向向量、法向量计算等
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 归一化float2向量
 * @param v 输入向量
 * @return 与v同方向的单位向量
 */
inline __host__ __device__ float2 normalize(float2 v)
{
    float invLen = rsqrtf(dot(v, v));  // 1/|v|
    return v * invLen;
}

/**
 * @brief 归一化float3向量
 */
inline __host__ __device__ float3 normalize(float3 v)
{
    float invLen = rsqrtf(dot(v, v));
    return v * invLen;
}

/**
 * @brief 归一化float4向量
 */
inline __host__ __device__ float4 normalize(float4 v)
{
    float invLen = rsqrtf(dot(v, v));
    return v * invLen;
}

////////////////////////////////////////////////////////////////////////////////
// floor
// 向下取整函数
//
// 说明：对向量的每个分量向下取整到最接近的整数
// 例如：floorf(1.7) = 1.0, floorf(-1.3) = -2.0
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief float2向量各分量向下取整
 */
inline __host__ __device__ float2 floorf(float2 v)
{
    return make_float2(floorf(v.x), floorf(v.y));
}

/**
 * @brief float3向量各分量向下取整
 */
inline __host__ __device__ float3 floorf(float3 v)
{
    return make_float3(floorf(v.x), floorf(v.y), floorf(v.z));
}

/**
 * @brief float4向量各分量向下取整
 */
inline __host__ __device__ float4 floorf(float4 v)
{
    return make_float4(floorf(v.x), floorf(v.y), floorf(v.z), floorf(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// frac - returns the fractional portion of a scalar or each vector component
// 小数部分提取函数
//
// 说明：返回数值的小数部分（值减去向下取整后的结果）
// 公式：frac(v) = v - floor(v)
// 例如：frac(3.7) = 0.7, frac(-1.3) = 0.7
// 应用：纹理坐标重复、周期性计算等
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 提取标量的小数部分
 * @param v 输入值
 * @return 小数部分，范围[0, 1)
 */
inline __host__ __device__ float fracf(float v)
{
    return v - floorf(v);
}
inline __host__ __device__ float2 fracf(float2 v)
{
    return make_float2(fracf(v.x), fracf(v.y));
}
inline __host__ __device__ float3 fracf(float3 v)
{
    return make_float3(fracf(v.x), fracf(v.y), fracf(v.z));
}
inline __host__ __device__ float4 fracf(float4 v)
{
    return make_float4(fracf(v.x), fracf(v.y), fracf(v.z), fracf(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// fmod
// 浮点取模运算
//
// 说明：计算浮点数的模运算，对向量的每个分量进行取模
// 与整数取模不同，fmod保留小数部分
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief float2向量的分量取模运算
 */
inline __host__ __device__ float2 fmodf(float2 a, float2 b)
{
    return make_float2(fmodf(a.x, b.x), fmodf(a.y, b.y));
}
inline __host__ __device__ float3 fmodf(float3 a, float3 b)
{
    return make_float3(fmodf(a.x, b.x), fmodf(a.y, b.y), fmodf(a.z, b.z));
}
inline __host__ __device__ float4 fmodf(float4 a, float4 b)
{
    return make_float4(fmodf(a.x, b.x), fmodf(a.y, b.y), fmodf(a.z, b.z), fmodf(a.w, b.w));
}

////////////////////////////////////////////////////////////////////////////////
// absolute value
// 绝对值函数
//
// 说明：对向量的每个分量取绝对值
// 浮点型使用fabs，整型使用abs
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief float2向量各分量取绝对值
 */
inline __host__ __device__ float2 fabs(float2 v)
{
    return make_float2(fabs(v.x), fabs(v.y));
}
inline __host__ __device__ float3 fabs(float3 v)
{
    return make_float3(fabs(v.x), fabs(v.y), fabs(v.z));
}
inline __host__ __device__ float4 fabs(float4 v)
{
    return make_float4(fabs(v.x), fabs(v.y), fabs(v.z), fabs(v.w));
}

inline __host__ __device__ int2 abs(int2 v)
{
    return make_int2(abs(v.x), abs(v.y));
}
inline __host__ __device__ int3 abs(int3 v)
{
    return make_int3(abs(v.x), abs(v.y), abs(v.z));
}
inline __host__ __device__ int4 abs(int4 v)
{
    return make_int4(abs(v.x), abs(v.y), abs(v.z), abs(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// reflect
// 反射向量计算
//
// 说明：计算入射光线相对于表面法向量的反射向量
// 物理意义：模拟光线在表面的反射
// 公式：r = i - 2*n*dot(n,i)
// 要求：法向量n应该是单位向量
// 结果：反射向量的长度等于入射向量的长度
// 应用：光照计算、镜面反射等
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 计算三维反射向量
 * @param i 入射向量
 * @param n 表面法向量（应为单位向量）
 * @return 反射向量
 */
inline __host__ __device__ float3 reflect(float3 i, float3 n)
{
    return i - 2.0f * n * dot(n,i);
}

////////////////////////////////////////////////////////////////////////////////
// cross product
// 叉积（外积）运算
//
// 说明：计算两个三维向量的叉积
// 几何意义：
// - 叉积结果垂直于两个输入向量构成的平面
// - |cross(a,b)| = |a|*|b|*sin(θ)，其中θ是两向量夹角
// - 方向遵循右手定则
// 公式：cross(a,b) = (a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x)
// 应用：
// - 计算法向量
// - 判断向量方向关系
// - 计算平行四边形面积
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 计算两个float3向量的叉积
 * @param a 第一个向量
 * @param b 第二个向量
 * @return 垂直于a和b的向量
 */
inline __host__ __device__ float3 cross(float3 a, float3 b)
{
    return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

////////////////////////////////////////////////////////////////////////////////
// smoothstep
// 平滑阶跃插值函数
//
// 说明：在两个值之间进行平滑的S型插值
// 行为：
// - 当 x < a 时返回 0
// - 当 x > b 时返回 1
// - 当 a ≤ x ≤ b 时返回平滑的0到1的过渡
// 算法：使用Hermite插值多项式 3t²-2t³
// 特点：
// - 在端点处导数为0，确保平滑过渡
// - 比线性插值更自然
// 应用：动画缓动、颜色渐变、边缘平滑等
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 标量平滑阶跃插值
 * @param a 下界
 * @param b 上界
 * @param x 插值点
 * @return 平滑插值结果[0,1]
 */
inline __device__ __host__ float smoothstep(float a, float b, float x)
{
    float y = clamp((x - a) / (b - a), 0.0f, 1.0f);  // 归一化到[0,1]
    return (y*y*(3.0f - (2.0f*y)));  // 3t²-2t³插值多项式
}
inline __device__ __host__ float2 smoothstep(float2 a, float2 b, float2 x)
{
    float2 y = clamp((x - a) / (b - a), 0.0f, 1.0f);
    return (y*y*(make_float2(3.0f) - (make_float2(2.0f)*y)));
}
inline __device__ __host__ float3 smoothstep(float3 a, float3 b, float3 x)
{
    float3 y = clamp((x - a) / (b - a), 0.0f, 1.0f);
    return (y*y*(make_float3(3.0f) - (make_float3(2.0f)*y)));
}
inline __device__ __host__ float4 smoothstep(float4 a, float4 b, float4 x)
{
    float4 y = clamp((x - a) / (b - a), 0.0f, 1.0f);
    return (y*y*(make_float4(3.0f) - (make_float4(2.0f)*y)));
}

#endif
