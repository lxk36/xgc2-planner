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

/**
 * @file helper_math.h
 * @brief CUDA向量数学辅助函数库
 *
 * 本文件实现了CUDA向量类型(float2, float3, float4, int2, int3, int4等)的常用数学运算。
 * 这些运算包括：构造函数、算术运算(加减乘除)、最小/最大值、线性插值、点积、叉积、
 * 归一化、反射等向量操作。
 *
 * 主要功能模块：
 * - 类型定义和构造函数
 * - 一元运算(取负)
 * - 二元运算(加、减、乘、除)
 * - 比较运算(最小值、最大值)
 * - 插值运算(线性插值、平滑插值)
 * - 限幅运算(clamp)
 * - 向量运算(点积、叉积、长度、归一化、反射)
 * - 数学函数(floor, frac, fmod, fabs等)
 */

#ifndef HELPER_MATH_H
#define HELPER_MATH_H

#include "cuda_runtime.h"

// 类型定义
typedef unsigned int uint;    // 无符号整型
typedef unsigned short ushort; // 无符号短整型

#ifndef EXIT_WAIVED
#define EXIT_WAIVED 2
#endif

#ifndef __CUDACC__
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
// host implementations of CUDA functions
// 主机端(CPU)对CUDA函数的实现
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 返回两个浮点数中的最小值
 * @param a 第一个浮点数
 * @param b 第二个浮点数
 * @return 较小的浮点数
 */
inline float fminf(float a, float b)
{
    return a < b ? a : b;
}

/**
 * @brief 返回两个浮点数中的最大值
 * @param a 第一个浮点数
 * @param b 第二个浮点数
 * @return 较大的浮点数
 */
inline float fmaxf(float a, float b)
{
    return a > b ? a : b;
}

/**
 * @brief 返回两个整数中的最大值
 * @param a 第一个整数
 * @param b 第二个整数
 * @return 较大的整数
 */
inline int max(int a, int b)
{
    return a > b ? a : b;
}

/**
 * @brief 返回两个整数中的最小值
 * @param a 第一个整数
 * @param b 第二个整数
 * @return 较小的整数
 */
inline int min(int a, int b)
{
    return a < b ? a : b;
}

/**
 * @brief 计算平方根的倒数 (1/sqrt(x))
 * @param x 输入值
 * @return x的平方根的倒数
 */
inline float rsqrtf(float x)
{
    return 1.0f / sqrtf(x);
}
#endif

////////////////////////////////////////////////////////////////////////////////
// constructors
// 向量类型构造函数
// 说明：提供各种向量类型之间的相互转换和构造方法
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 从单个标量构造float2向量 (s, s)
 * @param s 标量值
 * @return 两个分量都为s的float2向量
 */
inline __host__ __device__ float2 make_float2(float s)
{
    return make_float2(s, s);
}

/**
 * @brief 从float3向量构造float2向量，提取前两个分量
 * @param a float3向量
 * @return 包含a.x和a.y的float2向量
 */
inline __host__ __device__ float2 make_float2(float3 a)
{
    return make_float2(a.x, a.y);
}

/**
 * @brief 从int2向量构造float2向量
 * @param a int2向量
 * @return 类型转换后的float2向量
 */
inline __host__ __device__ float2 make_float2(int2 a)
{
    return make_float2(float(a.x), float(a.y));
}

/**
 * @brief 从uint2向量构造float2向量
 * @param a uint2向量
 * @return 类型转换后的float2向量
 */
inline __host__ __device__ float2 make_float2(uint2 a)
{
    return make_float2(float(a.x), float(a.y));
}

// int2构造函数
/** @brief 从单个标量构造int2向量 (s, s) */
inline __host__ __device__ int2 make_int2(int s)
{
    return make_int2(s, s);
}
/** @brief 从int3向量构造int2向量，提取前两个分量 */
inline __host__ __device__ int2 make_int2(int3 a)
{
    return make_int2(a.x, a.y);
}
/** @brief 从uint2向量构造int2向量，类型转换 */
inline __host__ __device__ int2 make_int2(uint2 a)
{
    return make_int2(int(a.x), int(a.y));
}
/** @brief 从float2向量构造int2向量，类型转换 */
inline __host__ __device__ int2 make_int2(float2 a)
{
    return make_int2(int(a.x), int(a.y));
}

// uint2构造函数
/** @brief 从单个标量构造uint2向量 (s, s) */
inline __host__ __device__ uint2 make_uint2(uint s)
{
    return make_uint2(s, s);
}
/** @brief 从uint3向量构造uint2向量，提取前两个分量 */
inline __host__ __device__ uint2 make_uint2(uint3 a)
{
    return make_uint2(a.x, a.y);
}
/** @brief 从int2向量构造uint2向量，类型转换 */
inline __host__ __device__ uint2 make_uint2(int2 a)
{
    return make_uint2(uint(a.x), uint(a.y));
}

// float3构造函数
/** @brief 从单个标量构造float3向量 (s, s, s) */
inline __host__ __device__ float3 make_float3(float s)
{
    return make_float3(s, s, s);
}
/** @brief 从float2向量构造float3向量，z分量设为0 */
inline __host__ __device__ float3 make_float3(float2 a)
{
    return make_float3(a.x, a.y, 0.0f);
}
/** @brief 从float2向量和标量构造float3向量 (a.x, a.y, s) */
inline __host__ __device__ float3 make_float3(float2 a, float s)
{
    return make_float3(a.x, a.y, s);
}
/** @brief 从float4向量构造float3向量，提取前三个分量 */
inline __host__ __device__ float3 make_float3(float4 a)
{
    return make_float3(a.x, a.y, a.z);
}
/** @brief 从int3向量构造float3向量，类型转换 */
inline __host__ __device__ float3 make_float3(int3 a)
{
    return make_float3(float(a.x), float(a.y), float(a.z));
}
/** @brief 从uint3向量构造float3向量，类型转换 */
inline __host__ __device__ float3 make_float3(uint3 a)
{
    return make_float3(float(a.x), float(a.y), float(a.z));
}

// int3构造函数
/** @brief 从单个标量构造int3向量 (s, s, s) */
inline __host__ __device__ int3 make_int3(int s)
{
    return make_int3(s, s, s);
}
/** @brief 从int2向量构造int3向量，z分量设为0 */
inline __host__ __device__ int3 make_int3(int2 a)
{
    return make_int3(a.x, a.y, 0);
}
/** @brief 从int2向量和标量构造int3向量 (a.x, a.y, s) */
inline __host__ __device__ int3 make_int3(int2 a, int s)
{
    return make_int3(a.x, a.y, s);
}
/** @brief 从uint3向量构造int3向量，类型转换 */
inline __host__ __device__ int3 make_int3(uint3 a)
{
    return make_int3(int(a.x), int(a.y), int(a.z));
}
/** @brief 从float3向量构造int3向量，类型转换 */
inline __host__ __device__ int3 make_int3(float3 a)
{
    return make_int3(int(a.x), int(a.y), int(a.z));
}

// uint3构造函数
/** @brief 从单个标量构造uint3向量 (s, s, s) */
inline __host__ __device__ uint3 make_uint3(uint s)
{
    return make_uint3(s, s, s);
}
/** @brief 从uint2向量构造uint3向量，z分量设为0 */
inline __host__ __device__ uint3 make_uint3(uint2 a)
{
    return make_uint3(a.x, a.y, 0);
}
/** @brief 从uint2向量和标量构造uint3向量 (a.x, a.y, s) */
inline __host__ __device__ uint3 make_uint3(uint2 a, uint s)
{
    return make_uint3(a.x, a.y, s);
}
/** @brief 从uint4向量构造uint3向量，提取前三个分量 */
inline __host__ __device__ uint3 make_uint3(uint4 a)
{
    return make_uint3(a.x, a.y, a.z);
}
/** @brief 从int3向量构造uint3向量，类型转换 */
inline __host__ __device__ uint3 make_uint3(int3 a)
{
    return make_uint3(uint(a.x), uint(a.y), uint(a.z));
}

// float4构造函数
/** @brief 从单个标量构造float4向量 (s, s, s, s) */
inline __host__ __device__ float4 make_float4(float s)
{
    return make_float4(s, s, s, s);
}
/** @brief 从float3向量构造float4向量，w分量设为0 */
inline __host__ __device__ float4 make_float4(float3 a)
{
    return make_float4(a.x, a.y, a.z, 0.0f);
}
/** @brief 从float3向量和标量构造float4向量 (a.x, a.y, a.z, w) */
inline __host__ __device__ float4 make_float4(float3 a, float w)
{
    return make_float4(a.x, a.y, a.z, w);
}
/** @brief 从int4向量构造float4向量，类型转换 */
inline __host__ __device__ float4 make_float4(int4 a)
{
    return make_float4(float(a.x), float(a.y), float(a.z), float(a.w));
}
/** @brief 从uint4向量构造float4向量，类型转换 */
inline __host__ __device__ float4 make_float4(uint4 a)
{
    return make_float4(float(a.x), float(a.y), float(a.z), float(a.w));
}

// int4构造函数
/** @brief 从单个标量构造int4向量 (s, s, s, s) */
inline __host__ __device__ int4 make_int4(int s)
{
    return make_int4(s, s, s, s);
}
/** @brief 从int3向量构造int4向量，w分量设为0 */
inline __host__ __device__ int4 make_int4(int3 a)
{
    return make_int4(a.x, a.y, a.z, 0);
}
/** @brief 从int3向量和标量构造int4向量 (a.x, a.y, a.z, w) */
inline __host__ __device__ int4 make_int4(int3 a, int w)
{
    return make_int4(a.x, a.y, a.z, w);
}
/** @brief 从uint4向量构造int4向量，类型转换 */
inline __host__ __device__ int4 make_int4(uint4 a)
{
    return make_int4(int(a.x), int(a.y), int(a.z), int(a.w));
}
/** @brief 从float4向量构造int4向量，类型转换 */
inline __host__ __device__ int4 make_int4(float4 a)
{
    return make_int4(int(a.x), int(a.y), int(a.z), int(a.w));
}

// uint4构造函数
/** @brief 从单个标量构造uint4向量 (s, s, s, s) */
inline __host__ __device__ uint4 make_uint4(uint s)
{
    return make_uint4(s, s, s, s);
}
/** @brief 从uint3向量构造uint4向量，w分量设为0 */
inline __host__ __device__ uint4 make_uint4(uint3 a)
{
    return make_uint4(a.x, a.y, a.z, 0);
}
/** @brief 从uint3向量和标量构造uint4向量 (a.x, a.y, a.z, w) */
inline __host__ __device__ uint4 make_uint4(uint3 a, uint w)
{
    return make_uint4(a.x, a.y, a.z, w);
}
/** @brief 从int4向量构造uint4向量，类型转换 */
inline __host__ __device__ uint4 make_uint4(int4 a)
{
    return make_uint4(uint(a.x), uint(a.y), uint(a.z), uint(a.w));
}

////////////////////////////////////////////////////////////////////////////////
// negate
// 取负运算符
// 功能：返回向量的相反数，即所有分量取负
////////////////////////////////////////////////////////////////////////////////

/** @brief float2向量取负运算 */
inline __host__ __device__ float2 operator-(float2 &a)
{
    return make_float2(-a.x, -a.y);
}
/** @brief int2向量取负运算 */
inline __host__ __device__ int2 operator-(int2 &a)
{
    return make_int2(-a.x, -a.y);
}
/** @brief float3向量取负运算 */
inline __host__ __device__ float3 operator-(float3 &a)
{
    return make_float3(-a.x, -a.y, -a.z);
}
/** @brief int3向量取负运算 */
inline __host__ __device__ int3 operator-(int3 &a)
{
    return make_int3(-a.x, -a.y, -a.z);
}
/** @brief float4向量取负运算 */
inline __host__ __device__ float4 operator-(float4 &a)
{
    return make_float4(-a.x, -a.y, -a.z, -a.w);
}
/** @brief int4向量取负运算 */
inline __host__ __device__ int4 operator-(int4 &a)
{
    return make_int4(-a.x, -a.y, -a.z, -a.w);
}

////////////////////////////////////////////////////////////////////////////////
// addition
// 加法运算符
// 功能：向量加法，支持向量+向量、向量+标量、标量+向量及对应的+=运算
////////////////////////////////////////////////////////////////////////////////

// float2加法运算
/** @brief float2向量相加：逐分量相加 */
inline __host__ __device__ float2 operator+(float2 a, float2 b)
{
    return make_float2(a.x + b.x, a.y + b.y);
}
/** @brief float2向量加法赋值运算 */
inline __host__ __device__ void operator+=(float2 &a, float2 b)
{
    a.x += b.x;
    a.y += b.y;
}
/** @brief float2向量与标量相加：所有分量加上标量 */
inline __host__ __device__ float2 operator+(float2 a, float b)
{
    return make_float2(a.x + b, a.y + b);
}
/** @brief 标量与float2向量相加 */
inline __host__ __device__ float2 operator+(float b, float2 a)
{
    return make_float2(a.x + b, a.y + b);
}
/** @brief float2向量标量加法赋值运算 */
inline __host__ __device__ void operator+=(float2 &a, float b)
{
    a.x += b;
    a.y += b;
}

// int2加法运算
/** @brief int2向量相加 */
inline __host__ __device__ int2 operator+(int2 a, int2 b)
{
    return make_int2(a.x + b.x, a.y + b.y);
}
/** @brief int2向量加法赋值运算 */
inline __host__ __device__ void operator+=(int2 &a, int2 b)
{
    a.x += b.x;
    a.y += b.y;
}
/** @brief int2向量与标量相加 */
inline __host__ __device__ int2 operator+(int2 a, int b)
{
    return make_int2(a.x + b, a.y + b);
}
/** @brief 标量与int2向量相加 */
inline __host__ __device__ int2 operator+(int b, int2 a)
{
    return make_int2(a.x + b, a.y + b);
}
/** @brief int2向量标量加法赋值运算 */
inline __host__ __device__ void operator+=(int2 &a, int b)
{
    a.x += b;
    a.y += b;
}

// uint2加法运算
/** @brief uint2向量相加 */
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


// float3加法运算
/** @brief float3向量相加 */
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

// int3加法运算
/** @brief int3向量相加 */
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

// uint3加法运算
/** @brief uint3向量相加 */
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

// float4加法运算
/** @brief float4向量相加 */
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

// int4加法运算
/** @brief int4向量相加 */
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

// uint4加法运算
/** @brief uint4向量相加 */
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
// 功能：向量减法，支持向量-向量、向量-标量、标量-向量及对应的-=运算
////////////////////////////////////////////////////////////////////////////////

// float2减法运算
/** @brief float2向量相减：逐分量相减 */
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
// 功能：向量乘法，支持向量*向量(逐分量乘)、向量*标量、标量*向量及对应的*=运算
////////////////////////////////////////////////////////////////////////////////

// float2乘法运算
/** @brief float2向量相乘：逐分量相乘 */
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
// 功能：向量除法，支持向量/向量(逐分量除)、向量/标量、标量/向量及对应的/=运算
////////////////////////////////////////////////////////////////////////////////

// float2除法运算
/** @brief float2向量相除：逐分量相除 */
inline __host__ __device__ float2 operator/(float2 a, float2 b)
{
    return make_float2(a.x / b.x, a.y / b.y);
}
/** @brief float2向量除法赋值运算 */
inline __host__ __device__ void operator/=(float2 &a, float2 b)
{
    a.x /= b.x;
    a.y /= b.y;
}
/** @brief float2向量除以标量：所有分量除以标量 */
inline __host__ __device__ float2 operator/(float2 a, float b)
{
    return make_float2(a.x / b, a.y / b);
}
/** @brief float2向量标量除法赋值运算 */
inline __host__ __device__ void operator/=(float2 &a, float b)
{
    a.x /= b;
    a.y /= b;
}
/** @brief 标量除以float2向量：标量除以每个分量 */
inline __host__ __device__ float2 operator/(float b, float2 a)
{
    return make_float2(b / a.x, b / a.y);
}

// float3除法运算
/** @brief float3向量相除 */
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

// float4除法运算
/** @brief float4向量相除 */
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
// 最小值运算
// 功能：返回两个向量按分量比较的最小值向量
////////////////////////////////////////////////////////////////////////////////

// float向量最小值运算
/** @brief 返回两个float2向量按分量比较的最小值 */
inline  __host__ __device__ float2 fminf(float2 a, float2 b)
{
    return make_float2(fminf(a.x,b.x), fminf(a.y,b.y));
}
/** @brief 返回两个float3向量按分量比较的最小值 */
inline __host__ __device__ float3 fminf(float3 a, float3 b)
{
    return make_float3(fminf(a.x,b.x), fminf(a.y,b.y), fminf(a.z,b.z));
}
/** @brief 返回两个float4向量按分量比较的最小值 */
inline  __host__ __device__ float4 fminf(float4 a, float4 b)
{
    return make_float4(fminf(a.x,b.x), fminf(a.y,b.y), fminf(a.z,b.z), fminf(a.w,b.w));
}

// int向量最小值运算
/** @brief 返回两个int2向量按分量比较的最小值 */
inline __host__ __device__ int2 min(int2 a, int2 b)
{
    return make_int2(min(a.x,b.x), min(a.y,b.y));
}
/** @brief 返回两个int3向量按分量比较的最小值 */
inline __host__ __device__ int3 min(int3 a, int3 b)
{
    return make_int3(min(a.x,b.x), min(a.y,b.y), min(a.z,b.z));
}
/** @brief 返回两个int4向量按分量比较的最小值 */
inline __host__ __device__ int4 min(int4 a, int4 b)
{
    return make_int4(min(a.x,b.x), min(a.y,b.y), min(a.z,b.z), min(a.w,b.w));
}

// uint向量最小值运算
/** @brief 返回两个uint2向量按分量比较的最小值 */
inline __host__ __device__ uint2 min(uint2 a, uint2 b)
{
    return make_uint2(min(a.x,b.x), min(a.y,b.y));
}
/** @brief 返回两个uint3向量按分量比较的最小值 */
inline __host__ __device__ uint3 min(uint3 a, uint3 b)
{
    return make_uint3(min(a.x,b.x), min(a.y,b.y), min(a.z,b.z));
}
/** @brief 返回两个uint4向量按分量比较的最小值 */
inline __host__ __device__ uint4 min(uint4 a, uint4 b)
{
    return make_uint4(min(a.x,b.x), min(a.y,b.y), min(a.z,b.z), min(a.w,b.w));
}

////////////////////////////////////////////////////////////////////////////////
// max
// 最大值运算
// 功能：返回两个向量按分量比较的最大值向量
////////////////////////////////////////////////////////////////////////////////

// float向量最大值运算
/** @brief 返回两个float2向量按分量比较的最大值 */
inline __host__ __device__ float2 fmaxf(float2 a, float2 b)
{
    return make_float2(fmaxf(a.x,b.x), fmaxf(a.y,b.y));
}
/** @brief 返回两个float3向量按分量比较的最大值 */
inline __host__ __device__ float3 fmaxf(float3 a, float3 b)
{
    return make_float3(fmaxf(a.x,b.x), fmaxf(a.y,b.y), fmaxf(a.z,b.z));
}
/** @brief 返回两个float4向量按分量比较的最大值 */
inline __host__ __device__ float4 fmaxf(float4 a, float4 b)
{
    return make_float4(fmaxf(a.x,b.x), fmaxf(a.y,b.y), fmaxf(a.z,b.z), fmaxf(a.w,b.w));
}

// int向量最大值运算
/** @brief 返回两个int2向量按分量比较的最大值 */
inline __host__ __device__ int2 max(int2 a, int2 b)
{
    return make_int2(max(a.x,b.x), max(a.y,b.y));
}
/** @brief 返回两个int3向量按分量比较的最大值 */
inline __host__ __device__ int3 max(int3 a, int3 b)
{
    return make_int3(max(a.x,b.x), max(a.y,b.y), max(a.z,b.z));
}
/** @brief 返回两个int4向量按分量比较的最大值 */
inline __host__ __device__ int4 max(int4 a, int4 b)
{
    return make_int4(max(a.x,b.x), max(a.y,b.y), max(a.z,b.z), max(a.w,b.w));
}

// uint向量最大值运算
/** @brief 返回两个uint2向量按分量比较的最大值 */
inline __host__ __device__ uint2 max(uint2 a, uint2 b)
{
    return make_uint2(max(a.x,b.x), max(a.y,b.y));
}
/** @brief 返回两个uint3向量按分量比较的最大值 */
inline __host__ __device__ uint3 max(uint3 a, uint3 b)
{
    return make_uint3(max(a.x,b.x), max(a.y,b.y), max(a.z,b.z));
}
/** @brief 返回两个uint4向量按分量比较的最大值 */
inline __host__ __device__ uint4 max(uint4 a, uint4 b)
{
    return make_uint4(max(a.x,b.x), max(a.y,b.y), max(a.z,b.z), max(a.w,b.w));
}

////////////////////////////////////////////////////////////////////////////////
// lerp
// 线性插值运算
// 功能：在a和b之间进行线性插值，基于参数t (t在[0,1]范围内)
// 公式：lerp(a, b, t) = a + t*(b-a)
//      当t=0时返回a，当t=1时返回b，当t∈(0,1)时返回中间值
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 标量线性插值
 * @param a 起始值
 * @param b 结束值
 * @param t 插值参数，范围[0,1]
 * @return 插值结果
 */
inline __device__ __host__ float lerp(float a, float b, float t)
{
    return a + t*(b-a);
}
/** @brief float2向量线性插值 */
inline __device__ __host__ float2 lerp(float2 a, float2 b, float t)
{
    return a + t*(b-a);
}
/** @brief float3向量线性插值 */
inline __device__ __host__ float3 lerp(float3 a, float3 b, float t)
{
    return a + t*(b-a);
}
/** @brief float4向量线性插值 */
inline __device__ __host__ float4 lerp(float4 a, float4 b, float t)
{
    return a + t*(b-a);
}

////////////////////////////////////////////////////////////////////////////////
// clamp
// 限幅运算
// 功能：将值v限制在[a, b]范围内
// 如果v<a则返回a，如果v>b则返回b，否则返回v
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 标量限幅
 * @param f 待限幅的值
 * @param a 下限
 * @param b 上限
 * @return 限幅后的值
 */
inline __device__ __host__ float clamp(float f, float a, float b)
{
    return fmaxf(a, fminf(f, b));
}
/** @brief 整型限幅 */
inline __device__ __host__ int clamp(int f, int a, int b)
{
    return max(a, min(f, b));
}
/** @brief 无符号整型限幅 */
inline __device__ __host__ uint clamp(uint f, uint a, uint b)
{
    return max(a, min(f, b));
}

// float2向量限幅
/** @brief float2向量限幅，使用标量上下限 */
inline __device__ __host__ float2 clamp(float2 v, float a, float b)
{
    return make_float2(clamp(v.x, a, b), clamp(v.y, a, b));
}
/** @brief float2向量限幅，使用向量上下限(逐分量限制) */
inline __device__ __host__ float2 clamp(float2 v, float2 a, float2 b)
{
    return make_float2(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y));
}
// float3向量限幅
/** @brief float3向量限幅，使用标量上下限 */
inline __device__ __host__ float3 clamp(float3 v, float a, float b)
{
    return make_float3(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b));
}
/** @brief float3向量限幅，使用向量上下限(逐分量限制) */
inline __device__ __host__ float3 clamp(float3 v, float3 a, float3 b)
{
    return make_float3(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z));
}
// float4向量限幅
/** @brief float4向量限幅，使用标量上下限 */
inline __device__ __host__ float4 clamp(float4 v, float a, float b)
{
    return make_float4(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b), clamp(v.w, a, b));
}
/** @brief float4向量限幅，使用向量上下限(逐分量限制) */
inline __device__ __host__ float4 clamp(float4 v, float4 a, float4 b)
{
    return make_float4(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z), clamp(v.w, a.w, b.w));
}

// int2向量限幅
/** @brief int2向量限幅，使用标量上下限 */
inline __device__ __host__ int2 clamp(int2 v, int a, int b)
{
    return make_int2(clamp(v.x, a, b), clamp(v.y, a, b));
}
/** @brief int2向量限幅，使用向量上下限(逐分量限制) */
inline __device__ __host__ int2 clamp(int2 v, int2 a, int2 b)
{
    return make_int2(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y));
}
// int3向量限幅
/** @brief int3向量限幅，使用标量上下限 */
inline __device__ __host__ int3 clamp(int3 v, int a, int b)
{
    return make_int3(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b));
}
/** @brief int3向量限幅，使用向量上下限(逐分量限制) */
inline __device__ __host__ int3 clamp(int3 v, int3 a, int3 b)
{
    return make_int3(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z));
}
// int4向量限幅
/** @brief int4向量限幅，使用标量上下限 */
inline __device__ __host__ int4 clamp(int4 v, int a, int b)
{
    return make_int4(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b), clamp(v.w, a, b));
}
/** @brief int4向量限幅，使用向量上下限(逐分量限制) */
inline __device__ __host__ int4 clamp(int4 v, int4 a, int4 b)
{
    return make_int4(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z), clamp(v.w, a.w, b.w));
}

// uint2向量限幅
/** @brief uint2向量限幅，使用标量上下限 */
inline __device__ __host__ uint2 clamp(uint2 v, uint a, uint b)
{
    return make_uint2(clamp(v.x, a, b), clamp(v.y, a, b));
}
/** @brief uint2向量限幅，使用向量上下限(逐分量限制) */
inline __device__ __host__ uint2 clamp(uint2 v, uint2 a, uint2 b)
{
    return make_uint2(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y));
}
// uint3向量限幅
/** @brief uint3向量限幅，使用标量上下限 */
inline __device__ __host__ uint3 clamp(uint3 v, uint a, uint b)
{
    return make_uint3(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b));
}
/** @brief uint3向量限幅，使用向量上下限(逐分量限制) */
inline __device__ __host__ uint3 clamp(uint3 v, uint3 a, uint3 b)
{
    return make_uint3(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z));
}
// uint4向量限幅
/** @brief uint4向量限幅，使用标量上下限 */
inline __device__ __host__ uint4 clamp(uint4 v, uint a, uint b)
{
    return make_uint4(clamp(v.x, a, b), clamp(v.y, a, b), clamp(v.z, a, b), clamp(v.w, a, b));
}
/** @brief uint4向量限幅，使用向量上下限(逐分量限制) */
inline __device__ __host__ uint4 clamp(uint4 v, uint4 a, uint4 b)
{
    return make_uint4(clamp(v.x, a.x, b.x), clamp(v.y, a.y, b.y), clamp(v.z, a.z, b.z), clamp(v.w, a.w, b.w));
}

////////////////////////////////////////////////////////////////////////////////
// dot product
// 点积运算
// 功能：计算两个向量的点积(内积)
// 公式：dot(a,b) = a.x*b.x + a.y*b.y + ... (所有对应分量的乘积之和)
////////////////////////////////////////////////////////////////////////////////

// float向量点积
/** @brief 计算两个float2向量的点积 */
inline __host__ __device__ float dot(float2 a, float2 b)
{
    return a.x * b.x + a.y * b.y;
}
/** @brief 计算两个float3向量的点积 */
inline __host__ __device__ float dot(float3 a, float3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
/** @brief 计算两个float4向量的点积 */
inline __host__ __device__ float dot(float4 a, float4 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

// int向量点积
/** @brief 计算两个int2向量的点积 */
inline __host__ __device__ int dot(int2 a, int2 b)
{
    return a.x * b.x + a.y * b.y;
}
/** @brief 计算两个int3向量的点积 */
inline __host__ __device__ int dot(int3 a, int3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
/** @brief 计算两个int4向量的点积 */
inline __host__ __device__ int dot(int4 a, int4 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

// uint向量点积
/** @brief 计算两个uint2向量的点积 */
inline __host__ __device__ uint dot(uint2 a, uint2 b)
{
    return a.x * b.x + a.y * b.y;
}
/** @brief 计算两个uint3向量的点积 */
inline __host__ __device__ uint dot(uint3 a, uint3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
/** @brief 计算两个uint4向量的点积 */
inline __host__ __device__ uint dot(uint4 a, uint4 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

////////////////////////////////////////////////////////////////////////////////
// length
// 向量长度计算
// 功能：计算向量的欧几里得长度(模)
// 公式：length(v) = sqrt(v.x² + v.y² + ... ) = sqrt(dot(v, v))
////////////////////////////////////////////////////////////////////////////////

/** @brief 计算float2向量的长度 */
inline __host__ __device__ float length(float2 v)
{
    return sqrtf(dot(v, v));
}
/** @brief 计算float3向量的长度 */
inline __host__ __device__ float length(float3 v)
{
    return sqrtf(dot(v, v));
}
/** @brief 计算float4向量的长度 */
inline __host__ __device__ float length(float4 v)
{
    return sqrtf(dot(v, v));
}

////////////////////////////////////////////////////////////////////////////////
// normalize
// 向量归一化
// 功能：将向量归一化为单位向量(长度为1的向量)
// 公式：normalize(v) = v / length(v) = v * (1 / sqrt(dot(v,v)))
////////////////////////////////////////////////////////////////////////////////

/** @brief 归一化float2向量，使其长度为1 */
inline __host__ __device__ float2 normalize(float2 v)
{
    float invLen = rsqrtf(dot(v, v));  // rsqrtf计算平方根的倒数，效率更高
    return v * invLen;
}
/** @brief 归一化float3向量，使其长度为1 */
inline __host__ __device__ float3 normalize(float3 v)
{
    float invLen = rsqrtf(dot(v, v));
    return v * invLen;
}
/** @brief 归一化float4向量，使其长度为1 */
inline __host__ __device__ float4 normalize(float4 v)
{
    float invLen = rsqrtf(dot(v, v));
    return v * invLen;
}

////////////////////////////////////////////////////////////////////////////////
// floor
// 向下取整运算
// 功能：对向量的每个分量向下取整到最接近的整数
////////////////////////////////////////////////////////////////////////////////

/** @brief 对float2向量的每个分量向下取整 */
inline __host__ __device__ float2 floorf(float2 v)
{
    return make_float2(floorf(v.x), floorf(v.y));
}
/** @brief 对float3向量的每个分量向下取整 */
inline __host__ __device__ float3 floorf(float3 v)
{
    return make_float3(floorf(v.x), floorf(v.y), floorf(v.z));
}
/** @brief 对float4向量的每个分量向下取整 */
inline __host__ __device__ float4 floorf(float4 v)
{
    return make_float4(floorf(v.x), floorf(v.y), floorf(v.z), floorf(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// frac - returns the fractional portion of a scalar or each vector component
// 小数部分提取
// 功能：返回标量或向量各分量的小数部分
// 公式：frac(v) = v - floor(v)，例如frac(3.7) = 0.7, frac(-2.3) = 0.7
////////////////////////////////////////////////////////////////////////////////

/** @brief 提取标量的小数部分 */
inline __host__ __device__ float fracf(float v)
{
    return v - floorf(v);
}
/** @brief 提取float2向量各分量的小数部分 */
inline __host__ __device__ float2 fracf(float2 v)
{
    return make_float2(fracf(v.x), fracf(v.y));
}
/** @brief 提取float3向量各分量的小数部分 */
inline __host__ __device__ float3 fracf(float3 v)
{
    return make_float3(fracf(v.x), fracf(v.y), fracf(v.z));
}
/** @brief 提取float4向量各分量的小数部分 */
inline __host__ __device__ float4 fracf(float4 v)
{
    return make_float4(fracf(v.x), fracf(v.y), fracf(v.z), fracf(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// fmod
// 浮点数取模运算
// 功能：计算浮点数除法的余数，对向量逐分量进行取模
// 公式：fmod(a, b) = a - n*b，其中n为a/b向零取整的结果
////////////////////////////////////////////////////////////////////////////////

/** @brief 对float2向量逐分量进行取模运算 */
inline __host__ __device__ float2 fmodf(float2 a, float2 b)
{
    return make_float2(fmodf(a.x, b.x), fmodf(a.y, b.y));
}
/** @brief 对float3向量逐分量进行取模运算 */
inline __host__ __device__ float3 fmodf(float3 a, float3 b)
{
    return make_float3(fmodf(a.x, b.x), fmodf(a.y, b.y), fmodf(a.z, b.z));
}
/** @brief 对float4向量逐分量进行取模运算 */
inline __host__ __device__ float4 fmodf(float4 a, float4 b)
{
    return make_float4(fmodf(a.x, b.x), fmodf(a.y, b.y), fmodf(a.z, b.z), fmodf(a.w, b.w));
}

////////////////////////////////////////////////////////////////////////////////
// absolute value
// 绝对值运算
// 功能：返回向量各分量的绝对值
////////////////////////////////////////////////////////////////////////////////

// float向量绝对值
/** @brief 返回float2向量各分量的绝对值 */
inline __host__ __device__ float2 fabs(float2 v)
{
    return make_float2(fabs(v.x), fabs(v.y));
}
/** @brief 返回float3向量各分量的绝对值 */
inline __host__ __device__ float3 fabs(float3 v)
{
    return make_float3(fabs(v.x), fabs(v.y), fabs(v.z));
}
/** @brief 返回float4向量各分量的绝对值 */
inline __host__ __device__ float4 fabs(float4 v)
{
    return make_float4(fabs(v.x), fabs(v.y), fabs(v.z), fabs(v.w));
}

// int向量绝对值
/** @brief 返回int2向量各分量的绝对值 */
inline __host__ __device__ int2 abs(int2 v)
{
    return make_int2(abs(v.x), abs(v.y));
}
/** @brief 返回int3向量各分量的绝对值 */
inline __host__ __device__ int3 abs(int3 v)
{
    return make_int3(abs(v.x), abs(v.y), abs(v.z));
}
/** @brief 返回int4向量各分量的绝对值 */
inline __host__ __device__ int4 abs(int4 v)
{
    return make_int4(abs(v.x), abs(v.y), abs(v.z), abs(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// reflect
// 反射向量计算
// 功能：计算入射光线I关于表面法线N的反射向量
// 注意：N应该是归一化的单位向量，反射向量的长度等于入射向量I的长度
// 公式：reflect(I, N) = I - 2*dot(N,I)*N
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 计算反射向量
 * @param i 入射向量
 * @param n 表面法线(应为归一化向量)
 * @return 反射向量
 */
inline __host__ __device__ float3 reflect(float3 i, float3 n)
{
    return i - 2.0f * n * dot(n,i);
}

////////////////////////////////////////////////////////////////////////////////
// cross product
// 叉积运算
// 功能：计算两个三维向量的叉积(外积)
// 公式：cross(a,b) = (a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x)
// 结果向量同时垂直于a和b，遵循右手定则
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 计算两个float3向量的叉积
 * @param a 第一个向量
 * @param b 第二个向量
 * @return 垂直于a和b的叉积向量
 */
inline __host__ __device__ float3 cross(float3 a, float3 b)
{
    return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

////////////////////////////////////////////////////////////////////////////////
// smoothstep
// 平滑阶跃函数
// 功能：在a和b之间进行平滑插值，比线性插值更平滑
// 特性：
//   - 当x < a时返回0
//   - 当x > b时返回1
//   - 当a <= x <= b时返回基于x的平滑插值(使用三次Hermite插值)
// 公式：t = clamp((x-a)/(b-a), 0, 1); smoothstep = t²(3-2t)
// 应用：常用于图形学中的平滑过渡、边缘柔化等
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 标量平滑阶跃插值
 * @param a 下限
 * @param b 上限
 * @param x 输入值
 * @return 平滑插值结果，范围[0,1]
 */
inline __device__ __host__ float smoothstep(float a, float b, float x)
{
    float y = clamp((x - a) / (b - a), 0.0f, 1.0f);  // 归一化到[0,1]
    return (y*y*(3.0f - (2.0f*y)));  // 三次Hermite插值公式
}
/** @brief float2向量平滑阶跃插值(逐分量) */
inline __device__ __host__ float2 smoothstep(float2 a, float2 b, float2 x)
{
    float2 y = clamp((x - a) / (b - a), 0.0f, 1.0f);
    return (y*y*(make_float2(3.0f) - (make_float2(2.0f)*y)));
}
/** @brief float3向量平滑阶跃插值(逐分量) */
inline __device__ __host__ float3 smoothstep(float3 a, float3 b, float3 x)
{
    float3 y = clamp((x - a) / (b - a), 0.0f, 1.0f);
    return (y*y*(make_float3(3.0f) - (make_float3(2.0f)*y)));
}
/** @brief float4向量平滑阶跃插值(逐分量) */
inline __device__ __host__ float4 smoothstep(float4 a, float4 b, float4 x)
{
    float4 y = clamp((x - a) / (b - a), 0.0f, 1.0f);
    return (y*y*(make_float4(3.0f) - (make_float4(2.0f)*y)));
}

#endif  // HELPER_MATH_H
