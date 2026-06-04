/*
 * 梯度下降优化器头文件
 *
 * 功能说明：
 * 实现了一个通用的梯度下降优化算法，用于解决非线性优化问题。
 * 该优化器支持自定义目标函数，并提供多种终止条件（最大迭代次数、梯度阈值等）。
 * 在Ego-Planner中主要用于B样条轨迹的优化，通过迭代方式调整控制点以最小化代价函数。
 */

#ifndef _GRADIENT_DESCENT_OPT_H_
#define _GRADIENT_DESCENT_OPT_H_

#include <iostream>
#include <vector>
#include <Eigen/Eigen>

using namespace std;

/**
 * @class GradientDescentOptimizer
 * @brief 梯度下降优化器类
 *
 * 该类实现了基于梯度下降的优化算法，用于求解多元函数的最小值问题。
 * 优化器通过迭代更新变量值，沿着目标函数梯度的负方向移动，直到满足收敛条件。
 *
 * 主要特点：
 * 1. 支持自定义目标函数（通过函数指针传入）
 * 2. 提供多种终止条件：最大迭代次数、梯度阈值、相对/绝对容差
 * 3. 支持传递自定义数据给目标函数
 * 4. 返回优化结果状态（成功、失败、达到最大迭代次数等）
 */
class GradientDescentOptimizer
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // Eigen库要求：确保动态分配的内存对齐，用于存储Eigen数据结构

  /**
   * @typedef objfunDef
   * @brief 目标函数的函数指针类型定义
   *
   * @param x 当前优化变量（输入）
   * @param grad 梯度向量（输出，由目标函数计算）
   * @param force_return 强制返回标志（输出，用于提前终止优化）
   * @param data 用户自定义数据指针，可传递额外信息给目标函数
   * @return 目标函数在当前点x处的函数值
   *
   * 说明：
   * - x: 优化变量的当前值
   * - grad: 函数在x处的梯度，由目标函数计算并填充
   * - force_return: 如果目标函数设置为true，优化器将立即停止迭代
   * - data: 指向额外数据的指针，可用于传递优化问题的上下文信息
   */
  typedef double (*objfunDef)(const Eigen::VectorXd &x, Eigen::VectorXd &grad, bool &force_return, void *data);

  /**
   * @enum RESULT
   * @brief 优化结果状态枚举
   */
  enum RESULT
  {
    FIND_MIN,              // 成功找到局部最小值（满足收敛条件）
    FAILED,                // 优化失败（可能由于数值问题或其他错误）
    RETURN_BY_ORDER,       // 由目标函数强制返回（force_return被设置为true）
    REACH_MAX_ITERATION    // 达到最大迭代次数限制
  };

  /**
   * @brief 构造函数
   *
   * @param v_num 优化变量的维度（变量个数）
   * @param objf 目标函数指针，用于计算函数值和梯度
   * @param f_data 传递给目标函数的用户数据指针
   *
   * 说明：
   * 初始化优化器，设置优化问题的基本参数。目标函数和数据指针将在优化过程中使用。
   */
  GradientDescentOptimizer(int v_num, objfunDef objf, void *f_data)
  {
    variable_num_ = v_num;  // 设置优化变量维度
    objfun_ = objf;         // 设置目标函数
    f_data_ = f_data;       // 设置用户数据指针
  };

  // ==================== 参数设置接口 ====================

  /**
   * @brief 设置最大迭代次数
   * @param limit 迭代次数上限
   */
  void set_maxiter(int limit) { iter_limit_ = limit; }

  /**
   * @brief 设置目标函数最大调用次数
   * @param limit 函数调用次数上限
   */
  void set_maxeval(int limit) { invoke_limit_ = limit; }

  /**
   * @brief 设置相对容差
   * @param xtol_rel 相对容差值，用于判断优化变量的相对变化是否足够小
   *
   * 说明：当 ||x_new - x_old|| / ||x_old|| < xtol_rel 时，认为收敛
   */
  void set_xtol_rel(double xtol_rel) { xtol_rel_ = xtol_rel; }

  /**
   * @brief 设置绝对容差
   * @param xtol_abs 绝对容差值，用于判断优化变量的绝对变化是否足够小
   *
   * 说明：当 ||x_new - x_old|| < xtol_abs 时，认为收敛
   */
  void set_xtol_abs(double xtol_abs) { xtol_abs_ = xtol_abs; }

  /**
   * @brief 设置最小梯度阈值
   * @param min_grad 梯度的最小范数，用于判断是否接近驻点
   *
   * 说明：当 ||grad|| < min_grad 时，认为已接近局部最小值（梯度接近零）
   */
  void set_min_grad(double min_grad) { min_grad_ = min_grad; }

  /**
   * @brief 执行优化
   *
   * @param x_init_optimal 输入输出参数：输入为初始值，输出为最优解
   * @param opt_f 输出参数：最优目标函数值
   * @return RESULT 优化结果状态
   *
   * 说明：
   * 这是优化器的核心函数，从给定的初始点开始，通过梯度下降迭代寻找最优解。
   * 优化过程会持续迭代直到满足以下任一条件：
   * 1. 梯度范数小于min_grad（接近驻点）
   * 2. 变量变化小于容差（相对或绝对）
   * 3. 达到最大迭代次数或函数调用次数
   * 4. 目标函数设置force_return标志
   */
  RESULT optimize(Eigen::VectorXd &x_init_optimal, double &opt_f);

private:
  // ==================== 优化器内部参数 ====================

  int variable_num_{0};        // 优化变量的维度（数量）
  int iter_limit_{1e10};       // 最大迭代次数限制（默认值很大，相当于无限制）
  int invoke_limit_{1e10};     // 目标函数最大调用次数限制
  double xtol_rel_;            // 相对容差：用于判断优化变量的相对变化
  double xtol_abs_;            // 绝对容差：用于判断优化变量的绝对变化
  double min_grad_;            // 最小梯度阈值：当梯度范数小于此值时停止优化
  double time_limit_;          // 时间限制（当前未使用）
  void *f_data_;               // 用户数据指针：传递给目标函数的额外数据
  objfunDef objfun_;           // 目标函数指针：用于计算函数值和梯度
};

#endif
