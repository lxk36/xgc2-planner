/**
 * @file gradient_descent_optimizer.cpp
 * @brief 梯度下降优化器实现文件
 * @description 实现基于BB步长（Barzilai-Borwein步长）的梯度下降优化算法
 *              用于Ego-Planner中的B样条轨迹优化，结合Armijo线搜索保证收敛性
 */

#include <ego_bspline_opt/gradient_descent_optimizer.h>

// 终端输出颜色控制宏定义
#define RESET "\033[0m"  // 重置颜色
#define RED "\033[31m"   // 红色（用于错误信息）

/**
 * @brief 梯度下降优化主函数
 * @description 使用BB步长的梯度下降算法优化目标函数，结合Armijo条件进行线搜索
 *
 * @param x_init_optimal 输入/输出：初始优化变量向量，优化完成后存储最优解
 * @param opt_f 输出：最优目标函数值
 * @return RESULT 优化结果状态（FIND_MIN/REACH_MAX_ITERATION/FAILED/RETURN_BY_ORDER）
 *
 * @note 算法特点：
 *       1. 使用BB步长自适应调整学习率，收敛速度快
 *       2. 通过Armijo准则进行回溯线搜索，保证每步都能降低目标函数值
 *       3. 采用交替更新策略（奇偶迭代），减少内存拷贝操作
 */
GradientDescentOptimizer::RESULT
GradientDescentOptimizer::optimize(Eigen::VectorXd &x_init_optimal, double &opt_f)
{
    // 参数有效性检查：最小梯度阈值必须大于极小值，防止过早终止
    if (min_grad_ < 1e-10)
    {
        cout << RED << "min_grad_ is invalid:" << min_grad_ << RESET << endl;
        return FAILED;
    }
    // 参数有效性检查：迭代次数限制必须大于2，保证至少完成初始化迭代
    if (iter_limit_ <= 2)
    {
        cout << RED << "iter_limit_ is invalid:" << iter_limit_ << RESET << endl;
        return FAILED;
    }

    // ========== 第一阶段：初始化变量和执行前两次迭代 ==========

    void *f_data = f_data_;  // 目标函数所需的额外数据指针
    int iter = 2;            // 迭代计数器，从2开始（因为前两次迭代在此完成）
    int invoke_count = 2;    // 目标函数调用次数计数器
    bool force_return;       // 强制返回标志（由目标函数设置）

    // 定义优化变量：x_k为当前解，x_kp1为下一步解
    Eigen::VectorXd x_k(x_init_optimal), x_kp1(x_init_optimal.rows());
    // 定义目标函数值：cost_k为当前代价，cost_kp1为下一步代价，cost_min为历史最小代价
    double cost_k, cost_kp1, cost_min;
    // 定义梯度向量：grad_k为当前梯度，grad_kp1为下一步梯度
    Eigen::VectorXd grad_k(x_init_optimal.rows()), grad_kp1(x_init_optimal.rows());

    // 第一次迭代：计算初始点的目标函数值和梯度
    cost_k = objfun_(x_k, grad_k, force_return, f_data);
    if (force_return)  // 如果目标函数要求强制返回（如检测到碰撞等紧急情况）
        return RETURN_BY_ORDER;
    cost_min = cost_k;  // 初始化历史最小代价

    // 计算初始步长alpha0：限制第一次迭代的最大移动距离，防止步长过大导致发散
    double max_grad = max(abs(grad_k.maxCoeff()), abs(grad_k.minCoeff()));  // 梯度的最大绝对值
    constexpr double MAX_MOVEMENT_AT_FIRST_ITERATION = 0.1;  // 第一次迭代的最大移动距离（米）
    // 如果梯度较小则步长为1.0，否则缩放步长使移动距离不超过0.1米
    double alpha0 = max_grad < MAX_MOVEMENT_AT_FIRST_ITERATION ? 1.0 : (MAX_MOVEMENT_AT_FIRST_ITERATION / max_grad);

    // 第二次迭代：使用初始步长进行梯度下降
    x_kp1 = x_k - alpha0 * grad_k;  // 沿负梯度方向更新
    cost_kp1 = objfun_(x_kp1, grad_kp1, force_return, f_data);
    if (force_return)
        return RETURN_BY_ORDER;

    // 更新历史最小代价
    if (cost_min > cost_kp1)
        cost_min = cost_kp1;

    // ========== 第二阶段：主迭代循环（使用BB步长的梯度下降） ==========

    /*** start iteration ***/
    // 循环条件：迭代次数不超过上限 且 目标函数调用次数不超过上限
    while (++iter <= iter_limit_ && invoke_count <= invoke_limit_)
    {
        /* BB步长计算（Barzilai-Borwein方法）
         * 核心思想：利用前后两步的位置差s和梯度差y来估计Hessian矩阵的信息
         * 从而自适应调整步长，在凸优化问题上具有超线性收敛速度
         */
        Eigen::VectorXd s = x_kp1 - x_k;      // s_k = x_{k+1} - x_k（位置变化量）
        Eigen::VectorXd y = grad_kp1 - grad_k; // y_k = ∇f_{k+1} - ∇f_k（梯度变化量）

        // BB步长公式：alpha = (s^T * y) / (y^T * y)
        // 这是BB方法的第二种形式（BB2），也称为短步长BB方法
        double alpha = s.dot(y) / y.dot(y);

        // 步长有效性检查：防止数值计算错误导致的NaN或Inf
        if (isnan(alpha) || isinf(alpha))
        {
            cout << RED << "step size invalid! alpha=" << alpha << RESET << endl;
            return FAILED;
        }

        /* 交替更新策略：避免不必要的内存拷贝操作
         * 奇数次迭代：更新x_k，保持x_kp1不变
         * 偶数次迭代：更新x_kp1，保持x_k不变
         * 这样可以减少变量复制，提高计算效率
         */
        if (iter % 2) // 奇数次迭代：更新x_k
        {
            /* Armijo回溯线搜索
             * 目标：找到满足Armijo条件的步长，保证目标函数充分下降
             * Armijo条件：f(x - α∇f) ≤ f(x) - c*α*‖∇f‖²，其中c=1e-4
             * 如果不满足，则将步长减半继续尝试
             */
            do
            {
                x_k = x_kp1 - alpha * grad_kp1;  // 沿负梯度方向更新x_k
                cost_k = objfun_(x_k, grad_k, force_return, f_data);  // 计算新点的目标函数值和梯度
                invoke_count++;  // 增加函数调用计数
                if (force_return)  // 检查是否需要强制返回
                    return RETURN_BY_ORDER;
                alpha *= 0.5;  // 步长减半（回溯）
            } while (cost_k > cost_kp1 - 1e-4 * alpha * grad_kp1.transpose() * grad_kp1); // Armijo条件检查

            // 收敛性检查：如果梯度范数小于阈值，说明已到达局部最小值
            if (grad_k.norm() < min_grad_)
            {
                opt_f = cost_k;  // 输出最优目标函数值
                return FIND_MIN;  // 返回找到最小值状态
            }
        }
        else  // 偶数次迭代：更新x_kp1
        {
            /* Armijo回溯线搜索（与奇数次迭代对称）
             * 在偶数次迭代中，基于x_k的梯度更新x_kp1
             */
            do
            {
                x_kp1 = x_k - alpha * grad_k;  // 沿负梯度方向更新x_kp1
                cost_kp1 = objfun_(x_kp1, grad_kp1, force_return, f_data);  // 计算新点的目标函数值和梯度
                invoke_count++;  // 增加函数调用计数
                if (force_return)  // 检查是否需要强制返回
                    return RETURN_BY_ORDER;
                alpha *= 0.5;  // 步长减半（回溯）
            } while (cost_kp1 > cost_k - 1e-4 * alpha * grad_k.transpose() * grad_k); // Armijo条件检查

            // 收敛性检查：如果梯度范数小于阈值，说明已到达局部最小值
            if (grad_kp1.norm() < min_grad_)
            {
                opt_f = cost_kp1;  // 输出最优目标函数值
                return FIND_MIN;   // 返回找到最小值状态
            }
        }
    }  // end while loop

    // ========== 第三阶段：达到最大迭代次数，返回当前最优解 ==========

    // 根据最后一次迭代的奇偶性，选择对应的目标函数值作为输出
    // 如果iter_limit_是奇数，最后一次更新的是x_k，使用cost_k
    // 如果iter_limit_是偶数，最后一次更新的是x_kp1，使用cost_kp1
    opt_f = iter_limit_ % 2 ? cost_k : cost_kp1;
    return REACH_MAX_ITERATION;  // 返回达到最大迭代次数状态
}
