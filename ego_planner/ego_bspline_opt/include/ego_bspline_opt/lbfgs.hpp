/**
 * @file lbfgs.hpp
 * @brief L-BFGS (Limited-memory Broyden–Fletcher–Goldfarb–Shanno) 优化算法实现
 *
 * L-BFGS是一种用于求解大规模无约束优化问题的拟牛顿方法。
 * 相比于标准的BFGS算法,L-BFGS通过仅存储有限数量的历史迭代信息来近似Hessian矩阵的逆,
 * 从而大幅降低内存消耗,适用于高维优化问题。
 *
 * 算法核心思想:
 * 1. 使用最近m次迭代的位置和梯度差值来近似Hessian逆矩阵
 * 2. 通过两循环递归(two-loop recursion)计算搜索方向
 * 3. 使用More-Thuente线搜索方法确定步长
 */

#ifndef LBFGS_HPP
#define LBFGS_HPP

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

namespace lbfgs
{
    // ----------------------- 数据类型定义部分 -----------------------

    /**
     * @brief lbfgs_optimize() 函数的返回值枚举
     *
     * 负数表示错误状态,非负数表示成功或达到停止条件
     */

    enum
    {
        /** L-BFGS达到收敛状态 (梯度范数满足终止条件) */
        LBFGS_CONVERGENCE = 0,
        /** L-BFGS满足停止准则 (目标函数下降速率过小) */
        LBFGS_STOP,
        /** 初始变量已经是目标函数的最小值点 */
        LBFGS_ALREADY_MINIMIZED,

        /** 未知错误 */
        LBFGSERR_UNKNOWNERROR = -1024,
        /** 逻辑错误 */
        LBFGSERR_LOGICERROR,
        /** 优化过程被用户取消 */
        LBFGSERR_CANCELED,
        /** 变量数量无效 (必须 > 0) */
        LBFGSERR_INVALID_N,
        /** 无效的历史记忆大小参数 mem_size */
        LBFGSERR_INVALID_MEMSIZE,
        /** 无效的梯度收敛阈值参数 g_epsilon */
        LBFGSERR_INVALID_GEPSILON,
        /** 无效的测试周期参数 past */
        LBFGSERR_INVALID_TESTPERIOD,
        /** 无效的收敛率阈值参数 delta */
        LBFGSERR_INVALID_DELTA,
        /** 无效的最小步长参数 min_step */
        LBFGSERR_INVALID_MINSTEP,
        /** 无效的最大步长参数 max_step */
        LBFGSERR_INVALID_MAXSTEP,
        /** 无效的函数下降系数参数 f_dec_coeff (Armijo条件) */
        LBFGSERR_INVALID_FDECCOEFF,
        /** 无效的曲率系数参数 s_curv_coeff (强Wolfe条件) */
        LBFGSERR_INVALID_SCURVCOEFF,
        /** 无效的相对宽度容忍度参数 xtol */
        LBFGSERR_INVALID_XTOL,
        /** 无效的最大线搜索次数参数 max_linesearch */
        LBFGSERR_INVALID_MAXLINESEARCH,
        /** 线搜索步长超出不确定性区间 */
        LBFGSERR_OUTOFINTERVAL,
        /** 逻辑错误;或者不确定性区间变得过小 */
        LBFGSERR_INCORRECT_TMINMAX,
        /** 舍入误差;或者没有线搜索步长能同时满足充分下降条件和曲率条件 */
        LBFGSERR_ROUNDING_ERROR,
        /** 线搜索步长小于最小步长 min_step */
        LBFGSERR_MINIMUMSTEP,
        /** 线搜索步长大于最大步长 max_step */
        LBFGSERR_MAXIMUMSTEP,
        /** 线搜索达到最大评估次数 */
        LBFGSERR_MAXIMUMLINESEARCH,
        /** 算法达到最大迭代次数 */
        LBFGSERR_MAXIMUMITERATION,
        /** 不确定性区间的相对宽度小于等于 xtol */
        LBFGSERR_WIDTHTOOSMALL,
        /** 逻辑错误 (负的线搜索步长) */
        LBFGSERR_INVALIDPARAMETERS,
        /** 当前搜索方向使目标函数值增加 (非下降方向) */
        LBFGSERR_INCREASEGRADIENT,
    };

    /**
     * @brief L-BFGS优化参数结构体
     *
     * 定义了L-BFGS算法的所有可调参数。
     * 使用 lbfgs_load_default_parameters() 函数可以加载默认参数值。
     */
    struct lbfgs_parameter_t
    {
        /**
         * @brief 用于近似Hessian逆矩阵的修正次数(历史记忆大小)
         *
         * L-BFGS算法存储最近m次迭代的计算结果来近似当前迭代的Hessian逆矩阵。
         * 此参数控制有限记忆的大小(修正次数)。
         * 默认值: 6
         * 建议值: ≥ 3 (小于3不推荐,过大会导致计算时间过长)
         */
        int mem_size;

        /**
         * @brief 梯度范数收敛测试的阈值 epsilon
         *
         * 此参数决定求解的精度。当满足以下条件时,优化终止:
         *     ||g|| < g_epsilon * max(1, ||x||)
         * 其中 ||.|| 表示欧几里得(L2)范数
         * 默认值: 1e-5
         */
        double g_epsilon;

        /**
         * @brief 基于delta的收敛测试的迭代距离
         *
         * 此参数确定计算目标函数下降率时回溯的迭代次数。
         * 如果值为0,则不执行基于delta的收敛测试。
         * 默认值: 0 (不启用)
         */
        int past;

        /**
         * @brief 收敛测试的delta阈值
         *
         * 此参数确定目标函数的最小下降率。当满足以下条件时,算法停止迭代:
         *     (f' - f) / f < delta
         * 其中 f' 是past次迭代前的目标函数值, f 是当前迭代的目标函数值
         * 默认值: 1e-5
         */
        double delta;

        /**
         * @brief 最大迭代次数
         *
         * 当迭代次数超过此参数时,lbfgs_optimize()函数返回
         * LBFGSERR_MAXIMUMITERATION 状态码。
         * 设置为0表示持续优化直到收敛或出错。
         * 默认值: 0 (无限制)
         */
        int max_iterations;

        /**
         * @brief 线搜索的最大尝试次数
         *
         * 此参数控制线搜索例程中每次迭代的函数和梯度评估次数。
         * 默认值: 40
         */
        int max_linesearch;

        /**
         * @brief 线搜索的最小步长
         *
         * 默认值: 1e-20
         * 除非机器的指数范围过大,或问题病态严重,否则无需修改
         */
        double min_step;

        /**
         * @brief 线搜索的最大步长
         *
         * 默认值: 1e+20
         * 除非机器的指数范围过大,或问题病态严重,否则无需修改
         */
        double max_step;

        /**
         * @brief 控制线搜索精度的函数下降系数 (Armijo条件参数)
         *
         * 用于Wolfe条件中的充分下降条件: f(x + αd) ≤ f(x) + c1·α·∇f(x)^T·d
         * 默认值: 1e-4
         * 取值范围: (0, 0.5)
         */
        double f_dec_coeff;

        /**
         * @brief 控制线搜索精度的曲率系数 (强Wolfe条件参数)
         *
         * 用于Wolfe条件中的曲率条件: |∇f(x + αd)^T·d| ≤ c2·|∇f(x)^T·d|
         * 如果函数和梯度评估的计算成本相对迭代成本较低(大规模问题常见),
         * 可以设置较小的值(如0.1)以提高效率。
         * 默认值: 0.9
         * 取值范围: (f_dec_coeff, 1.0), 典型值: 0.1 ~ 0.9
         */
        double s_curv_coeff;

        /**
         * @brief 浮点值的机器精度
         *
         * 客户程序必须设置此正值来估计机器精度。
         * 如果不确定性区间的相对宽度小于此参数,
         * 线搜索例程将以 LBFGSERR_ROUNDING_ERROR 状态码终止。
         * 默认值: 1.0e-16
         */
        double xtol;
    };

    /**
     * @brief 目标函数和梯度评估的回调接口
     *
     * lbfgs_optimize()函数在需要时调用此函数来获取目标函数值及其梯度。
     * 客户程序必须实现此函数,以便在给定当前变量值时评估目标函数值和梯度值。
     *
     * @param instance  客户端传递给lbfgs_optimize()函数的用户数据指针
     * @param x         当前变量值 (输入)
     * @param g         梯度向量 (输出)。回调函数必须计算当前变量的梯度值
     * @param n         变量的数量
     * @return          当前变量下的目标函数值
     */
    typedef double (*lbfgs_evaluate_t)(void *instance,
                                       const double *x,
                                       double *g,
                                       const int n);

    /**
     * @brief 提供当前线搜索开始时步长上界的回调接口
     *
     * lbfgs_optimize()函数调用此函数来获取步长的上界,
     * 输入参数包括线搜索前的变量初始值和当前步向量(可以是下降方向)。
     * 客户程序可以实现此函数以实现更高效的线搜索。
     * 如果不使用此功能,设置为NULL或nullptr即可。
     *
     * @param instance  客户端传递给lbfgs_optimize()函数的用户数据指针
     * @param xp        当前线搜索前的变量值
     * @param d         步向量,可以是下降方向
     * @param n         变量的数量
     * @return          当前线搜索例程中的步长上界,使得 stpbound*d 是最大合理步长
     */
    typedef double (*lbfgs_stepbound_t)(void *instance,
                                        const double *xp,
                                        const double *d,
                                        const int n);

    /**
     * @brief 接收优化过程进度的回调接口
     *
     * lbfgs_optimize()函数在每次迭代时调用此函数。
     * 通过实现此函数,客户程序可以存储或显示优化过程的当前进度。
     * 如果不使用此功能,设置为NULL或nullptr即可。
     *
     * @param instance  客户端传递给lbfgs_optimize()函数的用户数据指针
     * @param x         当前变量值
     * @param g         当前梯度值
     * @param fx        当前目标函数值
     * @param xnorm     变量的欧几里得范数
     * @param gnorm     梯度的欧几里得范数
     * @param step      本次迭代使用的线搜索步长
     * @param n         变量的数量
     * @param k         迭代计数
     * @param ls        本次迭代调用的评估次数
     * @return          返回0继续优化过程;返回非零值将取消优化过程
     */
    typedef int (*lbfgs_progress_t)(void *instance,
                                    const double *x,
                                    const double *g,
                                    const double fx,
                                    const double xnorm,
                                    const double gnorm,
                                    const double step,
                                    int n,
                                    int k,
                                    int ls);

    /**
     * @brief 回调数据结构体
     *
     * 存储所有回调函数指针和相关数据,便于在优化过程中传递
     */
    struct callback_data_t
    {
        int n;                              // 变量数量
        void *instance;                     // 用户自定义数据指针
        lbfgs_evaluate_t proc_evaluate;     // 目标函数和梯度评估回调
        lbfgs_stepbound_t proc_stepbound;   // 步长上界回调
        lbfgs_progress_t proc_progress;     // 进度报告回调
    };

    /**
     * @brief 迭代数据结构体
     *
     * 存储L-BFGS算法中每次迭代的关键信息,用于构建有限记忆近似Hessian逆矩阵
     */
    struct iteration_data_t
    {
        double alpha;   // 存储两循环递归中的 alpha 值
        double *s;      // 位置差向量: s_k = x_{k+1} - x_k (维度 [n])
        double *y;      // 梯度差向量: y_k = g_{k+1} - g_k (维度 [n])
        double ys;      // 内积 y^T * s,等于 1/rho (用于计算搜索方向)
    };

    // ----------------------- 算术运算部分 -----------------------

/**
 * @brief 定义用于计算最小化器的局部变量
 *
 * 这些变量在三次和二次插值函数的最小化器计算中使用
 */
#define USES_MINIMIZER_LBFGS \
    double a, d, gamm, theta, p, q, r, s;

/**
 * @brief 寻找插值三次函数的最小化器
 *
 * 使用两个点及其函数值和导数值来构造三次插值多项式,并求其最小值点。
 * 这是More-Thuente线搜索算法的核心组件。
 *
 * @param cm    插值三次函数的最小化器 (输出)
 * @param u     第一个点的位置
 * @param fu    第一个点的函数值 f(u)
 * @param du    第一个点的导数值 f'(u)
 * @param v     第二个点的位置
 * @param fv    第二个点的函数值 f(v)
 * @param dv    第二个点的导数值 f'(v)
 */
#define CUBIC_MINIMIZER_LBFGS(cm, u, fu, du, v, fv, dv) \
    d = (v) - (u);                                      \
    theta = ((fu) - (fv)) * 3 / d + (du) + (dv);        \
    p = fabs(theta);                                    \
    q = fabs(du);                                       \
    r = fabs(dv);                                       \
    s = p >= q ? p : q;                                 \
    s = s >= r ? s : r;                                 \
    /* gamm = s*sqrt((theta/s)**2 - (du/s) * (dv/s)) */ \
    a = theta / s;                                      \
    gamm = s * sqrt(a * a - ((du) / s) * ((dv) / s));   \
    if ((v) < (u))                                      \
        gamm = -gamm;                                   \
    p = gamm - (du) + theta;                            \
    q = gamm - (du) + gamm + (dv);                      \
    r = p / q;                                          \
    (cm) = (u) + r * d;

/**
 * @brief 寻找插值三次函数的最小化器 (带边界约束版本)
 *
 * 使用两个点及其函数值和导数值来构造三次插值多项式,并求其在给定区间内的最小值点。
 * 这个版本处理了数值稳定性问题,并在必要时返回边界值。
 *
 * @param cm    插值三次函数的最小化器 (输出)
 * @param u     第一个点的位置
 * @param fu    第一个点的函数值 f(u)
 * @param du    第一个点的导数值 f'(u)
 * @param v     第二个点的位置
 * @param fv    第二个点的函数值 f(v)
 * @param dv    第二个点的导数值 f'(v)
 * @param xmin  允许的最小值
 * @param xmax  允许的最大值
 */
#define CUBIC_MINIMIZER2_LBFGS(cm, u, fu, du, v, fv, dv, xmin, xmax) \
    d = (v) - (u);                                                   \
    theta = ((fu) - (fv)) * 3 / d + (du) + (dv);                     \
    p = fabs(theta);                                                 \
    q = fabs(du);                                                    \
    r = fabs(dv);                                                    \
    s = p >= q ? p : q;                                              \
    s = s >= r ? s : r;                                              \
    /* gamm = s*sqrt((theta/s)**2 - (du/s) * (dv/s)) */              \
    a = theta / s;                                                   \
    gamm = a * a - ((du) / s) * ((dv) / s);                          \
    gamm = gamm > 0 ? s * sqrt(gamm) : 0;                            \
    if ((u) < (v))                                                   \
        gamm = -gamm;                                                \
    p = gamm - (dv) + theta;                                         \
    q = gamm - (dv) + gamm + (du);                                   \
    r = p / q;                                                       \
    if (r < 0. && gamm != 0.)                                        \
    {                                                                \
        (cm) = (v)-r * d;                                            \
    }                                                                \
    else if (a < 0)                                                  \
    {                                                                \
        (cm) = (xmax);                                               \
    }                                                                \
    else                                                             \
    {                                                                \
        (cm) = (xmin);                                               \
    }

/**
 * @brief 寻找插值二次函数的最小化器
 *
 * 使用两个点及其函数值和第一个点的导数值来构造二次插值多项式,并求其最小值点。
 *
 * @param qm    插值二次函数的最小化器 (输出)
 * @param u     第一个点的位置
 * @param fu    第一个点的函数值 f(u)
 * @param du    第一个点的导数值 f'(u)
 * @param v     第二个点的位置
 * @param fv    第二个点的函数值 f(v)
 */
#define QUARD_MINIMIZER_LBFGS(qm, u, fu, du, v, fv) \
    a = (v) - (u);                                  \
    (qm) = (u) + (du) / (((fu) - (fv)) / a + (du)) / 2 * a;

/**
 * @brief 寻找插值二次函数的最小化器 (仅使用导数版本)
 *
 * 使用两个点的导数值来构造二次插值多项式(割线法),并求其最小值点。
 *
 * @param qm    插值二次函数的最小化器 (输出)
 * @param u     第一个点的位置
 * @param du    第一个点的导数值 f'(u)
 * @param v     第二个点的位置
 * @param dv    第二个点的导数值 f'(v)
 */
#define QUARD_MINIMIZER2_LBFGS(qm, u, du, v, dv) \
    a = (u) - (v);                               \
    (qm) = (v) + (dv) / ((dv) - (du)) * a;

    /**
     * @brief 分配并初始化向量内存
     *
     * @param size  要分配的字节数
     * @return      分配的内存块指针,如果分配失败则返回NULL
     */
    inline void *vecalloc(size_t size)
    {
        void *memblock = malloc(size);
        if (memblock)
        {
            memset(memblock, 0, size);  // 初始化为0
        }
        return memblock;
    }

    /**
     * @brief 释放向量内存
     *
     * @param memblock  要释放的内存块指针
     */
    inline void vecfree(void *memblock)
    {
        free(memblock);
    }

    /**
     * @brief 向量复制: y = x
     *
     * @param y  目标向量
     * @param x  源向量
     * @param n  向量维度
     */
    inline void veccpy(double *y, const double *x, const int n)
    {
        memcpy(y, x, sizeof(double) * n);
    }

    /**
     * @brief 向量取负复制: y = -x
     *
     * @param y  目标向量
     * @param x  源向量
     * @param n  向量维度
     */
    inline void vecncpy(double *y, const double *x, const int n)
    {
        int i;

        for (i = 0; i < n; ++i)
        {
            y[i] = -x[i];
        }
    }

    /**
     * @brief 向量加法: y = y + c*x (AXPY操作)
     *
     * @param y  目标向量,同时也是输入 (in-place操作)
     * @param x  加数向量
     * @param c  标量系数
     * @param n  向量维度
     */
    inline void vecadd(double *y, const double *x, const double c, const int n)
    {
        int i;

        for (i = 0; i < n; ++i)
        {
            y[i] += c * x[i];
        }
    }

    /**
     * @brief 向量差: z = x - y
     *
     * @param z  结果向量
     * @param x  被减向量
     * @param y  减数向量
     * @param n  向量维度
     */
    inline void vecdiff(double *z, const double *x, const double *y, const int n)
    {
        int i;

        for (i = 0; i < n; ++i)
        {
            z[i] = x[i] - y[i];
        }
    }

    /**
     * @brief 向量标量乘法: y = c*y (in-place操作)
     *
     * @param y  目标向量,同时也是输入
     * @param c  标量系数
     * @param n  向量维度
     */
    inline void vecscale(double *y, const double c, const int n)
    {
        int i;

        for (i = 0; i < n; ++i)
        {
            y[i] *= c;
        }
    }

    /**
     * @brief 向量点积(内积): s = x^T * y
     *
     * @param s  结果标量 (输出)
     * @param x  第一个向量
     * @param y  第二个向量
     * @param n  向量维度
     */
    inline void vecdot(double *s, const double *x, const double *y, const int n)
    {
        int i;
        *s = 0.;
        for (i = 0; i < n; ++i)
        {
            *s += x[i] * y[i];
        }
    }

    /**
     * @brief 计算向量的L2范数(欧几里得范数): s = ||x||_2
     *
     * @param s  结果标量 (输出)
     * @param x  输入向量
     * @param n  向量维度
     */
    inline void vec2norm(double *s, const double *x, const int n)
    {
        vecdot(s, x, x, n);
        *s = (double)sqrt(*s);
    }

    /**
     * @brief 计算向量L2范数的倒数: s = 1/||x||_2
     *
     * @param s  结果标量 (输出)
     * @param x  输入向量
     * @param n  向量维度
     */
    inline void vec2norminv(double *s, const double *x, const int n)
    {
        vec2norm(s, x, n);
        *s = (double)(1.0 / *s);
    }

    // ----------------------- L-BFGS算法核心部分 -----------------------

    /**
     * @brief 更新线搜索的保护试探值和不确定性区间
     *
     * 这是More-Thuente线搜索算法的核心函数,用于更新搜索区间。
     * 参数x代表具有最小函数值的步长点,参数t代表当前试探步长。
     * 假设在点x处沿搜索方向的导数已知。
     * 如果bracket设为true,则最小化器已被括在x和y之间的不确定性区间内。
     *
     * @param x       一个端点的值的指针
     * @param fx      f(x)的值的指针
     * @param dx      f'(x)(导数)的值的指针
     * @param y       另一个端点的值的指针
     * @param fy      f(y)的值的指针
     * @param dy      f'(y)(导数)的值的指针
     * @param t       试探值t的指针
     * @param ft      f(t)的值的指针
     * @param dt      f'(t)(导数)的值的指针
     * @param tmin    试探值t的最小值
     * @param tmax    试探值t的最大值
     * @param brackt  指示试探值是否被括住的谓词指针
     * @return        状态值。0表示正常终止
     *
     * @see Jorge J. More and David J. Thuente. "Line search algorithm with
     *      guaranteed sufficient decrease." ACM Transactions on Mathematical
     *      Software (TOMS), Vol 20, No 3, pp. 286-307, 1994.
     */
    inline int update_trial_interval(double *x,
                                     double *fx,
                                     double *dx,
                                     double *y,
                                     double *fy,
                                     double *dy,
                                     double *t,
                                     double *ft,
                                     double *dt,
                                     const double tmin,
                                     const double tmax,
                                     int *brackt)
    {
        int bound;
        int dsign = *dt * (*dx / fabs(*dx)) < 0.;  // 判断导数符号是否相反
        double mc;            // 三次插值函数的最小化器
        double mq;            // 二次插值函数的最小化器
        double newt;          // 新的试探值
        USES_MINIMIZER_LBFGS; // 声明用于三次和二次最小化器计算的局部变量

        /* 检查输入参数的错误 */
        if (*brackt)
        {
            if (*t <= (*x <= *y ? *x : *y) || (*x >= *y ? *x : *y) <= *t)
            {
                // 试探值t超出区间范围
                return LBFGSERR_OUTOFINTERVAL;
            }
            if (0. <= *dx * (*t - *x))
            {
                // 函数必须从x点下降
                return LBFGSERR_INCREASEGRADIENT;
            }
            if (tmax < tmin)
            {
                // tmin和tmax设置错误
                return LBFGSERR_INCORRECT_TMINMAX;
            }
        }

        /*
         * 试探值选择 - 根据函数值和导数的不同情况选择合适的插值方法
         */
        if (*fx < *ft)
        {
            /*
             * 情况1: 试探点函数值更高 (f(t) > f(x))
             * 最小值被括住。如果三次插值最小化器比二次插值更接近x,
             * 则选择三次插值结果,否则取两者的平均值。
             */
            *brackt = 1;
            bound = 1;
            CUBIC_MINIMIZER_LBFGS(mc, *x, *fx, *dx, *t, *ft, *dt);
            QUARD_MINIMIZER_LBFGS(mq, *x, *fx, *dx, *t, *ft);
            if (fabs(mc - *x) < fabs(mq - *x))
            {
                newt = mc;
            }
            else
            {
                newt = mc + 0.5 * (mq - mc);
            }
        }
        else if (dsign)
        {
            /*
             * 情况2: 函数值更低且导数符号相反 (f(t) <= f(x) 且 f'(t)*f'(x) < 0)
             * 最小值被括住。如果三次插值最小化器比二次插值(割线法)更接近x,
             * 则选择三次插值结果,否则选择二次插值结果。
             */
            *brackt = 1;
            bound = 0;
            CUBIC_MINIMIZER_LBFGS(mc, *x, *fx, *dx, *t, *ft, *dt);
            QUARD_MINIMIZER2_LBFGS(mq, *x, *dx, *t, *dt);
            if (fabs(mc - *t) > fabs(mq - *t))
            {
                newt = mc;
            }
            else
            {
                newt = mq;
            }
        }
        else if (fabs(*dt) < fabs(*dx))
        {
            /*
             * 情况3: 函数值更低,导数同号,且导数幅度减小
             * (f(t) <= f(x) 且 f'(t)*f'(x) > 0 且 |f'(t)| < |f'(x)|)
             * 只有当三次插值在最小化器方向趋于无穷,或三次插值的最小值超过t时,
             * 才使用三次插值最小化器。否则三次插值最小化器定义为tmin或tmax。
             * 同时计算二次插值(割线法)最小化器。如果最小值被括住,
             * 则选择距离x最近的最小化器;否则选择距离最远的。
             */
            bound = 1;
            CUBIC_MINIMIZER2_LBFGS(mc, *x, *fx, *dx, *t, *ft, *dt, tmin, tmax);
            QUARD_MINIMIZER2_LBFGS(mq, *x, *dx, *t, *dt);
            if (*brackt)
            {
                if (fabs(*t - mc) < fabs(*t - mq))
                {
                    newt = mc;
                }
                else
                {
                    newt = mq;
                }
            }
            else
            {
                if (fabs(*t - mc) > fabs(*t - mq))
                {
                    newt = mc;
                }
                else
                {
                    newt = mq;
                }
            }
        }
        else
        {
            /*
             * 情况4: 函数值更低,导数同号,且导数幅度不减小
             * (f(t) <= f(x) 且 f'(t)*f'(x) > 0 且 |f'(t)| >= |f'(x)|)
             * 如果最小值未被括住,步长取tmin或tmax;否则使用三次插值最小化器。
             */
            bound = 0;
            if (*brackt)
            {
                CUBIC_MINIMIZER_LBFGS(newt, *t, *ft, *dt, *y, *fy, *dy);
            }
            else if (*x < *t)
            {
                newt = tmax;
            }
            else
            {
                newt = tmin;
            }
        }

        /*
         * 更新不确定性区间。此更新不依赖于上述的新步长或情况分析。
         *
         * - 情况a: 如果 f(x) < f(t),
         *     x <- x, y <- t.
         * - 情况b: 如果 f(t) <= f(x) 且 f'(t)*f'(x) > 0,
         *     x <- t, y <- y.
         * - 情况c: 如果 f(t) <= f(x) 且 f'(t)*f'(x) < 0,
         *     x <- t, y <- x.
         */
        if (*fx < *ft)
        {
            /* 情况a */
            *y = *t;
            *fy = *ft;
            *dy = *dt;
        }
        else
        {
            /* 情况c */
            if (dsign)
            {
                *y = *x;
                *fy = *fx;
                *dy = *dx;
            }
            /* 情况b和c */
            *x = *t;
            *fx = *ft;
            *dx = *dt;
        }

        /* 将新试探值限制在 [tmin, tmax] 范围内 */
        if (tmax < newt)
            newt = tmax;
        if (newt < tmin)
            newt = tmin;

        /*
         * 如果新试探值接近区间上界,则重新定义
         */
        if (*brackt && bound)
        {
            mq = *x + 0.66 * (*y - *x);
            if (*x < *y)
            {
                if (mq < newt)
                    newt = mq;
            }
            else
            {
                if (newt < mq)
                    newt = mq;
            }
        }

        /* 返回新的试探值 */
        *t = newt;
        return 0;
    }

    /**
     * @brief More-Thuente线搜索算法
     *
     * 这是一种高效且数值稳定的线搜索方法,用于找到满足强Wolfe条件的步长。
     * 算法通过维护一个包含可接受步长的不确定性区间,并使用插值方法逐步缩小该区间。
     *
     * @param n        变量的数量
     * @param x        当前变量值,同时也是输出(更新后的变量值)
     * @param f        目标函数值的指针,同时也是输出(更新后的函数值)
     * @param g        梯度向量,同时也是输出(更新后的梯度)
     * @param s        搜索方向向量
     * @param stp      步长的指针,同时也是输出(最优步长)
     * @param xp       线搜索前的变量值
     * @param gp       线搜索前的梯度值
     * @param stpmin   最小允许步长
     * @param stpmax   最大允许步长
     * @param cd       回调数据结构
     * @param param    L-BFGS参数
     * @return         成功时返回评估次数,失败时返回负数错误码
     */
    inline int line_search_morethuente(int n,
                                       double *x,
                                       double *f,
                                       double *g,
                                       double *s,
                                       double *stp,
                                       const double *xp,
                                       const double *gp,
                                       const double *stpmin,
                                       const double *stpmax,
                                       callback_data_t *cd,
                                       const lbfgs_parameter_t *param)
    {
        int count = 0;                      // 函数评估计数器
        int brackt, stage1, uinfo = 0;      // 状态标志
        double dg;                          // 当前方向导数
        double stx, fx, dgx;                // 最佳步长点的步长、函数值、导数
        double sty, fy, dgy;                // 区间另一端点的步长、函数值、导数
        double fxm, dgxm, fym, dgym, fm, dgm;  // 修正函数的相关值
        double finit, ftest1, dginit, dgtest;  // 初始值和测试值
        double width, prev_width;           // 区间宽度和前一次的宽度
        double stmin, stmax;                // 当前搜索的步长范围

        /* 检查输入参数的错误 */
        if (*stp <= 0.)
        {
            return LBFGSERR_INVALIDPARAMETERS;
        }

        /* 计算搜索方向上的初始梯度(方向导数) g^T * s */
        vecdot(&dginit, g, s, n);

        /* 确保s是下降方向 (方向导数应该为负) */
        if (0 < dginit)
        {
            return LBFGSERR_INCREASEGRADIENT;
        }

        /* 初始化局部变量 */
        brackt = 0;                         // 最小值是否被括住的标志
        stage1 = 1;                         // 第一阶段标志
        finit = *f;                         // 初始函数值
        dgtest = param->f_dec_coeff * dginit;  // Armijo条件的测试值
        width = *stpmax - *stpmin;          // 初始区间宽度
        prev_width = 2.0 * width;           // 前一次区间宽度

        /*
         * 变量含义:
         * stx, fx, dgx: 最佳步长点的步长、函数值和方向导数
         * sty, fy, dgy: 不确定性区间另一端点的步长、函数值和导数
         * stp, f, dg: 当前步长点的步长、函数值和导数
         */
        stx = sty = 0.;
        fx = fy = finit;
        dgx = dgy = dginit;

        for (;;)
        {
            /* 报告进度(如果有进度回调函数) */
            if (cd->proc_progress)
            {
                double xnorm;
                double gnorm;
                vec2norm(&xnorm, x, n);
                vec2norm(&gnorm, g, n);
                if (cd->proc_progress(cd->instance, x, g, fx, xnorm, gnorm, *stp, cd->n, 0, 0))
                {
                    return LBFGSERR_CANCELED;
                }
            }

            /*
             * 设置最小和最大步长以对应当前的不确定性区间
             */
            if (brackt)
            {
                stmin = stx <= sty ? stx : sty;
                stmax = stx >= sty ? stx : sty;
            }
            else
            {
                stmin = stx;
                stmax = *stp + 4.0 * (*stp - stx);  // 如果未括住,扩展搜索范围
            }

            /* 将步长限制在 [stpmin, stpmax] 范围内 */
            if (*stp < *stpmin)
                *stp = *stpmin;
            if (*stpmax < *stp)
                *stp = *stpmax;

            /*
             * 如果即将发生异常终止,则让步长取目前获得的最低点
             */
            if ((brackt && ((*stp <= stmin || stmax <= *stp) || param->max_linesearch <= count + 1 || uinfo != 0)) || (brackt && (stmax - stmin <= param->xtol * stmax)))
            {
                *stp = stx;
            }

            /*
             * 计算当前x的值: x <- xp + (*stp) * s
             */
            veccpy(x, xp, n);
            vecadd(x, s, *stp, n);

            /* 评估函数和梯度值 */
            *f = cd->proc_evaluate(cd->instance, x, g, cd->n);
            vecdot(&dg, g, s, n);  // 计算方向导数

            ftest1 = finit + *stp * dgtest;  // Armijo条件右侧
            ++count;

            /* 测试错误和收敛条件 */
            if (brackt && ((*stp <= stmin || stmax <= *stp) || uinfo != 0))
            {
                /* 舍入误差阻止进一步进展 */
                return LBFGSERR_ROUNDING_ERROR;
            }
            if (*stp == *stpmax && *f <= ftest1 && dg <= dgtest)
            {
                /* 步长达到最大值 */
                return LBFGSERR_MAXIMUMSTEP;
            }
            if (*stp == *stpmin && (ftest1 < *f || dgtest <= dg))
            {
                /* 步长达到最小值 */
                return LBFGSERR_MINIMUMSTEP;
            }
            if (brackt && (stmax - stmin) <= param->xtol * stmax)
            {
                /* 不确定性区间的相对宽度最多为xtol */
                return LBFGSERR_WIDTHTOOSMALL;
            }
            if (param->max_linesearch <= count)
            {
                /* 达到最大迭代次数 */
                return LBFGSERR_MAXIMUMLINESEARCH;
            }
            if (*f <= ftest1 && fabs(dg) <= param->s_curv_coeff * (-dginit))
            {
                /* 同时满足充分下降条件和强曲率条件(强Wolfe条件) */
                return count;
            }

            /*
             * 在第一阶段,我们寻找一个步长,使得修正函数
             * 具有非正值和非负导数
             */
            if (stage1 && *f <= ftest1 &&
                (param->f_dec_coeff <= param->s_curv_coeff ? param->f_dec_coeff : param->s_curv_coeff) * dginit <= dg)
            {
                stage1 = 0;  // 退出第一阶段
            }

            /*
             * 只有在以下情况下才使用修正函数来预测步长:
             * 1. 尚未获得使修正函数具有非正函数值和非负导数的步长
             * 2. 已经获得了较低的函数值,但下降幅度不足
             */
            if (stage1 && ftest1 < *f && *f <= fx)
            {
                /* 定义修正函数和导数值 */
                fm = *f - *stp * dgtest;
                fxm = fx - stx * dgtest;
                fym = fy - sty * dgtest;
                dgm = dg - dgtest;
                dgxm = dgx - dgtest;
                dgym = dgy - dgtest;

                /*
                 * 调用 update_trial_interval() 来更新不确定性区间
                 * 并计算新的步长
                 */
                uinfo = update_trial_interval(
                    &stx, &fxm, &dgxm,
                    &sty, &fym, &dgym,
                    stp, &fm, &dgm,
                    stmin, stmax, &brackt);

                /* 重置f的函数值和梯度值 */
                fx = fxm + stx * dgtest;
                fy = fym + sty * dgtest;
                dgx = dgxm + dgtest;
                dgy = dgym + dgtest;
            }
            else
            {
                /*
                 * 调用 update_trial_interval() 来更新不确定性区间
                 * 并计算新的步长
                 */
                uinfo = update_trial_interval(
                    &stx, &fx, &dgx,
                    &sty, &fy, &dgy,
                    stp, f, &dg,
                    stmin, stmax, &brackt);
            }

            /*
             * 强制不确定性区间充分缩小
             * 如果区间缩小不够快,取区间中点
             */
            if (brackt)
            {
                if (0.66 * prev_width <= fabs(sty - stx))
                {
                    *stp = stx + 0.5 * (sty - stx);
                }
                prev_width = width;
                width = fabs(sty - stx);
            }
        }

        return LBFGSERR_LOGICERROR;  // 理论上不应该到达这里
    }

    /**
     * @brief L-BFGS默认参数
     *
     * 参数顺序: mem_size, g_epsilon, past, delta, max_iterations,
     *          max_linesearch, min_step, max_step, f_dec_coeff,
     *          s_curv_coeff, xtol
     */
    static const lbfgs_parameter_t _default_param = {
        8,          // mem_size: 历史记忆大小
        1e-5,       // g_epsilon: 梯度收敛阈值
        0,          // past: 不启用基于delta的收敛测试
        1e-5,       // delta: 收敛率阈值
        0,          // max_iterations: 无迭代次数限制
        40,         // max_linesearch: 最大线搜索次数
        1e-20,      // min_step: 最小步长
        1e20,       // max_step: 最大步长
        1e-4,       // f_dec_coeff: Armijo条件系数
        0.9,        // s_curv_coeff: 强Wolfe条件曲率系数
        1.0e-16,    // xtol: 机器精度
    };

    /**
     * @brief 初始化L-BFGS参数为默认值
     *
     * 调用此函数将参数结构体填充为默认值,
     * 之后可以根据需要覆盖特定参数的值。
     *
     * @param param  参数结构体的指针
     */
    inline void lbfgs_load_default_parameters(lbfgs_parameter_t *param)
    {
        memcpy(param, &_default_param, sizeof(*param));
    }

    /**
     * @brief 启动L-BFGS优化过程
     *
     * 用户必须实现一个兼容 lbfgs_evaluate_t 的回调函数(用于评估目标函数和梯度),
     * 并将该函数指针传递给 lbfgs_optimize()。
     * 类似地,用户可以实现 lbfgs_stepbound_t 函数来提供步长的外部上界,
     * 以及 lbfgs_progress_t 函数来获取当前进度(如变量值、函数值、||G||等)
     * 并在必要时取消迭代过程。步长边界和进度回调的实现是可选的:
     * 如果不需要进度通知,用户可以传递NULL。
     *
     * 算法在以下情况下终止优化:
     *   ||G|| < g_epsilon * max(1, ||x||)
     *
     * 其中 ||.|| 表示欧几里得范数。
     *
     * @param n               变量的数量
     * @param x               变量数组。客户程序可以设置优化的初始值,
     *                        并通过此数组接收优化结果。
     * @param ptr_fx          指向接收最终目标函数值的变量的指针。
     *                        如果不需要最终目标函数值,可以设置为NULL。
     * @param proc_evaluate   回调函数,用于在给定当前变量值时提供
     *                        函数和梯度评估。客户程序必须实现一个
     *                        兼容 lbfgs_evaluate_t 的回调函数。
     * @param proc_stepbound  回调函数,用于提供步长搜索的上界。
     *                        输入包括线搜索前的变量初始值和当前步向量
     *                        (可以是负梯度)。客户程序可以实现此函数
     *                        以实现更高效的线搜索。如果不使用,设置为NULL。
     * @param proc_progress   回调函数,用于接收优化过程的进度
     *                        (迭代次数、目标函数当前值)。
     *                        如果不需要进度报告,可以设置为NULL。
     * @param instance        客户程序的用户数据。回调函数将接收此参数的值。
     * @param param           指向L-BFGS优化参数结构的指针。
     *                        客户程序可以将此参数设置为NULL以使用默认参数。
     *                        调用 lbfgs_load_default_parameters() 函数
     *                        可以用默认值填充结构。
     * @return                状态码。如果优化过程无错误终止,则返回零。
     *                        非零值表示错误。
     */
    inline int lbfgs_optimize(int n,
                              double *x,
                              double *ptr_fx,
                              lbfgs_evaluate_t proc_evaluate,
                              lbfgs_stepbound_t proc_stepbound,
                              lbfgs_progress_t proc_progress,
                              void *instance,
                              lbfgs_parameter_t *_param)
    {
        int ret;                            // 返回状态码
        int i, j, k, ls, end, bound;        // 循环和索引变量
        double step;                        // 当前步长
        int loop;                           // 循环标志
        double step_min, step_max;          // 步长的最小值和最大值

        /* 常量参数及其默认值 */
        lbfgs_parameter_t param = (_param != NULL) ? (*_param) : _default_param;
        const int m = param.mem_size;       // 历史记忆大小

        double *xp = NULL;                  // 前一次迭代的变量值
        double *g = NULL, *gp = NULL;       // 当前和前一次的梯度
        double *d = NULL, *pf = NULL;       // 搜索方向和历史函数值
        iteration_data_t *lm = NULL, *it = NULL;  // 有限记忆存储和迭代器
        double ys, yy;                      // y^T*s 和 y^T*y
        double xnorm, gnorm, beta;          // 变量范数、梯度范数、beta系数
        double fx = 0.;                     // 当前目标函数值
        double rate = 0.;                   // 函数下降率

        /* 构造回调数据结构 */
        callback_data_t cd;
        cd.n = n;
        cd.instance = instance;
        cd.proc_evaluate = proc_evaluate;
        cd.proc_stepbound = proc_stepbound;
        cd.proc_progress = proc_progress;

        /* 检查输入参数的错误 */
        if (n <= 0)
        {
            return LBFGSERR_INVALID_N;
        }
        if (m <= 0)
        {
            return LBFGSERR_INVALID_MEMSIZE;
        }
        if (param.g_epsilon < 0.)
        {
            return LBFGSERR_INVALID_GEPSILON;
        }
        if (param.past < 0)
        {
            return LBFGSERR_INVALID_TESTPERIOD;
        }
        if (param.delta < 0.)
        {
            return LBFGSERR_INVALID_DELTA;
        }
        if (param.min_step < 0.)
        {
            return LBFGSERR_INVALID_MINSTEP;
        }
        if (param.max_step < param.min_step)
        {
            return LBFGSERR_INVALID_MAXSTEP;
        }
        if (param.f_dec_coeff < 0.)
        {
            return LBFGSERR_INVALID_FDECCOEFF;
        }
        if (param.s_curv_coeff <= param.f_dec_coeff || 1. <= param.s_curv_coeff)
        {
            return LBFGSERR_INVALID_SCURVCOEFF;
        }
        if (param.xtol < 0.)
        {
            return LBFGSERR_INVALID_XTOL;
        }
        if (param.max_linesearch <= 0)
        {
            return LBFGSERR_INVALID_MAXLINESEARCH;
        }

        /* 分配工作空间 */
        xp = (double *)vecalloc(n * sizeof(double));
        g = (double *)vecalloc(n * sizeof(double));
        gp = (double *)vecalloc(n * sizeof(double));
        d = (double *)vecalloc(n * sizeof(double));

        /* 分配有限记忆存储空间 */
        lm = (iteration_data_t *)vecalloc(m * sizeof(iteration_data_t));

        /* 初始化有限记忆 */
        for (i = 0; i < m; ++i)
        {
            it = &lm[i];
            it->alpha = 0;
            it->ys = 0;
            it->s = (double *)vecalloc(n * sizeof(double));
            it->y = (double *)vecalloc(n * sizeof(double));
        }

        /* 分配用于存储历史目标函数值的数组 */
        if (0 < param.past)
        {
            pf = (double *)vecalloc(param.past * sizeof(double));
        }

        /* 评估初始函数值和梯度 */
        fx = cd.proc_evaluate(cd.instance, x, g, cd.n);

        /* 存储初始目标函数值 */
        if (pf != NULL)
        {
            pf[0] = fx;
        }

        /*
         * 计算初始搜索方向;
         * 假设初始Hessian矩阵 H_0 为单位矩阵,因此初始方向为负梯度
         */
        vecncpy(d, g, n);

        /*
         * 确保初始变量不是最小化器
         */
        vec2norm(&xnorm, x, n);
        vec2norm(&gnorm, g, n);

        if (xnorm < 1.0)
            xnorm = 1.0;
        if (gnorm / xnorm <= param.g_epsilon)
        {
            ret = LBFGS_ALREADY_MINIMIZED;
        }
        else
        {
            /* 计算初始步长: step = 1.0 / ||d||_2 */
            vec2norminv(&step, d, n);

            k = 1;      // 迭代计数器
            end = 0;    // 有限记忆循环索引
            loop = 1;   // 主循环标志

            while (loop == 1)
            {
                /* 保存当前位置和梯度向量 */
                veccpy(xp, x, n);
                veccpy(gp, g, n);

                // 如果可以动态提供步长界限,则应用它
                step_min = param.min_step;
                step_max = param.max_step;
                if (cd.proc_stepbound)
                {
                    step_max = cd.proc_stepbound(cd.instance, xp, d, cd.n);
                    step_max = step_max < param.max_step ? step_max : param.max_step;
                    if (step >= step_max)
                        step = step_max / 2.0;
                }

                /* 搜索最优步长 */
                ls = line_search_morethuente(n, x, &fx, g, d, &step, xp, gp, &step_min, &step_max, &cd, &param);

                if (ls < 0)
                {
                    /* 线搜索失败,恢复到前一个点 */
                    veccpy(x, xp, n);
                    veccpy(g, gp, n);
                    ret = ls;
                    loop = 0;
                    continue;
                }

                /* 计算x和g的范数 */
                vec2norm(&xnorm, x, n);
                vec2norm(&gnorm, g, n);

                // /* 报告进度 (已注释) */
                // if (cd.proc_progress)
                // {
                //     if ((ret = cd.proc_progress(cd.instance, x, g, fx, xnorm, gnorm, step, cd.n, k, ls)))
                //     {
                //         loop = 0;
                //         continue;
                //     }
                // }

                /*
                 * 收敛性测试
                 * 准则: |g(x)| / max(1, |x|) < g_epsilon
                 */
                if (xnorm < 1.0)
                    xnorm = 1.0;
                if (gnorm / xnorm <= param.g_epsilon)
                {
                    /* 达到收敛 */
                    ret = LBFGS_CONVERGENCE;
                    break;
                }

                /*
                 * 停止准则测试
                 * 准则: |(f(past_x) - f(x))| / f(x) < delta
                 */
                if (pf != NULL)
                {
                    /* 当 k < past 时不测试停止准则 */
                    if (param.past <= k)
                    {
                        /* 计算相对于过去的改进 */
                        rate = (pf[k % param.past] - fx) / fx;

                        /* 停止准则 */
                        if (fabs(rate) < param.delta)
                        {
                            ret = LBFGS_STOP;
                            break;
                        }
                    }

                    /* 存储当前目标函数值 */
                    pf[k % param.past] = fx;
                }

                if (param.max_iterations != 0 && param.max_iterations < k + 1)
                {
                    /* 达到最大迭代次数 */
                    ret = LBFGSERR_MAXIMUMITERATION;
                    break;
                }

                /*
                 * 更新向量 s 和 y:
                 * s_{k+1} = x_{k+1} - x_{k} = step * d_{k}  (位置差)
                 * y_{k+1} = g_{k+1} - g_{k}                 (梯度差)
                 */
                it = &lm[end];
                vecdiff(it->s, x, xp, n);
                vecdiff(it->y, g, gp, n);

                /*
                 * 计算标量 ys 和 yy:
                 * ys = y^T * s = 1 / rho  (用于BFGS更新公式)
                 * yy = y^T * y            (用于缩放Hessian矩阵 H_0)
                 * 注意: yy 用于缩放Hessian矩阵 H_0 (Cholesky因子)
                 */
                vecdot(&ys, it->y, it->s, n);
                vecdot(&yy, it->y, it->y, n);
                it->ys = ys;

                /*
                 * 递归公式计算 dir = -(H * g),即搜索方向
                 * 这是L-BFGS两循环递归算法的核心
                 * 参考文献:
                 * Jorge Nocedal. "Updating Quasi-Newton Matrices with Limited Storage."
                 * Mathematics of Computation, Vol. 35, No. 151, pp. 773--782, 1980.
                 */
                bound = (m <= k) ? m : k;  // 实际使用的历史记忆数量
                ++k;
                end = (end + 1) % m;       // 循环更新索引

                /* 计算负梯度 (初始化为 q = -g) */
                vecncpy(d, g, n);

                /* 第一个循环: 从最近到最远,计算 alpha 并更新 q */
                j = end;
                for (i = 0; i < bound; ++i)
                {
                    j = (j + m - 1) % m; /* 等价于 if (--j == -1) j = m-1; */
                    it = &lm[j];
                    /* alpha_j = rho_j * s_j^T * q_{i+1} */
                    vecdot(&it->alpha, it->s, d, n);
                    it->alpha /= it->ys;
                    /* q_i = q_{i+1} - alpha_i * y_i */
                    vecadd(d, it->y, -it->alpha, n);
                }

                /* 使用初始Hessian近似 H_0 = (ys/yy) * I 进行缩放 */
                vecscale(d, ys / yy, n);

                /* 第二个循环: 从最远到最近,计算 beta 并更新 r */
                for (i = 0; i < bound; ++i)
                {
                    it = &lm[j];
                    /* beta_j = rho_j * y_j^T * r_i */
                    vecdot(&beta, it->y, d, n);
                    beta /= it->ys;
                    /* r_{i+1} = r_i + (alpha_j - beta_j) * s_j */
                    vecadd(d, it->s, it->alpha - beta, n);
                    j = (j + 1) % m; /* 等价于 if (++j == m) j = 0; */
                }

                /*
                 * 现在搜索方向 d 已经准备好
                 * 我们首先尝试 step = 1.0
                 */
                step = 1.0;
            }
        }

        /* 返回目标函数的最终值 */
        if (ptr_fx != NULL)
        {
            *ptr_fx = fx;
        }

        vecfree(pf);

        /* 释放此函数使用的内存块 */
        if (lm != NULL)
        {
            for (i = 0; i < m; ++i)
            {
                vecfree(lm[i].s);
                vecfree(lm[i].y);
            }
            vecfree(lm);
        }
        vecfree(d);
        vecfree(gp);
        vecfree(g);
        vecfree(xp);

        return ret;
    }

    /**
     * @brief 获取 lbfgs_optimize() 返回码的字符串描述
     *
     * 此函数将 lbfgs_optimize() 的返回码转换为可读的错误或状态描述字符串。
     *
     * @param err  lbfgs_optimize() 返回的状态码
     * @return     状态码对应的描述字符串
     */
    inline const char *lbfgs_strerror(int err)
    {
        switch (err)
        {
        case LBFGS_CONVERGENCE:
            return "Success: reached convergence (g_epsilon).";

        case LBFGS_STOP:
            return "Success: met stopping criteria (past f decrease less than delta).";

        case LBFGS_ALREADY_MINIMIZED:
            return "The initial variables already minimize the objective function.";

        case LBFGSERR_UNKNOWNERROR:
            return "Unknown error.";

        case LBFGSERR_LOGICERROR:
            return "Logic error.";

        case LBFGSERR_CANCELED:
            return "The minimization process has been canceled.";

        case LBFGSERR_INVALID_N:
            return "Invalid number of variables specified.";

        case LBFGSERR_INVALID_MEMSIZE:
            return "Invalid parameter lbfgs_parameter_t::mem_size specified.";

        case LBFGSERR_INVALID_GEPSILON:
            return "Invalid parameter lbfgs_parameter_t::g_epsilon specified.";

        case LBFGSERR_INVALID_TESTPERIOD:
            return "Invalid parameter lbfgs_parameter_t::past specified.";

        case LBFGSERR_INVALID_DELTA:
            return "Invalid parameter lbfgs_parameter_t::delta specified.";

        case LBFGSERR_INVALID_MINSTEP:
            return "Invalid parameter lbfgs_parameter_t::min_step specified.";

        case LBFGSERR_INVALID_MAXSTEP:
            return "Invalid parameter lbfgs_parameter_t::max_step specified.";

        case LBFGSERR_INVALID_FDECCOEFF:
            return "Invalid parameter lbfgs_parameter_t::f_dec_coeff specified.";

        case LBFGSERR_INVALID_SCURVCOEFF:
            return "Invalid parameter lbfgs_parameter_t::s_curv_coeff specified.";

        case LBFGSERR_INVALID_XTOL:
            return "Invalid parameter lbfgs_parameter_t::xtol specified.";

        case LBFGSERR_INVALID_MAXLINESEARCH:
            return "Invalid parameter lbfgs_parameter_t::max_linesearch specified.";

        case LBFGSERR_OUTOFINTERVAL:
            return "The line-search step went out of the interval of uncertainty.";

        case LBFGSERR_INCORRECT_TMINMAX:
            return "A logic error occurred; alternatively, the interval of uncertainty"
                   " became too small.";

        case LBFGSERR_ROUNDING_ERROR:
            return "A rounding error occurred; alternatively, no line-search step"
                   " satisfies the sufficient decrease and curvature conditions.";

        case LBFGSERR_MINIMUMSTEP:
            return "The line-search step became smaller than lbfgs_parameter_t::min_step.";

        case LBFGSERR_MAXIMUMSTEP:
            return "The line-search step became larger than lbfgs_parameter_t::max_step.";

        case LBFGSERR_MAXIMUMLINESEARCH:
            return "The line-search routine reaches the maximum number of evaluations.";

        case LBFGSERR_MAXIMUMITERATION:
            return "The algorithm routine reaches the maximum number of iterations.";

        case LBFGSERR_WIDTHTOOSMALL:
            return "Relative width of the interval of uncertainty is at most"
                   " lbfgs_parameter_t::xtol.";

        case LBFGSERR_INVALIDPARAMETERS:
            return "A logic error (negative line-search step) occurred.";

        case LBFGSERR_INCREASEGRADIENT:
            return "The current search direction increases the objective function value.";

        default:
            return "(unknown)";
        }
    }

} // namespace lbfgs

#endif