/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

// SFC_GEN_HPP - 安全飞行走廊(Safe Flight Corridor)生成工具
// 该文件提供了基于OMPL的路径规划和凸分解功能，用于生成无碰撞的飞行走廊
#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

// 引入几何工具库，提供几何计算功能
#include "geo_utils.hpp"
// 引入FIRI算法库，用于快速无碰撞区域识别
#include "firi.hpp"

// OMPL相关头文件 - 开源运动规划库
#include <ompl/util/Console.h>                                      // OMPL控制台输出工具
#include <ompl/base/SpaceInformation.h>                             // 空间信息基类
#include <ompl/base/spaces/RealVectorStateSpace.h>                  // 实数向量状态空间
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>            // Informed RRT*规划器
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>   // 路径长度优化目标
#include <ompl/base/DiscreteMotionValidator.h>                      // 离散运动验证器

// 标准库
#include <deque>       // 双端队列，用于高效的前后插入删除操作
#include <memory>      // 智能指针
#include <Eigen/Eigen> // Eigen线性代数库

// sfc_gen命名空间 - 安全飞行走廊生成相关功能
namespace sfc_gen
{

    /**
     * @brief 使用Informed RRT*算法规划从起点到终点的路径
     * @tparam Map 地图类型模板，需要提供query()方法用于碰撞检测
     * @param s 起点坐标(x, y, z)
     * @param g 终点坐标(x, y, z)
     * @param lb 搜索空间的下界(lower bound)
     * @param hb 搜索空间的上界(higher bound)
     * @param mapPtr 地图指针，用于碰撞检测
     * @param timeout 规划超时时间(秒)
     * @param p 输出参数，存储规划得到的路径点序列
     * @return 路径代价(长度)，如果规划失败返回INFINITY
     */
    template <typename Map>
    inline double planPath(const Eigen::Vector3d &s,
                           const Eigen::Vector3d &g,
                           const Eigen::Vector3d &lb,
                           const Eigen::Vector3d &hb,
                           const Map *mapPtr,
                           const double &timeout,
                           std::vector<Eigen::Vector3d> &p)
    {
        // 创建3维实数向量状态空间(用于表示3D位置)
        auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));

        // 设置状态空间的边界，将世界坐标系转换为以lb为原点的局部坐标系
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(0, 0.0);                // X轴下界设为0
        bounds.setHigh(0, hb(0) - lb(0));     // X轴上界为世界坐标系的范围
        bounds.setLow(1, 0.0);                // Y轴下界设为0
        bounds.setHigh(1, hb(1) - lb(1));     // Y轴上界为世界坐标系的范围
        bounds.setLow(2, 0.0);                // Z轴下界设为0
        bounds.setHigh(2, hb(2) - lb(2));     // Z轴上界为世界坐标系的范围
        space->setBounds(bounds);

        // 创建空间信息对象，包含状态空间和状态有效性检查器
        auto si(std::make_shared<ompl::base::SpaceInformation>(space));

        // 设置状态有效性检查器(用Lambda函数)，检查状态是否无碰撞
        si->setStateValidityChecker(
            [&](const ompl::base::State *state)
            {
                // 将OMPL状态转换为实数向量
                const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
                // 将局部坐标转换回世界坐标系
                const Eigen::Vector3d position(lb(0) + (*pos)[0],
                                               lb(1) + (*pos)[1],
                                               lb(2) + (*pos)[2]);
                // 查询地图，返回值为0表示无碰撞(有效状态)
                return mapPtr->query(position) == 0;
            });
        si->setup();  // 完成空间信息的设置

        // 关闭OMPL的日志输出，避免控制台信息干扰
        ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

        // 创建起点和终点状态，并将世界坐标转换为局部坐标
        ompl::base::ScopedState<> start(space), goal(space);
        start[0] = s(0) - lb(0);  // 起点X坐标
        start[1] = s(1) - lb(1);  // 起点Y坐标
        start[2] = s(2) - lb(2);  // 起点Z坐标
        goal[0] = g(0) - lb(0);   // 终点X坐标
        goal[1] = g(1) - lb(1);   // 终点Y坐标
        goal[2] = g(2) - lb(2);   // 终点Z坐标

        // 创建问题定义对象
        auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal);  // 设置起点和终点
        // 设置优化目标为路径长度最小化
        pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
        // 创建Informed RRT*规划器，该算法在RRT*基础上利用启发式信息加速搜索
        auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));
        planner->setProblemDefinition(pdef);  // 设置规划问题
        planner->setup();                     // 初始化规划器

        // 执行路径规划，设置超时时间
        ompl::base::PlannerStatus solved;
        solved = planner->ompl::base::Planner::solve(timeout);

        // 初始化路径代价为无穷大(表示未找到路径)
        double cost = INFINITY;
        if (solved)  // 如果成功找到路径
        {
            p.clear();  // 清空输出路径容器
            // 获取几何路径对象
            const ompl::geometric::PathGeometric path_ =
                ompl::geometric::PathGeometric(
                    dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
            // 遍历路径中的所有状态点
            for (size_t i = 0; i < path_.getStateCount(); i++)
            {
                // 获取状态的坐标值
                const auto state = path_.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                // 将局部坐标转换回世界坐标并添加到输出路径中
                p.emplace_back(lb(0) + state[0], lb(1) + state[1], lb(2) + state[2]);
            }
            // 计算路径的总代价(长度)
            cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
        }

        return cost;  // 返回路径代价
    }

    /**
     * @brief 为给定路径生成凸覆盖(凸多面体序列)，构建安全飞行走廊
     * @param path 待覆盖的路径点序列
     * @param points 障碍物点云，用于约束凸多面体的生成
     * @param lowCorner 地图空间的下角坐标
     * @param highCorner 地图空间的上角坐标
     * @param progress 路径分段的进度步长(米)，控制凸多面体的密集程度
     * @param range 每个凸多面体在各轴方向的扩展范围(米)
     * @param hpolys 输出参数，存储生成的凸多面体序列，每个多面体用半平面表示(H-representation)
     * @param eps 数值精度阈值，用于判断点是否在多面体边界上
     */
    inline void convexCover(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const double &progress,
                            const double &range,
                            std::vector<Eigen::MatrixX4d> &hpolys,
                            const double eps = 1.0e-6)
    {
        hpolys.clear();  // 清空输出容器
        const int n = path.size();  // 路径点数量

        // 初始化边界约束矩阵(6个半平面表示3D空间的立方体边界)
        // 每行表示一个半平面约束: [nx, ny, nz, d]，满足 n·p + d <= 0
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;   // +X方向的边界平面: x + d <= 0
        bd(1, 0) = -1.0;  // -X方向的边界平面: -x + d <= 0
        bd(2, 1) = 1.0;   // +Y方向的边界平面: y + d <= 0
        bd(3, 1) = -1.0;  // -Y方向的边界平面: -y + d <= 0
        bd(4, 2) = 1.0;   // +Z方向的边界平面: z + d <= 0
        bd(5, 2) = -1.0;  // -Z方向的边界平面: -z + d <= 0

        Eigen::MatrixX4d hp, gap;  // hp: 当前凸多面体, gap: 间隙多面体
        Eigen::Vector3d a, b = path[0];  // a: 当前段起点, b: 当前段终点
        std::vector<Eigen::Vector3d> valid_pc;  // 当前边界框内的有效点云
        std::vector<Eigen::Vector3d> bs;        // 存储所有段的终点
        valid_pc.reserve(points.size());  // 预分配内存以提高效率

        // 沿路径按progress步长生成凸多面体序列
        for (int i = 1; i < n;)
        {
            a = b;  // 更新段起点

            // 如果当前点到下一个路径点的距离大于progress，则插值一个中间点
            if ((a - path[i]).norm() > progress)
            {
                b = (path[i] - a).normalized() * progress + a;  // 沿方向前进progress距离
            }
            else  // 否则直接使用路径点
            {
                b = path[i];
                i++;  // 移动到下一个路径点
            }
            bs.emplace_back(b);  // 记录段终点

            // 计算当前段的边界框，在路径段周围扩展range距离，并限制在地图范围内
            // 对于每个轴，找到a和b中较大/较小的坐标，扩展range后限制在地图边界内
            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));  // X轴正向边界
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));   // X轴负向边界
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));  // Y轴正向边界
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));   // Y轴负向边界
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));  // Z轴正向边界
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));   // Z轴负向边界

            // 筛选出在当前边界框内的障碍物点
            valid_pc.clear();
            for (const Eigen::Vector3d &p : points)
            {
                // 检查点是否满足所有6个边界约束(所有半平面约束的最大值<0表示点在内部)
                if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0)
                {
                    valid_pc.emplace_back(p);  // 点在边界框内，加入有效点云
                }
            }
            // 将有效点云转换为Eigen矩阵格式(3×N矩阵，每列是一个点)
            Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());

            // 使用FIRI算法生成包含线段ab且避开障碍物的凸多面体
            firi::firi(bd, pc, a, b, hp);

            // 检查当前多面体与前一个多面体是否有足够的重叠
            if (hpolys.size() != 0)
            {
                const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);  // 齐次坐标形式的点a
                // 统计点a在当前多面体和前一个多面体边界上或外部的半平面数量
                // 如果总数>=3，说明两个多面体重叠不足，需要插入间隙多面体
                if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                             ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
                {
                    // 生成一个以点a为中心的小间隙多面体，确保走廊连续性
                    firi::firi(bd, pc, a, a, gap, 1);
                    hpolys.emplace_back(gap);  // 添加间隙多面体
                }
            }

            hpolys.emplace_back(hp);  // 添加当前生成的凸多面体
        }
    }

    /**
     * @brief 对凸多面体序列进行短切(shortcut)优化，移除冗余的中间多面体
     *
     * 该函数通过寻找可以直接连接的非相邻多面体来简化走廊，减少多面体数量
     * 同时保持路径的连通性和安全性
     *
     * @param hpolys 输入输出参数，凸多面体序列，函数会就地修改该序列
     */
    inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys)
    {
        std::vector<Eigen::MatrixX4d> htemp = hpolys;  // 复制原始多面体序列

        // 特殊处理：如果只有一个多面体，复制一份以确保后续算法正常工作
        if (htemp.size() == 1)
        {
            Eigen::MatrixX4d headPoly = htemp.front();
            htemp.insert(htemp.begin(), headPoly);  // 在开头插入副本
        }
        hpolys.clear();  // 清空输出序列，准备重新填充优化后的结果

        int M = htemp.size();  // 原始多面体数量
        Eigen::MatrixX4d hPoly;
        bool overlap;  // 标记两个多面体是否重叠
        std::deque<int> idices;  // 存储优化后保留的多面体索引

        // 从后向前贪心搜索最优短切路径
        idices.push_front(M - 1);  // 首先加入最后一个多面体(目标端)
        for (int i = M - 1; i >= 0; i--)
        {
            // 对于当前多面体i，尝试找到最远的可直接连接的多面体j
            for (int j = 0; j < i; j++)
            {
                // 如果j和i不相邻，需要检查它们是否重叠(可直接连接)
                if (j < i - 1)
                {
                    overlap = geo_utils::overlap(htemp[i], htemp[j], 0.01);
                }
                else  // 相邻多面体默认重叠
                {
                    overlap = true;
                }

                // 如果找到可连接的多面体
                if (overlap)
                {
                    idices.push_front(j);  // 将其索引加入序列
                    i = j + 1;  // 跳过中间的多面体，从j继续向前搜索
                    break;
                }
            }
        }

        // 按优化后的索引顺序重建多面体序列
        for (const auto &ele : idices)
        {
            hpolys.push_back(htemp[ele]);
        }
    }

}  // namespace sfc_gen

#endif  // SFC_GEN_HPP
