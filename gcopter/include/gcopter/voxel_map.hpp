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

#ifndef VOXEL_MAP_HPP
#define VOXEL_MAP_HPP

#include "voxel_dilater.hpp"
#include <memory>
#include <vector>
#include <Eigen/Eigen>

// 体素地图命名空间
namespace voxel_map
{
    // 体素状态常量定义
    constexpr uint8_t Unoccupied = 0;  // 未占用状态：表示该体素为空闲空间
    constexpr uint8_t Occupied = 1;    // 占用状态：表示该体素被障碍物占据
    constexpr uint8_t Dilated = 2;     // 膨胀状态：表示该体素为障碍物膨胀后的安全边界

    /**
     * @brief 体素地图类
     *
     * 该类实现了一个三维体素网格地图，用于环境的离散化表示。
     * 主要功能包括：
     * - 存储障碍物占用信息
     * - 障碍物膨胀处理（用于安全距离保证）
     * - 碰撞查询
     * - 表面点提取
     */
    class VoxelMap
    {

    public:
        // 默认构造函数
        VoxelMap() = default;

        /**
         * @brief 带参数的构造函数
         * @param size 地图的体素尺寸（x, y, z方向的体素数量）
         * @param origin 地图的原点坐标（世界坐标系）
         * @param voxScale 单个体素的边长（米）
         *
         * 初始化列表说明：
         * - mapSize: 地图体素尺寸
         * - o: 地图原点
         * - scale: 体素尺寸
         * - voxNum: 总体素数量（x*y*z）
         * - step: 坐标索引步长（x方向为1，y方向为mapSize(0)，z方向为mapSize(0)*mapSize(1)）
         * - oc: 体素中心偏移后的原点（加上半个体素边长，使得索引对应体素中心）
         * - bounds: 边界索引（每个维度的最大有效索引）
         * - stepScale: 索引到物理坐标的转换系数
         * - voxels: 体素数组，初始化为全部未占用
         */
        VoxelMap(const Eigen::Vector3i &size,
                 const Eigen::Vector3d &origin,
                 const double &voxScale)
            : mapSize(size),
              o(origin),
              scale(voxScale),
              voxNum(mapSize.prod()),
              step(1, mapSize(0), mapSize(1) * mapSize(0)),
              oc(o + Eigen::Vector3d::Constant(0.5 * scale)),
              bounds((mapSize.array() - 1) * step.array()),
              stepScale(step.cast<double>().cwiseInverse() * scale),
              voxels(voxNum, Unoccupied) {}

    private:
        Eigen::Vector3i mapSize;            // 地图的体素尺寸 (nx, ny, nz)
        Eigen::Vector3d o;                  // 地图原点的世界坐标
        double scale;                       // 单个体素的边长（分辨率）
        int voxNum;                         // 总体素数量
        Eigen::Vector3i step;               // 索引步长：用于将(x,y,z)索引转换为一维数组索引
        Eigen::Vector3d oc;                 // 偏移后的原点（对应体素中心而非体素角点）
        Eigen::Vector3i bounds;             // 边界索引：每个维度的最大有效步长索引
        Eigen::Vector3d stepScale;          // 索引到物理坐标的转换系数
        std::vector<uint8_t> voxels;        // 体素数组：存储每个体素的占用状态
        std::vector<Eigen::Vector3i> surf;  // 表面体素列表：存储障碍物表面（膨胀外层）的体素索引

    public:
        /**
         * @brief 获取地图的体素尺寸
         * @return 返回三维体素数量 (nx, ny, nz)
         */
        inline Eigen::Vector3i getSize(void) const
        {
            return mapSize;
        }

        /**
         * @brief 获取体素的尺度（分辨率）
         * @return 返回单个体素的边长（米）
         */
        inline double getScale(void) const
        {
            return scale;
        }

        /**
         * @brief 获取地图原点坐标
         * @return 返回地图原点的世界坐标
         */
        inline Eigen::Vector3d getOrigin(void) const
        {
            return o;
        }

        /**
         * @brief 获取地图的对角顶点坐标
         * @return 返回与原点相对的顶点坐标（地图的右上角）
         */
        inline Eigen::Vector3d getCorner(void) const
        {
            return mapSize.cast<double>() * scale + o;
        }

        /**
         * @brief 获取体素数组的常量引用
         * @return 返回存储所有体素状态的数组
         */
        inline const std::vector<uint8_t> &getVoxels(void) const
        {
            return voxels;
        }

        /**
         * @brief 根据世界坐标设置体素为占用状态
         * @param pos 世界坐标系下的三维位置
         *
         * 工作流程：
         * 1. 将世界坐标转换为体素索引
         * 2. 检查索引是否在地图范围内
         * 3. 如果在范围内，将对应体素标记为占用
         */
        inline void setOccupied(const Eigen::Vector3d &pos)
        {
            // 计算体素索引：(pos - origin) / scale
            const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
            // 边界检查：确保索引在有效范围内
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                // 将三维索引转换为一维数组索引并标记为占用
                // 一维索引 = id(0)*step(0) + id(1)*step(1) + id(2)*step(2)
                voxels[id.dot(step)] = Occupied;
            }
        }

        /**
         * @brief 根据体素索引设置体素为占用状态
         * @param id 体素的三维索引 (ix, iy, iz)
         *
         * 直接使用体素索引进行设置，无需坐标转换
         */
        inline void setOccupied(const Eigen::Vector3i &id)
        {
            // 边界检查
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                // 标记为占用
                voxels[id.dot(step)] = Occupied;
            }
        }

        /**
         * @brief 对障碍物进行膨胀操作
         * @param r 膨胀半径（体素单位）
         *
         * 功能说明：
         * 障碍物膨胀用于在障碍物周围创建安全边界，防止轨迹过于接近障碍物。
         * 膨胀采用层层扩展的方式，类似于广度优先搜索（BFS）。
         *
         * 算法流程：
         * 1. 遍历所有占用体素，对其26邻域进行膨胀标记（第一层）
         * 2. 迭代r-1次，每次基于上一层的膨胀体素继续向外扩展
         * 3. 最外层的膨胀体素作为障碍物表面点存储
         *
         * @note 使用VOXEL_DILATER宏实现高效的26邻域遍历和标记
         */
        inline void dilate(const int &r)
        {
            // 如果膨胀半径<=0，不进行膨胀
            if (r <= 0)
            {
                return;
            }
            else
            {
                // lvec: 上一层膨胀的体素列表
                // cvec: 当前层膨胀的体素列表
                std::vector<Eigen::Vector3i> lvec, cvec;
                lvec.reserve(voxNum);
                cvec.reserve(voxNum);
                int i, j, k, idx;
                bool check;

                // 第一步：遍历所有占用体素，对其邻域进行膨胀
                for (int x = 0; x <= bounds(0); x++)
                {
                    for (int y = 0; y <= bounds(1); y += step(1))
                    {
                        for (int z = 0; z <= bounds(2); z += step(2))
                        {
                            if (voxels[x + y + z] == Occupied)
                            {
                                // VOXEL_DILATER宏：对当前占用体素的26邻域进行膨胀
                                // 将新膨胀的体素添加到cvec中
                                VOXEL_DILATER(i, j, k,
                                              x, y, z,
                                              step(1), step(2),
                                              bounds(0), bounds(1), bounds(2),
                                              check, voxels, idx, Dilated, cvec)
                            }
                        }
                    }
                }

                // 第二步：迭代膨胀r-1次
                for (int loop = 1; loop < r; loop++)
                {
                    // 交换当前层和上一层
                    std::swap(cvec, lvec);
                    // 对上一层的每个膨胀体素继续向外膨胀
                    for (const Eigen::Vector3i &id : lvec)
                    {
                        VOXEL_DILATER(i, j, k,
                                      id(0), id(1), id(2),
                                      step(1), step(2),
                                      bounds(0), bounds(1), bounds(2),
                                      check, voxels, idx, Dilated, cvec)
                    }
                    lvec.clear();
                }

                // 保存最外层膨胀体素作为障碍物表面
                surf = cvec;
            }
        }

        /**
         * @brief 获取指定立方体区域内的表面点
         * @param center 立方体中心的体素索引
         * @param halfWidth 立方体的半边长（体素单位）
         * @param points 输出参数，存储区域内的表面点世界坐标
         *
         * 功能：从障碍物表面体素中筛选出位于指定立方体区域内的点，
         * 并将其索引转换为世界坐标系下的物理坐标。
         *
         * 应用场景：局部路径规划时只需要关注机器人附近的障碍物表面
         */
        inline void getSurfInBox(const Eigen::Vector3i &center,
                                 const int &halfWidth,
                                 std::vector<Eigen::Vector3d> &points) const
        {
            // 遍历所有表面体素
            for (const Eigen::Vector3i &id : surf)
            {
                // 检查体素是否在立方体区域内（切比雪夫距离判断）
                // 注意：id(1)和id(2)需要除以step才能得到真实的体素索引
                if (std::abs(id(0) - center(0)) <= halfWidth &&
                    std::abs(id(1) / step(1) - center(1)) <= halfWidth &&
                    std::abs(id(2) / step(2) - center(2)) <= halfWidth)
                {
                    // 将体素索引转换为世界坐标：id * stepScale + oc
                    points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);
                }
            }

            return;
        }

        /**
         * @brief 获取所有障碍物表面点
         * @param points 输出参数，存储所有表面点的世界坐标
         *
         * 功能：将所有表面体素的索引转换为世界坐标系下的物理坐标。
         * 表面点可用于可视化、碰撞检测、距离场计算等。
         */
        inline void getSurf(std::vector<Eigen::Vector3d> &points) const
        {
            points.reserve(surf.size());  // 预分配内存以提高效率
            // 遍历所有表面体素并转换为世界坐标
            for (const Eigen::Vector3i &id : surf)
            {
                points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);
            }
            return;
        }

        /**
         * @brief 查询指定世界坐标位置是否被占用
         * @param pos 世界坐标系下的三维位置
         * @return true表示占用或超出地图范围，false表示空闲
         *
         * 功能：碰撞检测的核心函数，用于判断给定位置是否可通行。
         *
         * @note 超出地图范围的位置被视为占用（保守策略）
         */
        inline bool query(const Eigen::Vector3d &pos) const
        {
            // 将世界坐标转换为体素索引
            const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
            // 检查索引是否在地图范围内
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                // 返回体素的占用状态（0=未占用，非0=占用或膨胀）
                return voxels[id.dot(step)];
            }
            else
            {
                // 超出地图范围，返回true（视为占用）
                return true;
            }
        }

        /**
         * @brief 查询指定体素索引位置是否被占用
         * @param id 体素的三维索引
         * @return true表示占用或超出地图范围，false表示空闲
         *
         * 功能：直接使用体素索引进行查询，无需坐标转换，效率更高。
         */
        inline bool query(const Eigen::Vector3i &id) const
        {
            // 检查索引是否在地图范围内
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                // 返回体素的占用状态
                return voxels[id.dot(step)];
            }
            else
            {
                // 超出地图范围，返回true（视为占用）
                return true;
            }
        }

        /**
         * @brief 将体素索引转换为世界坐标
         * @param id 体素的三维索引 (ix, iy, iz)
         * @return 体素中心点的世界坐标
         *
         * 转换公式：pos = id * scale + oc
         * 其中oc已包含半个体素边长的偏移，使得结果为体素中心坐标
         */
        inline Eigen::Vector3d posI2D(const Eigen::Vector3i &id) const
        {
            return id.cast<double>() * scale + oc;
        }

        /**
         * @brief 将世界坐标转换为体素索引
         * @param pos 世界坐标系下的三维位置
         * @return 对应的体素索引 (ix, iy, iz)
         *
         * 转换公式：id = floor((pos - origin) / scale)
         *
         * @note 返回的索引可能超出地图范围，使用前需进行边界检查
         */
        inline Eigen::Vector3i posD2I(const Eigen::Vector3d &pos) const
        {
            return ((pos - o) / scale).cast<int>();
        }
    };
}  // namespace voxel_map

#endif
