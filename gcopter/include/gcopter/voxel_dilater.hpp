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

/**
 * @file voxel_dilater.hpp
 * @brief 体素膨胀宏定义
 * @details 该文件定义了一个用于3D栅格地图中障碍物膨胀的宏VOXEL_DILATER。
 *          膨胀操作用于为障碍物增加安全边界，确保路径规划时保持足够的安全距离。
 *          该宏通过遍历当前体素的26个邻居（3x3x3立方体中除中心点外的所有点）来实现膨胀。
 */

#ifndef VOXEL_DILATER
/**
 * @brief 体素膨胀宏 - 用于在3D栅格地图中膨胀障碍物
 *
 * @param i 临时变量：计算后的x坐标索引
 * @param j 临时变量：计算后的y坐标索引
 * @param k 临时变量：计算后的z坐标索引
 * @param x 当前体素的x坐标
 * @param y 当前体素的y坐标
 * @param z 当前体素的z坐标
 * @param sy y方向的步长（用于一维数组索引计算）
 * @param sz z方向的步长（用于一维数组索引计算）
 * @param bx x方向的边界最大值
 * @param by y方向的边界最大值
 * @param bz z方向的边界最大值
 * @param ck 边界检查标志：如果当前体素在边界上则为true
 * @param ogm 占用栅格地图（Occupancy Grid Map）：存储体素占用状态的一维数组
 * @param ofst 偏移量：在一维数组中的索引位置
 * @param val 膨胀值：要设置给邻居体素的值
 * @param fdl 前沿列表（Frontier List）：存储待处理体素的队列/列表
 *
 * @details 该宏执行以下操作：
 *          1. 首先检查当前体素是否在地图边界上
 *          2. 遍历当前体素的26个邻居（3x3x3立方体，除中心点）
 *          3. 对每个邻居：
 *             - 计算其在一维数组中的索引
 *             - 检查是否在有效范围内
 *             - 如果该邻居未被占用（值为0），则：
 *               a) 设置其值为膨胀值
 *               b) 将其加入前沿列表以便后续处理
 *
 * @note 邻居遍历顺序（相对于中心点(x,y,z)的偏移）：
 *       - 9个位于x-1平面的邻居（左侧）
 *       - 8个位于x平面的邻居（中间，不包括中心点）
 *       - 9个位于x+1平面的邻居（右侧）
 */
#define VOXEL_DILATER(i, j, k, x, y, z, sy, sz, bx, by, bz, ck, ogm, ofst, val, fdl)                                                                                                                                                      \
/* 检查当前体素是否在地图边界上 */                                                                                                                                                                                                         \
(ck) = (x) == 0 || (x) == (bx) || (y) == 0 || (y) == (by) || (z) == 0 || (z) == (bz);                                                                                                                                                   \
/* === x-1 平面的9个邻居（左侧平面） === */                                                                                                                                                                                               \
/* 邻居1: (x-1, y-sy, z-sz) - 左下后 */                                                                                                                                                                                                  \
(i) = (x) - 1; (j) = (y) - (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) >= 0      && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居2: (x-1, y-sy, z) - 左下中 */                                                                                                                                                                                                      \
(i) = (x) - 1; (j) = (y) - (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) >= 0                    )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居3: (x-1, y-sy, z+sz) - 左下前 */                                                                                                                                                                                                  \
(i) = (x) - 1; (j) = (y) - (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) >= 0      && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居4: (x-1, y, z-sz) - 左中后 */                                                                                                                                                                                                      \
(i) = (x) - 1; (j) = (y);        (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0                     && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居5: (x-1, y, z) - 左中中 */                                                                                                                                                                                                         \
(i) = (x) - 1; (j) = (y);        (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0                                   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居6: (x-1, y, z+sz) - 左中前 */                                                                                                                                                                                                      \
(i) = (x) - 1; (j) = (y);        (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0                     && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居7: (x-1, y+sy, z-sz) - 左上后 */                                                                                                                                                                                                  \
(i) = (x) - 1; (j) = (y) + (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) <= (by)   && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居8: (x-1, y+sy, z) - 左上中 */                                                                                                                                                                                                      \
(i) = (x) - 1; (j) = (y) + (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) <= (by)                 )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居9: (x-1, y+sy, z+sz) - 左上前 */                                                                                                                                                                                                  \
(i) = (x) - 1; (j) = (y) + (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) <= (by)   && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* === x 平面的8个邻居（中间平面，不包括中心点(x,y,z)） === */                                                                                                                                                                                 \
/* 邻居10: (x, y-sy, z-sz) - 中下后 */                                                                                                                                                                                                       \
(i) = (x);     (j) = (y) - (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) >= 0      && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居11: (x, y-sy, z) - 中下中 */                                                                                                                                                                                                          \
(i) = (x);     (j) = (y) - (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) >= 0                    )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居12: (x, y-sy, z+sz) - 中下前 */                                                                                                                                                                                                       \
(i) = (x);     (j) = (y) - (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) >= 0      && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居13: (x, y, z-sz) - 中中后 */                                                                                                                                                                                                          \
(i) = (x);     (j) = (y);        (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                                 && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 注意: (x, y, z) 是中心点本身，不进行处理 */                                                                                                                                                                                                 \
/* 邻居14: (x, y, z+sz) - 中中前 */                                                                                                                                                                                                          \
(i) = (x);     (j) = (y);        (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                                 && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居15: (x, y+sy, z-sz) - 中上后 */                                                                                                                                                                                                       \
(i) = (x);     (j) = (y) + (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) <= (by)   && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居16: (x, y+sy, z) - 中上中 */                                                                                                                                                                                                          \
(i) = (x);     (j) = (y) + (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) <= (by)                 )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居17: (x, y+sy, z+sz) - 中上前 */                                                                                                                                                                                                       \
(i) = (x);     (j) = (y) + (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) <= (by)   && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* === x+1 平面的9个邻居（右侧平面） === */                                                                                                                                                                                                \
/* 邻居18: (x+1, y-sy, z-sz) - 右下后 */                                                                                                                                                                                                  \
(i) = (x) + 1; (j) = (y) - (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) >= 0      && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居19: (x+1, y-sy, z) - 右下中 */                                                                                                                                                                                                      \
(i) = (x) + 1; (j) = (y) - (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) >= 0                    )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居20: (x+1, y-sy, z+sz) - 右下前 */                                                                                                                                                                                                  \
(i) = (x) + 1; (j) = (y) - (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) >= 0      && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居21: (x+1, y, z-sz) - 右中后 */                                                                                                                                                                                                      \
(i) = (x) + 1; (j) = (y);        (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx)                  && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居22: (x+1, y, z) - 右中中 */                                                                                                                                                                                                         \
(i) = (x) + 1; (j) = (y);        (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx)                                )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居23: (x+1, y, z+sz) - 右中前 */                                                                                                                                                                                                      \
(i) = (x) + 1; (j) = (y);        (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx)                  && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居24: (x+1, y+sy, z-sz) - 右上后 */                                                                                                                                                                                                  \
(i) = (x) + 1; (j) = (y) + (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) <= (by)   && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居25: (x+1, y+sy, z) - 右上中 */                                                                                                                                                                                                      \
(i) = (x) + 1; (j) = (y) + (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) <= (by)                 )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
/* 邻居26: (x+1, y+sy, z+sz) - 右上前 */                                                                                                                                                                                                  \
(i) = (x) + 1; (j) = (y) + (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) <= (by)   && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }
#endif
