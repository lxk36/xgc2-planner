/**
 * @file empty.cpp
 * @brief 占位源文件 - 用于满足ROS/CMake构建系统的最小要求
 *
 * @details
 * 这是一个空的C++源文件，主要用途是作为构建系统的占位符。
 * 在某些情况下，CMake或catkin构建系统可能需要至少一个源文件来成功编译，
 * 即使实际的功能实现都在头文件中或通过其他方式完成。
 *
 * 在ego_local_sensing包中，实际的传感器仿真功能由以下文件实现：
 * - pcl_render_node.cpp: 基于CUDA的点云渲染节点（ENABLE_CUDA=true时）
 * - pointcloud_render_node.cpp: 基于PCL的点云渲染节点（ENABLE_CUDA=false时）
 * - depth_render.cu: CUDA深度图渲染实现
 * - euroc.cpp: EuRoC数据集相关处理
 *
 * 此文件不包含任何实际功能代码，仅包含一个空头文件以维持编译系统的正确性。
 *
 * @note 此文件在CMakeLists.txt中未被显式引用，可能是历史遗留或作为备用占位符
 */

#include "empty.h"  // 包含空头文件（empty.h本身也是空的）