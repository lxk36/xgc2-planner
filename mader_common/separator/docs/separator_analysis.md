# 功能包: separator

## 1. 功能概述

*   **一句话总结**: 基于线性规划(Linear Programming)求解的几何分离平面算法库，用于判断两个3D点集是否线性可分，并找到分离超平面
*   **核心节点**: 无ROS节点（纯库形式提供）
*   **算法类型**: 基于线性规划(Linear Programming)的凸集分离算法，支持GLPK和Gurobi两种求解器

## 2. 依赖关系

*   **主要依赖项**:
    *   `catkin`: ROS构建系统
    *   `Eigen3`: 线性代数运算库，用于矩阵和向量操作
    *   `GLPK` (默认): GNU线性规划工具包，开源LP求解器
    *   `Gurobi` (可选): 商业优化求解器，性能更强但需要许可证

## 3. 接口说明 (API)

### 3.1 主要类和方法

#### `separator::Separator` 类

**构造函数**:
```cpp
Separator();  // 使用默认参数构造
```

**核心求解方法**:
```cpp
// 方法1: 使用std::vector输入
bool solveModel(Eigen::Vector3d& solution, double& d,
                const std::vector<Eigen::Vector3d>& pointsA,
                const std::vector<Eigen::Vector3d>& pointsB);

// 方法2: 使用Eigen矩阵输入（推荐）
bool solveModel(Eigen::Vector3d& solutionN, double& solutionD,
                const Eigen::Matrix<double, 3, Eigen::Dynamic>& pointsA,
                const Eigen::Matrix<double, 3, Eigen::Dynamic>& pointsB);
```

**统计方法**:
```cpp
long int getNumOfLPsRun();     // 获取已运行的LP问题数量
double meanSolveTimeMs();      // 获取平均求解时间（毫秒）
```

### 3.2 输入参数

| 参数名 | 类型 | 描述 |
| :--- | :--- | :--- |
| `pointsA` | `std::vector<Eigen::Vector3d>` 或 `Eigen::Matrix<double, 3, Dynamic>` | 第一个点集的3D坐标，每个点是3维向量 |
| `pointsB` | `std::vector<Eigen::Vector3d>` 或 `Eigen::Matrix<double, 3, Dynamic>` | 第二个点集的3D坐标，每个点是3维向量 |

### 3.3 输出结果

| 参数名 | 类型 | 描述 |
| :--- | :--- | :--- |
| `solutionN` | `Eigen::Vector3d` | 分离超平面的法向量 n = [n1, n2, n3] |
| `solutionD` | `double` | 分离超平面的截距参数 d |
| 返回值 | `bool` | 求解成功标志：true表示两点集线性可分，false表示不可分 |

### 3.4 分离超平面方程

找到的分离超平面方程为：**n^T * x + d = 0**

其中：
- 对于点集A中的所有点：**n^T * x_A + d ≥ ε** (ε = 1.0)
- 对于点集B中的所有点：**n^T * x_B + d ≤ -ε**

## 4. 核心算法原理

该算法通过求解线性规划问题来寻找能够分离两个点集的超平面。核心思想是将几何分离问题转化为线性约束优化问题。

### 4.1 线性规划建模

给定两个点集 A = {x₁ᴬ, x₂ᴬ, ..., xₘᴬ} 和 B = {x₁ᴮ, x₂ᴮ, ..., xₙᴮ}，目标是找到超平面 n^T x + d = 0 使得两个点集位于超平面的不同侧。

**决策变量**:
- n = [n₁, n₂, n₃]^T: 超平面法向量
- d: 超平面截距参数

**约束条件**:
```
对于点集A: n^T * xᵢᴬ + d ≥ ε,  i = 1, 2, ..., m
对于点集B: n^T * xⱼᴮ + d ≤ -ε, j = 1, 2, ..., n
```

其中 ε = 1.0 是预设的分离间隔，避免使用过小的epsilon值导致数值不稳定。

### 4.2 GLPK求解器实现

算法使用GLPK（GNU Linear Programming Kit）作为默认求解器：

#### 步骤1: 问题初始化
```cpp
glp_prob* lp = glp_create_prob();
glp_set_obj_dir(lp, GLP_MAX);  // 设置为最大化问题
```

#### 步骤2: 添加约束
- 为点集A的每个点添加下界约束：`n^T * x_A + d ≥ 1.0`
- 为点集B的每个点添加上界约束：`n^T * x_B + d ≤ -1.0`

#### 步骤3: 设置决策变量
设置4个自由变量：n₁, n₂, n₃, d，范围为(-∞, +∞)

#### 步骤4: 构建系数矩阵
将约束条件转化为标准的线性规划矩阵形式 Ax ≤ b

#### 步骤5: 求解和结果提取
```cpp
glp_simplex(lp, &params);  // 执行单纯形算法
int status = glp_get_status(lp);  // 获取求解状态
```

### 4.3 可行性判断

算法的返回结果解释：
- **返回true**: 问题有可行解，两点集线性可分，找到分离超平面
- **返回false**: 问题无可行解，两点集线性不可分（可能存在交集或凸包相交）

### 4.4 几何意义

分离超平面将3D空间分为两个半空间：
- 正半空间：n^T * x + d > 0（包含点集A）
- 负半空间：n^T * x + d < 0（包含点集B）

这在多智能体路径规划中特别有用，可以：
1. **碰撞检测**: 判断两个凸多面体（由顶点表示）是否相交
2. **约束生成**: 为轨迹优化生成避障约束
3. **空间分割**: 将复杂环境分割为可通行和不可通行区域

## 5. 使用与配置

### 5.1 编译配置

在CMakeLists.txt中设置求解器选项：
```cmake
option(USE_GLPK "Use GLPK as the solver" ON)  # ON使用GLPK，OFF使用Gurobi
```

### 5.2 使用示例

```cpp
#include "separator.hpp"

int main() {
    separator::Separator separator_solver;

    // 定义两个点集
    Eigen::Matrix<double, 3, 4> pointsA;  // 点集A（4个点）
    Eigen::Matrix<double, 3, 5> pointsB;  // 点集B（5个点）

    // 填充点坐标
    pointsA.col(0) = Eigen::Vector3d(-3.50, 21, 1.4);
    pointsA.col(1) = Eigen::Vector3d(-2.71, 2.13, 1.6);
    // ... 更多点

    // 求解分离平面
    Eigen::Vector3d normal;
    double offset;
    bool success = separator_solver.solveModel(normal, offset, pointsA, pointsB);

    if (success) {
        std::cout << "分离平面法向量: " << normal.transpose() << std::endl;
        std::cout << "截距参数: " << offset << std::endl;
        std::cout << "分离平面方程: " << normal(0) << "*x + "
                  << normal(1) << "*y + " << normal(2) << "*z + "
                  << offset << " = 0" << std::endl;
    } else {
        std::cout << "两点集不可线性分离" << std::endl;
    }

    return 0;
}
```

### 5.3 在其他ROS包中使用

在目标包的CMakeLists.txt中添加：
```cmake
find_package(catkin REQUIRED COMPONENTS separator)
target_link_libraries(your_target ${catkin_LIBRARIES})
```

### 5.4 性能考虑

- **GLPK vs Gurobi**: 对于MADER框架中的典型LP问题，GLPK通常更快
- **输入格式**: 使用Eigen矩阵格式比std::vector更高效
- **问题规模**: 算法复杂度与点数量线性相关，适合实时应用

## 6. 在MADER框架中的应用

该separator库是MADER (Multi-Agent Dynamic Environment Replanning) 框架的核心组件之一，主要用途包括：

1. **动态障碍物避障**: 判断智能体轨迹与动态障碍物是否存在冲突
2. **多智能体碰撞避免**: 检测不同智能体的轨迹是否会产生碰撞
3. **安全走廊生成**: 为轨迹优化生成凸约束集合
4. **实时重规划**: 在动态环境中快速判断原有计划的可行性

该库的高效线性规划求解能力使得MADER能够在复杂的多智能体动态环境中实现实时的轨迹规划和避障。