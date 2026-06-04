/**
 * @file raycast.h
 * @brief 3D空间射线投射(Raycast)算法实现
 *
 * 本文件实现了基于DDA(Digital Differential Analyzer)算法的3D射线投射，
 * 用于路径规划中的碰撞检测、视线检测和障碍物探测等功能。
 *
 * 应用场景：
 * - 碰撞检测：检测从起点到终点的路径上是否有障碍物
 * - 视线检测：判断两点之间是否可视
 * - 体素遍历：遍历射线经过的所有网格/体素
 * - 地图探索：获取传感器视野范围内的体素
 */

#ifndef RAYCAST_H_
#define RAYCAST_H_

#include <Eigen/Eigen>
#include <vector>

/**
 * @brief 符号函数，返回数值的符号
 * @param x 输入数值
 * @return 返回 1.0 (x>0), 0.0 (x=0), 或 -1.0 (x<0)
 */
double signum(double x);

/**
 * @brief 模运算函数
 * @param value 被除数
 * @param modulus 模数（除数）
 * @return 返回 value % modulus 的浮点数结果
 */
double mod(double value, double modulus);

/**
 * @brief 计算到下一个整数边界的距离比例
 *
 * 用于DDA算法中计算射线到达下一个网格边界所需的参数增量。
 * 这是射线投射算法的核心辅助函数。
 *
 * @param s 起始位置坐标
 * @param ds 方向增量（速度分量）
 * @return 返回从当前位置到下一个整数边界的归一化距离 t，使得 s + t*ds 刚好到达边界
 */
double intbound(double s, double ds);

/**
 * @brief 3D射线投射函数（数组输出版本）
 *
 * 从起点到终点进行射线投射，返回射线经过的所有体素（网格）中心点。
 * 使用DDA算法高效遍历3D网格空间。
 *
 * @param start 射线起点（3D坐标）
 * @param end 射线终点（3D坐标）
 * @param min 网格空间的最小边界（用于坐标转换）
 * @param max 网格空间的最大边界（用于坐标转换）
 * @param output_points_cnt 输出参数：返回经过的体素数量
 * @param output 输出数组：存储所有经过的体素中心点坐标（需预先分配足够空间）
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output);

/**
 * @brief 3D射线投射函数（vector输出版本）
 *
 * 从起点到终点进行射线投射，返回射线经过的所有体素（网格）中心点。
 * 使用vector容器自动管理内存，更方便使用。
 *
 * @param start 射线起点（3D坐标）
 * @param end 射线终点（3D坐标）
 * @param min 网格空间的最小边界（用于坐标转换）
 * @param max 网格空间的最大边界（用于坐标转换）
 * @param output 输出vector：存储所有经过的体素中心点坐标
 */
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output);

/**
 * @class RayCaster
 * @brief 射线投射器类 - 用于逐步遍历3D射线经过的体素
 *
 * 这是一个增量式的射线投射实现，允许逐步获取射线经过的每个体素。
 * 相比于一次性返回所有体素的函数版本，这个类更适合：
 * - 需要提前终止遍历的场景（如找到第一个障碍物）
 * - 需要在遍历过程中进行复杂处理的场景
 * - 内存受限的场景（不需要存储所有中间结果）
 *
 * 算法基于Amanatides & Woo的快速体素遍历算法（A Fast Voxel Traversal Algorithm）
 */
class RayCaster {
private:
  /* ==================== 射线参数 ==================== */
  Eigen::Vector3d start_;      ///< 射线起点（世界坐标）
  Eigen::Vector3d end_;        ///< 射线终点（世界坐标）
  Eigen::Vector3d direction_;  ///< 射线方向向量（归一化）
  Eigen::Vector3d min_;        ///< 网格空间的最小边界
  Eigen::Vector3d max_;        ///< 网格空间的最大边界

  /* ==================== 当前体素坐标 ==================== */
  int x_;     ///< 当前体素的X坐标（网格索引）
  int y_;     ///< 当前体素的Y坐标（网格索引）
  int z_;     ///< 当前体素的Z坐标（网格索引）

  /* ==================== 目标体素坐标 ==================== */
  int endX_;  ///< 终点体素的X坐标（网格索引）
  int endY_;  ///< 终点体素的Y坐标（网格索引）
  int endZ_;  ///< 终点体素的Z坐标（网格索引）

  /* ==================== 距离参数 ==================== */
  double maxDist_;  ///< 射线的最大长度（起点到终点的距离）

  /* ==================== 方向增量 ==================== */
  double dx_;  ///< X方向的增量
  double dy_;  ///< Y方向的增量
  double dz_;  ///< Z方向的增量

  /* ==================== 步进方向 ==================== */
  int stepX_;  ///< X方向的步进符号 (+1 或 -1)
  int stepY_;  ///< Y方向的步进符号 (+1 或 -1)
  int stepZ_;  ///< Z方向的步进符号 (+1 或 -1)

  /* ==================== DDA算法核心参数 ==================== */
  /**
   * tMax: 到达下一个网格边界所需的射线参数 t
   * - tMaxX_: 到达X方向下一个边界的 t 值
   * - tMaxY_: 到达Y方向下一个边界的 t 值
   * - tMaxZ_: 到达Z方向下一个边界的 t 值
   * 每次选择最小的tMax对应的方向进行步进
   */
  double tMaxX_;   ///< X方向到达下一个边界的参数值
  double tMaxY_;   ///< Y方向到达下一个边界的参数值
  double tMaxZ_;   ///< Z方向到达下一个边界的参数值

  /**
   * tDelta: 沿射线方向移动一个网格单元所需的参数增量
   * - tDeltaX_: X方向移动一个网格的参数增量
   * - tDeltaY_: Y方向移动一个网格的参数增量
   * - tDeltaZ_: Z方向移动一个网格的参数增量
   */
  double tDeltaX_;  ///< X方向的参数增量（每个网格）
  double tDeltaY_;  ///< Y方向的参数增量（每个网格）
  double tDeltaZ_;  ///< Z方向的参数增量（每个网格）

  /* ==================== 状态参数 ==================== */
  double dist_;     ///< 当前已遍历的距离

  int step_num_;    ///< 已执行的步数（遍历的体素数量）

public:
  /**
   * @brief 默认构造函数
   */
  RayCaster(/* args */) {
  }

  /**
   * @brief 析构函数
   */
  ~RayCaster() {
  }

  /**
   * @brief 设置射线投射的输入参数并初始化
   *
   * 该函数会初始化所有DDA算法所需的参数，包括步进方向、
   * 边界参数、增量参数等。调用此函数后，可以通过step()函数
   * 逐步获取射线经过的体素。
   *
   * @param start 射线起点（3D坐标）
   * @param end 射线终点（3D坐标）
   * @return 成功返回true，失败返回false（如射线长度为0）
   *
   * @note 参数中注释掉的min和max表示这些参数可能在未来版本中添加
   */
  bool setInput(const Eigen::Vector3d& start,
                const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                const Eigen::Vector3d& max */);

  /**
   * @brief 执行一步射线投射，获取下一个体素
   *
   * 每次调用会沿着射线方向前进到下一个体素，并返回该体素的中心点坐标。
   * 这是一个增量式的过程，适合需要在遍历过程中进行判断或处理的场景。
   *
   * @param ray_pt 输出参数：返回当前体素的中心点坐标
   * @return 如果还有下一个体素返回true，到达终点或超出范围返回false
   *
   * 使用示例：
   * @code
   * RayCaster raycaster;
   * raycaster.setInput(start, end);
   * Eigen::Vector3d pt;
   * while (raycaster.step(pt)) {
   *   // 处理当前体素点 pt
   *   if (isCollision(pt)) break;  // 遇到障碍物时可以提前终止
   * }
   * @endcode
   */
  bool step(Eigen::Vector3d& ray_pt);
};

#endif  // RAYCAST_H_