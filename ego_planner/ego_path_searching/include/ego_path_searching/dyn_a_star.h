/**
 * @file dyn_a_star.h
 * @brief 动态A*路径搜索算法头文件
 * @details 实现基于栅格地图的A*路径搜索算法，用于Ego-Planner中的全局路径规划
 *          支持动态环境下的路径搜索，使用启发式函数指导搜索方向
 */

#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <ego_plan_env/grid_map.h>
#include <queue>

// 定义无穷大常量，用于初始化节点代价
constexpr double inf = 1 >> 20;

// 前向声明栅格节点结构体
struct GridNode;
// 定义栅格节点指针类型别名
typedef GridNode *GridNodePtr;

/**
 * @struct GridNode
 * @brief 栅格节点结构体，用于A*算法的搜索节点
 * @details 存储节点在搜索过程中的状态、代价值、父节点等信息
 */
struct GridNode
{
	/**
	 * @enum enum_state
	 * @brief 节点状态枚举
	 */
	enum enum_state
	{
		OPENSET = 1,    // 节点在开放列表中（待探索）
		CLOSEDSET = 2,  // 节点在关闭列表中（已探索）
		UNDEFINED = 3   // 节点状态未定义（未访问）
	};

	int rounds{0}; // 搜索轮次，用于区分不同次的A*调用，避免重复初始化节点

	// 节点当前状态，默认为未定义
	enum enum_state state
	{
		UNDEFINED
	};

	Eigen::Vector3i index; // 节点在3D栅格地图中的索引坐标 (x, y, z)

	double gScore{inf}; // g值：从起点到当前节点的实际代价
	double fScore{inf}; // f值：f = g + h，用于优先队列排序（h为启发式估计值）
	GridNodePtr cameFrom{NULL}; // 父节点指针，用于回溯路径
};

/**
 * @class NodeComparator
 * @brief 节点比较器，用于优先队列的节点排序
 * @details 实现小顶堆，使得fScore较小的节点优先被取出
 *          在A*算法中，优先扩展f值最小的节点
 */
class NodeComparator
{
public:
	/**
	 * @brief 比较两个节点的fScore
	 * @param node1 节点1
	 * @param node2 节点2
	 * @return true 如果node1的fScore大于node2（用于构建小顶堆）
	 */
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

/**
 * @class AStar
 * @brief A*路径搜索算法类
 * @details 实现基于栅格地图的A*路径搜索算法
 *          特点：
 *          1. 支持3D空间搜索
 *          2. 使用对角线启发式函数提高搜索效率
 *          3. 支持动态环境（通过rounds机制避免重复初始化）
 *          4. 考虑障碍物膨胀，保证安全距离
 */
class AStar
{
private:
	GridMap::Ptr grid_map_; // 栅格地图指针，存储环境障碍物信息

	/**
	 * @brief 快速将连续坐标转换为栅格索引
	 * @param x, y, z 连续空间坐标
	 * @param id_x, id_y, id_z 输出的栅格索引
	 */
	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	/**
	 * @brief 计算对角线启发式函数（Diagonal Heuristic）
	 * @details 考虑对角移动的启发式，比曼哈顿距离更准确
	 * @param node1 当前节点
	 * @param node2 目标节点
	 * @return 启发式估计值
	 */
	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);

	/**
	 * @brief 计算曼哈顿距离启发式函数
	 * @param node1 当前节点
	 * @param node2 目标节点
	 * @return 曼哈顿距离
	 */
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);

	/**
	 * @brief 计算欧几里得距离启发式函数
	 * @param node1 当前节点
	 * @param node2 目标节点
	 * @return 欧几里得距离
	 */
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);

	/**
	 * @brief 获取启发式函数值（内联函数，实际调用getDiagHeu）
	 * @param node1 当前节点
	 * @param node2 目标节点
	 * @return 启发式估计值（带tie_breaker_修正）
	 */
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	/**
	 * @brief 转换起点和终点为栅格索引，并调整到有效位置
	 * @param start_pt 起点坐标
	 * @param end_pt 终点坐标
	 * @param start_idx 输出的起点索引
	 * @param end_idx 输出的终点索引
	 * @return 转换是否成功
	 */
	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	/**
	 * @brief 将栅格索引转换为连续空间坐标
	 * @param index 栅格索引
	 * @return 对应的连续空间坐标
	 */
	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;

	/**
	 * @brief 将连续空间坐标转换为栅格索引
	 * @param pt 连续空间坐标
	 * @param idx 输出的栅格索引
	 * @return 转换是否成功（是否在地图范围内）
	 */
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	// 函数指针方式检查占据（已注释）
	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	/**
	 * @brief 检查指定位置是否被障碍物占据（考虑膨胀）
	 * @param pos 待检查的位置
	 * @return true表示被占据，false表示自由空间
	 */
	inline bool checkOccupancy(const Eigen::Vector3d &pos) { return (bool)grid_map_->getInflateOccupancy(pos); }

	/**
	 * @brief 从目标节点回溯路径到起点
	 * @param current 当前节点（通常是目标节点）
	 * @return 从起点到终点的路径节点序列
	 */
	std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	double step_size_;      // 栅格分辨率，每个栅格的边长
	double inv_step_size_;  // 栅格分辨率的倒数，用于加速坐标转换
	Eigen::Vector3d center_; // 地图中心坐标
	Eigen::Vector3i CENTER_IDX_; // 地图中心的栅格索引
	Eigen::Vector3i POOL_SIZE_;  // 节点池大小（地图的栅格数量）

	// Tie-breaker系数，用于打破启发式函数值相同的节点
	// 稍微增大启发式值，使搜索更倾向于朝目标方向移动
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> gridPath_; // 存储搜索得到的栅格路径

	GridNodePtr ***GridNodeMap_; // 3D节点池数组，存储所有栅格节点

	// 开放列表，使用优先队列实现，按fScore排序
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	int rounds_{0}; // 当前搜索轮次，用于区分不同次调用

public:
	// 定义智能指针类型，方便对象管理
	typedef std::shared_ptr<AStar> Ptr;

	/**
	 * @brief 默认构造函数
	 */
	AStar(){};

	/**
	 * @brief 析构函数，释放动态分配的节点池内存
	 */
	~AStar();

	/**
	 * @brief 初始化栅格地图和节点池
	 * @param occ_map 障碍物栅格地图指针
	 * @param pool_size 节点池大小（地图的栅格维度）
	 */
	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

	/**
	 * @brief 执行A*路径搜索
	 * @param step_size 栅格分辨率
	 * @param start_pt 起点坐标
	 * @param end_pt 终点坐标
	 * @return true表示搜索成功找到路径，false表示无解
	 */
	bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

	/**
	 * @brief 获取搜索得到的路径（连续空间坐标序列）
	 * @return 从起点到终点的路径点序列
	 */
	std::vector<Eigen::Vector3d> getPath();
};

/**
 * @brief 计算启发式函数值（内联实现）
 * @details 使用对角线启发式，并乘以tie_breaker_系数
 *          tie_breaker_略大于1，用于打破f值相同时的平局
 *          使算法更倾向于选择朝向目标的节点
 * @param node1 当前节点
 * @param node2 目标节点
 * @return 调整后的启发式估计值
 */
inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

/**
 * @brief 将栅格索引转换为连续空间坐标（内联实现）
 * @details 转换公式：coord = (index - CENTER_IDX) * step_size + center
 *          先计算相对中心的偏移量，乘以分辨率，再加上中心坐标
 * @param index 栅格索引
 * @return 对应的连续空间坐标
 */
inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

/**
 * @brief 将连续空间坐标转换为栅格索引（内联实现）
 * @details 转换公式：index = floor((pt - center) * inv_step_size + 0.5) + CENTER_IDX
 *          1. 计算相对中心的偏移量
 *          2. 乘以分辨率倒数得到栅格偏移
 *          3. 加0.5进行四舍五入
 *          4. 转为整数后加上中心索引
 *          同时检查索引是否在有效范围内
 * @param pt 连续空间坐标
 * @param idx 输出的栅格索引
 * @return true表示坐标在地图范围内，false表示超出边界
 */
inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	// 坐标转索引：先减去中心坐标，乘以分辨率倒数，加0.5四舍五入，最后加上中心索引
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

	// 检查索引是否在节点池范围内
	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}

	return true;
};

#endif
