#include "ego_path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

/**
 * @brief 析构函数：释放三维网格节点池的所有内存
 *
 * 遍历整个三维网格节点池(GridNodeMap_)，逐个删除动态分配的GridNode对象
 * 防止内存泄漏
 */
AStar::~AStar()
{
    // 三层嵌套循环释放所有网格节点的内存
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

/**
 * @brief 初始化网格地图和节点池
 *
 * @param occ_map 占据栅格地图的智能指针，用于碰撞检测
 * @param pool_size 节点池的三维尺寸(x, y, z方向的网格数量)
 *
 * 功能：
 * 1. 设置节点池大小和中心索引
 * 2. 动态分配三维网格节点池内存
 * 3. 为每个网格位置创建GridNode对象
 * 4. 保存占据地图的引用
 */
void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;           // 保存节点池大小
    CENTER_IDX_ = pool_size / 2;      // 计算中心索引，用于局部地图中心定位

    // 分配三维数组的第一维(x方向)
    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        // 分配第二维(y方向)
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            // 分配第三维(z方向)
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                // 为每个网格位置创建GridNode对象
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    // 保存占据地图的引用，用于后续的碰撞检测
    grid_map_ = occ_map;
}

/**
 * @brief 计算对角线启发式距离(Diagonal Heuristic)
 *
 * @param node1 起始节点
 * @param node2 目标节点
 * @return double 启发式距离值
 *
 * 算法原理：
 * 在允许对角线移动的网格中，计算从node1到node2的最优路径估计
 * - 三维对角线移动代价为√3
 * - 二维对角线移动代价为√2
 * - 一维直线移动代价为1.0
 *
 * 策略：优先使用三维对角线移动，然后二维对角线，最后直线移动
 */
double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    // 计算三个维度的索引差值(绝对值)
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    // 找出最小差值，这是可以进行三维对角线移动的步数
    int diag = min(min(dx, dy), dz);
    // 减去三维对角线移动后的剩余距离
    dx -= diag;
    dy -= diag;
    dz -= diag;

    // 根据哪个维度先被消耗完，计算剩余的移动代价
    if (dx == 0)
    {
        // x维度先消耗完，剩余在yz平面内移动
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        // y维度先消耗完，剩余在xz平面内移动
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        // z维度先消耗完，剩余在xy平面内移动
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

/**
 * @brief 计算曼哈顿启发式距离(Manhattan Heuristic)
 *
 * @param node1 起始节点
 * @param node2 目标节点
 * @return double 曼哈顿距离(三个维度差值的绝对值之和)
 *
 * 适用场景：只允许沿坐标轴方向移动(上下左右前后)时的最短路径估计
 * 曼哈顿距离是A*算法中常用的启发函数，满足可接受性(admissible)
 */
double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;  // 曼哈顿距离 = |Δx| + |Δy| + |Δz|
}

/**
 * @brief 计算欧几里得启发式距离(Euclidean Heuristic)
 *
 * @param node1 起始节点
 * @param node2 目标节点
 * @return double 欧几里得直线距离
 *
 * 欧几里得距离是两点间的真实直线距离，是最优的启发函数(最接近真实距离)
 * 在允许任意方向移动时提供最准确的估计
 */
double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    // 计算索引差值向量的L2范数(欧几里得距离)
    return (node2->index - node1->index).norm();
}

/**
 * @brief 回溯提取路径
 *
 * @param current 终点节点指针
 * @return vector<GridNodePtr> 从终点到起点的路径节点序列
 *
 * 通过回溯每个节点的cameFrom指针，从终点回溯到起点
 * 注意：返回的路径是反向的(终点→起点)，使用时需要reverse
 */
vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);  // 首先加入终点

    // 沿着cameFrom指针链回溯，直到起点(起点的cameFrom为NULL)
    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;  // 返回路径(终点→起点的顺序)
}

/**
 * @brief 转换起点终点坐标为索引，并调整障碍物内的点
 *
 * @param start_pt 起点的世界坐标
 * @param end_pt 终点的世界坐标
 * @param start_idx 输出：起点对应的网格索引
 * @param end_idx 输出：终点对应的网格索引
 * @return true 转换和调整成功
 * @return false 转换失败或无法找到有效位置
 *
 * 功能：
 * 1. 将世界坐标转换为网格索引
 * 2. 检查起点和终点是否在障碍物内
 * 3. 如果在障碍物内，沿着远离对方的方向逐步移动，直到找到自由空间
 */
bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    // 将世界坐标转换为网格索引，如果转换失败则返回false
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    // 检查起点是否在障碍物内
    if (checkOccupancy(Index2Coord(start_idx)))
    {
        //ROS_WARN("Start point is insdide an obstacle.");
        // 沿着远离终点的方向逐步移动起点，直到找到自由空间
        do
        {
            // 计算从终点指向起点的单位向量，乘以步长后移动起点
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;  // 如果移出网格范围，返回失败
        } while (checkOccupancy(Index2Coord(start_idx)));  // 直到不在障碍物内
    }

    // 检查终点是否在障碍物内
    if (checkOccupancy(Index2Coord(end_idx)))
    {
        //ROS_WARN("End point is insdide an obstacle.");
        // 沿着远离起点的方向逐步移动终点，直到找到自由空间
        do
        {
            // 计算从起点指向终点的单位向量，乘以步长后移动终点
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;  // 如果移出网格范围，返回失败
        } while (checkOccupancy(Index2Coord(end_idx)));  // 直到不在障碍物内
    }

    return true;
}

/**
 * @brief A*路径搜索主函数
 *
 * @param step_size 网格步长(米)
 * @param start_pt 起点世界坐标
 * @param end_pt 终点世界坐标
 * @return true 找到可行路径
 * @return false 搜索失败
 *
 * A*算法核心流程：
 * 1. 初始化：转换坐标、准备起点和终点节点
 * 2. 将起点加入OpenSet(优先队列)
 * 3. 主循环：
 *    a. 从OpenSet中取出f值最小的节点(当前节点)
 *    b. 检查是否到达终点
 *    c. 将当前节点移至ClosedSet
 *    d. 扩展当前节点的所有邻居(26连通)
 *    e. 对每个邻居计算g值和f值，更新或加入OpenSet
 * 4. 回溯路径或超时返回
 *
 * f(n) = g(n) + h(n)
 * - g(n): 从起点到节点n的实际代价
 * - h(n): 从节点n到终点的启发式估计代价
 */
bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();  // 记录搜索开始时间
    ++rounds_;  // 轮次计数器递增，用于区分不同的搜索过程

    // 保存步长参数
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;  // 步长的倒数，用于坐标索引转换
    center_ = (start_pt + end_pt) / 2;  // 计算起点和终点的中心，用于局部地图定位

    // 转换起点和终点坐标为网格索引，并调整障碍物内的点
    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return false;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    // 获取起点和终点在节点池中的指针
    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    // 清空OpenSet优先队列(通过swap空队列的方式)
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    // 初始化邻居节点和当前节点指针
    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    // 初始化起点节点
    startPtr->index = start_idx;
    startPtr->rounds = rounds_;  // 标记本轮搜索
    startPtr->gScore = 0;  // 起点的g值为0
    startPtr->fScore = getHeu(startPtr, endPtr);  // f = g + h = 0 + h
    startPtr->state = GridNode::OPENSET;  // 标记为OpenSet状态
    startPtr->cameFrom = NULL;  // 起点没有父节点
    openSet_.push(startPtr);  // 将起点加入OpenSet

    // 设置终点的索引
    endPtr->index = end_idx;

    double tentative_gScore;  // 临时g值，用于邻居节点的代价计算

    int num_iter = 0;  // 迭代计数器
    // A*主循环：当OpenSet不为空时继续搜索
    while (!openSet_.empty())
    {
        num_iter++;
        // 从OpenSet中取出f值最小的节点(优先队列自动排序)
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        // 检查是否到达终点(索引完全相同)
        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );

            // 找到路径！回溯生成完整路径
            gridPath_ = retrievePath(current);
            return true;
        }
        // 将当前节点从OpenSet移至ClosedSet
        current->state = GridNode::CLOSEDSET;

        // 扩展当前节点的所有邻居(3D网格中的26连通，即周围3x3x3-1=26个节点)
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    // 跳过当前节点本身
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    // 计算邻居节点的索引
                    Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    // 边界检查：邻居索引必须在节点池范围内(留1格边界)
                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 ||
                        neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 ||
                        neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;
                    }

                    // 获取邻居节点指针
                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    // 检查邻居是否在本轮搜索中已被访问过
                    bool flag_explored = neighborPtr->rounds == rounds_;

                    // 如果邻居已在ClosedSet中，跳过(已找到最优路径)
                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue;
                    }

                    // 标记邻居为本轮搜索
                    neighborPtr->rounds = rounds_;

                    // 碰撞检测：如果邻居在障碍物内，跳过
                    if (checkOccupancy(Index2Coord(neighborPtr->index)))
                    {
                        continue;
                    }

                    // 计算从当前节点到邻居的移动代价
                    // 对角线移动代价为√2或√3，直线移动代价为1
                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    // 计算从起点经当前节点到邻居的总代价
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        // 发现新节点：首次访问该邻居
                        neighborPtr->state = GridNode::OPENSET;  // 标记为OpenSet
                        neighborPtr->cameFrom = current;  // 设置父节点
                        neighborPtr->gScore = tentative_gScore;  // 设置g值
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);  // 计算f值
                        openSet_.push(neighborPtr);  // 加入OpenSet
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    {
                        // 找到更优路径：邻居已在OpenSet中，但新路径代价更小
                        neighborPtr->cameFrom = current;  // 更新父节点
                        neighborPtr->gScore = tentative_gScore;  // 更新g值
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);  // 更新f值
                        // 注意：优先队列不支持直接更新元素，但由于f值变小，
                        // 该节点会在后续迭代中以更小的f值再次被取出
                    }
                }
        // 超时检查：如果搜索时间超过0.2秒，终止搜索
        ros::Time time_2 = ros::Time::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    }

    // 如果OpenSet为空仍未找到路径，说明不存在可行路径
    ros::Time time_2 = ros::Time::now();

    // 如果搜索时间超过0.1秒，记录警告信息
    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    return false;  // 搜索失败
}

/**
 * @brief 获取路径的世界坐标序列
 *
 * @return vector<Vector3d> 从起点到终点的路径点序列(世界坐标)
 *
 * 将内部存储的网格节点路径(gridPath_)转换为世界坐标序列
 * 由于gridPath_是反向的(终点→起点)，需要reverse得到正向路径
 */
vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    // 遍历网格路径，将每个节点的索引转换为世界坐标
    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    // 反转路径，得到从起点到终点的顺序
    reverse(path.begin(), path.end());
    return path;
}
