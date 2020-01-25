#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

/*
* @function     AstarPathFinder::initGridMap
* @brief        初始化每个栅格，设定索引与真实坐标的对应关系
* @param        double _resolution          地图分辨率
                Vector3d global_xyz_l       地图下界坐标向量
                Vector3d global_xyz_u       地图上届坐标向量
                int max_x_id                X轴的最大栅格索引值
                int max_y_id                Y轴的最大栅格索引值
                int max_z_id                Z轴的最大栅格索引值
* @return       None
*/
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{
    gl_xl = global_xyz_l(0); //地图下界坐标(栅格坐标以此为原点)
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0); //地图上界坐标
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    GLX_SIZE = max_x_id; //栅格地图索引最大值
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;

    GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;   //YZ平面的栅格总数
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE; //空间上的栅格总数

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;

    data = new uint8_t[GLXYZ_SIZE]; //存储栅格地图障碍物的占用概率(1:占用 0:未占用)
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

    GridNodeMap = new GridNodePtr **[GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; j++)
        {
            GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
            for (int k = 0; k < GLZ_SIZE; k++)
            {
                Vector3i tmpIdx(i, j, k);                         // i,j,k为栅格索引
                Vector3d pos = gridIndex2coord(tmpIdx);           //将栅格索引转换为空间真实坐标
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos); //初始化每个栅格，设定索引与真实坐标的对应关系
            }
        }
    }
}

/*
* @function     AstarPathFinder::resetGrid
* @brief        重置某个栅格
* @param        GridNodePtr ptr        指向一个栅格的指针
* @return       None
*/
void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

/*
* @function     AstarPathFinder::resetUsedGrids
* @brief        重置整张栅格地图
* @param        None
* @return       None
*/
void AstarPathFinder::resetUsedGrids()
{
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
            for (int k = 0; k < GLZ_SIZE; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

/*
* @function     AstarPathFinder::setObs
* @brief        设置栅格地图中的障碍物
* @param        double coord_x          障碍物X坐标
                double coord_y          障碍物Y坐标
                double coord_z          障碍物Z坐标
* @return       None
*/
void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
        return;

    int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1; //设置数组中对应索引的空间状态
}

/*
* @function     AstarPathFinder::getVisitedNodes
* @brief        显示访问过的栅格
* @param        None
* @return       vector<Vector3d> visited_nodes
*/
vector<Vector3d> AstarPathFinder::getVisitedNodes()
{
    vector<Vector3d> visited_nodes;
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
            for (int k = 0; k < GLZ_SIZE; k++)
            {
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if (GridNodeMap[i][j][k]->id == -1) // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

/*
* @function     AstarPathFinder::gridIndex2coord
* @brief        将某个栅格的索引转化为欧式空间中的坐标(取栅格的中心位置)
* @param        Vector3i & index        栅格的索引向量
* @return       Vector3d pt             对应的欧式空间坐标向量
*/
Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index)
{
    Vector3d pt;

    //栅格坐标以地图下界为原点
    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

/*
* @function     AstarPathFinder::coord2gridIndex
* @brief        将某个欧式空间中的坐标转化为栅格的索引(向无穷小取整)
* @param        Vector3d & pt           对应的欧式空间坐标向量
* @return       Vector3i idx            栅格的索引向量
*/
Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt)
{
    Vector3i idx;
    idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
        min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
        min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

/*
* @function 	AstarPathFinder::coordRounding
* @brief	    欧式空间坐标 -> 索引值 ->欧式空间坐标，为了显示
* @param	    Vector3d & coord       欧式空间坐标向量  
* @return	    Vector3d coord         欧式空间坐标向量 
*/
Vector3d AstarPathFinder::coordRounding(const Vector3d &coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

/*
* @function     AstarPathFinder::isOccupied
* @brief        判断某一栅格是否占用
* @param        const Vector3i &index   要查询的节点索引向量
* @return       bool isOccupied         是否占用
*/
inline bool AstarPathFinder::isOccupied(const Vector3i &index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

/*
* @function     AstarPathFinder::isFree
* @brief        判断某一栅格是否空闲
* @param        const Vector3i &index   要查询的节点索引向量
* @return       bool isFree             是否空闲
*/
inline bool AstarPathFinder::isFree(const Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

/*
* @function     AstarPathFinder::AstarGetSucc
* @brief        获得当前节点周围的nerigbor,并计算每个nerigbor的fscore
* @param        GridNodePtr currentPtr                  当前节点
                vector<GridNodePtr> &neighborPtrSets    neighbor集合
* @return       vector<double> &edgeCostSets            neighbor的cost集合
*/
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets, vector<double> &edgeCostSets)
{
    Vector3d coord_nerigbor;
    Vector3i idx_nerigbor;

    neighborPtrSets.clear();
    edgeCostSets.clear();
    static int count = 0;
    count++;
    ROS_INFO("Iteration Counts %d", count);

    for (int i = -1; i <= 1; i++)
        for (int j = -1; j <= 1; j++)
            for (int k = -1; k <= 1; k++)
            {
                if (!(i == j && j == k && i == 0))  //遍历当前节点为中心的3*3*3格子,除了自身
                {
                    idx_nerigbor(0) = currentPtr->index(0) + i;
                    idx_nerigbor(1) = currentPtr->index(1) + j;
                    idx_nerigbor(2) = currentPtr->index(2) + k;
                    coord_nerigbor = gridIndex2coord(idx_nerigbor);

                    if ((coord_nerigbor(0) >= gl_xl && coord_nerigbor(0) <= gl_xu)  //判断当前nerigbor是否有效
                    && (coord_nerigbor(1) >= gl_yl && coord_nerigbor(1) <= gl_yu) 
                    && (coord_nerigbor(2) >= gl_zl && coord_nerigbor(2) <= gl_zu))
                    {
                        idx_nerigbor = coord2gridIndex(coord_nerigbor);

                        if (isFree(idx_nerigbor))   //查询nerigbor所在栅格是否被障碍物占用,若已经存在障碍,则放弃该nerigbor
                        {
                            GridNodePtr neighborPtr = new GridNode(idx_nerigbor, coord_nerigbor);

                            neighborPtr->gScore = currentPtr->gScore + calEuclideanDistance(currentPtr, neighborPtr);  //邻居节点的g(n)
                            neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);  //邻居节点的f(n)
                            neighborPtr->cameFrom = currentPtr;   //设置该节点的parent node
                            // neighborPtr -> id = 1;   //标记该邻居节点为未访问状态

                            edgeCostSets.push_back(neighborPtr->fScore);
                            neighborPtrSets.push_back(neighborPtr);
                        }
                    }
                }
            }
}

/*
* @function     AstarPathFinder::calDistance
* @brief        计算Euclidean距离
* @param        GridNodePtr node1       节点1
                GridNodePtr node2       节点2
* @return       double distance         Euclidean距离
*/
double AstarPathFinder::calEuclideanDistance(GridNodePtr node1, GridNodePtr node2)
{
    double x1_node = node1->coord(0);
    double y1_node = node1->coord(1);
    double z1_node = node1->coord(2);
    double x2_node = node2->coord(0);
    double y2_node = node2->coord(1);
    double z2_node = node2->coord(2);
    double distance = sqrt(pow((x1_node - x2_node), 2) + pow((y1_node - y2_node), 2) + pow((z1_node - z2_node), 2));
    return distance;
}

/*
* @function     AstarPathFinder::calClosedFormedDistance
* @brief        计算任意两个节点之间距离的closed-form solution
* @param        GridNodePtr node1       节点1
                GridNodePtr node2       节点2
* @return       double distance         closed-form solution
*/
double AstarPathFinder::calClosedFormedDistance(GridNodePtr node1, GridNodePtr node2)
{
    double delta_x = abs(node1->coord(0) - node2->coord(0));
    double delta_y = abs(node1->coord(1) - node2->coord(1));
    double delta_z = abs(node1->coord(2) - node2->coord(2));
    double delta_min = min(min(delta_x, delta_y), min(delta_y, delta_z));
    if (delta_min == delta_x)
        return (sqrt(3) * delta_min + sqrt(2) * min((delta_y - delta_min), (delta_z - delta_min)) + abs(delta_y - delta_z));
    else if (delta_min == delta_y)
        return (sqrt(3) * delta_min + sqrt(2) * min((delta_x - delta_min), (delta_z - delta_min)) + abs(delta_z - delta_x));
    else
        return (sqrt(3) * delta_min + sqrt(2) * min((delta_x - delta_min), (delta_y - delta_min)) + abs(delta_y - delta_x));
}

/*
* @function     AstarPathFinder::getHeu
* @brief        计算某个节点的启发函数值h(n)
* @param        GridNodePtr node1       节点1
                GridNodePtr node2       目标节点
* @return       double score            h(n)
*/
double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    double score;

    //根据Euclidean距离计算h(n)
    // score = calEuclideanDistance(node1, node2);

    //根据该节点到目标节点距离的closed-form solution计算h(n)
    score = calClosedFormedDistance(node1, node2);
    return score;
}

/*
* @function     AstarPathFinder::AstarGraphSearch
* @brief        A*算法主流程
* @param        Vector3d start_pt       起点
                Vector3d end_pt         终点
* @return       None
*/
void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{
    ROS_INFO("Map Bound (%f,%f,%f,%f,%f,%f)", gl_xl, gl_yl, gl_zl, gl_xu, gl_yu, gl_zu);
    ros::Time time_start = ros::Time::now();

    //起点和终点的索引向量
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx = coord2gridIndex(end_pt);

    if(isOccupied(end_idx))
    {
         ROS_WARN("The goal is Occupied, Please choose a new goal");
         return;
    }

    //起点和终点的位置向量
    start_pt = gridIndex2coord(start_idx);
    end_pt = gridIndex2coord(end_idx);

    //初始化起点和终点的节点(指针)
    startPtr = new GridNode(start_idx, start_pt);
    endPtr = new GridNode(end_idx, end_pt);

    ROS_INFO("Start Node position : (%f,%f,%f)", startPtr->coord(0), startPtr->coord(1), startPtr->coord(2));
    ROS_INFO("Goal Node position : (%f,%f,%f)", endPtr->coord(0), endPtr->coord(1), endPtr->coord(2));
    
    
    openSet.clear();    //使用STL库中的multimap维护openset
    
    GridNodePtr currentPtr = NULL;    // currentPtr代表openset中f(n)最小的节点
    GridNodePtr neighborPtr = NULL;

    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->id = 1;
    openSet.insert(make_pair(startPtr->fScore, startPtr));   //将起点加入openset中

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    //主循环
    while (!openSet.empty())
    {

        std::multimap<double, GridNodePtr>::iterator itFoundmin;
        for (itFoundmin = openSet.begin(); itFoundmin != openSet.end(); itFoundmin++)
        {
            if (itFoundmin->second->id == 1)    //说明该节点没被访问过
            {
                currentPtr = itFoundmin->second;
                currentPtr->id = -1;    //标记当前节点为已访问状态
                GridNodeMap[currentPtr->index(0)][currentPtr->index(1)][currentPtr->index(2)]->id = -1;
                break;
            }
        }

        // ROS_INFO("Current Node index : (%d,%d,%d)", currentPtr->index(0), currentPtr->index(1), currentPtr->index(2));
        // ROS_INFO("Current Node Coord : (%f,%f,%f)", currentPtr->coord(0), currentPtr->coord(1), currentPtr->coord(2));

        //找到Goal,返回
        if (currentPtr->index == end_idx)
        {
            ros::Time time_end = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*] succeed, Time consumed in Astar path finding is %f ms, path cost is %f m", (time_end - time_start).toSec() * 1000.0, currentPtr->gScore * resolution);
            return;
        }

        //得到当前节点的neighbor
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        //处理发现的neighbor
        for (int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            neighborPtr = neighborPtrSets[i];

            std::multimap<double, GridNodePtr>::iterator itFoundSame = openSet.begin();
            for (itFoundSame = openSet.begin(); itFoundSame != openSet.end(); itFoundSame++)
            {
                if (neighborPtr->index(0) == itFoundSame->second->index(0)  //openset中已经存在此neighbor
                && neighborPtr->index(1) == itFoundSame->second->index(1) 
                && neighborPtr->index(2) == itFoundSame->second->index(2))
                    break;
            }

            if (itFoundSame == openSet.end())   //openSet中没有该neighbor
            {
                neighborPtr->id = 1;   //标记该节点为未访问状态
                // ROS_INFO("neighborPtr -> fScore = %f", neighborPtr -> fScore);
                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
            }
            else    //该节点已经在openSet内
            {
                if (itFoundSame->second->gScore > neighborPtr->gScore)   //openSet中的g(n) > 当前路线的g(n), 需要更新
                {
                    itFoundSame->second->cameFrom = currentPtr;
                    itFoundSame->second->gScore = neighborPtr->gScore;
                    itFoundSame->second->fScore = neighborPtr->fScore;
                    delete neighborPtr;    //若节点已经在openSet内,更新openSet中的节点内容后,删除内容相同的多余内存
                }
            }
        }
    }

    //搜索失败
    ros::Time time_end = ros::Time::now();
    if ((time_end - time_start).toSec() > 0.1)
        ROS_WARN("[A*] failed, Time consumed in Astar path finding is %f", (time_end - time_start).toSec());
}

/*
* @function     AstarPathFinder::getPath
* @brief        回溯获得最优路径
* @param        None
* @return       vector<Vector3d> path     最优路径
*/
vector<Vector3d> AstarPathFinder::getPath()
{
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    GridNodePtr currentPtr = terminatePtr;
    while (currentPtr != startPtr)
    {
        path.push_back(currentPtr->coord);
        currentPtr = currentPtr->cameFrom;    //回溯
    }
    // path.push_back(currentPtr->coord);

    reverse(path.begin(), path.end());

    return path;
}