#include "JPS_searcher.h"

using namespace std;
using namespace Eigen;

void JPSPathFinder::JPSGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();

    //起点和终点的索引向量
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //起点和终点的位置向量
    start_pt = gridIndex2coord(start_idx);
    end_pt = gridIndex2coord(end_idx);

    //初始化起点和终点的节点(指针)
    GridNodePtr startPtr = GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap[end_idx(0)][end_idx(1)][end_idx(2)];

    openSet.clear();  //使用STL库中的multimap维护openset

    GridNodePtr currentPtr = NULL;  // currentPtr代表openset中f(n)最小的节点
    GridNodePtr neighborPtr = NULL;

    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->id = 1;   //标记起始节点为未访问
    openSet.insert(make_pair(startPtr->fScore, startPtr));  //将起始节点加入openset

    double tentative_gScore;
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
                break;
            }
        }

        //找到Goal,返回
        if (currentPtr->index == goalIdx)
        {
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[JPS] succeed, Time consumed in JPS path finding is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution);
            return;
        }

        //得到当前节点的继承
        JPSGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        //处理发现的neighbor
        for (int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
        
            neighborPtr = neighborPtrSets[i];
            if (neighborPtr->id == 0)   //发现一个新节点
            { 
                neighborPtr->id = 1;    //标记该节点为未访问状态
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                continue;
            }
            else if(neighborPtr->id == 1)   //该节点已经在openSet内
            {
                tentative_gScore = currentPtr->gScore + edgeCostSets[i];

                if(tentative_gScore <= neighborPtr->gScore)   //当前路线的g(n) <= openSet中的g(n), 需要更新
                
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                    neighborPtr->cameFrom = currentPtr;
                    for (int i = 0; i < 3; i++)   //计算新的扩展方向 
                    {
                        neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i);
                        if (neighborPtr->dir(i) != 0)
                            neighborPtr->dir(i) /= abs(neighborPtr->dir(i));
                    }
            }

        }
    }
    
    //搜索失败
    ros::Time time_2 = ros::Time::now();
    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("[JPS] failed, Time consume in JPS path finding is %f", (time_2 - time_1).toSec());
}

inline void JPSPathFinder::JPSGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets, vector<double> &edgeCostSets)
{
    neighborPtrSets.clear();
    edgeCostSets.clear();
    const int norm1 = abs(currentPtr->dir(0)) + abs(currentPtr->dir(1)) + abs(currentPtr->dir(2));

    int num_neib = jn3d->nsz[norm1][0];
    int num_fneib = jn3d->nsz[norm1][1];
    int id = (currentPtr->dir(0) + 1) + 3 * (currentPtr->dir(1) + 1) + 9 * (currentPtr->dir(2) + 1);

    for (int dev = 0; dev < num_neib + num_fneib; ++dev)
    {
        Vector3i neighborIdx;
        Vector3i expandDir;

        if (dev < num_neib)
        {
            expandDir(0) = jn3d->ns[id][0][dev];
            expandDir(1) = jn3d->ns[id][1][dev];
            expandDir(2) = jn3d->ns[id][2][dev];

            if (!jump(currentPtr->index, expandDir, neighborIdx))
                continue;
        }
        else
        {
            int nx = currentPtr->index(0) + jn3d->f1[id][0][dev - num_neib];
            int ny = currentPtr->index(1) + jn3d->f1[id][1][dev - num_neib];
            int nz = currentPtr->index(2) + jn3d->f1[id][2][dev - num_neib];

            if (isOccupied(nx, ny, nz))
            {
                expandDir(0) = jn3d->f2[id][0][dev - num_neib];
                expandDir(1) = jn3d->f2[id][1][dev - num_neib];
                expandDir(2) = jn3d->f2[id][2][dev - num_neib];

                if (!jump(currentPtr->index, expandDir, neighborIdx))
                    continue;
            }
            else
                continue;
        }

        GridNodePtr nodePtr = GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
        nodePtr->dir = expandDir;

        neighborPtrSets.push_back(nodePtr);
        edgeCostSets.push_back(     //当前节点与邻居节点之间的cost
            sqrt(
                (neighborIdx(0) - currentPtr->index(0)) * (neighborIdx(0) - currentPtr->index(0)) +
                (neighborIdx(1) - currentPtr->index(1)) * (neighborIdx(1) - currentPtr->index(1)) +
                (neighborIdx(2) - currentPtr->index(2)) * (neighborIdx(2) - currentPtr->index(2))));
    }
}

bool JPSPathFinder::jump(const Vector3i &curIdx, const Vector3i &expDir, Vector3i &neiIdx)
{
    neiIdx = curIdx + expDir;

    if (!isFree(neiIdx))
        return false;

    if (neiIdx == goalIdx)
        return true;

    if (hasForced(neiIdx, expDir))
        return true;

    const int id = (expDir(0) + 1) + 3 * (expDir(1) + 1) + 9 * (expDir(2) + 1);
    const int norm1 = abs(expDir(0)) + abs(expDir(1)) + abs(expDir(2));
    int num_neib = jn3d->nsz[norm1][0];

    for (int k = 0; k < num_neib - 1; ++k)
    {
        Vector3i newNeiIdx;
        Vector3i newDir(jn3d->ns[id][0][k], jn3d->ns[id][1][k], jn3d->ns[id][2][k]);
        if (jump(neiIdx, newDir, newNeiIdx))
            return true;
    }

    return jump(neiIdx, expDir, neiIdx);
}

inline bool JPSPathFinder::hasForced(const Vector3i &idx, const Vector3i &dir)
{
    int norm1 = abs(dir(0)) + abs(dir(1)) + abs(dir(2));
    int id = (dir(0) + 1) + 3 * (dir(1) + 1) + 9 * (dir(2) + 1);

    switch (norm1)
    {
    case 1:
        // 1-d move, check 8 neighbors
        for (int fn = 0; fn < 8; ++fn)
        {
            int nx = idx(0) + jn3d->f1[id][0][fn];
            int ny = idx(1) + jn3d->f1[id][1][fn];
            int nz = idx(2) + jn3d->f1[id][2][fn];
            if (isOccupied(nx, ny, nz))
                return true;
        }
        return false;

    case 2:
        // 2-d move, check 8 neighbors
        for (int fn = 0; fn < 8; ++fn)
        {
            int nx = idx(0) + jn3d->f1[id][0][fn];
            int ny = idx(1) + jn3d->f1[id][1][fn];
            int nz = idx(2) + jn3d->f1[id][2][fn];
            if (isOccupied(nx, ny, nz))
                return true;
        }
        return false;

    case 3:
        // 3-d move, check 6 neighbors
        for (int fn = 0; fn < 6; ++fn)
        {
            int nx = idx(0) + jn3d->f1[id][0][fn];
            int ny = idx(1) + jn3d->f1[id][1][fn];
            int nz = idx(2) + jn3d->f1[id][2][fn];
            if (isOccupied(nx, ny, nz))
                return true;
        }
        return false;

    default:
        return false;
    }
}

inline bool JPSPathFinder::isOccupied(const Eigen::Vector3i &index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool JPSPathFinder::isFree(const Eigen::Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool JPSPathFinder::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool JPSPathFinder::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}
