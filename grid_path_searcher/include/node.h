#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"

#define inf 1>>20

struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
    int id;                      //1 -> 未访问, -1 --> 已访问
    Eigen::Vector3d coord; 
    Eigen::Vector3i dir;        //扩展方向,JPS使用
    Eigen::Vector3i index;
	
    double gScore, fScore;
    GridNodePtr cameFrom;
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord)      
    {  
        id = 0;
        index = _index;
        coord = _coord;
        dir   = Eigen::Vector3i::Zero();

        gScore = inf;
        fScore = inf;
        cameFrom = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};


#endif
