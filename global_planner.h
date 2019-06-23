//
// Created by patrolrobot on 19-6-6.
//

#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_H

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <memory.h>

#define MAXVEX   20
#define INFI     65535

typedef int nodeType;
typedef nodeType Patharc[MAXVEX][MAXVEX];
typedef double ShortPathTable[MAXVEX][MAXVEX];

//构造巡检点与路径关键点的图结构
typedef struct
{
    nodeType vexs[MAXVEX];
    double arc[MAXVEX][MAXVEX];
    int numVertexes, numEdges;
}MGraph;

//用于对输入的巡检点进行先后排序的结构体
typedef struct
{
    double x,y;
    double value;
    nodeType index;
}node,nodes;

namespace global_planner{
    class Floyd{
    public:
        Floyd();
        ~Floyd();

        void CreateMGraph(MGraph *G);
        void ShortestPath_Floyd(MGraph G, Patharc *P, ShortPathTable *D);
        double calDistance(node a, node b);
        std::vector<nodeType> generatePath(const unsigned int &num, const std::vector<int> &id);
        void permulation(std::vector< std::vector<nodeType> > &res, std::vector<nodeType> &patrolPoints, unsigned int index);   //求巡检点所有可能排列
        double calSumDistance(const std::vector<nodeType> &path, ShortPathTable &D);        //求总权值
        int findMinIndex(double nums[], int len);              //找到数组中最小值的下标
        void showPath(const std::vector<nodeType> &path);

    private:
        int numVertexes, numEdges;
        std::vector< std::vector<nodeType> > perm;
    };
};

#endif //GLOBAL_PLANNER_GLOBAL_PLANNER_H
