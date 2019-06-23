//
// Created by patrolrobot on 19-6-6.
//

#include "global_planner.h"
namespace global_planner{

    Floyd::Floyd():
    numEdges(13), numVertexes(12)
    {

    }

    Floyd::~Floyd(){

    }

    //初始化图中节点信息{x,y,w,id}
    nodes point[MAXVEX] = {
            {0,0,0,0},{1.5,0,0,1},{3,0,0,2},{4.5,0,0,3},
            {4.5,1.2,0,4},{3,1.2,0,5},{1.5,1.2,0,6},{0,1.2,0,7},
            {0,2.4,0,8},{1.5,2.4,0,9},{3,2.4,0,10},{4.5,2.4,0,11}
    };

    void swap(int &a, int &b)
    {
        int tmp = a;
        a = b;
        b = tmp;
    }

    double Floyd::calDistance(node a, node b)
    {
        double distance;
        distance = sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2));
        return distance;
    }

    void Floyd::CreateMGraph(MGraph *G)
    {
        int i,j;

        for(i = 0; i < numVertexes; i++)
        {
            G->vexs[i] = i;
        }

        for(i = 0; i < numVertexes; i++)
        {
            for(j = 0; j < numVertexes; j++)
            {
                if(i == j)
                    G->arc[i][j] = 0;
                else
                    G->arc[i][j] = G->arc[j][i] = INFI;
            }
        }

        G->arc[0][1] = calDistance(point[0], point[1]);
        G->arc[1][2] = calDistance(point[1], point[2]);
        G->arc[2][3] = calDistance(point[2], point[3]);
        G->arc[3][4] = calDistance(point[3], point[4]);
        G->arc[4][5] = calDistance(point[4], point[5]);
        G->arc[5][6] = calDistance(point[5], point[6]);
        G->arc[6][7] = calDistance(point[6], point[7]);
        G->arc[0][7] = calDistance(point[0], point[7]);
        G->arc[7][8] = calDistance(point[7], point[8]);
        G->arc[8][9] = calDistance(point[8], point[9]);
        G->arc[9][10] = calDistance(point[9], point[10]);
        G->arc[10][11] = calDistance(point[10], point[11]);
        G->arc[11][4] = calDistance(point[11], point[4]);


        for(i = 0; i < numVertexes; i++)
        {
            for(j = i; j < numVertexes; j++)
            {
                G->arc[j][i] = G->arc[i][j];
            }
        }
    }

    void Floyd::ShortestPath_Floyd(MGraph G, Patharc *P, ShortPathTable *D)
    {
        nodeType v,w,k;

        for(v = 0; v < numVertexes; ++v)      //初始化D与P
        {
            for(w = 0; w < numVertexes; ++w)
            {
                (*D)[v][w] = G.arc[v][w];     //D[v][w]值即为对应点间的权值
                (*P)[v][w] = w;               //初始化P
            }
        }

        for(k = 0; k < numVertexes; ++k)
        {
            for(v = 0; v < numVertexes; ++v)
            {
                for(w = 0; w < numVertexes; ++w)
                {
                    if((*D)[v][w] > (*D)[v][k] + (*D)[k][w])    /*如果经过下标为k顶点路径比原两点间路径更短，将当前两点间的权值设为最小的一个*/
                    {
                        (*D)[v][w] = (*D)[v][k] + (*D)[k][w];   /*将当前两点间的权值设为更小的一个*/
                        (*P)[v][w] = (*P)[v][k];                /*路径设置经过下标为k的顶点*/
                    }
                }
            }
        }
    }


    //找出数组最小值的下标
    int Floyd::findMinIndex(double nums[], int len)
    {
        int pos = 0;
        double min = nums[0];
        for(unsigned int i = 1; i < len; ++i){
            if(min > nums[i]){
                min = nums[i];
                pos = i;
            }
        }
        return pos;
    }

    //计算总权值
    double Floyd::calSumDistance(const std::vector<nodeType> &path, ShortPathTable &D)
    {
        double sum = 0;
        for(unsigned int i = 0; i < path.size() - 1; ++i){
            sum += D[path[i]][path[i+1]];
        }
        return sum;
    }

    //求巡检点所有可能排列
    void Floyd::permulation(std::vector< std::vector<nodeType> > &res, std::vector<nodeType> &patrolPoints, unsigned int index)
    {
        if(index >= patrolPoints.size()){
            res.push_back(patrolPoints);
            return;
        }

        for(unsigned int i = index; i < patrolPoints.size(); ++i){
            swap(patrolPoints[i], patrolPoints[index]);
            permulation(res, patrolPoints, index+1);
            swap(patrolPoints[index], patrolPoints[i]);
        }
    }

    //生成路径
    std::vector<nodeType> Floyd::generatePath(const unsigned int &num, const std::vector<int> &id)  //num为目标点个数，id为目标点id
    {
        nodeType v, k;
        v = 0;             //设置初始点

        MGraph G;
        Patharc P;
        ShortPathTable D;

        CreateMGraph(&G);
        ShortestPath_Floyd(G, &P, &D);

        //求指定巡检点所有可能排列
        perm.clear();
        std::vector<int> patrol_points(id);
        permulation(perm, patrol_points, 0);

        //生成路径
        std::vector< std::vector<nodeType> > path(perm.size());
        path.clear();
        double *sumDistance = new double[perm.size()];       //声明总权值数组
        memset(sumDistance, 0, perm.size()*sizeof(sumDistance));  //初始化总权值数组为0
        for(unsigned int i = 0; i < perm.size(); ++i){
            //对第一个巡检点规划
            path[i].push_back(v);
            k = P[v][perm[i][0]];
            while(k != perm[i][0]){
                path[i].push_back(k);
                k = P[k][perm[i][0]];
            }
            path[i].push_back(perm[i][0]);
            //对剩下的点规划
            for(unsigned int j = 0; j < num-1; ++j){
                k = P[perm[i][j]][perm[i][j+1]];
                while(k != perm[i][j+1]){
                    path[i].push_back(k);
                    k = P[k][perm[i][j+1]];
                }
                path[i].push_back(perm[i][j+1]);
            }
            //对返程规划
            k = P[perm[i][num-1]][v];
            while(k != v){
                path[i].push_back(k);
                k = P[k][v];
            }
            path[i].push_back(v);
            sumDistance[i] = calSumDistance(path[i], D);
        }

        //找到总权值最小的那条路径并返回index
        int index = findMinIndex(sumDistance, perm.size());
        delete[] sumDistance;
        std::vector<nodeType> result(path[index]);
        return result;

    }

    void Floyd::showPath(const std::vector<nodeType> &path)
    {
        if(!path.empty()) {
            int cnt = 0;
            printf("path: ");
            for (unsigned int j = 0; j < path.size() - 1; j++) {
                printf("%d->", path[j]);
                cnt++;
            }
            printf("%d\n", path[cnt]);
        }
    }
};

