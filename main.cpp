#include <iostream>
#include "global_planner.h"

using namespace global_planner;

int main() {
    Floyd path_planner;
    std::vector<nodeType> id;
    id = {2, 9, 5, 7};//指定巡检点的id
    std::vector<nodeType> path;
    path = path_planner.generatePath(id.size(), id);
    path_planner.showPath(path);
    return 0;
}
