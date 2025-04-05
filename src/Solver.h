#ifndef SOLVER_H
#define SOLVER_H

#include "Graph.h"
#include "Solution.h"  // Bạn cần tạo thêm file Solution.h để định nghĩa cấu trúc lời giải

class Solver {
public:
    virtual ~Solver() {}
    virtual Solution solve(const Graph &graph) = 0;
};

#endif // SOLVER_H
