//
// Created by Admin on 16/04/2025.
//

#ifndef NODE_H
#define NODE_H

#include <string>

class Node {
public:
    int id;
    std::string type;
    double x, y;
    double demand;
    double ready_time;
    double due_date;
    double service_time;
};

#endif // ETSP_WCL_OPTIMIZATION_NODE_H
