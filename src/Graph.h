#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include "Node.h"
#include "Arc.h"

class Graph {
public:
    std::vector<Node> nodes;
    std::vector<Arc> arcs;

    // Các tập hợp đặc biệt có thể được trích xuất từ nodes:
    // Ví dụ: customerNodes, chargingStations, depots, v.v.
    Graph();

    // Hàm thêm node và arc
    void addNode(const Node &node);
    void addArc(const Arc &arc);

    // Hàm tính toán khoảng cách giữa các node (nếu cần)
    double computeDistance(const Node &a, const Node &b) const;

    // Các hàm trợ giúp lấy tập hợp: lấy tất cả các customer, depot, charging station, v.v.
};

#endif // GRAPH_H
