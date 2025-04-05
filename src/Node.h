#ifndef NODE_H
#define NODE_H

enum class NodeType {
    Depot,
    Customer,
    ChargingStation
};

class Node {
public:
    int id;                // ID duy nhất của node
    NodeType type;         // Kiểu node: Depot, Customer, ChargingStation
    double x, y;           // Tọa độ (sử dụng để tính khoảng cách)
    double serviceTime;    // τᵢ: thời gian phục vụ (áp dụng cho Customer hoặc ChargingStation nếu cần)

    Node(int id, NodeType type, double x = 0.0, double y = 0.0, double serviceTime = 0.0);
};

#endif // NODE_H
