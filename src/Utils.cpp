#include "Node.h"

#include <fstream>
#include <sstream>
#include <vector>


// Ví dụ: đọc dữ liệu node từ file
std::vector<Node> readNodesFromFile(const std::string &filename) {
    std::vector<Node> nodes;
    std::ifstream file(filename);
    std::string line;
    while (getline(file, line)) {
        std::istringstream iss(line);
        int id;
        double x, y, serviceTime;
        int typeInt;
        if (iss >> id >> typeInt >> x >> y >> serviceTime) {
            NodeType type = static_cast<NodeType>(typeInt);
            nodes.push_back(Node(id, type, x, y, serviceTime));
        }
    }
    return nodes;
}

// Các hàm tiện ích khác...
