//
// Created by Admin on 16/04/2025.
//

// Node.h
#ifndef NODE_H
#define NODE_H

#include <string>

class Node {
public:
    int id;                // ID của nút
    std::string string_id; // StringID (D0, S0, C1,...)
    std::string type;      // Loại nút: Depot, Station, Customer
    double x, y;           // Tọa độ
    double demand;         // Không dùng trong ETSP-WCL
    double ready_time;     // Không dùng nếu không có cửa sổ thời gian
    double due_date;       // Không dùng nếu không có cửa sổ thời gian
    double service_time;   // Thời gian phục vụ (τ_i)
};

#endif // ETSP_WCL_OPTIMIZATION_NODE_H
