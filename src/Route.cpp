//
// Created by Admin on 23/04/2025.
//

#include "Route.h"
#include "Route.h"
#include "Optimizer.h"
#include <iostream>
#include <iomanip>

// Constructor
Route::Route(const std::vector<int>& initial_nodes)
    : node_ids(initial_nodes), total_cost(0.0), is_feasible(true) {
}

// Tối ưu hóa tuyến đường bằng MILP
double Route::optimize(const std::vector<Arc>& arcs,
                       const std::vector<std::vector<ChargingOption>>& charge_options,
                       const Params& params,
                       const std::vector<Node>& nodes,
                       Optimizer& optimizer) {
    // Gọi hàm optimize của Optimizer để tối ưu hóa tuyến đường
    total_cost = optimizer.optimize(node_ids, arcs, charge_options, params, nodes);

    // Cập nhật tính khả thi dựa trên chi phí
    if (total_cost >= 1e9) { // Giả định chi phí lớn biểu thị không khả thi
        is_feasible = false;
    } else {
        is_feasible = true;
    }

    return total_cost;
}

// In tuyến đường
void Route::print() const {
    std::cout << "Route: ";
    for (size_t i = 0; i < node_ids.size(); ++i) {
        std::cout << node_ids[i];
        if (i < node_ids.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << "\n Cost: " << std::fixed << std::setprecision(2) << total_cost
              << ", feasible: " << (is_feasible ? "Yes" : "No") << "\n";
}