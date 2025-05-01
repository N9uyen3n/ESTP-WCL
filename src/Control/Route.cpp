//
// Created by Admin on 23/04/2025.
//

#include "../../include/Route.h"
#include "../../include/Optimizer.h"
#include <iostream>
#include <iomanip>

// Constructor
Route::Route(const std::vector<int>& initial_nodes)
    : node_ids(initial_nodes), total_cost(0.0), is_feasible(true) {
}

// Optimize the route using MILP
double Route::optimize(const std::vector<Arc>& arcs,
                       const std::vector<std::vector<ChargingOption>>& charge_options,
                       const Params& params,
                       const std::vector<Node>& nodes,
                       Optimizer& optimizer) {
    // Call the optimize function of Optimizer to optimize the route
    total_cost = optimizer.optimize(node_ids, arcs, charge_options, params, nodes);

    // Update feasibility based on cost
    if (total_cost >= 1e9) { // Assuming large cost indicates infeasibility
        is_feasible = false;
    } else {
        is_feasible = true;
    }

    return total_cost;
}

// Print the route
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