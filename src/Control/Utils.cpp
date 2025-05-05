//
// Created by Admin on 29/04/2025.
//

#include "../../include/Utils.h"
#include <set>
#include <algorithm>
#include <iomanip>
#include <utility>
#include <iostream>
#include <limits>

bool isCustomer(int id, const std::vector<Node>& nodes) {
    for (const auto& node : nodes) {
        if (node.id == id && node.type == "c") {
            return true;
        }
    }
    return false;
}

bool can_reach(int i, int j, double SOC, const std::vector<std::vector<Arc>>& arc_matrix, const Params& params) {
    const Arc& arc = arc_matrix[i][j];
    if (!arc.is_wireless) {
        return SOC - params.h * arc.dij >= 0;
    } else {
        double max_energy_gained = arc.beta_ij * (arc.dij / arc.U_min);
        return SOC - params.h * arc.dij + max_energy_gained >= 0;
    }
}

double update_SOC(int i, int j, double SOC, const std::vector<std::vector<Arc>>& arc_matrix, const Params& params) {
    const Arc& arc = arc_matrix[i][j];
    if (!arc.is_wireless) {
        return SOC - params.h * arc.dij;
    } else {
        double energy_gained = arc.beta_ij * (arc.dij / arc.U_min);
        return SOC - params.h * arc.dij + energy_gained;
    }
}
std::vector<int> generateInitialRoute(const std::vector<Node>& nodes,
                                      const std::vector<Arc>& arcs,
                                      const Params& params,
                                      std::mt19937& gen,
                                      double insert_prob) {
    // Get list of customers
    std::vector<int> customers;
    for (const auto& node : nodes) {
        if (node.type == "c") {
            customers.push_back(node.id);
        }
    }
    if (customers.empty()) {
        std::cerr << "No customers found in nodes.\n";
        return {};
    }

    // Shuffle customer list to create a random route
    std::shuffle(customers.begin(), customers.end(), gen);

    // Initialize route with starting depot, customers, and ending depot
    std::vector<int> route = {0}; // Starting depot
    route.insert(route.end(), customers.begin(), customers.end());
    route.push_back(0); // Ending depot (same as starting depot)

    return route;
}

bool isChargingStation(int nodeId, const std::vector<Node>& nodes) {
    for (const auto& node : nodes) {
        if (node.id == nodeId && node.type == "f") {
            return true;
        }
    }
    return false;
}


