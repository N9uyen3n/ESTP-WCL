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
std::vector<int> generateInitialRoute(const std::vector<Node>& nodes, const std::vector<Arc>& arcs, const Params& params, std::mt19937& gen, double insert_prob) {
    int n = nodes.size();
    int starting_depot = 0;
    int ending_depot = 0;

    std::vector<std::vector<Arc>> arc_matrix(n, std::vector<Arc>(n));
    for (const auto& arc : arcs) {
        arc_matrix[arc.from][arc.to] = arc;
    }

    // Get list of customers
    std::vector<int> customers;
    for (const auto& node : nodes) {
        if (node.type == "c") customers.push_back(node.id);
    }
    if (customers.empty()) {
        std::cerr << "No customers found in nodes.\n";
        return {};
    }

    // Shuffle customer list to create a random route
    std::shuffle(customers.begin(), customers.end(), gen);

    // Initialize route
    std::vector<int> route = {starting_depot};
    std::set<int> visited = {starting_depot};
    double current_SOC = params.initial_SOC;
    int current = starting_depot;

    // Visit all customers in random order
    for (int next : customers) {
        // With a probability of insert_prob, insert a random charging station if possible
        if (std::uniform_real_distribution<>(0.0, 1.0)(gen) < insert_prob) {
            std::vector<int> charging_stations;
            for (const auto& node : nodes) {
                if (node.type == "f") charging_stations.push_back(node.id);
            }
            if (!charging_stations.empty()) {
                std::uniform_int_distribution<> dis(0, charging_stations.size() - 1);
                int station = charging_stations[dis(gen)];
                if (can_reach(current, station, current_SOC, arc_matrix, params)) {
                    route.push_back(station);
                    current_SOC = params.Q; // Fully recharge at the station
                    current = station;
                }
            }
        }

        // Add customer if reachable
        if (can_reach(current, next, current_SOC, arc_matrix, params)) {
            route.push_back(next);
            visited.insert(next);
            current_SOC = update_SOC(current, next, current_SOC, arc_matrix, params);
            current = next;
        } else {
            // If the customer is unreachable, insert the nearest feasible charging station
            std::vector<std::pair<double, int>> feasible_stations;
            for (const auto& node : nodes) {
                if (node.type == "f" && can_reach(current, node.id, current_SOC, arc_matrix, params)) {
                    double dist = arc_matrix[current][node.id].dij;
                    feasible_stations.push_back({dist, node.id});
                }
            }
            if (!feasible_stations.empty()) {
                std::sort(feasible_stations.begin(), feasible_stations.end());
                int station = feasible_stations[0].second;
                route.push_back(station);
                current_SOC = params.Q;
                current = station;
                // Retry reaching the customer
                if (can_reach(current, next, current_SOC, arc_matrix, params)) {
                    route.push_back(next);
                    visited.insert(next);
                    current_SOC = update_SOC(current, next, current_SOC, arc_matrix, params);
                    current = next;
                } else {
                    std::cerr << "Cannot reach customer " << next << " from " << current << "\n";
                    return {};
                }
            } else {
                std::cerr << "Cannot reach any charging station from " << current << "\n";
                return {};
            }
        }
    }

    // Return to ending depot
    while (true) {
        if (can_reach(current, ending_depot, current_SOC, arc_matrix, params)) {
            route.push_back(ending_depot);
            break;
        } else {
            std::vector<std::pair<double, int>> feasible_stations;
            for (const auto& node : nodes) {
                if (node.type == "f" && can_reach(current, node.id, current_SOC, arc_matrix, params)) {
                    double dist = arc_matrix[current][node.id].dij;
                    feasible_stations.push_back({dist, node.id});
                }
            }
            if (!feasible_stations.empty()) {
                std::sort(feasible_stations.begin(), feasible_stations.end());
                int station = feasible_stations[0].second;
                route.push_back(station);
                current_SOC = params.Q;
                current = station;
            } else {
                std::cerr << "Cannot reach depot from " << current << "\n";
                return {};
            }
        }
    }

    return route;
}


