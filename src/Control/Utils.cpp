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

class ChargingOption;

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

void printNodesInfo(const std::vector<Node>& nodes) {
    std::cout << "\n=== Node Information ===\n";
    std::cout << std::fixed << std::setprecision(2);
    for (const auto& node : nodes) {
        std::cout << "ID: " << node.id
                  << ", StringID: " << node.string_id
                  << ", Type: " << node.type
                  << ", x: " << node.x
                  << ", y: " << node.y
                  << ", Service Time: " << node.service_time << "\n";
    }
}

// printArcsInfo function
void printArcsInfo(const std::vector<Arc>& arcs) {
    std::cout << "\n=== Arc Information ===\n";
    std::cout << std::fixed << std::setprecision(2);
    for (const auto& arc : arcs) {
        if (arc.is_wireless) {
            std::cout << "Wireless Arc: From " << arc.from << " to " << arc.to
                      << ", d_ij: " << arc.dij
                      << ", s_ij: " << arc.sij
                      << ", Beta_ij: " << arc.beta_ij
                      << ", U_min: " << arc.U_min
                      << ", U_max: " << arc.U_max << "\n";
        }
    }
}

// printChargingOptionsInfo function
void printChargingOptionsInfo(const std::vector<std::vector<ChargingOption>>& options) {
    std::cout << "\n=== Charging Options Information ===\n";
    std::cout << std::fixed << std::setprecision(2);
    for (size_t i = 0; i < options.size(); ++i) {
        if (!options[i].empty()) {
            std::cout << "Node ID: " << i << " has " << options[i].size() << " options:\n";
            for (const auto& opt : options[i]) {
                std::cout << "  Station ID: " << opt.station_id
                          << ", Option: " << opt.option
                          << ", Rate: " << opt.rate
                          << ", Cost: " << opt.cost << "\n";
            }
        }
    }
}

// printParamsInfo function
void printParamsInfo(const Params& params) {
    std::cout << "\n=== Parameter Information ===\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Q: " << params.Q << "\n";
    std::cout << "h: " << params.h << "\n";
    std::cout << "v: " << params.v << "\n";
    std::cout << "cw: " << params.cw << "\n";
    std::cout << "ct: " << params.ct << "\n";
    std::cout << "minSOC: " << params.minSOC << "\n";
    std::cout << "initial_SOC: " << params.initial_SOC << "\n";
    std::cout << "M: " << params.M << "\n";
}

// printModelParameters function
void printModelParameters(const ModelParameters& params, const std::vector<int>& route) {
    std::cout << "=== Model Parameters ===\n";
    std::cout << std::fixed << std::setprecision(2);

    std::cout << "Charging Time (phi):\n";
    for (size_t i = 0; i < params.phi.size(); ++i) {
        int node_id = (route.empty() || i >= route.size()) ? static_cast<int>(i) : route[i];
        std::cout << "Node " << node_id << ": " << params.phi[i] << "\n";
    }

    std::cout << "\nCharging Option Selection (w):\n";
    for (size_t i = 0; i < params.w.size(); ++i) {
        if (!params.w[i].empty()) {
            int node_id = (route.empty() || i >= route.size()) ? static_cast<int>(i) : route[i];
            std::cout << "Node " << node_id << ":\n";
            for (size_t j = 0; j < params.w[i].size(); ++j) {
                std::cout << "  Option " << j << ": " << params.w[i][j] << "\n";
            }
        }
    }

    std::cout << "\nTravel Time (s):\n";
    for (size_t k = 0; k < params.s.size(); ++k) {
        if (route.empty() || k + 1 >= route.size()) {
            std::cout << "Arc " << k << ": " << params.s[k] << "\n";
        } else {
            std::cout << "Arc (" << route[k] << " -> " << route[k + 1] << "): " << params.s[k] << "\n";
        }
    }

    std::cout << "\nSOC on Arrival (ya):\n";
    for (size_t i = 0; i < params.ya.size(); ++i) {
        int node_id = (route.empty() || i >= route.size()) ? static_cast<int>(i) : route[i];
        std::cout << "Node " << node_id << ": " << params.ya[i] << "\n";
    }

    std::cout << "\nSOC on Departure (yd):\n";
    for (size_t i = 0; i < params.yd.size(); ++i) {
        int node_id = (route.empty() || i >= route.size()) ? static_cast<int>(i) : route[i];
        std::cout << "Node " << node_id << ": " << params.yd[i] << "\n";
    }

    std::cout << "\nArrival Time (t):\n";
    for (size_t i = 0; i < params.t.size(); ++i) {
        int node_id = (route.empty() || i >= route.size()) ? static_cast<int>(i) : route[i];
        std::cout << "Node " << node_id << ": " << params.t[i] << "\n";
    }

    std::cout << "\nDeparture Time (depart):\n";
    for (size_t i = 0; i < params.depart.size(); ++i) {
        int node_id = (route.empty() || i >= route.size()) ? static_cast<int>(i) : route[i];
        std::cout << "Node " << node_id << ": " << params.depart[i] << "\n";
    }

    std::cout << "\nWireless Charging Selection (z):\n";
    for (size_t l = 0; l < params.z.size(); ++l) {
        std::cout << "Wireless Arc " << l << ": " << params.z[l] << "\n";
    }

    std::cout << "\nLinearized Wireless Charging (w_s_z):\n";
    for (size_t l = 0; l < params.w_s_z.size(); ++l) {
        std::cout << "Wireless Arc " << l << ": " << params.w_s_z[l] << "\n";
    }
}
// clearModelParameters function clears the model parameters
void clearModelParameters(ModelParameters& params) {
    params.phi.clear();
    params.w.clear();
    params.s.clear();
    params.ya.clear();
    params.yd.clear();
    params.t.clear();
    params.depart.clear();
    params.z.clear();
    params.w_s_z.clear();
}
