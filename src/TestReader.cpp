#include <iostream>
#include <iomanip>
#include <vector>
#include <set>
#include <algorithm>
#include <utility> // For std::pair
#include "CSVReader.h"

// Function to print node information
void printNodesInfo(const std::vector<Node>& nodes) {
    std::cout << "\n=== Node Information ===\n";
    std::cout << std::fixed << std::setprecision(2);
    for (const auto& node : nodes) {
        std::cout << "Node ID: " << node.id
                  << ", StringID: " << node.string_id
                  << ", Type: " << node.type
                  << ", x: " << node.x
                  << ", y: " << node.y
                  << ", ServiceTime: " << node.service_time << "\n";
    }
}

// Function to print arc information
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

// Function to print charging option information
void printChargingOptionsInfo(const std::vector<std::vector<ChargingOption>>& options) {
    std::cout << "\n=== Charging Options Information ===\n";
    std::cout << std::fixed << std::setprecision(2);
    for (size_t i = 0; i < options.size(); ++i) {
        if (!options[i].empty()) {
            std::cout << "Node ID: " << i << " has " << options[i].size() << " options:\n";
            for (const auto& opt : options[i]) {
                std::cout << "  Option: " << opt.option
                          << ", Rate: " << opt.rate
                          << ", Cost: " << opt.cost << "\n";
            }
        }
    }
}

// Function to print parameter information
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

// Helper function: Check if we can reach node j from i with current SOC
bool can_reach(int i, int j, double SOC, const std::vector<std::vector<Arc>>& arc_matrix, const Params& params) {
    const Arc& arc = arc_matrix[i][j];
    if (!arc.is_wireless) {
        return SOC - params.h * arc.dij >= 0;
    } else {
        double max_energy_gained = arc.beta_ij * (arc.dij / arc.U_min);
        return SOC - params.h * arc.dij + max_energy_gained >= 0;
    }
}

// Helper function: Update SOC after moving from i to j
double update_SOC(int i, int j, double SOC, const std::vector<std::vector<Arc>>& arc_matrix, const Params& params) {
    const Arc& arc = arc_matrix[i][j];
    if (!arc.is_wireless) {
        return SOC - params.h * arc.dij;
    } else {
        double energy_gained = arc.beta_ij * (arc.dij / arc.U_min); // Maximize energy gain
        return SOC - params.h * arc.dij + energy_gained;
    }
}

// Function to generate an initial route using nearest neighbor with SOC checks
std::vector<int> generateInitialRoute(const std::vector<Node>& nodes, const std::vector<Arc>& arcs, const Params& params) {
    int n = nodes.size();

    // Create arc matrix for quick access
    std::vector<std::vector<Arc>> arc_matrix(n, std::vector<Arc>(n));
    for (const auto& arc : arcs) {
        arc_matrix[arc.from][arc.to] = arc;
    }

    // Identify depot, customers, and charging stations
    int depot = 0; // Assume node 0 is the depot
    std::vector<int> customers;
    std::vector<int> charging_stations;
    for (const auto& node : nodes) {
        if (node.type == "c") customers.push_back(node.id); // Assume type 1 is customer
        else if (node.type == "f") charging_stations.push_back(node.id); // Assume type 2 is charging station
    }

    // Initialize route
    std::vector<int> route = {depot};
    std::set<int> visited = {depot};
    double current_SOC = params.initial_SOC;
    int current = depot;
    std::set<int> unvisited_customers(customers.begin(), customers.end());

    while (!unvisited_customers.empty()) {
        bool found = false;

        // Find feasible customers
        std::vector<std::pair<double, int>> feasible_customers;
        for (int j : unvisited_customers) {
            if (can_reach(current, j, current_SOC, arc_matrix, params)) {
                double dist = arc_matrix[current][j].dij;
                feasible_customers.push_back({dist, j});
            }
        }

        if (!feasible_customers.empty()) {
            // Sort by distance
            std::sort(feasible_customers.begin(), feasible_customers.end());
            int next = feasible_customers[0].second;
            route.push_back(next);
            visited.insert(next);
            unvisited_customers.erase(next);
            current_SOC = update_SOC(current, next, current_SOC, arc_matrix, params);
            current = next;
            found = true;
        }

        if (!found) {
            // Go to nearest feasible charging station
            std::vector<std::pair<double, int>> feasible_stations;
            for (int k : charging_stations) {
                if (can_reach(current, k, current_SOC, arc_matrix, params)) {
                    double dist = arc_matrix[current][k].dij;
                    feasible_stations.push_back({dist, k});
                }
            }

            if (!feasible_stations.empty()) {
                std::sort(feasible_stations.begin(), feasible_stations.end());
                int next = feasible_stations[0].second;
                route.push_back(next);
                current_SOC = params.Q; // Charge to full
                current = next;
            } else {
                // Cannot reach any station (problem)
                std::cout << "Cannot reach any charging station from " << current << " with SOC " << current_SOC << "\n";
                break;
            }
        }
    }

    // Return to depot
    if (can_reach(current, depot, current_SOC, arc_matrix, params)) {
        route.push_back(depot);
    } else {
        // Need to charge first
        std::vector<std::pair<double, int>> feasible_stations;
        for (int k : charging_stations) {
            if (can_reach(current, k, current_SOC, arc_matrix, params)) {
                double dist = arc_matrix[current][k].dij;
                feasible_stations.push_back({dist, k});
            }
        }

        if (!feasible_stations.empty()) {
            std::sort(feasible_stations.begin(), feasible_stations.end());
            int k = feasible_stations[0].second;
            route.push_back(k);
            current_SOC = params.Q; // Charge to full
            current = k;
        }

        // Now go to depot
        route.push_back(depot);
    }

    return route;
}

int main() {
    std::cout << "Testing CSVReader for ETSP-WCL...\n";

    try {
        // Read and print parameter information
        Params params = CSVReader::readParams("D:\\Work\\NEULab\\ESTP-WCL-Test1\\Test1\\data\\Input\\params.csv");
        printParamsInfo(params);

        // Read and print node information
        std::vector<Node> nodes = CSVReader::readNodes("D:\\Work\\NEULab\\ESTP-WCL-Test1\\Test1\\data\\Input\\nodes.csv");
        printNodesInfo(nodes);

        // Read and print arc information
        std::vector<Arc> arcs = CSVReader::generateArcs(nodes, "D:\\Work\\NEULab\\ESTP-WCL-Test1\\Test1\\data\\Input\\wireless_arcs.csv", params);
        printArcsInfo(arcs);

        // Read and print charging option information
        std::vector<std::vector<ChargingOption>> charge_options = CSVReader::readChargingOptions(nodes, "D:\\Work\\NEULab\\ESTP-WCL-Test1\\Test1\\data\\Input\\charging_options.csv");
        printChargingOptionsInfo(charge_options);

        // Generate and print initial route
        std::vector<int> initial_route = generateInitialRoute(nodes, arcs, params);
        std::cout << "Initial Route: ";
        for (size_t i = 0; i < initial_route.size(); ++i) {
            std::cout << initial_route[i];
            if (i < initial_route.size() - 1) std::cout << " -> ";
        }
        std::cout << "\n";

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "\nTesting completed. Please verify the results.\n";
    return 0;
}