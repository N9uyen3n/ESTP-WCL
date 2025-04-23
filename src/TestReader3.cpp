#include <iostream>
#include <iomanip>
#include <vector>
#include <set>
#include <algorithm>
#include <utility>
#include <random>
#include "CSVReader.h"
#include "Route.h"
#include "Optimizer.h"

// Function to print node information
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

// Function to print charging options
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

// Check if node j can be reached from i with current SOC
bool can_reach(int i, int j, double SOC, const std::vector<std::vector<Arc>>& arc_matrix, const Params& params) {
    const Arc& arc = arc_matrix[i][j];
    if (!arc.is_wireless) {
        return SOC - params.h * arc.dij >= 0;
    } else {
        double max_energy_gained = arc.beta_ij * (arc.dij / arc.U_min);
        return SOC - params.h * arc.dij + max_energy_gained >= 0;
    }
}

// Update SOC after moving from i to j
double update_SOC(int i, int j, double SOC, const std::vector<std::vector<Arc>>& arc_matrix, const Params& params) {
    const Arc& arc = arc_matrix[i][j];
    if (!arc.is_wireless) {
        return SOC - params.h * arc.dij;
    } else {
        double energy_gained = arc.beta_ij * (arc.dij / arc.U_min);
        return SOC - params.h * arc.dij + energy_gained;
    }
}

// Function to generate an initial route with random charging station insertion
std::vector<int> generateInitialRoute(const std::vector<Node>& nodes, const std::vector<Arc>& arcs, const Params& params, std::mt19937& gen) {
    int n = nodes.size();
    int starting_depot = 0;
    int ending_depot = 0;

    std::vector<std::vector<Arc>> arc_matrix(n, std::vector<Arc>(n));
    for (const auto& arc : arcs) {
        arc_matrix[arc.from][arc.to] = arc;
    }

    std::vector<int> customers, charging_stations;
    for (const auto& node : nodes) {
        if (node.type == "c") customers.push_back(node.id);
        else if (node.type == "f") charging_stations.push_back(node.id);
    }

    std::shuffle(customers.begin(), customers.end(), gen);

    std::vector<int> route = {starting_depot};
    std::set<int> visited = {starting_depot};
    double current_SOC = params.initial_SOC;
    int current = starting_depot;
    std::set<int> unvisited_customers(customers.begin(), customers.end());

    std::uniform_real_distribution<> dis(0.0, 1.0);
    const double insert_station_prob = 0.2;

    for (int next : customers) {
        if (!charging_stations.empty() && dis(gen) < insert_station_prob) {
            std::uniform_int_distribution<> station_dis(0, charging_stations.size() - 1);
            int station = charging_stations[station_dis(gen)];
            if (can_reach(current, station, current_SOC, arc_matrix, params)) {
                route.push_back(station);
                current_SOC = params.Q;
                current = station;
            }
        }

        if (can_reach(current, next, current_SOC, arc_matrix, params)) {
            route.push_back(next);
            visited.insert(next);
            unvisited_customers.erase(next);
            current_SOC = update_SOC(current, next, current_SOC, arc_matrix, params);
            current = next;
        } else {
            std::vector<std::pair<double, int>> feasible_stations;
            for (int k : charging_stations) {
                if (can_reach(current, k, current_SOC, arc_matrix, params)) {
                    double dist = arc_matrix[current][k].dij;
                    feasible_stations.push_back({dist, k});
                }
            }
            if (!feasible_stations.empty()) {
                std::sort(feasible_stations.begin(), feasible_stations.end());
                int station = feasible_stations[0].second;
                route.push_back(station);
                current_SOC = params.Q;
                current = station;
                route.push_back(next);
                visited.insert(next);
                unvisited_customers.erase(next);
                current_SOC = update_SOC(current, next, current_SOC, arc_matrix, params);
                current = next;
            } else {
                std::cout << "Cannot reach customer " << next << " from " << current << " with SOC " << current_SOC << "\n";
                break;
            }
        }
    }

    while (true) {
        if (can_reach(current, ending_depot, current_SOC, arc_matrix, params)) {
            route.push_back(ending_depot);
            break;
        } else {
            if (!charging_stations.empty() && dis(gen) < insert_station_prob) {
                std::uniform_int_distribution<> station_dis(0, charging_stations.size() - 1);
                int station = charging_stations[station_dis(gen)];
                if (can_reach(current, station, current_SOC, arc_matrix, params)) {
                    route.push_back(station);
                    current_SOC = params.Q;
                    current = station;
                    continue;
                }
            }

            std::vector<std::pair<double, int>> feasible_stations;
            for (int k : charging_stations) {
                if (can_reach(current, k, current_SOC, arc_matrix, params)) {
                    double dist = arc_matrix[current][k].dij;
                    feasible_stations.push_back({dist, k});
                }
            }
            if (!feasible_stations.empty()) {
                std::sort(feasible_stations.begin(), feasible_stations.end());
                int station = feasible_stations[0].second;
                route.push_back(station);
                current_SOC = params.Q;
                current = station;
            } else {
                std::cout << "Cannot reach the ending depot (ID=0) from " << current << " with SOC " << current_SOC << "\n";
                break;
            }
        }
    }

    return route;
}

// Generate n routes
std::vector<std::vector<int>> generateMultipleRoutes(const std::vector<Node>& nodes, const std::vector<Arc>& arcs, const Params& params, int n) {
    std::vector<std::vector<int>> routes;
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = 0; i < n; ++i) {
        std::vector<int> route = generateInitialRoute(nodes, arcs, params, gen);
        routes.push_back(route);
    }

    return routes;
}

int main() {
    std::cout << "Testing CSVReader for ETSP-WCL...\n";

    try {
        Params params = CSVReader::readParams("D:\\Work\\NEULab\\ESTP-WCL-Test2\\data\\Input\\params.csv");
        std::vector<Node> nodes = CSVReader::readNodes("D:\\Work\\NEULab\\ESTP-WCL-Test2\\data\\Input\\nodes.csv");
        std::vector<Arc> arcs = CSVReader::generateArcs(nodes, "D:\\Work\\NEULab\\ESTP-WCL-Test2\\data\\Input\\wireless_arcs.csv", params);
        std::vector<std::vector<ChargingOption>> charge_options = CSVReader::readChargingOptions(nodes, "D:\\Work\\NEULab\\ESTP-WCL-Test2\\data\\Input\\charging_options.csv");

        printParamsInfo(params);
        printNodesInfo(nodes);
        printArcsInfo(arcs);
        printChargingOptionsInfo(charge_options);

        int n = 5; // Number of routes to generate

        std::vector<std::vector<int>> routes = generateMultipleRoutes(nodes, arcs, params, n);

        for (size_t i = 0; i < routes.size(); ++i) {
            std::cout << "\nRoute " << i + 1 << ": ";
            for (size_t j = 0; j < routes[i].size(); ++j) {
                std::cout << routes[i][j];
                if (j < routes[i].size() - 1) std::cout << " -> ";
            }
            std::cout << "\n";

            Route route(routes[i]);
            Optimizer optimizer;
            double optimized_cost = route.optimize(arcs, charge_options, params, nodes, optimizer);
            std::cout << "Optimized Cost: " << std::fixed << std::setprecision(2) << optimized_cost << "\n";
            route.print();
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "\nTest completed. Please verify the results.\n";
    return 0;
}
