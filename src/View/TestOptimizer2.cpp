#include "../../include/CSVReader.h"
#include "../../include/Route.h"
#include "../../include/Optimizer.h"
#include "../../include/Utils.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <random>


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
        Params params = CSVReader::readParams("D:\\Work\\NEULab\\ESTP-WCL-Test5\\data\\Input\\params.csv");
        std::vector<Node> nodes = CSVReader::readNodes("D:\\Work\\NEULab\\ESTP-WCL-Test5\\data\\Input\\nodes.csv");
        std::vector<Arc> arcs = CSVReader::generateArcs(nodes,
            "D:\\Work\\NEULab\\ESTP-WCL-Test5\\data\\Input\\wireless_arcs.csv", params);
        std::vector<std::vector<ChargingOption>> charge_options = CSVReader::readChargingOptions(nodes,
            "D:\\Work\\NEULab\\ESTP-WCL-Test5\\data\\Input\\charging_options.csv");

        printParamsInfo(params);
        printNodesInfo(nodes);
        printArcsInfo(arcs);
        printChargingOptionsInfo(charge_options);

        int n = 10; // Number of routes to generate

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
