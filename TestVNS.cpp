#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <stdexcept>
#include <random>
#include "include/VNS.h"
#include "include/Utils.h"
#include "include/Graph.h"
#include "include/Parameters.h"
#include "include/Route.h"

int main() {
    try {
        std::cout << std::fixed << std::setprecision(2);
        std::string csv_file_dir = "../data/Input/c5-s5/";

        // Load data from CSV files
        std::vector<Node> nodes = Utils::readNodes(csv_file_dir + "nodes.csv");
        Parameters params = Utils::readParams(csv_file_dir + "params.csv");
        std::vector<Arc> arcs = Utils::generateArcs(nodes, csv_file_dir + "wireless_arcs.csv", params);
        Graph graph(nodes, arcs);
        auto charge_options = Utils::readChargingOptions(nodes, csv_file_dir + "charging_options.csv");

        // Print loaded data
        Utils::printNodesInfo(nodes);
        Utils::printParamsInfo(params);
        Utils::printArcsInfo(arcs);
        Utils::printChargingOptionsInfo(charge_options);

        // Create initial route
        std::cout << "\nInitial Route: 0 ";
        std::vector<int> initial_route = {0};
        for (const auto& node : nodes) {
            if (Utils::isCustomer(node.getId(), nodes)) {
                initial_route.push_back(node.getId());
                std::cout << node.getId() << " ";
            }
        }
        std::cout << "0 \n";
        initial_route.push_back(0);

        // Initialize random number generator
        std::random_device rd;
        std::mt19937 rng(rd());

        // Create and run VNS solver
        int max_iterations = 1000;
        VNS vns(initial_route, graph, charge_options, params, max_iterations, rng);
        Route optimized_route = vns.run();

        // Print results
        std::cout << "\nOptimization Results:\n";
        std::cout << "Route: ";
        const auto& final_nodes = optimized_route.getNodeIds();
        for (size_t i = 0; i < final_nodes.size(); ++i) {
            std::cout << final_nodes[i];
            if (i < final_nodes.size() - 1) std::cout << " -> ";
        }
        std::cout << "\nCost: " << optimized_route.getTotalCost();
        std::cout << "\nFeasible: " << (optimized_route.isFeasible() ? "Yes" : "No") << "\n";

        // // Print detailed results if feasible
        // if (optimized_route.isFeasible()) {
        //     printDetailedResults(optimized_route);
        // }

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}

void printDetailedResults(const Route& route) {
    std::cout << "\nDetailed Route Information:\n";

    // Print header
    std::cout << std::left
              << std::setw(10) << "Node"
              << std::setw(15) << "Arrival SOC"
              << std::setw(15) << "Depart SOC"
              << std::setw(15) << "Arr Time"
              << std::setw(15) << "Dep Time"
              << std::setw(20) << "Charging\n"
              << std::string(90, '-') << "\n";

    // Print node details
    const auto& nodes = route.getNodeIds();
    const auto& soc_arr = route.getSocArrival();
    const auto& soc_dep = route.getSocDeparture();
    const auto& arr_time = route.getArrivalTime();
    const auto& dep_time = route.getDepartureTime();
    const auto& charging = route.getChargingDecisions();

    for (size_t i = 0; i < nodes.size(); ++i) {
        std::cout << std::setw(10) << nodes[i]
                  << std::setw(15) << soc_arr[i]
                  << std::setw(15) << soc_dep[i]
                  << std::setw(15) << arr_time[i]
                  << std::setw(15) << dep_time[i];

        // Find charging decision for this node
        auto it = std::find_if(charging.begin(), charging.end(),
            [&](const ChargingDecision& d) { return d.station_id == nodes[i]; });

        if (it != charging.end()) {
            std::cout << std::setw(20)
                      << "Opt " + std::to_string(it->option_index) +
                         " (" + std::to_string(it->charging_time) + ")";
        } else {
            std::cout << std::setw(20) << "-";
        }
        std::cout << "\n";
    }

    // Print wireless charging decisions
    std::cout << "\nWireless Charging:\n";
    const auto& wireless = route.getWirelessDecisions();
    for (size_t i = 0; i < nodes.size() - 1; ++i) {
        std::cout << nodes[i] << " -> " << nodes[i+1] << ": "
                  << (wireless[i] ? "Yes" : "No") << "\n";
    }
}