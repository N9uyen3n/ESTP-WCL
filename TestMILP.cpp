#include <iostream>
#include <iomanip>
#include <chrono>
#include "include/Utils.h"
#include "include/MILP.h"
#include "include/Graph.h"

int main() {
    try {
        // Read input data
        std::string csv_file_dir = "../data/Input/";
        std::vector<Node> nodes = Utils::readNodes(csv_file_dir + "nodes.csv");
        Utils::printNodesInfo(nodes);

        Parameters params = Utils::readParams(csv_file_dir + "params.csv");
        Utils::printParamsInfo(params);

        std::vector<Arc> arcs = Utils::generateArcs(nodes, csv_file_dir + "wireless_arcs.csv", params);
        Utils::printArcsInfo(arcs);

        auto charging_options = Utils::readChargingOptions(nodes, csv_file_dir + "charging_options.csv");
        Utils::printChargingOptionsInfo(charging_options);

        // Create graph instance
        Graph graph(nodes, arcs);

        // Create initial route (depot + customers)
        std::vector<int> initial_nodes;
        initial_nodes.push_back(0); // Depot
        for (const auto& node : nodes) {
            if (node.getType() == NodeType::CUSTOMER) {
                initial_nodes.push_back(node.getId());
            }
        }
        initial_nodes.push_back(0); // Return to depot

        // Create and run MILP optimizer
        MILP milp;
        auto start_time = std::chrono::high_resolution_clock::now();

        Route optimized_route = milp.optimize(initial_nodes, graph, charging_options, params);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();

        // Print results
        std::cout << "\nOptimization Results:\n";
        std::cout << "Total cost: " << optimized_route.getTotalCost() << "\n";
        std::cout << "Computation time: " << duration << " seconds\n";
        std::cout << "Route: ";
        for (int node_id : optimized_route.getNodeIds()) {
            std::cout << node_id << " ";
        }
        std::cout << "\n";

        std::cout << "\nDetailed Route Information:\n";
        std::cout << std::fixed << std::setprecision(2);
        for (size_t i = 0; i < optimized_route.getNodeIds().size(); ++i) {
            int node_id = optimized_route.getNodeIds()[i];
            std::cout << "Node " << node_id << ":\n";
            std::cout << "  Arrival Time: " << optimized_route.getArrivalTime()[i] << "\n";
            std::cout << "  SOC on Arrival: " << optimized_route.getSocArrival()[i] << "\n";
            std::cout << "  Departure Time: " << optimized_route.getDepartureTime()[i] << "\n";
            std::cout << "  SOC on Departure: " << optimized_route.getSocDeparture()[i] << "\n";
        }

        std::cout << "\nCharging Decisions:\n";
        for (const auto& decision : optimized_route.getChargingDecisions()) {
            std::cout << "Node " << decision.station_id << ": Option " << decision.option_index
                      << ", Duration " << decision.charging_time << "\n";
        }

        std::cout << "\nWireless Charging:\n";
        const auto& wireless = optimized_route.getWirelessDecisions();
        for (size_t i = 0; i < wireless.size(); ++i) {
            if (wireless[i]) {
                std::cout << "Wireless charging used between nodes "
                          << optimized_route.getNodeIds()[i] << " and "
                          << optimized_route.getNodeIds()[i + 1] << "\n";
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}