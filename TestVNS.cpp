#include <iostream>
#include <iomanip>
#include <vector>
#include "include/VNS.h"
#include "include/Utils.h"

int main() {
    try {
        // Set output precision
        std::cout << std::fixed << std::setprecision(2);

        // Directory for input files
        std::string csv_file_dir = "../data/Input/";

        // Read nodes
        std::vector<Node> nodes = Utils::readNodes(csv_file_dir + "nodes.csv");
        std::cout << "Nodes loaded: " << nodes.size() << std::endl;
        Utils::printNodesInfo(nodes);

        // Read parameters
        Parameters params = Utils::readParams(csv_file_dir + "params.csv");
        std::cout << "\nParameters loaded:\n";
        Utils::printParamsInfo(params);

        // Read arcs and generate complete graph
        std::vector<Arc> arcs = Utils::generateArcs(nodes, csv_file_dir + "wireless_arcs.csv", params);
        std::cout << "\nArcs loaded: " << arcs.size() << std::endl;
        Utils::printArcsInfo(arcs);

        // Create graph
        Graph graph(nodes, arcs);
        std::cout << "\nGraph created with " << graph.getNodes().size() << " nodes and "
                  << arcs.size() << " arcs.\n";

        // Read charging options
        auto charge_options = Utils::readChargingOptions(nodes, csv_file_dir + "charging_options.csv");
        std::cout << "\nCharging options loaded:\n";
        Utils::printChargingOptionsInfo(charge_options);

        // Define initial nodes and S_prime (required customers)
        std::vector<int> initial_nodes = {0}; // Start at depot
        std::vector<int> S_prime;
        for (const auto& node : nodes) {
            if (node.getType() == NodeType::CUSTOMER) {
                S_prime.push_back(node.getId());
                initial_nodes.push_back(node.getId());
            }
        }
        initial_nodes.push_back(0); // End at depot
        std::cout << "\nInitial route: [";
        for (size_t i = 0; i < initial_nodes.size(); ++i) {
            std::cout << initial_nodes[i];
            if (i < initial_nodes.size() - 1) std::cout << " -> ";
        }
        std::cout << "]\n";
        std::cout << "Required customers (S_prime): [";
        for (size_t i = 0; i < S_prime.size(); ++i) {
            std::cout << S_prime[i];
            if (i < S_prime.size() - 1) std::cout << ", ";
        }
        std::cout << "]\n";

        // Initialize VNS
        int max_iterations = 100;
        int max_neighborhoods = 4;
        unsigned int seed = 42;
        VNS vns(max_iterations, max_neighborhoods, seed, S_prime);

        // Optimize route
        std::cout << "\nStarting VNS optimization...\n";
        Route optimized_route = vns.optimize(initial_nodes, graph, charge_options, params);

        // Print results
        std::cout << "\n=== Optimization Results ===\n";
        std::cout << "Optimized Route: [";
        const auto& route_nodes = optimized_route.getNodeIds();
        for (size_t i = 0; i < route_nodes.size(); ++i) {
            std::cout << route_nodes[i];
            if (i < route_nodes.size() - 1) std::cout << " -> ";
        }
        std::cout << "]\n";
        std::cout << "Total Cost: " << optimized_route.getTotalCost() << "\n";
        std::cout << "Feasible: " << (optimized_route.isFeasible() ? "Yes" : "No") << "\n";

        // Print detailed route information
        std::cout << "\nDetailed Route Information:\n";
        std::cout << std::left << std::setw(10) << "Node ID"
                  << std::setw(15) << "SOC Arrival"
                  << std::setw(15) << "SOC Departure"
                  << std::setw(15) << "Arrival Time"
                  << std::setw(15) << "Departure Time"
                  << std::setw(20) << "Charging Decision"
                  << "\n";
        std::cout << std::string(90, '-') << "\n";
        const auto& soc_arrival = optimized_route.getSocArrival();
        const auto& soc_departure = optimized_route.getSocDeparture();
        const auto& arrival_time = optimized_route.getArrivalTime();
        const auto& departure_time = optimized_route.getDepartureTime();
        const auto& charging_decisions = optimized_route.getChargingDecisions();
        size_t charge_idx = 0;
        for (size_t i = 0; i < route_nodes.size(); ++i) {
            std::cout << std::setw(10) << route_nodes[i]
                      << std::setw(15) << soc_arrival[i]
                      << std::setw(15) << soc_departure[i]
                      << std::setw(15) << arrival_time[i]
                      << std::setw(15) << departure_time[i];
            bool has_charging = false;
            if (charge_idx < charging_decisions.size() &&
                charging_decisions[charge_idx].station_id == route_nodes[i]) {
                std::cout << std::setw(20) << ("Option " + std::to_string(charging_decisions[charge_idx].option_index) +
                                               ", Time: " + std::to_string(charging_decisions[charge_idx].charging_time));
                charge_idx++;
                has_charging = true;
            }
            if (!has_charging) {
                std::cout << std::setw(20) << "None";
            }
            std::cout << "\n";
        }

        // Print wireless charging decisions
        std::cout << "\nWireless Charging Decisions:\n";
        const auto& wireless_decisions = optimized_route.getWirelessDecisions();
        for (size_t i = 0; i < wireless_decisions.size(); ++i) {
            if (wireless_decisions[i]) {
                std::cout << "Arc (" << route_nodes[i] << " -> " << route_nodes[i + 1] << "): Wireless charging used\n";
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred" << std::endl;
        return 1;
    }

    return 0;
}