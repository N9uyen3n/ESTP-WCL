#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <random>
#include "include/Utils.h"
#include "include/Graph.h"
#include "include/VNS.h"
#include "include/MILP.h"

void testMILP(const std::string& csv_file_dir, std::ofstream& out_file) {
    try {
        // Write directory header to file
        out_file << "===== Test Results for " << csv_file_dir << " =====\n\n";
        std::cout << "Testing directory: " << csv_file_dir << "\n";

        // 1. Read data
        out_file << "Reading input data from directory: " << csv_file_dir << "\n";
        std::cout << "Reading input data from directory: " << csv_file_dir << "\n";

        // Read nodes
        std::vector<Node> nodes = Utils::readNodes(csv_file_dir + "nodes.csv");
        out_file << "\nNodes loaded:\n";
        std::cout << "\nNodes loaded:\n";
        Utils::printNodesInfo(nodes);
        // Redirect nodes info to file
        std::stringstream nodes_ss;
        auto old_cout_buf = std::cout.rdbuf(nodes_ss.rdbuf());
        Utils::printNodesInfo(nodes);
        std::cout.rdbuf(old_cout_buf);
        out_file << nodes_ss.str();

        // Read parameters
        Parameters params = Utils::readParams(csv_file_dir + "params.csv");
        out_file << "\nParameters loaded:\n";
        std::cout << "\nParameters loaded:\n";
        Utils::printParamsInfo(params);
        std::stringstream params_ss;
        std::cout.rdbuf(params_ss.rdbuf());
        Utils::printParamsInfo(params);
        std::cout.rdbuf(old_cout_buf);
        out_file << params_ss.str();

        // Read arcs
        std::vector<Arc> arcs = Utils::generateArcs(nodes, csv_file_dir + "wireless_arcs.csv", params);
        out_file << "\nArcs loaded:\n";
        std::cout << "\nArcs loaded:\n";
        Utils::printArcsInfo(arcs);
        std::stringstream arcs_ss;
        std::cout.rdbuf(arcs_ss.rdbuf());
        Utils::printArcsInfo(arcs);
        std::cout.rdbuf(old_cout_buf);
        out_file << arcs_ss.str();

        // Read charging options
        std::vector<std::vector<ChargingOption>> charging_options =
            Utils::readChargingOptions(nodes, csv_file_dir + "charging_options.csv");
        out_file << "\nCharging options loaded:\n";
        std::cout << "\nCharging options loaded:\n";
        Utils::printChargingOptionsInfo(charging_options);
        std::stringstream options_ss;
        std::cout.rdbuf(options_ss.rdbuf());
        Utils::printChargingOptionsInfo(charging_options);
        std::cout.rdbuf(old_cout_buf);
        out_file << options_ss.str();

        // 2. Create graph
        Graph graph(nodes, arcs);
        graph.setChargingOptions(charging_options);

        // 3. Generate initial route
        std::random_device rd;
        std::mt19937 gen(rd());
        std::vector<int> initial_route = Utils::generateInitialRoute(nodes, arcs, params, gen);
        if (initial_route.empty()) {
            throw std::runtime_error("Failed to generate initial route");
        }
        out_file << "\nInitial route: ";
        std::cout << "\nInitial route: ";
        for (int id : initial_route) {
            out_file << id << " ";
            std::cout << id << " ";
        }
        out_file << "\n";
        std::cout << "\n";

        // 4. Run MILP
        MILP milp;
        Route best_route = milp.optimize(initial_route, graph, charging_options, params);

        // 5. Output results
        out_file << "\n====== MILP Results ======\n";
        std::cout << "\n====== MILP Results ======\n";
        if (best_route.isFeasible()) {
            out_file << "Best route found: ";
            std::cout << "Best route found: ";
            for (int id : best_route.getNodeIds()) {
                out_file << id << " (" << Utils::getNodeType(id, nodes) << ") ";
                std::cout << id << " (" << Utils::getNodeType(id, nodes) << ") ";
            }
            out_file << "\nTotal cost: " << std::fixed << std::setprecision(2) << best_route.getTotalCost() << "\n";
            std::cout << "\nTotal cost: " << std::fixed << std::setprecision(2) << best_route.getTotalCost() << "\n";

            out_file << "\nDetailed route information:\n";
            std::cout << "\nDetailed route information:\n";
            const auto& soc_arrival = best_route.getSocArrival();
            const auto& soc_departure = best_route.getSocDeparture();
            const auto& arrival_time = best_route.getArrivalTime();
            const auto& departure_time = best_route.getDepartureTime();
            const auto& charging_decisions = best_route.getChargingDecisions();
            const auto& wireless_decisions = best_route.getArcWirelessDecisions();

            for (size_t i = 0; i < best_route.getNodeIds().size(); ++i) {
                int node_id = best_route.getNodeIds()[i];
                out_file << "Node " << node_id << " (" << Utils::getNodeType(node_id, nodes) << "):\n"
                         << "  SOC Arrival: " << soc_arrival[i] << "\n"
                         << "  SOC Departure: " << soc_departure[i] << "\n"
                         << "  Arrival Time: " << arrival_time[i] << "\n"
                         << "  Departure Time: " << departure_time[i] << "\n";
                std::cout << "Node " << node_id << " (" << Utils::getNodeType(node_id, nodes) << "):\n"
                          << "  SOC Arrival: " << soc_arrival[i] << "\n"
                          << "  SOC Departure: " << soc_departure[i] << "\n"
                          << "  Arrival Time: " << arrival_time[i] << "\n"
                          << "  Departure Time: " << departure_time[i] << "\n";
            }

            out_file << "\nCharging decisions:\n";
            std::cout << "\nCharging decisions:\n";
            for (const auto& decision : charging_decisions) {
                out_file << "  Station ID: " << decision.getStationId()
                         << ", Option: " << decision.getOption()
                         << ", Charging Time: " << decision.getChargingTime() << "\n";
                std::cout << "  Station ID: " << decision.getStationId()
                          << ", Option: " << decision.getOption()
                          << ", Charging Time: " << decision.getChargingTime() << "\n";
            }

            out_file << "\nWireless charging decisions:\n";
            std::cout << "\nWireless charging decisions:\n";
            for (const auto& decision : wireless_decisions) {
                out_file << "  Arc (" << decision.first << ", " << decision.second << "): "
                         << "Wireless Charging Enabled\n";
                std::cout << "  Arc (" << decision.first << ", " << decision.second << "): "
                          << "Wireless Charging Enabled\n";
            }
        } else {
            out_file << "No feasible route found.\n";
            std::cout << "No feasible route found.\n";
        }

        out_file << "\n----------------------------------------\n\n";
    } catch (const std::exception& e) {
        std::cerr << "Error in " << csv_file_dir << ": " << e.what() << "\n";
        out_file << "Error: " << e.what() << "\n\n----------------------------------------\n\n";
    }
}

int main() {
    // Open output file
    std::ofstream out_file("test_results_3.txt");
    if (!out_file.is_open()) {
        std::cerr << "Failed to open test_results.txt\n";
        return 1;
    }

    // Base directory
    std::string base_dir = "../data/Input/Small_2/";
    
    // Iterate through all subdirectories in Small_2
    try {
        for (const auto& entry : std::filesystem::directory_iterator(base_dir)) {
            if (entry.is_directory()) {
                std::string csv_file_dir = entry.path().string() + "/";
                testMILP(csv_file_dir, out_file);
            }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << "\n";
        out_file << "Filesystem error: " << e.what() << "\n";
    }

    out_file.close();
    std::cout << "Testing complete. Results saved to test_results.txt\n";
    return 0;
}