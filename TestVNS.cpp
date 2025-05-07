#include <iostream>
#include <iomanip>
#include <vector>
#include <string> // Required for std::string
#include <stdexcept> // Required for std::exception

// Assuming your header files are in an 'include' subdirectory
// relative to where this main.cpp might be compiled from (e.g., a project root)
// Or, if main.cpp is in a 'src' directory, and 'include' is a sibling,
// then the path might be "../include/VNS.h"
// For now, using the paths as you provided in your example.
#include "include/VNS.h"
#include "include/Utils.h"
#include "include/Graph.h"      // Added as Node, Arc, Parameters, Route might be used by Utils
#include "include/Parameters.h" // Added
#include "include/Route.h"      // Added


int main() {
    try {
        // Set output precision for floating-point numbers
        std::cout << std::fixed << std::setprecision(2);

        // Directory for input files - adjust if your structure is different
        std::string csv_file_dir = "../data/Input/";

        // 1. Read nodes from CSV
        std::vector<Node> nodes = Utils::readNodes(csv_file_dir + "nodes.csv");
        std::cout << "Nodes loaded: " << nodes.size() << std::endl;
        Utils::printNodesInfo(nodes); // Optional: print node details

        // 2. Read parameters from CSV
        Parameters params = Utils::readParams(csv_file_dir + "params.csv");
        std::cout << "\nParameters loaded:\n";
        Utils::printParamsInfo(params); // Optional: print parameter details

        // 3. Read/Generate arcs
        // Assuming generateArcs creates a complete graph or reads specific arcs
        // and handles wireless properties based on "wireless_arcs.csv"
        std::vector<Arc> arcs = Utils::generateArcs(nodes, csv_file_dir + "wireless_arcs.csv", params);
        std::cout << "\nArcs loaded/generated: " << arcs.size() << std::endl;
        Utils::printArcsInfo(arcs); // Optional: print arc details

        // 4. Create graph object
        Graph graph(nodes, arcs);
        std::cout << "\nGraph created with " << graph.getNodes().size() << " nodes and "
                  << graph.getArcs().size() << " arcs.\n"; // Use graph.getArcs() for consistency

        // 5. Read charging options from CSV
        // Utils::readChargingOptions might return std::vector<std::vector<ChargingOption>>
        // indexed by node ID.
        auto charge_options = Utils::readChargingOptions(nodes, csv_file_dir + "charging_options.csv");
        std::cout << "\nCharging options loaded:\n";
        Utils::printChargingOptionsInfo(charge_options); // Optional: print charging option details

        // 6. Define initial node sequence for the route and S_prime (required customers)
        std::vector<int> initial_nodes_sequence = {0}; // Start at depot (node ID 0)
        std::vector<int> required_customer_ids; // S_prime in your VNS terminology

        for (const auto& node : nodes) {
            if (node.getType() == NodeType::CUSTOMER) {
                required_customer_ids.push_back(node.getId());
                initial_nodes_sequence.push_back(node.getId()); // Add customer to initial sequence
            }
        }
        initial_nodes_sequence.push_back(0); // End at depot (node ID 0)

        std::cout << "\nInitial route sequence: [";
        for (size_t i = 0; i < initial_nodes_sequence.size(); ++i) {
            std::cout << initial_nodes_sequence[i];
            if (i < initial_nodes_sequence.size() - 1) std::cout << " -> ";
        }
        std::cout << "]\n";

        std::cout << "Required customers (S_prime): [";
        for (size_t i = 0; i < required_customer_ids.size(); ++i) {
            std::cout << required_customer_ids[i];
            if (i < required_customer_ids.size() - 1) std::cout << ", ";
        }
        std::cout << "]\n";

        // 7. Initialize VNS algorithm
        int max_iterations = 100;       // Example: Number of VNS iterations
        int max_neighborhoods = 5;      // Corresponds to the number of shake operators (0 to 4)
                                        // The VNS constructor uses this, but the current VNS::optimize loop
                                        // doesn't iterate k from 1 to k_max in a traditional VNS sense.
                                        // It selects one operator randomly based on weights.
        unsigned int seed = 42;         // Seed for random number generation

        VNS vns_solver(max_iterations, max_neighborhoods, seed, required_customer_ids);

        // 8. Run VNS optimization
        std::cout << "\nStarting VNS optimization...\n";
        // The VNS::optimize method will take the initial sequence and try to find an optimized, feasible route.
        Route optimized_route = vns_solver.optimize(initial_nodes_sequence, graph, charge_options, params);

        // 9. Print optimization results
        std::cout << "\n=== Optimization Results ===\n";
        std::cout << "Optimized Route: [";
        const auto& final_route_nodes = optimized_route.getNodeIds();
        for (size_t i = 0; i < final_route_nodes.size(); ++i) {
            std::cout << final_route_nodes[i];
            if (i < final_route_nodes.size() - 1) std::cout << " -> ";
        }
        std::cout << "]\n";
        std::cout << "Total Cost: " << optimized_route.getTotalCost() << "\n";
        std::cout << "Feasible: " << (optimized_route.isFeasible() ? "Yes" : "No") << "\n";

        // 10. Print detailed route information if the route is feasible and has details
        if (optimized_route.isFeasible() && !final_route_nodes.empty()) {
            std::cout << "\nDetailed Route Information:\n";
            std::cout << std::left << std::setw(10) << "Node ID"
                      << std::setw(15) << "SOC Arrival"
                      << std::setw(15) << "SOC Departure"
                      << std::setw(15) << "Arrival Time"
                      << std::setw(15) << "Departure Time"
                      << std::setw(30) << "Charging Decision (Opt, Time)" // Adjusted width
                      << "\n";
            std::cout << std::string(100, '-') << "\n"; // Adjusted width

            const auto& soc_arrival = optimized_route.getSocArrival();
            const auto& soc_departure = optimized_route.getSocDeparture();
            const auto& arrival_time = optimized_route.getArrivalTime();
            const auto& departure_time = optimized_route.getDepartureTime();
            const auto& route_charging_decisions = optimized_route.getChargingDecisions();

            // Ensure vectors are not empty and have the same size as final_route_nodes
            bool details_available = (soc_arrival.size() == final_route_nodes.size() &&
                                     soc_departure.size() == final_route_nodes.size() &&
                                     arrival_time.size() == final_route_nodes.size() &&
                                     departure_time.size() == final_route_nodes.size());

            if (details_available) {
                size_t charge_decision_idx = 0;
                for (size_t i = 0; i < final_route_nodes.size(); ++i) {
                    std::cout << std::setw(10) << final_route_nodes[i]
                              << std::setw(15) << soc_arrival[i]
                              << std::setw(15) << soc_departure[i]
                              << std::setw(15) << arrival_time[i]
                              << std::setw(15) << departure_time[i];

                    std::string charge_info_str = "None";
                    // Iterate through all charging decisions to find one for the current node
                    // This is more robust if decisions are not strictly ordered or if a station is visited multiple times.
                    for(const auto& decision : route_charging_decisions) {
                        if (decision.station_id == final_route_nodes[i] &&
                            (i > 0 && i < final_route_nodes.size() -1 ) ) { // Only for non-depot nodes
                            // Check if this decision corresponds to the current visit (e.g. by comparing times, if complex)
                            // For simplicity, we assume the first match is the one for this stop.
                            // Or, if SubproblemResult populates decisions in order of visit, this can be simpler.
                            // The previous main.cpp logic assumed a single pass with charge_idx.
                            charge_info_str = "Opt " + std::to_string(decision.option_index) +
                                              ", Time: " + std::to_string(decision.charging_time);
                            break; // Found a decision for this node visit
                        }
                    }
                    std::cout << std::setw(30) << charge_info_str << "\n";
                }
            } else {
                std::cout << "Detailed SOC/time information not fully populated for the route.\n";
            }

            // Print wireless charging decisions for arcs
            std::cout << "\nWireless Charging Decisions (for arcs):\n";
            const auto& wireless_decisions_on_arcs = optimized_route.getWirelessDecisions();
            if (wireless_decisions_on_arcs.size() == final_route_nodes.size() - 1 && final_route_nodes.size() > 1) {
                for (size_t i = 0; i < wireless_decisions_on_arcs.size(); ++i) {
                    if (wireless_decisions_on_arcs[i]) {
                        std::cout << "Arc (" << final_route_nodes[i] << " -> " << final_route_nodes[i + 1]
                                  << "): Wireless charging USED\n";
                    } else {
                         std::cout << "Arc (" << final_route_nodes[i] << " -> " << final_route_nodes[i + 1]
                                  << "): Wireless charging NOT used\n";
                    }
                }
            } else if (!final_route_nodes.empty()) {
                 std::cout << "Wireless decision data size mismatch or route too short.\n";
            }
        } else if (!optimized_route.isFeasible()) {
            std::cout << "Route is not feasible. No detailed information to display.\n";
        }


    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1; // Indicate failure
    } catch (...) {
        // Catch all other types of exceptions
        std::cerr << "An unknown error occurred." << std::endl;
        return 1; // Indicate failure
    }

    return 0; // Indicate success
}
