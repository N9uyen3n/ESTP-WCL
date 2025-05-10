#include <iostream>
#include <iomanip>
#include <random>
#include "include/Utils.h"
#include "include/Graph.h"
#include "include/VNS.h"

void testVNS(const std::string& csv_file_dir, int max_iterations = 100) {
    try {
        // 1. Đọc dữ liệu
        std::cout << "Reading input data from directory: " << csv_file_dir << "\n";

        // Đọc nodes
        std::vector<Node> nodes = Utils::readNodes(csv_file_dir + "nodes.csv");
        std::cout << "\nNodes loaded:\n";
        Utils::printNodesInfo(nodes);

        // Đọc parameters
        Parameters params = Utils::readParams(csv_file_dir + "params.csv");
        std::cout << "\nParameters loaded:\n";
        Utils::printParamsInfo(params);

        // Đọc arcs và tạo đồ thị hoàn chỉnh
        std::vector<Arc> arcs = Utils::generateArcs(nodes, csv_file_dir + "wireless_arcs.csv", params);
        std::cout << "\nArcs loaded:\n";
        Utils::printArcsInfo(arcs);

        // Đọc charging options
        std::vector<std::vector<ChargingOption>> charging_options =
            Utils::readChargingOptions(nodes, csv_file_dir + "charging_options.csv");
        std::cout << "\nCharging options loaded:\n";
        Utils::printChargingOptionsInfo(charging_options);

        // 2. Tạo đồ thị
        Graph graph(nodes, arcs);
        graph.setChargingOptions(charging_options);

        // 3. Tạo lộ trình ban đầu
        std::random_device rd;
        std::mt19937 gen(rd());
        std::vector<int> initial_route = Utils::generateInitialRoute(nodes, arcs, params, gen);
        if (initial_route.empty()) {
            throw std::runtime_error("Failed to generate initial route");
        }
        std::cout << "\nInitial route: ";
        for (int id : initial_route) {
            std::cout << id << " ";
        }
        std::cout << "\n";

        // 4. Khởi tạo và chạy VNS
        VNS vns(initial_route, graph, charging_options, params, max_iterations, gen);
        Route best_route = vns.run();

        // 5. In kết quả
        std::cout << "\n=== VNS Results ===\n";
        if (best_route.isFeasible()) {
            std::cout << "Best route found: ";
            for (int id : best_route.getNodeIds()) {
                std::cout << id << " (" << Utils::getNodeType(id, nodes) << ") ";
            }
            std::cout << "\nTotal cost: " << std::fixed << std::setprecision(2) << best_route.getTotalCost() << "\n";

            std::cout << "\nDetailed route information:\n";
            const auto& soc_arrival = best_route.getSocArrival();
            const auto& soc_departure = best_route.getSocDeparture();
            const auto& arrival_time = best_route.getArrivalTime();
            const auto& departure_time = best_route.getDepartureTime();
            const auto& wireless_decisions = best_route.getWirelessDecisions();
            const auto& charging_decisions = best_route.getChargingDecisions();

            for (size_t i = 0; i < best_route.getNodeIds().size(); ++i) {
                int node_id = best_route.getNodeIds()[i];
                std::cout << "Node " << node_id << " (" << Utils::getNodeType(node_id, nodes) << "):\n"
                          << "  SOC Arrival: " << soc_arrival[i] << "\n"
                          << "  SOC Departure: " << soc_departure[i] << "\n"
                          << "  Arrival Time: " << arrival_time[i] << "\n"
                          << "  Departure Time: " << departure_time[i] << "\n";
                if (i < wireless_decisions.size()) {
                    std::cout << "  Wireless Charging to next node: " << (wireless_decisions[i] ? "Yes" : "No") << "\n";
                }
            }

            std::cout << "\nCharging decisions:\n";
            for (const auto& decision : charging_decisions) {
                std::cout << "  Station ID: " << decision.getStationId()
                          << ", Option: " << decision.getOption()
                          << ", Charging Time: " << decision.getChargingTime() << "\n";
            }
        } else {
            std::cout << "No feasible route found.\n";
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

int main() {
    std::string csv_file_dir = "../data/Input/c5-s5/";
    testVNS(csv_file_dir, 100); // Chạy VNS với 100 vòng lặp
    return 0;
}