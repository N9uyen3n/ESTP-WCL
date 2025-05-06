//
// Created by Admin on 29/04/2025.
//
#include "../../include/CSVReader.h"
#include "../../include/VNSOptimizer.h"
#include "../../include/Utils.h"
#include "../../include/Route.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <random>


int main() {
    try {
        // Đọc dữ liệu từ file CSV
        std::string data_dir = "D:\\Work\\NEULab\\ESTP-WCL-Test5\\data\\Input\\";
        Params params = CSVReader::readParams(data_dir + "params.csv");
        std::vector<Node> nodes = CSVReader::readNodes(data_dir + "nodes.csv");
        std::vector<Arc> arcs = CSVReader::generateArcs(nodes, data_dir + "wireless_arcs.csv", params);
        std::vector<std::vector<ChargingOption>> charge_options = CSVReader::readChargingOptions(nodes, data_dir + "charging_options.csv");
        std::random_device rd;
        std::mt19937 gen(rd());
        std::vector<int> initial_route = generateInitialRoute(nodes, arcs, params, gen, 0.2);
        // Test MILP
        std::cout << "Route: ";
        for (int node : initial_route) {
            std::cout << node << " ";
        }
        std::cout << "Testing MILP...\n";
        std::vector<int> test_route = initial_route; // Tuyến đường đơn giản để test
        Optimizer optimizer;
        double milp_cost = optimizer.optimize(test_route, arcs, charge_options, params, nodes);
        std::cout << "MILP Cost: " << milp_cost << "\n";

        // Test VNS + MILP
        std::cout << "Testing VNS + MILP...\n";

        VNSOptimizer vns(initial_route, nodes, arcs, charge_options, params);
        Route best_route = vns.run(20);
        std::cout << "Best Route: ";
        for (int node : best_route.getNodeIds()) {
            std::cout << node << " ";
        }
        std::cout << "\nVNS + MILP Cost: " << best_route.getTotalCost() << "\n";

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
