#include <iomanip>
#include "../../include/CSVReader.h"
#include "../../include/Optimizer.h"
#include "../../include/Utils.h" // Để sử dụng các hàm tiện ích
#include <iostream>
#include <vector>
#include <stdexcept>
#include <set>
#include <random>

// Function to validate a route
bool isValidRoute(const std::vector<int>& route, const std::vector<Node>& nodes) {
    if (route.empty()) return false;
    std::set<int> node_ids;
    for (const auto& node : nodes) {
        node_ids.insert(node.id);
    }
    for (int id : route) {
        if (node_ids.find(id) == node_ids.end()) {
            std::cerr << "Invalid node ID in route: " << id << std::endl;
            return false;
        }
    }
    return route.front() == 0 && route.back() == 0; // Assume depot is 0
}

// Function to run a test case and print results
void runTestCase(const std::string& testName, const std::vector<int>& route,
                 const std::vector<Arc>& arcs, const std::vector<std::vector<ChargingOption>>& charge_options,
                 const Params& params, const std::vector<Node>& nodes, Optimizer& optimizer) {
    std::cout << "Running test case: " << testName << std::endl;
    std::cout << "Route: ";
    for (int node : route) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

    if (!isValidRoute(route, nodes)) {
        std::cerr << "Error: Invalid route for test case " << testName << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        return;
    }

    try {
        ModelParameters modelParams; // Tạo đối tượng để lưu tham số
        double cost = optimizer.optimize(route, arcs, charge_options, params, nodes, modelParams);

        if (cost < 1e9) {
            std::cout << "Result: Feasible route with cost = " << std::fixed << std::setprecision(2) << cost << std::endl;
            // In tham số bằng hàm trong Utils.h
            std::cout << "\nModel Parameters for Test Case " << testName << ":\n";
            printModelParameters(modelParams, route);
        } else {
            std::cout << "Result: Infeasible route" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error in test case " << testName << ": " << e.what() << std::endl;
    }
    std::cout << "----------------------------------------" << std::endl;
}

int main() {
    try {
        // Specify the directory containing CSV files
        std::string data_dir = "D:\\Work\\NEULab\\ESTP-WCL-Test5\\data\\Input\\";

        // Read parameters first
        Params params = CSVReader::readParams(data_dir + "params.csv");

        // Read nodes from CSV
        std::vector<Node> nodes = CSVReader::readNodes(data_dir + "nodes.csv");

        // Generate arcs using nodes and parameters
        std::vector<Arc> arcs = CSVReader::generateArcs(nodes, data_dir + "wireless_arcs.csv", params);

        // Read charging options
        std::vector<std::vector<ChargingOption>> charge_options = CSVReader::readChargingOptions(nodes, data_dir + "charging_options.csv");

        // In thông tin đầu vào bằng các hàm trong Utils.h
        printParamsInfo(params);
        printNodesInfo(nodes);
        printArcsInfo(arcs);
        printChargingOptionsInfo(charge_options);

        // Create an Optimizer instance
        Optimizer optimizer;

        // Generate multiple initial routes using generateInitialRoute from Utils.h
        std::random_device rd;
        std::mt19937 gen(rd());
        std::vector<std::vector<int>> test_routes;
        int num_routes = 5; // Số lượng tuyến đường cần tạo

        for (int i = 0; i < num_routes; ++i) {
            std::vector<int> route = generateInitialRoute(nodes, arcs, params, gen);
            if (!route.empty()) {
                test_routes.push_back(route);
            } else {
                std::cout << "Could not generate route for Test Case " << (i + 1) << std::endl;
            }
        }

        // Kiểm tra nếu không tạo được tuyến nào
        if (test_routes.empty()) {
            std::cerr << "No feasible routes could be generated.\n";
            return 1;
        }

        // Run test cases
        for (size_t i = 0; i < test_routes.size(); ++i) {
            std::string test_name = "Test Case " + std::to_string(i + 1);
            runTestCase(test_name, test_routes[i], arcs, charge_options, params, nodes, optimizer);
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred.\n";
        return 1;
    }

    std::cout << "All test cases completed." << std::endl;
    return 0;
}