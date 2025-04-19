#include <iomanip>

#include "CSVReader.h"
#include "Optimizer.h"
#include <iostream>
#include <vector>
#include <stdexcept>
#include <set>

// Function to print node information for verification
void printNodesInfo(const std::vector<Node>& nodes) {
    std::cout << "\n=== Node Information ===\n";
    std::cout << std::fixed << std::setprecision(2);
    for (const auto& node : nodes) {
        std::cout << "Node ID: " << node.id
                  << ", StringID: " << node.string_id
                  << ", Type: " << node.type
                  << ", x: " << node.x
                  << ", y: " << node.y
                  << ", ServiceTime: " << node.service_time << "\n";
    }
}

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
        double cost = optimizer.optimize(route, arcs, charge_options, params, nodes);

        if (cost < 1e9) {
            std::cout << "Result: Feasible route with cost = " << cost << std::endl;
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
        std::string data_dir = "D:\\Work\\NEULab\\ESTP-WCL-Test1\\Test1\\data\\Input\\";

        // Read parameters first
        Params params = CSVReader::readParams(data_dir + "params.csv");

        // Read nodes from CSV
        std::vector<Node> nodes = CSVReader::readNodes(data_dir + "nodes.csv");

        // Print node information for verification
        printNodesInfo(nodes);

        // Generate arcs using nodes and parameters
        std::vector<Arc> arcs = CSVReader::generateArcs(nodes, data_dir + "wireless_arcs.csv", params);

        // Read charging options
        std::vector<std::vector<ChargingOption>> charge_options = CSVReader::readChargingOptions(nodes, data_dir + "charging_options.csv");

        // Initialize optimizer
        Optimizer optimizer;

        // Test case 1: Full route visiting all customers
        // Assumes node IDs: 0 (depot), 6-15 (customers C1-C10)
        std::vector<int> route1 = {0, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0};
        runTestCase("Full route (all customers)", route1, arcs, charge_options, params, nodes, optimizer);

        // // Test case 2: Short route with charging station
        // // Assumes node IDs: 0 (depot), 1 (charging station S0), 6 (customer C1)
        // std::vector<int> route2 = {0, 1, 6, 0};
        // runTestCase("Short route with charging station", route2, arcs, charge_options, params, nodes, optimizer);
        //
        // // Test case 3: Potentially infeasible route (short route, may lack energy)
        // // Assumes node IDs: 0 (depot), 6 (customer C1)
        // std::vector<int> route3 = {0, 6, 0};
        // runTestCase("Potentially infeasible route", route3, arcs, charge_options, params, nodes, optimizer);
        //
        // // Test case 4: Minimal route (depot to customer and back)
        // // Assumes node IDs: 0 (depot), 6 (customer C1)
        // std::vector<int> route4 = {0, 6, 0};
        // runTestCase("Minimal route", route4, arcs, charge_options, params, nodes, optimizer);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "All test cases completed." << std::endl;
    return 0;
}