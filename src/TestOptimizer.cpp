#include "CSVReader.h"
#include "Optimizer.h"
#include <iostream>
#include <vector>
#include <stdexcept>

void runTestCase(const std::string& testName, const std::vector<int>& S_prime,
                 const std::vector<Arc>& arcs, const std::vector<std::vector<ChargingOption>>& charge_options,
                 const Params& params, Optimizer& optimizer) {
    std::cout << "Running test case: " << testName << std::endl;
    std::cout << "Route: ";
    for (int node : S_prime) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

    double cost = optimizer.optimize(S_prime, arcs, charge_options, params);
    if (cost < 1e9) {
        std::cout << "Result: Feasible route with cost = " << cost << std::endl;
    } else {
        std::cout << "Result: Infeasible route" << std::endl;
    }
    std::cout << "----------------------------------------" << std::endl;
}

int main() {
    try {
        // Read data from CSV files
        std::string data_dir = "data/";
        auto nodes = CSVReader::readNodes(data_dir + "nodes.csv");
        auto arcs = CSVReader::generateArcs(nodes, data_dir + "wireless_arcs.csv");
        auto charge_options = CSVReader::readChargingOptions(nodes, data_dir + "charging_options.csv");
        auto params = CSVReader::readParams(data_dir + "params.csv");

        // Set U_min and U_max for all arcs
        for (auto& arc : arcs) {
            arc.U_min = params.v;
            arc.U_max = params.v;
        }

        Optimizer optimizer;

        // Test case 1: Full route visiting all customers
        std::vector<int> route1 = {0, 12, 15, 14, 6, 13, 8, 11, 7, 10, 9, 0};
        runTestCase("Full route (all customers)", route1, arcs, charge_options, params, optimizer);

        // Test case 2: Short route with charging station
        std::vector<int> route2 = {0, 1, 15, 0};
        runTestCase("Short route with charging station", route2, arcs, charge_options, params, optimizer);

        // Test case 3: Potentially infeasible route
        std::vector<int> route3 = {0, 6, 0};
        runTestCase("Potentially infeasible route", route3, arcs, charge_options, params, optimizer);

        // Test case 4: Minimal route
        std::vector<int> route4 = {0, 15, 0};
        runTestCase("Minimal route", route4, arcs, charge_options, params, optimizer);


    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}