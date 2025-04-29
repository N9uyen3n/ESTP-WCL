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
    std::cout << "Testing VNSOptimizer for ETSP-WCL...\n";

    try {
        Params params = CSVReader::readParams("D:\\Work\\NEULab\\ESTP-WCL-Test5\\data\\Input\\params.csv");
        std::vector<Node> nodes = CSVReader::readNodes("D:\\Work\\NEULab\\ESTP-WCL-Test5\\data\\Input\\nodes.csv");
        std::vector<Arc> arcs = CSVReader::generateArcs(nodes,
            "D:\\Work\\NEULab\\ESTP-WCL-Test5\\data\\Input\\wireless_arcs.csv", params);
        std::vector<std::vector<ChargingOption>> charge_options = CSVReader::readChargingOptions(nodes,
            "D:\\Work\\NEULab\\ESTP-WCL-Test5\\data\\Input\\charging_options.csv");

        std::cout << "\n";

        VNSOptimizer vns_optimizer(nodes, arcs, charge_options, params);
        Route best_route = vns_optimizer.run(100);

        std::cout << "Best route after VNS optimization:\n";
        best_route.print();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "\nTesting completed. Please verify the results.\n";
    return 0;
}
