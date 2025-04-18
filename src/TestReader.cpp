// TestReader.cpp
#include <iostream>
#include <iomanip>
#include "CSVReader.h"

// Function to print node information
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

// Function to print arc information
void printArcsInfo(const std::vector<Arc>& arcs) {
    std::cout << "\n=== Arc Information ===\n";
    std::cout << std::fixed << std::setprecision(2);
    for (const auto& arc : arcs) {
        if (arc.is_wireless) {
            std::cout << "Wireless Arc: From " << arc.from << " to " << arc.to
                      << ", d_ij: " << arc.dij
                      << ", s_ij: " << arc.sij
                      << ", β_ij: " << arc.beta_ij
                      << ", U_min: " << arc.U_min
                      << ", U_max: " << arc.U_max << "\n";
        }
    }
}

// Function to print charging option information
void printChargingOptionsInfo(const std::vector<std::vector<ChargingOption>>& options) {
    std::cout << "\n=== Charging Options Information ===\n";
    std::cout << std::fixed << std::setprecision(2);
    for (size_t i = 0; i < options.size(); ++i) {
        if (!options[i].empty()) {
            std::cout << "Node ID: " << i << " has " << options[i].size() << " options:\n";
            for (const auto& opt : options[i]) {
                std::cout << "  Option: " << opt.option
                          << ", Rate: " << opt.rate
                          << ", Cost: " << opt.cost << "\n";
            }
        }
    }
}

// Function to print parameter information
void printParamsInfo(const Params& params) {
    std::cout << "\n=== Parameter Information ===\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Q: " << params.Q << "\n";
    std::cout << "h: " << params.h << "\n";
    std::cout << "v: " << params.v << "\n";
    std::cout << "cw: " << params.cw << "\n";
    std::cout << "ct: " << params.ct << "\n";
    std::cout << "minSOC: " << params.minSOC << "\n";
    std::cout << "initial_SOC: " << params.initial_SOC << "\n";
    std::cout << "M: " << params.M << "\n";
}

int main() {
    std::cout << "Testing CSVReader for ETSP-WCL...\n";

    try {
        // Read and print parameter information
        Params params = CSVReader::readParams("D:\\Work\\NEULab\\ESTP-WCL-Test1\\Test1\\data\\Input\\params.csv");
        printParamsInfo(params);

        // Read and print node information
        std::vector<Node> nodes = CSVReader::readNodes("D:\\Work\\NEULab\\ESTP-WCL-Test1\\Test1\\data\\Input\\nodes.csv");
        printNodesInfo(nodes);

        // Read and print arc information
        std::vector<Arc> arcs = CSVReader::generateArcs(nodes, "D:\\Work\\NEULab\\ESTP-WCL-Test1\\Test1\\data\\Input\\wireless_arcs.csv", params);
        printArcsInfo(arcs);

        // Read and print charging option information
        std::vector<std::vector<ChargingOption>> charge_options = CSVReader::readChargingOptions(nodes, "D:\\Work\\NEULab\\ESTP-WCL-Test1\\Test1\\data\\Input\\charging_options.csv");
        printChargingOptionsInfo(charge_options);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "\nTesting completed. Please verify the results.\n";
    return 0;
}