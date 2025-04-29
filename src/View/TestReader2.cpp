//
// Created by Admin on 23/04/2025.
//
#include "../../include/CSVReader.h"
#include "../../include/Route.h"
#include "../../include/Optimizer.h"
#include "../../include/Utils.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <set>
#include <algorithm>
#include <utility>


// Hàm in thông tin nút
void printNodesInfo(const std::vector<Node>& nodes) {
    std::cout << "\n=== Node Information ===\n";
    std::cout << std::fixed << std::setprecision(2);
    for (const auto& node : nodes) {
        std::cout << "ID: " << node.id
                  << ", StringID: " << node.string_id
                  << ", Type: " << node.type
                  << ", x: " << node.x
                  << ", y: " << node.y
                  << ", Time Sirvce: " << node.service_time << "\n";
    }
}

// Hàm in thông tin cung
void printArcsInfo(const std::vector<Arc>& arcs) {
    std::cout << "\n=== Arcs Information ===\n";
    std::cout << std::fixed << std::setprecision(2);
    for (const auto& arc : arcs) {
        if (arc.is_wireless) {
            std::cout << "Wireless arcs: from " << arc.from << " to " << arc.to
                      << ", d_ij: " << arc.dij
                      << ", s_ij: " << arc.sij
                      << ", Beta_ij: " << arc.beta_ij
                      << ", U_min: " << arc.U_min
                      << ", U_max: " << arc.U_max << "\n";
        }
    }
}

// Hàm in thông tin lựa chọn sạc
void printChargingOptionsInfo(const std::vector<std::vector<ChargingOption>>& options) {
    std::cout << "\n=== Option charge Information ===\n";
    std::cout << std::fixed << std::setprecision(2);
    for (size_t i = 0; i < options.size(); ++i) {
        if (!options[i].empty()) {
            std::cout << "Node ID: " << i << " have " << options[i].size() << " options:\n";
            for (const auto& opt : options[i]) {
                std::cout << "  Option: " << opt.option
                          << ", Rate: " << opt.rate
                          << ", Cost: " << opt.cost << "\n";
            }
        }
    }
}

// Hàm in thông tin tham số
void printParamsInfo(const Params& params) {
    std::cout << "\n=== Thông tin tham số ===\n";
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
    std::cout << "Testing CSVReader2 for ETSP-WCL...\n";

    try {
        // Đọc dữ liệu từ file CSV
        Params params = CSVReader::readParams("D:\\Work\\NEULab\\ESTP-WCL-Test2\\data\\Input\\params.csv");
        std::vector<Node> nodes = CSVReader::readNodes("D:\\Work\\NEULab\\ESTP-WCL-Test2\\data\\Input\\nodes.csv");
        std::vector<Arc> arcs = CSVReader::generateArcs(nodes, "D:\\Work\\NEULab\\ESTP-WCL-Test2\\data\\Input\\wireless_arcs.csv", params);
        std::vector<std::vector<ChargingOption>> charge_options = CSVReader::readChargingOptions(nodes, "D:\\Work\\NEULab\\ESTP-WCL-Test2\\data\\Input\\charging_options.csv");

        // In thông tin dữ liệu
        printParamsInfo(params);
        printNodesInfo(nodes);
        printArcsInfo(arcs);
        printChargingOptionsInfo(charge_options);
        std::cout << "Route:     \n";
        // Tạo tuyến đường ban đầu
        std::random_device rd;
        std::mt19937 gen(rd());
        std::vector<int> initial_route = generateInitialRoute(nodes, arcs, params, gen);
        std::cout << "The first route: ";
        for (size_t i = 0; i < initial_route.size(); ++i) {
            std::cout << initial_route[i];
            if (i < initial_route.size() - 1) std::cout << " -> ";
        }
        std::cout << "\n";

        // Tạo đối tượng Route và tối ưu hóa bằng MILP
        Route route(initial_route);
        Optimizer optimizer;
        double optimized_cost = route.optimize(arcs, charge_options, params, nodes, optimizer);
        std::cout << "cost: " << optimized_cost << "\n";

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "\nTesting completed. Please verify the results.\n";
    return 0;
}
