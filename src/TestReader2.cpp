//
// Created by Admin on 23/04/2025.
//
#include <iostream>
#include <iomanip>
#include <vector>
#include <set>
#include <algorithm>
#include <utility>
#include "CSVReader.h"
#include "Route.h"
#include "Optimizer.h"

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

// Hàm kiểm tra khả năng đến được nút j từ i với SOC hiện tại
bool can_reach(int i, int j, double SOC, const std::vector<std::vector<Arc>>& arc_matrix, const Params& params) {
    const Arc& arc = arc_matrix[i][j];
    if (!arc.is_wireless) {
        return SOC - params.h * arc.dij >= 0;
    } else {
        double max_energy_gained = arc.beta_ij * (arc.dij / arc.U_min);
        return SOC - params.h * arc.dij + max_energy_gained >= 0;
    }
}

// Hàm cập nhật SOC sau khi di chuyển từ i đến j
double update_SOC(int i, int j, double SOC, const std::vector<std::vector<Arc>>& arc_matrix, const Params& params) {
    const Arc& arc = arc_matrix[i][j];
    if (!arc.is_wireless) {
        return SOC - params.h * arc.dij;
    } else {
        double energy_gained = arc.beta_ij * (arc.dij / arc.U_min);
        return SOC - params.h * arc.dij + energy_gained;
    }
}

// Hàm tạo tuyến đường ban đầu bằng Nearest Neighbor với kiểm tra SOC
std::vector<int> generateInitialRoute(const std::vector<Node>& nodes, const std::vector<Arc>& arcs, const Params& params) {
    int n = nodes.size();
    int starting_depot = 0; // Depot bắt đầu (D0)
    int ending_depot = 16;  // Depot kết thúc (D1)

    // Tạo ma trận cung
    std::vector<std::vector<Arc>> arc_matrix(n, std::vector<Arc>(n));
    for (const auto& arc : arcs) {
        arc_matrix[arc.from][arc.to] = arc;
    }

    // Xác định khách hàng và trạm sạc
    std::vector<int> customers;
    std::vector<int> charging_stations;
    for (const auto& node : nodes) {
        if (node.type == "c") customers.push_back(node.id);
        else if (node.type == "f") charging_stations.push_back(node.id);
    }

    // Khởi tạo tuyến đường
    std::vector<int> route = {starting_depot};
    std::set<int> visited = {starting_depot};
    double current_SOC = params.initial_SOC;
    int current = starting_depot;
    std::set<int> unvisited_customers(customers.begin(), customers.end());

    // Thăm tất cả khách hàng
    while (!unvisited_customers.empty()) {
        bool found = false;

        // Tìm khách hàng khả thi
        std::vector<std::pair<double, int>> feasible_customers;
        for (int j : unvisited_customers) {
            if (can_reach(current, j, current_SOC, arc_matrix, params)) {
                double dist = arc_matrix[current][j].dij;
                feasible_customers.push_back({dist, j});
            }
        }

        if (!feasible_customers.empty()) {
            std::sort(feasible_customers.begin(), feasible_customers.end());
            int next = feasible_customers[0].second;
            route.push_back(next);
            visited.insert(next);
            unvisited_customers.erase(next);
            current_SOC = update_SOC(current, next, current_SOC, arc_matrix, params);
            current = next;
            found = true;
        }

        if (!found) {
            // Đi đến trạm sạc gần nhất
            std::vector<std::pair<double, int>> feasible_stations;
            for (int k : charging_stations) {
                if (can_reach(current, k, current_SOC, arc_matrix, params)) {
                    double dist = arc_matrix[current][k].dij;
                    feasible_stations.push_back({dist, k});
                }
            }

            if (!feasible_stations.empty()) {
                std::sort(feasible_stations.begin(), feasible_stations.end());
                int next = feasible_stations[0].second;
                route.push_back(next);
                current_SOC = params.Q;
                current = next;
            } else {
                std::cout << "Can not go to any charging station " << current << " with SOC " << current_SOC << "\n";
                break;
            }
        }
    }

    // Quay về depot kết thúc
    while (true) {
        if (can_reach(current, ending_depot, current_SOC, arc_matrix, params)) {
            route.push_back(ending_depot);
            break;
        } else {
            std::vector<std::pair<double, int>> feasible_stations;
            for (int k : charging_stations) {
                if (can_reach(current, k, current_SOC, arc_matrix, params)) {
                    double dist = arc_matrix[current][k].dij;
                    feasible_stations.push_back({dist, k});
                }
            }

            if (!feasible_stations.empty()) {
                std::sort(feasible_stations.begin(), feasible_stations.end());
                int next = feasible_stations[0].second;
                route.push_back(next);
                current_SOC = params.Q;
                current = next;
            } else {
                std::cout << "Can't come to Depot to end from" << current << " witg SOC " << current_SOC << "\n";
                break;
            }
        }
    }

    return route;
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
        std::vector<int> initial_route = generateInitialRoute(nodes, arcs, params);
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
