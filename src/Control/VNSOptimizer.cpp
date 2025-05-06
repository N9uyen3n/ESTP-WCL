//
// Created by Admin on 29/04/2025.
//

#include "../../include/VNSOptimizer.h"
#include "../../include/Utils.h"
#include <random>
#include <iostream>
#include <set>
#include <iomanip>

VNSOptimizer::VNSOptimizer(const std::vector<int>& S_prime,
                           const std::vector<Node>& nodes,
                           const std::vector<Arc>& ars,
                           const std::vector<std::vector<ChargingOption>>& chg_opts,
                           const Params& params)
    : S_prime(S_prime), nodes(nodes), arcs(ars), charge_options(chg_opts), params(params) {}

// Hàm kiểm tra tính hợp lệ của tuyến đường
bool isValidRoute(const std::vector<int>& route, const std::vector<Node>& nodes, const std::vector<int>& S_prime) {
    if (route.front() != 0 || route.back() != 0) return false; // Phải bắt đầu và kết thúc tại depot

    std::set<int> required_customers;
    for (int id : S_prime) {
        if (isCustomer(id, nodes)) {
            required_customers.insert(id);
        }
    }

    std::set<int> visited_customers;
    for (size_t i = 1; i < route.size() - 1; ++i) {
        if (route[i] == 0) return false; // Depot không được xuất hiện ở giữa
        if (isCustomer(route[i], nodes)) {
            if (visited_customers.count(route[i])) return false; // Khách hàng không được lặp lại
            visited_customers.insert(route[i]);
        }
    }
    return visited_customers == required_customers; // Phải ghé thăm tất cả khách hàng
}

// Hàm chọn lân cận ngẫu nhiên dựa trên trọng số
int selectWeightedNeighborhood(std::mt19937& gen, const std::vector<double>& weights) {
    std::discrete_distribution<> dist(weights.begin(), weights.end());
    return dist(gen);
}

// Hàm kiểm tra cải thiện đáng kể (≥ 0.01%)
bool checkSignificantImprovement(double old_cost, double new_cost) {
    if (old_cost <= 0) return false;
    double improvement = (old_cost - new_cost) / old_cost * 100.0;
    return improvement >= 0.01;
}

Route VNSOptimizer::run(int max_iterations) {
    // Khởi tạo random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    // Tạo danh sách khách hàng cần ghé thăm từ S_prime
    std::vector<int> initial_nodes;
    initial_nodes.push_back(0); // Depot đầu
    for (int id : S_prime) {
        if (isCustomer(id, nodes)) {
            initial_nodes.push_back(id);
        }
    }
    initial_nodes.push_back(0); // Depot cuối

    // Khởi tạo tuyến đường ban đầu và tối ưu hóa
    Route best_route(initial_nodes);
    best_route.optimize(arcs, charge_options, params, nodes, optimizer);
    Route current_route = best_route;

    // Khởi tạo trọng số toán tử
    std::vector<double> operator_weights = {0.4, 0.3, 0.2, 0.1}; // 2-opt, relocate, swap, station add/remove
    int no_improvement_counter = 0;

    // Số lượng tối đa các cấu trúc lân cận
    int k_max = 4; // TWO_OPT, RELOCATE, SWAP, INSERT_CHARGE/REMOVE_CHARGE

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Chọn lân cận ngẫu nhiên dựa trên trọng số
        int k = selectWeightedNeighborhood(gen, operator_weights) + 1;

        // Shaking: Tạo một tuyến đường lân cận ngẫu nhiên
        std::vector<int> s_prime_nodes;
        switch (k) {
            case 1:
                s_prime_nodes = generateRandomNeighbor(current_route.getNodeIds(), TWO_OPT, nodes);
                break;
            case 2:
                s_prime_nodes = generateRandomNeighbor(current_route.getNodeIds(), RELOCATE, nodes);
                break;
            case 3:
                s_prime_nodes = generateRandomNeighbor(current_route.getNodeIds(), SWAP, nodes);
                break;
            case 4:
                s_prime_nodes = generateRandomNeighbor(current_route.getNodeIds(), rand() % 2 ? INSERT_CHARGE : REMOVE_CHARGE, nodes);
                break;
            default:
                s_prime_nodes = current_route.getNodeIds();
        }
        Route s_prime(s_prime_nodes);
        s_prime.optimize(arcs, charge_options, params, nodes, optimizer);

        // Kiểm tra tính hợp lệ của tuyến đường
        if (!isValidRoute(s_prime.getNodeIds(), nodes, S_prime)) {
            no_improvement_counter++;
            continue;
        }

        // Local Search: Thực hiện tìm kiếm cục bộ trên s_prime
        Route best_local = s_prime;
        int num_trials = 10; // Số lần thử tìm kiếm cục bộ
        for (int t = 0; t < num_trials; ++t) {
            std::vector<int> neighbor_nodes = generateRandomNeighbor(best_local.getNodeIds(), TWO_OPT, nodes);
            Route neighbor(neighbor_nodes);
            neighbor.optimize(arcs, charge_options, params, nodes, optimizer);
            if (isValidRoute(neighbor.getNodeIds(), nodes, S_prime) &&
                neighbor.getIsFeasible() &&
                neighbor.getTotalCost() < best_local.getTotalCost()) {
                best_local = neighbor;
            }
        }
        Route s_double_prime = best_local;

        // Kiểm tra tiêu chí chấp nhận
        bool significant_improvement = false;
        if (s_double_prime.getIsFeasible() &&
            isValidRoute(s_double_prime.getNodeIds(), nodes, S_prime)) {
            if (s_double_prime.getTotalCost() < best_route.getTotalCost()) {
                // Cải thiện toàn cục
                best_route = s_double_prime;
                current_route = s_double_prime;
                significant_improvement = checkSignificantImprovement(best_route.getTotalCost(), s_double_prime.getTotalCost());
                no_improvement_counter = significant_improvement ? 0 : no_improvement_counter + 1;
                // Cập nhật trọng số toán tử
                operator_weights[k-1] += 0.1; // Tăng trọng số của toán tử thành công
                double sum_weights = 0;
                for (double w : operator_weights) sum_weights += w;
                for (double& w : operator_weights) w /= sum_weights; // Chuẩn hóa
            } else if (s_double_prime.getTotalCost() < current_route.getTotalCost()) {
                // Cải thiện cục bộ
                current_route = s_double_prime;
                no_improvement_counter++;
            } else {
                no_improvement_counter++;
            }
        } else {
            no_improvement_counter++;
        }

        // In thông tin tuyến đường tốt nhất mỗi 10 vòng lặp
        if ((iter + 1) % 10 == 0) {
            std::cout << "Iteration " << iter + 1 << ":\n";
            std::cout << "Best Route:\n";
            const auto& node_ids = best_route.getNodeIds();
            std::cout << "  Route: ";
            for (size_t i = 0; i < node_ids.size(); ++i) {
                std::string type = node_ids.at(i) == 0 ? "d" : (isCustomer(node_ids[i], nodes) ? "c" : "f");
                std::cout << node_ids[i] << "(" << type << ")";
                if (i < node_ids.size() - 1) {
                    std::cout << " -> ";
                }
            }
            std::cout << "\n";
            int num_customers = 0, num_stations = 0;
            for (size_t i = 1; i < node_ids.size() - 1; ++i) {
                if (isCustomer(node_ids[i], nodes)) num_customers++;
                if (isChargingStation(node_ids[i], nodes)) num_stations++;
            }
            std::cout << "  Number of Customers: " << num_customers << "\n";
            std::cout << "  Number of Charging Stations: " << num_stations << "\n";
            std::cout << "  Total Cost: " << std::fixed << std::setprecision(2) << best_route.getTotalCost() << "\n";
            std::cout << "  Feasible: " << (best_route.getIsFeasible() ? "Yes" : "No") << "\n";
            std::cout << "  No-Improvement Counter: " << no_improvement_counter << "\n";
            std::cout << "------------------------\n";
        }
    }

    return best_route;
}