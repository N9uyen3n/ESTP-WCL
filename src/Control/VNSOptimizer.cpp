//
// Created by Admin on 29/04/2025.
//

#include "../../include/VNSOptimizer.h"
#include "../../include/Utils.h"
#include <random>
#include <iostream>
#include <set>

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

    // Số lượng tối đa các cấu trúc lân cận
    int k_max = 4; // TWO_OPT, RELOCATE, INSERT_CHARGE, REMOVE_CHARGE

    for (int iter = 0; iter < max_iterations; ++iter) {
        int k = 1;
        while (k <= k_max) {
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
                    s_prime_nodes = generateRandomNeighbor(current_route.getNodeIds(), INSERT_CHARGE, nodes);
                    break;
                case 4:
                    s_prime_nodes = generateRandomNeighbor(current_route.getNodeIds(), REMOVE_CHARGE, nodes);
                    break;
                default:
                    s_prime_nodes = current_route.getNodeIds();
            }
            Route s_prime(s_prime_nodes);
            s_prime.optimize(arcs, charge_options, params, nodes, optimizer);

            // Kiểm tra tính hợp lệ của tuyến đường
            if (!isValidRoute(s_prime.getNodeIds(), nodes, S_prime)) {
                k++;
                continue;
            }

            // Local Search: Thực hiện tìm kiếm cục bộ trên s_prime bằng TWO_OPT
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

            // Kiểm tra xem s_double_prime có cải thiện current_route không
            if (s_double_prime.getIsFeasible() &&
                isValidRoute(s_double_prime.getNodeIds(), nodes, S_prime) &&
                s_double_prime.getTotalCost() < current_route.getTotalCost()) {
                current_route = s_double_prime;
                if (s_double_prime.getTotalCost() < best_route.getTotalCost()) {
                    best_route = s_double_prime;
                }
                k = 1; // Reset về cấu trúc lân cận đầu tiên
            } else {
                k++; // Chuyển sang cấu trúc lân cận tiếp theo
            }
        }

        // In thông tin tuyến đường tốt nhất mỗi 10 vòng lặp
        if ((iter + 1) % 10 == 0) {
            std::cout << "Iteration " << iter + 1 << ":\n";
            std::cout << "Best Route:\n";
            best_route.print();
            std::cout << "Total Cost: " << best_route.getTotalCost() << "\n";
            std::cout << "Feasible: " << (best_route.getIsFeasible() ? "Yes" : "No") << "\n";
            std::cout << "------------------------\n";
        }
    }

    return best_route;
}