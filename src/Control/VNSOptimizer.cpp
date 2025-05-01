//
// Created by Admin on 29/04/2025.
//

#include "../../include/VNSOptimizer.h"
#include "../../include/Utils.h"
#include <random>

VNSOptimizer::VNSOptimizer( const std::vector<int>& S_prime,
                            const std::vector<Node>& nodes,
                            const std::vector<Arc>& ars,
                            const std::vector<std::vector<ChargingOption>>& chg_opts,
                            const Params& params)
    : S_prime(S_prime),nodes(nodes), arcs(ars), charge_options(chg_opts), params(params) {}

Route VNSOptimizer::run(int max_iterations) {
    // Khởi tạo tuyến đường ban đầu
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<int> initial_nodes = S_prime;
    Route best_route(initial_nodes);
    best_route.optimize(arcs, charge_options, params, nodes, optimizer);
    Route current_route = best_route;

    int k_max = 2; // Số lượng cấu trúc lân cận
    for (int iter = 0; iter < max_iterations; ++iter) {
        int k = 1;
        while (k <= k_max) {
            // Shaking: Tạo lân cận ngẫu nhiên sử dụng cấu trúc lân cận k
            std::vector<int> s_prime_nodes;
            if (k == 1) {
                s_prime_nodes = generateRandomNeighbor(current_route.getNodeIds(), TWO_OPT, nodes);
            } else if (k == 2) {
                s_prime_nodes = generateRandomNeighbor(current_route.getNodeIds(), RELOCATE, nodes);
            }
            Route s_prime(s_prime_nodes);
            s_prime.optimize(arcs, charge_options, params, nodes, optimizer);

            // Local search: Tìm kiếm địa phương trên s_prime
            Route best_local = s_prime;
            int num_trials = 10; // Số lần thử tìm kiếm địa phương
            for (int t = 0; t < num_trials; ++t) {
                std::vector<int> neighbor_nodes;
                if (k == 1) {
                    neighbor_nodes = generateRandomNeighbor(s_prime_nodes, TWO_OPT, nodes);
                } else if (k == 2) {
                    neighbor_nodes = generateRandomNeighbor(s_prime_nodes, RELOCATE, nodes);
                }
                Route neighbor(neighbor_nodes);
                neighbor.optimize(arcs, charge_options, params, nodes, optimizer);
                if (neighbor.getIsFeasible() && neighbor.getTotalCost() < best_local.getTotalCost()) {
                    best_local = neighbor;
                }
            }
            Route s_double_prime = best_local;

            // Kiểm tra nếu s_double_prime tốt hơn current_route
            if (s_double_prime.getIsFeasible() && s_double_prime.getTotalCost() < current_route.getTotalCost()) {
                current_route = s_double_prime;
                if (s_double_prime.getTotalCost() < best_route.getTotalCost()) {
                    best_route = s_double_prime;
                }
                k = 1; // Đặt lại k về 1
            } else {
                k++;
            }
        }
    }
    return best_route;
}

