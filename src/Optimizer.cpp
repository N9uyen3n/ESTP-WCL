//
// Created by Admin on 07/05/2025.
//

#include "../include/Optimizer.h"
#include "../include/Route.h"
#include "../include/Graph.h"
#include "../include/Parameters.h"
#include <stdexcept>

namespace OptimizerUtils {
// Hàm hỗ trợ để kiểm tra tính hợp lệ của danh sách nút ban đầu
bool validateInitialNodes(const std::vector<int>& initial_nodes, const Graph& graph) {
    if (initial_nodes.empty()) {
        return false;
    }

    // Kiểm tra xem tất cả các nút có tồn tại trong đồ thị không
    for (int id : initial_nodes) {
        bool found = false;
        for (const auto& node : graph.getNodes()) {
            if (node.getId() == id) {
                found = true;
                break;
            }
        }
        if (!found) {
            return false;
        }
    }

    // Kiểm tra depot bắt đầu và kết thúc
    bool has_start_depot = false, has_end_depot = false;
    for (const auto& node : graph.getNodes()) {
        if (node.getType() == NodeType::DEPOT) {
            if (node.getStringId() == "D0" && initial_nodes.front() == node.getId()) {
                has_start_depot = true;
            }
            if (node.getStringId() == "D1" && initial_nodes.back() == node.getId()) {
                has_end_depot = true;
            }
        }
    }

    return has_start_depot && has_end_depot;
}

// Hàm hỗ trợ để tạo tuyến đường ban đầu từ danh sách nút
Route createInitialRoute(const std::vector<int>& initial_nodes, const Graph& graph, const Parameters& params) {
    Route route(initial_nodes);

    // Khởi tạo các giá trị mặc định
    std::vector<double> soc_arrival(initial_nodes.size(), params.getInitialSoc());
    std::vector<double> soc_departure(initial_nodes.size(), params.getInitialSoc());
    std::vector<double> arrival_time(initial_nodes.size(), 0.0);
    std::vector<double> departure_time(initial_nodes.size(), 0.0);
    std::vector<ChargingDecision> charging_decisions;
    std::vector<bool> wireless_decisions(initial_nodes.size() - 1, false);

    // Thiết lập SOC và thời gian ban đầu
    soc_arrival[0] = params.getInitialSoc();
    soc_departure[0] = params.getInitialSoc();

    // Kiểm tra tính khả thi đơn giản (có thể mở rộng)
    if (!route.checkFeasibility(graph, params)) {
        route.setFeasible(false);
    }

    route.setSocArrival(soc_arrival);
    route.setSocDeparture(soc_departure);
    route.setArrivalTime(arrival_time);
    route.setDepartureTime(departure_time);
    route.setChargingDecisions(charging_decisions);
    route.setWirelessDecisions(wireless_decisions);

    return route;
}
} // namespace OptimizerUtils

Route Optimizer::solveSubproblem1(const Route& current_route,
                                 const Graph& graph,
                                 const std::vector<std::vector<ChargingOption>>& charge_options,
                                 const Parameters& params) {
    // Base implementation - should be overridden by derived classes
    return current_route;
}

// Vì Optimizer là lớp trừu tượng, không có triển khai cụ thể ở đây.
// Các lớp con (VNSOptimizer, MILPOptimizer) sẽ ghi đè các phương thức thuần ảo.