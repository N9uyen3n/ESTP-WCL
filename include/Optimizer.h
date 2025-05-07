#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <vector>
#include "Route.h"
#include "Graph.h"
#include "Parameters.h"

struct SubproblemResult {
    double cost;                            // Chi phí tổng của tuyến đường
    std::vector<int> new_node_ids;          // Danh sách các nút của tuyến đường
    std::vector<double> soc_arrival;        // SOC khi đến mỗi nút
    std::vector<double> soc_departure;      // SOC khi rời mỗi nút
    std::vector<double> arrival_time;       // Thời gian đến mỗi nút
    std::vector<double> departure_time;     // Thời gian rời mỗi nút
    std::vector<ChargingDecision> charging_decisions; // Quyết định sạc tĩnh
    std::vector<bool> wireless_decisions;   // Quyết định sạc không dây cho mỗi cung
};

class Optimizer {
public:
    virtual ~Optimizer() = default;

    // Hàm tối ưu hóa chính
    virtual Route optimize(const std::vector<int>& initial_nodes,
                          const Graph& graph,
                          const std::vector<std::vector<ChargingOption>>& charge_options,
                          const Parameters& params) = 0;

    // Hàm giải subproblem cho  MILP Full
    virtual Route solveSubproblem(int i, int j, int a, const Route& current_route,
                                 const Graph& graph,
                                 const std::vector<std::vector<ChargingOption>>& charge_options,
                                 const Parameters& params) = 0;
    // Hàm giải subproblem cho VNS
    virtual Route solveSubproblem1(const Route& current_route,
                           const Graph& graph,
                           const std::vector<std::vector<ChargingOption>>& charge_options,
                           const Parameters& params);
};

#endif // OPTIMIZER_H