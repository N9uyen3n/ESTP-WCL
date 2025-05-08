#ifndef MILP_H
#define MILP_H

#include <ilcplex/ilocplex.h>
#include <vector>
#include <string>

// Giả định các file header khác đã được định nghĩa
#include "Route.h"
#include "Graph.h"
#include "Parameters.h"
#include "Utils.h"

struct SubproblemResult {
    double cost;                            // Chi phí tổng của tuyến đường
    std::vector<int> new_node_ids;          // Danh sách các nút của tuyến đường
    std::vector<double> soc_arrival;        // SOC khi đến mỗi nút
    std::vector<double> soc_departure;      // SOC khi rời mỗi nút
    std::vector<double> arrival_time;       // Thời gian đến mỗi nút
    std::vector<double> departure_time;     // Thời gian rời mỗi nút
    std::vector<ChargingDecision> charging_decisions; // Quyết định sạc tĩnh
    std::vector<bool> wireless_decisions;   // Quyết định sạc không dây cho mỗi cung
    bool feasible;                          // Tính khả thi của tuyến đường
};

class MILP {
public:
    MILP();
    ~MILP();

    Route optimize(const std::vector<int>& initial_nodes,
                   const Graph& graph,
                   const std::vector<std::vector<ChargingOption>>& charge_options,
                   const Parameters& params);

    Route solveSubproblem(int i, int j, int a, const Route& current_route,
                         const Graph& graph,
                         const std::vector<std::vector<ChargingOption>>& charge_options,
                         const Parameters& params);

private:
    IloEnv env;

    void updateRoute(Route& route, const SubproblemResult& result);
};

#endif // MILP_H