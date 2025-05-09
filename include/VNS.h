#ifndef VNS_H
#define VNS_H

#include "Graph.h"
#include "Route.h"
#include "Parameters.h"
#include <ilcplex/ilocplex.h>
#include <random>
#include <vector>
#include <string> // Added for std::string
#include <set>    // Added for std::set

struct SubproblemResult {
    double cost;                            // Total cost of the route
    std::vector<int> new_node_ids;          // List of node IDs in the route
    std::vector<double> soc_arrival;        // SOC upon arrival at each node
    std::vector<double> soc_departure;      // SOC upon departure from each node
    std::vector<double> arrival_time;       // Arrival time at each node
    std::vector<double> departure_time;     // Departure time from each node
    std::vector<ChargingDecision> charging_decisions; // Static charging decisions
    std::vector<bool> wireless_decisions;   // Wireless charging decisions for each arc
    bool feasible;                          // Whether the subproblem found a feasible solution
};

class VNS {
public:
    VNS(const std::vector<int>& initial_route,
         const Graph& graph,
         const std::vector<std::vector<ChargingOption>>& charge_options,
         const Parameters& params,
         int max_iterations,
         std::mt19937& rng);
    ~VNS();

    Route run(); // dùng để chạy vòng lặp VNS

private:
    IloEnv env;
    std::vector<int> init_route;
    std::vector<int> current_route;
    Graph graph;
    std::vector<std::vector<ChargingOption>> charge_options;
    Parameters params;
    int max_iterations;
    std::mt19937& rng;
    std::vector<double> operator_weights_; // Trọng số cho các cấu trúc hàng xóm
    int no_improvement_counter; // Đếm số lần không cải thiện
    // int max_neighborhoods;


    SubproblemResult solveSubproblem(const Route& current_route,
                               const Graph& graph,
                               const std::vector<std::vector<ChargingOption>>& charge_options,
                               const Parameters& params); // MILP Sub problem trong VNS

    Route localSearch(const Route& current_route); // hàm tìm kiếm địa phương

    Route shake(const Route& current_route, int neighborhood,
                const Graph& graph, const Parameters& params); // Shake

    // Các câ trúc hàng xóm
    Route swapNodes(const Route& route, const Graph& graph, const Parameters& params);
    Route relocate(const Route& route, const Graph& graph, const Parameters& params);
    Route insertStation(const Route& route, const Graph& graph, const Parameters& params);
    Route removeStation(const Route& route, const Graph& graph, const Parameters& params);
    Route twoOpt(const Route& route, const Graph& graph, const Parameters& params);

    // Cập nhật cấu trúc
    void updateRoute(Route& route, const SubproblemResult& result);

    // Điê chỉnh trọng số cho từng cấu trúc hàng xóm
    void normalizeWeights();
    // Chọn trọng số
    int selectOperatorWeighted();



    // Kiểm tra tính khả thi
    bool isValidRoute(const std::vector<int>& route, const Graph& graph, const std::vector<int>& initial_nodes);
    // bool isValidRoute(const std::vector<int>& route_node_ids, const Graph& graph, const Parameters& params); // Added params
    // Kiểm tra tính khả thi nhanh
    bool quickFeasibilityCheck(const Route& route, const Graph& graph, const Parameters& params);
};

#endif // VNS_H