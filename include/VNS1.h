#ifndef VNS1_H
#define VNS1_H

#include "Graph.h"
#include "Route.h"
#include "Parameters.h"
#include <ilcplex/ilocplex.h>
#include <random>
#include <vector>
#include <string>
#include <set>

struct ProblemInfos {
    double cost;
    std::vector<int> new_node_ids;
    std::vector<double> soc_arrival;
    std::vector<double> soc_departure;
    std::vector<double> arrival_time;
    std::vector<double> departure_time;
    std::vector<ChargingDecision> charging_decisions;
    std::vector<bool> wireless_decisions;
    std::vector<std::pair<int, int>> arc_wireless_decisions;
    bool feasible;
};

class VNS1 {
public:
    VNS1(const std::vector<int>& initial_route,
         const Graph& graph,
         const std::vector<std::vector<ChargingOption>>& charge_options,
         const Parameters& params,
         int max_iterations,
         std::mt19937& rng);
    ~VNS1();
    Route run();

private:
    IloEnv env;
    std::vector<int> init_route;
    Route current_route;
    Graph graph;
    std::vector<std::vector<ChargingOption>> charge_options;
    Parameters params;
    int max_iterations;
    std::mt19937& rng;
    std::vector<double> operator_weights_;
    int no_improvement_counter;

    Route solveSubproblem(const std::vector<int>& initial_nodes);
    Route localSearch(const Route& current_route);
    Route shake(const Route& route, int neighborhood);
    Route twoOpt(const Route& route);
    Route swapNodes(const Route& route);
    Route relocate(const Route& route);
    std::vector<int> getCustomerIndices(const Route& route);

    void updateRoute(Route& route, const ProblemInfos& result);
    void normalizeWeights();
    int selectOperatorWeighted();
    bool isValidRoute(const std::vector<int>& route);
    bool quickFeasibilityCheck(const Route& route);
};

#endif // VNS1_H