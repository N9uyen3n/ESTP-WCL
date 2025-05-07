#ifndef VNS_H
#define VNS_H

#include "Graph.h"
#include "Route.h"
#include "Parameters.h"
#include <ilcplex/ilocplex.h>
#include <random>
#include <vector>

struct SubproblemResult {
    double cost;                            // Total cost of the route
    std::vector<int> new_node_ids;          // List of node IDs in the route
    std::vector<double> soc_arrival;        // SOC upon arrival at each node
    std::vector<double> soc_departure;      // SOC upon departure from each node
    std::vector<double> arrival_time;       // Arrival time at each node
    std::vector<double> departure_time;     // Departure time from each node
    std::vector<ChargingDecision> charging_decisions; // Static charging decisions
    std::vector<bool> wireless_decisions;   // Wireless charging decisions for each arc
};

class VNS {
public:
    VNS(int max_iterations, int max_neighborhoods, unsigned int seed, const std::vector<int>& s_prime);
    ~VNS();

    Route optimize(const std::vector<int>& initial_nodes,
                   const Graph& graph,
                   const std::vector<std::vector<ChargingOption>>& charge_options,
                   const Parameters& params);

private:
    IloEnv env;
    int max_iterations;
    int max_neighborhoods;
    std::mt19937 rng;
    std::vector<int> S_prime;
    std::vector<double> operator_weights_;

    // double evaluateRoute(Route& route,
    //                      const Graph& graph,
    //                      const std::vector<std::vector<ChargingOption>>& charge_options,
    //                      const Parameters& params);

    Route solveSubproblem(int i, int j, int a, const Route& current_route,
                         const Graph& graph,
                         const std::vector<std::vector<ChargingOption>>& charge_options,
                         const Parameters& params);

    Route solveSubproblem1(const Route& current_route,
                           const Graph& graph,
                           const std::vector<std::vector<ChargingOption>>& charge_options,
                           const Parameters& params);

    Route localSearch(const Route& current_route,
                      const Graph& graph,
                      const std::vector<std::vector<ChargingOption>>& charge_options,
                      const Parameters& params);

    Route shake(const Route& current_route, int neighborhood,
                const Graph& graph, const Parameters& params);

    Route swapNodes(const Route& route, const Graph& graph, const Parameters& params);
    Route relocate(const Route& route, const Graph& graph, const Parameters& params);
    Route insertStation(const Route& route, const Graph& graph, const Parameters& params);
    Route removeStation(const Route& route, const Graph& graph, const Parameters& params);
    Route twoOpt(const Route& route, const Graph& graph, const Parameters& params);

    void updateRoute(Route& route, const SubproblemResult& result);
    Route evaluateRoute(Route& route,
                       const Graph& graph,
                       const std::vector<std::vector<ChargingOption>>& charge_options,
                       const Parameters& params);

    void normalizeWeights();
    bool checkSignificantImprovement(double old_cost, double new_cost);
    int selectOperatorWeighted();
    bool isValidRoute(const std::vector<int>& route, const Graph& graph);
    bool quickFeasibilityCheck(const Route& route, const Graph& graph, const Parameters& params);
};

#endif