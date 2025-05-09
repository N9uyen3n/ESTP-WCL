#ifndef VNS_H
#define VNS_H

#include "Graph.h"
#include "Route.h"
#include "Parameters.h"
#include <ilcplex/ilocplex.h>
#include <random>
#include <vector>
#include <string>
#include <set>

struct ProblemInfos {
    // Saved information about the problem
    //The value here will be saved every time the algorithm updates and uses this to know the final parameters
    double cost; // Total cost of the route
    std::vector<int> new_node_ids; // List of node IDs in the route (May be have stations)
    std::vector<double> soc_arrival; // SOC at arrival at each node
    std::vector<double> soc_departure; // SOC at departure from each node
    std::vector<double> arrival_time; // Arrival time at each node
    std::vector<double> departure_time; // Departure time from each node
    std::vector<ChargingDecision> charging_decisions; // Charging decisions at each node (ChargingOption in each station)
    std::vector<bool> wireless_decisions; // Wireless charging decisions for each arc wireless
    bool feasible; // Feasibility of the route
};

class VNS {
public:
    VNS(const std::vector<int>& initial_route,
        const Graph& graph,
        const std::vector<std::vector<ChargingOption>>& charge_options,
        const Parameters& params,
        int max_iterations,
        std::mt19937& rng); // Constructor
    ~VNS();

    Route run();

private:
    IloEnv env; // Cplex environment
    std::vector<int> init_route; // Initial route
    Route current_route; // Current route will be updated in the algorithm VNS
    Graph graph; // Graph object has all nodes and arcs
    std::vector<std::vector<ChargingOption>> charge_options; // Charging options for each node
    Parameters params; // Parameters object has all parameters
    int max_iterations; // Maximum iterations for VNS
    std::mt19937& rng; // Random number generator
    std::vector<double> operator_weights_; // Weights for each operator
    int no_improvement_counter; // Counter for no improvement in the solution

    Route solveSubproblem(const std::vector<int>& initial_nodes,
                                    const Graph& graph,
                                    const std::vector<std::vector<ChargingOption>>& charge_options,
                                    const Parameters& params); // Solve the subproblem using CPLEX with fixed customer sequence

    Route localSearch(const Route& current_route); // Perform local search on the current route

    // Shake the current route to create a new solution, return a new route by applying the operator
    Route shake(const Route& current_route, int neighborhood,
                const Graph& graph, const Parameters& params); // Shake the current route to create a new solution

    // Local search operators, return a new route by applying the operator
    Route swapNodes(const Route& route, const Graph& graph, const Parameters& params);
    Route relocate(const Route& route, const Graph& graph, const Parameters& params);
    Route insertStation(const Route& route, const Graph& graph, const Parameters& params);
    Route removeStation(const Route& route, const Graph& graph, const Parameters& params);
    Route twoOpt(const Route& route, const Graph& graph, const Parameters& params);

    // Update the route with the new information from the subproblem
    void updateRoute(Route& route, const ProblemInfos& result);

    // Update the weights of the operators based on the performance
    void normalizeWeights();
    // Select an operator based on the weights
    int selectOperatorWeighted();

    // Check if the route is valid
    bool isValidRoute(const std::vector<int>& route, const Graph& graph, const std::vector<int>& initial_nodes);
    // Quick feasibility check for the route
    bool quickFeasibilityCheck(const Route& route, const Graph& graph, const Parameters& params);
};

#endif // VNS_H