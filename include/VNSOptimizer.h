#ifndef VNS_OPTIMIZER_H
#define VNS_OPTIMIZER_H

#include "Optimizer.h"
#include "Route.h"
#include "Graph.h"
#include "Parameters.h"

#include <ilcplex/ilocplex.h>
#include <vector>
#include <random>


class VNSOptimizer : public Optimizer {
public:
    // Constructor: Initializes VNS with problem data and random seed
    VNSOptimizer(const std::vector<int>& S_prime,
                 const std::vector<Node>& nodes,
                 const std::vector<Arc>& arcs,
                 const std::vector<std::vector<ChargingOption>>& chg_opts,
                 const Parameters& params);

    // Optimizes the route using VNS algorithm
    Route optimize(const std::vector<int>& initial_nodes,
                   const Graph& graph,
                   const std::vector<std::vector<ChargingOption>>& charge_options,
                   const Parameters& params) override;

    // Solves MIP subproblem for segment (i,j) with charging station a
    Route solveSubproblem(int i, int j, int a, const Route& current_route,
                         const Graph& graph,
                         const std::vector<std::vector<ChargingOption>>& charge_options,
                         const Parameters& params) override;

    // Solves MIP subproblem for the entire route
    Route solveSubproblem1(const Route& current_route,
                           const Graph& graph,
                           const std::vector<std::vector<ChargingOption>>& charge_options,
                           const Parameters& params) override;
    Route VNSOptimizer::run(int max_iterations);

private:
    std::vector<int> S_prime; // Required customer nodes
    std::vector<Node> nodes; // All nodes
    std::vector<Arc> arcs; // All arcs
    std::vector<std::vector<ChargingOption>> charge_options; // Charging options per node
    Parameters params; // Problem parameters
    Graph graph_; // Graph instance for arc and node lookups
    mutable std::mt19937 rng_; // Random number generator
    std::vector<double> operator_weights_; // Weights for neighborhood operators
    Route best_route_; // Best route found
    int no_improvement_counter_; // Counter for iterations without improvement


    // Creates an initial feasible route
    Route createInitialRoute();
    // Checks if a route is valid (visits all required customers, starts/ends at depot)
    bool isValidRoute(const std::vector<int>& route) const;
    // Selects a neighborhood operator based on weights
    int selectOperatorWeighted();
    // Normalizes operator weights to sum to 1
    void normalizeWeights();
    // Checks if cost improvement is significant
    bool checkSignificantImprovement(double old_cost, double new_cost) const;
    // Performs quick feasibility check (SOC constraints)
    bool quickFeasibilityCheck(Route& route) const;
    // Performs local search using MILP
    Route localSearchPhase(const Route& candidate_route);
    // Evaluates a route's cost and feasibility
    Route evaluateRoute(Route& route);
    // Applies shaking by selecting a neighborhood
    Route shake(const Route& current_route, int neighborhood);
    // 2-opt neighborhood operator
    Route twoOpt(const Route& route);
    // Relocate neighborhood operator
    Route relocate(const Route& route);
    // Swap nodes neighborhood operator
    Route swapNodes(const Route& route);
    // Add or remove charging station neighborhood operator
    Route stationAdditionRemoval(const Route& route);
    // Solves MILP subproblem for the entire route
    Route solveSubproblemMILP(const Route& route);
    // Updates route based on subproblem result
    void updateRoute(Route& route, const SubproblemResult& result);
};

#endif