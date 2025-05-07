#include "../include/VNS.h"
#include "../include/Utils.h" // Assuming Utils.h contains isChargingStation, etc.
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <random>
#include <chrono>
#include <set>
#include <numeric>
#include <iomanip> // For std::fixed and std::setprecision
#include <map>     // For std::map in solveSubproblem1

// Constructor
VNS::VNS(int max_iterations, int max_neighborhoods, unsigned int seed, const std::vector<int>& required_customer_ids)
    : max_iterations(max_iterations), max_neighborhoods(max_neighborhoods), // max_neighborhoods currently unused
      rng(std::mt19937(seed)), required_customer_ids_(required_customer_ids),
      operator_weights_{0.4, 0.3, 0.2, 0.1, 0.1}, // Adjusted if more operators
      no_improvement_counter(0) {
    std::cout << "[DEBUG] VNS Constructor: Initializing..." << std::endl;
    normalizeWeights();
    std::cout << "[DEBUG] VNS Constructor: operator_weights_ normalized." << std::endl;
    // Note: CPLEX environment (env) is initialized by its default constructor.
    std::cout << "[DEBUG] VNS Constructor: CPLEX env initialized by default." << std::endl;
    std::cout << "[DEBUG] VNS Constructor: Done." << std::endl;
}

// Destructor
VNS::~VNS() {
    std::cout << "[DEBUG] VNS Destructor: Cleaning up CPLEX environment..." << std::endl;
    env.end(); // Clean up CPLEX environment
    std::cout << "[DEBUG] VNS Destructor: CPLEX environment cleaned up." << std::endl;
}

void VNS::normalizeWeights() {
    // std::cout << "[DEBUG] normalizeWeights: Normalizing weights..." << std::endl;
    double sum = std::accumulate(operator_weights_.begin(), operator_weights_.end(), 0.0);
    if (sum > 1e-9) {
        for (double& w : operator_weights_) {
            w /= sum;
        }
    } else if (!operator_weights_.empty()) { // Avoid division by zero if weights_ is empty
        // std::cout << "[DEBUG] normalizeWeights: Sum is near zero, re-initializing weights equally." << std::endl;
        double val = 1.0 / operator_weights_.size();
        for (double& w : operator_weights_) {
            w = val;
        }
    }
    // std::cout << "[DEBUG] normalizeWeights: Done." << std::endl;
}

bool VNS::checkSignificantImprovement(double old_cost, double new_cost, double threshold) {
    // std::cout << "[DEBUG] checkSignificantImprovement: Old=" << old_cost << ", New=" << new_cost << std::endl;
    if (old_cost <= 1e9 -1) { // Check if old_cost is a feasible cost
         return (old_cost - new_cost) / old_cost > threshold;
    }
    // std::cout << "[DEBUG] checkSignificantImprovement: Old cost was infeasible, any feasible new_cost is improvement." << std::endl;
    return true; // If old_cost was infeasible, any feasible new_cost is an improvement
}

int VNS::selectOperatorWeighted() {
    // std::cout << "[DEBUG] selectOperatorWeighted: Selecting operator..." << std::endl;
    std::discrete_distribution<int> dist(operator_weights_.begin(), operator_weights_.end());
    int selected_op = dist(rng);
    // std::cout << "[DEBUG] selectOperatorWeighted: Selected operator index: " << selected_op << std::endl;
    return selected_op;
}

// isValidRoute: Checks basic validity of a node sequence
bool VNS::isValidRoute(const std::vector<int>& route_node_ids, const Graph& graph, const Parameters& params) {
    std::cout << "[DEBUG] isValidRoute: Checking route validity. Route size: " << route_node_ids.size() << std::endl;
    if (route_node_ids.size() < 2) {
        std::cout << "[DEBUG] isValidRoute: Route size < 2. Invalid." << std::endl;
        return false;
    }

    int depot_id = 0; // Placeholder, ideally from params

    if (route_node_ids.front() != depot_id || route_node_ids.back() != depot_id) {
        std::cout << "[DEBUG] isValidRoute: Route does not start or end at depot. Front: " << route_node_ids.front() << ", Back: " << route_node_ids.back() << ". Invalid." << std::endl;
        return false;
    }

    std::set<int> visited_customers;
    std::set<int> required_customers_set(required_customer_ids_.begin(), required_customer_ids_.end());

    for (size_t i = 1; i < route_node_ids.size() - 1; ++i) {
        std::cout << "[DEBUG] isValidRoute: Checking node ID " << route_node_ids[i] << " at index " << i << std::endl;
        const Node* node = graph.findNode(route_node_ids[i]);
        if (!node) {
            std::cerr << "[ERROR_DEBUG] isValidRoute: Node with ID " << route_node_ids[i] << " not found in graph. Invalid." << std::endl;
            return false; // Node does not exist
        }
        if (node->getType() == NodeType::CUSTOMER) {
            if (visited_customers.count(node->getId())) {
                std::cerr << "[ERROR_DEBUG] isValidRoute: Customer " << node->getId() << " visited more than once. Invalid." << std::endl;
                return false; // Customer visited more than once
            }
            visited_customers.insert(node->getId());
        }
    }

    if (visited_customers != required_customers_set) {
        std::cout << "[DEBUG] isValidRoute: Not all required customers are visited, or extra customers visited. Invalid." << std::endl;
        std::cout << "[DEBUG] isValidRoute: Visited customers: ";
        for(int cust_id : visited_customers) std::cout << cust_id << " ";
        std::cout << std::endl;
        std::cout << "[DEBUG] isValidRoute: Required customers: ";
        for(int cust_id : required_customers_set) std::cout << cust_id << " ";
        std::cout << std::endl;
        return false;
    }
    std::cout << "[DEBUG] isValidRoute: Route is valid." << std::endl;
    return true;
}

// quickFeasibilityCheck: Performs a simplified SOC simulation
bool VNS::quickFeasibilityCheck(const Route& route, const Graph& graph, const Parameters& params) {
    std::cout << "[DEBUG] quickFeasibilityCheck: Starting for route. Node count: " << route.getNodeIds().size() << std::endl;
    const std::vector<int>& node_ids = route.getNodeIds();
    if (node_ids.size() < 2) {
        std::cout << "[DEBUG] quickFeasibilityCheck: Route size < 2. Trivially feasible." << std::endl;
        return true;
    }

    double current_soc = params.getInitialSoc();
    std::cout << "[DEBUG] quickFeasibilityCheck: Initial SOC: " << current_soc << std::endl;

    for (size_t i = 0; i < node_ids.size() - 1; ++i) {
        std::cout << "[DEBUG] quickFeasibilityCheck: Processing arc from node " << node_ids[i] << " to " << node_ids[i+1] << std::endl;
        const Node* from_node = graph.findNode(node_ids[i]);
        const Node* to_node = graph.findNode(node_ids[i+1]);
        if (!from_node) {
            std::cerr << "[ERROR_DEBUG] quickFeasibilityCheck: from_node " << node_ids[i] << " is NULL. Infeasible." << std::endl;
            return false;
        }
        if (!to_node) {
            std::cerr << "[ERROR_DEBUG] quickFeasibilityCheck: to_node " << node_ids[i+1] << " is NULL. Infeasible." << std::endl;
            return false;
        }

        const Arc* arc = graph.findArc(from_node->getId(), to_node->getId());
        if (!arc) {
            std::cerr << "[ERROR_DEBUG] quickFeasibilityCheck: Arc from " << from_node->getId() << " to " << to_node->getId() << " not found. Infeasible." << std::endl;
            return false;
        }

        double distance = arc->getDistance();
        double energy_consumed = params.getEnergyConsumption() * distance;
        current_soc -= energy_consumed;
        std::cout << "[DEBUG] quickFeasibilityCheck: Arc dist " << distance << ", energy consumed " << energy_consumed << ", SOC after consumption: " << current_soc << std::endl;

        if (arc->getIsWireless()) {
            double wireless_charge_on_arc = arc->getWirelessChargeRate() * arc->getTravelTime(); // Assuming TravelTime on arc is fixed here for simplicity
            current_soc += wireless_charge_on_arc;
            std::cout << "[DEBUG] quickFeasibilityCheck: Wireless arc. Charge on arc: " << wireless_charge_on_arc << ", SOC after wireless: " << current_soc << std::endl;
        }

        current_soc = std::min(current_soc, params.getBatteryCapacity());
        std::cout << "[DEBUG] quickFeasibilityCheck: SOC capped at battery capacity: " << current_soc << std::endl;

        if (current_soc < params.getMinSoc() - 1e-6) {
            std::cout << "[DEBUG] quickFeasibilityCheck: SOC " << current_soc << " is below MinSOC " << params.getMinSoc() << std::endl;
            if (Utils::isChargingStation(to_node->getId(), graph.getNodes())) { // Assumes Utils.h has this static method
                current_soc = params.getBatteryCapacity();
                std::cout << "[DEBUG] quickFeasibilityCheck: Next node " << to_node->getId() << " is charging station. Assuming full charge. SOC becomes " << current_soc << std::endl;
            } else {
                std::cout << "[DEBUG] quickFeasibilityCheck: Next node " << to_node->getId() << " is NOT a station. Infeasible." << std::endl;
                return false;
            }
        }
    }
    bool final_check = current_soc >= params.getMinSoc() - 1e-6;
    std::cout << "[DEBUG] quickFeasibilityCheck: Final SOC: " << current_soc << ". Feasible: " << final_check << std::endl;
    return final_check;
}


// evaluateRoute: Evaluates a given route, now calls localSearch
Route VNS::evaluateRoute(const Route& route,
                         const Graph& graph,
                         const std::vector<std::vector<ChargingOption>>& charge_options,
                         const Parameters& params) {
    std::cout << "[DEBUG] evaluateRoute: Evaluating route..." << std::endl;
    Route result_route = localSearch(route, graph, charge_options, params);
    std::cout << "[DEBUG] evaluateRoute: Done. Cost: " << result_route.getTotalCost() << ", Feasible: " << result_route.isFeasible() << std::endl;
    return result_route;
}

// localSearch: Applies MILP to optimize charging decisions for a fixed route
Route VNS::localSearch(const Route& current_route,
                      const Graph& graph,
                      const std::vector<std::vector<ChargingOption>>& charge_options,
                      const Parameters& params) {
    std::cout << "[DEBUG] localSearch: Starting for route. Node count: " << current_route.getNodeIds().size() << std::endl;
    Route route_to_evaluate = current_route;

    if (!quickFeasibilityCheck(route_to_evaluate, graph, params)) {
        std::cout << "[DEBUG] localSearch: Quick feasibility check FAILED." << std::endl;
        route_to_evaluate.setTotalCost(1e9 + (rng()%1000));
        route_to_evaluate.setFeasible(false);
        std::cout << "[DEBUG] localSearch: Route marked infeasible with cost " << route_to_evaluate.getTotalCost() << std::endl;
        return route_to_evaluate;
    }
    std::cout << "[DEBUG] localSearch: Quick feasibility check PASSED." << std::endl;

    std::cout << "[DEBUG] localSearch: Calling solveSubproblem1..." << std::endl;
    SubproblemResult result = solveSubproblem1(route_to_evaluate, graph, charge_options, params);
    std::cout << "[DEBUG] localSearch: solveSubproblem1 returned. Feasible: " << result.feasible << ", Cost: " << result.cost << std::endl;

    Route result_route_obj(result.new_node_ids.empty() ? route_to_evaluate.getNodeIds() : result.new_node_ids);
    if (result.feasible) {
        updateRoute(result_route_obj, result);
        std::cout << "[DEBUG] localSearch: Subproblem feasible. Route updated. New cost: " << result_route_obj.getTotalCost() << std::endl;
    } else {
        result_route_obj.setTotalCost(1e9 + (rng()%1000)); // Mark as infeasible
        result_route_obj.setFeasible(false);
        std::cout << "[DEBUG] localSearch: Subproblem infeasible. Route marked infeasible with cost " << result_route_obj.getTotalCost() << std::endl;
    }
    return result_route_obj;
}


// optimize: Main VNS algorithm loop
Route VNS::optimize(const std::vector<int>& initial_nodes,
                   const Graph& graph,
                   const std::vector<std::vector<ChargingOption>>& charge_options,
                   const Parameters& params) {
    std::cout << "[DEBUG] optimize: Starting VNS optimization..." << std::endl;

    Route initial_route_obj(initial_nodes);
    std::cout << "[DEBUG] optimize: Initial route object created. Node count: " << initial_nodes.size() << std::endl;
    std::cout << "[DEBUG] optimize: Validating initial route..." << std::endl;
    if (!isValidRoute(initial_nodes, graph, params)) {
        std::cerr << "[ERROR_DEBUG] optimize: Initial route is not valid according to isValidRoute check." << std::endl;
        initial_route_obj.setTotalCost(1e10);
        initial_route_obj.setFeasible(false);
        return initial_route_obj;
    }
    std::cout << "[DEBUG] optimize: Initial route is valid. Evaluating initial route..." << std::endl;

    Route best_route = evaluateRoute(initial_route_obj, graph, charge_options, params);
    Route current_route = best_route;
    std::cout << "[DEBUG] optimize: Initial route evaluated. Best cost: " << best_route.getTotalCost() << ", Current cost: " << current_route.getTotalCost() << std::endl;

    double best_cost = best_route.getTotalCost();
    double current_cost = current_route.getTotalCost();
    no_improvement_counter = 0;

    for (int iter = 0; iter < max_iterations; ++iter) {
        std::cout << "[DEBUG] optimize: Iteration " << iter << " / " << max_iterations << std::endl;
        std::cout << "[DEBUG] optimize: Current best cost: " << best_cost << ", Current route cost: " << current_cost << std::endl;
        std::cout << "[DEBUG] optimize: Selecting operator..." << std::endl;
        int k_op_idx = selectOperatorWeighted();
        std::cout << "[DEBUG] optimize: Operator index " << k_op_idx << " selected. Shaking current route..." << std::endl;

        Route s_prime = shake(current_route, k_op_idx, graph, params);
        std::cout << "[DEBUG] optimize: Shake completed. S_prime node count: " << s_prime.getNodeIds().size() << ". Performing local search on S_prime..." << std::endl;

        Route s_prime_optimized = localSearch(s_prime, graph, charge_options, params);
        double s_prime_cost = s_prime_optimized.getTotalCost();
        std::cout << "[DEBUG] optimize: Local search on S_prime completed. Cost: " << s_prime_cost << ", Feasible: " << s_prime_optimized.isFeasible() << std::endl;

        double previous_best_cost_for_check = best_cost;

        if (s_prime_optimized.isFeasible() && s_prime_cost < best_cost) {
            std::cout << "[DEBUG] optimize: New global best found! Cost: " << s_prime_cost << " (old best: " << best_cost << ")" << std::endl;
            best_route = s_prime_optimized;
            current_route = s_prime_optimized;
            best_cost = s_prime_cost;
            current_cost = s_prime_cost;

            operator_weights_[k_op_idx] = std::min(operator_weights_[k_op_idx] * 1.1, 1.0);
            normalizeWeights();
            std::cout << "[DEBUG] optimize: Operator " << k_op_idx << " weight boosted. Weights normalized." << std::endl;

            if (checkSignificantImprovement(previous_best_cost_for_check, best_cost)) {
                no_improvement_counter = 0;
                std::cout << "[DEBUG] optimize: Significant improvement. No_improvement_counter reset." << std::endl;
            } else {
                no_improvement_counter++;
                std::cout << "[DEBUG] optimize: Minor improvement. No_improvement_counter incremented to " << no_improvement_counter << std::endl;
            }
        } else if (s_prime_optimized.isFeasible() && s_prime_cost < current_cost) {
            std::cout << "[DEBUG] optimize: Improvement over current S (but not S_best). Cost: " << s_prime_cost << " (current_cost: " << current_cost << ")" << std::endl;
            current_route = s_prime_optimized;
            current_cost = s_prime_cost;
            no_improvement_counter++;
            std::cout << "[DEBUG] optimize: No_improvement_counter incremented to " << no_improvement_counter << std::endl;
        } else {
            std::cout << "[DEBUG] optimize: No improvement or worse solution. S_prime_cost: " << s_prime_cost << ", Current_cost: " << current_cost << std::endl;
            no_improvement_counter++;
            operator_weights_[k_op_idx] = std::max(operator_weights_[k_op_idx] * 0.9, 0.01);
            normalizeWeights();
            std::cout << "[DEBUG] optimize: Operator " << k_op_idx << " weight penalized. Weights normalized. No_improvement_counter incremented to " << no_improvement_counter << std::endl;
        }

        if (no_improvement_counter >= 50) {
             std::cout << "[DEBUG] optimize: No improvement for " << no_improvement_counter << " iterations. Resetting current_route to best_route." << std::endl;
             current_route = best_route;
             current_cost = best_cost;
             // no_improvement_counter = 0; // Optionally reset counter
        }
        if (iter % 10 == 0) { // Log progress more frequently for debugging
            std::cout << "VNS Iter: " << iter << ", Best Cost: " << std::fixed << std::setprecision(2) << best_cost
                      << ", Current Cost: " << current_cost << ", No Improve: " << no_improvement_counter << std::endl;
            std::cout << "Operator weights: ";
            for(double w : operator_weights_) std::cout << std::fixed << std::setprecision(3) << w << " ";
            std::cout << std::endl;
        }
    }
    std::cout << "[DEBUG] optimize: VNS finished. Best cost: " << best_route.getTotalCost() << std::endl;
    return best_route;
}


/// solveSubproblem1: MILP for fixed route charging optimization
#include <iostream> // Include iostream for std::cout

SubproblemResult VNS::solveSubproblem1(const Route& current_route_obj,
                                 const Graph& graph,
                                 const std::vector<std::vector<ChargingOption>>& charge_options_all_nodes,
                                 const Parameters& params) {
    std::cout << "Entering solveSubproblem1 for route: ";
    const std::vector<int>& route_node_ids_debug = current_route_obj.getNodeIds();
    for(int node_id : route_node_ids_debug) {
        std::cout << node_id << " ";
    }
    std::cout << std::endl;

    SubproblemResult result;
    result.feasible = false;
    result.cost = 1e9; // Default to infeasible cost

    const std::vector<int>& route_node_ids = current_route_obj.getNodeIds();
    if (route_node_ids.size() < 2) {
        std::cout << "Route size is less than 2 (" << route_node_ids.size() << "). Returning trivial result." << std::endl;
        result.new_node_ids = route_node_ids;
        result.cost = 0; // Or handle as appropriate
        result.feasible = true; // Or false if empty route is invalid
        return result;
    }

    IloModel model(env);
    try {
        int num_nodes_in_route = route_node_ids.size();
        int num_arcs_in_route = num_nodes_in_route - 1;
        std::cout << "Route has " << num_nodes_in_route << " nodes and " << num_arcs_in_route << " arcs." << std::endl;

        // Create a mapping from original node ID to its index in CPLEX variables (0 to num_nodes_in_route-1)
        std::map<int, int> node_to_var_map;
        for(int i=0; i < num_nodes_in_route; ++i) {
            node_to_var_map[route_node_ids[i]] = i;
        }
         std::cout << "Created node to variable index map." << std::endl;


        // --- Define decision variables ---
        std::cout << "Defining CPLEX variables..." << std::endl;
        IloNumVarArray ya(env, num_nodes_in_route, 0, params.getBatteryCapacity(), ILOFLOAT); // SOC on arrival
        IloNumVarArray yd(env, num_nodes_in_route, 0, params.getBatteryCapacity(), ILOFLOAT); // SOC on departure
        IloNumVarArray t(env, num_nodes_in_route, 0, IloInfinity, ILOFLOAT);                  // Arrival time
        IloNumVarArray depart(env, num_nodes_in_route, 0, IloInfinity, ILOFLOAT);             // Departure time
        IloNumVarArray s(env, num_arcs_in_route, 0, IloInfinity, ILOFLOAT);                   // Travel time on arc k

        // Charging decision variables phi[i] (charging time at node i)
        IloNumVarArray phi(env, num_nodes_in_route, 0, IloInfinity, ILOFLOAT); // Charging duration at node i
        std::cout << "Defined ya, yd, t, depart, s, phi variables." << std::endl;

        // w[i][k]: binary, 1 if charging option k is chosen at station node i (original node ID)
        // phi_w[i][k]: continuous, phi[i] * w[i][k] (linearized)
        std::vector<IloBoolVarArray> w(num_nodes_in_route);
        std::vector<IloNumVarArray> phi_w(num_nodes_in_route);

        // Wireless charging variables
        std::vector<int> wireless_arc_indices; // Stores k index of arcs that are wireless
        std::vector<double> d_ij(num_arcs_in_route); // distance for arc k
        std::vector<double> beta_ij(num_arcs_in_route); // wireless charge rate for arc k (Energy/Time)
        std::vector<bool> is_wireless_route(num_arcs_in_route); // true if arc k is wireless
        std::vector<double> min_s(num_arcs_in_route); // min travel time for arc k
        std::vector<double> max_s(num_arcs_in_route); // max travel time for arc k


         std::cout << "Processing arcs and wireless properties..." << std::endl;
        for (int k = 0; k < num_arcs_in_route; ++k) {
            int u_node_id = route_node_ids[k];
            int v_node_id = route_node_ids[k+1];
            const Arc* arc = graph.findArc(u_node_id, v_node_id);
            if (!arc) {
                 std::cerr << "Error: Arc " << u_node_id << " -> " << v_node_id << " not found in graph." << std::endl;
                 throw std::runtime_error("Arc not found in solveSubproblem1, route is inconsistent.");
            }

            d_ij[k] = arc->getDistance();
            is_wireless_route[k] = arc->getIsWireless();
             std::cout << "  Arc " << u_node_id << " -> " << v_node_id << ", Distance: " << d_ij[k] << ", Wireless: " << (is_wireless_route[k] ? "Yes" : "No") << std::endl;

            if (arc->getIsWireless()) {
                beta_ij[k] = arc->getWirelessChargeRate(); // Assumed Energy/Time
                wireless_arc_indices.push_back(k);
                 std::cout << "    Wireless charge rate: " << beta_ij[k] << std::endl;
            } else {
                beta_ij[k] = 0;
            }
            // Basic travel time bounds (can be refined if variable speed is allowed)
            if (d_ij[k] < 1e-6) { // Zero distance arc (e.g. self loop or between copies)
                 min_s[k] = 0.0;
                 max_s[k] = 0.0; // Or a small epsilon if needed
            } else {
                 // Add check for params.getVehicleSpeed() being zero or negative
                 if (params.getVehicleSpeed() <= 1e-9) {
                     std::cerr << "Error: Vehicle speed is zero or negative (" << params.getVehicleSpeed() << "). Cannot calculate travel time bounds for arc " << u_node_id << " -> " << v_node_id << std::endl;
                      // Handle this error appropriately, e.g., set bounds to infinity or return infeasible
                      min_s[k] = IloInfinity;
                      max_s[k] = IloInfinity;
                 } else {
                     min_s[k] = d_ij[k] / params.getVehicleSpeed(); // Assuming max speed for min time
                     // Add check for vehicle speed * 0.5 being zero or negative
                     if (params.getVehicleSpeed() * 0.5 <= 1e-9) {
                          std::cerr << "Error: Vehicle speed * 0.5 is zero or negative (" << params.getVehicleSpeed() * 0.5 << "). Cannot calculate max travel time bound for arc " << u_node_id << " -> " << v_node_id << std::endl;
                         max_s[k] = IloInfinity;
                     } else {
                         max_s[k] = d_ij[k] / (params.getVehicleSpeed() * 0.5); // Assuming min speed (e.g. 50% of avg) for max time
                     }
                 }
            }
             // Add check for valid bounds
            if (min_s[k] > max_s[k] + 1e-6 || std::isnan(min_s[k]) || std::isinf(min_s[k]) || std::isnan(max_s[k]) || std::isinf(max_s[k])) {
                std::cerr << "Error: Invalid travel time bounds calculated for arc " << u_node_id << " -> " << v_node_id << ": [" << min_s[k] << ", " << max_s[k] << "]. Adjusting." << std::endl;
                 // Adjust bounds to safe values to prevent CPLEX errors
                 min_s[k] = 0.0;
                 max_s[k] = IloInfinity; // Or a large finite number like 1e9
                 std::cerr << "Adjusted bounds to [" << min_s[k] << ", " << max_s[k] << "]" << std::endl;
            }
            s[k].setBounds(min_s[k], max_s[k]);
            std::cout << "    Travel time bounds for s[" << k << "]: [" << min_s[k] << ", " << max_s[k] << "]" << std::endl;

        }

        int p = wireless_arc_indices.size(); // Number of wireless-capable arcs in this specific route
         std::cout << "Found " << p << " wireless capable arcs in this route." << std::endl;
        IloBoolVarArray z(env, p);      // z[l]: 1 if wireless charging is used on l-th wireless_arc_in_route, 0 otherwise
        IloNumVarArray w_s_z(env, p);   // w_s_z[l] = s[wireless_arc_indices[l]] * z[l] (linearized)
        std::cout << "Defined z and w_s_z variables for wireless arcs with size " << p << std::endl;

        // Initialize charging decision variables w and phi_w
        std::cout << "Initializing charging decision variables w and phi_w..." << std::endl;
        for (int var_idx = 0; var_idx < num_nodes_in_route; ++var_idx) {
            int original_node_id = route_node_ids[var_idx];
            const Node* node = graph.findNode(original_node_id);
             if (!node) {
                std::cerr << "Error: Node " << original_node_id << " not found in graph during w/phi_w setup." << std::endl;
                continue; // Or throw
            }
            if (node->getType() == NodeType::CHARGING_STATION) {
                const auto& options_for_node = charge_options_all_nodes[original_node_id]; // Assumes charge_options_all_nodes is indexed by original_node_id
                if (!options_for_node.empty()) {
                    int num_options = options_for_node.size();
                    std::cout << "  Node " << original_node_id << " is a charging station with " << num_options << " options." << std::endl;
                    w[var_idx] = IloBoolVarArray(env, num_options);
                    phi_w[var_idx] = IloNumVarArray(env, num_options, 0, IloInfinity, ILOFLOAT);
                     std::cout << "    Initialized w[" << var_idx << "] and phi_w[" << var_idx << "] with " << num_options << " options." << std::endl;
                } else {
                    std::cout << "  Node " << original_node_id << " is a charging station but has no options defined." << std::endl;
                     // Initialize as empty arrays even if it's a station with no options defined
                    w[var_idx] = IloBoolVarArray(env);
                    phi_w[var_idx] = IloNumVarArray(env);
                }
            } else {
                 std::cout << "  Node " << original_node_id << " is not a charging station." << std::endl;
                 // Ensure w and phi_w remain empty or are handled correctly for non-stations
                 w[var_idx] = IloBoolVarArray(env); // Empty array for non-stations
                 phi_w[var_idx] = IloNumVarArray(env); // Empty array for non-stations
            }
        }
        std::cout << "Finished initializing w and phi_w." << std::endl;


        // --- Objective Function ---
        std::cout << "Setting up objective function..." << std::endl;
        IloExpr objective(env);
        // 1. Cost of time (travel + service + charging)
        std::cout << "  Adding time costs..." << std::endl;
        for (int k = 0; k < num_arcs_in_route; ++k) {
            objective += params.getTimeCost() * s[k];
             // std::cout << "  Added time cost for arc " << k << std::endl; // Too verbose
        }
        for (int var_idx = 0; var_idx < num_nodes_in_route; ++var_idx) {
            int original_node_id = route_node_ids[var_idx];
            const Node* node = graph.findNode(original_node_id);
             if (!node) continue;

            if (node->getServiceTime() > 1e-6) {
                 objective += params.getTimeCost() * node->getServiceTime(); // Service time cost
                 // std::cout << "  Added service time cost for node " << original_node_id << std::endl; // Too verbose
            }
            objective += params.getTimeCost() * phi[var_idx]; // Charging time cost
             // std::cout << "  Added charging time cost for node " << original_node_id << std::endl; // Too verbose
        }
         std::cout << "  Finished adding time costs." << std::endl;

        // 2. Cost of charging at stations
        std::cout << "  Adding charging costs at stations..." << std::endl;
        for (int var_idx = 0; var_idx < num_nodes_in_route; ++var_idx) {
            int original_node_id = route_node_ids[var_idx];
            const Node* node = graph.findNode(original_node_id);
            if (node && node->getType() == NodeType::CHARGING_STATION) {
                 const auto& options_for_node = charge_options_all_nodes[original_node_id];
                 // Check if phi_w[var_idx] was initialized with a non-zero size
                 if (phi_w[var_idx].getSize() > 0) {
                    // std::cout << "  Adding charging cost for station node " << original_node_id << " with " << options_for_node.size() << " options." << std::endl; // Too verbose
                    for (size_t opt_idx = 0; opt_idx < options_for_node.size(); ++opt_idx) {
                        objective += options_for_node[opt_idx].getCost() * phi_w[var_idx][opt_idx]; // Cost per unit time * total time for option
                         // std::cout << "    Added cost for option " << opt_idx << " with rate " << options_for_node[opt_idx].getRate() << " and cost " << options_for_node[opt_idx].getCost() << std::endl; // Too verbose
                    }
                 } else {
                      // std::cout << "  Node " << original_node_id << " is a charging station but has no options/phi_w initialized." << std::endl; // Too verbose
                 }
            }
        }
         std::cout << "  Finished adding charging costs at stations." << std::endl;

        // 3. Cost of wireless charging (proportional to energy gained, if beta is energy/time and w_s_z is time)
        std::cout << "Adding wireless charging cost..." << std::endl;
        std::cout << "Number of wireless arcs (p): " << p << std::endl;
        std::cout << "Wireless cost parameter: " << params.getWirelessCost() << std::endl;

        for (int l = 0; l < p; ++l) {
            int k = wireless_arc_indices[l]; // Original arc index in the route
             std::cout << "  Processing wireless arc l=" << l << ", route arc index k=" << k << std::endl;
             std::cout << "    beta_ij[" << k << "] = " << beta_ij[k] << std::endl;

            // The potential crash point is likely here or just after accessing w_s_z[l]
             std::cout << "    Accessing w_s_z[" << l << "] variable handle..." << std::endl;
            IloNumVar wireless_time_var = w_s_z[l];
             std::cout << "    Successfully accessed w_s_z[" << l << "] handle." << std::endl;

             // std::cout << "    Creating term: " << params.getWirelessCost() << " * " << beta_ij[k] << " * w_s_z[" << l << "]" << std::endl;

            // This line is the most suspect one for the crash
             std::cout << "    Adding term to objective..." << std::endl;
            objective += params.getWirelessCost() * beta_ij[k] * wireless_time_var;
             std::cout << "    Successfully added term to objective." << std::endl;

             std::cout << "  Added wireless cost term for l=" << l << std::endl;
        }
         std::cout << "Finished adding wireless charging costs to objective." << std::endl;

        model.add(IloMinimize(env, objective));
        objective.end();
        std::cout << "Finished setting up objective function." << std::endl;

        // --- Constraints ---
        std::cout << "Setting up constraints..." << std::endl;
        // Time constraints
        std::cout << "  Adding initial time constraints..." << std::endl;
        model.add(t[0] == 0);       // Arrive at depot at time 0
        model.add(depart[0] == t[0]); // Depart depot immediately (or after service if any, handled by general rule)
        std::cout << "  Added initial time constraints." << std::endl;


        std::cout << "  Adding time and charging constraints for each node..." << std::endl;
        for (int var_idx = 0; var_idx < num_nodes_in_route; ++var_idx) {
            int original_node_id = route_node_ids[var_idx];
            const Node* node = graph.findNode(original_node_id);
            if (!node) {
                 std::cerr << "Error: Node " << original_node_id << " not found during constraint setup." << std::endl;
                 throw std::runtime_error("Node not found during constraint setup.");
            }

            if (node->getType() == NodeType::CHARGING_STATION) {
                model.add(depart[var_idx] == t[var_idx] + node->getServiceTime() + phi[var_idx]);
                 // std::cout << "    Node " << original_node_id << ": depart[" << var_idx << "] == t[" << var_idx << "] + service_time + phi[" << var_idx << "]" << std::endl; // Too verbose
            } else { // Customer or Depot
                model.add(depart[var_idx] == t[var_idx] + node->getServiceTime());
                model.add(phi[var_idx] == 0); // No charging at non-stations
                 // std::cout << "    Node " << original_node_id << ": depart[" << var_idx << "] == t[" << var_idx << "] + service_time, phi[" << var_idx << "] == 0" << std::endl; // Too verbose
            }

            if (var_idx < num_arcs_in_route) { // For departure from var_idx to arrival at var_idx+1
                 model.add(t[var_idx+1] >= depart[var_idx] + s[var_idx]); // Arrival at next node
                 // std::cout << "    Arc " << route_node_ids[var_idx] << " -> " << route_node_ids[var_idx+1] << " (var_idx " << var_idx << " -> " << var_idx + 1 << "): t[" << var_idx + 1 << "] >= depart[" << var_idx << "] + s[" << var_idx << "]" << std::endl; // Too verbose
            }
        }
         std::cout << "  Finished adding time and charging constraints for nodes." << std::endl;

        // Max total duration constraint could be added here if necessary
        // model.add(depart[num_nodes_in_route-1] <= params.getMaxTourTime());

        // SOC constraints
        std::cout << "  Adding SOC constraints..." << std::endl;
        model.add(ya[0] == params.getInitialSoc()); // Initial SOC at depot start
        model.add(yd[0] == ya[0]);                  // No charging at depot start usually
        std::cout << "    Initial SOC at depot (node " << route_node_ids[0] << "): ya[0] = " << params.getInitialSoc() << ", yd[0] = ya[0]" << std::endl;


        int l_wireless_counter = 0; // Counter for wireless specific variables z and w_s_z
        for (int k = 0; k < num_arcs_in_route; ++k) { // Iterate over arcs in the route
            int i_var_idx = node_to_var_map[route_node_ids[k]];     // Variable index for start node of arc k
            int j_var_idx = node_to_var_map[route_node_ids[k+1]];   // Variable index for end node of arc k
             int u_node_id = route_node_ids[k];
            int v_node_id = route_node_ids[k+1];

            std::cout << "    Processing SOC for arc " << u_node_id << " -> " << v_node_id << " (var_idx " << i_var_idx << " -> " << j_var_idx << ")" << std::endl;
            // SOC update along arc k
            IloExpr soc_arrival_at_j(env);
            soc_arrival_at_j = yd[i_var_idx] - params.getEnergyConsumption() * d_ij[k];
            // std::cout << "      SOC arrival at " << v_node_id << " (ya[" << j_var_idx << "]) = yd[" << i_var_idx << "] - consumption(" << params.getEnergyConsumption() << ") * distance(" << d_ij[k] << ")" << std::endl; // Too verbose


            if (is_wireless_route[k]) {
                std::cout << "      Arc is wireless (route arc k=" << k << ", wireless index l=" << l_wireless_counter << "). Adding wireless charging term and McCormick constraints..." << std::endl;
                // w_s_z[l_wireless_counter] = s[k] * z[l_wireless_counter]
                // beta_ij[k] is energy/(unit of s[k], e.g. time)
                // So beta_ij[k] * w_s_z[l_wireless_counter] is energy gained
                soc_arrival_at_j += beta_ij[k] * w_s_z[l_wireless_counter];
                 std::cout << "        + beta_ij[" << k << "] * w_s_z[" << l_wireless_counter << "] (Wireless energy gain added to SOC arrival expression)" << std::endl;

                // McCormick for w_s_z[l] = s[k] * z[l]
                double M_s_val = max_s[k]; // Upper bound for s[k]
                 std::cout << "        McCormick M_s_val = " << M_s_val << std::endl;

                 // Constraint 1: w_s_z[l] <= s[k]
                 std::cout << "        Adding constraint: w_s_z[" << l_wireless_counter << "] <= s[" << k << "]" << std::endl;
                model.add(w_s_z[l_wireless_counter] <= s[k]);
                 std::cout << "        Constraint added." << std::endl;

                 // Constraint 2: w_s_z[l] <= M_s_val * z[l]
                 std::cout << "        Adding constraint: w_s_z[" << l_wireless_counter << "] <= " << M_s_val << " * z[" << l_wireless_counter << "]" << std::endl;
                model.add(w_s_z[l_wireless_counter] <= M_s_val * z[l_wireless_counter]);
                 std::cout << "        Constraint added." << std::endl;

                 // Constraint 3: w_s_z[l] >= s[k] - M_s_val * (1 - z[l])
                 std::cout << "        Adding constraint: w_s_z[" << l_wireless_counter << "] >= s[" << k << "] - " << M_s_val << " * (1 - z[" << l_wireless_counter << "])" << std::endl;
                model.add(w_s_z[l_wireless_counter] >= s[k] - M_s_val * (1 - z[l_wireless_counter]));
                 std::cout << "        Constraint added." << std::endl;

                 // Constraint 4: w_s_z[l] >= 0
                 std::cout << "        Adding constraint: w_s_z[" << l_wireless_counter << "] >= 0" << std::endl;
                model.add(w_s_z[l_wireless_counter] >= 0);
                 std::cout << "        Constraint added." << std::endl;


                l_wireless_counter++;
            }
            model.add(ya[j_var_idx] == soc_arrival_at_j);
            soc_arrival_at_j.end();
            std::cout << "      Added constraint: ya[" << j_var_idx << "] == calculated_soc_arrival." << std::endl;


            // SOC bounds at arrival
            model.add(ya[j_var_idx] >= params.getMinSoc());
            model.add(ya[j_var_idx] <= params.getBatteryCapacity()); // Redundant if yd is also capped, but good practice
            std::cout << "      Added bounds for ya[" << j_var_idx << "]: [" << params.getMinSoc() << ", " << params.getBatteryCapacity() << "]" << std::endl;


            // SOC at departure from node j_var_idx (which is node route_node_ids[k+1])
            const Node* current_node_obj = graph.findNode(route_node_ids[k+1]); // This is node j
             if (!current_node_obj) {
                 std::cerr << "Error: Node " << route_node_ids[k+1] << " not found during SOC departure setup." << std::endl;
                 continue; // Or throw
             }

             std::cout << "    Processing SOC departure for node " << route_node_ids[k+1] << " (var_idx " << j_var_idx << ")" << std::endl;
             if (current_node_obj->getType() == NodeType::CHARGING_STATION) {
                const auto& options_for_node_j = charge_options_all_nodes[route_node_ids[k+1]];
                 // Check if phi_w[j_var_idx] was initialized with a non-zero size
                 if (phi_w[j_var_idx].getSize() > 0) {
                    std::cout << "      Node " << route_node_ids[k+1] << " is a charging station with " << options_for_node_j.size() << " options. Adding charging constraints..." << std::endl;
                    IloExpr total_charge_amount_at_j(env);
                    IloExpr sum_w_at_j(env); // Sum of w[j_var_idx][opt_idx]
                    for (size_t opt_idx = 0; opt_idx < options_for_node_j.size(); ++opt_idx) {
                        // Linearization for phi_w[j_var_idx][opt_idx] = phi[j_var_idx] * w[j_var_idx][opt_idx]
                        double rate = options_for_node_j[opt_idx].getRate();
                        double U_phi_val = params.getBatteryCapacity() / (rate > 1e-9 ? rate : 1.0) ; // Max charging time estimate, use 1e-9 to avoid division by zero
                         // Add check for valid U_phi_val
                         if (std::isnan(U_phi_val) || std::isinf(U_phi_val)) {
                             std::cerr << "Warning: Invalid U_phi_val calculated for node " << route_node_ids[k+1] << ", option " << opt_idx << ". Rate: " << rate << ". Setting to IloInfinity." << std::endl;
                             U_phi_val = IloInfinity; // Or a large finite number
                         }

                        model.add(phi_w[j_var_idx][opt_idx] <= phi[j_var_idx]);
                        model.add(phi_w[j_var_idx][opt_idx] <= U_phi_val * w[j_var_idx][opt_idx]);
                        model.add(phi_w[j_var_idx][opt_idx] >= phi[j_var_idx] - U_phi_val * (1 - w[j_var_idx][opt_idx]));
                        model.add(phi_w[j_var_idx][opt_idx] >= 0);
                         // std::cout << "        Added McCormick for phi_w[" << j_var_idx << "][" << opt_idx << "]" << std::endl; // Too verbose


                        total_charge_amount_at_j += options_for_node_j[opt_idx].getRate() * phi_w[j_var_idx][opt_idx];
                        sum_w_at_j += w[j_var_idx][opt_idx];
                    }
                    model.add(sum_w_at_j <= 1); // Choose at most one charging option
                     std::cout << "        Added constraint: sum(w[" << j_var_idx << "][:]) <= 1" << std::endl;
                    model.add(yd[j_var_idx] == ya[j_var_idx] + total_charge_amount_at_j);
                     std::cout << "        Added constraint: yd[" << j_var_idx << "] == ya[" << j_var_idx << "] + total_charge_amount_at_j" << std::endl;
                    total_charge_amount_at_j.end();
                    sum_w_at_j.end();
                } else { // Charging station but no options defined or phi_w not initialized
                    std::cout << "      Node " << route_node_ids[k+1] << " is a charging station but has no options defined or w/phi_w not initialized. No charging possible." << std::endl;
                    model.add(yd[j_var_idx] == ya[j_var_idx]);
                     model.add(phi[j_var_idx] == 0); // Ensure no charging time if no options
                }
            } else { // Not a charging station
                std::cout << "      Node " << route_node_ids[k+1] << " is not a charging station. No charging possible." << std::endl;
                model.add(yd[j_var_idx] == ya[j_var_idx]);
                 model.add(phi[j_var_idx] == 0); // Ensure no charging time
            }
            model.add(yd[j_var_idx] <= params.getBatteryCapacity());
            model.add(yd[j_var_idx] >= params.getMinSoc()); // SOC at departure should also be above min
             std::cout << "      Added bounds for yd[" << j_var_idx << "]: [" << params.getMinSoc() << ", " << params.getBatteryCapacity() << "]" << std::endl;
        }

        std::cout << "Finished setting up SOC constraints." << std::endl;


        // Solve the model
        std::cout << "Solving CPLEX model..." << std::endl;
        IloCplex cplex(model);
        cplex.setOut(env.getNullStream()); // Suppress CPLEX output
        // cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.01); // Example: set MIP gap
        // cplex.setParam(IloCplex::Param::TimeLimit, 60); // Example: set time limit

        if (cplex.solve()) {
            std::cout << "CPLEX solve successful. Status: " << cplex.getStatus() << std::endl;
            result.feasible = true;
            result.cost = cplex.getObjValue();
            result.new_node_ids = route_node_ids; // Route sequence is fixed
            std::cout << "Objective value (cost): " << result.cost << std::endl;

            result.soc_arrival.resize(num_nodes_in_route);
            result.soc_departure.resize(num_nodes_in_route);
            result.arrival_time.resize(num_nodes_in_route);
            result.departure_time.resize(num_nodes_in_route);
            // result.charging_decisions.clear(); // Implemented by phi_w
            result.wireless_decisions.assign(num_arcs_in_route, false);

            std::cout << "Extracting results..." << std::endl;
            for (int i = 0; i < num_nodes_in_route; ++i) {
                result.soc_arrival[i] = cplex.getValue(ya[i]);
                result.soc_departure[i] = cplex.getValue(yd[i]);
                result.arrival_time[i] = cplex.getValue(t[i]);
                result.departure_time[i] = cplex.getValue(depart[i]);
                 std::cout << "  Node " << route_node_ids[i] << " (var_idx " << i << "): Arrival Time=" << result.arrival_time[i] << ", Departure Time=" << result.departure_time[i] << ", SOC_Arr=" << result.soc_arrival[i] << ", SOC_Dep=" << result.soc_departure[i];

                // Store charging decisions if needed (phi > 0)
                 int original_node_id = route_node_ids[i];
                 const Node* node = graph.findNode(original_node_id);
                 if (node && node->getType() == NodeType::CHARGING_STATION) {
                     double charge_time = cplex.getValue(phi[i]);
                     if (charge_time > 1e-6) {
                        // Check if w[i] was initialized before accessing its size
                        if (w[i].getSize() > 0) {
                            const auto& options = charge_options_all_nodes[original_node_id];
                             std::cout << ", Charge Time=" << charge_time << ", Chosen Option: ";
                            for(int opt_idx = 0; opt_idx < options.size(); ++opt_idx) {
                                if (cplex.getValue(w[i][opt_idx]) > 0.5) { // If this option was chosen
                                    result.charging_decisions.emplace_back(original_node_id, opt_idx, charge_time);
                                    std::cout << "Option " << opt_idx << " (Rate: " << options[opt_idx].getRate() << ", Cost: " << options[opt_idx].getCost() << ")";
                                    break; // Assuming at most one option is chosen
                                }
                            }
                        } else {
                             std::cout << ", Charge Time=" << charge_time << " (w array not initialized)";
                        }
                     } else {
                          std::cout << ", Charge Time=0";
                     }
                 } else {
                      std::cout << ", Not a charging station";
                 }
                 std::cout << std::endl;
            }
            l_wireless_counter = 0; // Reset counter for extraction
             std::cout << "Extracting wireless decisions..." << std::endl;
            for (int k = 0; k < num_arcs_in_route; ++k) {
                 if (is_wireless_route[k]) {
                    // Check if z[l_wireless_counter] is a valid handle before accessing its value
                     if (l_wireless_counter < p && z[l_wireless_counter].getImpl()) {
                        if (cplex.getValue(z[l_wireless_counter]) > 0.5) { // If wireless charging was used on this arc segment
                            result.wireless_decisions[k] = true;
                            std::cout << "  Wireless charging used on arc " << route_node_ids[k] << " -> " << route_node_ids[k+1] << " (route index " << k << ", wireless index " << l_wireless_counter << ")" << std::endl;
                        } else {
                             std::cout << "  Wireless charging NOT used on arc " << route_node_ids[k] << " -> " << route_node_ids[k+1] << " (route index " << k << ", wireless index " << l_wireless_counter << ")" << std::endl;
                        }
                     } else {
                         std::cerr << "Warning: Attempted to access invalid z variable handle at wireless index " << l_wireless_counter << " (p=" << p << ")" << std::endl;
                     }
                    l_wireless_counter++;
                 }
            }
             std::cout << "Finished extracting wireless decisions." << std::endl;

        } else {
            std::cout << "CPLEX solve failed. Status: " << cplex.getStatus() << std::endl;
            result.feasible = false;
            result.cost = 1e9 + rng() % 1000; // Infeasible
             std::cerr << "Subproblem solve failed. CPLEX status: " << cplex.getStatus() << std::endl;
            // If you need to debug infeasibility:
             std::cout << "Attempting to export model and refine conflict..." << std::endl;
            cplex.exportModel("infeasible_model.lp");
            try {
            //      IloCplex::ConflictStatus conflict_status = cplex.refineConflict();
            //     if (conflict_status != IloCplex::ConflictStatus::ConflictNeverFound) {
            //        std::cout << "Conflict refined. Writing to conflict.txt" << std::endl;
            //        cplex.writeConflict("conflict.txt");
            //     } else {
            //         std::cout << "Conflict refinement did not find a conflict." << std::endl;
            //     }
            // } catch (const IloException& ce) {
            //      std::cerr << "CPLEX Exception during conflict refinement: " << ce.getMessage() << std::endl;
            } catch (...) {
                 std::cerr << "Unknown Exception during conflict refinement." << std::endl;
            }

        }
        cplex.end();
         std::cout << "CPLEX environment ended for solver." << std::endl;
    } catch (const IloException& e) {
        std::cerr << "CPLEX Exception: " << e.getMessage() << std::endl;
        result.feasible = false;
        result.cost = 1e9 + rng()%1000;
        // e.end(); // Not needed for IloException
         std::cout << "Caught CPLEX Exception." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Standard Exception: " << e.what() << std::endl;
        result.feasible = false;
        result.cost = 1e9 + rng()%1000;
         std::cout << "Caught Standard Exception." << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception in solveSubproblem1" << std::endl;
        result.feasible = false;
        result.cost = 1e9 + rng()%1000;
         std::cout << "Caught Unknown Exception." << std::endl;
    }
    model.end();
     std::cout << "CPLEX model ended." << std::endl;
    std::cout << "Exiting solveSubproblem1. Feasible: " << (result.feasible ? "Yes" : "No") << ", Cost: " << result.cost << std::endl;
    return result;
}

void VNS::updateRoute(Route& route, const SubproblemResult& result) {
    // std::cout << "[DEBUG] updateRoute: Updating route object..." << std::endl;
    route.setNodes(result.new_node_ids);
    route.setSocArrival(result.soc_arrival);
    route.setSocDeparture(result.soc_departure);
    route.setArrivalTime(result.arrival_time);
    route.setDepartureTime(result.departure_time);
    route.setChargingDecisions(result.charging_decisions);
    route.setWirelessDecisions(result.wireless_decisions);
    route.setTotalCost(result.cost);
    route.setFeasible(result.feasible && result.cost < (1e9 -1001) ); // Check against a margin below infeasible marker
    // std::cout << "[DEBUG] updateRoute: Done. Cost: " << route.getTotalCost() << ", Feasible: " << route.isFeasible() << std::endl;
}

// --- Neighborhood Structures (Shake operators) ---
Route VNS::shake(const Route& current_route, int neighborhood_idx, const Graph& graph, const Parameters& params) {
    std::cout << "[DEBUG] shake: Operator index " << neighborhood_idx % 5 << ". Current route size: " << current_route.getNodeIds().size() << std::endl;
    Route shaken_route = current_route; // Default to current route if operator fails or not applicable
    switch (neighborhood_idx % 5) {
        case 0:
            std::cout << "[DEBUG] shake: Applying swapNodes." << std::endl;
            shaken_route = swapNodes(current_route, graph, params);
            break;
        case 1:
            std::cout << "[DEBUG] shake: Applying relocate." << std::endl;
            shaken_route = relocate(current_route, graph, params);
            break;
        case 2:
            std::cout << "[DEBUG] shake: Applying twoOpt." << std::endl;
            shaken_route = twoOpt(current_route, graph, params);
            break;
        case 3:
            std::cout << "[DEBUG] shake: Applying insertStation." << std::endl;
            shaken_route = insertStation(current_route, graph, params);
            break;
        case 4:
            std::cout << "[DEBUG] shake: Applying removeStation." << std::endl;
            shaken_route = removeStation(current_route, graph, params);
            break;
        default:
            std::cout << "[DEBUG] shake: Default case, returning current_route." << std::endl;
            return current_route; // Should not happen
    }
    std::cout << "[DEBUG] shake: Operator applied. New route size: " << shaken_route.getNodeIds().size() << std::endl;
    return shaken_route;
}

Route VNS::swapNodes(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> node_ids = route.getNodeIds();
    std::cout << "[DEBUG] swapNodes: Route size " << node_ids.size() << std::endl;
    if (node_ids.size() <= 3) { // Need at least two distinct non-depot nodes
        std::cout << "[DEBUG] swapNodes: Route too small, returning original." << std::endl;
        return route;
    }
    // Indices for swappable nodes (excluding start and end depot)
    std::uniform_int_distribution<size_t> dist(1, node_ids.size() - 2);
    size_t i = dist(rng);
    size_t j = dist(rng);
    int attempts = 0;
    while (i == j && attempts < 10) { // Ensure i and j are different, with a safety break
        j = dist(rng);
        attempts++;
    }
    if (i == j) { // Still same after attempts, or if size is exactly 4, i and j might always be the same if not careful
         std::cout << "[DEBUG] swapNodes: Could not find two different indices or route too small for distinct non-depot. Returning original." << std::endl;
         if (node_ids.size() == 4 && i == 1 && j == 1) { // Specific case for size 4
            j = 2; // Try to make them different if possible
         } else if (node_ids.size() > 4) { // for larger routes, if i ==j, just pick another one if possible
            j = (i == 1) ? 2 : 1; // a simple way to make them different if only two non-depot nodes
         } else {
            return route;
         }
    }
    std::cout << "[DEBUG] swapNodes: Swapping indices " << i << " (node " << node_ids[i] << ") and " << j << " (node " << node_ids[j] << ")" << std::endl;
    std::swap(node_ids[i], node_ids[j]);
    return Route(node_ids);
}

Route VNS::relocate(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> node_ids = route.getNodeIds();
    std::cout << "[DEBUG] relocate: Route size " << node_ids.size() << std::endl;
    if (node_ids.size() <= 3) { // Need at least one non-depot node to move and one position to move to
        std::cout << "[DEBUG] relocate: Route too small, returning original." << std::endl;
        return route;
    }
    // Index of node to move (non-depot)
    std::uniform_int_distribution<size_t> dist_node_to_move(1, node_ids.size() - 2);
    size_t i = dist_node_to_move(rng);
    int node_to_move_val = node_ids[i];
    std::cout << "[DEBUG] relocate: Moving node " << node_to_move_val << " from index " << i << std::endl;
    node_ids.erase(node_ids.begin() + i);

    // Valid insertion positions are from index 1 (after start depot) to node_ids.size()-1 (before end depot of the MODIFIED list)
    // node_ids.size() is now original_size - 1.
    // So positions are 1 to (original_size - 1) - 1 = original_size - 2.
    // insert before index j.
    if (node_ids.size() <= 2) { // only depots left or one node + depots, nowhere to insert except its old conceptual place
        std::cout << "[DEBUG] relocate: Route too small after removal, returning original (conceptually)." << std::endl;
        // Reconstruct original to be safe if this happens, though logic should prevent it with initial check
        return route;
    }
    std::uniform_int_distribution<size_t> dist_insert_pos(1, node_ids.size() -1 ) ; // insert before index from 1 up to (new_size-1)
    size_t j_insert_before = dist_insert_pos(rng);

    std::cout << "[DEBUG] relocate: Inserting node " << node_to_move_val << " before index " << j_insert_before << " in the modified list (size " << node_ids.size() << ")" << std::endl;
    node_ids.insert(node_ids.begin() + j_insert_before, node_to_move_val);
    return Route(node_ids);
}

Route VNS::insertStation(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> node_ids = route.getNodeIds();
    std::cout << "[DEBUG] insertStation: Current route size " << node_ids.size() << std::endl;
    if (node_ids.empty()) {
        std::cout << "[DEBUG] insertStation: Route is empty, returning original." << std::endl;
        return route;
    }

    std::vector<int> available_stations;
    for (const auto& node : graph.getNodes()) {
        if (node.getType() == NodeType::CHARGING_STATION) {
            available_stations.push_back(node.getId());
        }
    }
    if (available_stations.empty()) {
        std::cout << "[DEBUG] insertStation: No available stations in graph, returning original route." << std::endl;
        return route;
    }
    std::cout << "[DEBUG] insertStation: Found " << available_stations.size() << " available stations." << std::endl;

    int station_to_insert = available_stations[rng() % available_stations.size()];
    std::cout << "[DEBUG] insertStation: Selected station " << station_to_insert << " to insert." << std::endl;

    if (node_ids.size() < 2) { // e.g. route is just [0] or [] - should not happen if correctly formed
         std::cout << "[DEBUG] insertStation: Route size < 2, cannot insert station. Returning original." << std::endl;
         return route; // Cannot insert meaningfully
    }
    // Insert at a random position (not before start depot, not after end depot)
    // Valid insertion indices: 1 (after start depot) to node_ids.size()-1 (before end depot)
    std::uniform_int_distribution<size_t> dist(1, node_ids.size() - 1);
    size_t pos_insert_before = dist(rng);
    std::cout << "[DEBUG] insertStation: Inserting station " << station_to_insert << " before index " << pos_insert_before << std::endl;
    node_ids.insert(node_ids.begin() + pos_insert_before, station_to_insert);
    return Route(node_ids);
}

Route VNS::removeStation(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> node_ids = route.getNodeIds();
    std::cout << "[DEBUG] removeStation: Current route size " << node_ids.size() << std::endl;
    std::vector<size_t> station_positions_in_route; // Stores indices in the route
    for (size_t i = 1; i < node_ids.size() - 1; ++i) { // Exclude depots
        const Node* current_node = graph.findNode(node_ids[i]);
        if (current_node && current_node->getType() == NodeType::CHARGING_STATION) {
            station_positions_in_route.push_back(i);
        }
    }
    if (station_positions_in_route.empty()) {
        std::cout << "[DEBUG] removeStation: No stations found in route to remove. Returning original." << std::endl;
        return route;
    }
    std::cout << "[DEBUG] removeStation: Found " << station_positions_in_route.size() << " stations in route." << std::endl;
    size_t random_idx_in_station_positions = rng() % station_positions_in_route.size();
    size_t actual_pos_to_remove_in_route = station_positions_in_route[random_idx_in_station_positions];
    std::cout << "[DEBUG] removeStation: Removing station " << node_ids[actual_pos_to_remove_in_route] << " from route index " << actual_pos_to_remove_in_route << std::endl;
    node_ids.erase(node_ids.begin() + actual_pos_to_remove_in_route);
    return Route(node_ids);
}

Route VNS::twoOpt(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> node_ids = route.getNodeIds();
    std::cout << "[DEBUG] twoOpt: Current route size " << node_ids.size() << std::endl;

    // Need at least 4 nodes for a valid 2-opt swap (e.g., D -> A -> B -> D, swap A-B link with D-D is not meaningful, need D->A->B->C->D)
    // To pick two distinct edges (i-1,i) and (j,j+1) where i is not j+1, we need:
    // node_ids[0] ... node_ids[i-1] | node_ids[i] ... node_ids[j] | node_ids[j+1] ... node_ids[n-1]
    // Reverse segment from i to j.
    // i must be at least 1. j must be less than node_ids.size()-2. i < j.
    // Smallest valid route: 0-1-2-0 (size 4). i=1, j=1. segment [1..1]. Reverse node_ids[1].
    // 0-1-2-3-0 (size 5). i can be 1 or 2.
    //  if i=1, j can be 1,2.
    //  if i=1, j=1: reverse node_ids[1]. Route: 0-1-2-3-0.
    //  if i=1, j=2: reverse node_ids[1],node_ids[2]. Route: 0-2-1-3-0
    // Min size for a meaningful 2-opt that changes path is 4 nodes (0-A-B-0). Indices i=1, j=1. Segment [A]. No change.
    // Consider 0-A-B-C-0 (size 5).
    // Choose edges (0,A) and (B,C). i=1 (A), j=2 (B). Reverse A,B -> 0-B-A-C-0
    // Indices for std::reverse are [first, last)
    // We select two nodes at index i and j in node_ids (1 <= i < j < node_ids.size()-1)
    // and reverse the sub-path from node_ids[i] to node_ids[j]
    if (node_ids.size() < 4) { // Need at least two non-depot nodes to potentially swap their order.
        std::cout << "[DEBUG] twoOpt: Route size < 4. Returning original." << std::endl;
        return route;
    }

    // Select index i from [1, size-3]
    // Select index j from [i+1, size-2]
    // This ensures there's at least one node between i and j, or j is the node before the end depot
    // And i is the node after the start depot.
    std::uniform_int_distribution<size_t> dist_i(1, node_ids.size() - 3); // i can be 1 up to (size-1)-2
    size_t i = dist_i(rng);

    std::uniform_int_distribution<size_t> dist_j(i + 1, node_ids.size() - 2); // j can be i+1 up to (size-1)-1
    size_t j = dist_j(rng);

    // Safety check if distributions become invalid due to small size not caught by initial check
    if (i > j || i >= node_ids.size() -1 || j >= node_ids.size()-1 ) {
         std::cout << "[DEBUG] twoOpt: Invalid i/j generated (i=" << i << ", j=" << j << ") for size " << node_ids.size() << ". Returning original." << std::endl;
         return route;
    }


    std::cout << "[DEBUG] twoOpt: Reversing segment from index " << i << " (node " << node_ids[i] << ") to index " << j << " (node " << node_ids[j] << ")" << std::endl;
    std::reverse(node_ids.begin() + i, node_ids.begin() + j + 1); // Reverse elements from index i to j inclusive
    return Route(node_ids);
}