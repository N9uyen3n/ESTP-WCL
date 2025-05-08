#include "../include/VNS.h"
#include "../include/Utils.h"
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <iostream>
#include <ilcplex/ilocplex.h>
#include <limits>
#include <vector>
ILOSTLBEGIN

VNS::VNS(const std::vector<int>& initial_route,
         const Graph& graph,
         const std::vector<std::vector<ChargingOption>>& charge_options,
         const Parameters& params,
         int max_iterations,
         std::mt19937& rng)
    : env(),
      current_route(initial_route),
      graph(graph),
      charge_options(charge_options),
      params(params),
      max_iterations(max_iterations),
      rng(rng),
      operator_weights_({0.4, 0.3, 0.2, 0.1}),
      no_improvement_counter(0),
      potential_stations(graph.getStationCopies()) { // Initialize with all stations
    if (!isValidRoute(initial_route, graph, params)) {
        throw std::invalid_argument("Invalid initial route");
    }
    for (const auto& node : graph.getNodes()) {
        if (Utils::isChargingStation(node.getId(), graph.getNodes())) {
            if (node.getId() >= static_cast<int>(charge_options.size()) || charge_options[node.getId()].empty()) {
                throw std::invalid_argument("Invalid charging options for station " + std::to_string(node.getId()));
            }
        }
    }
    // Ensure potential_stations is populated
    if (potential_stations.empty()) {
        std::cerr << "Warning: No charging stations available in potential_stations.\n";
        potential_stations = graph.getStationCopies();
    }
}

VNS::~VNS() {
    env.end();
}

Route VNS::run() {
    Route best_route(current_route);
    SubproblemResult initial_result = solveSubproblem(best_route, graph, charge_options, params);
    updateRoute(best_route, initial_result);
    double best_cost = initial_result.feasible ? initial_result.cost : std::numeric_limits<double>::infinity();
    double current_cost = best_cost;

    // Force insert charging stations if initial route is infeasible
    int max_insert_attempts = 3;
    int insert_attempts = 0;
    while (!initial_result.feasible && insert_attempts < max_insert_attempts) {
        std::cout << "Initial route infeasible, forcing insertion of a charging station (attempt " << insert_attempts + 1 << ").\n";
        best_route = insertStation(best_route, graph, params);
        initial_result = solveSubproblem(best_route, graph, charge_options, params);
        if (initial_result.feasible) {
            best_cost = initial_result.cost;
            current_cost = best_cost;
            updateRoute(best_route, initial_result);
        } else {
            std::cout << "Warning: Failed to make initial route feasible after station insertion (attempt " << insert_attempts + 1 << ").\n";
        }
        insert_attempts++;
    }
    if (!initial_result.feasible) {
        std::cout << "Error: Could not make initial route feasible after " << max_insert_attempts << " insertion attempts.\n";
    }

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        int neighborhood = selectOperatorWeighted();
        Route shaken_route = shake(Route(current_route), neighborhood, graph, params);
        Route local_optimum = shaken_route;

        // Quick feasibility check
        bool feasible = quickFeasibilityCheck(shaken_route, graph, params);
        SubproblemResult result;
        if (feasible) {
            local_optimum = localSearch(shaken_route);
            result = solveSubproblem(local_optimum, graph, charge_options, params);
        } else {
            std::cout << "Iteration " << iteration + 1 << ": Shaken route infeasible, forcing station insertion.\n";
            local_optimum = insertStation(shaken_route, graph, params);
            result = solveSubproblem(local_optimum, graph, charge_options, params);
            int sub_insert_attempts = 0;
            while (!result.feasible && sub_insert_attempts < max_insert_attempts) {
                std::cout << "Iteration " << iteration + 1 << ": Inserted route still infeasible, trying another insertion (attempt " << sub_insert_attempts + 1 << ").\n";
                local_optimum = insertStation(local_optimum, graph, params);
                result = solveSubproblem(local_optimum, graph, charge_options, params);
                sub_insert_attempts++;
            }
        }

        if (result.feasible && result.cost < best_cost) {
            // New global best
            best_cost = result.cost;
            best_route = local_optimum;
            current_route = result.new_node_ids;
            current_cost = result.cost;
            no_improvement_counter = 0;
            operator_weights_[neighborhood] *= 1.1;
            normalizeWeights();
            std::cout << "Iteration " << iteration + 1 << ": New global best, cost = " << best_cost << "\n";
        } else if (result.feasible && result.cost < current_cost) {
            // Improvement over current solution
            current_route = result.new_node_ids;
            current_cost = result.cost;
            no_improvement_counter++;
            std::cout << "Iteration " << iteration + 1 << ": Improved current solution, cost = " << current_cost << "\n";
        } else {
            no_improvement_counter++;
        }

        // Accept feasible solution even if not better
        if (result.feasible && result.cost < std::numeric_limits<double>::infinity()) {
            current_route = result.new_node_ids;
            current_cost = result.cost;
        }

        if ((iteration + 1) % 10 == 0) {
            std::cout << "Iteration " << iteration + 1 << ":\n";
            std::cout << "  Best Cost: " << best_cost << "\n";
            std::cout << "  Current Cost: " << current_cost << "\n";
            std::cout << "  Current Route: ";
            for (int id : current_route) {
                std::cout << id;
                if (Utils::isChargingStation(id, graph.getNodes())) {
                    std::cout << "(CS)";
                }
                std::cout << " ";
            }
            std::cout << "\n";

            Route temp_route(current_route);
            SubproblemResult current_result = solveSubproblem(temp_route, graph, charge_options, params);
            if (current_result.feasible) {
                std::cout << "  SOC Profile:\n";
                for (size_t i = 0; i < current_result.new_node_ids.size(); ++i) {
                    std::cout << "    Node " << current_result.new_node_ids[i]
                              << ": SOC Arrival = " << current_result.soc_arrival[i]
                              << ", SOC Departure = " << current_result.soc_departure[i] << "\n";
                }
                std::cout << "  Charging Decisions:\n";
                bool has_charging = false;
                for (size_t i = 0; i < current_result.new_node_ids.size(); ++i) {
                    if (current_result.charging_decisions[i].getChargingTime() > 0) {
                        has_charging = true;
                        std::cout << "    Node " << current_result.new_node_ids[i]
                                  << ": Station ID = " << current_result.charging_decisions[i].getStationId()
                                  << ", Option = " << current_result.charging_decisions[i].getOption()
                                  << ", Charging Time = " << current_result.charging_decisions[i].getChargingTime() << " units\n";
                    }
                }
                if (!has_charging) {
                    std::cout << "    No station charging used.\n";
                }
                std::cout << "  Wireless Charging Decisions:\n";
                bool has_wireless = false;
                for (size_t i = 0; i < current_result.wireless_decisions.size(); ++i) {
                    if (current_result.wireless_decisions[i]) {
                        has_wireless = true;
                        std::cout << "    Arc (" << current_result.new_node_ids[i] << ", "
                                  << current_result.new_node_ids[i + 1] << "): Wireless charging used\n";
                    }
                }
                if (!has_wireless) {
                    std::cout << "    No wireless charging used.\n";
                }
            } else {
                std::cout << "  Charging Decisions: Infeasible route\n";
            }

            std::cout << "  Operator Weights: ";
            for (double w : operator_weights_) std::cout << w << " ";
            std::cout << "\n  No Improvement Counter: " << no_improvement_counter << "\n";
        }

        if (no_improvement_counter >= 50) {
            std::cout << "Terminating early due to no improvement after 50 iterations.\n";
            break;
        }
    }
    updateRoute(best_route, solveSubproblem(best_route, graph, charge_options, params));
    return best_route;
}

SubproblemResult VNS::solveSubproblem(const Route& current_route,
                                      const Graph& graph,
                                      const std::vector<std::vector<ChargingOption>>& charge_options,
                                      const Parameters& params) {
    SubproblemResult result;
    result.new_node_ids = current_route.getNodeIds();
    result.feasible = false;
    result.cost = std::numeric_limits<double>::infinity();
    result.soc_arrival.resize(result.new_node_ids.size(), 0.0);
    result.soc_departure.resize(result.new_node_ids.size(), 0.0);
    result.arrival_time.resize(result.new_node_ids.size(), 0.0);
    result.departure_time.resize(result.new_node_ids.size(), 0.0);
    result.charging_decisions.resize(result.new_node_ids.size(), ChargingDecision(0, 0, 0.0));
    result.wireless_decisions.resize(result.new_node_ids.size() - 1, false);

    if (params.getInitialSoc() < params.getMinSoc() || params.getInitialSoc() > params.getBatteryCapacity()) {
        std::cerr << "Error: initial_SOC (" << params.getInitialSoc()
                  << ") must be between minSOC (" << params.getMinSoc()
                  << ") and Q (" << params.getBatteryCapacity() << ")." << std::endl;
        return result;
    }

    try {
        IloEnv env;
        IloModel model(env);
        IloCplex cplex(model);

        int n = result.new_node_ids.size();
        int m = n - 1;

        // Find valid insertion positions
        std::vector<size_t> valid_positions;
        for (size_t i = 1; i < n; ++i) {
            valid_positions.push_back(i);
        }

        // Potential stations to consider
        int p = std::min(2, static_cast<int>(potential_stations.size()));
        std::vector<int> stations_to_consider;
        if (p > 0) {
            std::vector<int> temp_stations = potential_stations;
            std::shuffle(temp_stations.begin(), temp_stations.end(), rng);
            stations_to_consider.assign(temp_stations.begin(), temp_stations.begin() + p);
        }

        // Check if route is initially infeasible (to force station insertion)
        bool needs_station = false;
        double soc = params.getInitialSoc();
        for (size_t i = 0; i < n - 1; ++i) {
            const Arc* arc = graph.findArc(result.new_node_ids[i], result.new_node_ids[i + 1]);
            if (!arc) continue;
            double energy_consumed = params.getEnergyConsumption() * arc->getDistance();
            double energy_charged = (arc->getIsWireless()) ? arc->getWirelessChargeRate() * arc->getDistance() : 0.0;
            soc = soc - energy_consumed + energy_charged;
            if (soc < params.getMinSoc() && !Utils::isChargingStation(result.new_node_ids[i + 1], graph.getNodes())) {
                needs_station = true;
                break;
            }
        }

        // Decision variables
        IloNumVarArray phi(env, n + p, 0, IloInfinity, ILOFLOAT); // Charging time
        std::vector<IloBoolVarArray> w(n + p);
        for (int i = 0; i < n; ++i) {
            w[i] = IloBoolVarArray(env, charge_options[result.new_node_ids[i]].size());
        }
        for (int i = n; i < n + p; ++i) {
            int station_id = stations_to_consider[i - n];
            w[i] = IloBoolVarArray(env, charge_options[station_id].size());
        }
        IloNumVarArray s(env, m + p, 0, IloInfinity, ILOFLOAT); // Travel time
        IloBoolVarArray z(env, m + p); // Wireless charging decisions
        IloNumVarArray s_z(env, m + p, 0, IloInfinity, ILOFLOAT); // Linearized s * z
        IloNumVarArray ya(env, n + p, params.getMinSoc() - 0.001, params.getBatteryCapacity(), ILOFLOAT); // SOC arrival
        IloNumVarArray yd(env, n + p, params.getMinSoc() - 0.001, params.getBatteryCapacity(), ILOFLOAT); // SOC departure
        IloNumVarArray t(env, n + p, 0, IloInfinity, ILOFLOAT); // Arrival time
        IloNumVarArray depart(env, n + p, 0, IloInfinity, ILOFLOAT); // Departure time

        // Linearization variables for phi[i] * w[i][k]
        std::vector<IloNumVarArray> phi_w(n + p);
        for (int i = 0; i < n + p; ++i) {
            int node_id = (i < n) ? result.new_node_ids[i] : stations_to_consider[i - n];
            phi_w[i] = IloNumVarArray(env, charge_options[node_id].size(), 0, IloInfinity, ILOFLOAT);
        }

        // Variables for station insertion
        IloBoolVarArray insert_z(env, p);
        std::vector<IloBoolVarArray> y(p);
        for (int l = 0; l < p; ++l) {
            y[l] = IloBoolVarArray(env, valid_positions.size());
        }

        // Ensure at most one station per position
        IloExprArray pos_usage(env, valid_positions.size());
        for (size_t k = 0; k < valid_positions.size(); ++k) {
            pos_usage[k] = IloExpr(env);
            for (int l = 0; l < p; ++l) {
                pos_usage[k] += y[l][k];
            }
            model.add(pos_usage[k] <= 1);
        }

        // Each station inserted at most once
        for (int l = 0; l < p; ++l) {
            IloExpr sum_y(env);
            for (size_t k = 0; k < valid_positions.size(); ++k) {
                sum_y += y[l][k];
            }
            model.add(sum_y == insert_z[l]);
        }

        // Force at least one station if route is infeasible
        if (needs_station && p > 0) {
            IloExpr sum_insert_z(env);
            for (int l = 0; l < p; ++l) {
                sum_insert_z += insert_z[l];
            }
            model.add(sum_insert_z >= 1);
            std::cout << "VNS::solveSubproblem: Forcing at least one station insertion due to infeasible SOC.\n";
        }

        // Objective function
        IloExpr obj(env);
        for (int i = 0; i < n + p; ++i) {
            int node_id = (i < n) ? result.new_node_ids[i] : stations_to_consider[i - n];
            for (size_t kk = 0; kk < charge_options[node_id].size(); ++kk) {
                obj += charge_options[node_id][kk].getCost() * charge_options[node_id][kk].getRate() * phi_w[i][kk];
            }
        }
        for (int i = 0; i < m + p; ++i) {
            int from = (i < m) ? result.new_node_ids[i] : stations_to_consider[i - m];
            int to = (i < m) ? result.new_node_ids[i + 1] : (i < m + p - 1 ? stations_to_consider[i - m + 1] : result.new_node_ids.back());
            const Arc* arc = graph.findArc(from, to);
            if (arc && arc->getIsWireless()) {
                obj += params.getWirelessCost() * arc->getWirelessChargeRate() * s_z[i];
            }
        }
        obj += params.getTimeCost() * t[n + p - 1];
        model.add(IloMinimize(env, obj));

        // Linearization constraints for phi[i] * w[i][k]
        for (int i = 0; i < n + p; ++i) {
            int node_id = (i < n) ? result.new_node_ids[i] : stations_to_consider[i - n];
            for (size_t kk = 0; kk < charge_options[node_id].size(); ++kk) {
                double U_phi = params.getBatteryCapacity() / charge_options[node_id][kk].getRate();
                model.add(phi_w[i][kk] <= phi[i]);
                model.add(phi_w[i][kk] <= U_phi * w[i][kk]);
                model.add(phi_w[i][kk] >= phi[i] - U_phi * (1 - w[i][kk]));
                model.add(phi_w[i][kk] >= 0);
            }
        }

        // Linearization constraints for s[i] * z[i]
        for (int i = 0; i < m + p; ++i) {
            int from = (i < m) ? result.new_node_ids[i] : stations_to_consider[i - m];
            int to = (i < m) ? result.new_node_ids[i + 1] : (i < m + p - 1 ? stations_to_consider[i - m + 1] : result.new_node_ids.back());
            const Arc* arc = graph.findArc(from, to);
            double M = arc ? (arc->getDistance() / params.getUmin()) : params.getBigM();
            model.add(s_z[i] <= s[i]);
            model.add(s_z[i] <= M * z[i]);
            model.add(s_z[i] >= s[i] - M * (1 - z[i]));
            model.add(s_z[i] >= 0);
        }

        // Constraints for no charging at nodes without options
        for (int i = 0; i < n; ++i) {
            int node = result.new_node_ids[i];
            if (charge_options[node].empty()) {
                model.add(phi[i] == 0);
            }
        }
        for (int i = n; i < n + p; ++i) {
            int l = i - n;
            for (size_t kk = 0; kk < charge_options[stations_to_consider[l]].size(); ++kk) {
                model.add(phi_w[i][kk] <= params.getBatteryCapacity() * insert_z[l]);
            }
        }

        // Time constraints
        model.add(depart[0] == t[0]);
        model.add(t[0] == 0);

        // Build new route dynamically
        std::vector<int> new_route = result.new_node_ids;
        std::vector<int> node_mapping(n + p, -1);
        node_mapping[0] = 0;
        int current_idx = 1;

        for (size_t i = 1; i < n; ++i) {
            for (int l = 0; l < p; ++l) {
                for (size_t k = 0; k < valid_positions.size(); ++k) {
                    if (valid_positions[k] == i) {
                        const Arc* arc = graph.findArc(new_route[current_idx - 1], stations_to_consider[l]);
                        if (!arc) {
                            std::cerr << "Error: Arc from " << new_route[current_idx - 1] << " to station " << stations_to_consider[l] << " not found.\n";
                            continue;
                        }
                        double d = arc->getDistance();
                        model.add(t[n + l] >= depart[current_idx - 1] + s[current_idx - 1]);
                        model.add(depart[n + l] == t[n + l] + phi[n + l]);
                        node_mapping[n + l] = current_idx;
                        // Replace setBounds() with explicit lower and upper bound constraints
                        model.add(s[current_idx - 1] >= d / params.getUmax() * y[l][k]);
                        model.add(s[current_idx - 1] <= d / params.getUmin() * y[l][k] + params.getBigM() * (1 - y[l][k]));
                        current_idx++;
                    }
                }
            }
            const Arc* arc = graph.findArc(new_route[current_idx - 1], result.new_node_ids[i]);
            if (!arc) {
                std::cerr << "Error: Arc from " << new_route[current_idx - 1] << " to " << result.new_node_ids[i] << " not found.\n";
                env.end();
                return result;
            }
            double d = arc->getDistance();
            double min_s = d / params.getUmax();
            double max_s = d / params.getUmin();
            model.add(t[i] >= depart[current_idx - 1] + s[current_idx - 1]);
            s[current_idx - 1].setBounds(min_s, max_s);
            node_mapping[i] = current_idx;
            int node = result.new_node_ids[i];
            if (!charge_options[node].empty()) {
                model.add(depart[i] == t[i] + phi[i]);
            } else if (node != 0) {
                model.add(depart[i] == t[i] + graph.findNode(node)->getServiceTime());
            } else {
                model.add(depart[i] == t[i]);
            }
            current_idx++;
        }

        // SOC constraints
        model.add(ya[0] == params.getInitialSoc());
        for (int i = 0; i < n + p; ++i) {
            model.add(ya[i] >= params.getMinSoc() - 0.001);
            model.add(yd[i] >= params.getMinSoc() - 0.001);
            model.add(ya[i] <= params.getBatteryCapacity());
            model.add(yd[i] <= params.getBatteryCapacity());
        }
        for (int i = 0; i < n; ++i) {
            if (!charge_options[result.new_node_ids[i]].empty()) {
                IloExpr charge_amount(env);
                for (size_t kk = 0; kk < charge_options[result.new_node_ids[i]].size(); ++kk) {
                    charge_amount += charge_options[result.new_node_ids[i]][kk].getRate() * phi_w[i][kk];
                }
                model.add(yd[i] == ya[i] + charge_amount);
            } else {
                model.add(yd[i] == ya[i]);
            }
        }
        for (int i = n; i < n + p; ++i) {
            int l = i - n;
            IloExpr charge_amount(env);
            for (size_t kk = 0; kk < charge_options[stations_to_consider[l]].size(); ++kk) {
                charge_amount += charge_options[stations_to_consider[l]][kk].getRate() * phi_w[i][kk];
            }
            model.add(yd[i] == ya[i] + charge_amount);
        }

        // SOC for arcs
        for (int i = 1; i < current_idx; ++i) {
            int var_idx = -1;
            for (int j = 0; j < n + p; ++j) {
                if (node_mapping[j] == i) {
                    var_idx = j;
                    break;
                }
            }
            int prev_var_idx = -1;
            for (int j = 0; j < n + p; ++j) {
                if (node_mapping[j] == i - 1) {
                    prev_var_idx = j;
                    break;
                }
            }
            if (var_idx == -1 || prev_var_idx == -1) {
                std::cerr << "Error: Invalid node mapping at index " << i << ".\n";
                env.end();
                return result;
            }
            const Arc* arc = graph.findArc(new_route[i - 1], new_route[i]);
            if (!arc) {
                std::cerr << "Error: Arc from " << new_route[i - 1] << " to " << new_route[i] << " not found.\n";
                env.end();
                return result;
            }
            double d = arc->getDistance();
            if (arc->getIsWireless()) {
                model.add(ya[var_idx] == yd[prev_var_idx] - params.getEnergyConsumption() * d +
                          arc->getWirelessChargeRate() * s_z[i - 1]);
            } else {
                model.add(ya[var_idx] == yd[prev_var_idx] - params.getEnergyConsumption() * d);
            }
        }

        // Charging selection constraints
        for (int i = 0; i < n; ++i) {
            int node = result.new_node_ids[i];
            if (!charge_options[node].empty()) {
                IloExpr sum_w(env);
                for (size_t kk = 0; kk < charge_options[node].size(); ++kk) {
                    sum_w += w[i][kk];
                }
                model.add(sum_w <= 1);
            }
        }
        for (int i = n; i < n + p; ++i) {
            int l = i - n;
            IloExpr sum_w(env);
            for (size_t kk = 0; kk < charge_options[stations_to_consider[l]].size(); ++kk) {
                sum_w += w[i][kk];
            }
            model.add(sum_w <= insert_z[l]);
        }

        // Solve the model
        cplex.setOut(std::cout);
        cplex.setParam(IloCplex::TiLim, 90.0); // Increase time limit
        cplex.setParam(IloCplex::EpGap, 0.05); // Allow 5% gap
        if (!cplex.solve()) {
            std::cerr << "CPLEX: Failed to solve. Status: " << cplex.getStatus() << "\n";
            if (cplex.getStatus() == IloCplex::Infeasible) {
                std::cerr << "CPLEX: Model is infeasible.\n";
            } else if (cplex.getStatus() == IloCplex::TiLim) {
                std::cerr << "CPLEX: Time limit exceeded.\n";
            }
            env.end();
            return result;
        }

        // Check if a feasible solution exists
        if (cplex.getStatus() == IloCplex::Optimal || cplex.getStatus() == IloCplex::Feasible) {
            result.cost = cplex.getObjValue();
            result.feasible = true;
            std::cout << "CPLEX: Found feasible solution with cost = " << result.cost << ", gap = " << cplex.getMIPRelativeGap() * 100 << "%\n";
        } else {
            std::cerr << "CPLEX: No feasible solution found. Status: " << cplex.getStatus() << "\n";
            env.end();
            return result;
        }

        // Print insertion decisions
        for (int l = 0; l < p; ++l) {
            double insert_val = cplex.getValue(insert_z[l]);
            std::cout << "VNS::solveSubproblem: insert_z[" << l << "] = " << insert_val << "\n";
            if (insert_val > 0.5) {
                for (size_t k = 0; k < valid_positions.size(); ++k) {
                    double y_val = cplex.getValue(y[l][k]);
                    std::cout << "VNS::solveSubproblem: y[" << l << "][" << k << "] = " << y_val << "\n";
                    if (y_val > 0.5) {
                        std::cout << "VNS::solveSubproblem: Inserting station " << stations_to_consider[l]
                                  << " at position " << valid_positions[k] << "\n";
                    }
                }
            }
        }

        // Extract solution
        result.new_node_ids = current_route.getNodeIds();
        for (int l = 0; l < p; ++l) {
            if (cplex.getValue(insert_z[l]) > 0.5) {
                for (size_t k = 0; k < valid_positions.size(); ++k) {
                    if (cplex.getValue(y[l][k]) > 0.5) {
                        result.new_node_ids.insert(result.new_node_ids.begin() + valid_positions[k], stations_to_consider[l]);
                        std::cout << "VNS::solveSubproblem: Inserted charging station " << stations_to_consider[l]
                                  << " at position " << valid_positions[k] << "\n";
                        break;
                    }
                }
            }
        }

        // Update variables
        result.soc_arrival.resize(result.new_node_ids.size(), 0.0);
        result.soc_departure.resize(result.new_node_ids.size(), 0.0);
        result.arrival_time.resize(result.new_node_ids.size(), 0.0);
        result.departure_time.resize(result.new_node_ids.size(), 0.0);
        result.charging_decisions.resize(result.new_node_ids.size(), ChargingDecision(0, 0, 0.0));
        result.wireless_decisions.resize(result.new_node_ids.size() - 1, false);

        for (int i = 0; i < n + p; ++i) {
            if (node_mapping[i] != -1 && node_mapping[i] < static_cast<int>(result.new_node_ids.size())) {
                result.soc_arrival[node_mapping[i]] = cplex.getValue(ya[i]);
                result.soc_departure[node_mapping[i]] = cplex.getValue(yd[i]);
                result.arrival_time[node_mapping[i]] = cplex.getValue(t[i]);
                result.departure_time[node_mapping[i]] = cplex.getValue(depart[i]);
                int node_id = (i < n) ? result.new_node_ids[node_mapping[i]] : stations_to_consider[i - n];
                for (size_t kk = 0; kk < charge_options[node_id].size(); ++kk) {
                    if (cplex.getValue(w[i][kk]) > 0.5) {
                        result.charging_decisions[node_mapping[i]] = ChargingDecision(node_id, static_cast<int>(kk), cplex.getValue(phi[i]));
                        std::cout << "VNS::solveSubproblem: Charging at station " << node_id
                                  << " with option " << kk
                                  << ", charging time: " << cplex.getValue(phi[i]) << " units, "
                                  << "SOC after charging: " << cplex.getValue(yd[i]) << "\n";
                        break;
                    }
                }
            }
        }

        for (int i = 0; i < m + p; ++i) {
            if (cplex.getValue(z[i]) > 0.5) {
                int route_idx = (i < m) ? i : (i - m + valid_positions.size());
                if (route_idx < static_cast<int>(result.wireless_decisions.size())) {
                    result.wireless_decisions[route_idx] = true;
                    std::cout << "VNS::solveSubproblem: Wireless charging used on arc ("
                              << result.new_node_ids[route_idx] << ", "
                              << result.new_node_ids[route_idx + 1] << "), charging time: "
                              << cplex.getValue(s[i]) << " units\n";
                }
            }
        }

        // Validate SOC constraints
        for (size_t i = 0; i < result.soc_arrival.size(); ++i) {
            if (result.soc_arrival[i] < params.getMinSoc() - 0.001 || result.soc_departure[i] < params.getMinSoc() - 0.001) {
                std::cerr << "Warning: Invalid SOC at node " << result.new_node_ids[i]
                          << ": Arrival SOC = " << result.soc_arrival[i]
                          << ", Departure SOC = " << result.soc_departure[i] << "\n";
                result.feasible = false;
                result.cost = std::numeric_limits<double>::infinity();
                env.end();
                return result;
            }
        }

        env.end();
        return result;
    } catch (IloException& e) {
        std::cerr << "CPLEX Exception: " << e.getMessage() << "\n";
        return result;
    } catch (std::exception& e) {
        std::cerr << "Standard Exception: " << e.what() << "\n";
        return result;
    }
}

Route VNS::localSearch(const Route& current_route) {
    Route best_local = current_route;
    SubproblemResult best_result = solveSubproblem(best_local, graph, charge_options, params);
    if (!best_result.feasible) return current_route;

    bool improved = true;
    while (improved) {
        improved = false;
        Route new_route = twoOpt(best_local, graph, params);
        SubproblemResult new_result = solveSubproblem(new_route, graph, charge_options, params);
        if (new_result.feasible && new_result.cost < best_result.cost) {
            best_local = new_route;
            best_result = new_result;
            improved = true;
        }
    }
    updateRoute(best_local, best_result);
    return best_local;
}

Route VNS::shake(const Route& current_route, int neighborhood,
                 const Graph& graph, const Parameters& params) {
    Route shaken_route = current_route;
    switch (neighborhood) {
        case 0: return twoOpt(shaken_route, graph, params);
        case 1: return relocate(shaken_route, graph, params);
        case 2: return swapNodes(shaken_route, graph, params);
        case 3: {
            if (std::uniform_real_distribution<>(0, 1)(rng) < 0.95) { // Increase probability
                return insertStation(shaken_route, graph, params);
            } else {
                return removeStation(shaken_route, graph, params);
            }
        }
        default: return shaken_route;
    }
}

Route VNS::swapNodes(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> new_nodes = route.getNodeIds();
    int i = std::uniform_int_distribution<>(1, new_nodes.size() - 2)(rng);
    int j = std::uniform_int_distribution<>(1, new_nodes.size() - 2)(rng);
    std::swap(new_nodes[i], new_nodes[j]);
    if (isValidRoute(new_nodes, graph, params)) {
        return Route(new_nodes);
    }
    return route;
}

Route VNS::relocate(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> new_nodes = route.getNodeIds();
    int i = std::uniform_int_distribution<>(1, new_nodes.size() - 2)(rng);
    int j = std::uniform_int_distribution<>(1, new_nodes.size() - 2)(rng);
    int node = new_nodes[i];
    new_nodes.erase(new_nodes.begin() + i);
    new_nodes.insert(new_nodes.begin() + j, node);
    if (isValidRoute(new_nodes, graph, params)) {
        return Route(new_nodes);
    }
    return route;
}

Route VNS::insertStation(const Route& route, const Graph& graph, const Parameters& params) {
    const auto& stations = graph.getStationCopies();
    if (stations.empty()) {
        std::cout << "VNS::insertStation: No stations available.\n";
        return route;
    }

    std::vector<int> new_nodes = route.getNodeIds();
    std::vector<size_t> valid_positions;
    for (size_t i = 1; i < new_nodes.size(); ++i) {
        if (graph.findArc(new_nodes[i - 1], new_nodes[i])) {
            valid_positions.push_back(i);
        }
    }
    if (valid_positions.empty()) {
        std::cout << "VNS::insertStation: No valid positions for insertion.\n";
        return route;
    }

    // Calculate SOC to find critical points
    double soc = params.getInitialSoc();
    size_t critical_pos = 1;
    int critical_node = new_nodes[0];
    bool needs_station = false;
    for (size_t i = 0; i < new_nodes.size() - 1; ++i) {
        const Arc* arc = graph.findArc(new_nodes[i], new_nodes[i + 1]);
        if (!arc) {
            std::cout << "VNS::insertStation: Arc from " << new_nodes[i] << " to " << new_nodes[i + 1] << " not found.\n";
            continue;
        }
        double energy_consumed = params.getEnergyConsumption() * arc->getDistance();
        double energy_charged = (arc->getIsWireless()) ? arc->getWirelessChargeRate() * arc->getDistance() : 0.0;
        soc = soc - energy_consumed + energy_charged;
        std::cout << "VNS::insertStation: Node " << new_nodes[i + 1] << ", SOC = " << soc
                  << ", Consumed = " << energy_consumed << ", Charged = " << energy_charged << "\n";
        if (soc < params.getMinSoc() && !Utils::isChargingStation(new_nodes[i + 1], graph.getNodes())) {
            critical_pos = i + 1;
            critical_node = new_nodes[i];
            needs_station = true;
            break;
        }
    }

    if (!needs_station) {
        std::cout << "VNS::insertStation: No station needed, SOC sufficient.\n";
        return route;
    }

    // Select the closest station to the critical node
    int station = stations[0];
    double min_distance = std::numeric_limits<double>::infinity();
    for (int s : stations) {
        const Arc* arc = graph.findArc(critical_node, s);
        if (arc && arc->getDistance() < min_distance) {
            min_distance = arc->getDistance();
            station = s;
        }
    }

    // Ensure station is in potential_stations
    if (std::find(potential_stations.begin(), potential_stations.end(), station) == potential_stations.end()) {
        potential_stations.push_back(station);
        std::cout << "VNS::insertStation: Added station " << station << " to potential_stations.\n";
    }

    // Select insertion position
    size_t pos = critical_pos;
    if (!valid_positions.empty()) {
        auto it = std::find(valid_positions.begin(), valid_positions.end(), critical_pos);
        if (it == valid_positions.end()) {
            pos = valid_positions[std::uniform_int_distribution<>(0, valid_positions.size() - 1)(rng)];
        }
    }

    // Insert station into route
    new_nodes.insert(new_nodes.begin() + pos, station);

    if (isValidRoute(new_nodes, graph, params)) {
        std::cout << "VNS::insertStation: Inserted station " << station << " at position " << pos
                  << " near node " << critical_node << ", SOC before = " << soc << "\n";
        return Route(new_nodes);
    }
    std::cout << "VNS::insertStation: Insertion failed, route invalid.\n";
    return route;
}

Route VNS::removeStation(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> new_nodes = route.getNodeIds();
    std::vector<size_t> station_indices;
    for (size_t i = 1; i < new_nodes.size() - 1; ++i) {
        if (Utils::isChargingStation(new_nodes[i], graph.getNodes())) {
            station_indices.push_back(i);
        }
    }
    if (station_indices.empty()) return route;
    int idx = station_indices[std::uniform_int_distribution<>(0, station_indices.size() - 1)(rng)];
    new_nodes.erase(new_nodes.begin() + idx);
    if (isValidRoute(new_nodes, graph, params)) {
        return Route(new_nodes);
    }
    return route;
}

Route VNS::twoOpt(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> new_nodes = route.getNodeIds();
    int i = std::uniform_int_distribution<>(1, new_nodes.size() - 2)(rng);
    int j = std::uniform_int_distribution<>(i + 1, new_nodes.size() - 1)(rng);
    std::reverse(new_nodes.begin() + i, new_nodes.begin() + j);
    if (isValidRoute(new_nodes, graph, params)) {
        return Route(new_nodes);
    }
    return route;
}

void VNS::updateRoute(Route& route, const SubproblemResult& result) {
    route.setNodes(result.new_node_ids);
    route.setArrivalTime(result.arrival_time);
    route.setDepartureTime(result.departure_time);
    route.setSocArrival(result.soc_arrival);
    route.setSocDeparture(result.soc_departure);
    route.setChargingDecisions(result.charging_decisions);
    route.setWirelessDecisions(result.wireless_decisions);
    route.setTotalCost(result.cost);
    route.setFeasible(result.feasible);
}

void VNS::normalizeWeights() {
    double sum = std::accumulate(operator_weights_.begin(), operator_weights_.end(), 0.0);
    for (auto& w : operator_weights_) {
        w /= sum;
    }
}

int VNS::selectOperatorWeighted() {
    std::discrete_distribution<int> dist(operator_weights_.begin(), operator_weights_.end());
    return dist(rng);
}

bool VNS::isValidRoute(const std::vector<int>& route_node_ids, const Graph& graph, const Parameters& params) {
    if (route_node_ids.empty()) return false;
    if (Utils::isCustomer(route_node_ids[0], graph.getNodes()) || Utils::isCustomer(route_node_ids.back(), graph.getNodes())) {
        return false;
    }
    std::set<int> customers;
    for (size_t i = 0; i < route_node_ids.size() - 1; ++i) {
        if (!graph.findArc(route_node_ids[i], route_node_ids[i + 1])) return false;
        if (Utils::isCustomer(route_node_ids[i], graph.getNodes())) customers.insert(route_node_ids[i]);
    }
    if (Utils::isCustomer(route_node_ids.back(), graph.getNodes())) customers.insert(route_node_ids.back());
    return true;
}

bool VNS::quickFeasibilityCheck(const Route& route, const Graph& graph, const Parameters& params) {
    double soc = params.getInitialSoc();
    const auto& nodes = route.getNodeIds();
    std::cout << "quickFeasibilityCheck: Starting SOC = " << soc << "\n";
    for (size_t i = 0; i < nodes.size() - 1; ++i) {
        const Arc* arc = graph.findArc(nodes[i], nodes[i + 1]);
        if (!arc) {
            std::cout << "quickFeasibilityCheck: Arc from " << nodes[i] << " to " << nodes[i + 1] << " not found.\n";
            return false;
        }
        double energy_consumed = params.getEnergyConsumption() * arc->getDistance();
        double energy_charged = (arc->getIsWireless()) ? arc->getWirelessChargeRate() * arc->getDistance() : 0.0;
        soc = soc - energy_consumed + energy_charged;
        std::cout << "quickFeasibilityCheck: Node " << nodes[i + 1] << ", SOC = " << soc
                  << ", Consumed = " << energy_consumed << ", Charged = " << energy_charged << "\n";
        if (soc < params.getMinSoc()) {
            if (Utils::isChargingStation(nodes[i + 1], graph.getNodes()) && !charge_options[nodes[i + 1]].empty()) {
                soc = params.getBatteryCapacity();
                std::cout << "quickFeasibilityCheck: Charging at station " << nodes[i + 1] << ", SOC reset to " << soc << "\n";
            } else {
                std::cout << "quickFeasibilityCheck: SOC too low (" << soc << ") at node " << nodes[i + 1]
                          << ", suggesting station insertion before node " << nodes[i + 1] << ".\n";
                return false;
            }
        }
    }
    if (soc < params.getMinSoc()) {
        std::cout << "quickFeasibilityCheck: Final SOC too low (" << soc << ").\n";
        return false;
    }
    std::cout << "quickFeasibilityCheck: Route is feasible, final SOC = " << soc << "\n";
    return true;
}