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
      potential_stations() { // Initialize empty potential_stations
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
}

VNS::~VNS() {
    env.end();
}

Route VNS::run() {
    Route best_route(current_route);
    SubproblemResult initial_result = solveSubproblem(best_route, graph, charge_options, params);
    updateRoute(best_route, initial_result);
    double best_cost = initial_result.cost;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        int neighborhood = selectOperatorWeighted();
        Route shaken_route = shake(Route(current_route), neighborhood, graph, params);
        Route local_optimum = localSearch(shaken_route);
        SubproblemResult result = solveSubproblem(local_optimum, graph, charge_options, params);

        if (result.feasible && result.cost < best_cost) {
            best_cost = result.cost;
            best_route = local_optimum;
            current_route = result.new_node_ids; // Update with new_node_ids including inserted stations
            no_improvement_counter = 0;
            operator_weights_[neighborhood] *= 1.1;
            normalizeWeights();
        } else {
            // Handle infeasible route
            if (!result.feasible) {
                const int max_attempts = 5;
                Route temp_route = local_optimum;
                bool fixed = false;

                for (int attempt = 0; attempt < max_attempts && !fixed; ++attempt) {
                    bool needs_charging = false;
                    double soc = params.getInitialSoc();
                    const auto& nodes = temp_route.getNodeIds();
                    for (size_t i = 0; i < nodes.size() - 1; ++i) {
                        const Arc* arc = graph.findArc(nodes[i], nodes[i + 1]);
                        if (!arc) break;
                        soc -= params.getEnergyConsumption() * arc->getDistance();
                        if (soc < params.getMinSoc()) {
                            needs_charging = true;
                            break;
                        }
                        if (Utils::isChargingStation(nodes[i + 1], graph.getNodes())) {
                            soc = params.getBatteryCapacity();
                        }
                    }

                    if (needs_charging) {
                        temp_route = insertStation(temp_route, graph, params); // Adds to potential_stations
                    } else {
                        int alt_neighborhood = std::uniform_int_distribution<>(0, 3)(rng);
                        temp_route = shake(temp_route, alt_neighborhood, graph, params);
                    }

                    result = solveSubproblem(temp_route, graph, charge_options, params);
                    if (result.feasible && result.cost < best_cost) {
                        best_cost = result.cost;
                        best_route = temp_route;
                        current_route = result.new_node_ids;
                        no_improvement_counter = 0;
                        operator_weights_[neighborhood] *= 1.1;
                        normalizeWeights();
                        fixed = true;
                    }
                }
            }
            no_improvement_counter++;
            if (no_improvement_counter >= 50) {
                break;
            }
        }

        if ((iteration + 1) % 10 == 0) {
            std::cout << "Iteration " << iteration + 1 << ":\n";
            std::cout << "  Best Cost: " << best_cost << "\n";
            std::cout << "  Current Route: ";
            for (int id : current_route) {
                std::cout << id;
                if (Utils::isChargingStation(id, graph.getNodes())) {
                    std::cout << "(CS)";
                }
                std::cout << " ";
            }
            std::cout << "\n";

            // Solve subproblem for current route to get charging decisions
            Route temp_route(current_route);
            SubproblemResult current_result = solveSubproblem(temp_route, graph, charge_options, params);
            if (current_result.feasible) {
                std::cout << "  Charging Decisions:\n";
                for (size_t i = 0; i < current_result.new_node_ids.size(); ++i) {
                    if (current_result.charging_decisions[i].getChargingTime() > 0) {
                        std::cout << "    Node " << current_result.new_node_ids[i]
                                  << ": Station ID = " << current_result.charging_decisions[i].getStationId()
                                  << ", Option = " << current_result.charging_decisions[i].getOption()
                                  << ", Charging Time = " << current_result.charging_decisions[i].getChargingTime() << " units\n";
                    }
                }
            } else {
                std::cout << "  Charging Decisions: Infeasible route\n";
            }

            std::cout << "  Operator Weights: ";
            for (double w : operator_weights_) std::cout << w << " ";
            std::cout << "\n  No Improvement Counter: " << no_improvement_counter << "\n";
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
    result.cost = 1e9;
    result.soc_arrival.resize(result.new_node_ids.size(), 0.0);
    result.soc_departure.resize(result.new_node_ids.size(), 0.0);
    result.arrival_time.resize(result.new_node_ids.size(), 0.0);
    result.departure_time.resize(result.new_node_ids.size(), 0.0);
    result.charging_decisions.resize(result.new_node_ids.size(), ChargingDecision(0, 0, 0.0));
    result.wireless_decisions.clear();

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

        // Find valid insertion positions (between two customers)
        std::vector<size_t> valid_positions;
        for (size_t i = 1; i < n - 1; ++i) {
            if (Utils::isCustomer(result.new_node_ids[i - 1], graph.getNodes()) &&
                Utils::isCustomer(result.new_node_ids[i], graph.getNodes())) {
                valid_positions.push_back(i);
            }
        }

        // Potential stations to consider
        int p = potential_stations.size();
        std::vector<int> stations_to_consider = potential_stations;

        // Decision variables
        IloNumVarArray phi(env, n + p, 0, IloInfinity, ILOFLOAT); // Charging time for original and inserted nodes
        std::vector<IloBoolVarArray> w(n + p);
        for (int i = 0; i < n; ++i) {
            w[i] = IloBoolVarArray(env, charge_options[result.new_node_ids[i]].size());
        }
        for (int i = n; i < n + p; ++i) {
            int station_id = stations_to_consider[i - n];
            w[i] = IloBoolVarArray(env, charge_options[station_id].size());
        }
        IloNumVarArray s(env, m + p, 0, IloInfinity, ILOFLOAT); // Travel time
        IloNumVarArray ya(env, n + p, params.getMinSoc(), params.getBatteryCapacity(), ILOFLOAT);
        IloNumVarArray yd(env, n + p, params.getMinSoc(), params.getBatteryCapacity(), ILOFLOAT);
        IloNumVarArray t(env, n + p, 0, IloInfinity, ILOFLOAT);
        IloNumVarArray depart(env, n + p, 0, IloInfinity, ILOFLOAT);

        // Linearization variables for phi[i] * w[i][k]
        std::vector<IloNumVarArray> phi_w(n + p);
        for (int i = 0; i < n + p; ++i) {
            int node_id = (i < n) ? result.new_node_ids[i] : stations_to_consider[i - n];
            phi_w[i] = IloNumVarArray(env, charge_options[node_id].size(), 0, IloInfinity, ILOFLOAT);
        }

        // Variables for station insertion
        IloBoolVarArray z(env, p); // Whether to use each potential station
        std::vector<IloBoolVarArray> y(p); // Position selection for each station
        for (int l = 0; l < p; ++l) {
            y[l] = IloBoolVarArray(env, valid_positions.size());
        }

        // Ensure at most one station is inserted at each position
        IloExprArray pos_usage(env, valid_positions.size());
        for (int k = 0; k < valid_positions.size(); ++k) {
            pos_usage[k] = IloExpr(env);
            for (int l = 0; l < p; ++l) {
                pos_usage[k] += y[l][k];
            }
            model.add(pos_usage[k] <= 1);
        }

        // Each station is inserted at most at one position
        for (int l = 0; l < p; ++l) {
            IloExpr sum_y(env);
            for (int k = 0; k < valid_positions.size(); ++k) {
                sum_y += y[l][k];
            }
            model.add(sum_y == z[l]);
        }

        // Objective function
        IloExpr obj(env);
        for (int i = 0; i < n + p; ++i) {
            int node_id = (i < n) ? result.new_node_ids[i] : stations_to_consider[i - n];
            for (size_t kk = 0; kk < charge_options[node_id].size(); ++kk) {
                obj += charge_options[node_id][kk].getCost() * charge_options[node_id][kk].getRate() * phi_w[i][kk];
            }
        }
        obj += params.getTimeCost() * t[n + p - 1];
        model.add(IloMinimize(env, obj));

        // Linearization constraints for phi[i] * w[i][k]
        for (int i = 0; i < n + p; ++i) {
            int node_id = (i < n) ? result.new_node_ids[i] : stations_to_consider[i - n];
            for (size_t kk = 0; kk < charge_options[node_id].size(); ++kk) {
                double U_phi = params.getBatteryCapacity() / (charge_options[node_id].size() > 0 ? charge_options[node_id][0].getRate() : 1.0);
                model.add(phi_w[i][kk] <= phi[i]);
                model.add(phi_w[i][kk] <= U_phi * w[i][kk]);
                model.add(phi_w[i][kk] >= phi[i] - U_phi * (1 - w[i][kk]));
                model.add(phi_w[i][kk] >= 0);
            }
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
                model.add(phi_w[i][kk] <= params.getBatteryCapacity() * z[l]);
            }
        }

        // Time constraints
        model.add(depart[0] == t[0]);
        model.add(t[0] == 0);

        // Build new route dynamically
        std::vector<int> new_route = result.new_node_ids;
        std::vector<int> node_mapping(n + p, -1); // Maps variable indices to new_route indices
        node_mapping[0] = 0; // First node is always the depot
        int current_idx = 1; // Index in new_route

        for (size_t i = 1; i < n; ++i) {
            // Check if a station is inserted before position i
            IloExpr travel_time(env);
            for (int l = 0; l < p; ++l) {
                for (size_t k = 0; k < valid_positions.size(); ++k) {
                    if (valid_positions[k] == i) {
                        // Insert station l at position i
                        model.add(t[n + l] >= depart[current_idx - 1] + travel_time);
                        model.add(depart[n + l] == t[n + l] + phi[n + l]);
                        node_mapping[n + l] = current_idx;
                        current_idx++;
                        const Arc* arc = graph.findArc(stations_to_consider[l], result.new_node_ids[i]);
                        if (!arc) {
                            std::cerr << "Error: Arc from " << stations_to_consider[l] << " to " << result.new_node_ids[i] << " not found." << std::endl;
                            env.end();
                            return result;
                        }
                        double d = arc->getDistance();
                        travel_time = d / params.getUmax() * y[l][k];
                    }
                }
            }
            // Add original node i
            node_mapping[i] = current_idx;
            const Arc* arc = graph.findArc(new_route[current_idx - 1], result.new_node_ids[i]);
            if (!arc) {
                std::cerr << "Error: Arc from " << new_route[current_idx - 1] << " to " << result.new_node_ids[i] << " not found." << std::endl;
                env.end();
                return result;
            }
            double d = arc->getDistance();
            double min_s = d / params.getUmax();
            double max_s = d / params.getUmin();
            model.add(t[i] >= depart[current_idx - 1] + s[current_idx - 1]);
            s[current_idx - 1].setBounds(min_s, max_s);
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
            model.add(ya[i] >= params.getMinSoc());
            model.add(yd[i] >= params.getMinSoc());
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
            const Arc* arc = graph.findArc(new_route[i - 1], new_route[i]);
            if (!arc) {
                std::cerr << "Error: Arc from " << new_route[i - 1] << " to " << new_route[i] << " not found." << std::endl;
                env.end();
                return result;
            }
            double d = arc->getDistance();
            model.add(ya[var_idx] <= yd[prev_var_idx] - params.getEnergyConsumption() * d);
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
            model.add(sum_w <= z[l]);
        }

        // Solve the model
        cplex.setOut(env.getNullStream());
        if (!cplex.solve()) {
            std::cerr << "CPLEX: No solution found." << std::endl;
            env.end();
            return result;
        }

        // Extract solution
        result.cost = cplex.getObjValue();
        result.feasible = true;

        // Build new route
        result.new_node_ids = current_route.getNodeIds();
        for (int l = 0; l < p; ++l) {
            if (cplex.getValue(z[l]) > 0.5) {
                for (size_t k = 0; k < valid_positions.size(); ++k) {
                    if (cplex.getValue(y[l][k]) > 0.5) {
                        result.new_node_ids.insert(result.new_node_ids.begin() + valid_positions[k], stations_to_consider[l]);
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

        for (int i = 0; i < n + p; ++i) {
            if (node_mapping[i] != -1) {
                result.soc_arrival[node_mapping[i]] = cplex.getValue(ya[i]);
                result.soc_departure[node_mapping[i]] = cplex.getValue(yd[i]);
                result.arrival_time[node_mapping[i]] = cplex.getValue(t[i]);
                result.departure_time[node_mapping[i]] = cplex.getValue(depart[i]);
                int node_id = (i < n) ? result.new_node_ids[node_mapping[i]] : stations_to_consider[i - n];
                for (size_t kk = 0; kk < charge_options[node_id].size(); ++kk) {
                    if (cplex.getValue(w[i][kk]) > 0.5) {
                        result.charging_decisions[node_mapping[i]] = ChargingDecision(node_id, static_cast<int>(kk + 1), cplex.getValue(phi[i]));
                        break;
                    }
                }
            }
        }

        env.end();
        return result;
    } catch (IloException& e) {
        std::cerr << "CPLEX Exception: " << e.getMessage() << std::endl;
        return result;
    } catch (std::exception& e) {
        std::cerr << "Standard Exception: " << e.what() << std::endl;
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
            if (std::uniform_real_distribution<>(0, 1)(rng) < 0.5) {
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
    if (stations.empty()) return route;

    // Add a random station to potential_stations
    int station = stations[std::uniform_int_distribution<>(0, stations.size() - 1)(rng)];
    potential_stations.push_back(station);

    // Return original route (no direct insertion)
    return route;
}

Route VNS::removeStation(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> new_nodes = route.getNodeIds();
    std::vector<size_t> station_indices;
    for (size_t i = 1; i < new_nodes.size() - 1; ++i) {
        if (Utils::isChargingStation(new_nodes[i], graph.getNodes()) &&
            Utils::isCustomer(new_nodes[i - 1], graph.getNodes()) &&
            Utils::isCustomer(new_nodes[i + 1], graph.getNodes())) {
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
    for (int id : route_node_ids) {
        if (!graph.findNode(id)) return false;
        if (Utils::isCustomer(id, graph.getNodes())) customers.insert(id);
    }
    return true;
}

bool VNS::quickFeasibilityCheck(const Route& route, const Graph& graph, const Parameters& params) {
    double soc = params.getInitialSoc();
    const auto& nodes = route.getNodeIds();
    for (size_t i = 0; i < nodes.size() - 1; ++i) {
        const Arc* arc = graph.findArc(nodes[i], nodes[i + 1]);
        if (!arc) return false;
        soc -= params.getEnergyConsumption() * arc->getDistance();
        if (soc < params.getMinSoc()) return false;
        if (Utils::isChargingStation(nodes[i + 1], graph.getNodes())) {
            soc = params.getBatteryCapacity();
        }
    }
    return soc >= params.getMinSoc();
}