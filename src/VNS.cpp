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
      no_improvement_counter(0) {
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
            current_route = local_optimum.getNodeIds();
            no_improvement_counter = 0;
            operator_weights_[neighborhood] *= 1.1;
            normalizeWeights();
        } else {
            no_improvement_counter++;
            if (no_improvement_counter >= 50) {
                break;
            }
            continue;
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
                std::cout << "  Wireless Charging Decisions:\n";
                for (size_t k = 0; k < current_result.wireless_decisions.size(); ++k) {
                    if (current_result.wireless_decisions[k]) {
                        std::cout << "    Arc " << current_result.new_node_ids[k] << " -> "
                                  << current_result.new_node_ids[k + 1] << ": Wireless charging enabled\n";
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
    result.cost = 1e9; // Default cost for infeasible cases
    result.soc_arrival.resize(result.new_node_ids.size(), 0.0);
    result.soc_departure.resize(result.new_node_ids.size(), 0.0);
    result.arrival_time.resize(result.new_node_ids.size(), 0.0);
    result.departure_time.resize(result.new_node_ids.size(), 0.0);
    result.charging_decisions.resize(result.new_node_ids.size(), ChargingDecision(0, 0, 0.0));
    result.wireless_decisions.resize(result.new_node_ids.size() - 1, false);

    // Validate initial_SOC
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
        int m = n - 1; // Number of arcs in the route

        // Calculate data for each arc in the route
        std::vector<double> d_ij(m);
        std::vector<double> beta_ij(m);
        std::vector<bool> is_wireless_route(m);
        std::vector<double> U_min_route(m);
        std::vector<double> U_max_route(m);
        std::vector<double> min_s(m);
        std::vector<double> max_s(m);
        for (int k = 0; k < m; ++k) {
            int i = result.new_node_ids[k];
            int j = result.new_node_ids[k + 1];
            const Arc* arc = graph.findArc(i, j);
            if (!arc) {
                std::cerr << "Error: Arc from " << i << " to " << j << " not found." << std::endl;
                env.end();
                return result; // Return infeasible result
            }
            d_ij[k] = arc->getDistance();
            beta_ij[k] = arc->getWirelessChargeRate();
            is_wireless_route[k] = arc->getIsWireless();
            U_min_route[k] = params.getUmin();
            U_max_route[k] = params.getUmax();
            min_s[k] = d_ij[k] / U_max_route[k];
            max_s[k] = d_ij[k] / U_min_route[k];
        }

        // Decision variables
        IloNumVarArray phi(env, n, 0, IloInfinity, ILOFLOAT); // Charging time
        std::vector<IloBoolVarArray> w(n); // Charging option selection
        for (int i = 0; i < n; ++i) {
            int node = result.new_node_ids[i];
            w[i] = IloBoolVarArray(env, charge_options[node].size());
        }
        IloNumVarArray s(env, m, 0, IloInfinity, ILOFLOAT); // Travel time
        for (int k = 0; k < m; ++k) {
            s[k].setBounds(min_s[k], max_s[k]);
        }
        IloNumVarArray ya(env, n, params.getMinSoc(), params.getBatteryCapacity(), ILOFLOAT); // Arrival SOC
        IloNumVarArray yd(env, n, params.getMinSoc(), params.getBatteryCapacity(), ILOFLOAT); // Departure SOC
        IloNumVarArray t(env, n, 0, IloInfinity, ILOFLOAT); // Arrival time
        IloNumVarArray depart(env, n, 0, IloInfinity, ILOFLOAT); // Departure time

        // Linearization variables for phi[i] * w[i][k]
        std::vector<IloNumVarArray> phi_w(n);
        for (int i = 0; i < n; ++i) {
            int node = result.new_node_ids[i];
            phi_w[i] = IloNumVarArray(env, charge_options[node].size(), 0, IloInfinity, ILOFLOAT);
        }

        // Decision variables for wireless charging
        std::vector<int> wireless_k;
        for (int k = 0; k < m; ++k) {
            if (is_wireless_route[k]) {
                wireless_k.push_back(k);
            }
        }
        int p = wireless_k.size();
        IloBoolVarArray z(env, p); // Wireless charging decision
        std::vector<IloNumVar> w_s_z(p); // Linearization for s[k] * z[l]
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            double M = max_s[k];
            w_s_z[l] = IloNumVar(env, 0, M, ILOFLOAT);
        }

        // Objective function
        IloExpr obj(env);
        for (int i = 0; i < n; ++i) {
            int node_i = result.new_node_ids[i];
            for (size_t kk = 0; kk < charge_options[node_i].size(); ++kk) {
                obj += charge_options[node_i][kk].getCost() * charge_options[node_i][kk].getRate() * phi_w[i][kk];
            }
        }
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            obj += params.getWirelessCost() * beta_ij[k] * w_s_z[l];
        }
        obj += params.getTimeCost() * t[n - 1];
        model.add(IloMinimize(env, obj));

        // Linearization constraints for phi[i] * w[i][k]
        for (int i = 0; i < n; ++i) {
            int node = result.new_node_ids[i];
            for (size_t kk = 0; kk < charge_options[node].size(); ++kk) {
                double U_phi = params.getBatteryCapacity() / (charge_options[node].size() > 0 ? charge_options[node][0].getRate() : 1.0);
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

        // Time constraints
        model.add(depart[0] == t[0]);
        for (int i = 1; i < n; ++i) {
            int node = result.new_node_ids[i];
            if (!charge_options[node].empty()) {
                model.add(depart[i] == t[i] + phi[i]);
            } else if (node != 0) {
                model.add(depart[i] == t[i] + graph.findNode(node)->getServiceTime());
            } else {
                model.add(depart[i] == t[i]);
            }
        }
        model.add(t[0] == 0);
        for (int k = 0; k < m; ++k) {
            int i_idx = k;
            int j_idx = k + 1;
            model.add(t[j_idx] >= depart[i_idx] + s[k]);
        }

        // SOC constraints
        model.add(ya[0] == params.getInitialSoc());
        for (int i = 0; i < n; ++i) {
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
        for (int k = 0; k < m; ++k) {
            int i_idx = k;
            int j_idx = k + 1;
            if (!is_wireless_route[k]) {
                model.add(ya[j_idx] <= yd[i_idx] - params.getEnergyConsumption() * d_ij[k]);
            } else {
                int l = -1;
                for (int ll = 0; ll < p; ++ll) {
                    if (wireless_k[ll] == k) {
                        l = ll;
                        break;
                    }
                }
                if (l != -1) {
                    model.add(ya[j_idx] <= yd[i_idx] - params.getEnergyConsumption() * d_ij[k] + beta_ij[k] * w_s_z[l]);
                } else {
                    std::cerr << "Error: Wireless arc " << k << " not found in wireless_k" << std::endl;
                    env.end();
                    return result;
                }
            }
        }

        // Linearization for w_s_z
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            double M = max_s[k];
            model.add(-k <= s[k]);
            model.add(w_s_z[l] <= M * z[l]);
            model.add(w_s_z[l] >= s[k] - M * (1 - z[l]));
            model.add(w_s_z[l] >= 0);
        }

        // Charging selection constraints
        for (int i = 0; i < n; ++i) {
            int node = result.new_node_ids[i];
            if (!charge_options[node].empty()) {
                IloExpr sum_w(env);
                for (size_t kk = 0; kk < charge_options[node].size(); ++kk) {
                    sum_w += w[i][kk];
                }
                model.add(sum_w <= 1); // Allow no charging
            }
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
        for (int i = 0; i < n; ++i) {
            result.soc_arrival[i] = cplex.getValue(ya[i]);
            result.soc_departure[i] = cplex.getValue(yd[i]);
            result.arrival_time[i] = cplex.getValue(t[i]);
            result.departure_time[i] = cplex.getValue(depart[i]);
            int node = result.new_node_ids[i];
            if (!charge_options[node].empty()) {
                for (size_t kk = 0; kk < charge_options[node].size(); ++kk) {
                    if (cplex.getValue(w[i][kk]) > 0.5) {
                        result.charging_decisions[i] = ChargingDecision(node, static_cast<int>(kk + 1), cplex.getValue(phi[i]));
                        break;
                    }
                }
            }
        }
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            result.wireless_decisions[k] = cplex.getValue(z[l]) > 0.5;
        }

        // Print charging states
        // std::cout << "Charging States for Route: ";
        // for (int id : result.new_node_ids) std::cout << id << " ";
        // std::cout << "\n";
        // for (int i = 0; i < n; ++i) {
        //     if (result.charging_decisions[i].getChargingTime() > 0) {
        //         std::cout << "  Node " << result.new_node_ids[i]
        //                   << ": Station ID = " << result.charging_decisions[i].getStationId()
        //                   << ", Option = " << result.charging_decisions[i].getOption()
        //                   << ", Charging Time = " << result.charging_decisions[i].getChargingTime() << " units\n";
        //     }
        // }
        // std::cout << "Wireless Charging Decisions:\n";
        // for (int l = 0; l < p; ++l) {
        //     int k = wireless_k[l];
        //     if (result.wireless_decisions[k]) {
        //         std::cout << "  Arc " << result.new_node_ids[k] << " -> " << result.new_node_ids[k + 1]
        //                   << ": Wireless charging enabled\n";
        //     }
        // }

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
        // case 0: return twoOpt(shaken_route, graph, params);
        // case 1: return relocate(shaken_route, graph, params);
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
    std::vector<int> new_nodes = route.getNodeIds();
    const auto& stations = graph.getStationCopies();
    if (stations.empty()) return route;
    int station = stations[std::uniform_int_distribution<>(0, stations.size() - 1)(rng)];
    int pos = std::uniform_int_distribution<>(1, new_nodes.size() - 2)(rng);
    new_nodes.insert(new_nodes.begin() + pos, station);
    if (isValidRoute(new_nodes, graph, params)) {
        return Route(new_nodes);
    }
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

    if (route_node_ids[0] != 0 || route_node_ids.back() != 0) return false;
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