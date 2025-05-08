#include "../include/VNS.h"
#include "../include/Utils.h"
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <iostream>

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
        }

        if ((iteration + 1) % 10 == 0) {
            std::cout << "Iteration " << iteration + 1 << ":\n";
            std::cout << "  Best Cost: " << best_cost << "\n";
            std::cout << "  Current Route: ";
            for (int id : current_route) std::cout << id << " ";
            std::cout << "\n  Operator Weights: ";
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
    result.cost = params.getBigM();

    try {
        IloModel model(env);

        // Decision variables
        size_t n = result.new_node_ids.size();
        IloNumVarArray arrival_time(env, n, 0, IloInfinity, ILOFLOAT); // t_ik
        IloNumVarArray soc_arrival(env, n, 0, params.getBatteryCapacity(), ILOFLOAT); // ya_ik
        IloNumVarArray soc_departure(env, n, 0, params.getBatteryCapacity(), ILOFLOAT); // yd_ik
        IloNumVarArray travel_time(env, n - 1, 0, IloInfinity, ILOFLOAT); // s_(ik-1,ik)
        std::vector<IloBoolVarArray> charging_option; // w_ik,k
        std::vector<IloNumVarArray> charging_time; // phi_ik
        std::vector<IloNumVarArray> charge_amount; // Biến phụ cho tuyến tính hóa
        IloBoolVarArray wireless_usage(env, n - 1); // z_(ik-1,ik)

        // Initialize charging variables for charging stations
        for (size_t i = 0; i < n; ++i) {
            if (Utils::isChargingStation(result.new_node_ids[i], graph.getNodes())) {
                const Node* node = graph.findNode(result.new_node_ids[i]);
                if (!node) throw std::runtime_error("Invalid station node ID");
                int station_idx = node->getId();
                if (station_idx >= static_cast<int>(charge_options.size()) || charge_options[station_idx].empty()) {
                    throw std::runtime_error("Invalid charging options for station");
                }
                charging_option.emplace_back(env, charge_options[station_idx].size());
                charging_time.emplace_back(env, charge_options[station_idx].size(), 0, IloInfinity, ILOFLOAT);
                charge_amount.emplace_back(env, charge_options[station_idx].size(), 0, params.getBatteryCapacity(), ILOFLOAT);
            }
        }

        // Objective function
        IloExpr obj(env);
        for (size_t i = 0, station_count = 0; i < n; ++i) {
            if (Utils::isChargingStation(result.new_node_ids[i], graph.getNodes())) {
                int station_idx = result.new_node_ids[i];
                for (size_t k = 0; k < charge_options[station_idx].size(); ++k) {
                    obj += charge_options[station_idx][k].getCost() * charge_amount[station_count][k];
                }
                station_count++;
            }
        }
        for (size_t i = 0; i < n - 1; ++i) {
            const Arc* arc = graph.findArc(result.new_node_ids[i], result.new_node_ids[i + 1]);
            if (arc && arc->getIsWireless()) {
                obj += params.getWirelessCost() * arc->getWirelessChargeRate() * travel_time[i] * wireless_usage[i];
            }
        }
        obj += params.getTimeCost() * arrival_time[n - 1];
        model.add(IloMinimize(env, obj));

        // Constraints
        // 4.5.1 Time Progression
        for (size_t i = 1; i < n; ++i) {
            const Arc* arc = graph.findArc(result.new_node_ids[i - 1], result.new_node_ids[i]);
            if (!arc) throw std::runtime_error("Invalid arc");
            IloExpr time_expr(env);
            time_expr = arrival_time[i] - arrival_time[i - 1] - travel_time[i - 1];
            if (Utils::isCustomer(result.new_node_ids[i - 1], graph.getNodes())) {
                time_expr -= graph.findNode(result.new_node_ids[i - 1])->getServiceTime();
            } else if (Utils::isChargingStation(result.new_node_ids[i - 1], graph.getNodes())) {
                int station_idx = 0;
                for (size_t k = 0; k < i - 1; ++k) {
                    if (Utils::isChargingStation(result.new_node_ids[k], graph.getNodes())) station_idx++;
                }
                for (size_t k = 0; k < charge_options[result.new_node_ids[i - 1]].size(); ++k) {
                    time_expr -= charging_time[station_idx][k];
                }
            }
            model.add(time_expr >= 0);
        }
        model.add(arrival_time[0] == 0);

        // 4.5.2 Travel Time Bounds
        for (size_t i = 0; i < n - 1; ++i) {
            const Arc* arc = graph.findArc(result.new_node_ids[i], result.new_node_ids[i + 1]);
            if (arc) {
                model.add(travel_time[i] >= arc->getDistance() / params.getUmax());
                model.add(travel_time[i] <= arc->getDistance() / params.getUmin());
            }
        }

        // 4.5.3 SOC Consistency
        for (size_t i = 1; i < n; ++i) {
            const Arc* arc = graph.findArc(result.new_node_ids[i - 1], result.new_node_ids[i]);
            if (!arc) continue;
            if (arc->getIsWireless()) {
                model.add(soc_arrival[i] == soc_departure[i - 1] -
                          params.getEnergyConsumption() * arc->getDistance() +
                          arc->getWirelessChargeRate() * travel_time[i - 1] * wireless_usage[i - 1]);
            } else {
                model.add(soc_arrival[i] == soc_departure[i - 1] -
                          params.getEnergyConsumption() * arc->getDistance());
            }

            if (Utils::isChargingStation(result.new_node_ids[i], graph.getNodes())) {
                int station_idx = 0;
                for (size_t k = 0; k < i; ++k) {
                    if (Utils::isChargingStation(result.new_node_ids[k], graph.getNodes())) station_idx++;
                }
                IloExpr charge_total(env);
                for (size_t k = 0; k < charge_options[result.new_node_ids[i]].size(); ++k) {
                    charge_total += charge_amount[station_idx][k];
                }
                model.add(soc_departure[i] == soc_arrival[i] + charge_total);
            } else {
                model.add(soc_departure[i] == soc_arrival[i]);
            }
        }
        model.add(soc_departure[0] == params.getBatteryCapacity());
        model.add(soc_arrival[0] == soc_departure[0]);
        model.add(soc_departure[n - 1] == soc_arrival[n - 1]);

        // 4.5.4 Charging Rate Constraints
        for (size_t i = 0, station_count = 0; i < n; ++i) {
            if (Utils::isChargingStation(result.new_node_ids[i], graph.getNodes())) {
                IloExpr sum_options(env);
                for (size_t k = 0; k < charge_options[result.new_node_ids[i]].size(); ++k) {
                    sum_options += charging_option[station_count][k];
                }
                model.add(sum_options == 1);
                station_count++;
            }
        }

        // 4.5.5 Battery Capacity
        for (size_t i = 0; i < n; ++i) {
            model.add(soc_arrival[i] >= 0);
            model.add(soc_departure[i] >= 0);
            model.add(soc_arrival[i] <= params.getBatteryCapacity());
            model.add(soc_departure[i] <= params.getBatteryCapacity());
        }

        // Linearization Constraints for charging_option and charging_time
        for (size_t i = 0, station_count = 0; i < n; ++i) {
            if (Utils::isChargingStation(result.new_node_ids[i], graph.getNodes())) {
                int station_idx = result.new_node_ids[i];
                for (size_t k = 0; k < charge_options[station_idx].size(); ++k) {
                    // charge_amount_ik_k <= M * w_ik,k
                    model.add(charge_amount[station_count][k] <= params.getBatteryCapacity() * charging_option[station_count][k]);
                    // charge_amount_ik_k <= r_ik,k * phi_ik
                    model.add(charge_amount[station_count][k] <=
                              charge_options[station_idx][k].getRate() * charging_time[station_count][k]);
                    // charge_amount_ik_k >= 0
                    model.add(charge_amount[station_count][k] >= 0);
                }
                station_count++;
            }
        }

        // Solve MILP
        IloCplex cplex(model);
        cplex.setOut(env.getNullStream());
        if (cplex.solve()) {
            result.cost = cplex.getObjValue();
            result.feasible = true;
            result.arrival_time.resize(n);
            result.departure_time.resize(n);
            result.soc_arrival.resize(n);
            result.soc_departure.resize(n);
            for (size_t i = 0; i < n; ++i) {
                result.arrival_time[i] = cplex.getValue(arrival_time[i]);
                result.soc_arrival[i] = cplex.getValue(soc_arrival[i]);
                result.soc_departure[i] = cplex.getValue(soc_departure[i]);
                result.departure_time[i] = result.arrival_time[i];
                if (Utils::isChargingStation(result.new_node_ids[i], graph.getNodes())) {
                    int station_idx = 0;
                    for (size_t k = 0; k < i; ++k) {
                        if (Utils::isChargingStation(result.new_node_ids[k], graph.getNodes())) station_idx++;
                    }
                    for (size_t k = 0; k < charge_options[result.new_node_ids[i]].size(); ++k) {
                        double ct = cplex.getValue(charging_time[station_idx][k]);
                        if (cplex.getValue(charging_option[station_idx][k]) > 0.5 && ct > 0) {
                            result.charging_decisions.emplace_back(result.new_node_ids[i], k, ct);
                            result.departure_time[i] += ct;
                        }
                    }
                } else if (Utils::isCustomer(result.new_node_ids[i], graph.getNodes())) {
                    result.departure_time[i] += graph.findNode(result.new_node_ids[i])->getServiceTime();
                }
            }
            for (size_t i = 0; i < n - 1; ++i) {
                result.wireless_decisions.push_back(cplex.getValue(wireless_usage[i]) > 0.5);
            }
        }
    } catch (IloException& e) {
        result.feasible = false;
        result.cost = params.getBigM();
    }
    return result;
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
    std::vector<int> new_nodes = route.getNodeIds();
    const auto& stations = graph.getStationCopies();
    if (stations.empty()) return route;
    int station = stations[std::uniform_int_distribution<>(0, stations.size() - 1)(rng)];
    int pos = std::uniform_int_distribution<>(1, new_nodes.size() - 1)(rng);
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
    if (!Utils::isCustomer(route_node_ids[0], graph.getNodes()) || !Utils::isCustomer(route_node_ids.back(), graph.getNodes())) {
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