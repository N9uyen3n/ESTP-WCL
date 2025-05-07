#include "../include/VNS.h"
#include "../include/Utils.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <random>
#include <chrono>
#include <set>
#include <numeric>
#include <iomanip>

VNS::VNS(int max_iterations, int max_neighborhoods, unsigned int seed, const std::vector<int>& s_prime)
    : max_iterations(max_iterations), max_neighborhoods(max_neighborhoods),
      rng(std::mt19937(seed)), S_prime(s_prime),
      operator_weights_{0.4, 0.3, 0.2, 0.1} {
    normalizeWeights();
}

VNS::~VNS() {
    env.end();
}

void VNS::normalizeWeights() {
    double sum = std::accumulate(operator_weights_.begin(), operator_weights_.end(), 0.0);
    if (sum > 1e-9) {
        for (double& w : operator_weights_) {
            w /= sum;
        }
    } else {
        double val = 1.0 / operator_weights_.size();
        for (double& w : operator_weights_) {
            w = val;
        }
    }
}

bool VNS::checkSignificantImprovement(double old_cost, double new_cost) {
    if (old_cost <= 0) return false;
    return (old_cost - new_cost) / old_cost >= 0.0001; // 0.01%
}

int VNS::selectOperatorWeighted() {
    std::discrete_distribution<int> dist(operator_weights_.begin(), operator_weights_.end());
    return dist(rng); // Returns 0 to 3
}

bool VNS::quickFeasibilityCheck(const Route& route, const Graph& graph, const Parameters& params) {
    const auto& node_ids = route.getNodeIds();
    if (!isValidRoute(node_ids, graph)) {
        return false;
    }

    double soc = params.getInitialSoc();
    double total_distance = 0.0;
    for (size_t i = 0; i < node_ids.size() - 1; ++i) {
        const Arc* arc = graph.findArc(node_ids[i], node_ids[i + 1]);
        if (!arc) return false;
        double distance = arc->getDistance();
        total_distance += distance;
        double energy_consumed = params.getEnergyConsumption() * distance;
        soc -= energy_consumed;
        if (arc->getIsWireless()) {
            double wireless_charge = arc->getWirelessChargeRate() * distance; // Charge per km
            soc = std::min(soc + wireless_charge, params.getBatteryCapacity());
        }
        if (soc < params.getMinSoc() - 1e-6) {
            if (Utils::isChargingStation(node_ids[i + 1], graph.getNodes())) {
                soc = params.getBatteryCapacity(); // Full charge at station
            } else {
                return false; // SOC too low, no charging available
            }
        }
    }
    // Ensure total energy consumption is feasible
    double min_energy_needed = params.getEnergyConsumption() * total_distance;
    double max_wireless_charge = 0.0;
    for (size_t i = 0; i < node_ids.size() - 1; ++i) {
        const Arc* arc = graph.findArc(node_ids[i], node_ids[i + 1]);
        if (arc && arc->getIsWireless()) {
            max_wireless_charge += arc->getWirelessChargeRate() * arc->getDistance();
        }
    }
    if (params.getInitialSoc() + max_wireless_charge < min_energy_needed + params.getMinSoc()) {
        return false; // Insufficient energy even with wireless charging
    }
    return soc >= params.getMinSoc();
}

Route VNS::evaluateRoute(Route& route,
                         const Graph& graph,
                         const std::vector<std::vector<ChargingOption>>& charge_options,
                         const Parameters& params) {
    if (!quickFeasibilityCheck(route, graph, params)) {
        route.setTotalCost(1e9);
        route.setFeasible(false);
        return route;
    }
    return solveSubproblem1(route, graph, charge_options, params);
}

Route VNS::optimize(const std::vector<int>& initial_nodes,
                    const Graph& graph,
                    const std::vector<std::vector<ChargingOption>>& charge_options,
                    const Parameters& params) {
    if (!Utils::validateInitialNodes(initial_nodes, graph)) {
        throw std::runtime_error("The original node list is invalid");
    }

    Route best_route = Route(initial_nodes);
    best_route = evaluateRoute(best_route, graph, charge_options, params);
    double best_cost = best_route.getTotalCost();
    Route current_route = best_route;
    double current_cost = best_cost;
    int no_improvement_counter = 0;

    for (int iter = 0; iter < max_iterations; ++iter) {
        int k = selectOperatorWeighted();
        Route s_prime = shake(current_route, k, graph, params);

        if (quickFeasibilityCheck(s_prime, graph, params)) {
            s_prime = localSearch(s_prime, graph, charge_options, params);
        } else {
            s_prime.setTotalCost(1e9);
            s_prime.setFeasible(false);
        }

        double s_prime_cost = s_prime.getTotalCost();

        if (s_prime_cost < best_cost) {
            best_route = s_prime;
            current_route = s_prime;
            best_cost = s_prime_cost;
            current_cost = s_prime_cost;
            operator_weights_[k] *= 1.1;
            normalizeWeights();
            if (checkSignificantImprovement(best_cost, s_prime_cost)) {
                no_improvement_counter = 0;
            } else {
                no_improvement_counter++;
            }
        } else if (s_prime_cost < current_cost) {
            current_route = s_prime;
            current_cost = s_prime_cost;
            no_improvement_counter++;
        } else {
            no_improvement_counter++;
        }

        if ((iter + 1) % 10 == 0) {
            std::cout << "Iteration " << iter + 1 << ":\n";
            std::cout << "Best cost: " << std::fixed << std::setprecision(2) << best_cost << "\n";
            std::cout << "No-Improvement Counter: " << no_improvement_counter << "\n";
            std::cout << "Best route: [";
            const auto& nodes = best_route.getNodeIds();
            for (size_t i = 0; i < nodes.size(); ++i) {
                std::cout << nodes[i];
                if (i < nodes.size() - 1) std::cout << "->";
            }
            std::cout << "]\n";
            std::cout << "------------------------\n";
        }
    }

    return best_route;
}

bool VNS::isValidRoute(const std::vector<int>& route, const Graph& graph) {
    if (route.empty() || route.front() != 0 || route.back() != 0) {
        return false;
    }

    std::set<int> required_customers;
    for (int id : S_prime) {
        const Node* node = graph.findNode(id);
        if (node && node->getType() == NodeType::CUSTOMER) {
            required_customers.insert(id);
        }
    }

    std::set<int> visited_customers;
    for (size_t i = 1; i < route.size() - 1; ++i) {
        const Node* node = graph.findNode(route[i]);
        if (!node || route[i] == 0) return false;
        if (node->getType() == NodeType::CUSTOMER) {
            if (visited_customers.count(route[i])) return false;
            visited_customers.insert(route[i]);
        }
    }

    if (visited_customers != required_customers) {
        return false;
    }

    for (size_t i = 0; i < route.size() - 1; ++i) {
        if (!graph.findArc(route[i], route[i + 1])) {
            return false;
        }
    }
    return true;
}

Route VNS::solveSubproblem(int i, int j, int a, const Route& current_route,
                           const Graph& graph,
                           const std::vector<std::vector<ChargingOption>>& charge_options,
                           const Parameters& params) {
    return current_route; // Not used in VNS
}

Route VNS::solveSubproblem1(const Route& current_route,
                           const Graph& graph,
                           const std::vector<std::vector<ChargingOption>>& charge_options,
                           const Parameters& params) {
    IloModel model(env);
    SubproblemResult result;

    try {
        const auto& node_ids = current_route.getNodeIds();
        if (node_ids.size() < 2) {
            Route infeasible_route(node_ids);
            infeasible_route.setTotalCost(1e9);
            infeasible_route.setFeasible(false);
            return infeasible_route;
        }

        int n = node_ids.size();
        int m = n - 1;

        // Arc data
        std::vector<double> d_ij(m);
        std::vector<double> beta_ij(m);
        std::vector<bool> is_wireless_route(m);
        std::vector<double> min_s(m);
        std::vector<double> max_s(m);
        for (int k = 0; k < m; ++k) {
            int i = node_ids[k];
            int j = node_ids[k + 1];
            const Arc* arc = graph.findArc(i, j);
            if (!arc) {
                Route infeasible_route(node_ids);
                infeasible_route.setTotalCost(1e9);
                infeasible_route.setFeasible(false);
                return infeasible_route;
            }
            d_ij[k] = arc->getDistance();
            beta_ij[k] = arc->getWirelessChargeRate();
            is_wireless_route[k] = arc->getIsWireless();
            // Handle zero-distance arcs
            if (d_ij[k] < 1e-6) {
                min_s[k] = 0.0;
                max_s[k] = 0.0;
            } else {
                min_s[k] = d_ij[k] / params.getUmax();
                max_s[k] = d_ij[k] / params.getUmin();
            }
        }

        // Decision variables
        IloNumVarArray phi(env, n, 0, IloInfinity, ILOFLOAT); // Charging time
        std::vector<IloBoolVarArray> w(n);
        std::vector<IloNumVarArray> phi_w(n);
        for (int i = 0; i < n; ++i) {
            int node = node_ids[i];
            size_t num_options = charge_options[node].size();
            w[i] = IloBoolVarArray(env, num_options);
            phi_w[i] = IloNumVarArray(env, num_options, 0, IloInfinity, ILOFLOAT);
            for (size_t k = 0; k < num_options; ++k) {
                w[i][k] = IloBoolVar(env, ("w_" + std::to_string(i) + "_" + std::to_string(k)).c_str());
            }
        }
        IloNumVarArray s(env, m, 0, IloInfinity, ILOFLOAT); // Travel time
        for (int k = 0; k < m; ++k) {
            s[k].setBounds(min_s[k], max_s[k]);
        }
        IloNumVarArray ya(env, n, 0.0, params.getBatteryCapacity(), ILOFLOAT); // SOC upon arrival
        IloNumVarArray yd(env, n, 0.0, params.getBatteryCapacity(), ILOFLOAT); // SOC upon departure
        IloNumVarArray t(env, n, 0, IloInfinity, ILOFLOAT); // Arrival time
        IloNumVarArray depart(env, n, 0, IloInfinity, ILOFLOAT); // Departure time

        // Wireless charging variables
        std::vector<int> wireless_k;
        for (int k = 0; k < m; ++k) {
            if (is_wireless_route[k]) {
                wireless_k.push_back(k);
            }
        }
        int p = wireless_k.size();
        IloBoolVarArray z(env, p);
        std::vector<IloNumVar> w_s_z(p);
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            double M = max_s[k] + 1e-6; // Avoid zero M
            w_s_z[l] = IloNumVar(env, 0, M, ILOFLOAT);
        }

        // Objective function
        IloExpr obj(env);
        for (int i = 0; i < n; ++i) {
            int node_i = node_ids[i];
            for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                obj += charge_options[node_i][k].getCost() * charge_options[node_i][k].getRate() * phi_w[i][k];
            }
        }
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            obj += params.getWirelessCost() * beta_ij[k] * w_s_z[l];
        }
        obj += params.getTimeCost() * t[n - 1];
        model.add(IloMinimize(env, obj));
        obj.end();

        // Linearization constraints for phi_w
        for (int i = 0; i < n; ++i) {
            int node = node_ids[i];
            double max_rate = 1.0;
            for (const auto& option : charge_options[node]) {
                max_rate = std::max(max_rate, option.getRate());
            }
            double U_phi = params.getBatteryCapacity() / (max_rate > 0 ? max_rate : 1.0);
            for (size_t k = 0; k < charge_options[node].size(); ++k) {
                model.add(phi_w[i][k] <= phi[i]);
                model.add(phi_w[i][k] <= U_phi * w[i][k]);
                model.add(phi_w[i][k] >= phi[i] - U_phi * (1 - w[i][k]));
                model.add(phi_w[i][k] >= 0);
            }
        }

        // Zero charging time for non-charging nodes
        for (int i = 0; i < n; ++i) {
            int node = node_ids[i];
            if (charge_options[node].empty()) {
                model.add(phi[i] == 0);
                for (size_t k = 0; k < w[i].getSize(); ++k) {
                    model.add(w[i][k] == 0);
                    model.add(phi_w[i][k] == 0);
                }
            }
        }

        // Time constraints
        model.add(t[0] == 0);
        model.add(depart[0] == t[0]);
        for (int i = 1; i < n; ++i) {
            int node = node_ids[i];
            if (!charge_options[node].empty()) {
                model.add(depart[i] == t[i] + phi[i]);
            } else {
                const Node* node_ptr = graph.findNode(node);
                double service_time = node_ptr ? node_ptr->getServiceTime() : 0.0;
                model.add(depart[i] == t[i] + service_time);
            }
        }
        for (int k = 0; k < m; ++k) {
            if (d_ij[k] < 1e-6) {
                model.add(t[k + 1] == depart[k]); // Zero-distance arc
            } else {
                model.add(t[k + 1] >= depart[k] + s[k]);
            }
        }

        // SOC constraints
        model.add(ya[0] == params.getInitialSoc());
        for (int i = 0; i < n; ++i) {
            int node = node_ids[i];
            if (!charge_options[node].empty()) {
                IloExpr charge_amount(env);
                for (size_t k = 0; k < charge_options[node].size(); ++k) {
                    charge_amount += charge_options[node][k].getRate() * phi_w[i][k];
                }
                model.add(yd[i] <= ya[i] + charge_amount);
                model.add(yd[i] <= params.getBatteryCapacity());
                model.add(yd[i] >= ya[i]);
                charge_amount.end();
            } else {
                model.add(yd[i] == ya[i]);
            }
        }
        for (int k = 0; k < m; ++k) {
            int i_idx = k;
            int j_idx = k + 1;
            if (d_ij[k] < 1e-6) {
                model.add(ya[j_idx] == yd[i_idx]); // No energy consumption
            } else if (!is_wireless_route[k]) {
                model.add(ya[j_idx] == yd[i_idx] - params.getEnergyConsumption() * d_ij[k]);
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
                    model.add(ya[j_idx] >= yd[i_idx] - params.getEnergyConsumption() * d_ij[k]);
                } else {
                    Route infeasible_route(node_ids);
                    infeasible_route.setTotalCost(1e9);
                    infeasible_route.setFeasible(false);
                    return infeasible_route;
                }
            }
            // Ensure SOC does not drop below min_soc
            model.add(ya[j_idx] >= params.getMinSoc());
        }

        // Linearization constraints for w_s_z
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            double M = max_s[k] + 1e-6;
            model.add(w_s_z[l] <= s[k]);
            model.add(w_s_z[l] <= M * z[l]);
            model.add(w_s_z[l] >= s[k] - M * (1 - z[l]));
            model.add(w_s_z[l] >= 0);
        }

        // Charging option selection
        for (int i = 0; i < n; ++i) {
            int node = node_ids[i];
            if (!charge_options[node].empty()) {
                IloExpr sum_w(env);
                for (size_t k = 0; k < charge_options[node].size(); ++k) {
                    sum_w += w[i][k];
                }
                model.add(sum_w <= 1);
                sum_w.end();
            }
        }

        // Solve MILP
        IloCplex cplex(model);
        cplex.setOut(std::cout); // Enable CPLEX logging for debugging
        cplex.setParam(IloCplex::TiLim, 30.0);
        cplex.setParam(IloCplex::EpGap, 0.01);
        if (!cplex.solve()) {
            std::cerr << "CPLEX failed to solve: " << cplex.getStatus() << "\n";
            if (cplex.getStatus() == IloAlgorithm::Infeasible) {
                // Attempt to find the infeasibility
                cplex.setParam(IloCplex::ConflictDisplay, 2);

                std::cout << "Conflict found:" << std::endl;
                cplex.writeConflict("conflict.log");
            }
            Route infeasible_route(node_ids);
            infeasible_route.setTotalCost(1e9);
            infeasible_route.setFeasible(false);
            return infeasible_route;
        }

        // Populate result
        result.cost = cplex.getObjValue();
        result.new_node_ids = node_ids;
        result.soc_arrival.resize(n);
        result.soc_departure.resize(n);
        result.arrival_time.resize(n);
        result.departure_time.resize(n);
        result.charging_decisions.clear();
        result.wireless_decisions.resize(m, false);

        for (int i = 0; i < n; ++i) {
            result.soc_arrival[i] = cplex.getValue(ya[i]);
            result.soc_departure[i] = cplex.getValue(yd[i]);
            result.arrival_time[i] = cplex.getValue(t[i]);
            result.departure_time[i] = cplex.getValue(depart[i]);
            int node = node_ids[i];
            if (!charge_options[node].empty()) {
                for (size_t k = 0; k < charge_options[node].size(); ++k) {
                    if (cplex.getValue(w[i][k]) > 0.5) {
                        result.charging_decisions.emplace_back(node, static_cast<int>(k), cplex.getValue(phi[i]));
                        break;
                    }
                }
            }
        }
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            result.wireless_decisions[k] = cplex.getValue(z[l]) > 0.5;
        }

        Route result_route(result.new_node_ids);
        updateRoute(result_route, result);
        return result_route;

    } catch (const IloException& e) {
        std::cerr << "CPLEX Exception in solveSubproblem1: " << e.getMessage() << "\n";
        Route infeasible_route(current_route.getNodeIds());
        infeasible_route.setTotalCost(1e9);
        infeasible_route.setFeasible(false);
        return infeasible_route;
    } catch (const std::exception& e) {
        std::cerr << "Exception in solveSubproblem1: " << e.what() << "\n";
        Route infeasible_route(current_route.getNodeIds());
        infeasible_route.setTotalCost(1e9);
        infeasible_route.setFeasible(false);
        return infeasible_route;
    }
}

Route VNS::localSearch(const Route& current_route, const Graph& graph,
                       const std::vector<std::vector<ChargingOption>>& charge_options,
                       const Parameters& params) {
    if (!isValidRoute(current_route.getNodeIds(), graph) || !quickFeasibilityCheck(current_route, graph, params)) {
        Route infeasible_route(current_route.getNodeIds());
        infeasible_route.setTotalCost(1e9);
        infeasible_route.setFeasible(false);
        return infeasible_route;
    }
    return solveSubproblem1(current_route, graph, charge_options, params);
}

Route VNS::shake(const Route& current_route, int neighborhood, const Graph& graph, const Parameters& params) {
    switch (neighborhood) {
        case 0: return twoOpt(current_route, graph, params);
        case 1: return relocate(current_route, graph, params);
        case 2: return swapNodes(current_route, graph, params);
        case 3: {
            bool has_station = false;
            for (int id : current_route.getNodeIds()) {
                if (Utils::isChargingStation(id, graph.getNodes())) {
                    has_station = true;
                    break;
                }
            }
            if (!has_station || rng() % 2 == 0) {
                return insertStation(current_route, graph, params);
            } else {
                return removeStation(current_route, graph, params);
            }
        }
        default: return current_route;
    }
}

Route VNS::swapNodes(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> node_ids = route.getNodeIds();
    if (node_ids.size() <= 3) return route;
    std::uniform_int_distribution<size_t> dist(1, node_ids.size() - 2);
    size_t i = dist(rng);
    size_t j = dist(rng);
    while (i == j) j = dist(rng);
    std::swap(node_ids[i], node_ids[j]);
    return Route(node_ids);
}

Route VNS::relocate(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> node_ids = route.getNodeIds();
    if (node_ids.size() <= 3) return route;
    std::uniform_int_distribution<size_t> dist(1, node_ids.size() - 2);
    size_t i = dist(rng);
    size_t j = dist(rng);
    while (i == j) j = dist(rng);
    int node = node_ids[i];
    node_ids.erase(node_ids.begin() + i);
    node_ids.insert(node_ids.begin() + j, node);
    return Route(node_ids);
}

Route VNS::insertStation(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> node_ids = route.getNodeIds();
    std::vector<int> stations;
    for (const auto& node : graph.getNodes()) {
        if (node.getType() == NodeType::CHARGING_STATION) {
            stations.push_back(node.getId());
        }
    }
    if (stations.empty() || node_ids.size() <= 2) return route;
    std::uniform_int_distribution<size_t> dist(1, node_ids.size() - 1);
    size_t pos = dist(rng);
    int station = stations[rng() % stations.size()];
    node_ids.insert(node_ids.begin() + pos, station);
    return Route(node_ids);
}

Route VNS::removeStation(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> node_ids = route.getNodeIds();
    std::vector<size_t> station_positions;
    for (size_t i = 1; i < node_ids.size() - 1; ++i) {
        if (Utils::isChargingStation(node_ids[i], graph.getNodes())) {
            station_positions.push_back(i);
        }
    }
    if (station_positions.empty()) return route;
    size_t pos = station_positions[rng() % station_positions.size()];
    node_ids.erase(node_ids.begin() + pos);
    return Route(node_ids);
}

Route VNS::twoOpt(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> node_ids = route.getNodeIds();
    if (node_ids.size() <= 3) return route;
    std::uniform_int_distribution<size_t> dist(1, node_ids.size() - 2);
    size_t i = dist(rng);
    size_t j = i + 1 + (rng() % (node_ids.size() - i - 1));
    std::reverse(node_ids.begin() + i, node_ids.begin() + j);
    return Route(node_ids);
}

void VNS::updateRoute(Route& route, const SubproblemResult& result) {
    route.setSocArrival(result.soc_arrival);
    route.setSocDeparture(result.soc_departure);
    route.setArrivalTime(result.arrival_time);
    route.setDepartureTime(result.departure_time);
    route.setChargingDecisions(result.charging_decisions);
    route.setWirelessDecisions(result.wireless_decisions);
    route.setTotalCost(result.cost);
    route.setFeasible(result.cost < 1e9);
}