#include "../include/VNSOptimizer.h"
#include <algorithm>
#include <iostream>
#include <numeric>
#include <set>
#include <iomanip>
#include <stdexcept>

VNSOptimizer::VNSOptimizer(const std::vector<int>& S_prime,
                           const std::vector<Node>& nodes,
                           const std::vector<Arc>& arcs,
                           const std::vector<std::vector<ChargingOption>>& chg_opts,
                           const Parameters& params)
    : S_prime(S_prime),
      nodes(nodes),
      arcs(arcs),
      charge_options(chg_opts),
      params(params),
      graph_(nodes, arcs),
      rng_(std::random_device()()),
      operator_weights_{0.4, 0.3, 0.2, 0.1},
      best_route_(Route(std::vector<int>{0, 0})), // Placeholder
      no_improvement_counter_(0) {
    normalizeWeights();
}

Route VNSOptimizer::optimize(const std::vector<int>& initial_nodes,
                             const Graph& graph,
                             const std::vector<std::vector<ChargingOption>>& charge_options,
                             const Parameters& params) {
    best_route_ = Route(initial_nodes);
    if (!isValidRoute(initial_nodes) || !quickFeasibilityCheck(best_route_)) {
        best_route_ = createInitialRoute();
    }
    return run(500);
}

Route VNSOptimizer::solveSubproblem(int i, int j, int a, const Route& current_route,
                                   const Graph& graph,
                                   const std::vector<std::vector<ChargingOption>>& charge_options,
                                   const Parameters& params) {
    return current_route;
}

Route VNSOptimizer::solveSubproblem1(const Route& current_route,
                                    const Graph& graph,
                                    const std::vector<std::vector<ChargingOption>>& charge_options,
                                    const Parameters& params) {
    IloEnv env;
    SubproblemResult result;

    try {
        IloModel model(env, "Subproblem1");
        IloCplex cplex(model);

        const auto& node_ids = current_route.getNodeIds();
        if (node_ids.size() < 2) {
            throw std::runtime_error("Invalid route: too short");
        }

        int n = node_ids.size();
        int m = n - 1;

        std::vector<double> d_ij(m);
        std::vector<double> beta_ij(m);
        std::vector<bool> is_wireless_route(m);
        std::vector<double> min_s(m);
        std::vector<double> max_s(m);
        for (int k = 0; k < m; ++k) {
            int i = node_ids[k];
            int j = node_ids[k + 1];
            const Arc* arc = graph_.findArc(i, j);
            if (!arc) {
                Route infeasible_route(node_ids);
                infeasible_route.setTotalCost(1e9);
                infeasible_route.setFeasible(false);
                env.end();
                return infeasible_route;
            }
            d_ij[k] = arc->getDistance();
            beta_ij[k] = arc->getWirelessChargeRate();
            is_wireless_route[k] = arc->getIsWireless();
            min_s[k] = d_ij[k] / params.getUmax();
            max_s[k] = d_ij[k] / params.getUmin();
        }

        IloNumVarArray phi(env, n, 0, IloInfinity, ILOFLOAT);
        std::vector<IloBoolVarArray> w(n);
        for (int i = 0; i < n; ++i) {
            int node = node_ids[i];
            w[i] = IloBoolVarArray(env, charge_options[node].size());
            for (size_t k = 0; k < charge_options[node].size(); ++k) {
                w[i][k] = IloBoolVar(env, ("w_" + std::to_string(i) + "_" + std::to_string(k)).c_str());
            }
        }
        IloNumVarArray s(env, m, 0, IloInfinity, ILOFLOAT);
        for (int k = 0; k < m; ++k) {
            s[k].setBounds(min_s[k], max_s[k]);
        }
        IloNumVarArray ya(env, n, params.getMinSoc(), params.getBatteryCapacity(), ILOFLOAT);
        IloNumVarArray yd(env, n, params.getMinSoc(), params.getBatteryCapacity(), ILOFLOAT);
        IloNumVarArray t(env, n, 0, IloInfinity, ILOFLOAT);
        IloNumVarArray depart(env, n, 0, IloInfinity, ILOFLOAT);

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
            double M = max_s[k];
            w_s_z[l] = IloNumVar(env, 0, M, ILOFLOAT);
        }

        IloExpr obj(env);
        for (int i = 0; i < n; ++i) {
            int node_i = node_ids[i];
            for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                obj += charge_options[node_i][k].getCost() * charge_options[node_i][k].getRate() * phi[i] * w[i][k];
            }
        }
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            obj += params.getWirelessCost() * beta_ij[k] * w_s_z[l];
        }
        obj += params.getTimeCost() * t[n - 1];
        model.add(IloMinimize(env, obj));
        obj.end();

        for (int i = 0; i < n; ++i) {
            int node = node_ids[i];
            if (charge_options[node].empty()) {
                model.add(phi[i] == 0);
            }
        }

        model.add(depart[0] == t[0]);
        for (int i = 1; i < n; ++i) {
            int node = node_ids[i];
            if (!charge_options[node].empty()) {
                model.add(depart[i] == t[i] + phi[i]);
            } else {
                const Node* node_ptr = graph_.findNode(node);
                if (!node_ptr) {
                    Route infeasible_route(node_ids);
                    infeasible_route.setTotalCost(1e9);
                    infeasible_route.setFeasible(false);
                    env.end();
                    return infeasible_route;
                }
                model.add(depart[i] == t[i] + node_ptr->getServiceTime());
            }
        }
        model.add(t[0] == 0);
        for (int k = 0; k < m; ++k) {
            int i_idx = k;
            int j_idx = k + 1;
            model.add(t[j_idx] >= depart[i_idx] + s[k]);
        }

        model.add(ya[0] == params.getInitialSoc());
        for (int i = 0; i < n; ++i) {
            model.add(ya[i] >= params.getMinSoc());
            model.add(yd[i] >= params.getMinSoc());
        }
        for (int i = 0; i < n; ++i) {
            int node = node_ids[i];
            if (!charge_options[node].empty()) {
                IloExpr charge_amount(env);
                for (size_t k = 0; k < charge_options[node].size(); ++k) {
                    charge_amount += charge_options[node][k].getRate() * phi[i] * w[i][k];
                }
                model.add(yd[i] == ya[i] + charge_amount);
                charge_amount.end();
            } else {
                model.add(yd[i] == ya[i]);
            }
        }
        for (int k = 0; k < m; ++k) {
            int i_idx = k;
            int j_idx = k + 1;
            if (!is_wireless_route[k]) {
                model.add(ya[j_idx] == yd[i_idx] - params.getEnergyConsumption() * d_ij[k]);
            } else {
                int l = -1;
                for (int ll = 0; ll < p; ++ll) {
                    if (wireless_k[ll] == k) {
                        l = ll;
                        break;
                    }
                }
                if (l == -1) {
                    Route infeasible_route(node_ids);
                    infeasible_route.setTotalCost(1e9);
                    infeasible_route.setFeasible(false);
                    env.end();
                    return infeasible_route;
                }
                model.add(ya[j_idx] == yd[i_idx] - params.getEnergyConsumption() * d_ij[k] + beta_ij[k] * w_s_z[l]);
            }
        }

        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            double M = max_s[k];
            model.add(w_s_z[l] <= s[k]);
            model.add(w_s_z[l] <= M * z[l]);
            model.add(w_s_z[l] >= s[k] - M * (1 - z[l]));
            model.add(w_s_z[l] >= 0);
        }

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

        cplex.setOut(env.getNullStream());
        cplex.setParam(IloCplex::TiLim, 30.0);
        cplex.setParam(IloCplex::EpGap, 0.01);
        if (!cplex.solve()) {
            Route infeasible_route(node_ids);
            infeasible_route.setTotalCost(1e9);
            infeasible_route.setFeasible(false);
            env.end();
            return infeasible_route;
        }

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
        env.end();
        return result_route;

    } catch (const IloException& e) {
        std::cerr << "solveSubproblem1: CPLEX Exception: " << e.getMessage() << "\n";
        Route infeasible_route(current_route.getNodeIds());
        infeasible_route.setTotalCost(1e9);
        infeasible_route.setFeasible(false);
        env.end();
        return infeasible_route;
    } catch (const std::exception& e) {
        std::cerr << "solveSubproblem1: Exception: " << e.what() << "\n";
        Route infeasible_route(current_route.getNodeIds());
        infeasible_route.setTotalCost(1e9);
        infeasible_route.setFeasible(false);
        env.end();
        return infeasible_route;
    }
}

Route VNSOptimizer::createInitialRoute() {
    std::vector<int> route_ids = {0};
    std::vector<int> unvisited = S_prime;

    while (!unvisited.empty()) {
        int best_customer = -1;
        int best_pos = -1;
        double min_cost = std::numeric_limits<double>::max();

        for (int customer : unvisited) {
            for (size_t pos = 1; pos < route_ids.size(); ++pos) {
                std::vector<int> temp_ids = route_ids;
                temp_ids.insert(temp_ids.begin() + pos, customer);
                Route temp_route(temp_ids);
                if (quickFeasibilityCheck(temp_route)) {
                    temp_route = evaluateRoute(temp_route);
                    double cost = temp_route.getTotalCost();
                    if (cost < min_cost) {
                        min_cost = cost;
                        best_customer = customer;
                        best_pos = pos;
                    }
                }
            }
        }

        if (best_customer == -1) {
            std::vector<int> stations;
            for (const auto& node : nodes) {
                if (node.getType() == NodeType::CHARGING_STATION) {
                    stations.push_back(node.getId());
                }
            }
            if (!stations.empty()) {
                std::uniform_int_distribution<size_t> dist(0, stations.size() - 1);
                int station = stations[dist(rng_)];
                std::uniform_int_distribution<size_t> pos_dist(1, route_ids.size() - 1);
                route_ids.insert(route_ids.begin() + pos_dist(rng_), station);
            }
        } else {
            route_ids.insert(route_ids.begin() + best_pos, best_customer);
            unvisited.erase(std::remove(unvisited.begin(), unvisited.end(), best_customer), unvisited.end());
        }
    }

    route_ids.push_back(0);
    Route initial_route(route_ids);
    return quickFeasibilityCheck(initial_route) ? initial_route : Route(route_ids);
}

bool VNSOptimizer::isValidRoute(const std::vector<int>& route) const {
    if (route.front() != 0 || route.back() != 0) return false;

    std::set<int> required_customers(S_prime.begin(), S_prime.end());
    std::set<int> visited_customers;
    for (size_t i = 1; i < route.size() - 1; ++i) {
        if (route[i] == 0) return false;
        const Node* node = graph_.findNode(route[i]);
        if (node && node->getType() == NodeType::CUSTOMER) {
            if (visited_customers.count(route[i])) return false;
            visited_customers.insert(route[i]);
        }
    }
    return visited_customers == required_customers;
}

int VNSOptimizer::selectOperatorWeighted() {
    std::discrete_distribution<int> dist(operator_weights_.begin(), operator_weights_.end());
    return dist(rng_);
}

void VNSOptimizer::normalizeWeights() {
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

bool VNSOptimizer::checkSignificantImprovement(double old_cost, double new_cost) const {
    if (old_cost <= 0) return false;
    return (old_cost - new_cost) / old_cost >= 0.0001;
}

bool VNSOptimizer::quickFeasibilityCheck(Route& route) const {
    std::vector<int> nodes_ids = route.getNodeIds();
    if (nodes_ids.size() < 2) return false;

    double soc = params.getInitialSoc();
    for (size_t i = 0; i < nodes_ids.size() - 1; ++i) {
        const Arc* arc = graph_.findArc(nodes_ids[i], nodes_ids[i + 1]);
        if (!arc) return false;

        double energy_consumed = params.getEnergyConsumption() * arc->getDistance();
        soc -= energy_consumed;
        if (arc->getIsWireless()) {
            double wireless_charge = arc->getWirelessChargeRate() * arc->getTravelTime();
            soc = std::min(soc + wireless_charge, params.getBatteryCapacity());
        }

        if (soc < params.getMinSoc()) {
            std::vector<int> stations;
            for (const auto& node : nodes) {
                if (node.getType() == NodeType::CHARGING_STATION) {
                    stations.push_back(node.getId());
                }
            }
            if (!stations.empty()) {
                std::uniform_int_distribution<size_t> dist(0, stations.size() - 1);
                int station = stations[dist(rng_)];
                nodes_ids.insert(nodes_ids.begin() + i + 1, station);
                route = Route(nodes_ids);
                soc = params.getBatteryCapacity();
            } else {
                return false;
            }
        }
    }
    return soc >= params.getMinSoc();
}

Route VNSOptimizer::localSearchPhase(const Route& candidate_route) {
    if (!isValidRoute(candidate_route.getNodeIds()) || !quickFeasibilityCheck(const_cast<Route&>(candidate_route))) {
        Route infeasible_route(candidate_route.getNodeIds());
        infeasible_route.setTotalCost(1e9);
        infeasible_route.setFeasible(false);
        return infeasible_route;
    }
    return solveSubproblem1(candidate_route, graph_, charge_options, params);
}

Route VNSOptimizer::evaluateRoute(Route& route) {
    if (!quickFeasibilityCheck(route)) {
        route.setTotalCost(1e9);
        route.setFeasible(false);
        return route;
    }
    return solveSubproblem1(route, graph_, charge_options, params);
}

Route VNSOptimizer::shake(const Route& current_route, int neighborhood) {
    switch (neighborhood) {
        case 0: return twoOpt(current_route);
        case 1: return relocate(current_route);
        case 2: return swapNodes(current_route);
        case 3: return stationAdditionRemoval(current_route);
        default: return current_route;
    }
}

Route VNSOptimizer::twoOpt(const Route& route) {
    std::vector<int> nodes_ids = route.getNodeIds();
    if (nodes_ids.size() <= 3) return route;
    std::uniform_int_distribution<size_t> dist_i(1, nodes_ids.size() - 2);
    size_t i = dist_i(rng_);
    std::uniform_int_distribution<size_t> dist_j(i + 1, nodes_ids.size() - 1);
    size_t j = dist_j(rng_);
    std::reverse(nodes_ids.begin() + i, nodes_ids.begin() + j);
    return Route(nodes_ids);
}

Route VNSOptimizer::relocate(const Route& route) {
    std::vector<int> nodes_ids = route.getNodeIds();
    if (nodes_ids.size() <= 3) return route;
    std::uniform_int_distribution<size_t> dist(1, nodes_ids.size() - 2);
    size_t i = dist(rng_);
    size_t j = dist(rng_);
    if (i == j) return route;
    int node = nodes_ids[i];
    nodes_ids.erase(nodes_ids.begin() + i);
    nodes_ids.insert(nodes_ids.begin() + j, node);
    return Route(nodes_ids);
}

Route VNSOptimizer::swapNodes(const Route& route) {
    std::vector<int> nodes_ids = route.getNodeIds();
    if (nodes_ids.size() <= 3) return route;
    std::uniform_int_distribution<size_t> dist(1, nodes_ids.size() - 2);
    size_t i = dist(rng_);
    size_t j = dist(rng_);
    if (i != j) std::swap(nodes_ids[i], nodes_ids[j]);
    return Route(nodes_ids);
}

Route VNSOptimizer::stationAdditionRemoval(const Route& route) {
    std::vector<int> nodes_ids = route.getNodeIds();
    bool has_station = false;
    for (int id : nodes_ids) {
        const Node* node = graph_.findNode(id);
        if (node && node->getType() == NodeType::CHARGING_STATION) {
            has_station = true;
            break;
        }
    }

    std::uniform_int_distribution<int> coin(0, 1);
    bool add_station = (!has_station || coin(rng_) == 0);
    if (add_station) {
        std::vector<int> stations;
        for (const auto& node : nodes) {
            if (node.getType() == NodeType::CHARGING_STATION) {
                stations.push_back(node.getId());
            }
        }
        if (!stations.empty()) {
            std::uniform_int_distribution<size_t> dist_pos(1, nodes_ids.size() - 1);
            std::uniform_int_distribution<size_t> dist_station(0, stations.size() - 1);
            size_t pos = dist_pos(rng_);
            int station = stations[dist_station(rng_)];
            nodes_ids.insert(nodes_ids.begin() + pos, station);
        }
    } else {
        std::vector<size_t> station_indices;
        for (size_t i = 1; i < nodes_ids.size() - 1; ++i) {
            const Node* node = graph_.findNode(nodes_ids[i]);
            if (node && node->getType() == NodeType::CHARGING_STATION) {
                station_indices.push_back(i);
            }
        }
        if (!station_indices.empty()) {
            std::uniform_int_distribution<size_t> dist(0, station_indices.size() - 1);
            size_t idx = station_indices[dist(rng_)];
            nodes_ids.erase(nodes_ids.begin() + idx);
        }
    }
    return Route(nodes_ids);
}

void VNSOptimizer::updateRoute(Route& route, const SubproblemResult& result) {
    route.setSocArrival(result.soc_arrival);
    route.setSocDeparture(result.soc_departure);
    route.setArrivalTime(result.arrival_time);
    route.setDepartureTime(result.departure_time);
    route.setWirelessDecisions(result.wireless_decisions);
    route.setChargingDecisions(result.charging_decisions);
    route.setTotalCost(result.cost);
    route.setFeasible(true);
}

Route VNSOptimizer::run(int max_iterations) {
    best_route_ = evaluateRoute(best_route_);
    Route current_route = best_route_;
    double current_cost = best_route_.getTotalCost();
    double best_cost = current_cost;

    for (int iter = 0; iter < max_iterations; ++iter) {
        int k = selectOperatorWeighted();
        Route s_prime = shake(current_route, k);

        if (quickFeasibilityCheck(s_prime)) {
            s_prime = localSearchPhase(s_prime);
        } else {
            s_prime.setTotalCost(1e9);
            s_prime.setFeasible(false);
        }

        double s_prime_cost = s_prime.getTotalCost();

        if (s_prime_cost < best_cost) {
            best_route_ = s_prime;
            current_route = s_prime;
            best_cost = s_prime_cost;
            current_cost = s_prime_cost;
            operator_weights_[k] *= 1.1;
            normalizeWeights();
            no_improvement_counter_ = 0;
        } else if (s_prime_cost < current_cost) {
            current_route = s_prime;
            current_cost = s_prime_cost;
            no_improvement_counter_++;
        } else {
            no_improvement_counter_++;
        }

        if ((iter + 1) % 10 == 0) {
            std::cout << "Iteration " << iter + 1 << ":\n";
            std::cout << "Best cost: " << std::fixed << std::setprecision(2) << best_cost << "\n";
            std::cout << "No-Improvement Counter: " << no_improvement_counter_ << "\n";
            std::cout << "Best route: [";
            const auto& nodes_ids = best_route_.getNodeIds();
            for (size_t i = 0; i < nodes_ids.size(); ++i) {
                std::cout << nodes_ids[i];
                if (i < nodes_ids.size() - 1) std::cout << "->";
            }
            std::cout << "]\n";
            std::cout << "------------------------\n";
        }
    }

    return best_route_;
}