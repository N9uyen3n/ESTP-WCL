#include "../include/VNS1.h"
#include "../include/Utils.h"
#include <algorithm>
#include <numeric>
#include <limits>
#include <random>
#include <set>
#include <chrono>

VNS1::VNS1(const std::vector<int>& initial_route,
           const Graph& g,
           const std::vector<std::vector<ChargingOption>>& c_options,
           const Parameters& p,
           int max_iter,
           std::mt19937& r)
    : env(), init_route(initial_route), current_route(initial_route), graph(g), charge_options(c_options),
      params(p), max_iterations(max_iter), rng(r), operator_weights_{0.4, 0.3, 0.2},
      no_improvement_counter(0) {
}

VNS1::~VNS1() {
    env.end();
}

ILOSTLBEGIN
Route VNS1::run() {
    Route S0 = init_route;
    if (!quickFeasibilityCheck(S0)) {
        S0 = localSearch(S0);
    }

    Route S = S0;
    double C_S = S.getTotalCost();
    Route Sbest = S0;
    double C_Sbest = S.getTotalCost();

    int noImprovement = 0;
    const int max_no_improvement = 50;

    for (int iter = 0; iter < max_iterations && noImprovement < max_no_improvement; ++iter) {
        int op = selectOperatorWeighted();
        Route S_ = shake(S, op);
        double C_S_;
        if (quickFeasibilityCheck(S_)) {
            S_ = localSearch(S_);
            C_S_ = S_.getTotalCost();
        } else {
            C_S_ = std::numeric_limits<double>::infinity();
        }
        if (C_S_ < C_Sbest) {
            Sbest = S_;
            S = S_;
            C_S = C_S_;
            C_Sbest = C_S_;
            operator_weights_[op] += 0.1;
            normalizeWeights();
            if ((C_Sbest - C_S_) / C_Sbest >= 0.0001) {
                noImprovement = 0;
            } else {
                noImprovement++;
            }
        } else if (C_S_ < C_S) {
            S = S_;
            noImprovement++;
        } else {
            noImprovement++;
        }

        if ((iter + 1) % 10 == 0) {
            std::cout << "Iteration " << iter + 1 << ":\n";
            std::cout << "Best cost: " << std::fixed << std::setprecision(2) << C_Sbest << "\n";
            std::cout << "No-Improvement Counter: " << noImprovement << "\n";
            std::cout << "Best route: [";
            const auto& nodes_ids = Sbest.getNodeIds();
            for (size_t i = 0; i < nodes_ids.size(); ++i) {
                std::cout << nodes_ids[i];
                if (i < nodes_ids.size() - 1) std::cout << "->";
            }
            std::cout << "]\n";
            std::cout << "Charging decisions: " << Sbest.getChargingDecisions().size() << "\n";
            for (const auto& decision : Sbest.getChargingDecisions()) {
                std::cout << "Station " << decision.station_id << ": Option " << decision.option_index
                          << ", Duration " << decision.charging_time << "\n";
            }
            if (!Sbest.getArcWirelessDecisions().empty()) {
                std::cout << "Wireless decisions: " << Sbest.getArcWirelessDecisions().size() << "\n";
                for (const auto& arc_wireless : Sbest.getArcWirelessDecisions()) {
                    std::cout << "Wireless charging between nodes " << arc_wireless.first << " and "
                              << arc_wireless.second << "\n";
                }
            } else {
                std::cout << "No use wireless arcs\n";
            }
            std::cout << "------------------------------------------------------------------------\n";
        }
    }
    return Sbest;
}

Route VNS1::solveSubproblem(const std::vector<int>& initial_nodes) {
    auto start_time = std::chrono::high_resolution_clock::now();
    IloModel model(env);
    IloCplex cplex(model);

    try {
        if (!Utils::validateInitialNodes(initial_nodes, graph)) {
            std::cout << "MILP::optimize: Invalid initial nodes\n";
            throw std::runtime_error("Invalid initial node list");
        }

        const auto& nodes = graph.getNodes();
        const auto& arcs = graph.getArcs();
        std::vector<int> node_ids;
        std::vector<int> customer_ids;
        std::vector<int> station_ids;
        int depot_start_id = 0;
        int depot_end_id = -1;
        for (const auto& node : nodes) {
            node_ids.push_back(node.getId());
            if (node.getType() == NodeType::CUSTOMER) {
                customer_ids.push_back(node.getId());
            } else if (node.getType() == NodeType::CHARGING_STATION) {
                station_ids.push_back(node.getId());
            } else if (node.getType() == NodeType::DEPOT && node.getId() != depot_start_id) {
                depot_end_id = node.getId();
            }
        }
        if (depot_end_id == -1) {
            throw std::runtime_error("Depot end not found");
        }
        std::vector<std::pair<int, int>> wireless_arcs;
        for (const auto& arc : arcs) {
            if (arc.getIsWireless()) {
                wireless_arcs.emplace_back(arc.getFrom(), arc.getTo());
            }
        }

        std::set<std::pair<int, int>> allowed_arcs;
        allowed_arcs.insert({depot_start_id, initial_nodes[0]});
        for (size_t i = 0; i < initial_nodes.size() - 1; ++i) {
            allowed_arcs.insert({initial_nodes[i], initial_nodes[i + 1]});
            for (int station : station_ids) {
                allowed_arcs.insert({initial_nodes[i], station});
                allowed_arcs.insert({station, initial_nodes[i + 1]});
            }
        }
        allowed_arcs.insert({initial_nodes.back(), depot_end_id});
        for (int s1 : station_ids) {
            for (int s2 : station_ids) {
                if (s1 != s2 && graph.findArc(s1, s2)) {
                    allowed_arcs.insert({s1, s2});
                }
            }
        }

        IloArray<IloBoolVarArray> x(env, node_ids.size());
        for (size_t i = 0; i < node_ids.size(); ++i) {
            x[i] = IloBoolVarArray(env, node_ids.size());
            for (size_t j = 0; j < node_ids.size(); ++j) {
                x[i][j] = IloBoolVar(env, ("x_" + std::to_string(node_ids[i]) + "_" + std::to_string(node_ids[j])).c_str());
            }
        }
        IloArray<IloBoolVarArray> z(env, node_ids.size());
        for (size_t i = 0; i < node_ids.size(); ++i) {
            z[i] = IloBoolVarArray(env, node_ids.size());
            for (size_t j = 0; j < node_ids.size(); ++j) {
                z[i][j] = IloBoolVar(env, ("z_" + std::to_string(node_ids[i]) + "_" + std::to_string(node_ids[j])).c_str());
            }
        }

        for (size_t i = 0; i < node_ids.size(); ++i) {
            for (size_t j = 0; j < node_ids.size(); ++j) {
                if (i != j && !allowed_arcs.count({node_ids[i], node_ids[j]}) && graph.findArc(node_ids[i], node_ids[j])) {
                    x[i][j].setBounds(0, 0);
                    z[i][j].setBounds(0, 0);
                }
            }
        }
        IloNumVarArray y_a(env, node_ids.size(), 0, params.getBatteryCapacity(), ILOFLOAT);
        IloNumVarArray y_d(env, node_ids.size(), 0, params.getBatteryCapacity(), ILOFLOAT);
        IloNumVarArray t(env, node_ids.size(), 0, IloInfinity, ILOFLOAT);
        IloNumVarArray phi(env, node_ids.size(), 0, IloInfinity, ILOFLOAT);
        IloArray<IloBoolVarArray> w(env, node_ids.size());
        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_id = node_ids[i];
            w[i] = IloBoolVarArray(env, charge_options[node_id].size());
            for (size_t k = 0; k < charge_options[node_id].size(); ++k) {
                w[i][k] = IloBoolVar(env, ("w_" + std::to_string(node_id) + "_" + std::to_string(k)).c_str());
            }
        }
        IloArray<IloNumVarArray> s(env, node_ids.size());
        for (size_t i = 0; i < node_ids.size(); ++i) {
            s[i] = IloNumVarArray(env, node_ids.size(), 0, IloInfinity, ILOFLOAT);
        }
        IloNumVarArray u(env, node_ids.size(), 0, IloInfinity, ILOFLOAT);

        IloArray<IloNumVarArray> phi_w(env, node_ids.size());
        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            phi_w[i] = IloNumVarArray(env, charge_options[node_i].size(), 0, IloInfinity, ILOFLOAT);
        }
        IloArray<IloNumVarArray> s_z(env, node_ids.size());
        for (size_t i = 0; i < node_ids.size(); ++i) {
            s_z[i] = IloNumVarArray(env, node_ids.size(), 0, IloInfinity, ILOFLOAT);
        }

        IloExpr obj(env);
        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end()) {
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    obj += charge_options[node_i][k].getCost() * charge_options[node_i][k].getRate() * phi_w[i][k];
                }
            }
        }
        for (const auto& arc_pair : wireless_arcs) {
            int from = arc_pair.first;
            int to = arc_pair.second;
            size_t i_idx = std::find(node_ids.begin(), node_ids.end(), from) - node_ids.begin();
            size_t j_idx = std::find(node_ids.begin(), node_ids.end(), to) - node_ids.begin();
            const Arc* arc = graph.findArc(from, to);
            obj += params.getWirelessCost() * arc->getWirelessChargeRate() * s_z[i_idx][j_idx];
        }
        obj += params.getTimeCost() * t[node_ids.size() - 1];
        model.add(IloMinimize(env, obj));
        obj.end();

        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end()) {
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    double M = params.getBigM();
                    model.add(phi_w[i][k] <= phi[i]);
                    model.add(phi_w[i][k] <= M * w[i][k]);
                    model.add(phi_w[i][k] >= phi[i] - M * (1 - w[i][k]));
                    model.add(phi_w[i][k] >= 0);
                }
            }
        }

        for (size_t i = 0; i < node_ids.size(); ++i) {
            for (size_t j = 0; j < node_ids.size(); ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    const Arc* arc = graph.findArc(node_ids[i], node_ids[j]);
                    if (arc->getIsWireless()) {
                        double M = arc->getDistance() / params.getUmin();
                        model.add(s_z[i][j] <= s[i][j]);
                        model.add(s_z[i][j] <= M * z[i][j]);
                        model.add(s_z[i][j] >= s[i][j] - M * (1 - z[i][j]));
                        model.add(s_z[i][j] >= 0);
                    } else {
                        model.add(s_z[i][j] == 0);
                    }
                }
            }
        }

        IloExpr depot_start_flow(env);
        for (size_t j = 0; j < node_ids.size(); ++j) {
            if (node_ids[j] != depot_start_id && graph.findArc(depot_start_id, node_ids[j])) {
                depot_start_flow += x[0][j];
            }
        }
        model.add(depot_start_flow == 1);
        depot_start_flow.end();

        IloExpr depot_end_flow(env);
        for (size_t i = 0; i < node_ids.size(); ++i) {
            if (node_ids[i] != depot_end_id && graph.findArc(node_ids[i], depot_end_id)) {
                depot_end_flow += x[i][std::find(node_ids.begin(), node_ids.end(), depot_end_id) - node_ids.begin()];
            }
        }
        model.add(depot_end_flow == 1);
        depot_end_flow.end();

        for (int cust_id : customer_ids) {
            size_t i_idx = std::find(node_ids.begin(), node_ids.end(), cust_id) - node_ids.begin();
            IloExpr in_flow(env);
            IloExpr out_flow(env);
            for (size_t j = 0; j < node_ids.size(); ++j) {
                if (graph.findArc(node_ids[j], cust_id)) {
                    in_flow += x[j][i_idx];
                }
                if (graph.findArc(cust_id, node_ids[j])) {
                    out_flow += x[i_idx][j];
                }
            }
            model.add(in_flow == 1);
            model.add(out_flow == 1);
            in_flow.end();
            out_flow.end();
        }

        for (int stat_id : station_ids) {
            size_t i_idx = std::find(node_ids.begin(), node_ids.end(), stat_id) - node_ids.begin();
            IloExpr in_flow(env);
            IloExpr out_flow(env);
            for (size_t j = 0; j < node_ids.size(); ++j) {
                if (graph.findArc(node_ids[j], stat_id)) {
                    in_flow += x[j][i_idx];
                }
                if (graph.findArc(stat_id, node_ids[j])) {
                    out_flow += x[i_idx][j];
                }
            }
            model.add(in_flow == out_flow);
            model.add(in_flow <= 1);
            in_flow.end();
            out_flow.end();
        }

        for (size_t i = 1; i < node_ids.size(); ++i) {
            for (size_t j = 1; j < node_ids.size(); ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    IloExpr expr(env);
                    expr = u[i] - u[j] + IloNum(node_ids.size()) * x[i][j];
                    model.add(expr <= IloNum(node_ids.size()) - 1);
                    expr.end();
                }
            }
        }
        model.add(u[0] == 0);

        model.add(y_a[0] == params.getInitialSoc());
        model.add(y_d[0] == y_a[0]);
        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end()) {
                IloExpr charge_amount(env);
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    charge_amount += charge_options[node_i][k].getRate() * phi_w[i][k];
                }
                model.add(y_d[i] == y_a[i] + charge_amount);
                charge_amount.end();
            } else {
                model.add(y_d[i] == y_a[i]);
            }
        }
        for (size_t i = 0; i < node_ids.size(); ++i) {
            for (size_t j = 0; j < node_ids.size(); ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    const Arc* arc = graph.findArc(node_ids[i], node_ids[j]);
                    double dist = arc->getDistance();
                    if (arc->getIsWireless()) {
                        model.add(y_a[j] <= y_d[i] - params.getEnergyConsumption() * dist +
                                  arc->getWirelessChargeRate() * s_z[i][j] +
                                  params.getBatteryCapacity() * (1 - x[i][j]));
                    } else {
                        model.add(y_a[j] <= y_d[i] - params.getEnergyConsumption() * dist +
                                  params.getBatteryCapacity() * (1 - x[i][j]));
                    }
                    model.add(y_a[j] >= params.getMinSoc());
                }
            }
        }

        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            const Node* node = graph.findNode(node_i);
            double tau_i = node ? node->getServiceTime() : 0.0;
            for (size_t j = 0; j < node_ids.size(); ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    const Arc* arc = graph.findArc(node_ids[i], node_ids[j]);
                    double dist = arc->getDistance();
                    model.add(s[i][j] >= (dist / params.getUmax()) * x[i][j]);
                    model.add(s[i][j] <= (dist / params.getUmin()) * x[i][j] + params.getBigM() * (1 - x[i][j]));
                    if (std::find(customer_ids.begin(), customer_ids.end(), node_i) != customer_ids.end()) {
                        model.add(t[j] >= t[i] + tau_i + s[i][j] - params.getBigM() * (1 - x[i][j]));
                    } else if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end()) {
                        model.add(t[j] >= t[i] + phi[i] + s[i][j] - params.getBigM() * (1 - x[i][j]));
                    } else {
                        model.add(t[j] >= t[i] + s[i][j] - params.getBigM() * (1 - x[i][j]));
                    }
                }
            }
        }

        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end()) {
                IloExpr sum_w(env);
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    sum_w += w[i][k];
                }
                IloExpr in_flow(env);
                for (size_t j = 0; j < node_ids.size(); ++j) {
                    if (graph.findArc(node_ids[j], node_i)) {
                        in_flow += x[j][i];
                    }
                }
                model.add(sum_w <= in_flow);
                model.add(sum_w <= 1);
                sum_w.end();
                in_flow.end();
            } else {
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    model.add(w[i][k] == 0);
                }
                model.add(phi[i] == 0);
            }
        }
        for (size_t i = 0; i < node_ids.size(); ++i) {
            for (size_t j = 0; j < node_ids.size(); ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    const Arc* arc = graph.findArc(node_ids[i], node_ids[j]);
                    if (arc->getIsWireless()) {
                        model.add(z[i][j] <= x[i][j]);
                    } else {
                        model.add(z[i][j] == 0);
                    }
                }
            }
        }

        cplex.setOut(env.getNullStream());
        cplex.setParam(IloCplex::TiLim, 20);
        if (!cplex.solve()) {
            std::cout << "MILP::optimize: CPLEX failed, status = " << cplex.getStatus() << "\n";
            return Route(initial_nodes);
        }

        ProblemInfos result;
        result.cost = cplex.getObjValue();
        result.new_node_ids.clear();
        result.soc_arrival.resize(node_ids.size());
        result.soc_departure.resize(node_ids.size());
        result.arrival_time.resize(node_ids.size());
        result.departure_time.resize(node_ids.size());
        result.wireless_decisions.clear();
        result.charging_decisions.clear();
        result.arc_wireless_decisions.clear();

        int current = depot_start_id;
        result.new_node_ids.push_back(current);
        std::set<int> visited;
        visited.insert(current);
        while (current != depot_end_id) {
            bool found = false;
            for (size_t j = 0; j < node_ids.size(); ++j) {
                size_t i_idx = std::find(node_ids.begin(), node_ids.end(), current) - node_ids.begin();
                if (node_ids[j] != current && graph.findArc(current, node_ids[j]) &&
                    cplex.getValue(x[i_idx][j]) > 0.5 && visited.find(node_ids[j]) == visited.end()) {
                    result.new_node_ids.push_back(node_ids[j]);
                    result.wireless_decisions.push_back(cplex.getValue(z[i_idx][j]) > 0.5);
                    if (cplex.getValue(z[i_idx][j]) > 0.5) {
                        result.arc_wireless_decisions.emplace_back(current, node_ids[j]);
                    }
                    current = node_ids[j];
                    visited.insert(current);
                    found = true;
                    break;
                }
            }
            if (!found) {
                std::cout << "MILP::optimize: Failed to construct route\n";
                return Route(initial_nodes);
            }
        }

        for (size_t i = 0; i < result.new_node_ids.size(); ++i) {
            size_t idx = std::find(node_ids.begin(), node_ids.end(), result.new_node_ids[i]) - node_ids.begin();
            result.soc_arrival[i] = cplex.getValue(y_a[idx]);
            result.soc_departure[i] = cplex.getValue(y_d[idx]);
            result.arrival_time[i] = cplex.getValue(t[idx]);
            double tau_i = graph.findNode(result.new_node_ids[i])->getServiceTime();
            double phi_i = cplex.getValue(phi[idx]);
            result.departure_time[i] = result.arrival_time[i] + phi_i + tau_i;
            int node_i = result.new_node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end()) {
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    if (cplex.getValue(w[idx][k]) > 0.5) {
                        result.charging_decisions.emplace_back(node_i, static_cast<int>(k + 1), phi_i);
                        break;
                    }
                }
            }
        }

        result.feasible = true;
        Route result_route(result.new_node_ids);
        updateRoute(result_route, result);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
        std::cout << "MILP::optimize: Success, cost = " << result.cost << ", Time = " << duration << " s\n";
        return result_route;
    } catch (IloException& e) {
        std::cout << "MILP::optimize: CPLEX Exception: " << e.getMessage() << "\n";
        return Route(initial_nodes);
    }
}

bool VNS1::isValidRoute(const std::vector<int>& route) {
    if (route.empty() || route.front() != 0 || route.back() != static_cast<int>(graph.getNodes().size()) - 1) {
        return false;
    }
    std::set<int> required_nodes(init_route.begin(), init_route.end());
    required_nodes.erase(0);
    required_nodes.erase(graph.getNodes().size() - 1);
    std::set<int> visited;
    for (int node : route) {
        if (node != 0 && node != static_cast<int>(graph.getNodes().size()) - 1) {
            visited.insert(node);
        }
    }
    for (int node : required_nodes) {
        if (visited.find(node) == visited.end()) {
            return false;
        }
    }
    for (size_t i = 0; i < route.size() - 1; ++i) {
        if (!graph.findArc(route[i], route[i + 1])) {
            return false;
        }
    }
    return true;
}

bool VNS1::quickFeasibilityCheck(const Route& route) {
    double current_soc = params.getInitialSoc();
    const auto& nodes = route.getNodeIds();
    const double min_soc = params.getMinSoc();
    const double battery_capacity = params.getBatteryCapacity();
    const double energy_consumption = params.getEnergyConsumption();
    const double threshold = -0.1 * battery_capacity;

    for (size_t i = 0; i < nodes.size() - 1; ++i) {
        const Arc* arc = graph.findArc(nodes[i], nodes[i + 1]);
        if (!arc) return false;
        double distance = arc->getDistance();
        double energy_used = distance * energy_consumption;

        if (arc->getIsWireless()) {
            double travel_time = distance / params.getVehicleSpeed();
            double charge_rate = arc->getWirelessChargeRate();
            current_soc += charge_rate * travel_time;
            current_soc = std::min(current_soc, battery_capacity);
        }
        current_soc -= energy_used;

        if (current_soc < threshold) return false;

        if (graph.findNode(nodes[i + 1])->getType() == NodeType::CHARGING_STATION) {
            current_soc = battery_capacity;
        }
    }
    return current_soc >= min_soc;
}

void VNS1::normalizeWeights() {
    double sum = std::accumulate(operator_weights_.begin(), operator_weights_.end(), 0.0);
    for (auto& weight : operator_weights_) {
        weight /= sum;
    }
}

int VNS1::selectOperatorWeighted() {
    std::discrete_distribution<> dist(operator_weights_.begin(), operator_weights_.end());
    return dist(rng);
}

void VNS1::updateRoute(Route& route, const ProblemInfos& result) {
    route.setNodes(result.new_node_ids);
    route.setSocArrival(result.soc_arrival);
    route.setSocDeparture(result.soc_departure);
    route.setArrivalTime(result.arrival_time);
    route.setDepartureTime(result.departure_time);
    route.setChargingDecisions(result.charging_decisions);
    route.setWirelessDecisions(result.wireless_decisions);
    route.setTotalCost(result.cost);
    route.setFeasible(result.feasible);
    route.setArcWirelessDecisions(result.arc_wireless_decisions);
}

Route VNS1::shake(const Route& route, int neighborhood) {
    switch (neighborhood) {
        case 0: return twoOpt(route);
        case 1: return swapNodes(route);
        case 2: return relocate(route);
        default: return route;
    }
}

std::vector<int> VNS1::getCustomerIndices(const Route& route) {
    std::vector<int> customer_indices;
    const auto& nodes = route.getNodeIds();
    for (size_t i = 1; i < nodes.size() - 1; ++i) {
        if (graph.findNode(nodes[i])->getType() == NodeType::CUSTOMER) {
            customer_indices.push_back(i);
        }
    }
    return customer_indices;
}

Route VNS1::twoOpt(const Route& route) {
    Route new_route = route;
    auto nodes = new_route.getNodeIds();
    auto customer_indices = getCustomerIndices(route);
    if (customer_indices.size() < 2) return new_route;

    std::vector<std::pair<int, int>> customer_segments;
    int start = customer_indices[0];
    int end = start;
    for (size_t i = 1; i < customer_indices.size(); ++i) {
        if (customer_indices[i] == customer_indices[i - 1] + 1) {
            end = customer_indices[i];
        } else {
            customer_segments.push_back({start, end});
            start = customer_indices[i];
            end = start;
        }
    }
    customer_segments.push_back({start, end});

    if (customer_segments.empty()) return new_route;

    std::uniform_int_distribution<> seg_dist(0, customer_segments.size() - 1);
    auto seg = customer_segments[seg_dist(rng)];
    int seg_start = seg.first;
    int seg_end = seg.second;

    if (seg_end - seg_start < 1) return new_route;

    std::uniform_int_distribution<> dist(seg_start, seg_end - 1);
    int i = dist(rng);
    std::uniform_int_distribution<> dist2(i + 1, seg_end);
    int j = dist2(rng);

    std::reverse(nodes.begin() + i, nodes.begin() + j + 1);
    new_route.setNodes(nodes);
    return new_route;
}

Route VNS1::swapNodes(const Route& route) {
    Route new_route = route;
    auto nodes = new_route.getNodeIds();
    auto customer_indices = getCustomerIndices(route);
    if (customer_indices.size() < 2) return new_route;

    std::uniform_int_distribution<> dist(0, customer_indices.size() - 1);
    int idx1 = dist(rng);
    int idx2 = dist(rng);
    while (idx1 == idx2) idx2 = dist(rng);

    int i = customer_indices[idx1];
    int j = customer_indices[idx2];
    std::swap(nodes[i], nodes[j]);
    new_route.setNodes(nodes);
    return new_route;
}

Route VNS1::relocate(const Route& route) {
    Route new_route = route;
    auto nodes = new_route.getNodeIds();
    auto customer_indices = getCustomerIndices(route);
    if (customer_indices.size() < 2) return new_route;

    std::uniform_int_distribution<> dist(0, customer_indices.size() - 1);
    int idx = dist(rng);
    int i = customer_indices[idx];
    int node = nodes[i];

    std::vector<int> possible_positions;
    for (size_t pos = 1; pos < nodes.size() - 1; ++pos) {
        if (pos != static_cast<size_t>(i) &&
            (graph.findNode(nodes[pos - 1])->getType() == NodeType::CUSTOMER ||
             graph.findNode(nodes[pos])->getType() == NodeType::CUSTOMER)) {
            possible_positions.push_back(pos);
        }
    }
    if (possible_positions.empty()) return new_route;

    std::uniform_int_distribution<> pos_dist(0, possible_positions.size() - 1);
    int new_pos = possible_positions[pos_dist(rng)];

    nodes.erase(nodes.begin() + i);
    if (new_pos > i) new_pos--;
    nodes.insert(nodes.begin() + new_pos, node);
    new_route.setNodes(nodes);
    return new_route;
}

Route VNS1::localSearch(const Route& current_route) {
    return solveSubproblem(current_route.getNodeIds());
}