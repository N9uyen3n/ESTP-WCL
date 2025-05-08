#include "../include/MILP.h"
#include "../include/Utils.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <set>

MILP::MILP() : env() {}

MILP::~MILP() {
    env.end();
}

Route MILP::optimize(const std::vector<int>& initial_nodes,
                     const Graph& graph,
                     const std::vector<std::vector<ChargingOption>>& charge_options,
                     const Parameters& params) {
    auto start_time = std::chrono::high_resolution_clock::now();
    IloModel model(env);
    IloCplex cplex(model);

    try {
        // Validate input
        if (!Utils::validateInitialNodes(initial_nodes, graph)) {
            std::cout << "MILP::optimize: Invalid initial nodes\n";
            throw std::runtime_error("Invalid initial node list");
        }

        // Sets
        const auto& nodes = graph.getNodes();
        const auto& arcs = graph.getArcs();
        std::vector<int> node_ids;
        std::vector<int> customer_ids;
        std::vector<int> station_ids;
        int depot_end_id = -1;
        for (const auto& node : nodes) {
            node_ids.push_back(node.getId());
            if (node.getType() == NodeType::CUSTOMER) {
                customer_ids.push_back(node.getId());
            } else if (node.getType() == NodeType::CHARGING_STATION) {
                station_ids.push_back(node.getId());
            } else if (node.getType() == NodeType::DEPOT && node.getId() != 0) {
                depot_end_id = node.getId();
            }
        }
        std::vector<std::pair<int, int>> wireless_arcs;
        for (const auto& arc : arcs) {
            if (arc.getIsWireless()) {
                wireless_arcs.emplace_back(arc.getFrom(), arc.getTo());
            }
        }

        // Decision Variables
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

        // Objective Function
        IloExpr obj(env);
        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end()) {
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    obj += charge_options[node_i][k].getCost() * charge_options[node_i][k].getRate() * phi[i] * w[i][k];
                }
            }
        }
        for (const auto& arc_pair : wireless_arcs) {
            int from = arc_pair.first;
            int to = arc_pair.second;
            size_t i_idx = std::find(node_ids.begin(), node_ids.end(), from) - node_ids.begin();
            size_t j_idx = std::find(node_ids.begin(), node_ids.end(), to) - node_ids.begin();
            const Arc* arc = graph.findArc(from, to);
            obj += params.getWirelessCost() * arc->getWirelessChargeRate() * s[i_idx][j_idx] * z[i_idx][j_idx];
        }
        obj += params.getTimeCost() * t[node_ids.size() - 1];
        model.add(IloMinimize(env, obj));
        obj.end();

        // Flow Constraints
        IloExpr depot_start_flow(env);
        for (size_t j = 0; j < node_ids.size(); ++j) {
            if (node_ids[j] != 0 && graph.findArc(0, node_ids[j])) {
                depot_start_flow += x[0][j];
            }
        }
        model.add(depot_start_flow == 1);
        depot_start_flow.end();

        IloExpr depot_end_flow(env);
        for (size_t i = 0; i < node_ids.size(); ++i) {
            if (node_ids[i] != depot_end_id && graph.findArc(node_ids[i], depot_end_id)) {
                depot_end_flow += x[i][node_ids.size() - 1];
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

        // SOC Constraints
        model.add(y_a[0] == params.getInitialSoc());
        model.add(y_d[0] == y_a[0]);
        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end()) {
                IloExpr charge_amount(env);
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    charge_amount += charge_options[node_i][k].getRate() * phi[i] * w[i][k];
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
                                  arc->getWirelessChargeRate() * s[i][j] * z[i][j] +
                                  params.getBatteryCapacity() * (1 - x[i][j]));
                    } else {
                        model.add(y_a[j] <= y_d[i] - params.getEnergyConsumption() * dist +
                                  params.getBatteryCapacity() * (1 - x[i][j]));
                    }
                }
            }
        }

        // Time Constraints
        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            const Node* node = graph.findNode(node_i);
            double tau_i = node ? node->getServiceTime() : 0.0;
            for (size_t j = 0; j < node_ids.size(); ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    const Arc* arc = graph.findArc(node_ids[i], node_ids[j]);
                    double dist = arc->getDistance();
                    double s_ij = arc->getTravelTime();
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

        // Charging Constraints
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
                model.add(sum_w == in_flow);
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

        // Solve
        cplex.setOut(std::cout);
        cplex.setParam(IloCplex::TiLim, 300.0); // 5-minute time limit
        cplex.setParam(IloCplex::EpGap, 0.01); // 1% MIP gap
        if (!cplex.solve()) {
            std::cout << "MILP::optimize: CPLEX failed, status = " << cplex.getStatus() << "\n";
            return Route(initial_nodes);
        }

        // Extract Route
        SubproblemResult result;
        result.cost = cplex.getObjValue();
        result.new_node_ids.clear();
        result.soc_arrival.resize(node_ids.size());
        result.soc_departure.resize(node_ids.size());
        result.arrival_time.resize(node_ids.size());
        result.departure_time.resize(node_ids.size());
        result.wireless_decisions.clear();
        result.charging_decisions.clear();

        int current = 0;
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
            result.departure_time[i] = cplex.getValue(t[idx]) + cplex.getValue(phi[idx]) + tau_i;
            int node_i = result.new_node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end()) {
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    if (cplex.getValue(w[idx][k]) > 0.5) {
                        result.charging_decisions.emplace_back(node_i, static_cast<int>(k), cplex.getValue(phi[idx]));
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

Route MILP::solveSubproblem(int i, int j, int a, const Route& current_route,
                           const Graph& graph,
                           const std::vector<std::vector<ChargingOption>>& charge_options,
                           const Parameters& params) {
    auto start_time = std::chrono::high_resolution_clock::now();
    IloModel model(env);
    IloCplex cplex(model);
    SubproblemResult result;

    try {
        // Validate arcs
        const Arc* arc_ij = graph.findArc(i, j);
        const Arc* arc_ia = graph.findArc(i, a);
        const Arc* arc_aj = graph.findArc(a, j);
        if (!arc_ij || !arc_ia || !arc_aj) {
            std::cout << "MILP::solveSubproblem: Invalid arc i=" << i << ", j=" << j << ", a=" << a << "\n";
            throw std::runtime_error("Invalid arc in subproblem");
        }

        // Decision Variables
        IloBoolVar x_ij(env, "x_ij");
        IloBoolVar x_ia(env, "x_ia");
        IloBoolVar x_aj(env, "x_aj");
        IloBoolVar z_ij(env, "z_ij");
        IloNumVar y_i_a(env, 0, params.getBatteryCapacity(), "y_i_a");
        IloNumVar y_i_d(env, 0, params.getBatteryCapacity(), "y_i_d");
        IloNumVar y_a_a(env, 0, params.getBatteryCapacity(), "y_a_a");
        IloNumVar y_a_d(env, 0, params.getBatteryCapacity(), "y_a_d");
        IloNumVar y_j_a(env, 0, params.getBatteryCapacity(), "y_j_a");
        IloNumVar y_j_d(env, 0, params.getBatteryCapacity(), "y_j_d");
        IloNumVar t_i(env, 0, IloInfinity, "t_i");
        IloNumVar t_a(env, 0, IloInfinity, "t_a");
        IloNumVar t_j(env, 0, IloInfinity, "t_j");
        IloNumVar s_ij(env, 0, IloInfinity, "s_ij");
        IloNumVar s_ia(env, 0, IloInfinity, "s_ia");
        IloNumVar s_aj(env, 0, IloInfinity, "s_aj");
        IloNumVar phi_a(env, 0, IloInfinity, "phi_a");
        IloBoolVarArray w_ak(env, charge_options[a].size());
        for (size_t k = 0; k < charge_options[a].size(); ++k) {
            w_ak[k] = IloBoolVar(env, ("w_a_" + std::to_string(k)).c_str());
        }

        // Objective Function
        IloExpr obj(env);
        for (size_t k = 0; k < charge_options[a].size(); ++k) {
            obj += charge_options[a][k].getCost() * charge_options[a][k].getRate() * phi_a * w_ak[k];
        }
        if (arc_ij->getIsWireless()) {
            obj += params.getWirelessCost() * arc_ij->getWirelessChargeRate() * s_ij * z_ij;
        }
        obj += params.getTimeCost() * (t_j - t_i);
        model.add(IloMinimize(env, obj));
        obj.end();

        // Flow Constraints
        model.add(x_ij + x_ia == 1);
        model.add(x_ij + x_aj == 1);
        model.add(x_ia == x_aj);

        // SOC Constraints
        auto node_ids = current_route.getNodeIds();
        size_t pos_i = std::find(node_ids.begin(), node_ids.end(), i) - node_ids.begin();
        model.add(y_i_a == current_route.getSocArrival()[pos_i]);
        model.add(y_i_d == y_i_a);
        if (arc_ij->getIsWireless()) {
            model.add(y_j_a <= y_i_d - params.getEnergyConsumption() * arc_ij->getDistance() +
                      arc_ij->getWirelessChargeRate() * s_ij * z_ij +
                      params.getBatteryCapacity() * (1 - x_ij));
        } else {
            model.add(y_j_a <= y_i_d - params.getEnergyConsumption() * arc_ij->getDistance() +
                      params.getBatteryCapacity() * (1 - x_ij));
        }
        model.add(y_a_a <= y_i_d - params.getEnergyConsumption() * arc_ia->getDistance() +
                  params.getBatteryCapacity() * (1 - x_ia));
        IloExpr charge_amount(env);
        for (size_t k = 0; k < charge_options[a].size(); ++k) {
            charge_amount += charge_options[a][k].getRate() * phi_a * w_ak[k];
        }
        model.add(y_a_d == y_a_a + charge_amount);
        charge_amount.end();
        model.add(y_j_a <= y_a_d - params.getEnergyConsumption() * arc_aj->getDistance() +
                  params.getBatteryCapacity() * (1 - x_aj));
        model.add(y_j_d == y_j_a);

        // Time Constraints
        model.add(t_i == current_route.getArrivalTime()[pos_i]);
        model.add(s_ij >= (arc_ij->getDistance() / params.getUmax()) * x_ij);
        model.add(s_ij <= (arc_ij->getDistance() / params.getUmin()) * x_ij + params.getBigM() * (1 - x_ij));
        model.add(s_ia >= (arc_ia->getDistance() / params.getUmax()) * x_ia);
        model.add(s_ia <= (arc_ia->getDistance() / params.getUmin()) * x_ia + params.getBigM() * (1 - x_ia));
        model.add(s_aj >= (arc_aj->getDistance() / params.getUmax()) * x_aj);
        model.add(s_aj <= (arc_aj->getDistance() / params.getUmin()) * x_aj + params.getBigM() * (1 - x_aj));
        model.add(t_j >= t_i + s_ij - params.getBigM() * (1 - x_ij));
        model.add(t_a >= t_i + s_ia - params.getBigM() * (1 - x_ia));
        model.add(t_j >= t_a + phi_a + s_aj - params.getBigM() * (1 - x_aj));

        // Charging Constraints
        IloExpr sum_w(env);
        for (size_t k = 0; k < charge_options[a].size(); ++k) {
            sum_w += w_ak[k];
        }
        model.add(sum_w == x_ia);
        sum_w.end();
        if (arc_ij->getIsWireless()) {
            model.add(z_ij <= x_ij);
        } else {
            model.add(z_ij == 0);
        }

        // Solve
        cplex.setOut(std::cout);
        cplex.setParam(IloCplex::TiLim, 5.0); // 5-second time limit
        cplex.setParam(IloCplex::EpGap, 0.01); // 1% MIP gap
        if (!cplex.solve()) {
            std::cout << "MILP::solveSubproblem: CPLEX failed, status = " << cplex.getStatus() << "\n";
            return current_route;
        }

        // Extract Results
        result.cost = cplex.getObjValue();
        result.new_node_ids = current_route.getNodeIds();
        result.soc_arrival = current_route.getSocArrival();
        result.soc_departure = current_route.getSocDeparture();
        result.arrival_time = current_route.getArrivalTime();
        result.departure_time = current_route.getDepartureTime();
        result.wireless_decisions = current_route.getWirelessDecisions();
        result.charging_decisions = current_route.getChargingDecisions();
        result.feasible = true;

        size_t pos_j = std::find(node_ids.begin(), node_ids.end(), j) - node_ids.begin();
        if (cplex.getValue(x_ia) > 0.5 && cplex.getValue(x_aj) > 0.5) {
            // Insert station a between i and j
            result.new_node_ids.insert(result.new_node_ids.begin() + pos_j, a);
            result.soc_arrival.insert(result.soc_arrival.begin() + pos_j, cplex.getValue(y_a_a));
            result.soc_departure.insert(result.soc_departure.begin() + pos_j, cplex.getValue(y_a_d));
            result.arrival_time.insert(result.arrival_time.begin() + pos_j, cplex.getValue(t_a));
            result.departure_time.insert(result.departure_time.begin() + pos_j, cplex.getValue(t_a) + cplex.getValue(phi_a));
            result.wireless_decisions.insert(result.wireless_decisions.begin() + pos_j - 1, false);
            result.wireless_decisions[pos_j - 1] = false; // No wireless charging on (i, a)
            result.soc_arrival[pos_j + 1] = cplex.getValue(y_j_a);
            result.soc_departure[pos_j + 1] = cplex.getValue(y_j_d);
            result.arrival_time[pos_j + 1] = cplex.getValue(t_j);
            result.departure_time[pos_j + 1] = cplex.getValue(t_j);
            for (size_t k = 0; k < charge_options[a].size(); ++k) {
                if (cplex.getValue(w_ak[k]) > 0.5) {
                    result.charging_decisions.emplace_back(a, static_cast<int>(k), cplex.getValue(phi_a));
                    break;
                }
            }
        } else {
            // Direct path i to j
            result.soc_arrival[pos_j] = cplex.getValue(y_j_a);
            result.soc_departure[pos_j] = cplex.getValue(y_j_d);
            result.arrival_time[pos_j] = cplex.getValue(t_j);
            result.departure_time[pos_j] = cplex.getValue(t_j);
            result.wireless_decisions[pos_j - 1] = (arc_ij->getIsWireless() && cplex.getValue(z_ij) > 0.5);
        }

        Route result_route(result.new_node_ids);
        updateRoute(result_route, result);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "MILP::solveSubproblem: Success, cost = " << result.cost << ", Time = " << duration << " ms\n";
        return result_route;
    } catch (IloException& e) {
        std::cout << "MILP::solveSubproblem: CPLEX Exception: " << e.getMessage() << "\n";
        return current_route;
    }
}

void MILP::updateRoute(Route& route, const SubproblemResult& result) {
    route.setNodes(result.new_node_ids);
    route.setSocArrival(result.soc_arrival);
    route.setSocDeparture(result.soc_departure);
    route.setArrivalTime(result.arrival_time);
    route.setDepartureTime(result.departure_time);
    route.setChargingDecisions(result.charging_decisions);
    route.setWirelessDecisions(result.wireless_decisions);
    route.setTotalCost(result.cost);
    route.setFeasible(result.feasible);
}