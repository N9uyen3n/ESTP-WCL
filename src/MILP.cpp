#include "../include/MILP.h"
#include "../include/Utils.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <chrono>

MILP::MILP() : env() {
}

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
        for (const auto& node : nodes) {
            node_ids.push_back(node.getId());
            if (node.getType() == NodeType::CUSTOMER || node.getId() == 0) {
                customer_ids.push_back(node.getId());
            }
            if (node.getType() == NodeType::CHARGING_STATION) {
                station_ids.push_back(node.getId());
            }
        }
        std::vector<std::pair<int, int>> arc_pairs;
        std::vector<int> wireless_arc_indices;
        for (size_t i = 0; i < arcs.size(); ++i) {
            arc_pairs.emplace_back(arcs[i].getFrom(), arcs[i].getTo());
            if (arcs[i].getIsWireless()) {
                wireless_arc_indices.push_back(i);
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
        IloIntVarArray u(env, node_ids.size(), 1, node_ids.size());
        IloNumVarArray phi(env, node_ids.size(), 0, IloInfinity, ILOFLOAT);
        IloArray<IloBoolVarArray> w(env, node_ids.size());
        for (size_t i = 0; i < node_ids.size(); ++i) {
            w[i] = IloBoolVarArray(env, charge_options[node_ids[i]].size());
            for (size_t k = 0; k < charge_options[node_ids[i]].size(); ++k) {
                w[i][k] = IloBoolVar(env, ("w_" + std::to_string(node_ids[i]) + "_" + std::to_string(k)).c_str());
            }
        }
        IloArray<IloNumVarArray> phi_w(env, node_ids.size());
        for (size_t i = 0; i < node_ids.size(); ++i) {
            phi_w[i] = IloNumVarArray(env, charge_options[node_ids[i]].size(), 0, IloInfinity, ILOFLOAT);
        }
        IloArray<IloNumVarArray> s(env, node_ids.size());
        IloArray<IloBoolVarArray> z(env, node_ids.size());
        IloArray<IloNumVarArray> s_z(env, node_ids.size());
        for (size_t i = 0; i < node_ids.size(); ++i) {
            s[i] = IloNumVarArray(env, node_ids.size(), 0, IloInfinity, ILOFLOAT);
            z[i] = IloBoolVarArray(env, node_ids.size());
            s_z[i] = IloNumVarArray(env, node_ids.size(), 0, IloInfinity, ILOFLOAT);
            for (size_t j = 0; j < node_ids.size(); ++j) {
                z[i][j] = IloBoolVar(env, ("z_" + std::to_string(node_ids[i]) + "_" + std::to_string(node_ids[j])).c_str());
            }
        }
        IloNumVarArray t(env, node_ids.size(), 0, IloInfinity, ILOFLOAT);
        IloNumVarArray depart(env, node_ids.size(), 0, IloInfinity, ILOFLOAT);
        IloNumVarArray y_a(env, node_ids.size(), params.getMinSoc(), params.getBatteryCapacity(), ILOFLOAT);
        IloNumVarArray y_d(env, node_ids.size(), params.getMinSoc(), params.getBatteryCapacity(), ILOFLOAT);

        // Objective
        IloExpr obj(env);
        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                obj += charge_options[node_i][k].getCost() * charge_options[node_i][k].getRate() * phi_w[i][k];
            }
        }
        for (size_t idx = 0; idx < wireless_arc_indices.size(); ++idx) {
            int arc_idx = wireless_arc_indices[idx];
            int i = arcs[arc_idx].getFrom();
            int j = arcs[arc_idx].getTo();
            size_t i_idx = std::find(node_ids.begin(), node_ids.end(), i) - node_ids.begin();
            size_t j_idx = std::find(node_ids.begin(), node_ids.end(), j) - node_ids.begin();
            obj += params.getWirelessCost() * arcs[arc_idx].getWirelessChargeRate() * s_z[i_idx][j_idx];
        }
        obj += params.getTimeCost() * t[node_ids.size() - 1];
        model.add(IloMinimize(env, obj));
        obj.end();

        // Routing Constraints
        for (size_t i = 0; i < node_ids.size(); ++i) {
            IloExpr out_flow(env);
            IloExpr in_flow(env);
            for (size_t j = 0; j < node_ids.size(); ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    out_flow += x[i][j];
                    in_flow += x[j][i];
                }
            }
            model.add(out_flow == 1);
            model.add(in_flow == 1);
            out_flow.end();
            in_flow.end();
        }
        model.add(u[0] == 1);
        for (size_t i = 1; i < node_ids.size(); ++i) {
            model.add(u[i] >= 2);
            model.add(u[i] <= node_ids.size());
        }
        for (size_t i = 1; i < node_ids.size(); ++i) {
            for (size_t j = 1; j < node_ids.size(); ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    model.add(u[i] - u[j] + node_ids.size() * x[i][j] <= node_ids.size() - 1);
                }
            }
        }

        // Time Constraints
        model.add(t[0] == 0);
        model.add(depart[0] == 0);
        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            const Node* node = graph.findNode(node_i);
            double service_time = node ? node->getServiceTime() : 0.0;
            model.add(depart[i] == t[i] + phi[i] + service_time);
        }
        for (size_t i = 0; i < node_ids.size(); ++i) {
            for (size_t j = 0; j < node_ids.size(); ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    const Arc* arc = graph.findArc(node_ids[i], node_ids[j]);
                    double min_time = arc->getDistance() / params.getUmax();
                    double max_time = arc->getDistance() / params.getUmin();
                    model.add(s[i][j] >= min_time * x[i][j]);
                    model.add(s[i][j] <= max_time * x[i][j] + params.getBigM() * (1 - x[i][j]));
                    model.add(t[j] >= depart[i] + s[i][j] - params.getBigM() * (1 - x[i][j]));
                }
            }
        }

        // SOC Constraints
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
                    bool is_wireless = arc->getIsWireless();
                    double dist = arc->getDistance();
                    if (!is_wireless) {
                        model.add(y_a[j] <= y_d[i] - params.getEnergyConsumption() * dist + params.getBigM() * (1 - x[i][j]));
                    } else {
                        model.add(y_a[j] <= y_d[i] - params.getEnergyConsumption() * dist + arc->getWirelessChargeRate() * s_z[i][j] + params.getBigM() * (1 - x[i][j]));
                    }
                }
            }
        }

        // Charging Constraints
        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) == station_ids.end()) {
                model.add(phi[i] == 0);
            } else {
                IloExpr sum_w(env);
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    sum_w += w[i][k];
                }
                model.add(sum_w <= 1);
                sum_w.end();
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

        // Linearization Constraints
        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            double min_rate = charge_options[node_i].empty() ? 1.0 : charge_options[node_i][0].getRate();
            for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                min_rate = std::min(min_rate, charge_options[node_i][k].getRate());
            }
            double U_phi = params.getBatteryCapacity() / min_rate;
            for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                model.add(phi_w[i][k] <= phi[i]);
                model.add(phi_w[i][k] <= U_phi * w[i][k]);
                model.add(phi_w[i][k] >= phi[i] - U_phi * (1 - w[i][k]));
                model.add(phi_w[i][k] >= 0);
            }
        }
        for (size_t i = 0; i < node_ids.size(); ++i) {
            for (size_t j = 0; j < node_ids.size(); ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    const Arc* arc = graph.findArc(node_ids[i], node_ids[j]);
                    double max_time = arc->getDistance() / params.getUmin();
                    model.add(s_z[i][j] <= s[i][j]);
                    model.add(s_z[i][j] <= max_time * z[i][j]);
                    model.add(s_z[i][j] >= s[i][j] - max_time * (1 - z[i][j]));
                    model.add(s_z[i][j] >= 0);
                }
            }
        }

        // Solve
        cplex.setOut(std::cout);
        cplex.setParam(IloCplex::TiLim, 300.0); // 5 minutes
        cplex.setParam(IloCplex::EpGap, 0.01);
        if (!cplex.solve()) {
            std::cout << "MILP::optimize: CPLEX failed, status = " << cplex.getStatus() << "\n";
            return Route(initial_nodes);
        }

        // Extract Route
        std::vector<int> route_nodes;
        int current = 0;
        route_nodes.push_back(current);
        while (true) {
            bool found = false;
            for (size_t j = 0; j < node_ids.size(); ++j) {
                size_t i_idx = std::find(node_ids.begin(), node_ids.end(), current) - node_ids.begin();
                if (current != node_ids[j] && graph.findArc(current, node_ids[j]) && cplex.getValue(x[i_idx][j]) > 0.5) {
                    route_nodes.push_back(node_ids[j]);
                    current = node_ids[j];
                    found = true;
                    break;
                }
            }
            if (!found || current == 0) break;
        }

        SubproblemResult result;
        result.new_node_ids = route_nodes;
        result.soc_arrival.resize(route_nodes.size());
        result.soc_departure.resize(route_nodes.size());
        result.arrival_time.resize(route_nodes.size());
        result.departure_time.resize(route_nodes.size());
        result.wireless_decisions.resize(route_nodes.size() - 1, false);
        result.cost = cplex.getObjValue();

        for (size_t i = 0; i < route_nodes.size(); ++i) {
            size_t idx = std::find(node_ids.begin(), node_ids.end(), route_nodes[i]) - node_ids.begin();
            result.soc_arrival[i] = cplex.getValue(y_a[idx]);
            result.soc_departure[i] = cplex.getValue(y_d[idx]);
            result.arrival_time[i] = cplex.getValue(t[idx]);
            result.departure_time[i] = cplex.getValue(depart[idx]);
            int node_i = route_nodes[i];
            size_t node_idx = std::find(node_ids.begin(), node_ids.end(), node_i) - node_ids.begin();
            for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                if (cplex.getValue(w[node_idx][k]) > 0.5) {
                    result.charging_decisions.emplace_back(node_i, static_cast<int>(k), cplex.getValue(phi[node_idx]));
                    break;
                }
            }
        }
        for (size_t i = 0; i < route_nodes.size() - 1; ++i) {
            int from = route_nodes[i];
            int to = route_nodes[i + 1];
            size_t from_idx = std::find(node_ids.begin(), node_ids.end(), from) - node_ids.begin();
            size_t to_idx = std::find(node_ids.begin(), node_ids.end(), to) - node_ids.begin();
            if (graph.findArc(from, to)->getIsWireless() && cplex.getValue(z[from_idx][to_idx]) > 0.5) {
                result.wireless_decisions[i] = true;
            }
        }

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
        const Arc* arc_ij = graph.findArc(i, j);
        const Arc* arc_ia = graph.findArc(i, a);
        const Arc* arc_aj = graph.findArc(a, j);
        if (!arc_ij || !arc_ia || !arc_aj) {
            std::cout << "MILP::solveSubproblem: Invalid arc i=" << i << ", j=" << j << ", a=" << a << "\n";
            throw std::runtime_error("Invalid arc in subproblem");
        }

        IloBoolVar x_ij(env, "x_ij");
        IloBoolVar x_ia(env, "x_ia");
        IloBoolVar x_aj(env, "x_aj");
        IloBoolVar z_ij(env, "z_ij");
        IloNumVar s_ij(env, 0, IloInfinity, "s_ij");
        IloNumVar s_ia(env, 0, IloInfinity, "s_ia");
        IloNumVar s_aj(env, 0, IloInfinity, "s_aj");
        IloNumVar y_i_a(env, 0, params.getBatteryCapacity(), "y_i_a");
        IloNumVar y_i_d(env, 0, params.getBatteryCapacity(), "y_i_d");
        IloNumVar y_a_a(env, 0, params.getBatteryCapacity(), "y_a_a");
        IloNumVar y_a_d(env, 0, params.getBatteryCapacity(), "y_a_d");
        IloNumVar y_j_a(env, 0, params.getBatteryCapacity(), "y_j_a");
        IloNumVar t_i(env, 0, IloInfinity, "t_i");
        IloNumVar t_a(env, 0, IloInfinity, "t_a");
        IloNumVar t_j(env, 0, IloInfinity, "t_j");
        IloNumVar phi_a(env, 0, IloInfinity, "phi_a");
        IloBoolVarArray w_ak(env, charge_options[a].size());
        for (size_t k = 0; k < charge_options[a].size(); ++k) {
            w_ak[k] = IloBoolVar(env, ("w_a_" + std::to_string(k)).c_str());
        }

        IloExpr cost_expr(env);
        for (size_t k = 0; k < charge_options[a].size(); ++k) {
            cost_expr += charge_options[a][k].getCost() * charge_options[a][k].getRate() * phi_a * w_ak[k];
        }
        if (arc_ij->getIsWireless()) {
            cost_expr += params.getWirelessCost() * arc_ij->getWirelessChargeRate() * s_ij * z_ij;
        }
        cost_expr += params.getTimeCost() * (t_j - t_i);
        model.add(IloMinimize(env, cost_expr));
        cost_expr.end();

        model.add(x_ij + x_ia == 1);
        model.add(x_ij + x_aj == 1);
        model.add(x_ia == x_aj);

        model.add(s_ij >= (arc_ij->getDistance() / params.getUmax()) * x_ij);
        model.add(s_ij <= (arc_ij->getDistance() / params.getUmin()) * x_ij);
        model.add(s_ia >= (arc_ia->getDistance() / params.getUmax()) * x_ia);
        model.add(s_ia <= (arc_ia->getDistance() / params.getUmin()) * x_ia);
        model.add(s_aj >= (arc_aj->getDistance() / params.getUmax()) * x_aj);
        model.add(s_aj <= (arc_aj->getDistance() / params.getUmin()) * x_aj);

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
        IloExpr charge_expr(env);
        for (size_t k = 0; k < charge_options[a].size(); ++k) {
            charge_expr += charge_options[a][k].getRate() * phi_a * w_ak[k];
        }
        model.add(y_a_d == y_a_a + charge_expr);
        model.add(y_j_a <= y_a_d - params.getEnergyConsumption() * arc_aj->getDistance() +
                  params.getBatteryCapacity() * (1 - x_aj));
        charge_expr.end();

        model.add(t_i == current_route.getArrivalTime()[pos_i]);
        model.add(t_j >= t_i + s_ij - params.getBigM() * (1 - x_ij));
        model.add(t_a >= t_i + s_ia - params.getBigM() * (1 - x_ia));
        model.add(t_j >= t_a + phi_a + s_aj - params.getBigM() * (1 - x_aj));

        IloExpr sum_w(env);
        for (size_t k = 0; k < charge_options[a].size(); ++k) {
            sum_w += w_ak[k];
        }
        model.add(sum_w == x_ia);
        if (arc_ij->getIsWireless()) {
            model.add(z_ij <= x_ij);
        }
        sum_w.end();

        cplex.setOut(std::cout);
        cplex.setParam(IloCplex::TiLim, 5.0);
        cplex.setParam(IloCplex::EpGap, 0.01);
        if (!cplex.solve()) {
            std::cout << "MILP::solveSubproblem: CPLEX failed, status = " << cplex.getStatus() << "\n";
            return current_route;
        }

        result.cost = cplex.getObjValue();
        result.new_node_ids = current_route.getNodeIds();
        result.soc_arrival = current_route.getSocArrival();
        result.soc_departure = current_route.getSocDeparture();
        result.arrival_time = current_route.getArrivalTime();
        result.departure_time = current_route.getDepartureTime();
        result.wireless_decisions = current_route.getWirelessDecisions();
        result.charging_decisions = current_route.getChargingDecisions();

        size_t pos_j = std::find(node_ids.begin(), node_ids.end(), j) - node_ids.begin();
        if (cplex.getValue(x_ia) > 0.5 && cplex.getValue(x_aj) > 0.5) {
            result.new_node_ids.insert(result.new_node_ids.begin() + pos_j, a);
            result.soc_arrival.insert(result.soc_arrival.begin() + pos_j, cplex.getValue(y_a_a));
            result.soc_departure.insert(result.soc_departure.begin() + pos_j, cplex.getValue(y_a_d));
            result.arrival_time.insert(result.arrival_time.begin() + pos_j, cplex.getValue(t_a));
            result.departure_time.insert(result.departure_time.begin() + pos_j, cplex.getValue(t_a) + cplex.getValue(phi_a));
            result.wireless_decisions.insert(result.wireless_decisions.begin() + pos_j - 1, false);
            result.soc_arrival[pos_j + 1] = cplex.getValue(y_j_a);
            result.soc_departure[pos_j + 1] = cplex.getValue(y_j_a);
            result.arrival_time[pos_j + 1] = cplex.getValue(t_j);
            result.departure_time[pos_j + 1] = cplex.getValue(t_j);
            for (size_t k = 0; k < charge_options[a].size(); ++k) {
                if (cplex.getValue(w_ak[k]) > 0.5) {
                    result.charging_decisions.emplace_back(a, static_cast<int>(k), cplex.getValue(phi_a));
                    break;
                }
            }
        } else {
            result.soc_arrival[pos_j] = cplex.getValue(y_j_a);
            result.soc_departure[pos_j] = cplex.getValue(y_j_a);
            result.arrival_time[pos_j] = cplex.getValue(t_j);
            result.departure_time[pos_j] = cplex.getValue(t_j);
            result.wireless_decisions[pos_j - 1] = (cplex.getValue(z_ij) > 0.5);
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
    route.setSocArrival(result.soc_arrival);
    route.setSocDeparture(result.soc_departure);
    route.setArrivalTime(result.arrival_time);
    route.setDepartureTime(result.departure_time);
    route.setChargingDecisions(result.charging_decisions);
    route.setWirelessDecisions(result.wireless_decisions);
    route.setTotalCost(result.cost);
    route.setFeasible(result.cost < 1e9);
}