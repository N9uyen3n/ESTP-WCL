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
        IloNumVarArray u(env, node_ids.size(), 0, IloInfinity, ILOFLOAT);

        // Auxiliary Variables for Linearization
        IloArray<IloNumVarArray> phi_w(env, node_ids.size());
        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            phi_w[i] = IloNumVarArray(env, charge_options[node_i].size(), 0, IloInfinity, ILOFLOAT);
        }
        IloArray<IloNumVarArray> s_z(env, node_ids.size());
        for (size_t i = 0; i < node_ids.size(); ++i) {
            s_z[i] = IloNumVarArray(env, node_ids.size(), 0, IloInfinity, ILOFLOAT);
        }

        // Objective Function
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

        // Linearization Constraints for phi_w
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

        // Linearization Constraints for s_z
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

        // Flow Constraints
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

        // Subtour Elimination Constraints (Miller-Tucker-Zemlin)
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
                    double dist = arc->getDistance();
                    if (arc->getIsWireless()) {
                        model.add(y_a[j] <= y_d[i] - params.getEnergyConsumption() * dist +
                                  arc->getWirelessChargeRate() * s_z[i][j] +
                                  params.getBatteryCapacity() * (1 - x[i][j]));
                    } else {
                        model.add(y_a[j] <= y_d[i] - params.getEnergyConsumption() * dist +
                                  params.getBatteryCapacity() * (1 - x[i][j]));
                    }
                    model.add(y_a[j] >= 0);
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

        // Solve
        cplex.setOut(std::cout);
        cplex.setParam(IloCplex::TiLim, 300.0);
        cplex.setParam(IloCplex::EpGap, 0.01);
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
                        result.charging_decisions.emplace_back(node_i, static_cast<int>(k), phi_i);
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

Route MILP::MILPFixVariable(const std::vector<std::pair<int, int>>& fixed_arcs,
                           const Graph& graph,
                           const std::vector<std::vector<ChargingOption>>& charge_options,
                           const Parameters& params) {
    auto start_time = std::chrono::high_resolution_clock::now();
    IloModel model(env);
    IloCplex cplex(model);

    try {
        // Sets
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

        // Decision Variables
        IloArray<IloBoolVarArray> x(env, node_ids.size());
        for (size_t i = 0; i < node_ids.size(); ++i) {
            x[i] = IloBoolVarArray(env, node_ids.size());
            for (size_t j = 0; j < node_ids.size(); ++j) {
                x[i][j] = IloBoolVar(env, ("x_" + std::to_string(node_ids[i]) + "_" + std::to_string(node_ids[j])).c_str());
            }
        }
        // Fix variables for arcs in fixed_arcs
        for (const auto& arc : fixed_arcs) {
            int from = arc.first;
            int to = arc.second;
            size_t i_idx = std::find(node_ids.begin(), node_ids.end(), from) - node_ids.begin();
            size_t j_idx = std::find(node_ids.begin(), node_ids.end(), to) - node_ids.begin();
            if (graph.findArc(from, to)) {
                x[i_idx][j_idx].setBounds(1, 1); // Fix x[i][j] = 1
            } else {
                throw std::runtime_error("Fixed arc does not exist in graph");
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
        IloNumVarArray u(env, node_ids.size(), 0, IloInfinity, ILOFLOAT);

        // Auxiliary Variables for Linearization
        IloArray<IloNumVarArray> phi_w(env, node_ids.size());
        for (size_t i = 0; i < node_ids.size(); ++i) {
            int node_i = node_ids[i];
            phi_w[i] = IloNumVarArray(env, charge_options[node_i].size(), 0, IloInfinity, ILOFLOAT);
        }
        IloArray<IloNumVarArray> s_z(env, node_ids.size());
        for (size_t i = 0; i < node_ids.size(); ++i) {
            s_z[i] = IloNumVarArray(env, node_ids.size(), 0, IloInfinity, ILOFLOAT);
        }

        // Objective Function
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

        // Linearization Constraints for phi_w
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

        // Linearization Constraints for s_z
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

        // Flow Constraints
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

        // Subtour Elimination Constraints (Miller-Tucker-Zemlin)
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
                    double dist = arc->getDistance();
                    if (arc->getIsWireless()) {
                        model.add(y_a[j] <= y_d[i] - params.getEnergyConsumption() * dist +
                                  arc->getWirelessChargeRate() * s_z[i][j] +
                                  params.getBatteryCapacity() * (1 - x[i][j]));
                    } else {
                        model.add(y_a[j] <= y_d[i] - params.getEnergyConsumption() * dist +
                                  params.getBatteryCapacity() * (1 - x[i][j]));
                    }
                    model.add(y_a[j] >= 0);
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

        // Solve
        cplex.setOut(std::cout);
        cplex.setParam(IloCplex::TiLim, 300.0);
        cplex.setParam(IloCplex::EpGap, 0.01);
        if (!cplex.solve()) {
            std::cout << "MILP::MILPFixVariable: CPLEX failed, status = " << cplex.getStatus() << "\n";
            return Route(std::vector<int>{});
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
                    current = node_ids[j];
                    visited.insert(current);
                    found = true;
                    break;
                }
            }
            if (!found) {
                std::cout << "MILP::MILPFixVariable: Failed to construct route\n";
                return Route(std::vector<int>{});
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
                        result.charging_decisions.emplace_back(node_i, static_cast<int>(k), phi_i);
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
        std::cout << "MILP::MILPFixVariable: Success, cost = " << result.cost << ", Time = " << duration << " s\n";
        return result_route;
    } catch (IloException& e) {
        std::cout << "MILP::MILPFixVariable: CPLEX Exception: " << e.getMessage() << "\n";
        return Route(std::vector<int>{});
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