#include "../include/MILP.h"
#include "../include/Utils.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <set>
#include <unordered_map>
#include <ilcplex/ilocplex.h>

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
        std::unordered_map<int, size_t> node_to_idx;
        for (const auto& node : nodes) {
            node_ids.push_back(node.getId());
            node_to_idx[node.getId()] = node_ids.size() - 1;
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

        size_t N = node_ids.size();
        double BigM_soc = params.getBatteryCapacity() + 1000.0;
        double BigM_time = 1000.0;

        // Decision Variables
        IloArray<IloBoolVarArray> x(env, N);
        IloArray<IloBoolVarArray> z(env, N);
        IloArray<IloNumVarArray> s_z(env, N);
        for (size_t i = 0; i < N; ++i) {
            x[i] = IloBoolVarArray(env, N);
            z[i] = IloBoolVarArray(env, N);
            s_z[i] = IloNumVarArray(env, N, 0, IloInfinity, ILOFLOAT);
            for (size_t j = 0; j < N; ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    x[i][j] = IloBoolVar(env, ("x_" + std::to_string(node_ids[i]) + "_" + std::to_string(node_ids[j])).c_str());
                    if (graph.findArc(node_ids[i], node_ids[j]) -> getIsWireless()) {
                        z[i][j] = IloBoolVar(env, ("z_" + std::to_string(node_ids[i]) + "_" + std::to_string(node_ids[j])).c_str());
                    }
                    s_z[i][j].setName(("s_z_" + std::to_string(node_ids[i]) + "_" + std::to_string(node_ids[j])).c_str());
                }
            }
        }

        IloNumVarArray y_a(env, N, params.getMinSoc(), params.getBatteryCapacity(), ILOFLOAT);
        IloNumVarArray y_d(env, N, params.getMinSoc(), params.getBatteryCapacity(), ILOFLOAT);
        IloNumVarArray t(env, N, 0, IloInfinity, ILOFLOAT);
        IloNumVarArray phi(env, N, 0, IloInfinity, ILOFLOAT);
        for (size_t i = 0; i < N; ++i) {
            y_a[i].setName(("y_a_" + std::to_string(node_ids[i])).c_str());
            y_d[i].setName(("y_d_" + std::to_string(node_ids[i])).c_str());
            t[i].setName(("t_" + std::to_string(node_ids[i])).c_str());
            phi[i].setName(("phi_" + std::to_string(node_ids[i])).c_str());
        }

        IloArray<IloBoolVarArray> w(env, N);
        IloArray<IloNumVarArray> psi(env, N);
        for (size_t i = 0; i < N; ++i) {
            int node_id = node_ids[i];
            size_t num_options = (node_id < static_cast<int>(charge_options.size()) &&
                                  std::find(station_ids.begin(), station_ids.end(), node_id) != station_ids.end())
                                 ? charge_options[node_id].size() : 0;
            w[i] = IloBoolVarArray(env, num_options);
            psi[i] = IloNumVarArray(env, num_options, 0, IloInfinity, ILOFLOAT);
            for (size_t k = 0; k < num_options; ++k) {
                w[i][k].setName(("w_" + std::to_string(node_id) + "_" + std::to_string(k)).c_str());
                psi[i][k].setName(("psi_" + std::to_string(node_id) + "_" + std::to_string(k)).c_str());
            }
        }

        IloArray<IloNumVarArray> s(env, N);
        for (size_t i = 0; i < N; ++i) {
            s[i] = IloNumVarArray(env, N, 0, IloInfinity, ILOFLOAT);
            for (size_t j = 0; j < N; ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    s[i][j].setName(("s_" + std::to_string(node_ids[i]) + "_" + std::to_string(node_ids[j])).c_str());
                }
            }
        }

        IloNumVarArray u(env, N, 0, N-1, ILOFLOAT);
        for (size_t i = 0; i < N; ++i) {
            u[i].setName(("u_" + std::to_string(node_ids[i])).c_str());
        }

        // Objective Function
        IloExpr obj(env);
        for (size_t i = 0; i < N; ++i) {
            int node_i = node_ids[i];
            for (size_t j = 0; j < N; ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    const Arc* arc = graph.findArc(node_ids[i], node_ids[j]);
                    if (arc -> getIsWireless() && s_z[i][j].getImpl()) {
                        obj += params.getWirelessCost() * arc -> getWirelessChargeRate() * s_z[i][j];
                    }
                }
            }
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end() &&
                node_i < static_cast<int>(charge_options.size())) {
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    obj += charge_options[node_i][k].getCost() * charge_options[node_i][k].getRate() * psi[node_i][k];
                }
            }
        }
        obj += params.getTimeCost() * t[node_to_idx[depot_end_id]];
        model.add(IloMinimize(env, obj));
        obj.end();

        // Linearization Constraints for Charging
        for (size_t i = 0; i < N; ++i) {
            int node_i = node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end() &&
                node_i < static_cast<int>(charge_options.size())) {
                IloExpr total_charge_time(env);
                IloExpr total_energy_charged(env);
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    double max_time = (params.getBatteryCapacity() - params.getMinSoc()) / charge_options[node_i][k].getRate();
                    model.add(psi[i][k] <= max_time * w[i][k]);
                    model.add(psi[i][k] >= 0);
                    total_charge_time += psi[i][k];
                    total_energy_charged += charge_options[node_i][k].getRate() * psi[i][k];
                }
                model.add(phi[i] == total_charge_time);
                model.add(y_d[i] == y_a[i] + total_energy_charged);
                total_charge_time.end();
                total_energy_charged.end();
            } else {
                model.add(phi[i] == 0);
                model.add(y_d[i] == y_a[i]);
                for (size_t k = 0; k < w[i].getSize(); ++k) {
                    model.add(w[i][k] == 0);
                    model.add(psi[i][k] == 0);
                }
            }
        }

        // Linearization Constraints for s_z
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    const Arc* arc = graph.findArc(node_ids[i], node_ids[j]);
                    if (arc -> getIsWireless() && s_z[i][j].getImpl()) {
                        double max_travel_time = arc -> getDistance() / params.getUmin();
                        model.add(s_z[i][j] <= s[i][j]);
                        model.add(s_z[i][j] <= max_travel_time * z[i][j]);
                        model.add(s_z[i][j] >= 0);
                        model.add(z[i][j] <= x[i][j]);
                    } else {
                        model.add(s_z[i][j] == 0);
                        if (z[i][j].getImpl()) {
                            model.add(z[i][j] == 0);
                        }
                    }
                }
            }
        }

        // Flow Constraints
        IloExpr depot_start_flow(env);
        for (size_t j = 0; j < N; ++j) {
            if (node_ids[j] != depot_start_id && graph.findArc(depot_start_id, node_ids[j])) {
                depot_start_flow += x[node_to_idx[depot_start_id]][j];
            }
        }
        model.add(depot_start_flow == 1);
        depot_start_flow.end();

        IloExpr depot_end_flow(env);
        for (size_t i = 0; i < N; ++i) {
            if (node_ids[i] != depot_end_id && graph.findArc(node_ids[i], depot_end_id)) {
                depot_end_flow += x[i][node_to_idx[depot_end_id]];
            }
        }
        model.add(depot_end_flow == 1);
        depot_end_flow.end();

        for (int cust_id : customer_ids) {
            size_t i_idx = node_to_idx[cust_id];
            IloExpr in_flow(env), out_flow(env);
            for (size_t j = 0; j < N; ++j) {
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
            size_t i_idx = node_to_idx[stat_id];
            IloExpr in_flow(env), out_flow(env);
            for (size_t j = 0; j < N; ++j) {
                if (graph.findArc(node_ids[j], stat_id)) {
                    in_flow += x[j][i_idx];
                }
                if (graph.findArc(stat_id, node_ids[j])) {
                    out_flow += x[i_idx][j];
                }
            }
            model.add(in_flow == out_flow);
            in_flow.end();
            out_flow.end();
        }

        // Subtour Elimination Constraints
        for (size_t i = 1; i < N; ++i) {
            for (size_t j = 1; j < N; ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    model.add(u[i] - u[j] + IloNum(N) * x[i][j] <= IloNum(N) - 1);
                }
            }
        }
        model.add(u[0] == 0);

        // SOC Constraints
        model.add(y_a[node_to_idx[depot_start_id]] == params.getInitialSoc());
        model.add(y_d[node_to_idx[depot_start_id]] == y_a[node_to_idx[depot_start_id]]);
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    const Arc* arc = graph.findArc(node_ids[i], node_ids[j]);
                    double dist = arc -> getDistance();
                    IloExpr soc_rhs(env);
                    soc_rhs += y_d[i] - params.getEnergyConsumption() * dist;
                    if (arc -> getIsWireless() && s_z[i][j].getImpl()) {
                        soc_rhs += arc -> getWirelessChargeRate() * s_z[i][j];
                    }
                    model.add(y_a[j] >= soc_rhs - BigM_soc * (1 - x[i][j]));
                    model.add(y_a[j] <= soc_rhs + BigM_soc * (1 - x[i][j]));
                    model.add(y_a[j] >= params.getMinSoc());
                    soc_rhs.end();
                }
            }
        }

        // Time Constraints
        model.add(t[node_to_idx[depot_start_id]] == 0);
        for (size_t i = 0; i < N; ++i) {
            int node_i = node_ids[i];
            const Node* node = graph.findNode(node_i);
            double tau_i = node ? node -> getServiceTime() : 0.0;
            for (size_t j = 0; j < N; ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    const Arc* arc = graph.findArc(node_ids[i], node_ids[j]);
                    double dist = arc -> getDistance();
                    model.add(s[i][j] >= (dist / params.getUmax()) * x[i][j]);
                    model.add(s[i][j] <= (dist / params.getUmin()) * x[i][j] + BigM_time * (1 - x[i][j]));
                    IloExpr time_rhs(env);
                    time_rhs += t[i] + s[i][j];
                    if (std::find(customer_ids.begin(), customer_ids.end(), node_i) != customer_ids.end()) {
                        time_rhs += tau_i;
                    } else if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end()) {
                        time_rhs += phi[i];
                    }
                    model.add(t[j] >= time_rhs - BigM_time * (1 - x[i][j]));
                    model.add(t[j] <= time_rhs + BigM_time * (1 - x[i][j]));
                    time_rhs.end();
                }
            }
        }

        // Charging Constraints
        for (size_t i = 0; i < N; ++i) {
            int node_i = node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end() &&
                node_i < static_cast<int>(charge_options.size())) {
                IloExpr sum_w(env);
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    sum_w += w[i][k];
                }
                IloExpr in_flow(env);
                for (size_t j = 0; j < N; ++j) {
                    if (graph.findArc(node_ids[j], node_i)) {
                        in_flow += x[j][i];
                    }
                }
                model.add(sum_w <= in_flow);
                sum_w.end();
                in_flow.end();
            }
        }

        // Solve
        cplex.setOut(std::cout);
        cplex.setParam(IloCplex::TiLim, 3600);
        cplex.exportModel("model.lp");

        if (!cplex.solve()) {
            std::cout << "MILP::optimize: CPLEX failed, status = " << cplex.getStatus() << "\n";
            return Route(initial_nodes);
        }

        // Extract Route and Costs
        SubproblemResult result;
        result.cost = cplex.getObjValue();
        result.new_node_ids.clear();
        result.soc_arrival.resize(N, 0.0);
        result.soc_departure.resize(N, 0.0);
        result.arrival_time.resize(N, 0.0);
        result.departure_time.resize(N, 0.0);
        result.charging_decisions.clear();
        result.wireless_decisions.resize(N, false);
        result.arc_wireless_decisions.clear();

        double travel_cost = 0.0, wireless_cost = 0.0, station_cost = 0.0;

        int current = depot_start_id;
        result.new_node_ids.push_back(current);
        std::set<int> visited;
        visited.insert(current);
        while (current != depot_end_id) {
            bool found = false;
            size_t i_idx = node_to_idx[current];
            for (size_t j = 0; j < N; ++j) {
                if (node_ids[j] != current && graph.findArc(current, node_ids[j]) &&
                    cplex.getValue(x[i_idx][j]) > 0.5) {
                    if (visited.find(node_ids[j]) != visited.end() && node_ids[j] != depot_end_id) {
                        continue;
                    }
                    result.new_node_ids.push_back(node_ids[j]);
                    const Arc* arc = graph.findArc(current, node_ids[j]);
                    travel_cost += arc -> getDistance();
                    bool wireless = arc -> getIsWireless() &&
                                    ((z[i_idx][j].getImpl() && cplex.getValue(z[i_idx][j]) > 0.1) ||
                                     (s_z[i_idx][j].getImpl() && cplex.getValue(s_z[i_idx][j]) > 0.001));
                    result.wireless_decisions[node_ids[j]] = wireless;
                    if (wireless) {
                        result.arc_wireless_decisions.emplace_back(current, node_ids[j]);
                        if (s_z[i_idx][j].getImpl()) {
                            double s_z_val = cplex.getValue(s_z[i_idx][j]);
                            wireless_cost += params.getWirelessCost() * arc -> getWirelessChargeRate() * s_z_val;
                            std::cout << "Debug: Wireless arc (" << current << "," << node_ids[j]
                                      << "), z_val=" << (z[i_idx][j].getImpl() ? cplex.getValue(z[i_idx][j]) : 0)
                                      << ", s_z_val=" << s_z_val << "\n";
                        }
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
            size_t idx = node_to_idx[result.new_node_ids[i]];
            result.soc_arrival[i] = cplex.getValue(y_a[idx]);
            result.soc_departure[i] = cplex.getValue(y_d[idx]);
            result.arrival_time[i] = cplex.getValue(t[idx]);
            double tau_i = graph.findNode(result.new_node_ids[i]) -> getServiceTime();
            double phi_i = cplex.getValue(phi[idx]);
            result.departure_time[i] = result.arrival_time[i] + tau_i + phi_i;
            int node_i = result.new_node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end() &&
                node_i < static_cast<int>(charge_options.size())) {
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    if (cplex.getValue(w[idx][k]) > 0.5) {
                        result.charging_decisions.emplace_back(node_i,
                            static_cast<int>(k), cplex.getValue(psi[idx][k]));
                        station_cost += charge_options[node_i][k].getCost() * cplex.getValue(psi[idx][k]);
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
        std::cout << "MILP::optimize: Success, cost = " << result.cost
                  << ", Travel cost = " << params.getTimeCost() * cplex.getValue(t[node_to_idx[depot_end_id]])
                  << ", Wireless cost = " << wireless_cost
                  << ", Station cost = " << station_cost
                  << ", Time = " << duration << " s\n";
        std::cout << "Route: ";
        for (int node : result.new_node_ids) {
            std::cout << node << " ";
        }
        std::cout << "\nSOC Arrival: ";
        for (double soc : result.soc_arrival) {
            std::cout << soc << " ";
        }
        std::cout << "\nWireless Arcs: ";
        for (const auto& arc : result.arc_wireless_decisions) {
            std::cout << "(" << arc.first << "," << arc.second << ") ";
        }
        std::cout << "\nStation Charging: ";
        for (const auto& charge : result.charging_decisions) {
            std::cout << "Node " << charge.station_id << ", Option " << charge.option_index << ", Time " << charge.charging_time << "; ";
        }
        std::cout << "\n";

        return result_route;
    } catch (IloException& e) {
        std::cout << "MILP::optimize: CPLEX Exception: " << e.getMessage() << "\n";
        e.end();
        return Route(initial_nodes);
    } catch (const std::exception& e) {
        std::cout << "MILP::optimize: Exception: " << e.what() << "\n";
        return Route(initial_nodes);
    }
}

Route MILP::optimizeNoWCL(const std::vector<int>& initial_nodes,
                          const Graph& graph,
                          const std::vector<std::vector<ChargingOption>>& charge_options,
                          const Parameters& params) {
    auto start_time = std::chrono::high_resolution_clock::now();
    IloModel model(env);
    IloCplex cplex(model);

    try {
        // Validate input
        if (!Utils::validateInitialNodes(initial_nodes, graph)) {
            std::cout << "MILP::optimizeNoWCL: Invalid initial nodes\n";
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
        std::unordered_map<int, size_t> node_to_idx;
        for (const auto& node : nodes) {
            node_ids.push_back(node.getId());
            node_to_idx[node.getId()] = node_ids.size() - 1;
            if (node.getType() == NodeType::CUSTOMER) {
                customer_ids.push_back(node.getId());
            } else if (node.getType() == NodeType::CHARGING_STATION) {
                station_ids.push_back(node.getId());
            } else if (node.getType() == NodeType::DEPOT && node.getId() != depot_start_id) {
                depot_end_id = node.getId();
            }
        }
        if (depot_end_id == -1) {
            const Node* start_depot_node = graph.findNode(depot_start_id);
            if (start_depot_node && start_depot_node->getType() == NodeType::DEPOT) {
                std::cout << "Warning: Only one depot (ID " << depot_start_id << ") found. Using it as both start and end depot.\n";
                depot_end_id = depot_start_id;
            } else {
                throw std::runtime_error("Depot end not found, and start depot ID 0 is not a valid depot.");
            }
        }

        size_t N = node_ids.size();

        // Calculate Big-M values
        double max_distance = 0.0;
        for (const auto& arc : arcs) {
            max_distance = std::max(max_distance, arc.getDistance());
        }
        double max_node_time_allowance = 30.0; // Estimate max service + charge time
        double BigM_time = (max_distance * N) / params.getUmin() + (N * max_node_time_allowance);
        double BigM_soc = params.getBatteryCapacity() + (max_distance * N * params.getEnergyConsumption());

        // Decision Variables
        IloArray<IloBoolVarArray> x(env, N);
        for (size_t i = 0; i < N; ++i) {
            x[i] = IloBoolVarArray(env, N);
            for (size_t j = 0; j < N; ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    x[i][j] = IloBoolVar(env, ("x_" + std::to_string(node_ids[i]) + "_" + std::to_string(node_ids[j])).c_str());
                }
            }
        }

        IloNumVarArray y_a(env, N, params.getMinSoc(), params.getBatteryCapacity(), ILOFLOAT);
        IloNumVarArray y_d(env, N, params.getMinSoc(), params.getBatteryCapacity(), ILOFLOAT);
        IloNumVarArray t(env, N, 0, IloInfinity, ILOFLOAT);
        IloNumVarArray phi(env, N, 0, IloInfinity, ILOFLOAT);
        for (size_t i = 0; i < N; ++i) {
            y_a[i].setName(("y_a_" + std::to_string(node_ids[i])).c_str());
            y_d[i].setName(("y_d_" + std::to_string(node_ids[i])).c_str());
            t[i].setName(("t_" + std::to_string(node_ids[i])).c_str());
            phi[i].setName(("phi_" + std::to_string(node_ids[i])).c_str());
        }

        IloArray<IloBoolVarArray> w(env, N);
        IloArray<IloNumVarArray> psi(env, N);
        for (size_t i = 0; i < N; ++i) {
            int node_id = node_ids[i];
            size_t num_options = (node_id < static_cast<int>(charge_options.size()) &&
                                  std::find(station_ids.begin(), station_ids.end(), node_id) != station_ids.end())
                                 ? charge_options[node_id].size() : 0;
            w[i] = IloBoolVarArray(env, num_options);
            psi[i] = IloNumVarArray(env, num_options, 0, IloInfinity, ILOFLOAT);
            for (size_t k = 0; k < num_options; ++k) {
                w[i][k].setName(("w_" + std::to_string(node_id) + "_" + std::to_string(k)).c_str());
                psi[i][k].setName(("psi_" + std::to_string(node_id) + "_" + std::to_string(k)).c_str());
            }
        }

        IloArray<IloNumVarArray> s(env, N);
        for (size_t i = 0; i < N; ++i) {
            s[i] = IloNumVarArray(env, N, 0, IloInfinity, ILOFLOAT);
            for (size_t j = 0; j < N; ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    s[i][j].setName(("s_" + std::to_string(node_ids[i]) + "_" + std::to_string(node_ids[j])).c_str());
                }
            }
        }

        IloNumVarArray u(env, N, 0, N-1, ILOFLOAT);
        for (size_t i = 0; i < N; ++i) {
            u[i].setName(("u_" + std::to_string(node_ids[i])).c_str());
        }

        // Objective Function
        IloExpr obj(env);
        for (size_t i = 0; i < N; ++i) {
            int node_i = node_ids[i];
            for (size_t j = 0; j < N; ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    const Arc* arc = graph.findArc(node_ids[i], node_ids[j]);
                    obj += arc->getDistance() * x[i][j];
                }
            }
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end() &&
                node_i < static_cast<int>(charge_options.size())) {
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    obj += charge_options[node_i][k].getCost() * w[i][k]; // Fixed cost per option
                }
            }
        }
        obj += params.getTimeCost() * t[node_to_idx[depot_end_id]];
        model.add(IloMinimize(env, obj));
        obj.end();

        // Linearization Constraints for Charging
        for (size_t i = 0; i < N; ++i) {
            int node_i = node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end() &&
                node_i < static_cast<int>(charge_options.size())) {
                IloExpr total_charge_time(env);
                IloExpr total_energy_charged(env);
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    double max_time = (params.getBatteryCapacity() - params.getMinSoc()) / charge_options[node_i][k].getRate();
                    model.add(psi[i][k] <= max_time * w[i][k]);
                    model.add(psi[i][k] >= 0);
                    total_charge_time += psi[i][k];
                    total_energy_charged += charge_options[node_i][k].getRate() * psi[i][k];
                }
                model.add(phi[i] == total_charge_time);
                model.add(y_d[i] == y_a[i] + total_energy_charged);
                total_charge_time.end();
                total_energy_charged.end();
            } else {
                model.add(phi[i] == 0);
                model.add(y_d[i] == y_a[i]);
                for (size_t k = 0; k < w[i].getSize(); ++k) {
                    model.add(w[i][k] == 0);
                    model.add(psi[i][k] == 0);
                }
            }
        }

        // Flow Constraints
        IloExpr depot_start_flow(env);
        for (size_t j = 0; j < N; ++j) {
            if (node_ids[j] != depot_start_id && graph.findArc(depot_start_id, node_ids[j])) {
                depot_start_flow += x[node_to_idx[depot_start_id]][j];
            }
        }
        model.add(depot_start_flow == 1);
        depot_start_flow.end();

        IloExpr depot_end_flow(env);
        for (size_t i = 0; i < N; ++i) {
            if (node_ids[i] != depot_end_id && graph.findArc(node_ids[i], depot_end_id)) {
                depot_end_flow += x[i][node_to_idx[depot_end_id]];
            }
        }
        model.add(depot_end_flow == 1);
        depot_end_flow.end();

        for (int cust_id : customer_ids) {
            size_t i_idx = node_to_idx[cust_id];
            IloExpr in_flow(env), out_flow(env);
            for (size_t j = 0; j < N; ++j) {
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
            size_t i_idx = node_to_idx[stat_id];
            IloExpr in_flow(env), out_flow(env);
            for (size_t j = 0; j < N; ++j) {
                if (graph.findArc(node_ids[j], stat_id)) {
                    in_flow += x[j][i_idx];
                }
                if (graph.findArc(stat_id, node_ids[j])) {
                    out_flow += x[i_idx][j];
                }
            }
            model.add(in_flow == out_flow);
            in_flow.end();
            out_flow.end();
        }

        // Subtour Elimination Constraints
        for (size_t i = 1; i < N; ++i) {
            for (size_t j = 1; j < N; ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    model.add(u[i] - u[j] + IloNum(N) * x[i][j] <= IloNum(N) - 1);
                }
            }
        }
        model.add(u[0] == 0);

        // SOC Constraints
        model.add(y_a[node_to_idx[depot_start_id]] == params.getInitialSoc());
        model.add(y_d[node_to_idx[depot_start_id]] == y_a[node_to_idx[depot_start_id]]);
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    const Arc* arc = graph.findArc(node_ids[i], node_ids[j]);
                    double dist = arc->getDistance();
                    IloExpr soc_rhs(env);
                    soc_rhs += y_d[i] - params.getEnergyConsumption() * dist;
                    model.add(y_a[j] >= soc_rhs - BigM_soc * (1 - x[i][j]));
                    model.add(y_a[j] <= soc_rhs + BigM_soc * (1 - x[i][j]));
                    model.add(y_a[j] >= params.getMinSoc());
                    soc_rhs.end();
                }
            }
        }

        // Time Constraints
        model.add(t[node_to_idx[depot_start_id]] == 0);
        for (size_t i = 0; i < N; ++i) {
            int node_i = node_ids[i];
            const Node* node = graph.findNode(node_i);
            double tau_i = node ? node->getServiceTime() : 0.0;
            for (size_t j = 0; j < N; ++j) {
                if (i != j && graph.findArc(node_ids[i], node_ids[j])) {
                    const Arc* arc = graph.findArc(node_ids[i], node_ids[j]);
                    double dist = arc->getDistance();
                    model.add(s[i][j] >= (dist / params.getUmax()) * x[i][j]);
                    model.add(s[i][j] <= (dist / params.getUmin()) * x[i][j] + BigM_time * (1 - x[i][j]));
                    IloExpr time_rhs(env);
                    time_rhs += t[i] + s[i][j];
                    if (std::find(customer_ids.begin(), customer_ids.end(), node_i) != customer_ids.end()) {
                        time_rhs += tau_i;
                    } else if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end()) {
                        time_rhs += phi[i];
                    }
                    model.add(t[j] >= time_rhs - BigM_time * (1 - x[i][j]));
                    model.add(t[j] <= time_rhs + BigM_time * (1 - x[i][j]));
                    time_rhs.end();
                }
            }
        }

        // Charging Constraints
        for (size_t i = 0; i < N; ++i) {
            int node_i = node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end() &&
                node_i < static_cast<int>(charge_options.size())) {
                IloExpr sum_w(env);
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    sum_w += w[i][k];
                }
                IloExpr in_flow(env);
                for (size_t j = 0; j < N; ++j) {
                    if (graph.findArc(node_ids[j], node_i)) {
                        in_flow += x[j][i];
                    }
                }
                model.add(sum_w <= in_flow);
                sum_w.end();
                in_flow.end();
                }
        }

        // Solve
        cplex.setOut(std::cout);
        cplex.setParam(IloCplex::TiLim, 3600);
        cplex.exportModel("model_no_wcl.lp");

        if (!cplex.solve()) {
            std::cout << "MILP::optimizeNoWCL: CPLEX failed, status = " << cplex.getStatus() << "\n";
            Route result_route(initial_nodes);
            result_route.setFeasible(false);
            return result_route;
        }

        // Extract Route and Costs
        SubproblemResult result;
        result.cost = cplex.getObjValue();
        result.new_node_ids.clear();
        result.soc_arrival.resize(N, 0.0);
        result.soc_departure.resize(N, 0.0);
        result.arrival_time.resize(N, 0.0);
        result.departure_time.resize(N, 0.0);
        result.charging_decisions.clear();
        result.wireless_decisions.resize(N, false);
        result.arc_wireless_decisions.clear();

        double travel_cost = 0.0, station_cost = 0.0;

        int current = depot_start_id;
        result.new_node_ids.push_back(current);
        std::set<int> visited;
        visited.insert(current);
        while (current != depot_end_id) {
            bool found = false;
            size_t i_idx = node_to_idx[current];
            for (size_t j = 0; j < N; ++j) {
                if (node_ids[j] != current && graph.findArc(current, node_ids[j]) &&
                    cplex.getValue(x[i_idx][j]) > 0.5) {
                    if (visited.find(node_ids[j]) != visited.end() && node_ids[j] != depot_end_id) {
                        continue;
                    }
                    result.new_node_ids.push_back(node_ids[j]);
                    const Arc* arc = graph.findArc(current, node_ids[j]);
                    travel_cost += arc->getDistance();
                    current = node_ids[j];
                    visited.insert(current);
                    found = true;
                    break;
                }
            }
            if (!found) {
                std::cout << "MILP::optimizeNoWCL: Failed to construct route\n";
                Route result_route(initial_nodes);
                result_route.setFeasible(false);
                return result_route;
            }
        }

        for (size_t i = 0; i < result.new_node_ids.size(); ++i) {
            size_t idx = node_to_idx[result.new_node_ids[i]];
            result.soc_arrival[i] = cplex.getValue(y_a[idx]);
            result.soc_departure[i] = cplex.getValue(y_d[idx]);
            result.arrival_time[i] = cplex.getValue(t[idx]);
            double tau_i = graph.findNode(result.new_node_ids[i])->getServiceTime();
            double phi_i = cplex.getValue(phi[idx]);
            result.departure_time[i] = result.arrival_time[i] + tau_i + phi_i;
            int node_i = result.new_node_ids[i];
            if (std::find(station_ids.begin(), station_ids.end(), node_i) != station_ids.end() &&
                node_i < static_cast<int>(charge_options.size())) {
                for (size_t k = 0; k < charge_options[node_i].size(); ++k) {
                    if (cplex.getValue(w[idx][k]) > 0.5) {
                        result.charging_decisions.emplace_back(node_i, static_cast<int>(k), cplex.getValue(psi[idx][k]));
                        station_cost += charge_options[node_i][k].getCost();
                        break;
                    }
                }
            }
        }

        // Check SoC feasibility
        for (size_t i = 0; i < result.new_node_ids.size(); ++i) {
            if (result.soc_arrival[i] < params.getMinSoc() - 1e-6 || result.soc_departure[i] < params.getMinSoc() - 1e-6) {
                std::cout << "Warning: SoC violation at node " << result.new_node_ids[i]
                          << ": Arrival SoC = " << result.soc_arrival[i]
                          << ", Departure SoC = " << result.soc_departure[i] << "\n";
                result.feasible = false;
            }
        }

        result.feasible = true;
        Route result_route(result.new_node_ids);
        updateRoute(result_route, result);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
        std::cout << "MILP::optimizeNoWCL: Success, cost = " << result.cost
                  << ", TimeCost = " << params.getTimeCost() * cplex.getValue(t[node_to_idx[depot_end_id]])
                  << ", StationCost = " << station_cost
                  << ", TotalDistance = " << travel_cost
                  << ", OptimizationTime = " << duration << " s\n";
        std::cout << "Route: ";
        for (int node : result.new_node_ids) {
            std::cout << node << " ";
        }
        std::cout << "\nSOC Arrival: ";
        for (double soc : result.soc_arrival) {
            std::cout << soc << " ";
        }
        std::cout << "\nStation Charging: ";
        for (const auto& charge : result.charging_decisions) {
            std::cout << "Node " << charge.station_id << ", Option " << charge.option_index << ", Time " << charge.charging_time << "; ";
        }
        std::cout << "\n";

        return result_route;
    } catch (IloException& e) {
        std::cout << "MILP::optimizeNoWCL: CPLEX Exception: " << e.getMessage() << "\n";
        e.end();
        Route result_route(initial_nodes);
        result_route.setFeasible(false);
        return result_route;
    } catch (const std::exception& e) {
        std::cout << "MILP::optimizeNoWCL: Exception: " << e.what() << "\n";
        Route result_route(initial_nodes);
        result_route.setFeasible(false);
        return result_route;
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
    route.setArcWirelessDecisions(result.arc_wireless_decisions);
    route.setTotalCost(result.cost);
    route.setFeasible(result.feasible);
}