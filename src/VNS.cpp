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
       init_route(init_route),
      current_route(initial_route),
      graph(graph),
      charge_options(charge_options),
      params(params),
      max_iterations(max_iterations),
      rng(rng),
      operator_weights_({0.4, 0.3, 0.2, 0.1}),
      no_improvement_counter(0){
    if (!Utils::validateInitialNodes(initial_route, graph)) {
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
// Hàm kiểm tra tính hợp lệ của tuyến đường (tương tự isValidRoute trong VNSOptimizer)
bool isValidRoute(const std::vector<int>& route, const Graph& graph, const std::vector<int>& initial_nodes) {
    if (route.front() != 0 || route.back() != 0) return false;

    // Lấy danh sách khách hàng bắt buộc từ initial_nodes
    std::set<int> required_customers;
    for (int id : initial_nodes) {
        if (Utils::isCustomer(id, graph.getNodes())) {
            required_customers.insert(id);
        }
    }

    // Kiểm tra khách hàng được thăm
    std::set<int> visited_customers;
    for (size_t i = 1; i < route.size() - 1; ++i) {
        if (route[i] == 0) return false; // Không được có depot giữa chừng
        if (Utils::isCustomer(route[i], graph.getNodes())) {
            if (visited_customers.count(route[i])) return false; // Không lặp lại khách hàng
            visited_customers.insert(route[i]);
        }
    }
    return visited_customers == required_customers;
}

// Hàm kiểm tra cải thiện đáng kể (tương tự checkSignificantImprovement)
bool checkSignificantImprovement(double old_cost, double new_cost) {
    if (old_cost <= 0) return false;
    double improvement = (old_cost - new_cost) / old_cost * 100.0;
    return improvement >= 0.01;
}

Route VNS::run() {
    Route best_route(current_route);
    SubproblemResult initial_result = solveSubproblem(best_route, graph, charge_options, params);

    // Nếu tuyến đường ban đầu không khả thi hoặc không hợp lệ, thử sửa chữa
    if (!initial_result.feasible || !isValidRoute(best_route.getNodeIds(), graph, init_route)) {
        best_route.setTotalCost(std::numeric_limits<double>::infinity());
        best_route.setFeasible(false);
        // Thử chèn trạm sạc
        best_route = insertStation(best_route, graph, params);
        initial_result = solveSubproblem(best_route, graph, charge_options, params);

        // Nếu vẫn không khả thi, thử shaking tối đa 10 lần
        int max_attempts = 10;
        int attempt = 0;
        while ((!initial_result.feasible || !isValidRoute(best_route.getNodeIds(), graph, init_route)) && attempt < max_attempts) {
            int neighborhood = attempt % 4; // Chọn toán tử lân cận
            best_route = shake(Route(best_route.getNodeIds()), neighborhood, graph, params);
            initial_result = solveSubproblem(best_route, graph, charge_options, params);
            if (!initial_result.feasible) {
                best_route = insertStation(best_route, graph, params);
                initial_result = solveSubproblem(best_route, graph, charge_options, params);
            }
            attempt++;
        }

        // Nếu vẫn không khả thi, đặt chi phí vô cực và tiếp tục
        if (!initial_result.feasible || !isValidRoute(best_route.getNodeIds(), graph, init_route)) {
            std::cerr << "Warning: Could not find a feasible initial route after " << max_attempts << " attempts." << std::endl;
            best_route.setTotalCost(std::numeric_limits<double>::infinity());
            best_route.setFeasible(false);
        } else {
            updateRoute(best_route, initial_result);
        }
    } else {
        updateRoute(best_route, initial_result);
    }

    double best_cost = initial_result.feasible ? initial_result.cost : std::numeric_limits<double>::infinity();
    current_route = best_route.getNodeIds();
    double current_cost = best_cost;
    int no_improvement_counter = 0;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        int neighborhood = selectOperatorWeighted();
        std::cout << "Selected Neighborhood: " << neighborhood + 1 << "\n";

        // Shaking
        Route shaken_route = shake(Route(current_route), neighborhood, graph, params);
        SubproblemResult shake_result = solveSubproblem(shaken_route, graph, charge_options, params);

        // Nếu tuyến đường không khả thi hoặc không hợp lệ, thử chèn trạm sạc
        if (!shake_result.feasible || !isValidRoute(shaken_route.getNodeIds(), graph, init_route)) {
            shaken_route = insertStation(shaken_route, graph, params);
            shake_result = solveSubproblem(shaken_route, graph, charge_options, params);
            if (!shake_result.feasible || !isValidRoute(shaken_route.getNodeIds(), graph, init_route)) {
                no_improvement_counter++;
                continue;
            }
        }

        // Local Search
        Route local_optimum = localSearch(shaken_route);
        SubproblemResult result = solveSubproblem(local_optimum, graph, charge_options, params);

        // Kiểm tra tính hợp lệ và khả thi
        if (!result.feasible || !isValidRoute(local_optimum.getNodeIds(), graph, init_route)) {
            no_improvement_counter++;
            continue;
        }

        // Evaluate cost
        double new_cost = result.cost;

        // Acceptance criterion
        bool significant_improvement = false;
        if (new_cost < best_cost) {
            best_cost = new_cost;
            best_route = local_optimum;
            current_route = Route(result.new_node_ids).getNodeIds();
            updateRoute(Route(current_route), result);
            current_cost = new_cost;
            significant_improvement = checkSignificantImprovement(best_cost, new_cost);
            no_improvement_counter = significant_improvement ? 0 : no_improvement_counter + 1;
            operator_weights_[neighborhood] += 0.1; // Tăng trọng số
            normalizeWeights();
        } else if (new_cost < current_cost) {
            current_route = Route(result.new_node_ids).getNodeIds();
            updateRoute(Route(current_route), result);
            current_cost = new_cost;
            no_improvement_counter++;
        } else {
            no_improvement_counter++;
        }

        // Thoát sớm nếu không cải thiện quá lâu
        if (no_improvement_counter >= 100) {
            break;
        }

        // Logging every 10 iterations
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
            std::cout << "  No-Improvement Counter: " << no_improvement_counter << "\n";
            std::cout << "  Operator Weights: ";
            for (double w : operator_weights_) {
                std::cout << std::fixed << std::setprecision(2) << w << " ";
            }
            std::cout << "\n------------------------\n";
        }
    }

    // Cập nhật lại best_route trước khi trả về
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
    result.cost = std::numeric_limits<double>::infinity();
    result.soc_arrival.resize(result.new_node_ids.size(), 0.0);
    result.soc_departure.resize(result.new_node_ids.size(), 0.0);
    result.arrival_time.resize(result.new_node_ids.size(), 0.0);
    result.departure_time.resize(result.new_node_ids.size(), 0.0);
    result.charging_decisions.resize(result.new_node_ids.size(), ChargingDecision(0, 0, 0.0));
    result.wireless_decisions.resize(result.new_node_ids.size() - 1, false);

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
        cplex.setParam(IloCplex::TiLim, 50);
        cplex.setParam(IloCplex::EpGap, 0.1);
        cplex.setOut(env.getNullStream());

        int n = result.new_node_ids.size();
        int m = n - 1;

        // Decision variables
        IloNumVarArray t(env, n, 0, IloInfinity, ILOFLOAT); // Arrival time
        IloNumVarArray ya(env, n, params.getMinSoc(), params.getBatteryCapacity(), ILOFLOAT); // SOC on arrival
        IloNumVarArray yd(env, n, params.getMinSoc(), params.getBatteryCapacity(), ILOFLOAT); // SOC on departure
        IloNumVarArray phi(env, n, 0, IloInfinity, ILOFLOAT); // Charging time
        std::vector<IloBoolVarArray> w(n); // Charging option selection
        for (int i = 0; i < n; ++i) {
            w[i] = IloBoolVarArray(env, charge_options[result.new_node_ids[i]].size());
        }
        IloNumVarArray s(env, m, 0, IloInfinity, ILOFLOAT); // Travel time
        IloBoolVarArray z(env, m); // Wireless charging decision

        // Linearization variables for phi[i] * w[i][k]
        std::vector<IloNumVarArray> phi_w(n);
        for (int i = 0; i < n; ++i) {
            phi_w[i] = IloNumVarArray(env, charge_options[result.new_node_ids[i]].size(), 0, IloInfinity, ILOFLOAT);
        }

        // Objective function
        IloExpr obj(env);
        for (int i = 0; i < n; ++i) {
            int node_id = result.new_node_ids[i];
            for (size_t k = 0; k < charge_options[node_id].size(); ++k) {
                obj += charge_options[node_id][k].getCost() * charge_options[node_id][k].getRate() * phi_w[i][k];
            }
        }
        for (int i = 0; i < m; ++i) {
            const Arc* arc = graph.findArc(result.new_node_ids[i], result.new_node_ids[i + 1]);
            if (arc && arc->getIsWireless()) {
                obj += params.getWirelessCost() * arc->getWirelessChargeRate() * s[i] * z[i];
            }
        }
        obj += params.getTimeCost() * t[n - 1];
        model.add(IloMinimize(env, obj));

        // Linearization constraints for phi[i] * w[i][k]
        for (int i = 0; i < n; ++i) {
            int node_id = result.new_node_ids[i];
            for (size_t k = 0; k < charge_options[node_id].size(); ++k) {
                double U_phi = params.getBatteryCapacity() / (charge_options[node_id].size() > 0 ? charge_options[node_id][0].getRate() : 1.0);
                model.add(phi_w[i][k] <= phi[i]);
                model.add(phi_w[i][k] <= U_phi * w[i][k]);
                model.add(phi_w[i][k] >= phi[i] - U_phi * (1 - w[i][k]));
                model.add(phi_w[i][k] >= 0);
            }
        }

        // No charging at nodes without options
        for (int i = 0; i < n; ++i) {
            if (charge_options[result.new_node_ids[i]].empty()) {
                model.add(phi[i] == 0);
                for (size_t k = 0; k < w[i].getSize(); ++k) {
                    model.add(w[i][k] == 0);
                }
            }
        }

        // Time progression constraints
        model.add(t[0] == 0);
        for (int i = 1; i < n; ++i) {
            const Arc* arc = graph.findArc(result.new_node_ids[i - 1], result.new_node_ids[i]);
            if (!arc) {
                std::cerr << "Error: Arc from " << result.new_node_ids[i - 1] << " to " << result.new_node_ids[i] << " not found." << std::endl;
                env.end();
                return result;
            }
            double service_time = graph.findNode(result.new_node_ids[i - 1])->getServiceTime();
            if (Utils::isChargingStation(result.new_node_ids[i - 1], graph.getNodes())) {
                model.add(t[i] >= t[i - 1] + phi[i - 1] + s[i - 1]);
            } else if (result.new_node_ids[i - 1] != 0) {
                model.add(t[i] >= t[i - 1] + service_time + s[i - 1]);
            } else {
                model.add(t[i] >= t[i - 1] + s[i - 1]);
            }
            double min_s = arc->getDistance() / params.getUmax();
            double max_s = arc->getDistance() / params.getUmin();
            s[i - 1].setBounds(min_s, max_s);
        }

        // SOC constraints
        model.add(ya[0] == params.getInitialSoc());
        model.add(yd[0] == ya[0]);
        for (int i = 1; i < n; ++i) {
            const Arc* arc = graph.findArc(result.new_node_ids[i - 1], result.new_node_ids[i]);
            double energy_consumed = params.getEnergyConsumption() * arc->getDistance();
            IloExpr soc_change(env);
            soc_change = yd[i - 1] - energy_consumed;
            if (arc->getIsWireless()) {
                soc_change += arc->getWirelessChargeRate() * s[i - 1] * z[i - 1];
            }
            model.add(ya[i] <= soc_change);
            if (Utils::isChargingStation(result.new_node_ids[i], graph.getNodes())) {
                IloExpr charge_amount(env);
                for (size_t k = 0; k < charge_options[result.new_node_ids[i]].size(); ++k) {
                    charge_amount += charge_options[result.new_node_ids[i]][k].getRate() * phi_w[i][k];
                }
                model.add(yd[i] == ya[i] + charge_amount);
            } else {
                model.add(yd[i] == ya[i]);
            }
        }

        // Charging selection constraints
        for (int i = 0; i < n; ++i) {
            if (!charge_options[result.new_node_ids[i]].empty()) {
                IloExpr sum_w(env);
                for (size_t k = 0; k < charge_options[result.new_node_ids[i]].size(); ++k) {
                    sum_w += w[i][k];
                }
                model.add(sum_w <= 1);
            }
        }

        // Battery capacity constraints
        for (int i = 0; i < n; ++i) {
            model.add(ya[i] >= params.getMinSoc());
            model.add(yd[i] >= params.getMinSoc());
            model.add(ya[i] <= params.getBatteryCapacity());
            model.add(yd[i] <= params.getBatteryCapacity());
        }

        // Solve the model
        if (!cplex.solve()) {
            std::cerr << "CPLEX: No solution found. Status: " << cplex.getStatus() << std::endl;
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
            result.departure_time[i] = result.arrival_time[i];
            if (Utils::isChargingStation(result.new_node_ids[i], graph.getNodes())) {
                result.departure_time[i] += cplex.getValue(phi[i]);
            } else if (result.new_node_ids[i] != 0 && i != n - 1) {
                result.departure_time[i] += graph.findNode(result.new_node_ids[i])->getServiceTime();
            }
            for (size_t k = 0; k < charge_options[result.new_node_ids[i]].size(); ++k) {
                if (cplex.getValue(w[i][k]) > 0.5) {
                    result.charging_decisions[i] = ChargingDecision(result.new_node_ids[i], static_cast<int>(k + 1), cplex.getValue(phi[i]));
                    break;
                }
            }
        }

        for (int i = 0; i < m; ++i) {
            const Arc* arc = graph.findArc(result.new_node_ids[i], result.new_node_ids[i + 1]);
            if (arc && arc->getIsWireless() && cplex.getValue(z[i]) > 0.5) {
                result.wireless_decisions[i] = true;
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
    std::cout << "Performing local search...\n";
    Route best_local = current_route;
    SubproblemResult best_result = solveSubproblem(best_local, graph, charge_options, params);
    if (!best_result.feasible) return current_route;

    bool improved = true;
    while (improved) {
        improved = false;
        // Try 2-opt
        Route new_route = twoOpt(best_local, graph, params);
        SubproblemResult new_result = solveSubproblem(new_route, graph, charge_options, params);
        if (new_result.feasible && new_result.cost < best_result.cost) {
            best_local = new_route;
            best_result = new_result;
            improved = true;
            continue;
        }
        // Try relocate
        new_route = relocate(best_local, graph, params);
        new_result = solveSubproblem(new_route, graph, charge_options, params);
        if (new_result.feasible && new_result.cost < best_result.cost) {
            best_local = new_route;
            best_result = new_result;
            improved = true;
            continue;
        }
        // Try swap
        new_route = swapNodes(best_local, graph, params);
        new_result = solveSubproblem(new_route, graph, charge_options, params);
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
    std::cout << "Shaking route...\n";
    Route shaken_route = current_route;
    switch (neighborhood) {
        case 0: return twoOpt(shaken_route, graph, params);
        case 1: return relocate(shaken_route, graph, params);
        case 2: return swapNodes(shaken_route, graph, params);
        case 3: {
            if (graph.getStationCopies().empty() || std::uniform_real_distribution<>(0, 1)(rng) < 0.5) {
                return insertStation(shaken_route, graph, params);
            } else {
                return removeStation(shaken_route, graph, params);
            }
        }
        default: return shaken_route;
    }
    std::cout << "Take neighborhood type: " << neighborhood << "\n";
}

Route VNS::swapNodes(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> new_nodes = route.getNodeIds();
    int i = std::uniform_int_distribution<>(1, new_nodes.size() - 2)(rng);
    int j = std::uniform_int_distribution<>(1, new_nodes.size() - 2)(rng);
    while (i == j) {
        j = std::uniform_int_distribution<>(1, new_nodes.size() - 2)(rng);
    }
    std::swap(new_nodes[i], new_nodes[j]);
    Route new_route(new_nodes);
    if (isValidRoute(new_nodes, graph, init_route)) {
        return new_route;
    }
    return route;
}

Route VNS::relocate(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> new_nodes = route.getNodeIds();
    int i = std::uniform_int_distribution<>(1, new_nodes.size() - 2)(rng);
    int j = std::uniform_int_distribution<>(1, new_nodes.size() - 2)(rng);
    while (i == j) {
        j = std::uniform_int_distribution<>(1, new_nodes.size() - 2)(rng);
    }
    int node = new_nodes[i];
    new_nodes.erase(new_nodes.begin() + i);
    new_nodes.insert(new_nodes.begin() + j, node);
    Route new_route(new_nodes);
    if (isValidRoute(new_nodes, graph, init_route)) {
        return new_route;
    }
    return route;
}

Route VNS::insertStation(const Route& route, const Graph& graph, const Parameters& params) {
    const auto& stations = graph.getStationCopies();
    if (stations.empty()) return route;

    std::vector<int> new_nodes = route.getNodeIds();
    std::vector<size_t> valid_positions;
    for (size_t i = 1; i < new_nodes.size() - 1; ++i) {
        if (Utils::isCustomer(new_nodes[i - 1], graph.getNodes()) &&
            Utils::isCustomer(new_nodes[i], graph.getNodes())) {
            valid_positions.push_back(i);
        }
    }
    if (valid_positions.empty()) return route;

    int station = stations[std::uniform_int_distribution<>(0, stations.size() - 1)(rng)];
    size_t pos = valid_positions[std::uniform_int_distribution<>(0, valid_positions.size() - 1)(rng)];
    new_nodes.insert(new_nodes.begin() + pos, station);

    Route new_route(new_nodes);
    if (isValidRoute(new_nodes, graph, init_route)) {
        return new_route;
    }
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
    Route new_route(new_nodes);
    if (isValidRoute(new_nodes, graph, init_route)) {
        return new_route;
    }
    return route;
}

Route VNS::twoOpt(const Route& route, const Graph& graph, const Parameters& params) {
    std::vector<int> new_nodes = route.getNodeIds();
    int i = std::uniform_int_distribution<>(1, new_nodes.size() - 2)(rng);
    int j = std::uniform_int_distribution<>(i + 1, new_nodes.size() - 1)(rng);
    std::reverse(new_nodes.begin() + i, new_nodes.begin() + j);
    Route new_route(new_nodes);
    if (isValidRoute(new_nodes, graph, init_route)) {
        return new_route;
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

// bool VNS::isValidRoute(const std::vector<int>& route_node_ids, const Graph& graph, const Parameters& params) {
//     if (route_node_ids.empty()) return false;
//     if (Utils::isCustomer(route_node_ids[0], graph.getNodes()) ||
//         Utils::isCustomer(route_node_ids.back(), graph.getNodes())) {
//         return false;
//     }
//
//     for (size_t i = 0; i < route_node_ids.size() - 1; ++i) {
//         if (!graph.findNode(route_node_ids[i]) || !graph.findNode(route_node_ids[i + 1])) {
//             return false;
//         }
//         if (!graph.findArc(route_node_ids[i], route_node_ids[i + 1])) {
//             return false;
//         }
//     }
//
//     std::set<int> customers;
//     for (int id : route_node_ids) {
//         if (Utils::isCustomer(id, graph.getNodes())) {
//             customers.insert(id);
//         }
//     }
//     std::set<int> required_customers;
//     for (const auto& node : graph.getNodes()) {
//         if (Utils::isCustomer(node.getId(), graph.getNodes())) {
//             required_customers.insert(node.getId());
//         }
//     }
//     return customers == required_customers;
// }