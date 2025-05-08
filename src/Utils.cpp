#include "../include/Utils.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <map>
#include <algorithm>
#include <random>
#include <set>

std::vector<Node> Utils::readNodes(const std::string& filename) {
    std::vector<Node> nodes;
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }

    std::string line;
    std::getline(file, line); // Skip header
    int id = 0;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string string_id, type_str, cell;
        double x, y, service_time;

        std::getline(ss, string_id, ',');
        std::getline(ss, type_str, ',');
        std::getline(ss, cell, ','); x = std::stod(cell);
        std::getline(ss, cell, ','); y = std::stod(cell);
        std::getline(ss, cell, ','); service_time = std::stod(cell);

        char type_char = type_str.empty() ? 'c' : std::tolower(type_str[0]);
        Node node(id, string_id, type_char, x, y, service_time);
        node.setId(id++);
        nodes.push_back(node);
    }
    file.close();


    // Add end depot (D1)
    for (const auto& node : nodes) {
        if (node.getStringId() == "D0") {
            Node depot_end = node;
            depot_end.setId(id++);
            depot_end.setStringId("D1");
            nodes.push_back(depot_end);
            break;
        }
    }

    return nodes;
}

bool Utils::validateInitialNodes(const std::vector<int>& nodes, const Graph& graph) {
    std::set<int> customer_ids;
    for (const auto& node : graph.getNodes()) {
        if (node.getType() == NodeType::CUSTOMER) {
            customer_ids.insert(node.getId());
        }
    }
    std::set<int> visited_customers;
    for (int id : nodes) {
        if (customer_ids.count(id)) {
            if (visited_customers.count(id)) {
                return false; // Khách hàng xuất hiện nhiều lần
            }
            visited_customers.insert(id);
        }
    }
    return visited_customers.size() == customer_ids.size(); // Đảm bảo thăm đủ khách hàng
}


std::vector<Arc> Utils::generateArcs(const std::vector<Node>& nodes,
                                    const std::string& wireless_arcs_filename,
                                    const Parameters& params) {
    std::map<std::string, int> string_id_to_id;
    for (const auto& node : nodes) {
        string_id_to_id[node.getStringId()] = node.getId();
    }

    std::vector<Arc> arcs;
    double v = params.getVehicleSpeed();
    for (const auto& from : nodes) {
        for (const auto& to : nodes) {
            if (from.getId() != to.getId()) {
                double dx = to.getX() - from.getX();
                double dy = to.getY() - from.getY();
                double distance = std::sqrt(dx * dx + dy * dy);
                double travel_time = distance / v;

                Arc arc(from.getId(), to.getId(), distance, travel_time);
                arcs.push_back(arc);
            }
        }
    }

    std::ifstream file(wireless_arcs_filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + wireless_arcs_filename);
    }

    std::string line;
    std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string from_id, to_id, cell;
        double beta;

        std::getline(ss, from_id, ',');
        std::getline(ss, to_id, ',');
        std::getline(ss, cell, ','); beta = std::stod(cell);

        if (string_id_to_id.find(from_id) == string_id_to_id.end() ||
            string_id_to_id.find(to_id) == string_id_to_id.end()) {
            continue;
        }

        int from_index = string_id_to_id[from_id];
        int to_index = string_id_to_id[to_id];
        for (auto& arc : arcs) {
            if (arc.getFrom() == from_index && arc.getTo() == to_index) {
                arc.setIsWireless(true);
                arc.setWirelessChargeRate(beta);
                break;
            }
        }
    }
    file.close();
    return arcs;
}

std::vector<std::vector<ChargingOption>> Utils::readChargingOptions(
        const std::vector<Node>& nodes, const std::string& filename) {
    std::vector<std::vector<ChargingOption>> charging_options(nodes.size());
    std::map<std::string, int> string_id_to_id;
    for (const auto& node : nodes) {
        string_id_to_id[node.getStringId()] = node.getId();
    }

    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }

    std::string line;
    std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string station_id, cell;
        int option;
        double rate, cost;

        std::getline(ss, station_id, ',');
        std::getline(ss, cell, ','); option = std::stoi(cell);
        std::getline(ss, cell, ','); rate = std::stod(cell);
        std::getline(ss, cell, ','); cost = std::stod(cell);

        for (const auto& node : nodes) {
            if (node.getStringId() == station_id || node.getStringId().find(station_id + "_") == 0) {
                ChargingOption opt(cost, rate);
                opt.setStationId(node.getId());
                opt.setOption(option);
                charging_options[node.getId()].push_back(opt);
            }
        }
    }
    file.close();

    // Add no-charging option for charging stations
    for (const auto& node : nodes) {
        if (node.getType() == NodeType::CHARGING_STATION || node.getStringId().find("_") != std::string::npos) {
            ChargingOption no_charge(0.0, 0.0);
            no_charge.setStationId(node.getId());
            no_charge.setOption(0);
            charging_options[node.getId()].push_back(no_charge);
        }
    }

    return charging_options;
}

Parameters Utils::readParams(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }

    std::string line;
    std::getline(file, line); // Skip header
    if (!std::getline(file, line)) {
        file.close();
        throw std::runtime_error("Empty parameters file: " + filename);
    }

    std::stringstream ss(line);
    double battery_capacity, energy_consumption, vehicle_speed, wireless_cost, time_cost;
    std::string cell;

    std::getline(ss, cell, ','); battery_capacity = std::stod(cell);
    std::getline(ss, cell, ','); energy_consumption = std::stod(cell);
    std::getline(ss, cell, ','); vehicle_speed = std::stod(cell);
    std::getline(ss, cell, ','); wireless_cost = std::stod(cell);
    std::getline(ss, cell, ','); time_cost = std::stod(cell);

    Parameters params(battery_capacity, 0.1 * battery_capacity, energy_consumption,
                     vehicle_speed, wireless_cost, time_cost,
                     0.3 * battery_capacity, 1e6);

    file.close();
    return params;
}

std::vector<int> Utils::generateInitialRoute(const std::vector<Node>& nodes,
                                            const std::vector<Arc>& arcs,
                                            const Parameters& params,
                                            std::mt19937& gen) {
    std::vector<int> customers;
    for (const auto& node : nodes) {
        if (node.getType() == NodeType::CUSTOMER) {
            customers.push_back(node.getId());
        }
    }
    if (customers.empty()) {
        std::cerr << "No customers found in nodes.\n";
        return {};
    }

    std::shuffle(customers.begin(), customers.end(), gen);

    int start_depot_id = -1, end_depot_id = -1;
    for (const auto& node : nodes) {
        if (node.getType() == NodeType::DEPOT) {
            if (node.getStringId() == "D0") {
                start_depot_id = node.getId();
            } else if (node.getStringId() == "D1") {
                end_depot_id = node.getId();
            }
        }
    }
    if (start_depot_id == -1 || end_depot_id == -1) {
        std::cerr << "Depot D0 or D1 not found.\n";
        return {};
    }

    std::vector<int> route = {start_depot_id};
    route.insert(route.end(), customers.begin(), customers.end());
    route.push_back(end_depot_id);

    return route;
}

std::string Utils::getNodeType(int id, const std::vector<Node>& nodes) {
    for (const auto& node : nodes) {
        if (node.getId() == id) {
            switch (node.getType()) {
                case NodeType::DEPOT: return "Depot";
                case NodeType::CUSTOMER: return "Customer";
                case NodeType::CHARGING_STATION: return "Station";
                default: return "Unknown";
            }
        }
    }
    return "Unknown";
}

bool Utils::isCustomer(int id, const std::vector<Node> &nodes) {
    for (const auto& node : nodes) {
        if (node.getId() == id && node.getType() == NodeType::CUSTOMER) {
            // std::cout<<"Node ID: " << id << " is a customer.\n";
            return true;
        }
    }
    // std::cout<<"Node ID: " << id << " is not a customer.\n";
    return false;
}


bool Utils::isChargingStation(int id, const std::vector<Node>& nodes) {
    for (const auto& node : nodes) {
        if (node.getId() == id && node.getType() == NodeType::CHARGING_STATION) {
            return true;
        }
    }
    return false;
}

void Utils::printNodesInfo(const std::vector<Node>& nodes) {
    std::cout << "Nodes Information:\n";
    for (const auto& node : nodes) {
        std::cout << "  ID: " << node.getId()
                  << ", StringID: " << node.getStringId()
                  << ", Type: " << getNodeType(node.getId(), nodes)
                  << ", Coordinates: (" << std::fixed << std::setprecision(2) << node.getX() << ", " << node.getY() << ")"
                  << ", Service Time: " << node.getServiceTime() << "\n";
    }
}

void Utils::printArcsInfo(const std::vector<Arc>& arcs) {
    std::cout << "Arcs Information:\n";
    for (const auto& arc : arcs) {
        std::cout << "  From: " << arc.getFrom()
                  << ", To: " << arc.getTo()
                  << ", Distance: " << std::fixed << std::setprecision(2) << arc.getDistance()
                  << ", Travel Time: " << arc.getTravelTime()
                  << ", Wireless: " << (arc.getIsWireless() ? "Yes" : "No")
                  << ", Charge Rate: " << arc.getWirelessChargeRate() << "\n";
    }
}

void Utils::printChargingOptionsInfo(const std::vector<std::vector<ChargingOption>>& options) {
    std::cout << "Charging Options Information:\n";
    for (size_t i = 0; i < options.size(); ++i) {
        if (!options[i].empty()) {
            std::cout << "  Node ID: " << i << "\n";
            for (size_t j = 0; j < options[i].size(); ++j) {
                std::cout << "    Option " << options[i][j].getOption()
                          << ": Cost = " << std::fixed << std::setprecision(2) << options[i][j].getCost()
                          << ", Rate = " << options[i][j].getRate() << "\n";
            }
        }
    }
}

void Utils::printParamsInfo(const Parameters& params) {
    std::cout << "Parameters Information:\n";
    std::cout << "  Battery Capacity: " << params.getBatteryCapacity() << "\n";
    std::cout << "  Min SOC: " << params.getMinSoc() << "\n";
    std::cout << "  Energy Consumption: " << params.getEnergyConsumption() << "\n";
    std::cout << "  Vehicle Speed: " << params.getVehicleSpeed() << "\n";
    std::cout << "  Wireless Cost: " << params.getWirelessCost() << "\n";
    std::cout << "  Time Cost: " << params.getTimeCost() << "\n";
    std::cout << "  Initial SOC: " << params.getInitialSoc() << "\n";
    std::cout << "  Big-M: " << params.getBigM() << "\n";
    std::cout << "  U_max: " << params.getUmax() << "\n";
    std::cout << "  U_min: " << params.getUmin() << "\n";
}