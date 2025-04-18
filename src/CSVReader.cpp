//
// Created by Admin on 16/04/2025.
//

#include "CSVReader.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <stdexcept>
#include <map>

std::vector<Node> CSVReader::readNodes(const std::string& filename) {
    std::vector<Node> nodes;
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }
    std::string line;
    std::getline(file, line); // Skip header
    int id = 0;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        Node node;
        std::getline(ss, cell, ','); // StringID
        std::getline(ss, cell, ','); node.type = cell;
        std::getline(ss, cell, ','); node.x = std::stod(cell);
        std::getline(ss, cell, ','); node.y = std::stod(cell);
        std::getline(ss, cell, ','); node.service_time = std::stod(cell);
        node.demand = 0; // Default
        node.ready_time = 0;
        node.due_date = 0;
        node.id = id++;
        nodes.push_back(node);
    }
    file.close();
    return nodes;
}

std::vector<Arc> CSVReader::generateArcs(const std::vector<Node>& nodes, const std::string& wireless_arcs_filename) {
    // Create StringID to id mapping
    std::map<std::string, int> string_id_to_id;
    for (const auto& node : nodes) {
        string_id_to_id[node.type == "d" ? "D0" : node.type == "f" ? "S" + std::to_string(node.id - 1) : "C" + std::to_string(node.id - 5)] = node.id;
    }

    std::vector<Arc> arcs;
    // Generate all possible directed arcs
    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t j = 0; j < nodes.size(); ++j) {
            if (i == j) continue;
            Arc arc;
            arc.from = nodes[i].id;
            arc.to = nodes[j].id;
            double dx = nodes[j].x - nodes[i].x;
            double dy = nodes[j].y - nodes[i].y;
            arc.dij = std::sqrt(dx * dx + dy * dy);
            arc.is_wireless = false;
            arc.beta_ij = 0.0;
            arc.U_min = 0.0;
            arc.U_max = 0.0;
            arcs.push_back(arc);
        }
    }

    // Read wireless arcs and mark them
    std::ifstream wfile(wireless_arcs_filename);
    if (!wfile.is_open()) {
        throw std::runtime_error("Could not open file: " + wireless_arcs_filename);
    }
    std::string line;
    std::getline(wfile, line); // Skip header
    while (std::getline(wfile, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::string from_str, to_str;
        double beta;
        std::getline(ss, from_str, ','); // From (StringID)
        std::getline(ss, to_str, ','); // To (StringID)
        std::getline(ss, cell, ','); beta = std::stod(cell); // Beta
        // Map StringID to id
        int from_id = string_id_to_id.at(from_str);
        int to_id = string_id_to_id.at(to_str);
        // Mark arcs
        for (auto& arc : arcs) {
            if ((arc.from == from_id && arc.to == to_id) || (arc.from == to_id && arc.to == from_id)) {
                arc.is_wireless = true;
                arc.beta_ij = beta;
            }
        }
    }
    wfile.close();
    return arcs;
}

std::vector<std::vector<ChargingOption>> CSVReader::readChargingOptions(const std::vector<Node>& nodes, const std::string& filename) {
    // Create StringID to id mapping
    std::map<std::string, int> string_id_to_id;
    for (const auto& node : nodes) {
        string_id_to_id[node.type == "d" ? "D0" : node.type == "f" ? "S" + std::to_string(node.id - 1) : "C" + std::to_string(node.id - 5)] = node.id;
    }

    std::vector<std::vector<ChargingOption>> charge_options(nodes.size());
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }
    std::string line;
    std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        ChargingOption option;
        std::string station_str;
        std::getline(ss, station_str, ','); // StationID (StringID)
        std::getline(ss, cell, ','); option.option = std::stoi(cell); // Option
        std::getline(ss, cell, ','); option.rate = std::stod(cell); // Rate
        std::getline(ss, cell, ','); option.cost = std::stod(cell); // Cost
        option.station_id = string_id_to_id.at(station_str); // Map StringID to id
        charge_options[option.station_id].push_back(option);
    }
    file.close();
    return charge_options;
}

Params CSVReader::readParams(const std::string& filename) {
    Params params;
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }
    std::string line;
    std::getline(file, line); // Skip header
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::getline(ss, cell, ','); params.Q = std::stod(cell); // capacity
        std::getline(ss, cell, ','); params.h = std::stod(cell); // rate
        std::getline(ss, cell, ','); params.v = std::stod(cell); // Velocity
        std::getline(ss, cell, ','); params.cw = std::stod(cell); // c_w
        std::getline(ss, cell, ','); params.ct = std::stod(cell); // c_t
        params.minSOC = 0.1 * params.Q;
        params.initial_SOC = params.Q;
    }
    file.close();
    return params;
}