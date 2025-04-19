// CSVReader.cpp
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
        throw std::runtime_error("Không thể mở file: " + filename);
    }
    std::string line;
    std::getline(file, line); // Bỏ qua tiêu đề
    int id = 0;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        Node node;
        std::getline(ss, cell, ','); node.string_id = cell;
        std::getline(ss, cell, ','); node.type = cell;
        std::getline(ss, cell, ','); node.x = std::stod(cell);
        std::getline(ss, cell, ','); node.y = std::stod(cell);
        std::getline(ss, cell, ','); node.demand = std::stod(cell);
        std::getline(ss, cell, ','); node.ready_time = std::stod(cell);
        std::getline(ss, cell, ','); node.due_date = std::stod(cell);
        std::getline(ss, cell, ','); node.service_time = std::stod(cell);
        node.id = id++;
        nodes.push_back(node);
    }
    file.close();

    // Tạo bản sao trạm sạc (F')
    std::vector<Node> station_copies;
    int m = 10;
    for (const auto& node : nodes) {
        if (node.type == "Station") {
            for (int k = 1; k <= m; ++k) {
                Node copy = node;
                copy.id = id++;
                copy.string_id = node.string_id + "_" + std::to_string(k);
                station_copies.push_back(copy);
            }
        }
    }
    nodes.insert(nodes.end(), station_copies.begin(), station_copies.end());

    // Thêm depot kết thúc (n+1)
    for (const auto& node : nodes) {
        if (node.string_id == "D0") {
            Node depot_end = node;
            depot_end.id = id++;
            depot_end.string_id = "D1";
            nodes.push_back(depot_end);
            break;
        }
    }

    return nodes;
}

std::vector<Arc> CSVReader::generateArcs(const std::vector<Node>& nodes, const std::string& wireless_arcs_filename, const Params& params) {
    std::map<std::string, int> string_id_to_id;
    for (const auto& node : nodes) {
        string_id_to_id[node.string_id] = node.id;
    }

    double v = params.v;
    std::vector<Arc> arcs;
    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t j = 0; j < nodes.size(); ++j) {
            if (i == j) continue;
            Arc arc;
            arc.from = nodes[i].id;
            arc.to = nodes[j].id;
            double dx = nodes[j].x - nodes[i].x;
            double dy = nodes[j].y - nodes[i].y;
            arc.dij = std::sqrt(dx * dx + dy * dy);
            arc.sij = arc.dij / v;
            arc.is_wireless = false;
            arc.beta_ij = 0.0;
            arc.U_min = v;
            arc.U_max = v;
            arcs.push_back(arc);
        }
    }

    std::ifstream wfile(wireless_arcs_filename);
    if (!wfile.is_open()) {
        throw std::runtime_error("Không thể mở file: " + wireless_arcs_filename);
    }
    std::string line;
    std::getline(wfile, line);
    while (std::getline(wfile, line)) {
        std::stringstream ss(line);
        std::string from_str, to_str, cell;
        double beta;
        std::getline(ss, from_str, ',');
        std::getline(ss, to_str, ',');
        std::getline(ss, cell, ',');
        beta = std::stod(cell);
        if (string_id_to_id.find(from_str) == string_id_to_id.end() ||
            string_id_to_id.find(to_str) == string_id_to_id.end()) {
            continue;
        }
        int from_id = string_id_to_id[from_str];
        int to_id = string_id_to_id[to_str];
        for (auto& arc : arcs) {
            if (arc.from == from_id && arc.to == to_id) {
                arc.is_wireless = true;
                arc.beta_ij = beta;
                arc.U_min = v * 0.5;
                arc.U_max = v * 1.5;
            }
        }
    }
    wfile.close();
    return arcs;
}

std::vector<std::vector<ChargingOption>> CSVReader::readChargingOptions(const std::vector<Node>& nodes, const std::string& filename) {
    std::map<std::string, int> string_id_to_id;
    for (const auto& node : nodes) {
        string_id_to_id[node.string_id] = node.id;
    }

    std::vector<std::vector<ChargingOption>> charge_options(nodes.size());
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Không thể mở file: " + filename);
    }
    std::string line;
    std::getline(file, line);
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell, station_str;
        ChargingOption option;
        std::getline(ss, station_str, ',');
        std::getline(ss, cell, ','); option.option = std::stoi(cell);
        std::getline(ss, cell, ','); option.rate = std::stod(cell);
        std::getline(ss, cell, ','); option.cost = std::stod(cell);
        for (const auto& node : nodes) {
            if (node.string_id == station_str || node.string_id.find(station_str + "_") == 0) {
                option.station_id = node.id;
                charge_options[option.station_id].push_back(option);
            }
        }
    }
    file.close();

    for (const auto& node : nodes) {
        if (node.type == "Station" || node.string_id.find("_") != std::string::npos) {
            ChargingOption no_charge;
            no_charge.station_id = node.id;
            no_charge.option = 0;
            no_charge.rate = 0.0;
            no_charge.cost = 0.0;
            charge_options[node.id].push_back(no_charge);
        }
    }

    return charge_options;
}

Params CSVReader::readParams(const std::string& filename) {
    Params params;
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Không thể mở file: " + filename);
    }
    std::string line;
    std::getline(file, line);
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::getline(ss, cell, ','); params.Q = std::stod(cell);
        std::getline(ss, cell, ','); params.h = std::stod(cell);
        std::getline(ss, cell, ','); params.v = std::stod(cell);
        std::getline(ss, cell, ','); params.cw = std::stod(cell);
        std::getline(ss, cell, ','); params.ct = std::stod(cell);
        // std::getline(ss, cell, ','); params.minSOC = std::stod(cell);
        params.minSOC = 0.1 * params.Q;
        params.initial_SOC = params.Q;
        params.M = 1e6;
    }
    file.close();
    return params;
}