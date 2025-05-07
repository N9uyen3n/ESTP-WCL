//
// Created by Admin on 07/05/2025.
//

#include "../include/Graph.h"

// Constructor
Graph::Graph(const std::vector<Node>& nodes, const std::vector<Arc>& arcs)
    : nodes(nodes), arcs(arcs) {
}

// Getter for nodes vector
const std::vector<Node>& Graph::getNodes() const {
    return nodes;
}

// Getter for arcs vector
const std::vector<Arc>& Graph::getArcs() const {
    return arcs;
}

// Getter for charging options vector
const std::vector<std::vector<ChargingOption>>& Graph::getChargingOptions() const {
    return charging_options;
}

// Find node by ID
const Node* Graph::findNode(int id) const {
    for (const auto& node : nodes) {
        if (node.getId() == id) {
            return &node;
        }
    }
    return nullptr;
}

// Find arc by from and to node IDs
const Arc* Graph::findArc(int from, int to) const {
    for (const auto& arc : arcs) {
        if (arc.getFrom() == from && arc.getTo() == to) {
            return &arc;
        }
    }
    return nullptr;
}

// Setter for charging options
const void Graph::setChargingOptions(const std::vector<std::vector<ChargingOption>>& options) {
    charging_options = options;
}
Node::Node(int id, std::string string_id, char type, double x, double y, double service_time)
    : id(id),
      string_id(std::move(string_id)),
      type(type == 'd' ? NodeType::DEPOT :
           type == 'f' ? NodeType::CHARGING_STATION :
                        NodeType::CUSTOMER),
      x(x),
      y(y),
      service_time(service_time) {
}

const void Graph::createStationCopies(int num_customers) {
    station_copies.clear();
    for (const auto& node : nodes) {
        if (node.getType() == NodeType::CHARGING_STATION) {
            for (int k = 0; k < num_customers; ++k) {
                int copy_id = nodes.size();
                // Convert NodeType to char: 'f' for CHARGING_STATION
                nodes.emplace_back(copy_id, node.getStringId() + "_copy" + std::to_string(k),
                                 'f', node.getX(), node.getY(), node.getServiceTime());
                station_copies.push_back(copy_id);
            }
        }
    }
}

const std::vector<int>& Graph::getStationCopies() const {
    return station_copies;
}

