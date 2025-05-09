//
// Created by Admin on 07/05/2025.
//

#include "../include/Graph.h"

// Constructor
Graph::Graph(const std::vector<Node>& nodes, const std::vector<Arc>& arcs)
    : nodes(nodes), arcs(arcs) {
    for (int i = 0; i < nodes.size(); ++i) {
        if (nodes[i].getType() == NodeType::CHARGING_STATION) {
            id_stations.push_back(i);
        }
    }
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
// Getter for station
const std::vector<int>&  Graph::getStations() const {
    return id_stations;
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





