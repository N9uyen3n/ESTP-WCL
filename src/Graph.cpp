#include "Graph.h"
#include <cmath>

Graph::Graph() {}

void Graph::addNode(const Node &node) {
    nodes.push_back(node);
}

void Graph::addArc(const Arc &arc) {
    arcs.push_back(arc);
}

double Graph::computeDistance(const Node &a, const Node &b) const {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}
