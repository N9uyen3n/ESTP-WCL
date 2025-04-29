#ifndef NEIGHBORHOOD_H
#define NEIGHBORHOOD_H

#include <vector>
#include "Node.h"

enum NeighborhoodType {
    TWO_OPT,
    RELOCATE
};

std::vector<int> generateRandomNeighbor(const std::vector<int>& route, NeighborhoodType type, const std::vector<Node>& nodes);

#endif // NEIGHBORHOOD_H