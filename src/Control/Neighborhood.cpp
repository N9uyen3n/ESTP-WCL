//
// Created by Admin on 29/04/2025.
//

#include "../../include/Neighborhood.h"
#include "../../include/Utils.h"
#include <random>
#include <algorithm>

std::vector<int> generateRandomNeighbor(const std::vector<int>& route, NeighborhoodType type, const std::vector<Node>& nodes) {
    int n = route.size();
    if (n < 4) return route; // Can't perform any operation on a route with less than 4 nodes

    std::random_device rd;
    std::mt19937 gen(rd());

    if (type == TWO_OPT) {
        // Select two random positions p and q
        int max_p = n - 3;
        std::uniform_int_distribution<> dis_p(0, max_p);
        int p = dis_p(gen);
        std::uniform_int_distribution<> dis_q(p + 2, n - 1);
        int q = dis_q(gen);
        std::vector<int> new_route = route;
        std::reverse(new_route.begin() + p + 1, new_route.begin() + q + 1);
        return new_route;
    } else if (type == RELOCATE) {
        // find all customer positions
        std::vector<int> customer_positions;
        for (int pos = 1; pos < n - 1; ++pos) {
            //check if the node is a customer
            if (isCustomer(route[pos], nodes)) {
                customer_positions.push_back(pos);
            }
        }
        if (customer_positions.empty()) return route;

        // Select a random customer position i
        std::uniform_int_distribution<> dis_cust(0, customer_positions.size() - 1);
        int idx = dis_cust(gen);
        int i = customer_positions[idx];
        int c = route[i];

        // Select a random position j to relocate c
        std::vector<int> new_route;
        new_route.reserve(n - 1);
        new_route.insert(new_route.end(), route.begin(), route.begin() + i);
        new_route.insert(new_route.end(), route.begin() + i + 1, route.end());

        // Select a random position j
        int new_n = new_route.size();
        std::uniform_int_distribution<> dis_j(1, new_n - 1);
        int j = dis_j(gen);

        // Insert c at position j
        new_route.insert(new_route.begin() + j, c);
        return new_route;
    }
    return route;
}
