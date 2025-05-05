//
// Created by Admin on 26/04/2025.
//

#include "Arc.h"
#include "Node.h"
#include "Params.h"
#include <random>
#include <vector>


#ifndef UTILS_H
#define UTILS_H


struct ModelParameters {
    std::vector<std::vector<bool>> x; // x_ij: Arc (i,j) is used
    std::vector<double> phi; // phi_i: Charging time at node i
    std::vector<std::vector<bool>> w; // w_ik: Charging option k selected at node i
    std::vector<std::vector<bool>> z; // z_ij: Wireless charging on arc (i,j)
    std::vector<std::vector<double>> s; // s_ij: Travel time on arc (i,j)
    std::vector<double> ya; // ya_i: State of charge (SOC) upon arrival at node i
    std::vector<double> yd; // yd_i: State of charge (SOC) upon departure from node i
    std::vector<double> t; // t_i: Arrival time at node i
};



bool isCustomer(int id, const std::vector<Node>& nodes);
bool can_reach(int i, int j, double SOC, const std::vector<std::vector<Arc>>& arc_matrix, const Params& params);
double update_SOC(int i, int j, double SOC, const std::vector<std::vector<Arc>>& arc_matrix, const Params& params);
// Declaration with default value for insert_prob
std::vector<int> generateInitialRoute(const std::vector<Node>& nodes,
                                      const std::vector<Arc>& arcs,
                                      const Params& params,
                                      std::mt19937& gen,
                                      double insert_prob = 0.2);
bool isChargingStation(int nodeId, const std::vector<Node>& nodes);


#endif //UTILS_H
