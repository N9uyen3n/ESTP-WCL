//
// Created by Admin on 26/04/2025.
//

#include "Arc.h"
#include "Node.h"
#include "Params.h"
#include "ChargingOption.h"
#include <random>
#include <vector>


#ifndef UTILS_H
#define UTILS_H


struct ModelParameters {
    std::vector<double> phi;
    std::vector<std::vector<int>> w;
    std::vector<double> s;
    std::vector<double> ya;
    std::vector<double> yd;
    std::vector<double> t;
    std::vector<double> depart;
    std::vector<int> z;
    std::vector<double> w_s_z;
};

void printModelParameters(const ModelParameters& params);
void clearModelParameters(ModelParameters& params);
void printNodesInfo(const std::vector<Node>& nodes);
// printArcsInfo function
void printArcsInfo(const std::vector<Arc>& arcs);
// printChargingOptionsInfo function
void printChargingOptionsInfo(const std::vector<std::vector<ChargingOption>>& options);
// printParamsInfo function
void printParamsInfo(const Params& params);
// printModelParameters function prints the model parameters
void printModelParameters(const ModelParameters& params, const std::vector<int>& route);
bool isCustomer(int id, const std::vector<Node>& nodes);
bool can_reach(int i, int j, double SOC, const std::vector<std::vector<Arc>>& arc_matrix, const Params& params);
double update_SOC(int i, int j, double SOC, const std::vector<std::vector<Arc>>& arc_matrix, const Params& params);
// Declaration with default value for insert_prob
std::vector<int> generateInitialRoute(const std::vector<Node>& nodes,
                                      const std::vector<Arc>& arcs,
                                      const Params& params,
                                      std::mt19937& gen,
                                      double insert_prob = 0.2);



#endif //UTILS_H
