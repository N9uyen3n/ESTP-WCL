//
// Created by Admin on 26/04/2025.
//

#ifndef VNSOPTIMIZER_H
#define VNSOPTIMIZER_H

#include <vector>
#include "Node.h"
#include "Arc.h"
#include "ChargingOption.h"
#include "Params.h"
#include "Optimizer.h"
#include "Route.h"
#include "Neighborhood.h"

class VNSOptimizer {
private:
    std::vector<int> S_prime;
    std::vector<Node> nodes;
    std::vector<Arc> arcs;
    std::vector<std::vector<ChargingOption>> charge_options;
    Params params;
    Optimizer optimizer;



public:
    VNSOptimizer( const std::vector<int>& S_prime,
                 const std::vector<Node>& nodes,
                 const std::vector<Arc>& ars,
                 const std::vector<std::vector<ChargingOption>>& chg_opts,
                 const Params& params);

    Route run(int max_iterations = 100);

    void printBestRouteParameters();
};

#endif // VNSOPTIMIZER_H
