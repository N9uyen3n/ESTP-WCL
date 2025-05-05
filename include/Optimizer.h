//
// Created by Admin on 17/04/2025.
//

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Arc.h"
#include "ChargingOption.h"
#include "Params.h"
#include "Node.h"
#include "Utils.h"
#include <vector>

class Optimizer {
public:
    double optimize(
        const std::vector<int>& S_prime,
        const std::vector<Arc>& arcs,
        const std::vector<std::vector<ChargingOption>>& charge_options,
        const Params& params,
        const std::vector<Node>& nodes
    );
};

#endif // OPTIMIZER_H