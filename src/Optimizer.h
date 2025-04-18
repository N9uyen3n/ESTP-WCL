//
// Created by Admin on 16/04/2025.
//

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <vector>
#include "Node.h"
#include "Arc.h"
#include "ChargingOption.h"
#include "Params.h"

class Optimizer {
public:
    double optimize(const std::vector<int>& S_prime, const std::vector<Arc>& arcs,
                    const std::vector<std::vector<ChargingOption>>& charge_options, const Params& params);
};

#endif // ETSP_WCL_OPTIMIZATION_OPTIMIZER_H
