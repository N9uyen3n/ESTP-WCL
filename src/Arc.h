//
// Created by Admin on 16/04/2025.
//

#ifndef ARC_H
#define ARC_H

class Arc {
public:
    int from, to;
    double dij;
    bool is_wireless;
    double beta_ij;
    double U_min, U_max;
};

#endif // ETSP_WCL_OPTIMIZATION_ARC_H
