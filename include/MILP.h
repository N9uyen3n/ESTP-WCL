#ifndef MILP_H
#define MILP_H

#include "Optimizer.h"
#include <ilcplex/ilocplex.h>




class MILP {
public:
    MILP();
    ~MILP();

    Route optimize(const std::vector<int>& initial_nodes,
                   const Graph& graph,
                   const std::vector<std::vector<ChargingOption>>& charge_options,
                   const Parameters& params) ;

    Route solveSubproblem(int i, int j, int a, const Route& current_route,
                         const Graph& graph,
                         const std::vector<std::vector<ChargingOption>>& charge_options,
                         const Parameters& params) ;

private:
    IloEnv env;

    void updateRoute(Route& route, const SubproblemResult& result);
};

#endif // MILP_H