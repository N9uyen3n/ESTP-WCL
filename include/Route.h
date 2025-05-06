    //
    // Created by Admin on 23/04/2025.
    //

    #ifndef ROUTE_H
    #define ROUTE_H

    #include <vector>
    #include "Node.h"
    #include "Arc.h"
    #include "ChargingOption.h"
    #include "Params.h"
    #include "Optimizer.h"

    class Route {
    private:
        std::vector<int> node_ids;
        double total_cost;
        bool is_feasible;

    public:
        // Constructor
        Route(const std::vector<int>& initial_nodes);

        // Getter
        std::vector<int> getNodeIds() const { return node_ids; }
        double getTotalCost() const { return total_cost; }
        bool getIsFeasible() const { return is_feasible; }

        // Setter
        void setNodeIds(const std::vector<int>& nodes) { node_ids = nodes; }
        void setTotalCost(double cost) { total_cost = cost; }
        void setIsFeasible(bool feasible) { is_feasible = feasible; }

        int countChargingStations(const std::vector<int>& route, const std::vector<Node>& nodes);

        // Tối ưu hóa tuyến đường bằng MILP
        double optimize(const std::vector<Arc>& arcs,
                        const std::vector<std::vector<ChargingOption>>& charge_options,
                        const Params& params,
                        const std::vector<Node>& nodes,
                        Optimizer& optimizer);

        // In tuyến đường
        void print() const;

    };

    #endif // ROUTE_H


