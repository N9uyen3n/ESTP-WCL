//
// Created by Admin on 07/05/2025.
//

#include "../include/Route.h"
#include "../include/Graph.h"
#include "../include/Parameters.h"
Route::Route(const std::vector<int>& node_ids)
    : node_ids(node_ids), total_cost(std::numeric_limits<double>::max()), feasible(false) {
    wireless_decisions.resize(node_ids.size() > 1 ? node_ids.size() - 1 : 0, false);
}

bool Route::checkFeasibility(const Graph& graph, const Parameters& params) const {
    if (node_ids.size() < 2) return false;

    double soc = params.getInitialSoc();
    double time = 0.0;
    const double min_soc = params.getMinSoc();
    const double battery_capacity = params.getBatteryCapacity();
    const double energy_consumption = params.getEnergyConsumption();

    for (size_t i = 0; i < node_ids.size() - 1; ++i) {
        const Arc* arc = graph.findArc(node_ids[i], node_ids[i + 1]);
        if (!arc) return false;

        double distance = arc->getDistance();
        double travel_time = arc->getTravelTime();
        double energy_used = distance * energy_consumption;

        if (i < wireless_decisions.size() && wireless_decisions[i] && arc->getIsWireless()) {
            double charge_rate = arc->getWirelessChargeRate();
            soc += charge_rate * travel_time;
            soc = std::min(soc, battery_capacity);
        }

        soc -= energy_used;
        if (soc < min_soc) return false;

        time += travel_time;
        for (const auto& node : graph.getNodes()) {
            if (node.getId() == node_ids[i + 1]) {
                time += node.getServiceTime();
                break;
            }
        }

        for (const auto& decision : charging_decisions) {
            if (decision.getStationId() == node_ids[i + 1]) {
                double charge_time = decision.getChargingTime();
                time += charge_time;
                double charge_rate = 0.0;
                for (const auto& option : graph.getChargingOptions()[node_ids[i + 1]]) {
                    if (option.getOption() == decision.getOption()) {
                        charge_rate = option.getRate();
                        break;
                    }
                }
                soc += charge_rate * charge_time;
                soc = std::min(soc, battery_capacity);
            }
        }
    }

    return soc >= min_soc;
}