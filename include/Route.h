#ifndef ROUTE_H
#define ROUTE_H
#include "Graph.h"
#include <vector>

#include "Utils.h"

struct ChargingDecision {
    int station_id, option_index;
    double charging_time;
    ChargingDecision(int sid, int opt, double time) : station_id(sid), option_index(opt), charging_time(time) {}
    int getStationId() const { return station_id; }
    int getOption() const { return option_index; }
    double getChargingTime() const { return charging_time; }
};

class Route {
    std::vector<int> node_ids;
    std::vector<ChargingDecision> charging_decisions;
    std::vector<bool> wireless_decisions;
    std::vector<std::pair<int, int>> arc_wireless_decisions;
    std::vector<double> soc_arrival;
    std::vector<double> soc_departure;
    std::vector<double> arrival_time;
    std::vector<double> departure_time;
    double total_cost;
    bool feasible;

public:
    Route(const std::vector<int>& nodes);
    // Getters
    const std::vector<int>& getNodeIds() const { return node_ids; }
    const std::vector<ChargingDecision>& getChargingDecisions() const { return charging_decisions; }
    const std::vector<bool>& getWirelessDecisions() const { return wireless_decisions; }
    const std::vector<std::pair<int, int>>& getArcWirelessDecisions() const { return arc_wireless_decisions; }
    const std::vector<double>& getSocArrival() const { return soc_arrival; }
    const std::vector<double>& getSocDeparture() const { return soc_departure; }
    const std::vector<double>& getArrivalTime() const { return arrival_time; }
    const std::vector<double>& getDepartureTime() const { return departure_time; }
    double getTotalCost() const { return total_cost; }
    bool isFeasible() const { return feasible; }
    // Setters
    void setNodes(const std::vector<int>& nodes) { node_ids = nodes; }
    void setChargingDecisions(const std::vector<ChargingDecision>& decisions) { charging_decisions = decisions; }
    void setWirelessDecisions(const std::vector<bool>& decisions) { wireless_decisions = decisions; }
    void setArcWirelessDecisions(const std::vector<std::pair<int, int>>& decisions) { arc_wireless_decisions = decisions; }
    void setSocArrival(const std::vector<double>& soc) { soc_arrival = soc; }
    void setSocDeparture(const std::vector<double>& soc) { soc_departure = soc; }
    void setArrivalTime(const std::vector<double>& times) { arrival_time = times; }
    void setDepartureTime(const std::vector<double>& times) { departure_time = times; }
    void setTotalCost(double cost) { total_cost = cost; }
    void setFeasible(bool f) { feasible = f; }

    // void deleteChargingDecisions(const std::vector<Node>& nodes) {
    //     for (const auto& node: node_ids) {
    //         if (Utils::isChargingStation(node, nodes)) {
    //             node_ids.erase(std::remove(node_ids.begin(), node_ids.end(), node));
    //         }
    //         charging_decisions.clear();
    //     }
    // }
    bool checkFeasibility(const Graph& graph, const Parameters& params) const;


};

#endif