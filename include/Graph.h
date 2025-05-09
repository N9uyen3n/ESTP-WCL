#ifndef GRAPH_H
#define GRAPH_H

#include "Parameters.h"
#include <vector>
#include <string>

enum class NodeType { DEPOT, CUSTOMER, CHARGING_STATION };

struct Node {
private:
    int id;
    std::string string_id;
    NodeType type;
    double x, y;
    double service_time;


public:
    Node(int id, std::string string_id, char type, double x, double y, double service_time);

    // Getters
    int getId() const { return id; }
    std::string getStringId() const { return string_id; }
    NodeType getType() const { return type; }
    double getX() const { return x; }
    double getY() const { return y; }
    double getServiceTime() const { return service_time; }

    // Setters
    void setId(int new_id) { id = new_id; }
    void setStringId(const std::string& new_string_id) { string_id = new_string_id; }
};

struct Arc {
private:
    int from;
    int to;
    double distance;
    double travel_time;
    bool is_wireless;
    double wireless_charge_rate;

public:
    Arc(int from, int to, double distance, double travel_time)
        : from(from), to(to), distance(distance), travel_time(travel_time),
          is_wireless(false), wireless_charge_rate(0.0) {}

    // Getters
    int getFrom() const { return from; }
    int getTo() const { return to; }
    double getDistance() const { return distance; }
    double getTravelTime() const { return travel_time; }
    bool getIsWireless() const { return is_wireless; }
    double getWirelessChargeRate() const { return wireless_charge_rate; }

    // Setters
    void setIsWireless(bool wireless) { is_wireless = wireless; }
    void setWirelessChargeRate(double rate) { wireless_charge_rate = rate; }
};

class Graph {
    std::vector<Node> nodes; // Lưu trữ các nút (Depot, Station, Customer)
    std::vector<Arc> arcs; // Lưu trữ các cung
    std::vector<std::vector<ChargingOption>> charging_options;
    std::vector<int> id_stations; // Lưu trữ các trạm sạc
public:
    Graph(const std::vector<Node>& nodes, const std::vector<Arc>& arcs);
    const std::vector<Node>& getNodes() const;
    const std::vector<Arc>& getArcs() const;
    const std::vector<int>& getStations() const; // Lấy id danh sách các trạm sạc (để quá trình VNS biết trạm sạc mang id gì).
    const std::vector<std::vector<ChargingOption>>& getChargingOptions() const;
    const Node* findNode(int id) const;
    const Arc* findArc(int from, int to) const;
    const void setChargingOptions(const std::vector<std::vector<ChargingOption>>& options);

    const std::vector<int>& getStationCopies() const;

};

#endif