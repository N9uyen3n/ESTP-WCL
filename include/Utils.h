#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>
#include <random>
#include "Graph.h"
#include "Parameters.h"
#include "Route.h"

class Utils {
private:
    // Helper methods
    static double calculateDistance(double x1, double y1, double x2, double y2);

public:
    // File reading methods
    static std::vector<Node> readNodes(const std::string& filename);
    static std::vector<Arc> generateArcs(const std::vector<Node>& nodes,
                                        const std::string& wireless_arcs_filename,
                                        const Parameters& params);
    static std::vector<std::vector<ChargingOption>> readChargingOptions(
            const std::vector<Node>& nodes,
            const std::string& filename);
    static Parameters readParams(const std::string& filename);

    // Validation and route generation methods
    static bool validateInitialNodes(const std::vector<int>& initial_nodes, const Graph& graph);
    static std::vector<int> generateInitialRoute(const std::vector<Node>& nodes,
                                                const std::vector<Arc>& arcs,
                                                const Parameters& params,
                                                std::mt19937& gen);

    // Node utility methods
    static std::string getNodeType(int id, const std::vector<Node>& nodes);
    static bool isChargingStation(int id, const std::vector<Node>& nodes);
    static bool isCustomer(int id, const std::vector<Node>& nodes);

    // Print methods
    static void printNodesInfo(const std::vector<Node>& nodes);
    static void printArcsInfo(const std::vector<Arc>& arcs);
    static void printChargingOptionsInfo(const std::vector<std::vector<ChargingOption>>& options);
    static void printParamsInfo(const Parameters& params);

};

#endif // UTILS_H