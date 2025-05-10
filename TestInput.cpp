#include <iostream>
#include <iomanip>
#include "include/Utils.h"


int main() {
    try {
        // Read nodes
        std::string csv_file_dir = "../data/Input/";
        std::vector<Node> nodes = Utils::readNodes(csv_file_dir + "nodes.csv");
        Utils::printNodesInfo(nodes);  // Sử dụng printNodesInfo thay vì printArcsInfo

        // Read parameters
        Parameters params = Utils::readParams(csv_file_dir + "params.csv");

        // Read arcs and generate complete graph
        std::vector<Arc> arcs = Utils::generateArcs(nodes,
            csv_file_dir + "wireless_arcs.csv", params);
        Utils::printArcsInfo(arcs);    // printArcsInfo dùng cho vector của Arc

        // Read charging options
        auto charging_options =
            Utils::readChargingOptions(nodes, csv_file_dir + "charging_options.csv");
        Utils::printChargingOptionsInfo(charging_options);

        // Print parameters
        Utils::printParamsInfo(params);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}