//
// Created by Admin on 16/04/2025.
//

#ifndef CSVREADER_H
#define CSVREADER_H

#include <vector>
#include <string>
#include "Node.h"
#include "Arc.h"
#include "ChargingOption.h"
#include "Params.h"

class CSVReader {
public:
    static std::vector<Node> readNodes(const std::string& filename);
    static std::vector<Arc> generateArcs(const std::vector<Node>& nodes, const std::string& wireless_arcs_filename, const Params& params);
    static std::vector<std::vector<ChargingOption>> readChargingOptions(const std::vector<Node>& nodes, const std::string& filename);
    static Params readParams(const std::string& filename);
};

#endif // ETSP_WCL_OPTIMIZATION_CSVREADER_H
