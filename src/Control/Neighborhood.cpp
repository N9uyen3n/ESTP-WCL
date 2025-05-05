//
// Created by Admin on 29/04/2025.
//

#include "../../include/Neighborhood.h"
#include "../../include/Utils.h"
#include <random>
#include <algorithm>

namespace {
    // Single RNG for reproducibility and efficiency
    std::mt19937& getRNG() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        return gen;
    }

    // 2-opt move, đảm bảo không đảo ngược depot
    std::vector<int> applyTwoOpt(const std::vector<int>& route) {
        int n = route.size();
        if (n < 4) return route; // Cần ít nhất 4 nút để thực hiện TWO_OPT
        // Giới hạn p và q để không chạm vào depot ở đầu (0) và cuối (n-1)
        std::uniform_int_distribution<> dis_p(1, n - 4); // Bắt đầu từ vị trí 1
        int p = dis_p(getRNG());
        std::uniform_int_distribution<> dis_q(p + 2, n - 2); // Kết thúc trước vị trí n-1
        int q = dis_q(getRNG());
        std::vector<int> new_route(route);
        std::reverse(new_route.begin() + p, new_route.begin() + q + 1);
        return new_route;
    }

    // Relocate a random customer
    std::vector<int> applyRelocate(const std::vector<int>& route, const std::vector<Node>& nodes) {
        int n = route.size();
        if (n < 4) return route;
        std::vector<int> cust_pos;
        for (int i = 1; i < n - 1; ++i) {
            if (isCustomer(route[i], nodes)) {
                cust_pos.push_back(i);
            }
        }
        if (cust_pos.empty()) return route;
        std::uniform_int_distribution<> dis_c(0, cust_pos.size() - 1);
        int i = cust_pos[dis_c(getRNG())]; // Vị trí khách hàng được chọn
        int c = route[i];
        std::vector<int> temp;
        temp.reserve(n - 1);
        temp.insert(temp.end(), route.begin(), route.begin() + i);
        temp.insert(temp.end(), route.begin() + i + 1, route.end());
        // Chọn vị trí chèn mới, tránh vị trí 0 và n-1
        std::uniform_int_distribution<> dis_j(1, temp.size() - 1);
        int j = dis_j(getRNG());
        temp.insert(temp.begin() + j, c);
        return temp;
    }

    // Insert a charging station between two nodes
    std::vector<int> applyInsertCharge(const std::vector<int>& route, const std::vector<Node>& nodes) {
        int n = route.size();
        // Collect station IDs
        std::vector<int> stations;
        for (const auto& nd : nodes) {
            if (nd.type == "f") stations.push_back(nd.id);
        }
        if (stations.empty() || n < 2) return route;
        // Choose random edge, tránh chèn trước depot đầu hoặc sau depot cuối
        std::uniform_int_distribution<> dis_edge(1, n - 2);
        int pos = dis_edge(getRNG());
        // Choose random station
        std::uniform_int_distribution<> dis_s(0, stations.size() - 1);
        int sid = stations[dis_s(getRNG())];
        // Insert
        std::vector<int> new_route;
        new_route.reserve(n + 1);
        new_route.insert(new_route.end(), route.begin(), route.begin() + pos);
        new_route.push_back(sid);
        new_route.insert(new_route.end(), route.begin() + pos, route.end());
        return new_route;
    }

    // Remove a random station visit
    std::vector<int> applyRemoveCharge(const std::vector<int>& route, const std::vector<Node>& nodes) {
        int n = route.size();
        // Find station positions
        std::vector<int> pos;
        for (int i = 1; i < n - 1; ++i) {
            if (isChargingStation(route[i], nodes)) pos.push_back(i);
        }
        if (pos.empty()) return route;
        std::uniform_int_distribution<> dis_p(0, pos.size() - 1);
        int idx = pos[dis_p(getRNG())];
        std::vector<int> new_route(route);
        new_route.erase(new_route.begin() + idx);
        return new_route;
    }
}

std::vector<int> generateRandomNeighbor(const std::vector<int>& route,
                                        NeighborhoodType type,
                                        const std::vector<Node>& nodes) {
    switch (type) {
        case TWO_OPT:
            return applyTwoOpt(route);
        case RELOCATE:
            return applyRelocate(route, nodes);
        case INSERT_CHARGE:
            return applyInsertCharge(route, nodes);
        case REMOVE_CHARGE:
            return applyRemoveCharge(route, nodes);
        default:
            return route;
    }
}