#ifndef ARC_H
#define ARC_H

class Arc {
public:
    int from;              // Node xuất phát (ID)
    int to;                // Node đích (ID)
    double distance;       // dᵢⱼ: khoảng cách
    double travelTime;     // sᵢⱼ: thời gian di chuyển
    bool hasWirelessCharging;  // Cho biết cung có sạc không dây hay không
    double effectiveChargingRate; // βᵢⱼ: hiệu suất sạc không dây (nếu có)

    Arc(int from, int to, double distance, double travelTime,
        bool hasWirelessCharging = false, double effectiveChargingRate = 0.0);
};

#endif // ARC_H
