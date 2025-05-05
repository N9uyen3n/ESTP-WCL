//
// Created by Admin on 16/04/2025.
//
// Arc.h
#ifndef ARC_H
#define ARC_H

class Arc {
public:
    int from, to;          // ID của nút đầu và cuối
    double dij;            // Khoảng cách (d_ij)
    double sij;            // Thời gian di chuyển (s_ij)
    bool is_wireless;      // Cung có sạc không dây không
    double beta_ij;        // Tốc độ sạc hiệu quả (β_ij)
    double U_min, U_max;   // Giới hạn tốc độ (dùng cho cung không dây)
};

#endif // ETSP_WCL_OPTIMIZATION_ARC_H