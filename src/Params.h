//
// Created by Admin on 16/04/2025.
//

// Params.h
#ifndef PARAMS_H
#define PARAMS_H

class Params {
public:
    double Q;              // Dung lượng pin
    double minSOC;         // SOC tối thiểu
    double h;              // Tỷ lệ tiêu thụ năng lượng
    double v;              // Tốc độ trung bình
    double cw;             // Chi phí sạc không dây (c_w)
    double ct;             // Chi phí thời gian (c_t)
    double initial_SOC;    // SOC ban đầu
    double M;              // Hằng số lớn cho ràng buộc big-M
};

#endif // ETSP_WCL_OPTIMIZATION_PARAMS_H
