#ifndef PARAMETERS_H
#define PARAMETERS_H

struct ChargingOption {
private:
    int station_id;
    int option;
    double rate;
    double cost;

public:
    ChargingOption(double cost, double rate) : cost(cost), rate(rate) {}

    // Getters
    double getCost() const { return cost; }
    double getRate() const { return rate; }
    double getStationId() const { return station_id; }
    double getOption() const { return option; }

    // Setters
    void setCost(double new_cost) { cost = new_cost; }
    void setRate(double new_rate) { rate = new_rate; }
    void setStationId(int id) { station_id = id; }
    void setOption(int opt) { option = opt; }
};

class Parameters {
private:
    double battery_capacity;      // Dung lượng pin (Q)
    double min_soc;              // SOC tối thiểu
    double energy_consumption;   // Tỷ lệ tiêu thụ năng lượng (h)
    double vehicle_speed;        // Tốc độ trung bình (v)
    double wireless_cost;        // Chi phí sạc không dây (c_w)
    double time_cost;            // Chi phí thời gian (c_t)
    double initial_soc;          // SOC ban đầu
    double big_m;                // Hằng số lớn (M)
    double U_max = vehicle_speed * 0.5;                 // Tốc độ tối đa
    double U_min = vehicle_speed * 2;                 // Tốc độ tối thiểu


public:
    Parameters(double battery_capacity, double min_soc, double energy_consumption,
               double vehicle_speed, double wireless_cost, double time_cost,
               double initial_soc, double big_m)
        : battery_capacity(battery_capacity), min_soc(min_soc),
          energy_consumption(energy_consumption), vehicle_speed(vehicle_speed),
          wireless_cost(wireless_cost), time_cost(time_cost),
          initial_soc(initial_soc), big_m(big_m) {}

    // Getters
    double getBatteryCapacity() const { return battery_capacity; }
    double getMinSoc() const { return min_soc; }
    double getEnergyConsumption() const { return energy_consumption; }
    double getVehicleSpeed() const { return vehicle_speed; }
    double getWirelessCost() const { return wireless_cost; }
    double getTimeCost() const { return time_cost; }
    double getInitialSoc() const { return initial_soc; }
    double getBigM() const { return big_m; }
    double getUmax() const { return U_max; }
    double getUmin() const { return U_min; }

    // Setters
    void setBatteryCapacity(double cap) { battery_capacity = cap; }
    void setMinSoc(double soc) { min_soc = soc; }
    void setEnergyConsumption(double consumption) { energy_consumption = consumption; }
    void setVehicleSpeed(double speed) { vehicle_speed = speed; }
    void setWirelessCost(double cost) { wireless_cost = cost; }
    void setTimeCost(double cost) { time_cost = cost; }
    void setInitialSoc(double soc) { initial_soc = soc; }
    void setBigM(double m) { big_m = m; }
};

#endif // PARAMETERS_H