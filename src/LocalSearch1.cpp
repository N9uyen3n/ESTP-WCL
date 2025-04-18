#include <ilcplex/ilocplex.h>
#include <vector>
#include <iostream>
#include <limits>

ILOSTLBEGIN

// Cấu trúc lưu trữ thông tin cung
struct Arc {
    int i, j;
    double dij; // Khoảng cách
    bool is_wireless; // Là cung sạc không dây?
    double beta_ij; // Hệ số năng lượng sạc không dây
    double U_min; // Tốc độ tối thiểu
    double U_max; // Tốc độ tối đa
};

// Cấu trúc lưu trữ thông tin tùy chọn sạc tại trạm
struct ChargeOption {
    double cik; // Chi phí mỗi đơn vị năng lượng
    double rik; // Tốc độ sạc
};

double LocalSearch(
    const std::vector<int>& S_prime,
    const std::vector<std::vector<Arc>>& arcs,
    const std::vector<std::vector<ChargeOption>>& charge_options,
    double h,
    double Q,
    double minSOC,
    double cw,
    double ct,
    double initial_SOC
) {
    try {
        IloEnv env;
        IloModel model(env);
        IloCplex cplex(model);

        int n = S_prime.size();
        int m = n - 1; // Số cung trong lộ trình

        // Tính toán dữ liệu cho từng cung trong lộ trình
        std::vector<double> d_route(m);
        std::vector<double> beta_route(m);
        std::vector<bool> is_w_route(m);
        std::vector<double> U_min_route(m);
        std::vector<double> U_max_route(m);
        std::vector<double> min_s(m);
        std::vector<double> max_s(m);
        for (int k = 0; k < m; ++k) {
            int i = S_prime[k];
            int j = S_prime[k + 1];
            bool found = false;
            for (const auto& arc : arcs[i]) {
                if (arc.j == j) {
                    d_route[k] = arc.dij;
                    beta_route[k] = arc.beta_ij;
                    is_w_route[k] = arc.is_wireless;
                    U_min_route[k] = arc.U_min;
                    U_max_route[k] = arc.U_max;
                    min_s[k] = arc.dij / arc.U_max; // Thời gian tối thiểu
                    max_s[k] = arc.dij / arc.U_min; // Thời gian tối đa
                    found = true;
                    break;
                }
            }
            if (!found) {
                env.end();
                return std::numeric_limits<double>::infinity();
            }
        }

        // Biến quyết định
        IloNumVarArray phi(env, n, 0, IloInfinity, ILOFLOAT);
        std::vector<IloBoolVarArray> w(n);
        for (int i = 0; i < n; ++i) {
            int node = S_prime[i];
            w[i] = IloBoolVarArray(env, charge_options[node].size());
        }
        IloNumVarArray s(env);
        for (int k = 0; k < m; ++k) {
            s.add(IloNumVar(env, min_s[k], max_s[k], ILOFLOAT));
        }
        IloNumVarArray ya(env, n, minSOC, Q, ILOFLOAT);
        IloNumVarArray yd(env, n, minSOC, Q, ILOFLOAT);
        IloNumVarArray t(env, n, 0, IloInfinity, ILOFLOAT);

        // Biến tuyến tính hóa cho phi[i] * w[i][k]
        std::vector<IloNumVarArray> phi_w(n);
        for (int i = 0; i < n; ++i) {
            int node = S_prime[i];
            phi_w[i] = IloNumVarArray(env, charge_options[node].size(), 0, IloInfinity, ILOFLOAT);
        }

        // Biến quyết định cho sạc không dây
        std::vector<int> wireless_k;
        for (int k = 0; k < m; ++k) {
            if (is_w_route[k]) {
                wireless_k.push_back(k);
            }
        }
        int p = wireless_k.size();
        IloBoolVarArray z(env, p);
        std::vector<IloNumVar> w_s_z(p);
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            double M = max_s[k];
            w_s_z[l] = IloNumVar(env, 0, M, ILOFLOAT);
        }

        // Hàm mục tiêu
        IloExpr obj(env);

        // Thành phần 1: Chi phí sạc cố định
        for (int i = 0; i < n; ++i) {
            int node = S_prime[i];
            for (size_t kk = 0; kk < charge_options[node].size(); ++kk) {
                obj += charge_options[node][kk].cik * charge_options[node][kk].rik * phi_w[i][kk];
            }
        }

        // Thành phần 2: Chi phí sạc không dây
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            obj += cw * beta_route[k] * w_s_z[l];
        }

        // Thành phần 3: Chi phí thời gian
        obj += ct * t[n - 1];

        model.add(IloMinimize(env, obj));

        // Ràng buộc tuyến tính hóa cho phi[i] * w[i][k]
        for (int i = 0; i < n; ++i) {
            int node = S_prime[i];
            for (size_t kk = 0; kk < charge_options[node].size(); ++kk) {
                double U_phi = Q / (charge_options[node].size() > 0 ? charge_options[node][0].rik : 1.0);
                model.add(phi_w[i][kk] <= phi[i]);
                model.add(phi_w[i][kk] <= U_phi * w[i][kk]);
                model.add(phi_w[i][kk] >= phi[i] - U_phi * (1 - w[i][kk]));
                model.add(phi_w[i][kk] >= 0);
            }
        }

        // Ràng buộc không sạc tại các nút không có tùy chọn sạc
        for (int i = 0; i < n; ++i) {
            int node = S_prime[i];
            if (charge_options[node].size() == 0) {
                model.add(phi[i] == 0);
            }
        }

        // Ràng buộc tiến triển thời gian
        model.add(t[0] == 0);
        for (int k = 0; k < m; ++k) {
            int i = S_prime[k];
            int j_idx = k + 1;
            IloExpr departure = t[k] + phi[i];
            model.add(t[j_idx] >= departure + s[k]);
        }

        // Ràng buộc SOC
        // SOC ban đầu
        model.add(ya[0] == initial_SOC);
        // SOC tại các nút
        for (int i = 0; i < n; ++i) {
            if (charge_options[S_prime[i]].size() > 0) {
                IloExpr charge_amount(env);
                for (size_t kk = 0; kk < charge_options[S_prime[i]].size(); ++kk) {
                    charge_amount += charge_options[S_prime[i]][kk].rik * phi_w[i][kk];
                }
                model.add(yd[i] == ya[i] + charge_amount);
            } else {
                model.add(yd[i] == ya[i]);
            }
        }
        // SOC trên các cung
        for (int k = 0; k < m; ++k) {
            int i_idx = k;
            int j_idx = k + 1;
            if (!is_w_route[k]) {
                model.add(ya[j_idx] <= yd[i_idx] - h * d_route[k]);
            } else {
                // Tìm l cho cung không dây này
                int l = -1;
                for (int ll = 0; ll < p; ++ll) {
                    if (wireless_k[ll] == k) {
                        l = ll;
                        break;
                    }
                }
                model.add(ya[j_idx] <= yd[i_idx] - h * d_route[k] + beta_route[k] * w_s_z[l]);
            }
        }

        // Tuyến tính hóa cho w_s_z
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            double M = max_s[k];
            model.add(w_s_z[l] <= s[k]);
            model.add(w_s_z[l] <= M * z[l]);
            model.add(w_s_z[l] >= s[k] - M * (1 - z[l]));
        }

        // Ràng buộc lựa chọn sạc
        for (int i = 0; i < n; ++i) {
            int node = S_prime[i];
            if (charge_options[node].size() > 0) {
                IloExpr sum_w(env);
                for (size_t kk = 0; kk < charge_options[node].size(); ++kk) {
                    sum_w += w[i][kk];
                }
                model.add(sum_w <= 1);
            }
        }

        // Giải bài toán
        cplex.setOut(env.getNullStream());
        if (!cplex.solve()) {
            env.end();
            return std::numeric_limits<double>::infinity();
        }

        double cost = cplex.getObjValue();
        env.end();
        return cost;

    } catch (IloException& e) {
        std::cerr << "CPLEX Exception: " << e.getMessage() << std::endl;
        return std::numeric_limits<double>::infinity();
    }
}

int main() {
    std::vector<int> S_prime = {0, 1, 2, 0}; // Lộ trình ví dụ
    std::vector<std::vector<Arc>> arcs(3);
    arcs[0].push_back({0, 1, 10.0, false, 0.0, 5.0, 10.0});
    arcs[1].push_back({1, 2, 15.0, true, 2.0, 4.0, 8.0});
    arcs[2].push_back({2, 0, 5.0, false, 0.0, 6.0, 12.0});

    std::vector<std::vector<ChargeOption>> charge_options(3);
    charge_options[0].push_back({0.1, 10.0});
    charge_options[2].push_back({0.2, 5.0});

    double h = 0.2;
    double Q = 50.0;
    double minSOC = 0.0;
    double cw = 0.15;
    double ct = 0.1;
    double initial_SOC = Q;

    double cost = LocalSearch(S_prime, arcs, charge_options, h, Q, minSOC, cw, ct, initial_SOC);
    if (cost == std::numeric_limits<double>::infinity()) {
        std::cout << "Lộ trình không khả thi!" << std::endl;
    } else {
        std::cout << "Chi phí tối ưu: " << cost << std::endl;
    }

    return 0;
}