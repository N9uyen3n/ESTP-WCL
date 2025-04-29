//
// Created by Admin on 17/04/2025.
//

#include "../../include/Optimizer.h"
#include "../../include/Utils.h"
#include <ilcplex/ilocplex.h>
#include <limits>
#include <vector>

ILOSTLBEGIN

double Optimizer::optimize(
    const std::vector<int>& S_prime,
    const std::vector<Arc>& arcs,
    const std::vector<std::vector<ChargingOption>>& charge_options,
    const Params& params,
    const std::vector<Node>& nodes,
    ModelParameters& modelParams) {
    try {
        IloEnv env;
        IloModel model(env);
        IloCplex cplex(model);

        int n = S_prime.size();
        int m = n - 1; // Number of arcs in the route

        // Calculate data for each arc in the route
        std::vector<double> d_ij(m);
        std::vector<double> beta_ij(m);
        std::vector<bool> is_wireless_route(m);
        std::vector<double> U_min_route(m);
        std::vector<double> U_max_route(m);
        std::vector<double> min_s(m);
        std::vector<double> max_s(m);
        for (int k = 0; k < m; ++k) {
            int i = S_prime[k];
            int j = S_prime[k + 1];
            bool found = false;
            for (const auto& arc : arcs) {
                if (arc.from == i && arc.to == j) {
                    d_ij[k] = arc.dij;
                    beta_ij[k] = arc.beta_ij;
                    is_wireless_route[k] = arc.is_wireless;
                    U_min_route[k] = arc.U_min;
                    U_max_route[k] = arc.U_max;
                    min_s[k] = arc.dij / arc.U_max; // Minimum time
                    max_s[k] = arc.dij / arc.U_min; // Maximum time
                    found = true;
                    break;
                }
            }
            if (!found) {
                env.end();
                return std::numeric_limits<double>::infinity();
            }
        }

        // Decision variables
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
        IloNumVarArray ya(env, n, params.minSOC, params.Q, ILOFLOAT);
        IloNumVarArray yd(env, n, params.minSOC, params.Q, ILOFLOAT);
        IloNumVarArray t(env, n, 0, IloInfinity, ILOFLOAT);
        IloNumVarArray depart(env, n, 0, IloInfinity, ILOFLOAT);

        // Linearization variables for phi[i] * w[i][k]
        std::vector<IloNumVarArray> phi_w(n);
        for (int i = 0; i < n; ++i) {
            int node = S_prime[i];
            phi_w[i] = IloNumVarArray(env, charge_options[node].size(), 0, IloInfinity, ILOFLOAT);
        }

        // Decision variables for wireless charging
        std::vector<int> wireless_k;
        for (int k = 0; k < m; ++k) {
            if (is_wireless_route[k]) {
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

        // Objective function
        IloExpr obj(env);

        // Component 1: Fixed charging costs
        for (int i = 0; i < n; ++i) {
            int node_i = S_prime[i];
            for (size_t kk = 0; kk < charge_options[node_i].size(); ++kk) {
                obj += charge_options[node_i][kk].cost * charge_options[node_i][kk].rate * phi_w[i][kk];
            }
        }

        // Component 2: Wireless charging costs
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            obj += params.cw * beta_ij[k] * w_s_z[l];
        }

        // Component 3: Time costs
        obj += params.ct * t[n - 1];

        model.add(IloMinimize(env, obj));

        // Constraints for linearization of phi[i] * w[i][k]
        for (int i = 0; i < n; ++i) {
            int node = S_prime[i];
            for (size_t kk = 0; kk < charge_options[node].size(); ++kk) {
                double U_phi = params.Q / (charge_options[node].size() > 0 ? charge_options[node][0].rate : 1.0);
                model.add(phi_w[i][kk] <= phi[i]);
                model.add(phi_w[i][kk] <= U_phi * w[i][kk]);
                model.add(phi_w[i][kk] >= phi[i] - U_phi * (1 - w[i][kk]));
                model.add(phi_w[i][kk] >= 0);
            }
        }

        // Constraints for no charging at nodes without options
        for (int i = 0; i < n; ++i) {
            int node = S_prime[i];
            if (charge_options[node].size() == 0) {
                model.add(phi[i] == 0);
            }
        }

        // Constraints for departure time
        model.add(depart[0] == t[0]);
        for (int i = 1; i < n; ++i) {
            int node = S_prime[i];
            if (charge_options[node].size() > 0) {
                model.add(depart[i] == t[i] + phi[i]);
            } else if (node != 0) {
                model.add(depart[i] == t[i] + nodes[node].service_time);
            } else {
                model.add(depart[i] == t[i]);
            }
        }

        // Time progression constraints
        model.add(t[0] == 0);
        for (int k = 0; k < m; ++k) {
            int i_idx = k;
            int j_idx = k + 1;
            model.add(t[j_idx] >= depart[i_idx] + s[k]);
        }

        // SOC constraints
        model.add(ya[0] == params.initial_SOC);
        for (int i = 0; i < n; i++) {
            model.add(ya[i] >= params.minSOC);
            model.add(yd[i] >= params.minSOC);
        }
        for (int i = 0; i < n; ++i) {
            if (charge_options[S_prime[i]].size() > 0) {
                IloExpr charge_amount(env);
                for (size_t kk = 0; kk < charge_options[S_prime[i]].size(); ++kk) {
                    charge_amount += charge_options[S_prime[i]][kk].rate * phi_w[i][kk];
                }
                model.add(yd[i] == ya[i] + charge_amount);
            } else {
                model.add(yd[i] == ya[i]);
            }
        }
        for (int k = 0; k < m; ++k) {
            int i_idx = k;
            int j_idx = k + 1;
            if (!is_wireless_route[k]) {
                model.add(ya[j_idx] <= yd[i_idx] - params.h * d_ij[k] * params.Q / 100.0);
            } else {
                int l = -1;
                for (int ll = 0; ll < p; ++ll) {
                    if (wireless_k[ll] == k) {
                        l = ll;
                        break;
                    }
                }
                if (l != -1) {
                    model.add(ya[j_idx] <= yd[i_idx] - params.h * d_ij[k] * params.Q / 100.0 + beta_ij[k] * w_s_z[l]);
                } else {
                    env.end();
                    return std::numeric_limits<double>::infinity();
                }
            }
        }

        // Linearization for w_s_z
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            double M = max_s[k];
            model.add(w_s_z[l] <= s[k]);
            model.add(w_s_z[l] <= M * z[l]);
            model.add(w_s_z[l] >= s[k] - M * (1 - z[l]));
        }

        // Charging selection constraints
        for (int i = 0; i < n; ++i) {
            int node = S_prime[i];
            if (charge_options[node].size() > 0) {
                IloExpr sum_w(env);
                for (size_t kk = 0; kk < charge_options[node].size(); ++kk) {
                    sum_w += w[i][kk];
                }
                model.add(sum_w == 1);
            }
        }

        // Solve the model
        cplex.setOut(env.getNullStream());
        if (!cplex.solve()) {
            env.end();
            return std::numeric_limits<double>::infinity();
        }

        double cost = cplex.getObjValue();

        // Lưu các tham số vào ModelParameters
        // Xóa các vector hiện tại trong modelParams để tránh dữ liệu cũ
        clearModelParameters(modelParams);

        // Lưu phi (thời gian sạc tại các nút)
        for (int i = 0; i < n; ++i) {
            modelParams.phi.push_back(cplex.getValue(phi[i]));
        }

        // Lưu w (lựa chọn tùy chọn sạc)
        modelParams.w.resize(n);
        for (int i = 0; i < n; ++i) {
            int node = S_prime[i];
            for (size_t kk = 0; kk < charge_options[node].size(); ++kk) {
                modelParams.w[i].push_back(static_cast<int>(cplex.getValue(w[i][kk])));
            }
        }

        // Lưu s (thời gian di chuyển trên các cung)
        for (int k = 0; k < m; ++k) {
            modelParams.s.push_back(cplex.getValue(s[k]));
        }

        // Lưu ya (SOC khi đến)
        for (int i = 0; i < n; ++i) {
            modelParams.ya.push_back(cplex.getValue(ya[i]));
        }

        // Lưu yd (SOC khi rời)
        for (int i = 0; i < n; ++i) {
            modelParams.yd.push_back(cplex.getValue(yd[i]));
        }

        // Lưu t (thời gian đến)
        for (int i = 0; i < n; ++i) {
            modelParams.t.push_back(cplex.getValue(t[i]));
        }

        // Lưu depart (thời gian rời)
        for (int i = 0; i < n; ++i) {
            modelParams.depart.push_back(cplex.getValue(depart[i]));
        }

        // Lưu z (lựa chọn sạc không dây)
        for (int l = 0; l < p; ++l) {
            modelParams.z.push_back(static_cast<int>(cplex.getValue(z[l])));
        }

        // Lưu w_s_z (giá trị tuyến tính hóa sạc không dây)
        for (int l = 0; l < p; ++l) {
            modelParams.w_s_z.push_back(cplex.getValue(w_s_z[l]));
        }

        env.end();
        return cost;

    } catch (IloException& e) {
        std::cerr << "CPLEX Exception: " << e.getMessage() << std::endl;
        return std::numeric_limits<double>::infinity();
    }
}