//
// Created by Admin on 17/04/2025.
//

#include "Optimizer.h"
#include "Optimizer.h"
#include <ilcplex/ilocplex.h>
#include <iostream>
#include <limits>

ILOSTLBEGIN

double Optimizer::optimize(
    const std::vector<int>& S_prime,
    const std::vector<Arc>& arcs,
    const std::vector<std::vector<ChargingOption>>& charge_options,
    const Params& params) {
    try {
        IloEnv env;
        IloModel model(env);
        IloCplex cplex(model);

        int n = S_prime.size();
        int m = n - 1; // Number of arcs in the route

        // Calculate data for each arc in the route
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
            for (const auto& arc : arcs) {
                if (arc.from == i && arc.to == j) {
                    d_route[k] = arc.dij;
                    beta_route[k] = arc.beta_ij;
                    is_w_route[k] = arc.is_wireless;
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

        // Linearization variables for phi[i] * w[i][k]
        std::vector<IloNumVarArray> phi_w(n);
        for (int i = 0; i < n; ++i) {
            int node = S_prime[i];
            phi_w[i] = IloNumVarArray(env, charge_options[node].size(), 0, IloInfinity, ILOFLOAT);
        }

        // Decision variables for wireless charging
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

        // Objective function
        IloExpr obj(env);

        // Thành phần 1: Chi phí sạc cố định
        for (int i = 0; i < n; ++i) {
            int node = S_prime[i];
            for (size_t kk = 0; kk < charge_options[node].size(); ++kk) {
                obj += charge_options[node][kk].cost * charge_options[node][kk].rate * phi_w[i][kk];
            }
        }

        // Thành phần 2: Chi phí sạc không dây
        for (int l = 0; l < p; ++l) {
            int k = wireless_k[l];
            obj += params.cw * beta_route[k] * w_s_z[l];
        }

        // Thành phần 3: Chi phí thời gian
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

        // Time progression constraints
        model.add(t[0] == 0);
        for (int k = 0; k < m; ++k) {
            int i = S_prime[k];
            int j_idx = k + 1;
            IloExpr departure = t[k] + phi[i];
            model.add(t[j_idx] >= departure + s[k]);
        }

        // SOC constraints
        // Initial SOC
        model.add(ya[0] == params.initial_SOC);
        // SOC at nodes
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
        // SOC on arcs
        // for (int k = 0; k < m; ++k) {
        //     int i_idx = k;
        //     int j_idx = k + 1;
        //     if (!is_w_route[k]) {
        //         model.add(ya[j_idx] <= yd[i_idx] - params.h * d_route[k]);
        //     } else {
        //         int l = -1;
        //         for (int ll = 0; ll < p; ++ll) {
        //             if (wireless_k[ll] == k) {
        //                 l = ll;
        //                 break;
        //             }
        //         }
        //         model.add(ya[j_idx] <= yd[i_idx] - params.h * d_route[k] + beta_route[k] * w_s_z[l]);
        //     }
        // }

        for (int k = 0; k < m; ++k) {
            int i_idx = k;
            int j_idx = k + 1;
            if (!is_w_route[k]) {
                model.add(ya[j_idx] <= yd[i_idx] - params.h * d_route[k]);
            } else {
                int l = -1;
                for (int ll = 0; ll < p; ++ll) {
                    if (wireless_k[ll] == k) {
                        l = ll;
                        break;
                    }
                }
                if (l != -1) {
                    model.add(ya[j_idx] <= yd[i_idx] - params.h * d_route[k] + beta_route[k] * w_s_z[l]);
                } else {
                    std::cerr << "Error: Wireless arc " << k << " not found in wireless_k" << std::endl;
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
                model.add(sum_w <= 1);
            }
        }

        // Solve the model
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