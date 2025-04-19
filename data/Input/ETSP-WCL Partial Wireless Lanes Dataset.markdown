# ETSP-WCL Partial Wireless Lanes Dataset

## Overview

This dataset extends the Schneider et al. (2014) EVRPTW dataset for ETSP-WCL, with 1 depot, 5 recharging stations, and 10 customers. Wireless charging lanes cover only a fraction of selected arcs.

## Files

- **nodes.csv**: 16 nodes (D0, S0–S4, C1–C10).
  - Columns: StringID, Type, x, y, demand, ReadyTime, DueDate, ServiceTime
- **wireless_arcs.csv**: \~48 arcs (\~20% of undirected pairs, bidirectional).
  - Columns: From, To, Beta (0.1-0.9)
- **charging_options.csv**: 10 rows (5 stations × 2 options).
  - Columns: StationID, Option, Rate, Cost
- **params.csv**: Vehicle and cost parameters.
  - Columns: Q, C, r, g, v, c_w, c_t


