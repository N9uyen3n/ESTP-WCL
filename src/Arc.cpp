//
// Created by Admin on 05/04/2025.
//

#include "Arc.h"

Arc::Arc(int from, int to, double distance, double travelTime,
         bool hasWirelessCharging, double effectiveChargingRate)
    : from(from), to(to), distance(distance), travelTime(travelTime),
      hasWirelessCharging(hasWirelessCharging), effectiveChargingRate(effectiveChargingRate) {}
