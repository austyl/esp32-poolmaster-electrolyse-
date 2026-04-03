#include <Arduino.h>
#include <math.h>
#include "Config.h"
#include "PoolMaster.h"
#include "InputSimulation.h"

namespace {
double simWaterTemp = 24.0;
double simAirTemp = 20.0;
double simPh = 7.20;
double simOrp = 690.0;
double simPsi = 0.05;
unsigned long lastUpdateMs = 0;
}

namespace InputSimulation {
void update()
{
  const unsigned long now = millis();
  if (lastUpdateMs == 0) {
    lastUpdateMs = now;
    return;
  }

  const double dtSeconds = (double)(now - lastUpdateMs) / 1000.0;
  lastUpdateMs = now;
  const double tHours = (double)now / 3600000.0;

  // Temperature waves keep dashboards moving while remaining realistic.
  simAirTemp = 18.0 + 6.0 * sin(tHours * 0.6);
  simWaterTemp = 24.0 + 2.5 * sin(tHours * 0.2);

  // Pressure follows filtration status.
  if (FiltrationPump.IsRunning()) {
    simPsi = 0.62 + 0.04 * sin(tHours * 2.0);
  } else {
    simPsi = 0.02;
  }

  // ORP slowly rises while electrolysis is really active and decays otherwise.
  if (ElectrolysisData.outputActive) {
    simOrp += dtSeconds * 0.35;
  } else {
    simOrp -= dtSeconds * 0.12;
  }
  simOrp = constrain(simOrp, 550.0, 820.0);

  // Electrolysis tends to push pH up slightly; idle state slowly recenters.
  if (ElectrolysisData.outputActive) {
    simPh += dtSeconds * 0.00035;
  } else if (simPh > 7.20) {
    simPh -= dtSeconds * 0.00015;
  } else {
    simPh += dtSeconds * 0.00005;
  }
  simPh = constrain(simPh, 6.8, 7.8);
}

double waterTemp() { return simWaterTemp; }
double airTemp() { return simAirTemp; }
double ph() { return simPh; }
double orp() { return simOrp; }
double psi() { return simPsi; }
bool flowOk() { return FiltrationPump.IsRunning(); }
}
