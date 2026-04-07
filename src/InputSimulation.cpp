#include <Arduino.h>
#include <math.h>
#include "Config.h"
#include "PoolMaster.h"
#include "InputSimulation.h"

namespace {
constexpr double kDefaultWaterTemp = 24.0;
constexpr double kDefaultAirTemp = 20.0;
constexpr double kDefaultPh = 7.20;
constexpr double kDefaultOrp = 690.0;
constexpr double kDefaultPsi = 0.05;

double simWaterTemp = kDefaultWaterTemp;
double simAirTemp = kDefaultAirTemp;
double simPh = kDefaultPh;
double simOrp = kDefaultOrp;
double simPsi = kDefaultPsi;
unsigned long lastUpdateMs = 0;

struct DoubleOverride {
  bool enabled = false;
  double value = 0.0;
};

struct BoolOverride {
  bool enabled = false;
  bool value = false;
};

DoubleOverride waterTempOverride;
DoubleOverride airTempOverride;
DoubleOverride phOverride;
DoubleOverride orpOverride;
DoubleOverride psiOverride;
BoolOverride flowOverride;

void applyOverrides()
{
  if (waterTempOverride.enabled) {
    simWaterTemp = waterTempOverride.value;
  }
  if (airTempOverride.enabled) {
    simAirTemp = airTempOverride.value;
  }
  if (phOverride.enabled) {
    simPh = phOverride.value;
  }
  if (orpOverride.enabled) {
    simOrp = orpOverride.value;
  }
  if (psiOverride.enabled) {
    simPsi = psiOverride.value;
  }
}

void logOverride(const char* label, double value)
{
  Debug.print(DBG_INFO, "[SIM] %s forced to %.2f", label, value);
}
}

namespace InputSimulation {
void update()
{
  const ElectrolysisRuntimeData electrolysisSnapshot = GetElectrolysisDataSnapshot();
  const unsigned long now = millis();
  if (lastUpdateMs == 0) {
    lastUpdateMs = now;
    applyOverrides();
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
  if (electrolysisSnapshot.outputActive) {
    simOrp += dtSeconds * 0.35;
  } else {
    simOrp -= dtSeconds * 0.12;
  }
  simOrp = constrain(simOrp, 550.0, 820.0);

  // Electrolysis tends to push pH up slightly; idle state slowly recenters.
  if (electrolysisSnapshot.outputActive) {
    simPh += dtSeconds * 0.00035;
  } else if (simPh > 7.20) {
    simPh -= dtSeconds * 0.00015;
  } else {
    simPh += dtSeconds * 0.00005;
  }
  simPh = constrain(simPh, 6.8, 7.8);

  applyOverrides();
}

double waterTemp() { return simWaterTemp; }
double airTemp() { return simAirTemp; }
double ph() { return simPh; }
double orp() { return simOrp; }
double psi() { return simPsi; }
bool flowOk()
{
  if (flowOverride.enabled) {
    return flowOverride.value;
  }
  return FiltrationPump.IsRunning();
}

void setWaterTemp(double value)
{
  waterTempOverride.enabled = true;
  waterTempOverride.value = constrain(value, -20.0, 80.0);
  simWaterTemp = waterTempOverride.value;
  logOverride("WaterTemp", simWaterTemp);
}

void setAirTemp(double value)
{
  airTempOverride.enabled = true;
  airTempOverride.value = constrain(value, -20.0, 80.0);
  simAirTemp = airTempOverride.value;
  logOverride("AirTemp", simAirTemp);
}

void setPh(double value)
{
  phOverride.enabled = true;
  phOverride.value = constrain(value, 0.0, 14.0);
  simPh = phOverride.value;
  logOverride("pH", simPh);
}

void setOrp(double value)
{
  orpOverride.enabled = true;
  orpOverride.value = constrain(value, -1000.0, 1500.0);
  simOrp = orpOverride.value;
  logOverride("ORP", simOrp);
}

void setPsi(double value)
{
  psiOverride.enabled = true;
  psiOverride.value = constrain(value, 0.0, 10.0);
  simPsi = psiOverride.value;
  logOverride("PSI", simPsi);
}

void setFlow(bool value)
{
  flowOverride.enabled = true;
  flowOverride.value = value;
  Debug.print(DBG_INFO, "[SIM] Flow forced to %s", value ? "ON" : "OFF");
}

void reset()
{
  waterTempOverride.enabled = false;
  airTempOverride.enabled = false;
  phOverride.enabled = false;
  orpOverride.enabled = false;
  psiOverride.enabled = false;
  flowOverride.enabled = false;

  simWaterTemp = kDefaultWaterTemp;
  simAirTemp = kDefaultAirTemp;
  simPh = kDefaultPh;
  simOrp = kDefaultOrp;
  simPsi = kDefaultPsi;
  lastUpdateMs = 0;

  Debug.print(DBG_INFO, "[SIM] Manual overrides cleared");
}
}
