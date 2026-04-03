#pragma once

namespace InputSimulation {
void update();
double waterTemp();
double airTemp();
double ph();
double orp();
double psi();
bool flowOk();
void setWaterTemp(double value);
void setAirTemp(double value);
void setPh(double value);
void setOrp(double value);
void setPsi(double value);
void setFlow(bool value);
void reset();
}
