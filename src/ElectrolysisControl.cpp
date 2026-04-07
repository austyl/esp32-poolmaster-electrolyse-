#include <Arduino.h>
#include "Config.h"
#include "PoolMaster.h"
#include "InputSimulation.h"

namespace {
bool isConfiguredPin(int pin)
{
  return pin >= 0;
}

bool electrolysisEnabled()
{
  // Keep compatibility with the legacy ElectrolyseMode flag used by the Nextion UI.
  return PMConfig.get<bool>(ELECTROLYSIS_ENABLED) || PMConfig.get<bool>(ELECTROLYSEMODE);
}

bool readFlowSwitchRaw()
{
#ifdef SIMULATE_PHYSICAL_INPUTS
  return InputSimulation::flowOk();
#else
  if (!isConfiguredPin(FLOW_SWITCH_PIN)) {
    // Stub mode before the input is wired: report "flow OK" while filtration runs.
    return FiltrationPump.IsRunning();
  }
  return digitalRead(FLOW_SWITCH_PIN) == FLOW_SWITCH_ACTIVE_STATE;
#endif
}

void setBridgeOutputs(bool enable, bool forward)
{
  if (enable) {
    if (!SWGPump.IsRunning()) {
      SWGPump.Start();
    }
  } else if (SWGPump.IsRunning()) {
    SWGPump.Stop();
  }

  if (!isConfiguredPin(IBT2_FWD_PIN) || !isConfiguredPin(IBT2_REV_PIN)) {
    return;
  }

  if (!enable) {
    digitalWrite(IBT2_FWD_PIN, LOW);
    digitalWrite(IBT2_REV_PIN, LOW);
  } else if (forward) {
    digitalWrite(IBT2_FWD_PIN, HIGH);
    digitalWrite(IBT2_REV_PIN, LOW);
  } else {
    digitalWrite(IBT2_FWD_PIN, LOW);
    digitalWrite(IBT2_REV_PIN, HIGH);
  }
}

void enterState(ElectrolysisRuntimeData& electrolysisState, ElectrolysisState nextState, unsigned long now)
{
  if (electrolysisState.state != nextState) {
    electrolysisState.state = nextState;
    electrolysisState.stateSinceMs = now;
  }
}
}

bool ElectrolysisFlowOk(void)
{
  if (!PMConfig.get<bool>(REQUIRE_FLOW_SWITCH)) {
    return true;
  }
  return readFlowSwitchRaw();
}

bool ElectrolysisPressureOk(void)
{
  const RunTimeData runtimeSnapshot = GetRunTimeDataSnapshot();
  if (!PMConfig.get<bool>(REQUIRE_PRESSURE_OK)) {
    return true;
  }

  if (PSIError) {
    return false;
  }

  return runtimeSnapshot.PSIValue >= PMConfig.get<double>(PSI_MEDTHRESHOLD) &&
         runtimeSnapshot.PSIValue <= PMConfig.get<double>(PSI_HIGHTHRESHOLD);
}

void ElectrolysisControl(void *pvParameters)
{
  RunTimeData runtimeSnapshot;
  ElectrolysisRuntimeData electrolysisSnapshot = GetElectrolysisDataSnapshot();

  while (!startTasks) ;
  vTaskDelay(DT12);

  TickType_t period = PT12;
  TickType_t ticktime = xTaskGetTickCount();
  unsigned long lastMillis = millis();
  bool nextForward = true;

  electrolysisSnapshot.bridgePresent = isConfiguredPin(IBT2_FWD_PIN) && isConfiguredPin(IBT2_REV_PIN);
  electrolysisSnapshot.flowSensorPresent = isConfiguredPin(FLOW_SWITCH_PIN);
  SetElectrolysisDataSnapshot(electrolysisSnapshot);

  for (;;)
  {
    const unsigned long now = millis();
    const unsigned long elapsedMs = now - lastMillis;
    lastMillis = now;

    const unsigned long startDelayMs = PMConfig.get<unsigned long>(ELECTROLYSIS_START_DELAY_S) * 1000UL;
    const unsigned long deadtimeMs = PMConfig.get<unsigned long>(ELECTROLYSIS_DEADTIME_S) * 1000UL;
    const unsigned long reverseIntervalMs = PMConfig.get<unsigned long>(ELECTROLYSIS_REVERSE_INTERVAL_MIN) * 60UL * 1000UL;
    const unsigned long windowMs = max(1UL, PMConfig.get<unsigned long>(ELECTROLYSIS_WINDOW_S) * 1000UL);

    runtimeSnapshot = GetRunTimeDataSnapshot();
    electrolysisSnapshot = GetElectrolysisDataSnapshot();

    electrolysisSnapshot.enabled = electrolysisEnabled();
    electrolysisSnapshot.flowOk = ElectrolysisFlowOk();
    electrolysisSnapshot.pressureOk = ElectrolysisPressureOk();
    electrolysisSnapshot.requestPct = runtimeSnapshot.OrpDemandPct;

    const bool filtrationRunning = FiltrationPump.IsRunning();
    const bool tempOk = runtimeSnapshot.WaterTemp >= PMConfig.get<double>(ELECTROLYSIS_MIN_TEMP_C);
    const bool requestActive = electrolysisSnapshot.requestPct > 0;
    const bool startDelayElapsed = (now - FiltrationPump.StartTime) >= startDelayMs;
    const bool hardFault = filtrationRunning && !electrolysisSnapshot.pressureOk;

    if (!filtrationRunning || !electrolysisSnapshot.enabled || !tempOk || !requestActive) {
      setBridgeOutputs(false, electrolysisSnapshot.polarityForward);
      electrolysisSnapshot.outputActive = false;
      electrolysisSnapshot.appliedPct = 0;
      electrolysisSnapshot.faultLatched = false;
      electrolysisSnapshot.polarityRunMs = 0;
      enterState(electrolysisSnapshot, ELECTROLYSIS_OFF, now);
      SetElectrolysisDataSnapshot(electrolysisSnapshot);
      vTaskDelayUntil(&ticktime, period);
      continue;
    }

    if (hardFault || SWGPump.UpTimeError) {
      setBridgeOutputs(false, electrolysisSnapshot.polarityForward);
      electrolysisSnapshot.outputActive = false;
      electrolysisSnapshot.appliedPct = 0;
      electrolysisSnapshot.faultLatched = true;
      enterState(electrolysisSnapshot, ELECTROLYSIS_FAULT, now);
      SetElectrolysisDataSnapshot(electrolysisSnapshot);
      vTaskDelayUntil(&ticktime, period);
      continue;
    }

    if (!startDelayElapsed) {
      setBridgeOutputs(false, electrolysisSnapshot.polarityForward);
      electrolysisSnapshot.outputActive = false;
      electrolysisSnapshot.appliedPct = 0;
      enterState(electrolysisSnapshot, ELECTROLYSIS_WAIT_FLOW, now);
      SetElectrolysisDataSnapshot(electrolysisSnapshot);
      vTaskDelayUntil(&ticktime, period);
      continue;
    }

    if (!electrolysisSnapshot.flowOk) {
      setBridgeOutputs(false, electrolysisSnapshot.polarityForward);
      electrolysisSnapshot.outputActive = false;
      electrolysisSnapshot.appliedPct = 0;
      enterState(electrolysisSnapshot, ELECTROLYSIS_OFF, now);
      SetElectrolysisDataSnapshot(electrolysisSnapshot);
      vTaskDelayUntil(&ticktime, period);
      continue;
    }

    if ((now - electrolysisSnapshot.windowStartMs) >= windowMs) {
      electrolysisSnapshot.windowStartMs = now;
    }

    if (electrolysisSnapshot.state == ELECTROLYSIS_FAULT) {
      electrolysisSnapshot.faultLatched = false;
      enterState(electrolysisSnapshot, ELECTROLYSIS_WAIT_FLOW, now);
    }

    if (electrolysisSnapshot.state == ELECTROLYSIS_OFF || electrolysisSnapshot.state == ELECTROLYSIS_WAIT_FLOW) {
      electrolysisSnapshot.polarityForward = nextForward;
      electrolysisSnapshot.polarityRunMs = 0;
      enterState(electrolysisSnapshot, electrolysisSnapshot.polarityForward ? ELECTROLYSIS_RUN_FWD : ELECTROLYSIS_RUN_REV, now);
    }

    if (electrolysisSnapshot.state == ELECTROLYSIS_DEADTIME) {
      setBridgeOutputs(false, electrolysisSnapshot.polarityForward);
      electrolysisSnapshot.outputActive = false;
      electrolysisSnapshot.appliedPct = 0;

      if ((now - electrolysisSnapshot.stateSinceMs) >= deadtimeMs) {
        electrolysisSnapshot.polarityForward = nextForward;
        electrolysisSnapshot.polarityRunMs = 0;
        enterState(electrolysisSnapshot, electrolysisSnapshot.polarityForward ? ELECTROLYSIS_RUN_FWD : ELECTROLYSIS_RUN_REV, now);
      }

      SetElectrolysisDataSnapshot(electrolysisSnapshot);
      vTaskDelayUntil(&ticktime, period);
      continue;
    }

    if (electrolysisSnapshot.state == ELECTROLYSIS_RUN_FWD || electrolysisSnapshot.state == ELECTROLYSIS_RUN_REV) {
      const unsigned long onTimeMs = (windowMs * (unsigned long)electrolysisSnapshot.requestPct) / 100UL;
      const bool shouldBeOn = (now - electrolysisSnapshot.windowStartMs) < onTimeMs;

      if (shouldBeOn) {
        setBridgeOutputs(true, electrolysisSnapshot.state == ELECTROLYSIS_RUN_FWD);
        electrolysisSnapshot.outputActive = true;
        electrolysisSnapshot.appliedPct = electrolysisSnapshot.requestPct;
        electrolysisSnapshot.polarityRunMs += elapsedMs;
      } else {
        setBridgeOutputs(false, electrolysisSnapshot.state == ELECTROLYSIS_RUN_FWD);
        electrolysisSnapshot.outputActive = false;
        electrolysisSnapshot.appliedPct = 0;
      }

      if (reverseIntervalMs > 0 && electrolysisSnapshot.polarityRunMs >= reverseIntervalMs) {
        setBridgeOutputs(false, electrolysisSnapshot.state == ELECTROLYSIS_RUN_FWD);
        electrolysisSnapshot.outputActive = false;
        electrolysisSnapshot.appliedPct = 0;
        nextForward = (electrolysisSnapshot.state == ELECTROLYSIS_RUN_FWD) ? false : true;
        enterState(electrolysisSnapshot, ELECTROLYSIS_DEADTIME, now);
      }
    }

    SetElectrolysisDataSnapshot(electrolysisSnapshot);
    vTaskDelayUntil(&ticktime, period);
  }
}
