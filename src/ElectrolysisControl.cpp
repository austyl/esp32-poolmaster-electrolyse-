#include <Arduino.h>
#include <math.h>
#include "Config.h"
#include "PoolMaster.h"

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
  if (!isConfiguredPin(FLOW_SWITCH_PIN)) {
    // Stub mode before the input is wired: report "flow OK" while filtration runs.
    return FiltrationPump.IsRunning();
  }
  return digitalRead(FLOW_SWITCH_PIN) == FLOW_SWITCH_ACTIVE_STATE;
}

double readCellCurrentA()
{
  if (!isConfiguredPin(CELL_CURRENT_PIN)) {
    return 0.0;
  }
  const int raw = analogRead(CELL_CURRENT_PIN);
  return ((double)raw / CELL_CURRENT_ADC_MAX) * CELL_CURRENT_MAX_A;
}

bool criticalSensorsValid()
{
  return isfinite(PMData.OrpValue) && isfinite(PMData.PSIValue) && isfinite(PMData.WaterTemp);
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

void enterState(ElectrolysisState nextState, unsigned long now)
{
  if (ElectrolysisData.state != nextState) {
    ElectrolysisData.state = nextState;
    ElectrolysisData.stateSinceMs = now;
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
  if (!PMConfig.get<bool>(REQUIRE_PRESSURE_OK)) {
    return true;
  }

  if (PSIError) {
    return false;
  }

  return PMData.PSIValue >= PMConfig.get<double>(PSI_MEDTHRESHOLD) &&
         PMData.PSIValue <= PMConfig.get<double>(PSI_HIGHTHRESHOLD);
}

void ElectrolysisControl(void *pvParameters)
{
  while (!startTasks) ;
  vTaskDelay(DT12);

  TickType_t period = PT12;
  TickType_t ticktime = xTaskGetTickCount();
  unsigned long lastMillis = millis();
  bool nextForward = true;

  ElectrolysisData.bridgePresent = isConfiguredPin(IBT2_FWD_PIN) && isConfiguredPin(IBT2_REV_PIN);
  ElectrolysisData.flowSensorPresent = isConfiguredPin(FLOW_SWITCH_PIN);
  ElectrolysisData.currentSensorPresent = isConfiguredPin(CELL_CURRENT_PIN);

  for (;;)
  {
    const unsigned long now = millis();
    const unsigned long elapsedMs = now - lastMillis;
    lastMillis = now;

    const unsigned long startDelayMs = PMConfig.get<unsigned long>(ELECTROLYSIS_START_DELAY_S) * 1000UL;
    const unsigned long deadtimeMs = PMConfig.get<unsigned long>(ELECTROLYSIS_DEADTIME_S) * 1000UL;
    const unsigned long reverseIntervalMs = PMConfig.get<unsigned long>(ELECTROLYSIS_REVERSE_INTERVAL_MIN) * 60UL * 1000UL;
    const unsigned long windowMs = max(1UL, PMConfig.get<unsigned long>(ELECTROLYSIS_WINDOW_S) * 1000UL);

    ElectrolysisData.enabled = electrolysisEnabled();
    ElectrolysisData.flowOk = ElectrolysisFlowOk();
    ElectrolysisData.pressureOk = ElectrolysisPressureOk();
    ElectrolysisData.requestPct = PMData.OrpDemandPct;
    ElectrolysisData.cellCurrentA = readCellCurrentA();

    const bool filtrationRunning = FiltrationPump.IsRunning();
    const bool tempOk = PMData.WaterTemp >= PMConfig.get<double>(ELECTROLYSIS_MIN_TEMP_C);
    const bool requestActive = ElectrolysisData.requestPct > 0;
    const bool startDelayElapsed = (now - FiltrationPump.StartTime) >= startDelayMs;
    const bool probesOk = criticalSensorsValid();

    ElectrolysisData.currentOk = true;
    if (ElectrolysisData.currentSensorPresent && ElectrolysisData.outputActive) {
      const double minCurrentA = PMConfig.get<double>(ELECTROLYSIS_CURRENT_MIN_A);
      const double maxCurrentA = PMConfig.get<double>(ELECTROLYSIS_CURRENT_MAX_A);
      ElectrolysisData.currentOk =
          (ElectrolysisData.cellCurrentA >= minCurrentA) &&
          (ElectrolysisData.cellCurrentA <= maxCurrentA);
    }

    ElectrolysisData.faultFlags = EL_FAULT_NONE;
    if (!filtrationRunning) ElectrolysisData.faultFlags |= EL_FAULT_PUMP_OFF;
    if (!ElectrolysisData.flowOk) ElectrolysisData.faultFlags |= EL_FAULT_FLOW;
    if (!ElectrolysisData.pressureOk) ElectrolysisData.faultFlags |= EL_FAULT_PRESSURE;
    if (!tempOk) ElectrolysisData.faultFlags |= EL_FAULT_TEMP;
    if (!probesOk) ElectrolysisData.faultFlags |= EL_FAULT_SENSORS;
    if (!ElectrolysisData.bridgePresent) ElectrolysisData.faultFlags |= EL_FAULT_BRIDGE;
    if (SWGPump.UpTimeError) ElectrolysisData.faultFlags |= EL_FAULT_RUNTIME;
    if (!ElectrolysisData.currentOk) ElectrolysisData.faultFlags |= EL_FAULT_CURRENT;

    const bool blockingFault = (ElectrolysisData.faultFlags &
                              (EL_FAULT_PRESSURE | EL_FAULT_SENSORS | EL_FAULT_RUNTIME | EL_FAULT_CURRENT)) != 0;
    const bool waitFlowOrDelay = !ElectrolysisData.flowOk || !startDelayElapsed;

    if (!filtrationRunning || !ElectrolysisData.enabled || !tempOk || !requestActive) {
      setBridgeOutputs(false, ElectrolysisData.polarityForward);
      ElectrolysisData.outputActive = false;
      ElectrolysisData.appliedPct = 0;
      ElectrolysisData.faultLatched = false;
      ElectrolysisData.polarityRunMs = 0;
      enterState(ELECTROLYSIS_OFF, now);
      vTaskDelayUntil(&ticktime, period);
      continue;
    }

    if (blockingFault) {
      setBridgeOutputs(false, ElectrolysisData.polarityForward);
      ElectrolysisData.outputActive = false;
      ElectrolysisData.appliedPct = 0;
      ElectrolysisData.faultLatched = true;
      enterState(ELECTROLYSIS_FAULT, now);
      vTaskDelayUntil(&ticktime, period);
      continue;
    }

    if (waitFlowOrDelay) {
      setBridgeOutputs(false, ElectrolysisData.polarityForward);
      ElectrolysisData.outputActive = false;
      ElectrolysisData.appliedPct = 0;
      enterState(ELECTROLYSIS_WAIT_FLOW, now);
      vTaskDelayUntil(&ticktime, period);
      continue;
    }

    if ((now - ElectrolysisData.windowStartMs) >= windowMs) {
      ElectrolysisData.windowStartMs = now;
    }

    if (ElectrolysisData.state == ELECTROLYSIS_FAULT) {
      ElectrolysisData.faultLatched = false;
      enterState(ELECTROLYSIS_WAIT_FLOW, now);
    }

    if (ElectrolysisData.state == ELECTROLYSIS_OFF || ElectrolysisData.state == ELECTROLYSIS_WAIT_FLOW) {
      ElectrolysisData.polarityForward = nextForward;
      ElectrolysisData.polarityRunMs = 0;
      enterState(ElectrolysisData.polarityForward ? ELECTROLYSIS_RUN_FWD : ELECTROLYSIS_RUN_REV, now);
    }

    if (ElectrolysisData.state == ELECTROLYSIS_DEADTIME) {
      setBridgeOutputs(false, ElectrolysisData.polarityForward);
      ElectrolysisData.outputActive = false;
      ElectrolysisData.appliedPct = 0;

      if ((now - ElectrolysisData.stateSinceMs) >= deadtimeMs) {
        ElectrolysisData.polarityForward = nextForward;
        ElectrolysisData.polarityRunMs = 0;
        enterState(ElectrolysisData.polarityForward ? ELECTROLYSIS_RUN_FWD : ELECTROLYSIS_RUN_REV, now);
      }

      vTaskDelayUntil(&ticktime, period);
      continue;
    }

    if (ElectrolysisData.state == ELECTROLYSIS_RUN_FWD || ElectrolysisData.state == ELECTROLYSIS_RUN_REV) {
      const unsigned long onTimeMs = (windowMs * (unsigned long)ElectrolysisData.requestPct) / 100UL;
      const bool shouldBeOn = (now - ElectrolysisData.windowStartMs) < onTimeMs;

      if (shouldBeOn) {
        setBridgeOutputs(true, ElectrolysisData.state == ELECTROLYSIS_RUN_FWD);
        ElectrolysisData.outputActive = true;
        ElectrolysisData.appliedPct = ElectrolysisData.requestPct;
        ElectrolysisData.polarityRunMs += elapsedMs;
      } else {
        setBridgeOutputs(false, ElectrolysisData.state == ELECTROLYSIS_RUN_FWD);
        ElectrolysisData.outputActive = false;
        ElectrolysisData.appliedPct = 0;
      }

      if (reverseIntervalMs > 0 && ElectrolysisData.polarityRunMs >= reverseIntervalMs) {
        setBridgeOutputs(false, ElectrolysisData.state == ELECTROLYSIS_RUN_FWD);
        ElectrolysisData.outputActive = false;
        ElectrolysisData.appliedPct = 0;
        nextForward = (ElectrolysisData.state == ELECTROLYSIS_RUN_FWD) ? false : true;
        enterState(ELECTROLYSIS_DEADTIME, now);
      }
    }

    vTaskDelayUntil(&ticktime, period);
  }
}
