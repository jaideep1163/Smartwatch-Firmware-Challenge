// Smartwatch firmware - Task 4 entry points.
// Needs engine_task4.h in the same project (add it as a new file in Wokwi).
#include "engine_task4.h"

// ---------------- Arduino entry points ----------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_CROWN, INPUT_PULLUP);
  pinMode(PIN_TOUCH, INPUT_PULLUP);

  // Read the checkpoint BEFORE wiping the state
  Checkpoint saved = S.ckpt;
  bool valid = ckptValid(saved);

  engineReset(S);                      // also invalidates the old checkpoint
  S.mode = MODE_ACTIVE;
  if (valid) {
    S.sampleCount = saved.sampleIndex;
    S.runAvg   = saved.rollingAvg;
    S.avgQ     = (uint32_t)saved.rollingAvg << AVG_SHIFT;
    S.floorAvg = saved.floorAvg;
    S.floorQ   = (uint32_t)saved.floorAvg << FLOOR_SHIFT;
    S.started  = 1;
    Serial.print("[BOOT] checkpoint VALID -> resuming from sample ");
    Serial.println(saved.sampleIndex);
  } else {
    Serial.println("[BOOT] checkpoint INVALID -> starting from sample 0");
  }

  memReport();

  S.nextSampleUs = micros();
  attachInterrupt(PIN_CROWN, crownISR, FALLING);
}

void loop() {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();

  if (S.mode == MODE_HALTED) { taskHalted(S, nowMs); return; }

  if (S.flags & FLAG_CROWN) {
    S.flags = 0;
    brownout(S, nowMs);
    return;
  }

  taskTouch(S, nowMs);
  taskPulse(S, nowUs, nowMs);
  ledOutput(S);
}
