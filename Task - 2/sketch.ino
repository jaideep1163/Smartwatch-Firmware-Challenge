// Smartwatch firmware - Task 2 entry points.
// Needs engine_task2.h in the same project (add it as a new file in Wokwi).
#include "engine_task2.h"

// ---------------- Arduino entry points ----------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_CROWN, INPUT_PULLUP);
  pinMode(PIN_TOUCH, INPUT_PULLUP);

  restoreFromCheckpoint(S);            // resume or start fresh

  Serial.print("sizeof(EngineState)="); Serial.println(sizeof(EngineState));
  Serial.print("sizeof(pulseBuf)=");    Serial.println(sizeof(S.pulseBuf));

  attachInterrupt(PIN_CROWN, crownISR, FALLING);
}

void loop() {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();

  if (S.mode == MODE_HALTED) { taskHalted(S, nowMs); return; }

  if (S.flags & FLAG_CROWN) {
    S.flags = 0;
    if ((uint32_t)(nowMs - S.bootAtMs) > 300) {    // ignore button bounce right after a restart
      brownout(S, nowMs);
      return;
    }
  }

  taskPulse(S, nowUs, nowMs);
  taskLeds(S, nowMs);
}
