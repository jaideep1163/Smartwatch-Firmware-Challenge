// Smartwatch firmware - Task 1: Pulse Monitoring
// Board: Raspberry Pi Pico (RP2040), Arduino-Pico core, Wokwi
//
// Rules honoured: all mutable state lives in EngineState (the single global),
// no malloc/calloc/free, no float/double, no ISRs used in this task.

#include <Arduino.h>
#include <stdint.h>

// ---------------- Pins (constants only) ----------------
constexpr uint8_t PIN_PPG   = 26;  // GP26 / ADC0
constexpr uint8_t PIN_CROWN = 14;  // GP14 (Task: brownout)
constexpr uint8_t PIN_TOUCH = 15;  // GP15 (Task: tap / long-press)
constexpr uint8_t PIN_LED_G = 16;  // heartbeat
constexpr uint8_t PIN_LED_R = 17;  // pulse spike
constexpr uint8_t PIN_LED_B = 18;  // SOS

// ---------------- Tunables (constants only) ----------------
constexpr uint8_t  WINDOW           = 16;       // 16 x 2 B = 32 B
constexpr uint16_t RUN_SAMPLES      = 2000;
constexpr uint32_t SAMPLE_PERIOD_US = 10000;    // 100 Hz
constexpr uint16_t MIN_SPIKE_SPAN   = 1500;     // ignore noise-level spans
constexpr uint32_t SPIKE_LED_MS     = 80;
constexpr uint32_t HEARTBEAT_MS     = 500;      // green toggles every 500 ms
constexpr uint8_t  FLOOR_SHIFT      = 3;        // floor EMA alpha = 1/8
constexpr uint16_t PRINT_EVERY      = 100;      // samples between Serial lines
#define USE_SYNTHETIC_PULSE 1                   // add fake 72 BPM wave to the pot

// ---------------- Engine state: THE ONLY mutable global ----------------
struct EngineState {
  // --- Task 1: pulse engine ---
  uint16_t pulseBuf[WINDOW];   // raw 16-bit PPG ring buffer (32 bytes)
  uint16_t peak;               // rolling peak (max of window)
  uint16_t floorAvg;           // rolling average floor
  uint32_t floorQ;             // floor accumulator, fixed point (avg << FLOOR_SHIFT)
  uint16_t sampleCount;        // samples in current 2000-sample run
  uint16_t runsDone;           // completed 2000-sample runs
  uint8_t  head;               // ring write index
  uint8_t  filled;             // valid entries in ring (0..WINDOW)
  uint8_t  aboveThresh;        // spike edge detector

  // --- timestamps (no delay()) ---
  uint32_t nextSampleUs;       // next sample deadline
  uint32_t redOffAtMs;         // 0 = red LED idle
  uint32_t nextBeatMs;         // next green toggle

  // --- LED output flags ---
  uint8_t  ledG, ledR, ledB;

  // --- reserved for later tasks ---
  uint8_t  mode;               // 0 = NORMAL (later: BROWNOUT, SOS)
  uint8_t  flags;              // ISR -> main-loop event bits
};

static_assert(sizeof(EngineState) <= 512, "EngineState too large");
static_assert(sizeof(EngineState::pulseBuf) < 64, "Raw pulse buffer must be < 64 bytes");

static EngineState S;

// ---------------- Pulse engine ----------------
static void pulseReset(EngineState &s) {
  memset(&s, 0, sizeof(s));
}

// Feed ONE raw 16-bit sample. Returns 1 on a pulse-spike rising edge.
static uint8_t pulseProcess(EngineState &s, uint16_t sample) {
  s.pulseBuf[s.head] = sample;
  s.head = (uint8_t)((s.head + 1) & (WINDOW - 1));   // WINDOW is a power of 2
  if (s.filled < WINDOW) s.filled++;

  uint16_t mx = 0, mn = 0xFFFF;
  for (uint8_t i = 0; i < s.filled; i++) {
    uint16_t v = s.pulseBuf[i];
    if (v > mx) mx = v;
    if (v < mn) mn = v;
  }
  s.peak = mx;

  // rolling average floor: EMA of window minimum, integer only
  if (s.filled == 1) {
    s.floorQ = (uint32_t)mn << FLOOR_SHIFT;
  } else {
    int32_t target = (int32_t)((uint32_t)mn << FLOOR_SHIFT);
    int32_t cur    = (int32_t)s.floorQ;
    s.floorQ = (uint32_t)(cur + ((target - cur) >> FLOOR_SHIFT));
  }
  s.floorAvg = (uint16_t)(s.floorQ >> FLOOR_SHIFT);

  // spike = sample enters top quarter of the (peak - floor) span
  uint8_t spike = 0;
  uint32_t span = (s.peak > s.floorAvg) ? (uint32_t)(s.peak - s.floorAvg) : 0;
  if (s.filled == WINDOW && span >= MIN_SPIKE_SPAN) {
    uint32_t thresh = (uint32_t)s.floorAvg + (span * 3u >> 2);
    uint8_t above = (sample >= thresh);
    spike = (uint8_t)(above && !s.aboveThresh);
    s.aboveThresh = above;
  } else {
    s.aboveThresh = 0;
  }

  s.sampleCount++;
  return spike;
}

// ---------------- Hardware access ----------------
static uint16_t readPPG16(uint32_t nowMs) {
  uint32_t v = (uint32_t)analogRead(PIN_PPG) << 4;   // 12-bit -> 16-bit
#if USE_SYNTHETIC_PULSE
  uint32_t phase = nowMs % 833;                       // ~72 BPM
  uint32_t wave = (phase < 120) ? phase * 60u
                : (phase < 300) ? (300u - phase) * 40u
                : 0u;
  v = (v >> 1) + wave;
  if (v > 65535u) v = 65535u;
#else
  (void)nowMs;
#endif
  return (uint16_t)v;
}

static void ledWrite(const EngineState &s) {
  digitalWrite(PIN_LED_G, s.ledG);
  digitalWrite(PIN_LED_R, s.ledR);
  digitalWrite(PIN_LED_B, s.ledB);
}

// ---------------- Tasks (called from loop) ----------------
static void taskPulse(EngineState &s, uint32_t nowUs, uint32_t nowMs) {
  if ((int32_t)(nowUs - s.nextSampleUs) < 0) return;
  s.nextSampleUs += SAMPLE_PERIOD_US;

  uint16_t sample = readPPG16(nowMs);
  if (pulseProcess(s, sample)) {
    s.ledR = 1;
    s.redOffAtMs = nowMs + SPIKE_LED_MS;
    if (s.redOffAtMs == 0) s.redOffAtMs = 1;
  }

  if ((s.sampleCount % PRINT_EVERY) == 0) {
    Serial.print(F("n="));      Serial.print(s.sampleCount);
    Serial.print(F(" raw="));   Serial.print(sample);
    Serial.print(F(" peak="));  Serial.print(s.peak);
    Serial.print(F(" floor=")); Serial.print(s.floorAvg);
    Serial.print(F(" amp="));   Serial.println((uint16_t)(s.peak - s.floorAvg));
  }

  if (s.sampleCount >= RUN_SAMPLES) {
    s.runsDone++;
    Serial.print(F("== run ")); Serial.print(s.runsDone);
    Serial.print(F(" done: 2000 samples, peak=")); Serial.print(s.peak);
    Serial.print(F(" floor=")); Serial.println(s.floorAvg);
    s.sampleCount = 0;   // keep window + rolling values: monitoring is continuous
  }
}

static void taskLeds(EngineState &s, uint32_t nowMs) {
  if (s.ledR && s.redOffAtMs && (int32_t)(nowMs - s.redOffAtMs) >= 0) {
    s.ledR = 0;
    s.redOffAtMs = 0;
  }
  if ((int32_t)(nowMs - s.nextBeatMs) >= 0) {
    s.ledG ^= 1;
    s.nextBeatMs = nowMs + HEARTBEAT_MS;
  }
  ledWrite(s);
}

// Stubs for later tasks: keep the loop structure fixed so they drop in.
static void taskInputs(EngineState &s, uint32_t nowMs) { (void)s; (void)nowMs; }  // tap / long-press, crown

// ---------------- Arduino entry points ----------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);              // arduino-pico defaults to 10-bit
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_CROWN, INPUT_PULLUP);
  pinMode(PIN_TOUCH, INPUT_PULLUP);

  pulseReset(S);
  S.nextSampleUs = micros();
  S.nextBeatMs   = millis();

  Serial.print(F("sizeof(EngineState)="));    Serial.println(sizeof(EngineState));
  Serial.print(F("sizeof(pulseBuf)="));       Serial.println(sizeof(S.pulseBuf));
}

void loop() {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();
  taskPulse(S, nowUs, nowMs);
  taskInputs(S, nowMs);
  taskLeds(S, nowMs);
}
