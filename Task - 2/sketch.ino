// Smartwatch firmware - Tasks 1-2 (cumulative)
// Board: Raspberry Pi Pico (RP2040), Arduino-Pico core, Wokwi
//
//  Task 1  pulse monitoring   : 2000 samples, rolling peak + rolling floor/average
//  Task 2  brownout recovery  : crown button -> checkpoint -> halt -> reset -> resume
//          (LEDs still use the simple Task 1 behaviour: 500 ms green blink,
//           red flash on a pulse spike. Task 3 replaces these rules.)
//
// Rules: ONE mutable global (EngineState S), no malloc/calloc/free, no float/double,
// ISR only sets a flag.

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#ifndef __uninitialized_ram
#define __uninitialized_ram(group) __attribute__((section(".uninitialized_data." #group))) group
#endif

// ---------------- Pins ----------------
constexpr uint8_t PIN_PPG   = 26;  // GP26 / ADC0
constexpr uint8_t PIN_CROWN = 14;  // GP14 side crown (brownout)
constexpr uint8_t PIN_TOUCH = 15;  // GP15 front touch key (later task)
constexpr uint8_t PIN_LED_G = 16;
constexpr uint8_t PIN_LED_R = 17;
constexpr uint8_t PIN_LED_B = 18;

// ---------------- Constants ----------------
constexpr uint8_t  WINDOW           = 16;       // 16 x 2 B = 32 B raw buffer
constexpr uint16_t RUN_SAMPLES      = 2000;
constexpr uint32_t SAMPLE_PERIOD_US = 10000;    // 100 Hz
constexpr uint8_t  FLOOR_SHIFT      = 3;        // floor EMA alpha 1/8
constexpr uint8_t  AVG_SHIFT        = 6;        // running average alpha 1/64
constexpr uint16_t MIN_SPIKE_SPAN   = 1500;
constexpr uint32_t SPIKE_LED_MS     = 80;
constexpr uint32_t HEARTBEAT_MS     = 500;
constexpr uint16_t PRINT_EVERY      = 100;

constexpr uint8_t MODE_ACTIVE = 0;
constexpr uint8_t MODE_HALTED = 2;

constexpr uint8_t  FLAG_CROWN = 0x01;
constexpr uint32_t CKPT_MAGIC = 0xC0FFEE42u;

// ---------------- Checkpoint (lives inside EngineState) ----------------
struct Checkpoint {
  uint32_t tag;          // validation tag, derived from the fields below
  uint16_t sampleIndex;
  uint16_t rollingAvg;
  uint16_t floorAvg;
  uint16_t reserved;
};

static uint32_t ckptTag(const Checkpoint &c) {
  return CKPT_MAGIC ^ (((uint32_t)c.sampleIndex << 16) | c.rollingAvg)
                    ^ ((uint32_t)c.floorAvg * 2654435761u);
}
static bool ckptValid(const Checkpoint &c) {
  return c.tag == ckptTag(c) && c.sampleIndex < RUN_SAMPLES;
}

// ---------------- Engine state: THE ONLY mutable global ----------------
struct EngineState {
  uint16_t pulseBuf[WINDOW];   // raw 16-bit PPG ring buffer (32 bytes)

  uint32_t floorQ;
  uint32_t avgQ;
  uint32_t nextSampleUs;
  uint32_t redOffAtMs;
  uint32_t nextBeatMs;
  uint32_t haltAt;

  Checkpoint ckpt;

  uint16_t peak;
  uint16_t floorAvg;
  uint16_t runAvg;
  uint16_t sampleCount;
  uint16_t runsDone;

  uint8_t  head, filled, started, aboveThresh;
  uint8_t  ledG, ledR, ledB;
  uint8_t  mode;
  uint8_t  crownReleased;
  volatile uint8_t flags;
};

static_assert(sizeof(EngineState) <= 512, "EngineState too large");
static_assert(sizeof(EngineState::pulseBuf) < 64, "Raw pulse buffer must be < 64 bytes");

// NOLOAD RAM section: not cleared by startup code, survives a watchdog reboot.
static EngineState __uninitialized_ram(S);

// ---------------- Helpers ----------------
static inline void wr(uint8_t pin, uint8_t v) { digitalWrite(pin, v ? HIGH : LOW); }
static void engineReset(EngineState &s) { memset((void *)&s, 0, sizeof(s)); }

// ---------------- ISR (non-blocking) ----------------
static void crownISR() { S.flags |= FLAG_CROWN; }

// ---------------- Pulse engine ----------------
// Returns 1 on a pulse-spike rising edge (sample enters top quarter of the span).
static uint8_t pulseProcess(EngineState &s, uint16_t sample) {
  s.pulseBuf[s.head] = sample;
  s.head = (uint8_t)((s.head + 1) & (WINDOW - 1));
  if (s.filled < WINDOW) s.filled++;

  uint16_t mx = 0, mn = 0xFFFF;
  for (uint8_t i = 0; i < s.filled; i++) {
    uint16_t v = s.pulseBuf[i];
    if (v > mx) mx = v;
    if (v < mn) mn = v;
  }
  s.peak = mx;

  if (!s.started) {
    s.floorQ = (uint32_t)mn << FLOOR_SHIFT;
    s.avgQ   = (uint32_t)sample << AVG_SHIFT;
    s.started = 1;
  } else {
    int32_t ft = (int32_t)((uint32_t)mn << FLOOR_SHIFT);
    s.floorQ = (uint32_t)((int32_t)s.floorQ + ((ft - (int32_t)s.floorQ) >> FLOOR_SHIFT));
    int32_t at = (int32_t)((uint32_t)sample << AVG_SHIFT);
    s.avgQ = (uint32_t)((int32_t)s.avgQ + ((at - (int32_t)s.avgQ) >> AVG_SHIFT));
  }
  s.floorAvg = (uint16_t)(s.floorQ >> FLOOR_SHIFT);
  s.runAvg   = (uint16_t)(s.avgQ >> AVG_SHIFT);

  uint8_t spike = 0;
  uint32_t span = (s.peak > s.floorAvg) ? (uint32_t)(s.peak - s.floorAvg) : 0;
  if (s.filled == WINDOW && span >= MIN_SPIKE_SPAN) {
    uint32_t thresh = (uint32_t)s.floorAvg + ((span * 3u) >> 2);
    uint8_t above = (sample >= thresh);
    spike = (uint8_t)(above && !s.aboveThresh);
    s.aboveThresh = above;
  } else {
    s.aboveThresh = 0;
  }

  s.sampleCount++;
  return spike;
}

static uint16_t readPPG16(uint32_t nowMs) {
  uint32_t v = (uint32_t)analogRead(PIN_PPG) << 4;     // 12-bit -> 16-bit
  uint32_t phase = nowMs % 833;                        // synthetic ~72 BPM pulse
  uint32_t wave = (phase < 120) ? phase * 60u
                : (phase < 300) ? (300u - phase) * 40u
                : 0u;
  v = (v >> 1) + wave;
  if (v > 65535u) v = 65535u;
  return (uint16_t)v;
}

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
    Serial.print("n=");       Serial.print(s.sampleCount);
    Serial.print(" raw=");    Serial.print(sample);
    Serial.print(" peak=");   Serial.print(s.peak);
    Serial.print(" floor=");  Serial.print(s.floorAvg);
    Serial.print(" avg=");    Serial.println(s.runAvg);
  }
  if (s.sampleCount >= RUN_SAMPLES) {
    s.runsDone++;
    Serial.print("== run "); Serial.print(s.runsDone);
    Serial.print(" done (2000 samples) peak="); Serial.print(s.peak);
    Serial.print(" floor="); Serial.println(s.floorAvg);
    s.sampleCount = 0;
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
  wr(PIN_LED_G, s.ledG);
  wr(PIN_LED_R, s.ledR);
  wr(PIN_LED_B, s.ledB);
}

// ---------------- Task 2: brownout recovery ----------------
static void ckptSave(EngineState &s) {
  s.ckpt.sampleIndex = s.sampleCount;
  s.ckpt.rollingAvg  = s.runAvg;
  s.ckpt.floorAvg    = s.floorAvg;
  s.ckpt.reserved    = 0;
  s.ckpt.tag         = ckptTag(s.ckpt);
}

static void brownout(EngineState &s, uint32_t nowMs) {
  ckptSave(s);
  s.mode = MODE_HALTED;
  s.ledG = s.ledR = s.ledB = 0;
  wr(PIN_LED_G, 0); wr(PIN_LED_R, 0); wr(PIN_LED_B, 0);
  s.haltAt = nowMs;
  s.crownReleased = 0;
  Serial.print("!! BROWNOUT: checkpoint saved, sample=");
  Serial.print(s.ckpt.sampleIndex);
  Serial.print(" avg="); Serial.print(s.ckpt.rollingAvg);
  Serial.println(" -> HALTED. Release, then press crown again to restore power.");
}

static void taskHalted(EngineState &s, uint32_t nowMs) {
  uint8_t pressed = (digitalRead(PIN_CROWN) == LOW);
  if (!s.crownReleased) {
    if (!pressed && (uint32_t)(nowMs - s.haltAt) >= 200) s.crownReleased = 1;
  } else if (pressed) {
    Serial.println("Power restored: rebooting...");
    Serial.flush();
    rp2040.reboot();                    // watchdog reboot: RAM contents survive
  }
}

// ---------------- Arduino entry points ----------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_CROWN, INPUT_PULLUP);
  pinMode(PIN_TOUCH, INPUT_PULLUP);

  Checkpoint saved = S.ckpt;           // read BEFORE wiping the state
  bool valid = ckptValid(saved);

  engineReset(S);
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

  Serial.print("sizeof(EngineState)="); Serial.println(sizeof(EngineState));
  Serial.print("sizeof(pulseBuf)=");    Serial.println(sizeof(S.pulseBuf));

  S.nextSampleUs = micros();
  S.nextBeatMs   = millis();
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

  taskPulse(S, nowUs, nowMs);
  taskLeds(S, nowMs);
}
