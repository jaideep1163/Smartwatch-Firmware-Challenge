#pragma once
// Smartwatch firmware - Tasks 1-3 (cumulative)
// Board: Raspberry Pi Pico (RP2040), Arduino-Pico core, Wokwi
//
//  Task 1  pulse monitoring   : 2000 samples, rolling peak + rolling floor/average
//  Task 2  brownout recovery  : crown button -> checkpoint -> halt -> reset -> resume
//  Task 3  indicators         : green toggles every 50 samples,
//                               red ON when sample >= 150% of running average
//
// Rules: ONE mutable global (EngineState S), no malloc/calloc/free, no float/double,
// ISR only sets a flag.

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

// 0 = restart in place after a brownout (works in Wokwi)
// 1 = real watchdog reset via rp2040.reboot() (EngineState kept in no-init RAM)
#define USE_HW_REBOOT 0

#ifndef __uninitialized_ram
#define __uninitialized_ram(group) __attribute__((section(".uninitialized_data." #group))) group
#endif

// ---------------- Pins ----------------
constexpr uint8_t PIN_PPG   = 26;  // GP26 / ADC0
constexpr uint8_t PIN_CROWN = 14;  // GP14 side crown (brownout)
constexpr uint8_t PIN_TOUCH = 15;  // GP15 front touch key (Task 4)
constexpr uint8_t PIN_LED_G = 16;
constexpr uint8_t PIN_LED_R = 17;
constexpr uint8_t PIN_LED_B = 18;

// ---------------- Constants ----------------
constexpr uint8_t  WINDOW           = 16;       // 16 x 2 B = 32 B raw buffer
constexpr uint16_t RUN_SAMPLES      = 2000;
constexpr uint32_t SAMPLE_PERIOD_US = 10000;    // 100 Hz
constexpr uint8_t  FLOOR_SHIFT      = 3;        // floor EMA alpha 1/8
constexpr uint8_t  AVG_SHIFT        = 6;        // running average alpha 1/64
constexpr uint16_t GREEN_EVERY      = 50;
constexpr uint16_t MIN_PULSE        = 1000;     // red ignores near-zero signal
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
  uint32_t haltAt;
  uint32_t bootAtMs;           // time of the last (re)start

  Checkpoint ckpt;

  uint16_t peak;
  uint16_t floorAvg;
  uint16_t runAvg;
  uint16_t sampleCount;
  uint16_t runsDone;

  uint8_t  head, filled, started;
  uint8_t  ledG, ledR, ledB;
  uint8_t  mode;
  uint8_t  crownReleased;
  volatile uint8_t flags;
};

static_assert(sizeof(EngineState) <= 512, "EngineState too large");
static_assert(sizeof(EngineState::pulseBuf) < 64, "Raw pulse buffer must be < 64 bytes");

#if USE_HW_REBOOT
static EngineState __uninitialized_ram(S);
#else
static EngineState S;
#endif

// ---------------- Helpers ----------------
static inline void wr(uint8_t pin, uint8_t v) { digitalWrite(pin, v ? HIGH : LOW); }
static void engineReset(EngineState &s) { memset((void *)&s, 0, sizeof(s)); }

// ---------------- ISR (non-blocking) ----------------
static void crownISR() { S.flags |= FLAG_CROWN; }

// ---------------- Pulse engine + indicators ----------------
static void pulseProcess(EngineState &s, uint16_t sample) {
  // Red: sample vs running average BEFORE folding it in. ON at >= avg * 1.5, OFF below.
  uint32_t thresh = (uint32_t)s.runAvg + ((uint32_t)s.runAvg >> 1);
  s.ledR = (s.started && sample >= thresh && sample >= MIN_PULSE) ? 1 : 0;

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

  s.sampleCount++;
  // Green: toggle every 50 processed samples
  if ((s.sampleCount % GREEN_EVERY) == 0) s.ledG ^= 1;
}

static uint16_t readPPG16(uint32_t nowMs) {
  uint32_t v = (uint32_t)analogRead(PIN_PPG) << 4;     // 12-bit -> 16-bit
  v >>= 3;                                             // baseline from the pot
  uint32_t phase = nowMs % 833;                        // synthetic ~72 BPM pulse
  uint32_t wave = (phase < 120) ? phase * 60u
                : (phase < 300) ? (300u - phase) * 40u
                : 0u;
  v += wave;
  if (v > 65535u) v = 65535u;
  return (uint16_t)v;
}

static void taskPulse(EngineState &s, uint32_t nowUs, uint32_t nowMs) {
  if ((int32_t)(nowUs - s.nextSampleUs) < 0) return;
  s.nextSampleUs += SAMPLE_PERIOD_US;

  uint16_t sample = readPPG16(nowMs);
  pulseProcess(s, sample);

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

// Re-initialise everything from the checkpoint. Used at power-up AND for the
// restart after a brownout. Valid checkpoint -> resume; invalid -> sample 0.
static void restoreFromCheckpoint(EngineState &s) {
  Checkpoint saved = s.ckpt;           // read BEFORE wiping the state
  bool valid = ckptValid(saved);

  engineReset(s);
  s.mode = MODE_ACTIVE;
  if (valid) {
    s.sampleCount = saved.sampleIndex;
    s.runAvg   = saved.rollingAvg;
    s.avgQ     = (uint32_t)saved.rollingAvg << AVG_SHIFT;
    s.floorAvg = saved.floorAvg;
    s.floorQ   = (uint32_t)saved.floorAvg << FLOOR_SHIFT;
    s.started  = 1;
    Serial.print("[BOOT] checkpoint VALID -> resuming from sample ");
    Serial.println(saved.sampleIndex);
  } else {
    Serial.println("[BOOT] checkpoint INVALID -> starting from sample 0");
  }
  s.nextSampleUs = micros();
  s.bootAtMs = millis();
}

static void taskHalted(EngineState &s, uint32_t nowMs) {
  uint8_t pressed = (digitalRead(PIN_CROWN) == LOW);
  if (!s.crownReleased) {
    if (!pressed && (uint32_t)(nowMs - s.haltAt) >= 200) s.crownReleased = 1;
  } else if (pressed) {
    Serial.println("Power restored: restarting...");
#if USE_HW_REBOOT
    Serial.flush();
    rp2040.reboot();                    // real watchdog reset, RAM survives
#else
    restoreFromCheckpoint(s);           // in-place restart (works in Wokwi)
#endif
  }
}

static void ledOutput(const EngineState &s) {
  wr(PIN_LED_G, s.ledG);
  wr(PIN_LED_R, s.ledR);
  wr(PIN_LED_B, s.ledB);
}