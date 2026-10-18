#pragma once
// Smartwatch firmware - Tasks 1-4 (FINAL, cumulative)
// Board: Raspberry Pi Pico (RP2040), Arduino-Pico core, Wokwi
//
//  Task 1  pulse monitoring   : 2000 samples, rolling peak + rolling floor/average
//  Task 2  brownout recovery  : crown button -> checkpoint -> halt -> reset -> resume
//  Task 3  indicators         : green toggles every 50 samples, red = 50% over average
//  Task 4  touch gestures     : double-tap toggles ACTIVE/STEALTH, long-press = SOS
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
constexpr uint8_t PIN_TOUCH = 15;  // GP15 front touch key
constexpr uint8_t PIN_LED_G = 16;  // activity
constexpr uint8_t PIN_LED_R = 17;  // pulse spike
constexpr uint8_t PIN_LED_B = 18;  // SOS

// ---------------- Constants ----------------
constexpr uint8_t  WINDOW           = 16;       // 16 x 2 B = 32 B raw buffer
constexpr uint16_t RUN_SAMPLES      = 2000;
constexpr uint32_t SAMPLE_PERIOD_US = 10000;    // 100 Hz
constexpr uint8_t  FLOOR_SHIFT      = 3;        // floor EMA alpha 1/8
constexpr uint8_t  AVG_SHIFT        = 6;        // running average alpha 1/64
constexpr uint16_t GREEN_EVERY      = 50;       // green toggles every 50 samples
constexpr uint16_t MIN_PULSE        = 1000;     // red ignores near-zero signal
constexpr uint16_t PRINT_EVERY      = 100;

constexpr uint32_t DEBOUNCE_MS = 25;
constexpr uint32_t TAP_MAX_MS  = 400;           // a tap is a short press
constexpr uint32_t DTAP_MIN_MS = 200;           // double-tap window
constexpr uint32_t DTAP_MAX_MS = 500;
constexpr uint32_t LONG_MS     = 2000;          // long-press time

constexpr uint8_t MODE_ACTIVE  = 0;
constexpr uint8_t MODE_STEALTH = 1;
constexpr uint8_t MODE_HALTED  = 2;

constexpr uint8_t  FLAG_CROWN  = 0x01;
constexpr uint32_t CKPT_MAGIC  = 0xC0FFEE42u;

// ---------------- Checkpoint (lives inside EngineState) ----------------
struct Checkpoint {
  uint32_t tag;          // validation tag, derived from the fields below
  uint16_t sampleIndex;  // samples processed in the current run
  uint16_t rollingAvg;   // running average
  uint16_t floorAvg;     // rolling average pulse floor
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

  uint32_t floorQ;             // floor accumulator (avg << FLOOR_SHIFT)
  uint32_t avgQ;               // average accumulator (avg << AVG_SHIFT)
  uint32_t nextSampleUs;       // next sample deadline
  uint32_t touchChangeAt;      // last raw touch edge time
  uint32_t touchPressAt;       // time the current press began
  uint32_t lastTapAt;          // time of the previous tap
  uint32_t haltAt;             // time brownout happened

  Checkpoint ckpt;             // checkpoint (survives reset together with S)

  uint16_t peak;               // rolling peak amplitude
  uint16_t floorAvg;           // rolling average pulse floor
  uint16_t runAvg;             // running average of the signal
  uint16_t sampleCount;        // samples in current run (0..1999)
  uint16_t runsDone;

  uint8_t  head, filled, started;
  uint8_t  ledG, ledR, ledB;   // logical LED states
  uint8_t  mode;               // ACTIVE / STEALTH / HALTED
  uint8_t  touchRaw, touchStable, tapPending, longFired;
  uint8_t  crownReleased;
  volatile uint8_t flags;      // set by ISR, handled in loop
};

static_assert(sizeof(EngineState) <= 512, "EngineState too large");
static_assert(sizeof(EngineState::pulseBuf) < 64, "Raw pulse buffer must be < 64 bytes");

// Placed in a NOLOAD RAM section so it is not cleared by startup code and
// survives a watchdog reboot. Validity is decided by the checkpoint tag.
static EngineState __uninitialized_ram(S);

// ---------------- Small helpers ----------------
static inline void wr(uint8_t pin, uint8_t v) { digitalWrite(pin, v ? HIGH : LOW); }

static void engineReset(EngineState &s) { memset((void *)&s, 0, sizeof(s)); }

// ---------------- ISR (non-blocking: sets a flag only) ----------------
static void crownISR() { S.flags |= FLAG_CROWN; }

// ---------------- Pulse engine (Task 1 + running average for Task 3) ----------------
static void pulseProcess(EngineState &s, uint16_t sample) {
  // Task 3 red LED: incoming sample vs running average BEFORE folding it in
  uint32_t thresh = (uint32_t)s.runAvg + ((uint32_t)s.runAvg >> 1);   // avg * 1.5
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
  // Task 3 green LED: toggle every 50 processed samples
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
    Serial.print(" avg=");    Serial.print(s.runAvg);
    Serial.print(" mode=");   Serial.println(s.mode == MODE_ACTIVE ? "ACTIVE" : "STEALTH");
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

// ---------------- Task 4: touch gestures ----------------
static void onTap(EngineState &s, uint32_t tapAt) {
  if (s.tapPending) {
    uint32_t dt = tapAt - s.lastTapAt;
    if (dt >= DTAP_MIN_MS && dt <= DTAP_MAX_MS) {         // valid double-tap
      s.tapPending = 0;
      if (s.mode == MODE_ACTIVE) {
        s.mode = MODE_STEALTH;
        s.ledB = 0;                                        // all LEDs off
        Serial.println("DOUBLE-TAP -> STEALTH_MODE");
      } else {
        s.mode = MODE_ACTIVE;
        Serial.println("DOUBLE-TAP -> ACTIVE_MODE");
      }
      return;
    }
    if (dt < DTAP_MIN_MS) return;                          // too fast: ignore
  }
  s.tapPending = 1;                                        // first tap
  s.lastTapAt = tapAt;
}

static void onLongPress(EngineState &s) {
  s.ledB = 1;                                              // SOS
  s.mode = MODE_ACTIVE;                                    // force ACTIVE
  s.tapPending = 0;
  Serial.println("LONG-PRESS -> SOS (blue ON, ACTIVE_MODE)");
}

static void taskTouch(EngineState &s, uint32_t nowMs) {
  uint8_t raw = (digitalRead(PIN_TOUCH) == LOW);           // 1 = pressed
  if (raw != s.touchRaw) { s.touchRaw = raw; s.touchChangeAt = nowMs; }

  if (raw != s.touchStable && (uint32_t)(nowMs - s.touchChangeAt) >= DEBOUNCE_MS) {
    s.touchStable = raw;
    if (raw) {                                             // press
      s.touchPressAt = s.touchChangeAt;
      s.longFired = 0;
    } else {                                               // release
      if (!s.longFired && (uint32_t)(s.touchChangeAt - s.touchPressAt) <= TAP_MAX_MS)
        onTap(s, s.touchChangeAt);
    }
  }

  if (s.touchStable && !s.longFired && (uint32_t)(nowMs - s.touchPressAt) >= LONG_MS) {
    s.longFired = 1;
    onLongPress(s);
  }
  if (s.tapPending && (uint32_t)(nowMs - s.lastTapAt) > DTAP_MAX_MS) s.tapPending = 0;
}

// ---------------- LED output ----------------
static void ledOutput(const EngineState &s) {
  uint8_t on = (s.mode == MODE_ACTIVE);                    // STEALTH = all LEDs off
  wr(PIN_LED_G, on && s.ledG);
  wr(PIN_LED_R, on && s.ledR);
  wr(PIN_LED_B, on && s.ledB);
}

// ---------------- Memory report ----------------
#define SZ(f) do { Serial.print("  " #f " = "); Serial.print(sizeof(S.f)); Serial.println(" B"); } while (0)
static void memReport() {
  Serial.print("EngineState = "); Serial.print(sizeof(EngineState)); Serial.println(" bytes (<= 512)");
  Serial.print("Raw pulse buffer = "); Serial.print(sizeof(S.pulseBuf)); Serial.println(" bytes (< 64)");
  SZ(pulseBuf); SZ(floorQ); SZ(avgQ); SZ(nextSampleUs); SZ(touchChangeAt);
  SZ(touchPressAt); SZ(lastTapAt); SZ(haltAt); SZ(ckpt);
  SZ(peak); SZ(floorAvg); SZ(runAvg); SZ(sampleCount); SZ(runsDone);
  SZ(head); SZ(filled); SZ(started); SZ(ledG); SZ(ledR); SZ(ledB); SZ(mode);
  SZ(touchRaw); SZ(touchStable); SZ(tapPending); SZ(longFired);
  SZ(crownReleased); SZ(flags);
}

