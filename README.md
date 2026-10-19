# Smartwatch Firmware Challenge

Firmware for a small smartwatch built on a Raspberry Pi Pico (RP2040) and simulated in Wokwi. The watch monitors a pulse signal, shows its state on three LEDs, responds to two buttons, and recovers its progress after a simulated brownout.

## Features

| Task | Feature | Behaviour |
|------|---------|-----------|
| 1 | Pulse monitoring | Processes 2,000 raw 16-bit samples one at a time. Maintains a rolling peak amplitude and a rolling average pulse floor. |
| 2 | Brownout recovery | Crown press saves a checkpoint and halts. The next press restarts the watch and resumes from the saved sample index. An invalid checkpoint restarts from sample 0. |
| 3 | Indicators | Green LED toggles every 50 processed samples. Red LED turns on when the sample is at least 1.5x the running average (and at least 1000), and off when it falls below. |
| 4 | Touch gestures | Double-tap (200-500 ms apart) toggles ACTIVE and STEALTH modes. In stealth, LEDs are off but tracking continues. Long press (2 s) turns the blue SOS LED on and forces ACTIVE. |

## Hardware and pin map

| Function | Pin | Wokwi part |
|----------|-----|------------|
| PPG signal (analogue) | GP26 / ADC0 | Potentiometer (SIG, VCC, GND) |
| Side crown (brownout) | GP14 | Pushbutton to GND, `INPUT_PULLUP` |
| Front touch key (gestures) | GP15 | Pushbutton to GND, `INPUT_PULLUP` |
| Green LED | GP16 | LED via 220 ohm resistor |
| Red LED | GP17 | LED via 220 ohm resistor |
| Blue LED | GP18 | LED via 220 ohm resistor |

Both buttons are wired between the pin and GND, so a press reads LOW.

## Signal model (simulation)

Wokwi has no PPG sensor part. The potentiometer sets the baseline, and `readPPG16()` adds a software-generated pulse (about 72 BPM, 833 ms period) on top. This function is the only place that knows the signal is simulated, so a real sensor driver can replace it without touching the rest of the firmware.

## Repository layout

```
diagram.json              Wokwi circuit (Raspberry Pi Pico, pot, buttons, LEDs, resistors)
smartwatch_task2.ino      Entry point for Tasks 1-2 (setup and loop only)
engine_task2.h            Engine for Tasks 1-2
smartwatch_task3.ino      Entry point for Tasks 1-3
engine_task3.h            Engine for Tasks 1-3
smartwatch_task4.ino      Entry point for Tasks 1-4 (final)
engine_task4.h            Engine for Tasks 1-4 (final)
```

Each task is a cumulative version. Use one pair at a time: a `.ino` together with the `engine_taskN.h` that has the same number. Do not mix pairs.

### Why the engine is in a header

The Arduino build generates function prototypes and inserts them above the sketch body. Functions that take custom struct types would then reference types the compiler has not seen yet. Keeping the types and logic in a `.h` file, which is included as plain text in order, avoids that.

## Running it in Wokwi

1. Create a new Raspberry Pi Pico project in Wokwi, using the Arduino-Pico (C++) template.
2. Replace `diagram.json` with the file from this repository.
3. Create a new file named exactly `engine_task4.h` (or the matching header for the task you are running) and paste its contents in.
4. Replace `sketch.ino` with `smartwatch_task4.ino`.
5. Start the simulation and open the Serial Monitor at 115200 baud.

On boot the Serial Monitor prints the checkpoint status (`[BOOT] checkpoint VALID` or `INVALID`) and a memory report. Every 100 samples it prints `n=`, `raw=`, `peak=`, `floor=`, `avg=` and `mode=`.

### Testing the features

- **Pulse:** turn the potentiometer and watch `peak` and `floor` move with it.
- **Indicators:** the green LED should blink at 1 Hz. The red LED should flash on each synthetic pulse.
- **Gestures:** double-tap the green button to toggle stealth mode. Hold it for 2 seconds for the long press.
- **Brownout:** press the red crown button to halt (`!! BROWNOUT: checkpoint saved, sample=...`). Release it, then press again. The watch should print `[BOOT] checkpoint VALID -> resuming from sample ...` and continue.

## Memory constraints

| Rule | Value | Enforced by |
|------|-------|-------------|
| `sizeof(EngineState)` at most 512 bytes | 100 bytes (measured on an x86-64 host) | `static_assert` |
| Raw pulse buffer under 64 bytes | 32 bytes (`uint16_t pulseBuf[16]`) | `static_assert` |

Other rules followed: all mutable state lives in one `EngineState` instance, there is no `malloc`, `calloc` or `free`, no `float` or `double` (integer maths with shifts only), and the crown ISR only sets a flag.

The sizes are also printed at boot so they can be checked against the running firmware.

## Design notes

- **Peak:** the maximum of a 16-sample window (160 ms). A short-window peak, not a long-term maximum.
- **Floor:** exponential moving average (alpha 1/8) of the window minimum, in fixed point.
- **Running average:** exponential moving average (alpha 1/64) of the raw sample, in fixed point.
- **Spike:** a rising edge into the top quarter of the peak-to-floor span, when the span is at least 1500.
- **Checkpoint:** a 12-byte struct (tag, sample index, rolling average, floor, reserved). The tag is a mixing function over the fields. The validity check also requires the sample index to be below 2,000.
- **Restart:** `restoreFromCheckpoint()` runs at boot and after a brownout. It reads the checkpoint before clearing state.
- **Hardware reset:** `USE_HW_REBOOT` in the header (default 0) selects between an in-place software restore and a real watchdog reset via `rp2040.reboot()`. Keep it at 0 for Wokwi. Set it to 1 for real hardware.
- **Bounce guard:** the crown interrupt is ignored for 300 ms after a restart.

## Known limitations

- **Checkpoint is held in RAM.** It survives an in-place restart but not a real power loss. A proper fix writes it to a flash sector with an integrity check.
- **Tag is not a CRC.** The validation tag catches most corruption but is not a real integrity code.
- **Synthetic pulse is always on** in Tasks 2-4. There is no switch to use a real sensor signal.
- **No filtering** of motion artefacts or noise.
- **No heart-rate (BPM) output.**
- **Possible flag race:** the ISR sets `S.flags` with a read-modify-write while the loop clears it with a plain store. A press landing in that gap can be lost. Fix: clear inside a short critical section.
- **Uses the Arduino-Pico API** (on top of the Pico SDK) rather than calling the SDK directly. Confirm this meets the project's HAL rule.
- **No automated tests.** The engine is plain C++ and could be tested on a PC with a host harness.

## Status

- The firmware compiles with `-Wall -Wextra` and no warnings against a stub Arduino header on a host PC.
- Behaviour has been checked in Wokwi only as far as the serial logs during development. Run the full test list above before relying on it.

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| `'Checkpoint' does not name a type` | The engine code is in the `.ino`. Move it into the matching `engine_taskN.h`. |
| Watch hangs after `Power restored` | `USE_HW_REBOOT` is 1 in the simulator. Set it to 0. |
| Restart always starts at sample 0 | The checkpoint was invalidated. Check that the brownout line printed before the restart. |
| ADC values are about a quarter of the expected range | `analogReadResolution(12)` is missing from `setup()`. The core defaults to 10 bits. |
| Gestures trigger on a single press | Button wiring is wrong. Each button must connect to GND. |

## Author

Jaideep (jaideep1163@gmail.com)
