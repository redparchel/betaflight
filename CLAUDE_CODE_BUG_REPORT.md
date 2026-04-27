# Bug Report: BARO task hangs ~25ms, blocks CALIB on `IFLIGHT_F722_TWING` (BF 2025.12.2)

## TL;DR for the agent

A user-reported regression on Betaflight 2025.12.2: on the `IFLIGHT_F722_TWING` target, the BARO task takes ~25ms (max) per execution, which prevents the `CALIBRATING` arming-disable flag from clearing. The quad cannot arm. Disabling baro (`set baro_hardware = NONE`) is a confirmed workaround across two physically different boards. Your job is to find the root cause in the source tree and propose a fix.

This bug report is structured for an autonomous coding agent. It contains:
1. **Confirmed observations** (high confidence)
2. **Hypotheses ranked by likelihood** (medium confidence — investigate, don't trust)
3. **Concrete code starting points** (file paths and symbols to read first)
4. **Acceptance criteria** (how to know you're done)
5. **Constraints** (what not to break)

Do not propose speculative fixes. Read the actual code, form a theory grounded in what you find, then propose the smallest diff that satisfies the acceptance criteria.

---

## 1. Confirmed observations

### 1.1 Reproduction

- **Target:** `IFLIGHT_F722_TWING`
- **Firmware:** Betaflight 2025.12.2
  - Build A: `c7b0b98a587eb01c9d966d06f5234808` (Apr 3 2026)
  - Build B: `feac9bfaf0e758bb9ac719d93291f15b` (Mar 21 2026)
- **Hardware (per `status` output):**
  - MCU: STM32F722
  - Gyros: 2× ICM20689 on SPI1, dual-gyro shared DMA
  - Accel: ICM20689
  - Baro: BMP280 on I2C2
  - Flash: 32MB SPI flash on SPI3
- **Two physically different boards** both exhibit the bug identically.

### 1.2 Symptom

```
Arming disable flags: ... CALIB ...
```

`CALIBRATING` flag never clears, even after long uptime. Quad cannot arm.

### 1.3 Smoking gun (`tasks` output)

```
16 - (BARO)   rate=0Hz   max=25575us   avg=1us   maxload=0.0%   late=1328
```

The BARO task max execution time is ~25 milliseconds across both boards. For comparison, healthy tasks on the same FC:

```
02 - (GYRO)    max=16us   avg=2us
03 - (FILTER)  max=28us   avg=11us
04 - (PID)     max=55us   avg=25us
05 - (ACC)     max=16us   avg=4us
```

A 25ms blocking call in any task on a real-time scheduler is pathological. The `late=1328` counter for BARO indicates the scheduler is repeatedly noticing the task running over its deadline.

### 1.4 Other diagnostics

- `DEVICES DETECTED: SPI=2, I2C=1 (1-2 errors)` — I2C error counter is non-zero but small.
- Gyro noise floor is clean (±0.7 deg/s at rest at scale=1).
- ACC calibration persists correctly.
- GYRO/FILTER/PID/ACC tasks all healthy.
- Behavior is consistent across USB-only power, battery-only, and both simultaneously.

### 1.5 Workaround that works (confirmed)

```
set baro_hardware = NONE
save
```

After this, CALIB clears reliably across power cycles. BARO task no longer runs. Quad arms normally. **This is the proof that the bug is in the baro init/read path or the gating logic that depends on it.**

### 1.6 Things tried that did NOT help

These narrow the search space:

- `set gyro_calib_noise_limit = 500` (renamed from `moron_threshold`)
- `set gyro_cal_on_first_arm = OFF` and `ON`
- `set gyro_enabled_bitmask = 1`, `2`, or `3` (single-gyro, both, swap)
- `defaults nosave`
- `set baro_i2c_address = 118` (force address rather than auto-detect)
- Freeing the `LED_STRIP` resource entirely (`feature -LED_STRIP` + `resource LED_STRIP 1 NONE` + `dma pin A01 NONE`). A previous comment on issue #12228 suggested this was a fix for a similar-looking bug in 4.4 — it is **not** the fix here. BARO task max time stays at ~25ms.
- USB vs battery power (no difference)
- Multiple power cycles (no improvement; behavior is consistent, not intermittent in any meaningful way)

### 1.7 Related prior issues (read these for context)

- [#12228](https://github.com/betaflight/betaflight/issues/12228) — same target, same symptom, BF 4.4 RC5. Closed. Reported user fix was reverting to 4.3.2.
- [#13360](https://github.com/betaflight/betaflight/issues/13360) — Mag and Baro stop working after arming on BF 4.5. Different target, similar I2C shared-bus theme.
- [#13505](https://github.com/betaflight/betaflight/issues/13505) — qmc5883 + DPS310 I2C bus issue, references PR #13467 ("Fix qmc5883l lockup") as an incomplete fix.

The pattern across these issues: I2C bus state-machine handling on shared buses has been a recurring source of bugs. This may be a different manifestation of the same underlying class of problem.

---

## 2. Hypotheses (ranked by likelihood)

Investigate in order. Each hypothesis includes "what to look at first" and "how to confirm or rule out."

### Hypothesis A: BMP280 init or read path does a blocking I2C wait with a long timeout

**Why this is most likely:** 25ms is suspiciously close to a typical I2C timeout value. The BARO task is allowed to take this long because the I2C transaction is busy-waiting for an ACK that doesn't come.

**Where to look first:**
- `src/main/drivers/barometer/barometer_bmp280.c` — BMP280 driver
- `src/main/drivers/barometer/barometer.c` — generic baro layer
- `src/main/sensors/barometer.c` — sensor scheduling/state machine
- `src/main/drivers/bus_i2c_*.c` — STM32F7 I2C driver, particularly `bus_i2c_hal_init.c` and `bus_i2c_timing.c`

**Specific things to grep for:**
- `I2C_TIMEOUT` constants and their values
- `bmp280_read_*` and `bmp280_write_*` functions
- Any `while (...) { ... }` loops in BMP280 driver that wait on a register
- `i2cBusy()`, `i2cWait()`, `i2cWaitDone()` — synchronous waits

**How to confirm:** if you find a busy-wait with a 20–50ms timeout in the BMP280 read path that's being hit on every read, that's almost certainly it.

### Hypothesis B: BARO state machine gets stuck in a state that always re-enters a slow path

**Why plausible:** Baro chips typically have a state machine — START_CONVERSION → WAIT → READ_TEMP → READ_PRESSURE — and if a state transition fails or a register read returns garbage, the FSM may either retry forever or get stuck waiting.

**Where to look first:**
- The state machine logic in `src/main/sensors/barometer.c`, particularly the `baroUpdate()` or `baroProcess()` function
- The BMP280 `_get_up()` and `_get_ut()` style functions

**How to confirm:** trace what state the BARO task is in when it takes 25ms. Look for missing default cases, missing timeouts on individual states, or a state that loops.

### Hypothesis C: I2C bus gets locked by another peripheral and BMP280 read waits

**Why plausible:** This target's `I2C2` is shared. Per the resource map: `I2C_SCL 2 B10`, `I2C_SDA 2 B11`. The DMA for I2C2 might collide with another peripheral's DMA, causing transactions to stall.

**Where to look first:**
- `src/main/target/IFLIGHT_F722_TWING/` (note: BF moved many target configs to a unified config-file system; the actual config may live in `unified_targets/configs/IFLIGHT_F722_TWING.config` or a similar path — find where the resources for this target are defined)
- `src/main/drivers/dma_*.c` for DMA stream allocation
- I2C driver init for evidence that the bus is being held in a busy state

**How to confirm:** look for DMA stream conflicts on the I2C2 path. The `# dma` section of the user's config dump (provided in section 3 below) shows the DMA mappings as Betaflight resolved them — compare to what the I2C2 driver expects.

### Hypothesis D: Recent refactor introduced a regression

**Why plausible:** Issue #12228 reports the same symptom in BF 4.4 RC5, with the user noting that 4.3.2 worked. That's 3+ years ago. Something in the baro code path has been broken on this target for a long time, or a fix was made and re-broken.

**Where to look first:**
- `git log --oneline --all -- src/main/drivers/barometer/barometer_bmp280.c`
- `git log --oneline --all -- src/main/sensors/barometer.c`
- Specifically look for commits between 4.3.x and 4.4.x that touched baro init or I2C transaction code.
- Then look for any reverts or follow-up commits that may have re-introduced a bug.

**How to confirm:** find a commit that, when reverted, makes the BARO task time drop to sub-millisecond on this target.

### Hypothesis E: BMP280 driver doesn't handle a specific chip revision correctly

**Why less likely but worth checking:** iFlight changed the baro component from BMP280 to DPS310 mid-production due to supply issues. If this is a counterfeit or revision-B BMP280 that responds slowly to certain commands, the driver might be coping poorly.

**How to confirm:** check the BMP280 chip ID register read result. If the user reports that the chip ID is non-standard or the chip responds with unusual timing, this might be it. Lower priority because the workaround (disable baro) confirms the rest of the system is fine — this only matters if you can't otherwise localize the hang.

---

## 3. Reproduction artifacts

### 3.1 User config (`diff all`) — minimal reproducer

```
# Betaflight / STM32F7X2 (F722) 2025.12.2 Mar 21 2026 / 18:23:39 (79065c96b) MSP API: 1.47

board_name IFLIGHT_F722_TWING
manufacturer_id IFRC

feature GPS
feature OSD
feature ESC_SENSOR

set gyro_enabled_bitmask = 3
set acc_calibration = -88,-49,23,1
set baro_bustype = I2C
set baro_i2c_device = 2
set baro_i2c_address = 0
set baro_hardware = AUTO
```

The minimum needed to repro: clean install of 2025.12.2 on this target with default baro settings. CALIB will appear and persist.

### 3.2 `status` output (relevant excerpt)

```
DEVICES DETECTED: SPI=2, I2C=1 (1 errors)
GYRO: (1) ICM20689 enabled locked dma shared, (2) ICM20689 enabled locked dma shared
ACC: ICM20689
BARO: BMP280
Arming disable flags: RXLOSS CALIB CLI MSP
```

### 3.3 `tasks` output (full)

```
Task list             rate/hz  max/us  avg/us maxload avgload  total/ms   late    run reqd/us
00 - (         SYSTEM)     10       5       0    0.0%    0.0%         0      0    400       3
01 - (         SYSTEM)    990      10       0    0.9%    0.0%        26      4  39446       5
02 - (           GYRO)   8019      16       2   12.8%    2.3%      1010      0 318043       0
03 - (         FILTER)   8019      28      11   22.4%    8.9%      3764      0 318043       0
04 - (            PID)   8019      55      25   44.1%   20.7%      8995      0 318043       0
05 - (            ACC)    995      16       4    1.5%    0.4%       176      3  39417       8
06 - (       ATTITUDE)    100      22      10    0.2%    0.1%        42      1   3965      15
07 - (             RX)     20      32      15    0.0%    0.0%        39      4   2505      17
08 - (         SERIAL)    100  331517       4 3315.1%    0.0%       885      0   3858      32
09 - (       DISPATCH)    992       9       0    0.8%    0.0%        25      1  39448       3
10 - (BATTERY_VOLTAGE)     50       8       2    0.0%    0.0%         4      0   1986       7
11 - (BATTERY_CURRENT)     50       6       0    0.0%    0.0%         1      0   1986       4
14 - (            GPS)    100      22       5    0.2%    0.0%        22      3   3964      18
16 - (           BARO)      0   25575       1    0.0%    0.0%        76   1328  38948       0
17 - (       ALTITUDE)    100      10       2    0.1%    0.0%        10      1   3857       7
```

(SERIAL task shows 3315% load because of the active CLI/MSP session — ignore it, this is a known artifact of the user being connected via Configurator.)

---

## 4. Acceptance criteria

A proposed fix is acceptable if and only if **all** of the following are true:

1. **The BARO task max time on `IFLIGHT_F722_TWING` with BMP280 enabled drops to under 1ms** on a typical `tasks` snapshot. (Healthy I2C baro reads should complete in tens of microseconds, not tens of milliseconds.)
2. **The `CALIBRATING` arming-disable flag clears within 5 seconds of boot** with `set baro_hardware = AUTO` (the default).
3. **`BARO: BMP280` continues to be detected** in `status` output. The fix must not break baro functionality, only fix the timing/hang.
4. **No regression on other targets.** The fix must not break any target that currently works. If the fix is in shared code (sensor layer, I2C driver, scheduler), include reasoning for why it's safe.
5. **No regression in baro-using features.** Alt Hold, GPS Rescue, and OSD altitude readout should continue to work.

A fix that simply disables baro on this target by default, or worsens performance for other targets, is not acceptable.

---

## 5. Constraints — what NOT to do

- **Don't disable baro on this target by default.** That's the workaround, not a fix.
- **Don't propose changes to user-facing CLI parameter names or defaults** without strong justification — those are part of Betaflight's stability contract.
- **Don't refactor unrelated code.** Keep the diff minimal and focused on the root cause.
- **Don't add new tests that require hardware to run** without also providing a path to test in CI.
- **Don't skip understanding why** the bug exists. A patch that papers over the symptom (e.g., "bump the timeout to 100ms" or "skip baro reads if too slow") is not acceptable. The 25ms hang is a symptom of an underlying bug — find the bug.

---

## 6. Suggested investigation workflow

1. **Read the related issues** linked in §1.7 to understand the historical context.
2. **Locate the BARO task entry point** in the scheduler. Likely in `src/main/scheduler/scheduler.c` or via a task table. Find which function is called when the BARO task runs.
3. **Trace from the BARO task entry point down** through the sensor layer, the BMP280 driver, and into the I2C bus driver. Note every function call that could block.
4. **Identify any synchronous I2C wait** with a timeout >10ms. That's a prime suspect.
5. **Check git log** for the relevant files to see what changed between 4.3.x and current. Pay special attention to commits touching the BMP280 driver or the F7 I2C driver.
6. **Form a hypothesis** based on what you found, then write a minimal diff.
7. **Verify the diff against the acceptance criteria**, especially the "no regression" criterion. State explicitly which other targets share the changed code path and why they should be unaffected.
8. **Write up your findings** in the format below.

---

## 7. Output format expected

When you're done, produce:

1. **Diagnosis section** — what the bug is, in one paragraph. Cite specific file paths and line numbers.
2. **Proposed diff** — minimal, focused, with comments explaining the change.
3. **Verification plan** — how the user can confirm the fix on their hardware:
   - CLI commands to run after flashing
   - What `tasks` output should look like
   - What `status` output should look like
4. **Risk analysis** — what other code paths use the same functions you changed, and why they are or aren't at risk.
5. **Open questions** — anything you couldn't determine without hardware access. Be honest about uncertainty.

---

## 8. User context (for empathy, not action)

The reporting user has spent 4+ hours debugging this on a long-range 7" build that needs baro for GPS Rescue auto-landing. The workaround (disable baro) costs them Alt Hold and reliable auto-landing. They're a real pilot, not a bug-hunter, and they've already done a methodical investigation including swapping FCs to rule out hardware. A working fix would let them fly safely.

If you find that this is **not** fixable from the firmware side (e.g., it's a hardware issue with this specific FC revision), say so clearly and explain what you'd need to confirm that conclusion.

---

## 9. Repo orientation tips

If you haven't worked on Betaflight before:

- The codebase is C, targets STM32 microcontrollers via the HAL/LL libraries.
- Build system uses Make (see `Makefile` and `mk/` directory). To build a specific target: `make TARGET=STM32F7X2` (this target uses the `STM32F7X2` MCU build target combined with the `IFLIGHT_F722_TWING` board config).
- Board configs live in the `unified_targets/configs/` directory as `.config` files. The `IFLIGHT_F722_TWING` config defines the resources, DMA, and timer mappings.
- The scheduler and task system is in `src/main/scheduler/`.
- Sensor drivers are in `src/main/drivers/` (low-level) and `src/main/sensors/` (high-level orchestration).
- The arming-disable flag enum is in `src/main/fc/runtime_config.h`. `ARMING_DISABLED_CALIBRATING` is what's blocking arming here.
- To find where CALIB is set/cleared: grep for `ARMING_DISABLED_CALIBRATING` and the related `unsetArmingDisabled(ARMING_DISABLED_CALIBRATING)` calls.
