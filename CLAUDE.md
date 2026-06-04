# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for a **4-quadrant pinhole sun sensor** on a TI **MSP430i2041**, built in **Code Composer Studio Theia** (CCS v70.5.0). The SD24 sigma-delta ADC reads four photodiode quadrants (TIAs on A0..A3); the firmware block-averages them, computes the **sun unit-vector on-chip** (trig-free closed form), reads an external **AT30TS74** I²C temperature sensor, and serves the result to a host.

Two mutually-exclusive output paths (compile-time, in [Include/Config_Common.h](MOVE-IIIa-SunSensor/Include/Config_Common.h)):

- **`USE_SPI_OUTPUT` (default, production):** 4-wire hardware-framed **SPI slave** on eUSCI_A0. The host polls 27-byte frames (sun vector, raw quadrant ADCs, dark-corrected sum, temperature in centi-°C, flags, CRC-16). The wire protocol is fully specified in [SUN_SENSOR_SPI_PROTOCOL.md](SUN_SENSOR_SPI_PROTOCOL.md) (v2).
- **`USE_UART_OUTPUT` (legacy/dev):** streams the same values over UART (plain text for the [tools/](tools/) Python scripts; the original TI GUI-Composer/mpack command path is also still present here).

The firmware base originated from TI's `msp430i20xx_sigma-delta_adc_demo_mpack` projectspec (`msp430ware_3_80_14_01`) — the GUI/mpack transport and the HAL are inherited from it; the sun-sensor application and the SPI slave are not.

All code lives in [MOVE-IIIa-SunSensor/](MOVE-IIIa-SunSensor/) (renamed from the original spaces-in-name CCS project).

## Build

Builds are driven by the CCS-generated GNU makefiles in `Debug/` and `Release/`. The TI MSP430 codegen toolchain must be at `C:/ti/ccs2050/ccs/tools/compiler/ti-cgt-msp430_21.6.1.LTS` (hard-coded as `CG_TOOL_ROOT` in those makefiles).

From the project directory:

```
gmake -C "MOVE-IIIa-SunSensor/Debug" all
gmake -C "MOVE-IIIa-SunSensor/Debug" clean
```

Replace `Debug` with `Release` for the optimized build. Output is `MOVE-IIIa-SunSensor.out` (ELF) plus a `.map` and `_linkInfo.xml`.

Inside the IDE, the active target/connection comes from [MOVE-IIIa-SunSensor/targetConfigs/MSP430i2041.ccxml](MOVE-IIIa-SunSensor/targetConfigs/MSP430i2041.ccxml) and the debug launch configs from [.theia/launch.json](.theia/launch.json) (TI MSP430 USB1 debug probe).

There is no test framework — this is bare-metal MCU firmware. Verification is done on hardware (an SPI master, or the `USE_UART_OUTPUT` build + the Python tools in [tools/](tools/)).

### Compile-time flags / preprocessor

Set in `.cproject` and reflected in the makefiles:

- `__MSP430i2041__`, `__ENABLE_GUI__`
- `--use_hw_mpy=16`, `--opt_for_speed=5`, `--printf_support=minimal`
- Heap = 80, Stack = 256 bytes (small — be deliberate about allocations; stack was raised from 80 to cover the `sqrtf` in the sun-vector math)
- Output path is chosen in [Include/Config_Common.h](MOVE-IIIa-SunSensor/Include/Config_Common.h): `USE_SPI_OUTPUT` (default) vs `USE_UART_OUTPUT`, mutually exclusive (`#error` enforced)

### clangd

[MOVE-IIIa-SunSensor/.clangd](MOVE-IIIa-SunSensor/.clangd) points at `Release/.clangd` as its compilation database and **suppresses all diagnostics** (`Suppress: '*'`) — clangd is for navigation only, not lint. The TI compiler is authoritative.

## Architecture

Three cooperating layers — keep changes within the layer that owns the concern:

### 1. Application — [Source/main.c](MOVE-IIIa-SunSensor/Source/main.c)

The production data path:

- **SD24 ISR** (`SD24IV_SD24MEM3` case): accumulates all four quadrant channels and, every `ADC_AVG_N` (32) raw 4 kHz samples, block-averages them, advances `sample_count` by 32, and sets `adcReady` — a **125 Hz** averaged frame rate. Keep the ISR short: averaging only, no float/CRC.
- **Main loop**: wakes on `adcReady` (LPM0 between frames), takes a GIE-guarded atomic snapshot of the four averaged ADCs + `sample_count`, refreshes the I²C temperature ~1 Hz, runs `compute_sun_vector()` (closed-form, one `sqrtf`, no per-frame trig), then publishes:
  - **SPI build:** `spi_publish_frame()` packs the 27-byte frame + CRC into a free **triple-buffer** slot and publishes by index; the `USCI_A0` SPI ISR serves `READ_FRAME`/`READ_ID`/`READ_STATUS`/`NOP` (always-RX, deterministic 2-byte lead). See [SUN_SENSOR_SPI_PROTOCOL.md](SUN_SENSOR_SPI_PROTOCOL.md).
  - **UART build:** emits the same values as one plain-text line for the Python tools.

The watchdog is **held** through init and only started (`WDT_RUN`) once the main loop is about to pet it; all I²C waits are bounded so a stuck bus can't hang boot.

**Legacy GUI-command path (UART build only):** the original TI demo's `GUI_RXCommands[]` table + `GUICallback_*` in [Source/callbacks_mpack.c](MOVE-IIIa-SunSensor/Source/callbacks_mpack.c) (set PGA gain / channel preload via mpack from a GUI Composer GUI) is retained but is **not** part of the SPI production path. Those callbacks defer hardware writes to the main loop via a `command` byte + `ADC_*` constants (from [Include/callbacks_mpack.h](MOVE-IIIa-SunSensor/Include/callbacks_mpack.h)) — preserve that ISR-defers-to-main-loop split if you extend it.

### 2. GUI transport — [Source/GUIComm_mpack.c](MOVE-IIIa-SunSensor/Source/GUIComm_mpack.c), [Source/MSP430_GUI/GUI_mpack.c](MOVE-IIIa-SunSensor/Source/MSP430_GUI/GUI_mpack.c), [Source/MSP430_GUI/mpack/](MOVE-IIIa-SunSensor/Source/MSP430_GUI/mpack/)

Serializes app values as MessagePack maps (`{cmd: value}`) over the HAL UART. `GUIComm_send*` are typed thin wrappers around `mpack_write_*`. `mpack/` is a vendored snapshot of [ludocode/mpack](https://github.com/ludocode/mpack) — don't hand-edit it; treat it as a third-party drop-in. Note `GUI_mpack.h` defines a *global* `mpack_writer_t writer` and a 50-byte `data[]` buffer — these are the single shared TX path; the protocol assumes one in-flight message at a time.

### 3. HAL — [Source/MSP430_HAL/](MOVE-IIIa-SunSensor/Source/MSP430_HAL/), [Include/HAL.h](MOVE-IIIa-SunSensor/Include/HAL.h)

The only device-specific layer. Filenames are suffixed `_i20xx` to mark MSP430i20xx-family code. To port to a different MSP430 family, replace the `_i20xx.c` files (and `HAL_Config_Private.h`) — `HAL.h` is the stable contract.

- `HAL_System_i20xx.c` — clocks, board init (called from `main`'s `HAL_System_Init()`).
- `HAL_GUIComm_UART_i20xx.c` — eUSCI_A0 UART at **921600 baud** (configured in [HAL_Config_Private.h](MOVE-IIIa-SunSensor/Source/MSP430_HAL/HAL_Config_Private.h)). Receive byte → `tGUICommRXCharCallback` → feeds the mpack reader.
- `HAL_IO_i20xx.c` — GPIO (status LED on P1.4).

[Include/Config_Common.h](MOVE-IIIa-SunSensor/Include/Config_Common.h) declares the clock frequencies (CPU/HSBUS = 8 MHz nominal in macros, but actual DCO is 16.384 MHz — see comments in `main.c`).

### Startup

[Source/low_level_init.c](MOVE-IIIa-SunSensor/Source/low_level_init.c) implements `_system_pre_init()`, called by TI's CRT0 **before** `main` and before `.data` init. It disables JTAG, calibrates the PMM 1.16V reference, calibrates DCO to 16.384 MHz, and trims SD24 references. Returning 0 would skip data-segment init — it returns 1.

## Available Claude skill

[.claude/skills/ti-ccstudio-clang-code-coverage/](.claude/skills/ti-ccstudio-clang-code-coverage/) provides an instrumentation/coverage workflow for TI Clang projects (not invoked automatically — use when the user asks for coverage).
