# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A TI **MSP430i2041** embedded firmware project (Sigma-Delta ADC demo) built in **Code Composer Studio Theia** (CCS v70.5.0). The firmware streams two SD24 ADC channels to a host PC running a GUI Composer GUI over UART, and accepts commands back from the GUI to change PGA gain and channel preload.

Source originates from TI's `msp430i20xx_sigma-delta_adc_demo_mpack` projectspec under `msp430ware_3_80_14_01`.

All code lives in [CCS Project using GUI Composer/](CCS Project using GUI Composer/) — note the spaces in the path; quote it in shell commands.

## Build

Builds are driven by the CCS-generated GNU makefiles in `Debug/` and `Release/`. The TI MSP430 codegen toolchain must be at `C:/ti/ccs2050/ccs/tools/compiler/ti-cgt-msp430_21.6.1.LTS` (hard-coded as `CG_TOOL_ROOT` in those makefiles).

From the project directory:

```
gmake -C "CCS Project using GUI Composer/Debug" all
gmake -C "CCS Project using GUI Composer/Debug" clean
```

Replace `Debug` with `Release` for the optimized build. Output is `CCS Project using GUI Composer.out` (ELF) plus a `.map` and `_linkInfo.xml`.

Inside the IDE, the active target/connection comes from [CCS Project using GUI Composer/targetConfigs/MSP430i2041.ccxml](CCS Project using GUI Composer/targetConfigs/MSP430i2041.ccxml) and the debug launch configs from [.theia/launch.json](.theia/launch.json) (TI MSP430 USB1 debug probe).

There is no test framework — this is bare-metal MCU firmware. Verification is done on hardware via the GUI.

### Compile-time flags / preprocessor

Set in `.cproject` and reflected in the makefiles:

- `__MSP430i2041__`, `__ENABLE_GUI__`
- `--use_hw_mpy=16`, `--opt_for_speed=5`, `--printf_support=minimal`
- Heap = 80, Stack = 80 bytes (very small — be deliberate about allocations)

### clangd

[CCS Project using GUI Composer/.clangd](CCS Project using GUI Composer/.clangd) points at `Release/.clangd` as its compilation database and **suppresses all diagnostics** (`Suppress: '*'`) — clangd is for navigation only, not lint. The TI compiler is authoritative.

## Architecture

Three cooperating layers — keep changes within the layer that owns the concern:

### 1. Application — [Source/main.c](CCS Project using GUI Composer/Source/main.c), [Source/callbacks_mpack.c](CCS Project using GUI Composer/Source/callbacks_mpack.c)

`main()` runs a foreground state machine driven by a single `volatile uint8_t command` byte. The protocol is:

- **ADC → GUI**: SD24 ISR (`SD24_ISR` in `main.c`) latches `SD24MEM0`/`SD24MEM1` into globals and sets `adcReady`. The main loop's `else` branch sends both via `GUIComm_sendInt16("0"/"1", ...)`.
- **GUI → MCU**: incoming mpack messages match against the `GUI_RXCommands[]` table (single-char string keys `"3"`..`"6"`), each entry firing a `GUICallback_*` in `callbacks_mpack.c`. Callbacks **never apply hardware changes directly** — they stop the ADC, stash the new value in a global, and set `command` to a `ADC_*` constant from [Include/callbacks_mpack.h](CCS Project using GUI Composer/Include/callbacks_mpack.h). The main loop's `if/else if` ladder is what actually writes `SD24INCTLx`/`SD24PREx` and re-arms the ADC. Preserve that ISR-defers-to-main-loop split when adding commands.

Adding a new GUI→MCU command means: (a) `#define` an `ADC_*`/command ID in `callbacks_mpack.h`, (b) add a callback in `callbacks_mpack.c`, (c) register it in `GUI_RXCommands[]` in `main.c`, (d) add a branch in the main-loop ladder.

### 2. GUI transport — [Source/GUIComm_mpack.c](CCS Project using GUI Composer/Source/GUIComm_mpack.c), [Source/MSP430_GUI/GUI_mpack.c](CCS Project using GUI Composer/Source/MSP430_GUI/GUI_mpack.c), [Source/MSP430_GUI/mpack/](CCS Project using GUI Composer/Source/MSP430_GUI/mpack/)

Serializes app values as MessagePack maps (`{cmd: value}`) over the HAL UART. `GUIComm_send*` are typed thin wrappers around `mpack_write_*`. `mpack/` is a vendored snapshot of [ludocode/mpack](https://github.com/ludocode/mpack) — don't hand-edit it; treat it as a third-party drop-in. Note `GUI_mpack.h` defines a *global* `mpack_writer_t writer` and a 50-byte `data[]` buffer — these are the single shared TX path; the protocol assumes one in-flight message at a time.

### 3. HAL — [Source/MSP430_HAL/](CCS Project using GUI Composer/Source/MSP430_HAL/), [Include/HAL.h](CCS Project using GUI Composer/Include/HAL.h)

The only device-specific layer. Filenames are suffixed `_i20xx` to mark MSP430i20xx-family code. To port to a different MSP430 family, replace the `_i20xx.c` files (and `HAL_Config_Private.h`) — `HAL.h` is the stable contract.

- `HAL_System_i20xx.c` — clocks, board init (called from `main`'s `HAL_System_Init()`).
- `HAL_GUIComm_UART_i20xx.c` — eUSCI_A0 UART at **921600 baud** (configured in [HAL_Config_Private.h](CCS Project using GUI Composer/Source/MSP430_HAL/HAL_Config_Private.h)). Receive byte → `tGUICommRXCharCallback` → feeds the mpack reader.
- `HAL_IO_i20xx.c` — GPIO (status LED on P1.4).

[Include/Config_Common.h](CCS Project using GUI Composer/Include/Config_Common.h) declares the clock frequencies (CPU/HSBUS = 8 MHz nominal in macros, but actual DCO is 16.384 MHz — see comments in `main.c`).

### Startup

[Source/low_level_init.c](CCS Project using GUI Composer/Source/low_level_init.c) implements `_system_pre_init()`, called by TI's CRT0 **before** `main` and before `.data` init. It disables JTAG, calibrates the PMM 1.16V reference, calibrates DCO to 16.384 MHz, and trims SD24 references. Returning 0 would skip data-segment init — it returns 1.

## Available Claude skill

[.claude/skills/ti-ccstudio-clang-code-coverage/](.claude/skills/ti-ccstudio-clang-code-coverage/) provides an instrumentation/coverage workflow for TI Clang projects (not invoked automatically — use when the user asks for coverage).
