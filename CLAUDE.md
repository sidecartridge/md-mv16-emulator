# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A SidecarTridge Multi-device microfirmware app that emulates the **MV16** cartridge (the 16-voice sample player bundled with *B.A.T.*) for Atari ST/STE and streams its audio out over Bluetooth A2DP. It is built from the [md-microfirmware-template](https://github.com/sidecartridge/md-microfirmware-template).

`AGENTS.md` holds host-setup tribal knowledge (toolchain paths, debug probe wiring, troubleshooting table) and is worth reading alongside this file.

## Build

There is no test suite. **A successful build is the validation step.**

```bash
# Full build: Atari ST target firmware + RP2040 firmware + dist/ artifacts
PICO_TOOLCHAIN_PATH=/Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin \
  ./build.sh pico_w debug 44444444-4444-4444-8444-444444444444

# RP2040 side only (skips the Atari 68000 rebuild — much faster while iterating on C)
cd rp && ./build.sh pico_w debug
```

- `./build.sh <board_type> <build_type> <app_uuid_key>` — board is `pico`/`pico_w` (`pico_w` is required: no CYW43 ⇒ no Bluetooth), build type is `debug`/`release`, and `44444444-4444-4444-8444-444444444444` is the agreed placeholder UUID for local validation. The real UUID comes from the `APP_UUID` CI secret.
- `build_type` only controls `_DEBUG` (UART stdio on/off). CMake is **always** invoked with `CMAKE_BUILD_TYPE=MinSizeRel` — a full `Release` build previously broke the firmware (memory/over-optimization). Don't "fix" this.
- `rp/build.sh` deletes `rp/build/` and re-runs CMake from scratch every time, and also `git submodule update`s and pins `pico-sdk` to `2.2.0` / `pico-extras` to `sdk-2.2.0`.
- Requires `atarist-toolkit-docker` (`stcmd`, needs a PTY) for the 68000 assembly half, and the ARM GNU Toolchain 14.2 via `PICO_TOOLCHAIN_PATH`.
- Output: `dist/<APP_UUID>-<version>.uf2` plus `dist/<APP_UUID>.json` (generated from `desc/app.json` with UUID, MD5 and version substituted).
- `make tag` tags and pushes `version.txt`, which triggers `.github/workflows/release.yml` (GitHub release + S3 upload to `atarist.sidecartridge.com`).

**Versioning:** `version.txt` at the repo root is the single source of truth; `build.sh` copies it into `rp/` and `target/`. The release workflow publishes the text above the first `---` in `CHANGELOG.md` as the release notes.

## Lint & format

`clang-tidy` runs automatically as part of every RP2040 compile when it's on `PATH` (`CMAKE_C_CLANG_TIDY`), and `.clang-tidy` sets `WarningsAsErrors: '*'` — a naming violation **fails the build**. The conventions that bite most often:

- Functions: `camelBack` matching `^[a-z]+(?:_[a-z]+)*[A-Z][a-zA-Z0-9]*$` — i.e. a lowercase module prefix then CamelCase, e.g. `term_printString`, `a2dpDemoStartScanning`, `display_termRefresh`.
- Variables/params/members: `camelBack`. Magic numbers are flagged (only `0`, `1`, `-1` are free) — hoist them to a `#define`.
- Linker symbols and other unavoidable violations are wrapped in `// NOLINTBEGIN(readability-identifier-naming)` blocks (see `constants.h`).

Formatting is Google-based `clang-format`; `make clang-format` from `rp/build` reformats `rp/src`.

`.clang-tidy-ignore` exempts vendored code: `pico-sdk/`, `pico-extras/`, `rp/src/settings/`, `rp/src/u8g2/`, `target/`.

## Architecture

### Two binaries, two CPUs

The app is one firmware image containing **two** programs:

1. **Atari ST side** — 68000 assembly in `target/atarist/src/main.s` (+ `inc/`), assembled with vasm/vlink into a 64 KB `FIRMWARE.IMG`, converted by `target/atarist/firmware.py` into `rp/src/include/target_firmware.h` (a `uint16_t` array), and checked into the RP2040 build. At runtime `emul_start()` does `COPY_FIRMWARE_TO_RAM(...)` so the Atari can execute it out of the emulated ROM4 cartridge space at `$FA0000`.
2. **RP2040 side** — C in `rp/src/`, the actual host driver.

Changing `main.s` requires a full `./build.sh` (or at least `target/atarist/build.sh`) to regenerate `target_firmware.h`; `rp/build.sh` alone will silently keep the old target firmware.

### Boot chain

`main.c` sets 260 MHz / 1.20 V, then `gconfig_init()` → `aconfig_init()` → `emul_start()`. Both config layers look the app up by `CURRENT_APP_UUID_KEY` (compiled in from `APP_UUID_KEY`) in a flash lookup table; **any failure jumps to the Booster app** via `reset_jump_to_booster()`. `main.c` is deliberately feature-free — new work belongs in `emul.c` and below.

### Flash & RAM map (`rp/src/memmap_rp.ld`)

The linker script carves flash so the app coexists with the Booster: app code `0x10000000`+1 MB, `ROM_TEMP` 128 KB, `BOOSTER_APP_FLASH` at `0x10120000` (768 KB, must not be touched), per-app config sectors at `0x101E0000`, a UUID→sector lookup page at `0x101FE000`, global config at `0x101FF000`. RAM is split into 128 KB general + a 128 KB `ROM_IN_RAM` window that the cartridge bus emulation serves reads from. Symbols like `_booster_app_flash_start` are declared in `constants.h`.

### Cartridge bus: `romemul` vs `romaddr` (mutually exclusive)

Both drive the same 16 address/data GPIOs (base 6) plus ROM3 (GPIO 26) / ROM4 (GPIO 22) and the inverted READ/WRITE side-set pins (27/28), so **only one can own PIO0 at a time**:

- `romemul.c` / `romemul.pio` — normal mode. Emulates a ROM cartridge: PIO + DMA watch the address bus and feed back words from `ROM_IN_RAM`. Its DMA IRQ handler is where the command protocol is decoded.
- `romaddr.c` / `romaddr.pio` — MV16 capture mode. The MV16 is written by *addressing*, so the sample value arrives on the address lines. The PIO program waits on ROM4, samples 16 bus pins, and DMAs them into a 1024-entry power-of-two ring buffer (`romaddr_read()` / `romaddr_available()`, with an overflow counter).

The switch happens in `a2dpDemoBootIntoRomaddrCapture()`: `deinit_romemul()`, sleep, `romaddr_init()`, `romaddr_start()`. After this point the terminal is gone and the Atari is running the user's own program.

### Command protocol & terminal

`tprotocol.h` defines the cartridge-bus command protocol (magic `0xABCD`, command id, payload size, checksum) that the Atari side speaks by *reading* addresses in the ROM3 window (`$FB0000`). `term_dma_irq_handler_lookup` (registered as the `init_romemul` response callback) decodes it; `APP_TERMINAL_KEYSTROKE` carries key presses up from the Atari.

Rendering goes the other way: the RP2040 draws into a u8g2 monochrome buffer (`display.c`, `display_term.c`, 40×25 tiles at 320×200) that lives in the ROM-in-RAM window and is read directly by the Atari (`DISPLAY_BYPASS_FRAMEBUFFER = 1`, so the endianness swap happens on the Atari side). `SEND_COMMAND_TO_DISPLAY(DISPLAY_COMMAND_RESET|_CONTINUE|_NOP|_TERM)` pokes a longword the 68000 loop polls.

`emul.c` owns the menu: a `Command` table (`S` scan / `X` booster / `E` exit) plus a `keepActive` loop that calls `select_checkPushReset()` and `term_loop()`. On exit it either resets the RP2040 or jumps to Booster depending on `resetDeviceAtBoot`.

### Audio path (`a2dp.c`, ~1600 lines — the bulk of the app)

`a2dp_source_demo_launch()` **blocks** in `btstack_run_loop_execute()` for the remainder of the app's life; everything after entering scan mode is driven by BTstack timers, not by `emul.c`'s loop:

- a term-poll timer (`a2dpDemoTermPollTimerHandler`, 20 ms) pumps the terminal so keys still work — `term_setRawKeyHandler(a2dpDemoHandleTerminalKey)` intercepts them for device selection.
- an audio timer (10 ms) fills an SBC buffer and sends media packets.
- `a2dpDemoPumpRomaddrIntoPcmRing()` drains the `romaddr` ring, converts the captured words (13-bit offset-binary, LSB always zero: `((raw & 0x1FFE) - 0x1000) << 3`) into PCM16, applies a Q15 low-pass and gain, and pushes into a 4096-byte mono PCM ring consumed by the SBC encoder.
- `a2dp_pcm_sample.h` is a built-in PCM clip (generated by `tools/wav_to_pcm16_mono.py`) used as the default source before capture starts.
- Device discovery keeps a 24-entry table with a 30 s per-address cooldown after failures; the local name/CoD are set for an audio source.

Hot-path functions are marked `__not_in_flash_func(...)` — keep that when editing ring-buffer or DMA code.

`btstack_cyw43_no_flash.c` **overrides** the Pico SDK's `btstack_cyw43_init`/`deinit` to install a no-op TLV backend and a NULL link-key DB, so BTstack never writes flash NVM (which would collide with the config/Booster sectors above). Pairing keys are intentionally not persisted.

### SELECT button

`select.c` polls GPIO 5 with debounce: short press → `reset_device()` (watchdog reboot back to the app menu), long press → `reset_deviceAndEraseFlash()`. Callbacks are wired in `main.c`.

## Guardrails

- **Never modify `pico-sdk/` or `pico-extras/`** — they're pinned git submodules.
- `rp/src/u8g2/` and `rp/src/settings/` are vendored; treat them as read-only.
- `secrets.sh` holds live AWS credentials and is gitignored — never read it into output or commit it.
- Debug output (`DPRINTF`) compiles away in release builds. Never add it inside DMA/PIO callbacks even in debug: UART latency breaks bus timing.
- `CHARACTER_GAP_MS` in `rp/src/include/blink.h` must stay defined (700) or the RP build fails.
- Harmless VASM warnings during the Atari build (`target data type overflow`, `trailing garbage after option -D`) are expected.
