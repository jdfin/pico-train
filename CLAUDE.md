# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

pico-train is a Raspberry Pi Pico (RP2040) DCC model railroad throttle with a touchscreen GUI. It generates real-time DCC packets to control locomotives, using a WS35 480x320 TFT display (SPI) with a GT911 capacitive touchscreen (I2C).

## Build Commands

```bash
# Initial setup (from project root)
mkdir build && cd build && cmake .. && ninja

# Rebuild after changes (from build/)
ninja

# The output binary is build/throttle/throttle.uf2
# Flash by copying .uf2 to the Pico in BOOTSEL mode
```

Requires Pico SDK 2.2.0 and ARM toolchain 14_2_Rel1 (configured via VS Code Pico extension at `~/.pico-sdk/`).

## Architecture

**Main application**: `throttle/throttle.cpp` — initializes hardware (display, touchscreen, DCC), creates GUI pages (number pad, speed control, function buttons, programming), and runs the main loop.

**Six libraries** under `libraries/` (each a git submodule, INTERFACE CMake libraries):

| Library | Purpose | Key classes |
|---------|---------|-------------|
| **dcc** | DCC protocol: packet encoding, transmission, throttle state, RailCom feedback | `DccCommand`, `DccLoco`, `DccAdc` |
| **framebuffer** | Display abstraction and graphics primitives (pixel, line, rect, circle, text) | `Framebuffer` (abstract), `Tft` (base), `Ws35` |
| **gui** | UI widgets and page management with touch event dispatch | `GuiWidget`, `GuiButton`, `GuiSlider`, `GuiPage`, `GuiLabel` |
| **touchscreen** | Touch input handling over I2C | `Touchscreen` (abstract), `Gt911` |
| **misc** | Utilities: I2C helpers, DMA/PWM IRQ muxing, logging, string ops | Various (see headers) |
| **pio_edges** | PIO-based edge detection with microsecond timing | `PioEdges` |

**Dependency flow**: gui → framebuffer + touchscreen → misc; dcc → misc; pio_edges → misc

**GPIO pin assignments** are defined in `throttle/dcc_gpio_cfg.h`, `throttle/fb_gpio_cfg.h`, and `throttle/ts_gpio_cfg.h`.

## Testing

When tests reveal bugs in production code, fix the bugs rather than working around them in tests.

## Code Style

- C11 / C++17, compiled with `-Wall -Wextra -Werror`
- Clang-format configured (Google-based, 4-space indent, right pointer alignment, braces on new line for functions/classes)
- stdio is routed over USB (`pico_enable_stdio_usb`), not UART
