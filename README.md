# TwoWire (STM32 HAL I²C Wrapper)

A lightweight, Arduino-style I²C (“Wire”) wrapper for **STM32 HAL**.
It provides a familiar API (`begin()`, `beginTransmission()`, `write()`, `endTransmission()`, `requestFrom()`, `available()`, `read()`) plus practical extras: register helpers, timeout handling, optional GPIO-based bus recovery, and non-blocking modes (Interrupt/DMA).

> Works on STM32 **F1/F4/H7** families (easily extensible to others).

---

## Table of Contents

* [Features](#features)
* [Supported MCUs](#supported-mcus)
* [Folder Structure](#folder-structure)
* [Getting Started](#getting-started)

  * [1) MCU Selection](#1-mcu-selection)
  * [2) Add Sources to Your Project](#2-add-sources-to-your-project)
  * [3) Initialize and Use](#3-initialize-and-use)
* [Quick Examples](#quick-examples)

  * [Master Write (blocking)](#master-write-blocking)
  * [Master Read (blocking)](#master-read-blocking)
  * [Register Helpers (8-bit/16-bit addr)](#register-helpers-8bit16bit-addr)
  * [Non-blocking (Interrupt/DMA)](#non-blocking-interruptdma)
  * [Slave Mode (minimal)](#slave-mode-minimal)
* [Callbacks (IT/DMA mode)](#callbacks-itdma-mode)
* [Bus Recovery (GPIO clocking)](#bus-recovery-gpio-clocking)
* [Configuration & Tuning](#configuration--tuning)

  * [Clock / Timing](#clock--timing)
  * [Transfer Modes](#transfer-modes)
  * [Timeouts](#timeouts)
  * [Buffers](#buffers)
* [Error Handling](#error-handling)
* [FAQ / Tips](#faq--tips)
* [Doxygen Docs](#doxygen-docs)
* [Contributing](#contributing)
* [License](#license)

---

## Features

* ✅ Familiar **Arduino-like API** for master mode
* ✅ **Register helpers** for 8-bit and 16-bit register addressing
* ✅ **Three transfer modes**: Polling (blocking), **Interrupt**, **DMA**
* ✅ **Timeout** handling with optional auto-recovery
* ✅ Optional **GPIO-based I²C bus recovery** (clock SCL, issue STOP)
* ✅ Minimal **slave-mode** helpers
* ✅ Compact, dependency-free (besides STM32 HAL)

---

## Supported MCUs

* **STM32F1**, **STM32F4**, **STM32H7**

> Other families can be added by adapting `#include` lines and `setClock()` behavior.

---

## Folder Structure

```
/your-project
  ├─ Core/
  ├─ Drivers/
  ├─ Middlewares/
  ├─ Src/
  ├─ Inc/
  ├─ Wire.cpp          <- this library
  ├─ Wire.h            <- this library
  └─ mcu_select.h      <- choose your target family here
```

---

## Getting Started

### 1) MCU Selection

Create `mcu_select.h` next to `Wire.h` and enable **exactly one**:

```c
// mcu_select.h
// Uncomment exactly one line:

// #define STM32F1
// #define STM32F4
// #define STM32H7
```

### 2) Add Sources to Your Project

* Add `Wire.h` to your include paths.
* Add `Wire.cpp` to your build sources.
* Make sure **CubeMX**/your project already initializes an `I2C_HandleTypeDef` (e.g. `hi2c1`).

### 3) Initialize and Use

```cpp
#include "Wire.h"

extern I2C_HandleTypeDef hi2c1;   // provided by CubeMX

TwoWire wire(&hi2c1);

int main(void) {
  // HAL_Init(), SystemClock_Config(), MX_GPIO_Init(), MX_I2C1_Init(), ...

  wire.setClock(100000); // 100 kHz (see Clock / Timing notes below)
  wire.begin();          // master mode

  // ... now use wire.beginTransmission(), wire.write(), wire.endTransmission(), etc.
}
```

---

## Quick Examples

### Master Write (blocking)

```cpp
uint8_t dev7 = 0x68;  // 7-bit address
wire.beginTransmission(dev7);
wire.write(0x01);
wire.write(0x02);
uint8_t rc = wire.endTransmission(); // 0 on success
```

### Master Read (blocking)

```cpp
uint8_t dev7 = 0x68;
uint8_t n = wire.requestFrom(dev7, 6); // read 6 bytes
while (wire.available()) {
  int b = wire.read(); // 0..255, or -1 if none
}
```

### Register Helpers (8-bit/16-bit addr)

```cpp
// 8-bit register addressing:
uint8_t tx[2] = {0x12, 0x34};
wire.writeReg8(0x68, 0x10, tx, sizeof(tx));

uint8_t rx[6];
wire.readReg8(0x68, 0x3B, rx, sizeof(rx));

// 16-bit register addressing:
uint8_t pld[4] = {1,2,3,4};
wire.writeReg16(0x50, 0x0123, pld, sizeof(pld));

uint8_t r2[2];
wire.readReg16(0x50, 0x0010, r2, sizeof(r2));
```

### Non-blocking (Interrupt/DMA)

```cpp
// choose mode BEFORE begin()
wire.setTxMode(WIRE_MODE_INTERRUPT);
wire.setRxMode(WIRE_MODE_DMA);
wire.begin();

// TX (IT):
wire.beginTransmission(0x68);
wire.write(0xAA);
wire.write(0xBB);
wire.endTransmission();   // returns immediately in IT mode
while (!wire.getTxCompleteFlag()) { /* poll or do other work */ }

// RX (DMA):
wire.requestFrom(0x68, 16);           // schedule DMA read
while (!wire.getRxCompleteFlag()) { /* ... */ }
// data is now in internal RX buffer; consume with read()/available()
```

### Slave Mode (minimal)

```cpp
wire.begin(0x42);                // become a 7-bit slave
wire.requestFromSlave(4);        // wait to receive 4 bytes via IT
int first = wire.readSlave();    // read from internal slave buffer
wire.writeSlave((uint8_t)0xAB);  // respond one byte (IT)
```

> Slave helpers are intentionally minimal; extend for your protocol as needed.

---

## Callbacks (IT/DMA mode)

For **Interrupt/DMA** modes, forward HAL’s weak callbacks to your `TwoWire` instance:

```c
// In a C/C++ file visible to the HAL (not inside the class file)
extern TwoWire wire;

// Master complete callbacks
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) { wire.masterTxCpltCallback(); }
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) { wire.masterRxCpltCallback(); }

// Slave complete callbacks (optional)
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)  { wire.slaveTxCpltCallback(); }
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)  { wire.slaveRxCpltCallback(); }
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t dir, uint16_t code) {
  wire.slaveAddrCallback();
}
```

> If you manage **multiple** `TwoWire` instances with different `I2C_HandleTypeDef*`, consider a tiny “handle → instance” registry to route callbacks to the right object.

---

## Bus Recovery (GPIO clocking)

If the bus is stuck (e.g., SDA held low by a slave), you can try **manual recovery**:

1. Provide SCL/SDA pins:

   ```cpp
   wire.setGPIO(GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9); // example: I2C1 on PB8/PB9
   ```
2. Call:

   ```cpp
   wire.busRecoveryGPIO();
   ```

This clocks SCL up to 9 pulses, attempts a STOP (SDA rising while SCL high), and then re-inits the peripheral.

> **Note:** Pins must be correct and externally pulled up.

---

## Configuration & Tuning

### Clock / Timing

* **F1/F4**: `setClock(100000)` or `setClock(400000)` updates `Init.ClockSpeed`.
* **H7**: uses `Init.Timing`. The library includes example timings; for production, compute accurate values for your clock tree (CubeMX timing calculator or datasheet).

Call `setClock()` **before** `begin()`.

### Transfer Modes

* `WIRE_MODE_BLOCK` (default): Simple, blocking HAL calls.
* `WIRE_MODE_INTERRUPT`: Non-blocking; completion via callbacks/flags.
* `WIRE_MODE_DMA`: Non-blocking; best for larger transfers / low CPU load.

Set **before** `begin()`:

```cpp
wire.setTxMode(WIRE_MODE_DMA);
wire.setRxMode(WIRE_MODE_INTERRUPT);
```

### Timeouts

Control blocking call timeouts and optional auto-recovery:

```cpp
wire.setWireTimeout(100, /*resetWithTimeout=*/true); // 100 ms + recovery attempt
if (wire.getWireTimeoutFlag()) {
  // previous blocking call timed out
  wire.clearWireTimeoutFlag();
}
```

### Buffers

Default buffer size:

```c
#define WIRE_BUFFER_LENGTH 32
```

You can override (compile-time) **before** including `Wire.h`.
When writing with `write()`, the library prevents overflow and returns how many bytes were queued.

---

## Error Handling

`endTransmission()` returns Arduino-style codes:

| Code | Meaning                                         |
| ---- | ----------------------------------------------- |
| 0    | Success                                         |
| 1    | Data too long (reserved/not used in most paths) |
| 2    | NACK on address                                 |
| 3    | NACK on data                                    |
| 4    | Other error                                     |
| 5    | Timeout                                         |

`requestFrom()` returns the number of bytes placed into the internal RX buffer (0 on error).
Several methods also set `errorCode` (uint8\_t) for quick checks.

For HAL-level analysis, inspect `hi2c->ErrorCode` (from your handle) after a failure.

---

## FAQ / Tips

**Q: My device needs “write register, then read N bytes” with a repeated-START.**
A: Use `readReg8()`/`readReg16()` helpers. They internally do the proper sequence with HAL.

**Q: I’m on STM32H7 and see NACK/timeout.**
A: Verify `Init.Timing` is correct for your kernel clock and desired bus speed. Start with 100 kHz, validate with a logic analyzer, then move to 400 kHz.

**Q: The bus gets stuck sometimes.**
A: Check pull-ups (typically 2.2k–4.7k for 3.3V, depending on bus length and capacitance). If stuck, try `busRecoveryGPIO()` and consider `setWireTimeout(..., true)` to auto-recover.

**Q: How do I handle multiple I²C peripherals and instances?**
A: Instantiate one `TwoWire` per `I2C_HandleTypeDef*`. For IT/DMA, route HAL callbacks to the correct object (via a small handle→instance map).

---

## Contributing

Issues and PRs are welcome!

* Keep the API minimal and portable.
* Match STM32 HAL naming and behavior where applicable.
* Prefer small, focused changes with clear commit messages.

---

## License

Choose a license (e.g., MIT) and place it here. Example:

```
MIT License — see LICENSE file for details.
```

---

