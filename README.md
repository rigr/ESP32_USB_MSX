# ESP32-S3 USB & BLE Mouse → Roland Sampler / MSX Protocol
**RiGr – April 2026**

**UPDATE 05-2026:** The sketch `ESP32_USB_BLE_Roland.ino` now supports **both USB and BLE mice**.  
However, this version has **not yet been tested with the Roland Sampler**.

---

This project allows you to connect a standard USB or Bluetooth Low Energy (BLE) mouse to an **ESP32-S3** (N16R8 board) and convert the mouse signals into the classic MSX mouse protocol.  
The Roland Sampler uses exactly the same protocol as old MSX computers.

### Features
- Direct USB-Host support for USB mice
- BLE mouse support (via NimBLE)
- Stable MSX mouse protocol with **direct GPIO register access**
- Mouse movement processed on Core 1 in a dedicated task → minimal jitter
- Zoom function using the mouse wheel (20 % – 200 %)
- Serial output for debugging

### Hardware
- **ESP32-S3 N16R8** (Dual USB port version recommended)  
  → See layout: [ESP32-S3-N16R8 User Guide](https://github.com/microrobotics/ESP32-S3-N16R8/blob/main/ESP32-S3-N16R8_User_Guide.pdf)
- USB mouse (any standard USB mouse) via OTG adapter
- BLE mouse
- Roland Sampler (or MSX computer) with mouse input (9-pin)

**Important Wiring Note:**
- The ESP32-S3 uses the **native USB-OTG port** (usually GPIO19 = D-, GPIO20 = D+).
- On many N16R8 boards you must set an **OTG jumper** or close a solder bridge so the port works as a host with 5 V supply.

### Pin Assignment (ESP32-S3 → Sampler)

| ESP32-S3 Pin | Function                  | Sampler Sub-D9F |
|--------------|---------------------------|-----------------|
| GPIO 14      | MX0 – Data Bit 0          | Pin 1           |
| GPIO 13      | MX1 – Data Bit 1          | Pin 2           |
| GPIO 12      | MX2 – Data Bit 2          | Pin 3           |
| GPIO 11      | MX3 – Data Bit 3          | Pin 4           |
| Vin          | 5V                        | Pin 5           |
| GPIO 8       | BTN_L – Left Mouse Button | Pin 6           |
| GPIO 3       | BTN_R – Right Mouse Button| Pin 7           |
| GPIO 46      | STROBE / CS (Input)       | Pin 8           |
| **GPIO 15**  | **Manual Scan Trigger**   | -               |
| GND          | Ground                    | Pin 9           |

> **GPIO 15 (D15)** is used as a **manual scan trigger** input.

### Serial Output
Example:

USB: buttons=0x01(L ), x=-3, y=1, wheel=0 Z:100%

USB: buttons=0x00(  ), x=0, y=0, wheel=1 Z:120%

- `L` = Left button pressed  
- `R` = Right button pressed  
- `Z:` shows the current zoom factor in percent

### Zoom Function
- Use the **mouse wheel** to adjust the mouse zoom between **20 % and 200 %**.
- Default value is **100 %**.

### Important Library Note
This project requires the **EspUsbHost** library by tanakamasayuki.

**You must manually copy the two files**  
`EspUsbHost.cpp` and `EspUsbHost.h`  
from this repository:  
→ https://github.com/tanakamasayuki/EspUsbHost

**Both files must be placed in the same folder as your `ESP32_USB_BLE_Roland.ino` file.**

### Compiling & Flashing
1. Install the remaining required libraries via the Arduino Library Manager.
2. Copy `EspUsbHost.cpp` and `EspUsbHost.h` into the sketch folder (see note above).
3. In Arduino IDE select:
   - Board: **ESP32S3 Dev Module**
   - USB Mode: **Hardware CDC and JTAG**
   - Flash Size: **16MB** (if available)
4. Upload the sketch.

The sketch starts automatically. Once a USB or BLE mouse is detected, movements and clicks will appear in the serial monitor.
To lanuch the web interface you need to press the BOOT button more than 3 seconds. You can then adjust all sorts of settings and calibrate the mouse connected.

---

Good luck and enjoy your Roland Sampler!

## License
This project is released under the **MIT License**.

## Acknowledgments
- **tanakamasayuki** – for the excellent [EspUsbHost](https://github.com/tanakamasayuki/EspUsbHost) library
- **NYYRIKKY** and **Peter Ullrich** – for their foundational work on the MSX/Roland mouse protocol
- Original BLE MSX mouse project: https://github.com/rigr/ESP32_USB_MSX

## Contribution
Initial support came from NYYRIKKY and Peter Ullrich – thank you both!
