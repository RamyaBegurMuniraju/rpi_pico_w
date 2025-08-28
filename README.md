# Raspberry Pi Pico W – BLE Examples
This project demonstrates BLE features such as advertising, I2C slave communication, and temperature sensing using the Raspberry Pi Pico W.

Modular C examples for Bluetooth Low Energy using Raspberry Pi Pico W and BTstack.

## Modules

- `advertising/`: BLE advertising + temperature sensor
- `connection/`: BLE connection initiation & handling
- `data_streaming/`: BLE connection initiation & handling


## Prerequisites

- Raspberry Pi Pico W
- Linux/macOS or WSL (Windows Subsystem for Linux)
- Git
- CMake
- GCC ARM toolchain (e.g., `arm-none-eabi-gcc`)
- Python 3 (for optional scripts)

---

## How to build

```bash
Clone the Pico SDK
git clone -b master https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk

Initialize submodules
git submodule update --init

Export PICO SDK Path
export PICO_SDK_PATH=~/pico-sdk

Download the source code
git clone https://github.com/
cd rpi_pico_w
mkdir build
cd build
cmake .. -DPICO_BOARD=pico_w
make

Flash the Pico W
- Hold down the BOOTSEL button on the Pico W.
- Plug it into your PC via USB.
- Release the button — it should appear as a USB drive named RPI-RP2.
- Drag and drop the .uf2 file into the drive.
- The board will reboot and run your program.
