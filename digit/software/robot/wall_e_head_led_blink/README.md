# WALL-E Head LED Blink

Minimal ESP-IDF project that alternates two active-low LEDs connected to the ESP32-S3 UART pins used as GPIO outputs.

The LEDs are turned on by pulling the pin low and turned off by driving the pin high.

The firmware turns one LED on while the other is off, waits 150 ms, then swaps them.

By default, the firmware uses:

- GPIO 43
- GPIO 44

## Build

```powershell
idf.py build
```

## Flash and Monitor

```powershell
idf.py -p COMx flash monitor
```

Replace `COMx` with the serial port for your board.

## Hardware Note

GPIO 43 and GPIO 44 are commonly used as UART0 TX/RX pins on ESP32-S3 boards. They can be used as GPIO outputs, but do not use them for LEDs if your board needs those pins for serial flashing, UART console logs, or another UART device.

If your ESP-IDF console is configured for UART0, using these pins as LEDs may interfere with serial monitor output. USB CDC console configurations are usually safer for this setup.
