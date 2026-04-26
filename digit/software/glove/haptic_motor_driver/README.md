# ESP-IDF Haptic Motor Driver

This project drives two DRV2605L haptic motor drivers connected behind a TCA9546A I2C mux.

## Hardware defaults

- ESP32 SDA: GPIO8
- ESP32 SCL: GPIO9
- Left DRV2605L EN: IO37
- Right DRV2605L EN: IO38
- TCA9546A address: `0x70`
- DRV2605L address: `0x5A`
- Motor 0 mux channel: `0`
- Motor 1 mux channel: `1`

If your mux address or channel wiring differs, update the macros at the top of
`main/haptic_motor_driver.c`.

On ESP32-S3-WROOM-1, module pins 30 and 31 correspond to IO37 and IO38.

## Build

```powershell
idf.py set-target esp32s3
idf.py build
```

Then flash with:

```powershell
idf.py -p COMx flash monitor
```

Replace `COMx` with the ESP32 serial port.

On boot, the firmware scans the upstream I2C bus, then selects each TCA9546A
channel and scans again before initializing the DRV2605L devices. If the mux at
`0x70` is not found, it logs the failure and stays alive instead of rebooting. If
no devices are found, check that the TCA9546A has power, common ground, SDA/SCL
are really on GPIO8/GPIO9, and the I2C lines have pullups.
