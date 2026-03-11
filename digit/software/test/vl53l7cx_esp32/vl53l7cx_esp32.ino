#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>

#define DEV_I2C Wire
#define SerialPort Serial

#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7

#define LPN_PIN 5
#define I2C_RST_PIN 4
// #define PWREN_PIN 8

VL53L7CX sensor_vl53l7cx_top(&DEV_I2C, LPN_PIN, I2C_RST_PIN);

// Change to VL53L7CX_RESOLUTION_8X8 if needed
static const uint8_t RESOLUTION = VL53L7CX_RESOLUTION_4X4;

unsigned long lastPrintMs = 0;
const unsigned long printIntervalMs = 300;

uint8_t getGridSize(uint8_t resolution);
uint16_t getPrimaryDistanceForZone(VL53L7CX_ResultsData *results, uint8_t zone);
void printDivider(uint8_t gridSize);
void printDistanceGrid(VL53L7CX_ResultsData *results, uint8_t resolution);

void setup() {
  SerialPort.begin(115200);
  delay(1000);

  SerialPort.println();
  SerialPort.println("Initializing VL53L7CX on ESP32-C6...");

  // pinMode(PWREN_PIN, OUTPUT);
  // digitalWrite(PWREN_PIN, HIGH);

  pinMode(LPN_PIN, OUTPUT);
  digitalWrite(LPN_PIN, HIGH);

  pinMode(I2C_RST_PIN, OUTPUT);
  digitalWrite(I2C_RST_PIN, HIGH);

  delay(50);

  DEV_I2C.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  delay(50);

  SerialPort.print("I2C initialized on SDA=");
  SerialPort.print(I2C_SDA_PIN);
  SerialPort.print(" SCL=");
  SerialPort.println(I2C_SCL_PIN);

  sensor_vl53l7cx_top.begin();

  uint8_t status = sensor_vl53l7cx_top.init_sensor();
  if (status != 0) {
    SerialPort.print("vl53l7cx init failed, status = ");
    SerialPort.println(status);
    while (1) {
      delay(1000);
    }
  }

  SerialPort.println("Sensor init OK");

  status = sensor_vl53l7cx_top.vl53l7cx_set_resolution(RESOLUTION);
  if (status != 0) {
    SerialPort.print("set_resolution failed, status = ");
    SerialPort.println(status);
    while (1) {
      delay(1000);
    }
  }

  status = sensor_vl53l7cx_top.vl53l7cx_set_ranging_frequency_hz(10);
  if (status != 0) {
    SerialPort.print("set_ranging_frequency_hz failed, status = ");
    SerialPort.println(status);
    while (1) {
      delay(1000);
    }
  }

  status = sensor_vl53l7cx_top.vl53l7cx_start_ranging();
  if (status != 0) {
    SerialPort.print("start_ranging failed, status = ");
    SerialPort.println(status);
    while (1) {
      delay(1000);
    }
  }

  SerialPort.println("Ranging started");
  SerialPort.println();
}

void loop() {
  VL53L7CX_ResultsData results;
  uint8_t dataReady = 0;
  uint8_t status = 0;

  status = sensor_vl53l7cx_top.vl53l7cx_check_data_ready(&dataReady);
  if (status != 0) {
    SerialPort.print("check_data_ready failed, status = ");
    SerialPort.println(status);
    delay(100);
    return;
  }

  if (dataReady) {
    status = sensor_vl53l7cx_top.vl53l7cx_get_ranging_data(&results);
    if (status != 0) {
      SerialPort.print("get_ranging_data failed, status = ");
      SerialPort.println(status);
      delay(100);
      return;
    }

    if (millis() - lastPrintMs >= printIntervalMs) {
      lastPrintMs = millis();

      SerialPort.println("========================================");
      SerialPort.print("Distance Grid ");
      if (RESOLUTION == VL53L7CX_RESOLUTION_4X4) {
        SerialPort.println("(4x4)");
      } else if (RESOLUTION == VL53L7CX_RESOLUTION_8X8) {
        SerialPort.println("(8x8)");
      } else {
        SerialPort.println("(unknown)");
      }

      printDistanceGrid(&results, RESOLUTION);
      SerialPort.println();
    }
  }

  delay(20);
}

uint8_t getGridSize(uint8_t resolution) {
  if (resolution == VL53L7CX_RESOLUTION_4X4) {
    return 4;
  }
  if (resolution == VL53L7CX_RESOLUTION_8X8) {
    return 8;
  }
  return 0;
}

uint16_t getPrimaryDistanceForZone(VL53L7CX_ResultsData *results,
                                   uint8_t zone) {
  uint8_t targetIndex = zone * VL53L7CX_NB_TARGET_PER_ZONE;
  return results->distance_mm[targetIndex];
}

void printDivider(uint8_t gridSize) {
  for (uint8_t col = 0; col < gridSize; col++) {
    SerialPort.print("+--------");
  }
  SerialPort.println("+");
}

void printDistanceGrid(VL53L7CX_ResultsData *results, uint8_t resolution) {
  uint8_t gridSize = getGridSize(resolution);
  if (gridSize == 0) {
    SerialPort.println("Unsupported resolution");
    return;
  }

  printDivider(gridSize);

  for (uint8_t row = 0; row < gridSize; row++) {
    for (uint8_t col = 0; col < gridSize; col++) {
      uint8_t zone = row * gridSize + col;

      if (results->nb_target_detected[zone] > 0) {
        uint16_t distance = getPrimaryDistanceForZone(results, zone);
        char buf[9];
        snprintf(buf, sizeof(buf), "%6u", distance);
        SerialPort.print("|");
        SerialPort.print(buf);
        SerialPort.print(" ");
      } else {
        SerialPort.print("|   --   ");
      }
    }
    SerialPort.println("|");
    printDivider(gridSize);
  }
}