#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLERemoteCharacteristic.h>
#include <vl53l7cx_class.h>

#define DEV_I2C Wire
#define SerialPort Serial

// =========================
// VL53L7CX wiring
// =========================
#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7
#define LPN_PIN 5
#define I2C_RST_PIN 4

// =========================
// ESP32-C6 onboard RGB LED
// =========================
#define RGB_LED_PIN 8
#define RGB_LED_COUNT 1
Adafruit_NeoPixel rgb(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// =========================
// BLE settings
// ESP32 = central/client
// nRF   = peripheral/server
// =========================
static const char *TARGET_DEVICE_NAME = "Feather52832_UART";

static BLEUUID SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID RX_CHAR_UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // write to nRF
static BLEUUID TX_CHAR_UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // notify from nRF

// =========================
// Lidar settings
// =========================
VL53L7CX sensor_vl53l7cx_top(&DEV_I2C, LPN_PIN, I2C_RST_PIN);
static const uint8_t RESOLUTION = VL53L7CX_RESOLUTION_4X4;

// =========================
// BLE globals
// =========================
BLEAdvertisedDevice *targetDevice = nullptr;
BLEClient *pClient = nullptr;
BLERemoteCharacteristic *pRemoteRx = nullptr;
BLERemoteCharacteristic *pRemoteTx = nullptr;

bool bleConnected = false;
bool doConnect = false;

// =========================
// Runtime state
// =========================
String rxLine = "";
String lastAck = "NONE";
String lastNrfState = "NONE";

uint32_t txCount = 0;
uint32_t rxCount = 0;
uint32_t noTargetCount = 0;

unsigned long lastStatusSendMs = 0;
unsigned long lastScreenRefreshMs = 0;
unsigned long lastLedBlinkMs = 0;

bool ledBlinkState = false;

bool haveGrid = false;
bool haveAverage = false;
uint16_t lastAvgDistanceMm = 0;
VL53L7CX_ResultsData lastResults;

static const uint32_t STATUS_SEND_INTERVAL_MS = 1000;
static const uint32_t SCREEN_REFRESH_INTERVAL_MS = 80;
static const uint32_t LED_BLINK_INTERVAL_MS = 180;
static const uint32_t SCAN_DURATION_SECONDS = 5;

// =========================
// Forward declarations
// =========================
bool initLidar();
void initStatusLed();
void setStatusLed(uint8_t r, uint8_t g, uint8_t b);
void updateStatusLed();
void startScan();
bool connectToServer();
void sendMessage(const String &msg);
void handleIncomingLine(const String &line);
uint8_t getGridSize(uint8_t resolution);
uint16_t getPrimaryDistanceForZone(VL53L7CX_ResultsData *results, uint8_t zone);
bool computeAverageDistanceMm(VL53L7CX_ResultsData *results, uint8_t resolution, uint16_t &avgDistanceMm);
void printDivider(uint8_t gridSize);
void printDistanceGrid(VL53L7CX_ResultsData *results, uint8_t resolution);
void redrawScreen();
void printI2CScan();

// =========================
// BLE notification callback
// =========================
static void notifyCallback(
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData,
    size_t length,
    bool isNotify) {
  (void)pBLERemoteCharacteristic;
  (void)isNotify;

  for (size_t i = 0; i < length; i++) {
    char c = (char)pData[i];

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (rxLine.length() > 0) {
        handleIncomingLine(rxLine);
        rxLine = "";
      }
    } else {
      rxLine += c;
      if (rxLine.length() > 128) {
        rxLine = "";
      }
    }
  }
}

// =========================
// BLE advertised device callback
// =========================
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    String name = advertisedDevice.getName().c_str();

    if (name == TARGET_DEVICE_NAME) {
      SerialPort.print("Matched target nRF device: ");
      SerialPort.println(name);

      targetDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      BLEDevice::getScan()->stop();
    }
  }
};

void setup() {
  SerialPort.begin(115200);
  delay(1200);

  initStatusLed();
  updateStatusLed();

  SerialPort.print("\033[2J");
  SerialPort.print("\033[H");

  SerialPort.println("ESP32-C6 BLE client + VL53L7CX fast stream");
  SerialPort.println();

  if (!initLidar()) {
    SerialPort.println("Lidar init failed. Halting.");
    while (1) {
      updateStatusLed();
    }
  }

  BLEDevice::init("");
  startScan();
}

void loop() {
  updateStatusLed();

  if (!bleConnected) {
    if (doConnect) {
      if (connectToServer()) {
        SerialPort.println("Connected to nRF BLE UART server");
      } else {
        SerialPort.println("Failed to connect to nRF, rescanning...");
        bleConnected = false;
        doConnect = false;
        targetDevice = nullptr;
        pRemoteRx = nullptr;
        pRemoteTx = nullptr;
        if (pClient != nullptr) {
          delete pClient;
          pClient = nullptr;
        }
        startScan();
      }
    }

    if (millis() - lastScreenRefreshMs >= SCREEN_REFRESH_INTERVAL_MS) {
      lastScreenRefreshMs = millis();
      redrawScreen();
    }

    return;
  }

  VL53L7CX_ResultsData results;
  uint8_t dataReady = 0;
  uint8_t status = sensor_vl53l7cx_top.vl53l7cx_check_data_ready(&dataReady);

  if (status == 0 && dataReady) {
    status = sensor_vl53l7cx_top.vl53l7cx_get_ranging_data(&results);

    if (status == 0) {
      lastResults = results;
      haveGrid = true;

      uint16_t avgDistanceMm = 0;
      if (computeAverageDistanceMm(&results, RESOLUTION, avgDistanceMm)) {
        lastAvgDistanceMm = avgDistanceMm;
        haveAverage = true;
        sendMessage("DIST:" + String(avgDistanceMm));
      } else {
        haveAverage = false;
        noTargetCount++;
        sendMessage("NO_TARGET");
      }
    } else {
      SerialPort.print("get_ranging_data failed, status = ");
      SerialPort.println(status);
    }
  } else if (status != 0) {
    SerialPort.print("check_data_ready failed, status = ");
    SerialPort.println(status);
  }

  if (millis() - lastStatusSendMs >= STATUS_SEND_INTERVAL_MS) {
    lastStatusSendMs = millis();
    sendMessage("STATE:ESP32_CONNECTED");
  }

  if (pClient != nullptr && !pClient->isConnected()) {
    SerialPort.println("BLE disconnected");
    bleConnected = false;
    pRemoteRx = nullptr;
    pRemoteTx = nullptr;
    doConnect = false;
    targetDevice = nullptr;
    if (pClient != nullptr) {
      delete pClient;
      pClient = nullptr;
    }
    startScan();
  }

  if (millis() - lastScreenRefreshMs >= SCREEN_REFRESH_INTERVAL_MS) {
    lastScreenRefreshMs = millis();
    redrawScreen();
  }
}

void handleIncomingLine(const String &line) {
  rxCount++;

  if (line.startsWith("ACK:")) {
    lastAck = line;
  } else if (line.startsWith("STATE:")) {
    lastNrfState = line;
  } else if (line == "PING") {
    sendMessage("PONG");
  }
}

void sendMessage(const String &msg) {
  if (!bleConnected || pRemoteRx == nullptr) {
    return;
  }

  String out = msg + "\n";
  pRemoteRx->writeValue((uint8_t *)out.c_str(), out.length(), false);
  txCount++;
}

bool initLidar() {
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

  printI2CScan();

  sensor_vl53l7cx_top.begin();

  uint8_t status = sensor_vl53l7cx_top.init_sensor();
  if (status != 0) {
    SerialPort.print("vl53l7cx init failed, status = ");
    SerialPort.println(status);
    return false;
  }

  status = sensor_vl53l7cx_top.vl53l7cx_set_resolution(RESOLUTION);
  if (status != 0) {
    SerialPort.print("set_resolution failed, status = ");
    SerialPort.println(status);
    return false;
  }

  status = sensor_vl53l7cx_top.vl53l7cx_set_ranging_frequency_hz(10);
  if (status != 0) {
    SerialPort.print("set_ranging_frequency_hz failed, status = ");
    SerialPort.println(status);
    return false;
  }

  status = sensor_vl53l7cx_top.vl53l7cx_start_ranging();
  if (status != 0) {
    SerialPort.print("start_ranging failed, status = ");
    SerialPort.println(status);
    return false;
  }

  SerialPort.println("Ranging started");
  return true;
}

void initStatusLed() {
  rgb.begin();
  rgb.setBrightness(32);
  rgb.clear();
  rgb.show();
}

void setStatusLed(uint8_t r, uint8_t g, uint8_t b) {
  rgb.setPixelColor(0, rgb.Color(r, g, b));
  rgb.show();
}

void updateStatusLed() {
  if (bleConnected) {
    setStatusLed(0, 0, 80);
    return;
  }

  if (millis() - lastLedBlinkMs >= LED_BLINK_INTERVAL_MS) {
    lastLedBlinkMs = millis();
    ledBlinkState = !ledBlinkState;
    if (ledBlinkState) {
      setStatusLed(0, 0, 80);
    } else {
      setStatusLed(0, 0, 0);
    }
  }
}

void startScan() {
  SerialPort.println("Starting BLE scan...");

  BLEScan *pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(80);
  pScan->start(SCAN_DURATION_SECONDS, false);
}

bool connectToServer() {
  if (targetDevice == nullptr) {
    return false;
  }

  pClient = BLEDevice::createClient();

  SerialPort.print("Connecting to: ");
  SerialPort.println(targetDevice->getAddress().toString().c_str());

  if (!pClient->connect(targetDevice)) {
    SerialPort.println("Client connect failed");
    return false;
  }

  BLERemoteService *pService = pClient->getService(SERVICE_UUID);
  if (pService == nullptr) {
    SerialPort.println("Failed to find NUS service");
    pClient->disconnect();
    return false;
  }

  pRemoteRx = pService->getCharacteristic(RX_CHAR_UUID);
  pRemoteTx = pService->getCharacteristic(TX_CHAR_UUID);

  if (pRemoteRx == nullptr || pRemoteTx == nullptr) {
    SerialPort.println("Failed to find NUS characteristics");
    pClient->disconnect();
    return false;
  }

  if (!pRemoteTx->canNotify()) {
    SerialPort.println("TX characteristic cannot notify");
    pClient->disconnect();
    return false;
  }

  pRemoteTx->registerForNotify(notifyCallback);

  bleConnected = true;
  doConnect = false;
  return true;
}

void printI2CScan() {
  int found = 0;
  SerialPort.println("Scanning I2C bus...");

  for (uint8_t addr = 1; addr < 127; addr++) {
    DEV_I2C.beginTransmission(addr);
    uint8_t err = DEV_I2C.endTransmission();

    if (err == 0) {
      SerialPort.print("Found I2C device at 0x");
      if (addr < 16) {
        SerialPort.print("0");
      }
      SerialPort.println(addr, HEX);
      found++;
    }
  }

  if (found == 0) {
    SerialPort.println("No I2C devices found");
  } else {
    SerialPort.print("Total I2C devices found: ");
    SerialPort.println(found);
  }
}

uint8_t getGridSize(uint8_t resolution) {
  if (resolution == VL53L7CX_RESOLUTION_4X4) return 4;
  if (resolution == VL53L7CX_RESOLUTION_8X8) return 8;
  return 0;
}

uint16_t getPrimaryDistanceForZone(VL53L7CX_ResultsData *results, uint8_t zone) {
  uint8_t targetIndex = zone * VL53L7CX_NB_TARGET_PER_ZONE;
  return results->distance_mm[targetIndex];
}

bool computeAverageDistanceMm(VL53L7CX_ResultsData *results, uint8_t resolution, uint16_t &avgDistanceMm) {
  uint8_t gridSize = getGridSize(resolution);
  if (gridSize == 0) return false;

  uint32_t sum = 0;
  uint16_t count = 0;
  uint8_t totalZones = gridSize * gridSize;

  for (uint8_t zone = 0; zone < totalZones; zone++) {
    if (results->nb_target_detected[zone] > 0) {
      uint16_t distance = getPrimaryDistanceForZone(results, zone);
      if (distance > 0) {
        sum += distance;
        count++;
      }
    }
  }

  if (count == 0) {
    return false;
  }

  avgDistanceMm = (uint16_t)(sum / count);
  return true;
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

void redrawScreen() {
  SerialPort.print("\033[2J");
  SerialPort.print("\033[H");

  SerialPort.println("ESP32-C6 BLE client + VL53L7CX fast stream");
  SerialPort.println("------------------------------------------");
  SerialPort.print("BLE connected: ");
  SerialPort.println(bleConnected ? "YES" : "NO");

  SerialPort.print("TX count: ");
  SerialPort.print(txCount);
  SerialPort.print("   RX count: ");
  SerialPort.print(rxCount);
  SerialPort.print("   NO_TARGET count: ");
  SerialPort.println(noTargetCount);

  SerialPort.print("Last ACK: ");
  SerialPort.println(lastAck);

  SerialPort.print("Last nRF state: ");
  SerialPort.println(lastNrfState);

  SerialPort.print("Average distance mm sent to nRF: ");
  if (haveAverage) {
    SerialPort.println(lastAvgDistanceMm);
  } else {
    SerialPort.println("NONE");
  }

  SerialPort.print("Grid resolution: ");
  if (RESOLUTION == VL53L7CX_RESOLUTION_4X4) {
    SerialPort.println("4x4");
  } else {
    SerialPort.println("8x8");
  }

  SerialPort.println();

  if (haveGrid) {
    printDistanceGrid(&lastResults, RESOLUTION);
  } else {
    SerialPort.println("No lidar frame yet");
  }
}