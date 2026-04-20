#include <Wire.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SDA_PIN 8
#define SCL_PIN 9

#define IMU_ADDR_LOW  0x6A
#define IMU_ADDR_HIGH 0x6B

uint8_t imuAddr = 0x00;

// ---------------- IMU REGISTERS ----------------
static const uint8_t REG_WHO_AM_I   = 0x0F;
static const uint8_t REG_CTRL1_XL   = 0x10;
static const uint8_t REG_CTRL2_G    = 0x11;
static const uint8_t REG_CTRL3_C    = 0x12;
static const uint8_t REG_OUT_TEMP_L = 0x20;
static const uint8_t REG_OUTX_L_G   = 0x22;
static const uint8_t REG_OUTX_L_A   = 0x28;

static const uint8_t WHO_AM_I_EXPECTED = 0x6C;

// accel: ±2g  => 0.061 mg/LSB
// gyro : ±250 dps => 8.75 mdps/LSB
static const float ACCEL_MG_PER_LSB = 0.061f;
static const float GYRO_MDPS_PER_LSB = 8.75f;

// ---------------- AXIS REMAP ----------------
// Update these if your board orientation changes.
// These settings assume you already figured out the sign directions.
static const int ACCEL_MAP[3] = {0, 1, 2};
static const int ACCEL_SIGN[3] = {-1, -1, -1};

static const int GYRO_MAP[3]  = {0, 1, 2};
static const int GYRO_SIGN[3] = {-1, -1, -1};

// ---------------- BLE SETTINGS ----------------
#define BLE_DEVICE_NAME         "ESP32S3_GLOVE"
#define BLE_SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define BLE_CHARACTERISTIC_UUID "abcdefab-1234-1234-1234-abcdefabcdef"

BLECharacteristic *pCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// ---------------- GESTURE SETTINGS ----------------
// Threshold angle in degrees before a movement command is triggered.
// Start around 20 degrees and tune later.
static const float TILT_THRESHOLD_DEG = 20.0f;

// How often to evaluate/send command
static const unsigned long COMMAND_UPDATE_MS = 100;

// ---------------- BLE CALLBACKS ----------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    deviceConnected = true;
    Serial.println("\nBLE client connected");
  }

  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    Serial.println("\nBLE client disconnected");
  }
};

// ---------------- IMU HELPERS ----------------
bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool readRegisters(uint8_t addr, uint8_t startReg, uint8_t *buffer, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t received = Wire.requestFrom((int)addr, (int)len);
  if (received != len) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool readRegister(uint8_t addr, uint8_t reg, uint8_t &value) {
  return readRegisters(addr, reg, &value, 1);
}

int16_t combineInt16(uint8_t lowByte, uint8_t highByte) {
  return (int16_t)((highByte << 8) | lowByte);
}

bool detectIMU(uint8_t addr) {
  uint8_t whoami = 0;
  if (!readRegister(addr, REG_WHO_AM_I, whoami)) {
    return false;
  }

  Serial.print("Address 0x");
  Serial.print(addr, HEX);
  Serial.print(" WHO_AM_I = 0x");
  Serial.println(whoami, HEX);

  return (whoami == WHO_AM_I_EXPECTED);
}

bool initIMU(uint8_t addr) {
  if (!writeRegister(addr, REG_CTRL3_C, 0x44)) return false; // BDU + IF_INC
  if (!writeRegister(addr, REG_CTRL1_XL, 0x40)) return false; // accel 104 Hz, ±2g
  if (!writeRegister(addr, REG_CTRL2_G, 0x40)) return false;  // gyro 104 Hz, ±250 dps
  delay(100);
  return true;
}

void remapAxes(
  float inX, float inY, float inZ,
  const int mapIdx[3],
  const int signArr[3],
  float &outX, float &outY, float &outZ
) {
  float v[3] = {inX, inY, inZ};
  outX = signArr[0] * v[mapIdx[0]];
  outY = signArr[1] * v[mapIdx[1]];
  outZ = signArr[2] * v[mapIdx[2]];
}

bool readIMU(
  float &ax_g, float &ay_g, float &az_g,
  float &gx_dps, float &gy_dps, float &gz_dps,
  float &temp_c
) {
  uint8_t gyroBuf[6];
  uint8_t accelBuf[6];
  uint8_t tempBuf[2];

  if (!readRegisters(imuAddr, REG_OUTX_L_G, gyroBuf, 6)) return false;
  if (!readRegisters(imuAddr, REG_OUTX_L_A, accelBuf, 6)) return false;
  if (!readRegisters(imuAddr, REG_OUT_TEMP_L, tempBuf, 2)) return false;

  int16_t gx_raw = combineInt16(gyroBuf[0], gyroBuf[1]);
  int16_t gy_raw = combineInt16(gyroBuf[2], gyroBuf[3]);
  int16_t gz_raw = combineInt16(gyroBuf[4], gyroBuf[5]);

  int16_t ax_raw = combineInt16(accelBuf[0], accelBuf[1]);
  int16_t ay_raw = combineInt16(accelBuf[2], accelBuf[3]);
  int16_t az_raw = combineInt16(accelBuf[4], accelBuf[5]);

  int16_t temp_raw = combineInt16(tempBuf[0], tempBuf[1]);

  float ax0 = (ax_raw * ACCEL_MG_PER_LSB) / 1000.0f;
  float ay0 = (ay_raw * ACCEL_MG_PER_LSB) / 1000.0f;
  float az0 = (az_raw * ACCEL_MG_PER_LSB) / 1000.0f;

  float gx0 = (gx_raw * GYRO_MDPS_PER_LSB) / 1000.0f;
  float gy0 = (gy_raw * GYRO_MDPS_PER_LSB) / 1000.0f;
  float gz0 = (gz_raw * GYRO_MDPS_PER_LSB) / 1000.0f;

  remapAxes(ax0, ay0, az0, ACCEL_MAP, ACCEL_SIGN, ax_g, ay_g, az_g);
  remapAxes(gx0, gy0, gz0, GYRO_MAP, GYRO_SIGN, gx_dps, gy_dps, gz_dps);

  temp_c = 25.0f + (temp_raw / 256.0f);
  return true;
}

// ---------------- BLE SETUP ----------------
void setupBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    BLE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("boot");

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  Serial.println("BLE advertising started");
  Serial.print("Device name: ");
  Serial.println(BLE_DEVICE_NAME);
}

// ---------------- COMMAND LOGIC ----------------
float radiansToDegrees(float r) {
  return r * 180.0f / PI;
}

const char* getTiltCommand(float ax, float ay, float az) {
  // pitch: forward/back tilt
  // roll : left/right tilt
  //
  // These formulas assume:
  // - flat: az magnitude is largest
  // - x controls forward/back
  // - y controls left/right
  //
  // If directions are reversed, swap the returned strings below.

  float pitchDeg = radiansToDegrees(atan2(ax, sqrt(ay * ay + az * az)));
  float rollDeg  = radiansToDegrees(atan2(ay, sqrt(ax * ax + az * az)));

  Serial.printf(
    "A[g] X:%6.3f Y:%6.3f Z:%6.3f | pitch:%7.2f roll:%7.2f   ",
    ax, ay, az, pitchDeg, rollDeg
  );

  // Choose dominant tilt axis so diagonal tilts do not send two commands
  if (fabs(pitchDeg) >= fabs(rollDeg)) {
    if (pitchDeg > TILT_THRESHOLD_DEG)  return "f";
    if (pitchDeg < -TILT_THRESHOLD_DEG) return "b";
  } else {
    if (rollDeg > TILT_THRESHOLD_DEG)   return "r";
    if (rollDeg < -TILT_THRESHOLD_DEG)  return "l";
  }

  return "stop";
}

void sendBLEMessage(const char *msg) {
  if (!pCharacteristic) return;

  pCharacteristic->setValue((uint8_t*)msg, strlen(msg));
  pCharacteristic->notify();

  Serial.print(" | sent: ");
  Serial.println(msg);
}

// ---------------- MAIN SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\nStarting...");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (detectIMU(IMU_ADDR_LOW)) {
    imuAddr = IMU_ADDR_LOW;
  } else if (detectIMU(IMU_ADDR_HIGH)) {
    imuAddr = IMU_ADDR_HIGH;
  } else {
    Serial.println("Could not find LSM6DSOX");
    while (true) delay(1000);
  }

  if (!initIMU(imuAddr)) {
    Serial.println("IMU init failed");
    while (true) delay(1000);
  }

  Serial.println("IMU init succeeded");
  setupBLE();
}

// ---------------- MAIN LOOP ----------------
void loop() {
  static unsigned long lastCommandUpdate = 0;
  static String lastCommand = "";

  if (millis() - lastCommandUpdate >= COMMAND_UPDATE_MS) {
    lastCommandUpdate = millis();

    float ax, ay, az;
    float gx, gy, gz;
    float tempC;

    if (!readIMU(ax, ay, az, gx, gy, gz, tempC)) {
      Serial.println("Read failed");
    } else {
      const char *cmd = getTiltCommand(ax, ay, az);

      if (String(cmd) != lastCommand) {
        lastCommand = String(cmd);

        if (deviceConnected) {
          sendBLEMessage(cmd);
        } else {
          Serial.print(" | cmd: ");
          Serial.println(cmd);
        }
      } else {
        Serial.print(" | cmd: ");
        Serial.println(cmd);
      }
    }
  }

  // Restart advertising after disconnect
  if (!deviceConnected && oldDeviceConnected) {
    delay(200);
    BLEDevice::startAdvertising();
    Serial.println("Restarted advertising");
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}