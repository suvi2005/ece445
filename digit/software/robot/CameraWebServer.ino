#include "esp_camera.h"
#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <TB6612_ESP32.h>
#include "Adafruit_VL53L0X.h"

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
#define CAMERA_MODEL_ESP32S3_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
//#define CAMERA_MODEL_M5STACK_ESP32CAM
//#define CAMERA_MODEL_M5STACK_UNITCAM
//#define CAMERA_MODEL_M5STACK_CAMS3_UNIT
//#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_TTGO_T_JOURNAL
//#define CAMERA_MODEL_XIAO_ESP32S3
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3
#include "camera_pins.h"

// =========================
// Pin configuration
// =========================
#define AIN1 46
#define BIN1 21
#define AIN2 14
#define BIN2 38
#define PWMA 47
#define PWMB 48
#define STBY 100

#define SDA_PIN 1
#define SCL_PIN 2

// =========================
// Stream verification
// =========================
const uint16_t VERIFY_MIN_WIDTH  = 320;
const uint16_t VERIFY_MIN_HEIGHT = 240;
const float    VERIFY_MIN_FPS    = 10.0;
const uint32_t VERIFY_TEST_MS    = 120000; // 2 minutes
const uint32_t VERIFY_MAX_FREEZE_MS = 2000;

volatile uint32_t verifyStartMs = 0;
volatile uint32_t verifyLastFrameMs = 0;
volatile uint32_t verifyFrameCount = 0;
volatile uint32_t verifyFreezeMs = 0;
volatile uint32_t verifyLongestGapMs = 0;
volatile bool verifyActive = false;
volatile bool verifyDone = false;
volatile bool verifyResolutionPass = false;
volatile uint16_t verifyWidth = 0;
volatile uint16_t verifyHeight = 0;

// =========================
// Motor configuration
// =========================
const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY, 5000, 8, 1);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY, 5000, 8, 2);

const int DRIVE_SPEED = 180;
char currentCommand = ' ';
const uint16_t BRAKE_DISTANCE_MM = 100;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool obstacleDetected = false;

unsigned long lastLidarReadMs = 0;
const unsigned long LIDAR_PERIOD_MS = 100;

// =========================
// WiFi credentials
// =========================
const char *ssid = "Palantiri";
const char *password = "eNeM3fFsT";

void startCameraServer();
void setupLedFlash(int pin);

// =========================
// Verification helpers
// =========================
void startStreamVerification(uint16_t width, uint16_t height) {
  verifyStartMs = millis();
  verifyLastFrameMs = verifyStartMs;
  verifyFrameCount = 0;
  verifyFreezeMs = 0;
  verifyLongestGapMs = 0;
  verifyDone = false;
  verifyActive = true;

  verifyWidth = width;
  verifyHeight = height;
  verifyResolutionPass = (width >= VERIFY_MIN_WIDTH && height >= VERIFY_MIN_HEIGHT);

  Serial.println("===== STREAM VERIFICATION STARTED =====");
  Serial.printf("Resolution: %u x %u\n", width, height);
  Serial.printf("Required: >= %u x %u, >= %.1f fps, <= %lu ms freeze, duration %lu ms\n",
                VERIFY_MIN_WIDTH, VERIFY_MIN_HEIGHT,
                VERIFY_MIN_FPS,
                (unsigned long)VERIFY_MAX_FREEZE_MS,
                (unsigned long)VERIFY_TEST_MS);
}

void recordStreamedFrame() {
  if (!verifyActive || verifyDone) return;

  uint32_t now = millis();

  if (verifyFrameCount == 0) {
    verifyLastFrameMs = now;
    verifyFrameCount = 1;
    return;
  }

  uint32_t gap = now - verifyLastFrameMs;
  verifyLastFrameMs = now;
  verifyFrameCount++;

  if (gap > verifyLongestGapMs) {
    verifyLongestGapMs = gap;
  }

  const uint32_t allowedGapMs = (uint32_t)(1000.0 / VERIFY_MIN_FPS);

  if (gap > allowedGapMs) {
    verifyFreezeMs += (gap - allowedGapMs);
  }

  if ((now - verifyStartMs) >= VERIFY_TEST_MS) {
    verifyDone = true;
    verifyActive = false;
  }
}

void printVerificationReport() {
  uint32_t elapsed = verifyLastFrameMs - verifyStartMs;
  float avgFps = 0.0f;
  if (elapsed > 0) {
    avgFps = (1000.0f * verifyFrameCount) / elapsed;
  }

  bool fpsPass = (avgFps >= VERIFY_MIN_FPS);
  bool freezePass = (verifyFreezeMs <= VERIFY_MAX_FREEZE_MS);
  bool overallPass = verifyResolutionPass && fpsPass && freezePass;

  Serial.println();
  Serial.println("===== STREAM VERIFICATION REPORT =====");
  Serial.printf("Test duration (ms): %lu\n", (unsigned long)elapsed);
  Serial.printf("Frames streamed: %lu\n", (unsigned long)verifyFrameCount);
  Serial.printf("Average FPS: %.2f\n", avgFps);
  Serial.printf("Resolution: %u x %u\n", verifyWidth, verifyHeight);
  Serial.printf("Total freeze time (ms): %lu\n", (unsigned long)verifyFreezeMs);
  Serial.printf("Longest frame gap (ms): %lu\n", (unsigned long)verifyLongestGapMs);
  Serial.printf("Resolution pass: %s\n", verifyResolutionPass ? "YES" : "NO");
  Serial.printf("FPS pass: %s\n", fpsPass ? "YES" : "NO");
  Serial.printf("Freeze pass: %s\n", freezePass ? "YES" : "NO");
  Serial.printf("OVERALL RESULT: %s\n", overallPass ? "PASS" : "FAIL");
  Serial.println("======================================");
  Serial.println();
}

// =========================
// Robot helpers
// =========================
bool commandMovesForward(char cmd) {
  return (cmd == 'w' || cmd == 'W');
}

void stopRobot() {
  brake(motor1, motor2);
}

void applyCommand(char cmd) {
  if (obstacleDetected && commandMovesForward(cmd)) {
    stopRobot();
    Serial.println("Blocked: obstacle within 100 mm");
    return;
  }

  switch (cmd) {
    case 'w':
    case 'W':
      forward(motor1, motor2, DRIVE_SPEED);
      Serial.println("Forward");
      break;
    case 's':
    case 'S':
      back(motor1, motor2, DRIVE_SPEED);
      Serial.println("Backward");
      break;
    case 'a':
    case 'A':
      left(motor1, motor2, DRIVE_SPEED);
      Serial.println("Left");
      break;
    case 'd':
    case 'D':
      right(motor1, motor2, DRIVE_SPEED);
      Serial.println("Right");
      break;
    case ' ':
      stopRobot();
      Serial.println("Stop");
      break;
    default:
      break;
  }
}

void readLidarAndEnforceBrake() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  bool validReading = (measure.RangeStatus != 4);
  bool nowObstacleDetected = false;

  if (validReading) {
    uint16_t distanceMm = measure.RangeMilliMeter;

    Serial.print("Distance (mm): ");
    Serial.println(distanceMm);

    if (distanceMm <= BRAKE_DISTANCE_MM) {
      nowObstacleDetected = true;
    }
  }

  if (nowObstacleDetected && !obstacleDetected) {
    Serial.println("Obstacle detected: braking");
  } else if (!nowObstacleDetected && obstacleDetected) {
    Serial.println("Obstacle cleared");
  }

  obstacleDetected = nowObstacleDetected;

  if (obstacleDetected && commandMovesForward(currentCommand)) {
    stopRobot();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println();
  Serial.println("Initializing VL53L0X...");

  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while (1) {
      delay(10);
    }
  }

  stopRobot();

  Serial.println("Robot + LiDAR control ready.");
  Serial.println("Use:");
  Serial.println("  w = forward");
  Serial.println("  s = backward");
  Serial.println("  a = left");
  Serial.println("  d = right");
  Serial.println("  space = stop");
  Serial.println();
  Serial.print("Automatic brake threshold: ");
  Serial.print(BRAKE_DISTANCE_MM);
  Serial.println(" mm");
  Serial.println();

  Serial.println("cam setup");
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  Serial.println("esp_camera_init");
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  Serial.println("esp_camera_init ok");

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA); // 320x240
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  Serial.println("wifi connect start");
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();
  startStreamVerification(320, 240);

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      continue;
    }

    currentCommand = c;
    applyCommand(currentCommand);
  }

  unsigned long now = millis();
  if (now - lastLidarReadMs >= LIDAR_PERIOD_MS) {
    lastLidarReadMs = now;
    readLidarAndEnforceBrake();
  }

  static bool reportPrinted = false;
  if (verifyDone && !reportPrinted) {
    printVerificationReport();
    reportPrinted = true;
  }
}