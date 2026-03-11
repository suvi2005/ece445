#include <bluefruit.h>

BLEUart bleuart;

void startAdv(void);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void updateRedLedBehavior();
void handleIncomingLine(const String &line);
void redrawScreen();
const char *getRangeLabel();
uint32_t getBlinkIntervalForDistance(uint16_t distanceMm);

// =========================
// LED config
// =========================
#define RED_LED_PIN LED_BUILTIN

// =========================
// Blink thresholds
// =========================
static const uint16_t RANGE_VERY_CLOSE_MM = 200;
static const uint16_t RANGE_CLOSE_MM = 400;
static const uint16_t RANGE_MEDIUM_MM = 800;
static const uint16_t RANGE_FAR_MM = 1000;

static const uint32_t BLINK_VERY_FAST_MS = 40;
static const uint32_t BLINK_FAST_MS = 90;
static const uint32_t BLINK_MEDIUM_MS = 160;
static const uint32_t BLINK_SLOW_MS = 260;

// =========================
// Timeout / refresh config
// =========================
static const uint32_t DATA_TIMEOUT_MS = 1200;
static const uint32_t SCREEN_REFRESH_INTERVAL_MS = 100;

// =========================
// Runtime state
// =========================
bool bleConnected = false;
String rxLine = "";

bool haveRecentDistance = false;
uint16_t latestDistanceMm = 65535;
unsigned long lastDistanceRxMs = 0;

unsigned long lastRedToggleMs = 0;
bool redLedState = false;
unsigned long lastScreenRefreshMs = 0;

uint32_t currentBlinkIntervalMs = BLINK_SLOW_MS;
bool targetCurrentlyInRange = false;
uint32_t rxCount = 0;
String lastRxLine = "NONE";
String connectionState = "ADVERTISING";

void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);

  Serial.begin(115200);
  delay(2000);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Feather52832_UART");

  bleuart.begin();

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  startAdv();

  Serial.print("\033[2J");
  Serial.print("\033[H");
  redrawScreen();
}

void loop() {
  while (bleuart.available()) {
    char c = (char)bleuart.read();

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
      if (rxLine.length() > 64) {
        rxLine = "";
      }
    }
  }

  updateRedLedBehavior();

  if (millis() - lastScreenRefreshMs >= SCREEN_REFRESH_INTERVAL_MS) {
    lastScreenRefreshMs = millis();
    redrawScreen();
  }
}

void handleIncomingLine(const String &line) {
  rxCount++;
  lastRxLine = line;

  if (line.startsWith("DIST:")) {
    int parsed = line.substring(5).toInt();
    if (parsed > 0) {
      latestDistanceMm = (uint16_t)parsed;
      lastDistanceRxMs = millis();
      haveRecentDistance = true;

      currentBlinkIntervalMs = getBlinkIntervalForDistance(latestDistanceMm);
      targetCurrentlyInRange = (latestDistanceMm <= RANGE_FAR_MM);

      if (!targetCurrentlyInRange) {
        digitalWrite(RED_LED_PIN, LOW);
        redLedState = false;
      } else {
        redLedState = true;
        digitalWrite(RED_LED_PIN, HIGH);
        lastRedToggleMs = millis();
      }

      redrawScreen();
    }
    return;
  }

  if (line == "NO_TARGET") {
    haveRecentDistance = false;
    latestDistanceMm = 65535;
    currentBlinkIntervalMs = BLINK_SLOW_MS;
    targetCurrentlyInRange = false;
    lastDistanceRxMs = millis();

    digitalWrite(RED_LED_PIN, LOW);
    redLedState = false;

    redrawScreen();
    return;
  }
}

uint32_t getBlinkIntervalForDistance(uint16_t distanceMm) {
  if (distanceMm <= RANGE_VERY_CLOSE_MM) {
    return BLINK_VERY_FAST_MS;
  }
  if (distanceMm <= RANGE_CLOSE_MM) {
    return BLINK_FAST_MS;
  }
  if (distanceMm <= RANGE_MEDIUM_MM) {
    return BLINK_MEDIUM_MS;
  }
  if (distanceMm <= RANGE_FAR_MM) {
    return BLINK_SLOW_MS;
  }
  return BLINK_SLOW_MS;
}

const char *getRangeLabel() {
  if (!haveRecentDistance) {
    return "NO DATA";
  }
  if (latestDistanceMm <= RANGE_VERY_CLOSE_MM) {
    return "VERY CLOSE";
  }
  if (latestDistanceMm <= RANGE_CLOSE_MM) {
    return "CLOSE";
  }
  if (latestDistanceMm <= RANGE_MEDIUM_MM) {
    return "MEDIUM";
  }
  if (latestDistanceMm <= RANGE_FAR_MM) {
    return "FAR";
  }
  return "OUT OF RANGE";
}

void updateRedLedBehavior() {
  if (!bleConnected) {
    digitalWrite(RED_LED_PIN, LOW);
    redLedState = false;
    targetCurrentlyInRange = false;
    return;
  }

  if (!haveRecentDistance || (millis() - lastDistanceRxMs > DATA_TIMEOUT_MS)) {
    digitalWrite(RED_LED_PIN, LOW);
    redLedState = false;
    targetCurrentlyInRange = false;
    return;
  }

  if (!targetCurrentlyInRange) {
    digitalWrite(RED_LED_PIN, LOW);
    redLedState = false;
    return;
  }

  if (millis() - lastRedToggleMs >= currentBlinkIntervalMs) {
    lastRedToggleMs = millis();
    redLedState = !redLedState;
    digitalWrite(RED_LED_PIN, redLedState ? HIGH : LOW);
  }
}

void redrawScreen() {
  Serial.print("\033[2J");
  Serial.print("\033[H");

  Serial.println("nRF52832 BLE UART server");
  Serial.println("------------------------");

  Serial.print("Connection state: ");
  Serial.println(connectionState);

  Serial.print("BLE connected: ");
  Serial.println(bleConnected ? "YES" : "NO");

  Serial.print("RX count: ");
  Serial.println(rxCount);

  Serial.print("Last RX: ");
  Serial.println(lastRxLine);

  Serial.print("Latest distance mm: ");
  if (haveRecentDistance) {
    Serial.println(latestDistanceMm);
  } else {
    Serial.println("NONE");
  }

  Serial.print("Range bucket: ");
  Serial.println(getRangeLabel());

  Serial.print("Blink interval ms: ");
  if (haveRecentDistance && targetCurrentlyInRange) {
    Serial.println(currentBlinkIntervalMs);
  } else {
    Serial.println("LED OFF");
  }

  Serial.print("Age since last distance ms: ");
  if (haveRecentDistance) {
    Serial.println(millis() - lastDistanceRxMs);
  } else {
    Serial.println("NONE");
  }

  Serial.print("LED state: ");
  Serial.println(redLedState ? "ON" : "OFF");
}

void startAdv(void) {
  Bluefruit.Advertising.stop();

  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  connectionState = "ADVERTISING";
}

void connect_callback(uint16_t conn_handle) {
  BLEConnection *connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = {0};
  connection->getPeerName(central_name, sizeof(central_name));

  bleConnected = true;
  digitalWrite(RED_LED_PIN, LOW);
  redLedState = false;
  haveRecentDistance = false;
  latestDistanceMm = 65535;
  currentBlinkIntervalMs = BLINK_SLOW_MS;
  targetCurrentlyInRange = false;
  lastRxLine = "CONNECTED";
  connectionState = String("CONNECTED TO ") + central_name;

  redrawScreen();
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;

  bleConnected = false;
  haveRecentDistance = false;
  latestDistanceMm = 65535;
  currentBlinkIntervalMs = BLINK_SLOW_MS;
  targetCurrentlyInRange = false;
  rxLine = "";
  lastRxLine = "DISCONNECTED";

  digitalWrite(RED_LED_PIN, LOW);
  redLedState = false;

  connectionState = "DISCONNECTED / ADVERTISING";
  redrawScreen();
}