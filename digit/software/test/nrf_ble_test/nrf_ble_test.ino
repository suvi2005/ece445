#include <bluefruit.h>

BLEUart bleuart;

void startAdv(void);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(115200);
    delay(2000);

    Serial.println("Starting Bluefruit BLE UART server...");

    Bluefruit.begin();
    Serial.println("Bluefruit.begin() done");

    Bluefruit.setTxPower(4);
    Bluefruit.setName("Feather52832_UART");
    Serial.println("Name set");

    bleuart.begin();
    Serial.println("BLE UART service started");

    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    startAdv();

    Serial.println("Advertising started");
}

void loop() {
    while (bleuart.available()) {
        char c = (char) bleuart.read();
        Serial.print("Received over BLE: ");
        Serial.println(c);

        if (c == '1') {
        digitalWrite(LED_BUILTIN, HIGH);
        bleuart.println("LED ON");
        } else if (c == '0') {
        digitalWrite(LED_BUILTIN, LOW);
        bleuart.println("LED OFF");
        } else {
        bleuart.print("Echo: ");
        bleuart.println(c);
        }
    }
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
}

void connect_callback(uint16_t conn_handle) {
    BLEConnection* connection = Bluefruit.Connection(conn_handle);

    char central_name[32] = {0};
    connection->getPeerName(central_name, sizeof(central_name));

    Serial.print("Connected to: ");
    Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    (void) conn_handle;
    (void) reason;
    Serial.println("Disconnected");
}