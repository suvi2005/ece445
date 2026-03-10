#include <NimBLEDevice.h>

static const char *DEVICE_NAME = "ESP32C6_CMD";
static const char *SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab";
static const char *CHARACTERISTIC_UUID = "abcdefab-1234-1234-1234-abcdefabcdef";

NimBLEServer *serverPtr = nullptr;
NimBLECharacteristic *characteristicPtr = nullptr;
bool isConnected = false;

class ServerCallbacks : public NimBLEServerCallbacks{
                            void onConnect(NimBLEServer * server, NimBLEConnInfo &connInfo) override{
                                isConnected = true;
Serial.println("Client connected");
}

void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override
{
    isConnected = false;
    Serial.println("Client disconnected");
    NimBLEDevice::startAdvertising();
    Serial.println("Advertising restarted");
}
}
;

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks{
                                    void onRead(NimBLECharacteristic * characteristic, NimBLEConnInfo &connInfo) override{
                                        Serial.println("Characteristic was read");
}

void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo) override
{
    std::string value = characteristic->getValue();
    Serial.print("Received command: ");
    Serial.println(value.c_str());

    std::string response = "ACK: " + value;
    characteristic->setValue(response);
    characteristic->notify();

    Serial.print("Sent response: ");
    Serial.println(response.c_str());
}
}
;

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("Starting BLE command interface");

    NimBLEDevice::init(DEVICE_NAME);
    NimBLEDevice::setPower(3);

    serverPtr = NimBLEDevice::createServer();
    serverPtr->setCallbacks(new ServerCallbacks());

    NimBLEService *service = serverPtr->createService(SERVICE_UUID);

    characteristicPtr = service->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ |
            NIMBLE_PROPERTY::WRITE |
            NIMBLE_PROPERTY::NOTIFY);

    characteristicPtr->setValue("READY");
    characteristicPtr->setCallbacks(new CharacteristicCallbacks());

    service->start();

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->setName(DEVICE_NAME);
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->enableScanResponse(true);

    bool ok = NimBLEDevice::startAdvertising();

    if (ok)
    {
        Serial.println("Advertising started");
    }
    else
    {
        Serial.println("Advertising failed");
    }

    Serial.print("Device name: ");
    Serial.println(DEVICE_NAME);
}

void loop()
{
    delay(20);
}