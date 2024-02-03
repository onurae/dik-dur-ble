#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "e653fc6d-d701-4a73-ab2c-794482caaba6"
#define CHARACTERISTIC_UUID "918f19a4-c5c1-4194-8fa4-d78a4eb9db94"

BLEServer *pServer = nullptr;
BLEService *pService = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
BLEAdvertising *pAdvertising = nullptr;
bool deviceConnected = false;
unsigned long period = 500;
unsigned long lastTime;
int16_t angle = 0;

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        Serial.println("Device connected.");
    }
    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        Serial.println("Device disconnected.");
        BLEDevice::startAdvertising();
    }
};

void setup()
{
    Serial.begin(115200);
    BLEDevice::init("dd-ble");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
    pService->start();
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x00);
    BLEDevice::startAdvertising();
    Serial.println("Device is waiting for connection...");
}

void loop()
{
    if (millis() - lastTime >= period)
    {
        angle += 1;
        if (deviceConnected == true)
        {
            pCharacteristic->setValue((uint8_t *)&angle, 2);
            pCharacteristic->notify();
        }
        lastTime = millis();
    }
}