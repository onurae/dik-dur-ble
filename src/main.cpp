/*******************************************************************************************
 *                                                                                         *
 *    dik-dur-ble                                                                          *
 *                                                                                         *
 *    Copyright (c) 2024 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include "imu.hpp"

#define SERVICE_UUID "e653fc6d-d701-4a73-ab2c-794482caaba6"
#define CHARACTERISTIC_UUID_ANGLE "918f19a4-c5c1-4194-8fa4-d78a4eb9db94"
#define CHARACTERISTIC_UUID_BATTERY "28faaba7-417a-413c-a827-b247e68c12df"

BLEServer *pServer = nullptr;
BLEService *pService = nullptr;
BLECharacteristic *pCharacteristicAngle = nullptr;
BLECharacteristic *pCharacteristicBattery = nullptr;
BLEAdvertising *pAdvertising = nullptr;
bool deviceConnected = false;
IMU imu(Wire);
float dt = 0;
const uint16_t freqSensor = 50;
unsigned long prevTime;
unsigned long wakeTime;
const uint16_t freqBLE = 2;
unsigned long lastTimeBLE;
float sumBatVoltage = 0;
int16_t batCounter = 0;
float batVoltage = 0;

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
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    Wire.begin(23, 19);
    imu.Init();
    // imu.Calibration();
    BLEDevice::init("dd-ble");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    pService = pServer->createService(SERVICE_UUID);
    pCharacteristicAngle = pService->createCharacteristic(CHARACTERISTIC_UUID_ANGLE, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
    pCharacteristicBattery = pService->createCharacteristic(CHARACTERISTIC_UUID_BATTERY, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
    pService->start();
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x00);
    BLEDevice::startAdvertising();
    Serial.println("Ready to connect.");
    wakeTime = millis();
}

void loop()
{
    prevTime = wakeTime;
    while (millis() - prevTime < 1000 / freqSensor) {}
    wakeTime = millis();
    dt = (wakeTime - prevTime) * 0.001f;
    imu.Update(dt);
    // imu.PrintRawData();
    // imu.PrintCalibData();
    // imu.PrintEulerAngles();
    if (millis() - lastTimeBLE >= 1000 / freqBLE)
    {
        if (deviceConnected == true)
        {
            int16_t angle = imu.GetPhiDeg();
            pCharacteristicAngle->setValue((uint8_t *)&angle, 2);
            pCharacteristicAngle->notify();
            int16_t battery = batVoltage * 100;
            pCharacteristicBattery->setValue((uint8_t *)&battery, 2);
            pCharacteristicBattery->notify();
        }
        lastTimeBLE = millis();
    }
    sumBatVoltage += 3.3f * analogRead(34) / 4096.0f;
    batCounter++;
    if (batCounter >= freqSensor) // per second.
    {
        batVoltage = 2.0f * (sumBatVoltage / batCounter); // voltage divider: half
        // Serial.println(batVoltage);
        sumBatVoltage = 0;
        batCounter = 0;
    }
}