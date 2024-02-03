/*******************************************************************************************
 *                                                                                         *
 *    dik-dur-ble, imu                                                                     *
 *                                                                                         *
 *    Copyright (c) 2024 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include "Wire.h"

class IMU
{
private:
    uint8_t address = 0x68;
    TwoWire *wire;
    bool Write(const uint8_t reg, const uint8_t data);
    uint8_t Read(const uint8_t reg);
    float aRaw[3];
    float tRaw;
    float gRaw[3];
    float aCal[3];
    float gCal[3];
    float aBias[3] = {0.061981f, -0.017035f, -0.116900f};
    float gBias[3] = {-0.169571f, 0.339561f, -1.594718f};
    bool ReadSensorData();
    float gRes;
    float aRes;
    float beta = 0.041;
    float q0 = 1.0;
    float q1 = 0.0;
    float q2 = 0.0;
    float q3 = 0.0;
    float InvSqrt(float x);
    void Madgwick6(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    void CalcEulerAngles();
    float phi;   // Roll [rad]
    float theta; // Pitch [rad]
    float psi;   // Yaw [rad]

public:
    IMU(TwoWire &w) { wire = &w; }
    virtual ~IMU() = default;
    bool Init();
    void Update(float dt);
    void PrintRawData();
    void Calibration();
    void PrintCalibData();
    void PrintEulerAngles();
    float DegToRad(float deg);
    float RadToDeg(float rad);
    float GetPhi() { return phi; }
    float GetTheta() { return theta; }
    float GetPsi() { return psi; }
    float GetPhiDeg() { return RadToDeg(phi); }
};

#endif /* IMU_HPP */
