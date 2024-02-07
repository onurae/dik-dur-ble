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
    float aBias[3] = {0.056015f, 0.000459f, -0.015436f};
    float gBias[3] = {-1.407173f, 2.051758f, -1.705595f};
    bool ReadSensorData();
    float gRes;
    float aRes;
    const float tau = 0.98;
    void ComplementaryFilter(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    const float beta = 0.041;
    float q0 = 1.0;
    float q1 = 0.0;
    float q2 = 0.0;
    float q3 = 0.0;
    float InvSqrt(float x);
    void Madgwick6(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    void CalcEulerAngles();
    float phi = 0;   // Roll [rad]
    float theta = 0; // Pitch [rad]
    float psi = 0;   // Yaw [rad]

public:
    IMU(TwoWire &w) { wire = &w; }
    virtual ~IMU() = default;
    bool Init();
    enum class Fusion
    {
        COMPLEMENTARY,
        MADGWICK
    };
    void Update(float dt, Fusion fusion = Fusion::COMPLEMENTARY);
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
