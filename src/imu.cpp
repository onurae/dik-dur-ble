/*******************************************************************************************
 *                                                                                         *
 *    dik-dur-ble, imu                                                                     *
 *                                                                                         *
 *    Copyright (c) 2024 Onur AKIN <https://github.com/onurae>                             *
 *    Licensed under the MIT License.                                                      *
 *                                                                                         *
 ******************************************************************************************/

#include "imu.hpp"

bool IMU::Write(const uint8_t reg, const uint8_t data)
{
    wire->beginTransmission(address);
    wire->write(reg);
    wire->write(data);
    return !((bool)wire->endTransmission());
}

uint8_t IMU::Read(const uint8_t reg)
{
    wire->beginTransmission(address);
    wire->write(reg);
    wire->endTransmission(true);
    wire->requestFrom(address, (uint8_t)1);
    return wire->read();
}

bool IMU::Init()
{
    pinMode(22, OUTPUT);
    digitalWrite(22, LOW);
    delay(500);
    uint8_t whoAmI = Read(0x75); // WhoAmI
    if (whoAmI != 0x68)
    {
        Serial.println(whoAmI);
        Serial.println("MPU6050 not found!");
        return false;
    }
    Serial.println("MPU6050 found.");
    Write(0x6B, 0x80); // Reset MPU6050
    delay(100);
    Write(0x6B, 0x01); // Clock source
    delay(200);
    Write(0x19, 0);         // Sample rate divider. SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
    Write(0x1A, 0x01);      // Gyro bandwitdh 188Hz, delay 1.9ms, fs 1kHz. Acc bandwitdh 184Hz, delay 2.0ms, fs 1kHz.
    Write(0x1B, 0x00);      // Gyro full scale select: +-250 deg/s
    Write(0x1C, 0x00);      // Acc full scale select: +-2 g
    gRes = 250.0 / 32768.0; // +-250 deg/s
    aRes = 2.0 / 32768.0;   // +-2 g
    delay(100);
    Serial.println("MPU6050 configured.");
    return true;
}

void IMU::Update(float dt, Fusion fusion)
{
    if (ReadSensorData() == true)
    {
        aCal[0] = aRaw[0] - aBias[0];
        aCal[1] = aRaw[1] - aBias[1];
        aCal[2] = aRaw[2] - aBias[2];
        gCal[0] = gRaw[0] - gBias[0];
        gCal[1] = gRaw[1] - gBias[1];
        gCal[2] = gRaw[2] - gBias[2];
    }

    if (fusion == Fusion::COMPLEMENTARY)
    {
        ComplementaryFilter(-gCal[0], gCal[1], -gCal[2], aCal[0], -aCal[1], aCal[2], dt);
    }
    else if (fusion == Fusion::MADGWICK)
    {
        Madgwick6(-gCal[0], gCal[1], -gCal[2], aCal[0], -aCal[1], aCal[2], dt);
        CalcEulerAngles();
    }
}

bool IMU::ReadSensorData()
{
    uint8_t dataReady = Read(0x3A);
    if (dataReady & 0x01)
    {
        int16_t count[7];
        wire->beginTransmission(address);
        wire->write(0x3B);
        wire->endTransmission(false);
        wire->requestFrom(address, (uint8_t)14);
        for (int i = 0; i < 7; i++)
        {
            count[i] = wire->read() << 8;
            count[i] |= wire->read();
        }
        aRaw[0] = (float)count[0] * aRes;
        aRaw[1] = (float)count[1] * aRes;
        aRaw[2] = (float)count[2] * aRes;
        tRaw = ((float)count[3]) / 340.0f + 36.53f;
        gRaw[0] = (float)count[4] * gRes;
        gRaw[1] = (float)count[5] * gRes;
        gRaw[2] = (float)count[6] * gRes;
        return true;
    }
    return false;
}

void IMU::ComplementaryFilter(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float accelPitch = atan2(-ax, az);
    float accelRoll = atan2(ay, az);
    theta = tau * (theta + DegToRad(gy) * dt) + (1 - tau) * accelPitch;
    phi = tau * (phi + DegToRad(gx) * dt) + (1 - tau) * accelRoll;
    psi = psi + DegToRad(gz) * dt;
}

float IMU::InvSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void IMU::Madgwick6(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    gx = DegToRad(gx);
    gy = DegToRad(gy);
    gz = DegToRad(gz);

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void IMU::CalcEulerAngles()
{
    phi = std::atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
    double sinTheta = 2.0f * (q0 * q2 - q1 * q3);
    if (std::abs(sinTheta) >= 1)
    {
        theta = std::copysign(M_PI / 2.0f, sinTheta);
    }
    else
    {
        theta = std::asin(sinTheta);
    }
    psi = std::atan2(q0 * q3 + q1 * q2, 0.5f - q2 * q2 - q3 * q3);
}

void IMU::Calibration()
{
    Serial.printf("%s", "\nSensor calibration\n");
    Serial.printf("%s", "The sensor should be on flat surface! This takes 10 seconds.\n");
    delay(5000);
    Serial.printf("%s", "Do not move the sensor!\n");
    float bias[6] = {};
    uint16_t cycle = 0;
    int16_t freq = 100;
    unsigned long prevTime;
    unsigned long wakeTime = millis();
    while (cycle < freq * 10)
    {
        prevTime = wakeTime;
        while (millis() - prevTime < 1000 / freq) {} 
        wakeTime = millis();
        float dt = (wakeTime - prevTime) * 0.001f;
        Update(dt);
        bias[0] += aRaw[0];
        bias[1] += aRaw[1];
        bias[2] += aRaw[2];
        bias[3] += gRaw[0];
        bias[4] += gRaw[1];
        bias[5] += gRaw[2];
        cycle++;
    }
    for (int i = 0; i < 6; i++)
    {
        bias[i] = bias[i] / cycle;
    }
    bias[2] -= 1.0f; // Gravity 1g

    aBias[0] = bias[0];
    aBias[1] = bias[1];
    aBias[2] = bias[2];
    gBias[0] = bias[3];
    gBias[1] = bias[4];
    gBias[2] = bias[5];
    Serial.printf("%s", "\nCalibration completed.\n");
    Serial.printf("%s", "Enter these values to the imu.hpp file and comment out the Calibration function.\n");
    Serial.printf("%s%f\n", "biasAx = ", aBias[0]);
    Serial.printf("%s%f\n", "biasAy = ", aBias[1]);
    Serial.printf("%s%f\n", "biasAz = ", aBias[2]);
    Serial.printf("%s%f\n", "biasGx = ", gBias[0]);
    Serial.printf("%s%f\n", "biasGy = ", gBias[1]);
    Serial.printf("%s%f\n", "biasGz = ", gBias[2]);
    while(true)
    {
        delay(10);
    }
}

void IMU::PrintRawData()
{
    Serial.printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", aRaw[0], aRaw[1], aRaw[2], tRaw, gRaw[0], gRaw[1], gRaw[2]);
}

void IMU::PrintCalibData()
{
    Serial.printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", aCal[0], aCal[1], aCal[2], tRaw, gCal[0], gCal[1], gCal[2]);
}

void IMU::PrintEulerAngles()
{
    printf("%s%.1f, %s%.1f, %s%.1f\n", "phi: ", RadToDeg(phi), "theta: ", RadToDeg(theta), "psi: ", RadToDeg(psi));
}

float IMU::DegToRad(float deg)
{
    return deg / 180.0f * M_PI;
}

float IMU::RadToDeg(float rad)
{
    return rad / M_PI * 180.0f;
}