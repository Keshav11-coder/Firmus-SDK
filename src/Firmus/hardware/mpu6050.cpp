/** © 2025 Keshav Haripersad
 *  Proxy class implementation for SJR mpu6050 class (sjr_mpu6050.h) - mpu6050.cpp
 *  | mpu6050.h for overview.
 *
 *  Licensed under the Apache 2.0 license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/Firmus-SDK
 *  Firmus SDK version: 1.3.4
 */

#include "mpu6050.h"

mpu6050::mpu6050(int sda, int scl, float timeStep)
{
    _sda = sda;
    _scl = scl;
    dt = timeStep; // Initialize the time step (defaults to 0.01 seconds)

    // Initialize velocities and previous accelerations
    vx = vy = vz = 0;
    prev_ax = prev_ay = prev_az = 0;
}

int mpu6050::begin()
{
    return _imu.begin(_sda, _scl);
}

void mpu6050::calibrate()
{
    _imu.calibrate();

    gyro_bias_x = 0;
    gyro_bias_y = 0;
    gyro_bias_z = 0;

    for (int i = 0; i < 1000; ++i)
    {
        _imu.read();
        gyro_bias_x += _imu.gx;
        gyro_bias_y += _imu.gy;
        gyro_bias_z += _imu.gz;
        delay(1);
    }

    gyro_bias_x /= 1000.0f;
    gyro_bias_y /= 1000.0f;
    gyro_bias_z /= 1000.0f;
}

mpu6050 &mpu6050::read()
{
    _imu.read(); // Read raw data from the MPU6050

    float gyro_scale = (PI / 180.0f) / 131.0f; // Convert raw gyro data to radians/s

    yaw = _imu.yaw;
    pitch = _imu.pitch;
    roll = _imu.roll;

    yaw_rad = _imu.yaw * PI / 180;
    pitch_rad = _imu.pitch * PI / 180;
    roll_rad = _imu.roll * PI / 180;

    ax = (_imu.ax / 16384.0) * 9.81; // Convert raw accelerometer data to m/s²
    ay = (_imu.ay / 16384.0) * 9.81;
    az = (_imu.az / 16384.0) * 9.81;

    gx = (_imu.gx - gyro_bias_x) * gyro_scale;
    gy = (_imu.gy - gyro_bias_y) * gyro_scale;
    gz = (_imu.gz - gyro_bias_z) * gyro_scale;

    // Convert raw accelerometer data to m/s²
    ax_mps2 = _imu.ax_mps2;
    ay_mps2 = _imu.ay_mps2;
    az_mps2 = _imu.az_mps2;

    return *this;
}

// Getter functions to retrieve current velocities (m/s)
float mpu6050::getVx()
{
    return vx;
}

float mpu6050::getVy()
{
    return vy;
}

float mpu6050::getVz()
{
    return vz;
}

// Setter and getter functions for various parameters
int mpu6050::updateGyroscopeOffsets(float *offsets)
{
    return _imu.updateGyroscopeOffsets(offsets);
}

int mpu6050::updateAccelerometerCalibrations(float *calibrations)
{
    return _imu.updateAccelerometerCalibrations(calibrations);
}

int mpu6050::updateGyroScale(float scale)
{
    return _imu.updateGyroScale(scale);
}

int mpu6050::changeAddress(int address)
{
    return _imu.changeAddress(address);
}

int mpu6050::updateMahonyKp(float Kp_)
{
    return _imu.updateMahonyKp(Kp_);
}

int mpu6050::updateMahonyKi(float Ki_)
{
    return _imu.updateMahonyKi(Ki_);
}

int mpu6050::changeCalibrationFlag(int calibrateGyro_)
{
    return _imu.changeCalibrationFlag(calibrateGyro_);
}

int mpu6050::updateAccelSensitivity(float sensitivity)
{
    return _imu.updateAccelSensitivity(sensitivity);
}

int mpu6050::changeGravity(float gravity_)
{
    return _imu.changeGravity(gravity_);
}