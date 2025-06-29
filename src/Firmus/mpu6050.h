#ifndef FIRMUS_MPU6050_H
#define FIRMUS_MPU6050_H

/** © 2025 Keshav Haripersad
 *  Proxy class header for SJR mpu6050 class (sjr_mpu6050.h) - mpu6050.h
 *  | mpu6050.cpp for logic.
 *  | /Firmus for modules/assets.
 *
 *  Licensed under the Apache 2.0 license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/Firmus-SDK
 *  Firmus SDK version: 1.3.3
 */

#include "Assets/sjr_mpu6050.h"

class mpu6050
{
private:
    sjr_mpu_6050 _imu;

    int _sda, _scl;

    // Variables to store velocity (m/s) for each axis
    float vx, vy, vz;

    // Variables to store previous accelerations for integration
    float prev_ax, prev_ay, prev_az;

    // Time step for velocity integration
    float dt;

    float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

public:
    float yaw, pitch, roll;
    float yaw_rad, pitch_rad, roll_rad;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    float ax_mps2, ay_mps2, az_mps2;

    mpu6050(int sda, int scl, float timeStep = 0.01); // Add time step parameter

    int begin();
    void calibrate();
    mpu6050 &read();

    // Functions to get the velocities (m/s)
    float getVx();
    float getVy();
    float getVz();

    // Setter and getter functions for various parameters
    int updateGyroscopeOffsets(float *offsets);
    int updateAccelerometerCalibrations(float *calibrations);
    int updateGyroScale(float scale);
    int changeAddress(int address);
    int updateMahonyKp(float Kp_);
    int updateMahonyKi(float Ki_);
    int changeCalibrationFlag(int calibrateGyro_);
    int updateAccelSensitivity(float sensitivity);
    int changeGravity(float gravity_);
};

#endif