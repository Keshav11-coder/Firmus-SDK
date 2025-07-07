#ifndef remington_mpu_h
#define remington_mpu_h

/** © 2020 S.J. Remington
 *  Mahony AHRS algorithm for MPU6050
 *
 *  Release notes:
 *  | 3/2020 - Initial
 *  | 7/2020 - Added provision to recalibrate gyro upon startup. (variable calibrateGyro)
 */

#include <Arduino.h>
#include "Wire.h"

class sjr_mpu_6050
{
private:
    /*
     * MPU6050 variables
     * | MPU address may be changed if you know what you are doing
     * | Flag calibrateGyro to 0 to bypass gyro calibration
     */

    int mpuAddress = 0x68;
    int calibrateGyro = 1;

    /*
     * Offsets and scale factors for MPU-6050
     * | These are the previously determined offsets and scale factors for accelerometer and gyro (for
     * | a particular example of an MPU-6050). They are not correct for other examples.
     * |
     * | These pre-defined values will work, but not as accurately for an uncalibrated accelerometer and gyroscope.
     * | If you wish to bypass offsetting or calibration, use this these instead:
     * |
     * | float accelCalibrations[6] = {0.0, 0.0, 0.0, 1.000, 1.000, 1.000};
     * |
     * [NOTE] The AHRS will NOT work well or at all if these values are not correct
     * [TIP] The defaults for gyro and accel sensitivity are 250 dps and +/- 2 g
     */

    float accelCalibrations[6] = {265.0, -80.0, -700.0, 0.994, 1.000, 1.014}; // 0..2 offset xyz, 3..5 scale xyz

    float gyroOffsets[3] = {-499.5, -17.7, -82.0};       // Raw offsets determined for gyro at rest
    float gyroScale = ((250. / 32768.0) * (PI / 180.0)); // 250 LSB per d/s -> rad/s

    float accelSensitivity = 16384.0; // LSB/g
    float gravity = 9.81;             // m/s^2

    /*
     * Mahony filter variables
     * | These variables are globally declared.
     * |
     * [CAUTION] These are not to be changed unless you know what you are doing.
     * |
     * [NOTE] With an MPU9250, the Mahony filter will oscillate at Kp=40. Ki does not seem to help and is not required.
     */

    float quaternions[4] = {1.0, 0.0, 0.0, 0.0};

    float Kp = 30.0; // Proportional feedback gain
    float Ki = 0.0;  // Integral feedback gain

public:
    /*
     * Class data output variables
     * | These are the variables that are updated with each read() call.
     * | They are public and can be accessed directly.
     * [NOTE] The Euler angles are in degrees, not radians
     * [NOTE] The yaw angle is relative to magnetic North
     * [NOTE] Accelerometer values (xx_mps2) are in m/s^2
     */

    float yaw, pitch, roll;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    float ax_mps2, ay_mps2, az_mps2;

    sjr_mpu_6050() {}
    sjr_mpu_6050(int address_) : mpuAddress(address_) {}

    int updateGyroscopeOffsets(float *offsets)
    {
        for (int i = 0; i < 3; i++)
            gyroOffsets[i] = offsets[i];
        return 0;
    }

    int updateAccelerometerCalibrations(float *calibrations)
    {
        for (int i = 0; i < 6; i++)
            accelCalibrations[i] = calibrations[i];
        return 0;
    }

    int updateGyroScale(float scale)
    {
        gyroScale = scale;
        return 0;
    }

    int changeAddress(int address)
    {
        mpuAddress = address;
        return 0;
    }

    int updateMahonyKp(float Kp_)
    {
        Kp = Kp_;
        return 0;
    }

    int updateMahonyKi(float Ki_)
    {
        Ki = Ki_;
        return 0;
    }

    int changeCalibrationFlag(int calibrateGyro_)
    {
        calibrateGyro = calibrateGyro_;
        return 0;
    }

    int updateAccelSensitivity(float sensitivity)
    {
        accelSensitivity = sensitivity;
        return 0;
    }

    int changeGravity(float gravity_)
    {
        gravity = gravity_;
        return 0;
    }

    int begin(int sda, int scl)
    {
        Wire.begin(sda, scl);
        Wire.beginTransmission(mpuAddress);
        Wire.write(0x6B); // PWR_MGMT_1 register
        Wire.write(0);    // Set to zero (wakes up the MPU6050)
        Wire.endTransmission(true);
        return 0;
    }

    int calibrate(int samples_ = 500)
    {
        long gyroSum[3] = {0};
        for (int i = 0; i < samples_; i++)
        {
            read();           // Read current sensor values
            gyroSum[0] += gx; // Sum up gyro readings
            gyroSum[1] += gy;
            gyroSum[2] += gz;
            delay(5); // Short delay to allow for multiple samples
        }

        // Calculate and set gyro offsets
        for (char k = 0; k < 3; k++)
            gyroOffsets[k] = ((float)gyroSum[k]) / samples_;

        calibrateGyro = 0; // Disable calibration after getting offsets
        return 0;
    }

    void read()
    {
        static unsigned int i = 0;              // loop counter
        static float deltat = 0;                // loop time in seconds
        static unsigned long now = 0, last = 0; // micros() timers
        static long gyroSum[3] = {0};

        // Raw data placeholder
        int16_t Tmp; // temperature

        // Scaled data placeholders
        float AccelXyz[3];
        float GyroXyz[3];

        // Begin reading with Wire
        Wire.beginTransmission(mpuAddress);
        Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(mpuAddress, 14); // Request a total of 14 registers
        int t = Wire.read() << 8;
        ax = t | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        t = Wire.read() << 8;
        ay = t | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        t = Wire.read() << 8;
        az = t | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        t = Wire.read() << 8;
        Tmp = t | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
        t = Wire.read() << 8;
        gx = t | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        t = Wire.read() << 8;
        gy = t | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        t = Wire.read() << 8;
        gz = t | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

        // Convert raw accelerometer data to m/s^2
        ax_mps2 = (float)ax / accelSensitivity * gravity;
        ay_mps2 = (float)ay / accelSensitivity * gravity;
        az_mps2 = (float)az / accelSensitivity * gravity;

        // Calibrate gyro upon startup. Sensor must be held still for a few seconds.
        i++;
        if (calibrateGyro)
        {
            gyroSum[0] += gx;
            gyroSum[1] += gy;
            gyroSum[2] += gz;

            if (i == 500)
            {
                calibrateGyro = 0; // Turn off calibration

                // Update offsets
                for (char k = 0; k < 3; k++)
                    gyroOffsets[k] = ((float)gyroSum[k]) / 500.0;
            }
        }
        else // Regular operations
        {
            AccelXyz[0] = (float)ax;
            AccelXyz[1] = (float)ay;
            AccelXyz[2] = (float)az;

            // Apply offsets and scale factors from Magneto
            for (i = 0; i < 3; i++)
                AccelXyz[i] = (AccelXyz[i] - accelCalibrations[i]) * accelCalibrations[i + 3];

            GyroXyz[0] = ((float)gx - gyroOffsets[0]) * gyroScale; // 250 LSB(d/s) default to radians/s
            GyroXyz[1] = ((float)gy - gyroOffsets[1]) * gyroScale;
            GyroXyz[2] = ((float)gz - gyroOffsets[2]) * gyroScale;

            now = micros();
            deltat = (now - last) * 1.0e-6; // Seconds since last update
            last = now;

            // Perform a Mahony filter update
            MahonyFilterUpdate(AccelXyz[0], AccelXyz[1], AccelXyz[2], GyroXyz[0], GyroXyz[1], GyroXyz[2], deltat);

            /*
             * Compute Tait-Bryan angles
             * | Positive z-axis is down toward Earth
             * | Yaw: angle between Sensor x-axis and Earth magnetic North
             * | Pitch: angle between sensor x-axis and Earth ground plane
             * | Roll: angle between sensor y-axis and Earth ground plane
             * | Rotations must be applied in the order: yaw, pitch, roll
             * | Reference: http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
             */

            pitch = atan2((quaternions[0] * quaternions[1] + quaternions[2] * quaternions[3]), 0.5 - (quaternions[1] * quaternions[1] + quaternions[2] * quaternions[2]));
            roll = asin(2.0 * (quaternions[0] * quaternions[2] - quaternions[1] * quaternions[3]));

            // Conventional yaw increases clockwise from North. The MPU is not aware of magnetic North, so we use quaternion data to calculate yaw.
            yaw = -atan2((quaternions[1] * quaternions[2] + quaternions[0] * quaternions[3]), 0.5 - (quaternions[2] * quaternions[2] + quaternions[3] * quaternions[3]));

            // Convert Euler angles to degrees
            yaw *= 180.0 / PI;
            if (yaw < 0)
                yaw += 360.0; // Correct for compass circle.

            // Correct for local magnetic declination
            pitch *= 180.0 / PI;
            roll *= 180.0 / PI;
        }
    }

    int MahonyFilterUpdate(float ax, float ay, float az, float gx, float gy, float gz, float deltat)
    {
        float reciprocalNorm;
        float vx, vy, vz;
        float ex, ey, ez; // Error Terms
        float qa, qb, qc;
        static float ix = 0.0, iy = 0.0, iz = 0.0; // integral feedback terms
        float tmp;

        tmp = ax * ax + ay * ay + az * az;

        // Ignore accelerometer if false. (Tested OK by S.J. R)
        if (tmp > 0.0)
        {
            // Normalise accelerometer (assumed to measure the direction of gravity in body frame).
            reciprocalNorm = 1.0 / sqrt(tmp);
            ax *= reciprocalNorm;
            ay *= reciprocalNorm;
            az *= reciprocalNorm;

            // Estimated direction of gravity in the body frame (factor of two divided out).
            vx = quaternions[1] * quaternions[3] - quaternions[0] * quaternions[2];
            vy = quaternions[0] * quaternions[1] + quaternions[2] * quaternions[3];
            vz = quaternions[0] * quaternions[0] - 0.5f + quaternions[3] * quaternions[3];

            // Error is cross product between estimated and measured direction of gravity in body frame (half the actual magnitude).
            ex = (ay * vz - az * vy);
            ey = (az * vx - ax * vz);
            ez = (ax * vy - ay * vx);

            // Compute and apply to the gyro term with the integral feedback (if enabled).
            if (Ki > 0.0f)
            {
                ix += Ki * ex * deltat; // Integral error scaled by Ki
                iy += Ki * ey * deltat;
                iz += Ki * ez * deltat;
                gx += ix; // Apply integral feedback
                gy += iy;
                gz += iz;
            }

            // Apply proportional feedback to gyro term
            gx += Kp * ex;
            gy += Kp * ey;
            gz += Kp * ez;
        }

        // Integrate rate of change of quaternion given by gyro term.
        deltat = 0.5 * deltat;
        gx *= deltat; // Pre-multiply common factors
        gy *= deltat;
        gz *= deltat;
        qa = quaternions[0];
        qb = quaternions[1];
        qc = quaternions[2];

        // Add qmult*delta_t to current orientation.
        quaternions[0] += (-qb * gx - qc * gy - quaternions[3] * gz);
        quaternions[1] += (qa * gx + qc * gz - quaternions[3] * gy);
        quaternions[2] += (qa * gy - qb * gz + quaternions[3] * gx);
        quaternions[3] += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion.
        reciprocalNorm = 1.0 / sqrt(quaternions[0] * quaternions[0] + quaternions[1] * quaternions[1] + quaternions[2] * quaternions[2] + quaternions[3] * quaternions[3]);
        quaternions[0] = quaternions[0] * reciprocalNorm;
        quaternions[1] = quaternions[1] * reciprocalNorm;
        quaternions[2] = quaternions[2] * reciprocalNorm;
        quaternions[3] = quaternions[3] * reciprocalNorm;

        return 0;
    }
};

#endif