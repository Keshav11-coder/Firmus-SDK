/** © 2025 Keshav Haripersad
 *  Base PID/PIDf implementation - pid.cpp
 *  | pid.h for partial logic/overview
 *  | /Firmus for modules/assets.
 *
 *  Licensed under the Apache 2.0 license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/Firmus-SDK
 *  Firmus SDK version: 1.3.3
 */

#include "pid.h"

PID::PID() : _kp(0), _ki(0), _kd(0), _integral(0), _lastError(0), _input(0), _target(0) {}

void PID::gains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::reset()
{
    _integral = 0;
    _lastError = 0;
}

float PID::compute(float dt)
{
    float error = _target - _input;

    float derivative = (error - _lastError) / dt;
    _integral += error * dt;
    _lastError = error;

    return _kp * error + _ki * _integral + _kd * derivative;
}

float PID::compute(float error, float dt)
{
    float derivative = (error - _lastError) / dt;
    _integral += error * dt;
    _lastError = error;

    return _kp * error + _ki * _integral + _kd * derivative;
}

float PID::compute(float input, float target, float dt)
{
    float error = target - input;

    float derivative = (error - _lastError) / dt;
    _integral += error * dt;
    _lastError = error;

    return _kp * error + _ki * _integral + _kd * derivative;
}
