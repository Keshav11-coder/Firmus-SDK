#ifndef FIRMUS_PID_H
#define FIRMUS_PID_H

/** © 2025 Keshav Haripersad
 *  Base PID/PIDf header - pid.h
 *  | pid.cpp for base logic.
 *  | /Firmus for modules/assets.
 *
 *  Licensed under the Apache 2.0 license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/Firmus-SDK
 *  Firmus SDK version: 1.3.3
 */

 // Classic PID
class PID
{
public:
    PID();
    void gains(float kp, float ki, float kd);
    void reset();

    float compute(float dt);
    float compute(float error, float dt);
    float compute(float input, float target, float dt);

private:
    float _kp, _ki, _kd;
    float _integral;
    float _lastError;
    float _input, _target;
};

// Filtered / Smooth PID
class PID_filtered
{
private:
    float kp, ki, kd, N; // PID gains
    float Ti, Td;        // Time constants

    float prev_error = 0.0f;
    float integral = 0.0f;
    float derivative = 0.0f;
    float prev_derivative_input = 0.0f;

public:
    PID_filtered() : kp(0), ki(0), kd(0), N(10.0f), Ti(1.0f), Td(0.0f) {}

    void gains(float p, float i, float d, float n = 10.0f)
    {
        kp = p;
        ki = i;
        kd = d;
        N = n;
        Ti = (i != 0.0f) ? (p / i) : 1.0f;
        Td = (d != 0.0f) ? (d / p) : 0.0f;
    }

    void reset()
    {
        prev_error = 0.0f;
        integral = 0.0f;
        derivative = 0.0f;
        prev_derivative_input = 0.0f;
    }

    float compute(float error, float dt)
    {
        if (dt <= 0.0f)
            return 0.0f;

        // Trapezoidal integral (anti-windup)
        integral += 0.5f * (error + prev_error) * dt;

        // Derivative with low-pass filter
        float D_input = (error - prev_error) / dt;
        float alpha = Td / (Td + 1.0f / (N * dt));
        derivative = alpha * derivative + (1.0f - alpha) * D_input;

        prev_error = error;

        return kp * (error + integral / Ti + Td * derivative);
    }

    float compute(float input, float target, float dt)
    {
        return compute(target - input, dt);
    }
};

#endif