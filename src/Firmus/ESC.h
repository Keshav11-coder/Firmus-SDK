#ifndef FIRMUS_ESC_H
#define FIRMUS_ESC_H

/** © 2025 Keshav Haripersad
 *  Basic ESC class implementation, inherits from ActuatorInterface - ESC.cpp
 *  | ESC.h for overview.
 *  | /Firmus for modules/assets.
 *  | This class was written for typical ESCs, and is not intended for other types of thrusters.
 *
 *  Licensed under the Apache 2.0 license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/Firmus-SDK
 *  Firmus SDK version: 1.3.3
 */

#include "../Firmus.h" // Include the base API header

// Required
#include <ESP32Servo.h>

class ESC : public Firmus::ActuatorInterface
{
private:
    Servo esc;
    int esc_pin;
    int esc_min;
    int esc_max;
    int motor_kv;
    bool armed = false;

public:
    ESC() {}
    ESC(int pin, int min, int max, int kv) : esc_pin(pin), esc_min(min), esc_max(max), motor_kv(kv)
    {
        esc.attach(pin);
    }

    struct failsafe_system : public Firmus::ActuatorInterface::failsafe_system
    {
    private:
        bool failsafe = false;

    public:
        bool engage() override;
        bool disengage() override;
        bool status() override;
    } failsafe;

    int arm() override;
    int stop() override;
    int restart() override;
    int max() override;
    int min() override;
    int write(float speed) override; // Ensure this is float
};

#endif
