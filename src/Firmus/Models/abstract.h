#pragma once

/** © 2025 Keshav Haripersad
 *  Models abstract header - abstract.h
 *  | Rigid.h for rigid body models (implementations).
 *
 *  Licensed under the Apache 2.0 license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/Firmus-SDK
 *  Firmus SDK version: 1.3.3
 */

#include "Arduino.h"

struct Vector3; // Forward declaration of Vector3

class IRigidModel
{
public:
    virtual ~IRigidModel() {}

    // ====== GETTERS ======
    virtual float getMomentArm(size_t i) = 0;
    virtual float getActuatorAngle(size_t i) = 0;
    virtual float getActuatorTilt(size_t i) = 0;

    virtual float getMass() = 0;
    virtual double getOperatingVoltage() = 0;
    virtual int getMotorKv(size_t i) = 0;
    virtual float getPropellerDiameter() = 0;
    virtual float getPropellerPitch() = 0;

    virtual size_t getActuatorCount() = 0;

    // Return raw pointer references since size is unknown here.
    virtual void getAllocationMatrix(void *out) = 0;
    virtual void getInverseAllocationMatrix(void *out) = 0;
    virtual void getInertiaMatrix(void *out) = 0;

    // Physical computation
    virtual Vector3 computeRequiredTorque(Vector3 alpha, Vector3 des_alpha) = 0;
    virtual Vector3 computeRequiredOrientation(Vector3 des_acc, float yaw_reference) = 0;
};