#pragma once

/** © 2025 Keshav Haripersad
 *  Controllers abstract header - abstract.h
 *  | Cascade.h for cascade control system (example implementations).
 *  | Utilizing controller interfaces IAttitude, IVelocity, and IPosition. See base (virtual) classes.
 *  | This is frame-level control; no mixing is done here.
 *
 *  Licensed under the Apache 2.0 license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/Firmus-SDK
 *  Firmus SDK version: 1.3.3
 */

#include <Arduino.h>

#include "../Models/Rigid.h" // Include the RigidModel class for model operations

// General Frame Vector (Vector3)
struct Vector3; // Forward declaration of Vector3 

// General PID data structure
struct PIDGains
{
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain
    float kf; // Derivative filter gain
    float il; // Integral limit
};

// [Output-> desired angular acceleration]
class IAttitude
{
public:
    struct IAttitudeFlags
    {
        // Attitude Control flags container

        virtual ~IAttitudeFlags() {}
    };

    // Set PID gains
    virtual IAttitude &configureOrientationGains(PIDGains gains_x, PIDGains gains_y, PIDGains gains_z) = 0;
    virtual IAttitude &configureRateGains(PIDGains gains_x, PIDGains gains_y, PIDGains gains_z) = 0;

    // Update actual attitude and rate
    virtual IAttitude &updateMeasuredOrientation(float x, float y, float z) = 0;
    virtual IAttitude &updateMeasuredRate(float x, float y, float z) = 0;

    virtual Vector3 getMeasuredOrientation()
    {
        return Vector3(); // Default implementation
    };

    virtual Vector3 getMeasuredRate()
    {
        return Vector3(); // Default implementation
    };

    // Target Configuration Functions
    virtual IAttitude &configureOrientationTarget(float x, float y, float z) = 0;
    virtual IAttitude &configureRateTarget(float x, float y, float z) = 0;

    // [Safety] Set the control & safety flags
    virtual IAttitude &setControlFlags(IAttitudeFlags *updFlags) = 0;

    // Main computing function
    virtual Vector3 compute(float dt)
    {
        return Vector3(); // Default implementation
    }

    // Reset controller (in case of failsafe)
    virtual void reset() = 0;
};

// [To Do] [Output-> desired orientation]
class IVelocity
{
public:
    struct IVelocityFlags
    {
        // Attitude Control flags container

        virtual ~IVelocityFlags() {}
    };

    // Set PID gains
    virtual IVelocity &configureVelocityGains(PIDGains gains_x, PIDGains gains_y, PIDGains gains_z) = 0;

    virtual IVelocity &configureAccelerationGains(PIDGains gains_x, PIDGains gains_y, PIDGains gains_z) = 0;

    // Update actual attitude and rate
    virtual IVelocity &updateMeasuredVelocity(float x, float y, float z) = 0;
    virtual IVelocity &updateMeasuredAcceleration(float x, float y, float z) = 0;

    // Target Configuration Functions
    virtual IVelocity &configureVelocityTarget(float x, float y, float z) = 0;
    virtual IVelocity &configureAccelerationTarget(float x, float y, float z) = 0;

    // [Safety] Set the control & safety flags
    virtual IVelocity &setControlFlags(IVelocityFlags *updFlags) = 0;

    // Main computing function
    // virtual IAttitudeAngularAcceleration compute(float dt) = 0;
    virtual Vector3 compute(float dt)
    {
        return Vector3(); // Default implementation
    }

    // Reset controller (in case of failsafe)
    virtual void reset() = 0;
};

// [To Do] [Output-> desired velocity]
class IPosition
{
public:
    struct IPositionFlags
    {
        // Attitude Control flags container

        virtual ~IPositionFlags() {}
    };

    // Set PID gains
    virtual IPosition &configurePositionGains(PIDGains gains_x, PIDGains gains_y, PIDGains gains_z) = 0;

    // Update actual attitude and rate
    virtual IPosition &updateDeltaPosition(float dx, float dy, float dz) = 0; // e.g. -10, 10

    // [Safety] Set the control & safety flags
    virtual IPosition &setControlFlags(IPositionFlags *updFlags) = 0;

    // Main computing function
    // virtual IAttitudeAngularAcceleration compute(float dt) = 0;
    virtual Vector3 compute(float dt)
    {
        return Vector3(); // Default implementation
    }

    // Reset controller (in case of failsafe)
    virtual void reset() = 0;
};

class IOrchestrator
{
public:
    virtual IOrchestrator &useAttitudeInterface(IAttitude *attitudeCtrl) = 0;
    virtual IOrchestrator &useVelocityInterface(IVelocity *velocityCtrl) = 0;
    virtual IOrchestrator &usePositionInterface(IPosition *positionCtrl) = 0;

    virtual IOrchestrator &useModel(IRigidModel *model) = 0;

    virtual Vector3 compute(float dt)
    {
        return Vector3();
    }

    // Reset controller (in case of failsafe)
    virtual void reset() = 0;

    virtual ~IOrchestrator() {}
};
