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

// General Frame data containers
struct Vector3;  // Forward declaration of Vector3
struct Snapshot; // Forward declaration of Snapshot

// Frame-level control interfaces (Outputs Snapshot)

// ICourseCorrectionStrategy can be used in all kinds of frame-level control systems, it's clear and can have different policies or implementations (PID, LQR, MPCPolicy, etc...)
class ICourseCorrectionStrategy
{
public:
    virtual ~ICourseCorrectionStrategy() = default; // Virtual destructor for cleanup

    virtual Vector3 compute(const Vector3 &target, const Vector3 &measured, float dt) = 0; // Compute the correction based on target and measured values
    virtual void reset() = 0;                                                              // Reset the controller state
};

class IControlLayer
{
public:
    virtual ~IControlLayer() = default; // Virtual destructor for cleanup

    virtual IControlLayer &setCorrectionStrategy(ICourseCorrectionStrategy *strategy) = 0; // Use a correction strategy for control
    virtual Snapshot process(const Snapshot &state, const Snapshot &target, float dt) = 0; // Process the input snapshot and return a new snapshot

    virtual void reset() = 0; // Reset the controller state
};

class IControlOrchestrator
{
public:
    virtual ~IControlOrchestrator() = default;

    virtual IControlOrchestrator &addControlLayer(IControlLayer *layer) = 0;
    virtual IControlOrchestrator &removeControlLayer(IControlLayer *layer) = 0;

    virtual IControlOrchestrator &useModel(IRigidModel *model) = 0; // Use a model for control

    virtual Snapshot process(const Snapshot &state, const Snapshot &target, float dt) = 0;
};

// General PID data structure
[[deprecated("These interfaces are legacy Cascade-centric abstractions. Use the new IControlLayer and IResolver framework for flexible, modular, and extensible control pipelines.")]] struct PIDGains
{
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain
    float kf; // Derivative filter gain
    float il; // Integral limit
};

// [Output-> desired angular acceleration]
[[deprecated("These interfaces are legacy Cascade-centric abstractions. Use the new IControlLayer and IResolver framework for flexible, modular, and extensible control pipelines.")]] class IAttitude
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
[[deprecated("These interfaces are legacy Cascade-centric abstractions. Use the new IControlLayer and IResolver framework for flexible, modular, and extensible control pipelines.")]] class IVelocity
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
[[deprecated("These interfaces are legacy Cascade-centric abstractions. Use the new IControlLayer and IResolver framework for flexible, modular, and extensible control pipelines.")]] class IPosition
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

[[deprecated("These interfaces are legacy Cascade-centric abstractions. Use the new IControlLayer and IResolver framework for flexible, modular, and extensible control pipelines.")]] class IOrchestrator
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
