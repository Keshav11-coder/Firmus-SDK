#ifndef FIRMUS_H
#define FIRMUS_H

/** © 2025 Keshav Haripersad
 *  Base SDK header - Firmus.h
 *  | ~/Firmus for controllers, mixers, tools, models
 *
 *  Licensed under the Apache 2.0 license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/Firmus-SDK
 *  Firmus SDK version: 1.3.4
 */

#include <Arduino.h>

#include "Firmus/tools/matrix2.h"
#include "Firmus/models/abstract.h"

/**
 * @struct Vector3
 * @brief 3D vector representing spatial or angular quantities.
 *
 * Used for positions, velocities, accelerations, orientations, and torques
 * in 3D space. Elements are stored in Cartesian form (x, y, z).
 *
 * Intended as a universal container within frame-level data structures.
 */
struct Vector3
{
    float x; ///< X component
    float y; ///< Y component
    float z; ///< Z component

    /**
     * @brief Construct a Vector3 with optional component values.
     *
     * @param x_ Initial x value. Defaults to 0.
     * @param y_ Initial y value. Defaults to 0.
     * @param z_ Initial z value. Defaults to 0.
     */
    Vector3(float x_ = 0, float y_ = 0, float z_ = 0) : x(x_), y(y_), z(z_) {}
};

/**
 * @struct Snapshot
 * @brief Encapsulates the full dynamic state of a frame at a point in time.
 *
 * Stores linear and angular quantities, timing, and final control output (torque).
 * Used as input/output to body controllers and control layers.
 *
 * Timestamp and loop_freq enable real-time tracking of control loop performance.
 */
struct Snapshot
{
    Vector3 position;             ///< Position (m) in 3D space.
    Vector3 velocity;             ///< Linear velocity (m/s).
    Vector3 acceleration;         ///< Linear acceleration (m/s^2).
    Vector3 orientation;          ///< Orientation (rad), e.g. Euler angles.
    Vector3 angular_velocity;     ///< Angular velocity (rad/s).
    Vector3 angular_acceleration; ///< Angular acceleration (rad/s^2).

    uint64_t timestamp; ///< Timestamp (ms since epoch or system start).
    uint64_t loop_freq; ///< Control loop frequency (Hz).

    Vector3 torque; ///< Final torque output (Nm) computed by controller.
};

/**
 * @interface IBodyCorrectionPolicy
 * @brief Abstract interface for computing control corrections between target and measured vectors.
 *
 * Serves as the correction strategy in frame-level controllers. Examples include PID, LQR, or learned policies.
 */
class IBodyCorrectionPolicy
{
public:
    virtual ~IBodyCorrectionPolicy() = default;

    /**
     * @brief Compute a correction vector from the error between target and measured values.
     *
     * @param target Desired vector (e.g., desired orientation or rate).
     * @param measured Actual measured vector.
     * @param dt Time delta (s) since last computation.
     * @return Vector3 Corrective output (e.g., angular velocity or acceleration).
     */
    virtual Vector3 compute(const Vector3 &target, const Vector3 &measured, float dt) = 0;

    /**
     * @brief Reset internal state (e.g., integrators) of the correction policy.
     */
    virtual void reset() = 0;
};

/**
 * @interface IBodyControlLayer
 * @brief Abstract interface for transforming high-level targets into lower-level control actions.
 *
 * Represents a layer in a control cascade. Layers are expected to modify the target Snapshot
 * (e.g., computing angular velocity or acceleration) based on state and a correction policy.
 */
class IBodyControlLayer
{
public:
    virtual ~IBodyControlLayer() = default;

    /**
     * @brief Assign a correction strategy to this control layer.
     *
     * @param strategy Pointer to an IBodyCorrectionPolicy instance.
     * @return Reference to this layer for chaining.
     */
    virtual IBodyControlLayer &setCorrectionPolicy(IBodyCorrectionPolicy *strategy) = 0;

    /**
     * @brief Process the current state and target, applying corrections.
     *
     * @param state Measured or estimated current frame state.
     * @param target Desired frame state.
     * @param dt Time delta (s) since last process call.
     * @return Snapshot Modified target with corrective fields populated.
     */
    virtual Snapshot process(const Snapshot &state, const Snapshot &target, float dt) = 0;

    /**
     * @brief Reset any internal state (e.g., PID memory).
     */
    virtual void reset() = 0;
};

/**
 * @interface IBodyController
 * @brief Abstract interface for composite controllers using multiple control layers.
 *
 * Manages control layers and a rigid-body model to compute final actuation commands
 * based on input states and desired targets.
 */
class IBodyController
{
public:
    virtual ~IBodyController() = default;

    /**
     * @brief Add a control layer to the cascade (outermost to innermost).
     *
     * @param layer Pointer to the control layer to add.
     * @return Reference to this controller for chaining.
     */
    virtual IBodyController &addControlLayer(IBodyControlLayer *layer) = 0;

    /**
     * @brief Remove a control layer from the cascade.
     *
     * @param layer Pointer to the control layer to remove.
     * @return Reference to this controller for chaining.
     */
    virtual IBodyController &removeControlLayer(IBodyControlLayer *layer) = 0;

    /**
     * @brief Assign a rigid-body model to compute torque outputs.
     *
     * @param model Pointer to the model implementing IRigidModel.
     * @return Reference to this controller for chaining.
     */
    virtual IBodyController &useModel(IRigidModel *model) = 0;

    /**
     * @brief Process the control stack from high-level target to low-level torque.
     *
     * @param state Current frame state (measured or estimated).
     * @param target Desired frame state.
     * @param dt Time delta (s) since last control cycle.
     * @return Snapshot Final target with computed torque and updated fields.
     */
    virtual Snapshot process(const Snapshot &state, const Snapshot &target, float dt) = 0;
};

/*
 * Actuator Interface class (SHOULD BE MOVED TO SEPARATE MODULE)
 * | Interface for thruster controllers (e.g. ESCs, Boosters)
 * | Provides a failsafe system for emergency situations
 * | Provides, at its minimum, basic control functions
 * | Class is abstract and must be inherited and implemented due to expected implmentations (e.g. failsafe / write)
 * | Functionality is not constrained. It is up to the user to implement the necessary functions
 * | An inherited class can be found in /Firmus: ESC.h
 */
class ActuatorInterface
{
public:
    struct failsafe_system // Failsafe system struct as example
    {
    private:
        bool failsafe = false;

    public:
        virtual bool engage() // Enable failsafe
        {
            failsafe = true;
            return true;
        }

        virtual bool disengage() // Disable failsafe
        {
            failsafe = false;
            return true;
        }

        virtual bool status() // Get failsafe status
        {
            return failsafe;
        }

        virtual ~failsafe_system() = default; // Destructor if needed
    };

    virtual int arm() = 0;              // Thruster arm function (e.g. ESC arming, or Booster initialization)
    virtual int stop() = 0;             // Thruster stop function (e.g. ESC stop, or Booster shutdown)
    virtual int restart() = 0;          // Thruster restart function (e.g. ESC restart, or Booster re-initialization)
    virtual int max() = 0;              // Thruster maximum speed function
    virtual int min() = 0;              // Thruster minimum speed function
    virtual int write(float speed) = 0; // Thruster write function (e.g. ESC write, or Booster write)

    failsafe_system failsafe;
};

#endif