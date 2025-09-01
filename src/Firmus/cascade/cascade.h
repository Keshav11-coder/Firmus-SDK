#pragma once

/** © 2025 Keshav Haripersad
 *  Cascaded Body Control implementation - cascade.h
 *  | Firmus.h for full virtual frame table/overview..
 *  | This is frame-level control; no mixing or hardware is done here.
 *
 *  Licensed under the Apache 2.0 license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/Firmus-SDK
 *  Firmus SDK version: 1.3.4+
 */

#include <cmath>

#include "../tools/pidf.h"
#include "../models/Rigid.h"

/* Forward declarations from ~/Firmus.h */
struct Vector3;
struct Snapshot;

class IBodyCorrectionPolicy;
class IBodyControlLayer;
class IBodyController;

/* Cascade Control namespace (containment) */
namespace Cascade
{
    /**
     * @class BodyPIDPolicy
     * @brief Implements a 3-axis PID-based correction strategy for orientation/rate control.
     *
     * This class untilizes the IBodyCorrectionPolicy interface by applying independent
     * filtered PID controllers on the X, Y, and Z axes. It computes corrections based
     * on the difference in actual and target values. Z-Axis uses angular error.
     *
     * Design Notes:
     * - Each axis has its own PID controller with configurable gains (this policy needs to be used with different flavors, see examples).
     * - Feedforward gain (kf) defaults unless specified.
     * - The correction is time-delta aware (msut be provided, computed in the outer-most loop).
     * - Stateless across calls except for PID integrator/memory.
     *
     * Usage Constraints:
     * - Input vectors must represent orientation or angular quantities.
     * - Call `reset()` to clear PID history when switching modes or targets.
     * - PID gains must be tuned externally; no adaptation is performed.
     * - Constricted to angular computations.
     *
     * Dependencies:
     * - Requires `PID_filtered` class from ~/Firmus/tools/pidf.h (filtered PID controller).
     * - Expects `Vector3` as defined in ~/Firmus.h
     */
    class BodyPIDPolicy : public IBodyCorrectionPolicy
    {
    private:
        PID_filtered X, Y, Z;

        static float error(float x_hat, float x)
        {
            float err = x_hat - x;
            return err > M_PI ? err - 2 * M_PI : (err < -M_PI ? err + 2 * M_PI : err);
        }

    public:
        BodyPIDPolicy(
            float kpx = 0, float kix = 0, float kdx = 0, float kfx = 10.0f,
            float kpy = 0, float kiy = 0, float kdy = 0, float kfy = 10.0f,
            float kpz = 0, float kiz = 0, float kdz = 0, float kfz = 10.0f)
        {
            X.gains(kpx, kix, kdx, kfx);
            Y.gains(kpy, kiy, kdy, kfy);
            Z.gains(kpz, kiz, kdz, kfz);
        }

        Vector3 compute(const Vector3 &target, const Vector3 &measured, float dt) override
        {
            float err_x = (target.x - measured.x);
            float err_y = (target.y - measured.y);
            float err_z = error(target.z, measured.z);

            return Vector3(
                X.compute(err_x, dt),
                Y.compute(err_y, dt),
                Z.compute(err_z, dt));
        }

        void reset() override
        {
            X.reset();
            Y.reset();
            Z.reset();
        }

        /* Stubs */
        Snapshot compute(const Snapshot &target, const Snapshot &measured, float dt) override { return Snapshot(); };
    };

    /**
     * @class BodyOrientationLayer
     * @brief Control layer converting orientation error to angular velocity.
     *
     * Implements IBodyControlLayer. Uses a correction strategy to compute angular velocity
     * from the difference between current and target orientations. Serves as an intermediate
     * layer in a cascaded controller architecture.
     *
     * Design Notes:
     * - Strategy must implement IBodyCorrectionPolicy.
     * - Orientation vectors are assumed to represent roll, pitch, yaw (in radians).
     * - Outputs are angular velocity estimates, written to Snapshot::angular_velocity.
     * - Timestamp is always updated to current time via millis().
     *
     * Constraints:
     * - Returns unmodified state if no correction strategy is assigned.
     * - Correction policy is externally managed; ownership not handled internally.
     */
    class BodyOrientationLayer : public IBodyControlLayer
    {
    private:
        IBodyCorrectionPolicy *correctionPolicy = nullptr;

    public:
        // Use a correction strategy for control
        IBodyControlLayer &setCorrectionPolicy(IBodyCorrectionPolicy *strategy) override
        {
            correctionPolicy = strategy;
            return *this;
        }

        // Process the input snapshot and return a new snapshot
        Snapshot process(const Snapshot &state, const Snapshot &target, float dt) override
        {
            if (!correctionPolicy)
            {
                // If no correction strategy is set, return current state with updated timestamp
                Snapshot state_copy = state;     // Copy the state to avoid modifying the original
                state_copy.timestamp = millis(); // Update timestamp
                return state_copy;
            }

            auto ang_vel_hat = correctionPolicy->compute(target.orientation, state.orientation, dt);

            Snapshot updated = target;
            updated.angular_velocity = ang_vel_hat;
            updated.timestamp = millis();
            return updated;
        }

        // Reset the controller state
        void reset() override
        {
            if (correctionPolicy)
                correctionPolicy->reset();
        }
    };

    /**
     * @class BodyRateLayer
     * @brief Control layer converting angular velocity error to angular acceleration.
     *
     * Computes the angular acceleration required to correct angular velocity error
     * using the assigned IBodyCorrectionPolicy strategy. Acts as a deeper control
     * layer after BodyOrientationLayer in a cascade.
     *
     * Design Notes:
     * - Assumes input vectors represent angular velocity in rad/s.
     * - Result stored in Snapshot::angular_acceleration.
     * - Output timestamp always reflects current time.
     *
     * Constraints:
     * - If correction strategy is null, returns current state with timestamp update.
     * - Caller manages lifecycle of strategy pointer.
     */
    class BodyRateLayer : public IBodyControlLayer
    {
    private:
        IBodyCorrectionPolicy *correctionPolicy = nullptr;

    public:
        // Use a correction strategy for control
        IBodyControlLayer &setCorrectionPolicy(IBodyCorrectionPolicy *strategy) override
        {
            correctionPolicy = strategy;
            return *this;
        }

        // Process the input snapshot and return a new snapshot
        Snapshot process(const Snapshot &state, const Snapshot &target, float dt) override
        {
            if (!correctionPolicy)
            {
                // If no correction strategy is set, return current state with updated timestamp
                Snapshot state_copy = state;     // Copy the state to avoid modifying the original
                state_copy.timestamp = millis(); // Update timestamp
                return state_copy;
            }

            auto ang_acc_hat = correctionPolicy->compute(target.angular_velocity, state.angular_velocity, dt);

            Snapshot updated = target;
            updated.angular_acceleration = ang_acc_hat;
            updated.timestamp = millis();
            return updated;
        }

        // Reset the controller state
        void reset() override
        {
            if (correctionPolicy)
                correctionPolicy->reset();
        }
    };

    /*class BodyRateLayer : public IBodyControlLayer
    {
    private:
        IBodyCorrectionPolicy *correctionPolicy = nullptr;

    public:
        // Use a correction strategy for control
        IBodyControlLayer &setCorrectionPolicy(IBodyCorrectionPolicy *strategy) override
        {
            correctionPolicy = strategy;
            return *this;
        }

        // Process the input snapshot and return a new snapshot
        Snapshot process(const Snapshot &state, const Snapshot &target, float dt) override
        {
            if (!correctionPolicy)
            {
                // If no correction strategy is set, return current state with updated timestamp
                Snapshot state_copy = state;     // Copy the state to avoid modifying the original
                state_copy.timestamp = millis(); // Update timestamp
                return state_copy;
            }

            auto ang_acc_hat = correctionPolicy->compute(target.angular_velocity, state.angular_velocity, dt);

            Snapshot updated = target;
            updated.angular_acceleration = ang_acc_hat;
            updated.timestamp = millis();
            return updated;
        }

        // Reset the controller state
        void reset() override
        {
            if (correctionPolicy)
                correctionPolicy->reset();
        }
    };*/

    /**
     * @class BodyController
     * @brief Manages a cascade of control layers and applies a rigid-body model.
     *
     * Receives an initial state and a target frame, and iteratively applies all added
     * control layers to compute a final control Snapshot. Final torque is computed using
     * an assigned IRigidModel, based on desired angular acceleration.
     *
     * Design Notes:
     * - Layers are applied in the order they are added; last-added is deepest.
     * - ControlLayer pointers are not owned; no deletion is performed.
     * - Capacity is fixed; excess layers are ignored.
     * - Final output is a Snapshot with computed torque and latest propagated timestamp.
     *   It resembles the final target state the body should be in.
     *
     * Usage:
     * - Call `useModel()` to assign a rigid-body dynamics model.
     * - Add layers using `addControlLayer()` from outermost to innermost.
     * - Use `process()` every control cycle with current state and new target.
     *
     * Constraints:
     * - No dynamic resizing; capacity must be set appropriately at construction.
     * - Behavior undefined if model is null and torque computation is required.
     * - Removing a layer shifts remaining layers down in sequence.
     */
    class BodyController : public IBodyController
    {
    private:
        IBodyControlLayer **layers = nullptr;
        size_t count = 0;
        size_t capacity = 0;

        IRigidModel *model = nullptr;

    public:
        BodyController(size_t initial_capacity = 4)
            : capacity(initial_capacity)
        {
            layers = new IBodyControlLayer *[capacity];
        }

        ~BodyController()
        {
            delete[] layers;
        }

        // Use a model for control
        IBodyController &useModel(IRigidModel *model_) override
        {
            model = model_;
            return *this;
        }

        IBodyController &addControlLayer(IBodyControlLayer *layer) override
        {
            if (count >= capacity)
            {
                return *this;
            }
            layers[count++] = layer;
            return *this;
        }

        /* Developer note: This is a sensitive function. It was originally intended for testing
           use, but will likely not be available in public releases as it can destroy flow of
           data and cause corruptions */
        IBodyController &removeControlLayer(IBodyControlLayer *layer) override
        {
            for (size_t i = 0; i < count; ++i)
            {
                if (layers[i] == layer)
                {
                    for (size_t j = i; j < count - 1; ++j)
                        layers[j] = layers[j + 1];
                    --count;
                    layers[count] = nullptr;
                    return *this; // TODO add warnings
                }
            }
            return *this;
        }

        Snapshot process(const Snapshot &state, const Snapshot &target_initial, float dt) override
        {
            Snapshot target = target_initial;

            for (size_t i = 0; i < count; ++i)
            {
                // Target gets modified per layer and sent back for the next layer
                target = layers[i]->process(state, target, dt); // Returns "this is what you have to do to the state" for the next layer (e.g. new target)
            }

            // Assuming results are angular accelerations:
            Vector3 torque = model ? model->computeRequiredTorque(state.angular_velocity, target.angular_acceleration) : state.torque;

            target.torque = torque;      // Update the torque in the result snapshot
            target.timestamp = millis(); // Update the timestamp to the current time

            return target;
        }
    };
}