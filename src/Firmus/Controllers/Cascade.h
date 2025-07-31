#pragma once

/** © 2025 Keshav Haripersad
 *  Controllers implementation header for Cascade Control example - Cascade.h
 *  | abstract.h for virtual table/overview.
 *  | Utilizing controller interfaces IAttitude, IVelocity, and IPosition. See base (virtual) classes.
 *  | This is frame-level control; no mixing is done here.
 *
 *  Licensed under the Apache 2.0 license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/Firmus-SDK
 *  Firmus SDK version: 1.3.3
 */

#include "abstract.h" // Contains base classes (IAttitude, IVelocity, IPosition)

// [Required] PID library
#include "../pid.h"

// [Required] Arduino.h for math
#include <Arduino.h>

// [Required] Rigid Model
#include "../Models/Rigid.h"

struct Vector3; // Forward declaration of Vector3

namespace Cascade
{
    class FrameCorrectionStrategy : public ICourseCorrectionStrategy
    {
    private:
        PID_filtered X, Y, Z;

        static float error(float x_hat, float x)
        {
            float err = x_hat - x;
            return err > M_PI ? err - 2 * M_PI : (err < -M_PI ? err + 2 * M_PI : err);
        }

    public:
        FrameCorrectionStrategy(
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
    };

    // Attitude Controller Implementation from IControlLayer
    class FrameOrientationControlLayer : public IControlLayer
    {
    private:
        ICourseCorrectionStrategy *correctionStrategy = nullptr;

    public:
        // Use a correction strategy for control
        IControlLayer &setCorrectionStrategy(ICourseCorrectionStrategy *strategy) override
        {
            correctionStrategy = strategy;
            return *this;
        }

        // Process the input snapshot and return a new snapshot
        Snapshot process(const Snapshot &state, const Snapshot &target, float dt) override
        {
            if (!correctionStrategy)
            {
                // If no correction strategy is set, return current state with updated timestamp
                Snapshot state_copy = state;     // Copy the state to avoid modifying the original
                state_copy.timestamp = millis(); // Update timestamp
                return state_copy;
            }

            auto ang_vel_hat = correctionStrategy->compute(target.orientation, state.orientation, dt);

            Snapshot updated = target;
            updated.angular_velocity = ang_vel_hat;
            updated.timestamp = millis();
            return updated;
        }

        // Reset the controller state
        void reset() override
        {
            if (correctionStrategy)
                correctionStrategy->reset();
        }
    };

    class FrameRateControlLayer : public IControlLayer
    {
    private:
        ICourseCorrectionStrategy *correctionStrategy = nullptr;

    public:
        // Use a correction strategy for control
        IControlLayer &setCorrectionStrategy(ICourseCorrectionStrategy *strategy) override
        {
            correctionStrategy = strategy;
            return *this;
        }

        // Process the input snapshot and return a new snapshot
        Snapshot process(const Snapshot &state, const Snapshot &target, float dt) override
        {
            if (!correctionStrategy)
            {
                // If no correction strategy is set, return current state with updated timestamp
                Snapshot state_copy = state;     // Copy the state to avoid modifying the original
                state_copy.timestamp = millis(); // Update timestamp
                return state_copy;
            }

            auto ang_acc_hat = correctionStrategy->compute(target.angular_velocity, state.angular_velocity, dt);

            Snapshot updated = target;
            updated.angular_acceleration = ang_acc_hat;
            updated.timestamp = millis();
            return updated;
        }

        // Reset the controller state
        void reset() override
        {
            if (correctionStrategy)
                correctionStrategy->reset();
        }
    };

    class FrameControlOrchestrator : public IControlOrchestrator
    {
    private:
        IControlLayer **layers = nullptr;
        size_t count = 0;
        size_t capacity = 0;

        IRigidModel *model = nullptr;

    public:
        FrameControlOrchestrator(size_t initial_capacity = 4)
            : capacity(initial_capacity)
        {
            layers = new IControlLayer *[capacity];
        }

        ~FrameControlOrchestrator()
        {
            delete[] layers;
        }

        // Use a model for control
        IControlOrchestrator &useModel(IRigidModel *model_) override
        {
            model = model_;
            return *this;
        }

        // You add a control layer from highest layer to lowest, e.g. position -> velocity -> acceleration -> orientation -> rate. The order determines the flow of data IN THIS IMPLEMENTATION.
        IControlOrchestrator &addControlLayer(IControlLayer *layer) override
        {
            if (count >= capacity)
            {
                return *this; // Prevent adding more layers than capacity
            }
            layers[count++] = layer;
            return *this;
        }

        IControlOrchestrator &removeControlLayer(IControlLayer *layer) override
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

    class Attitude : public IAttitude
    {
    private:
        // PID Controllers
        PID_filtered OrientationXPID, OrientationYPID, OrientationZPID;
        PID_filtered RateXPID, RateYPID, RateZPID;

        // Helper functions to detect abnormalities in attitude and rate
        bool abnormalAttitudeCheck(float x, float y, float z)
        {
            if (isnan(x) || isnan(y) || isnan(z))
                return false; // invalid data

            if (flags.useExtremeTiltLimit)
            {
                if (abs(x) > flags.extremeTiltLimitAbs || abs(y) > flags.extremeTiltLimitAbs)
                    return false;
            }

            if (flags.useTiltLimit)
            {
                if (abs(x) > flags.tiltLimitAbs || abs(y) > flags.tiltLimitAbs)
                    return false;
            }

            return true; // all checks passed
        }

        bool abnormalRateCheck(float x, float y, float z)
        {
            if (isnan(x) || isnan(y) || isnan(z))
                return false;

            if (flags.useRateLimit)
            {
                if (abs(x) > flags.rateLimitAbs || abs(y) > flags.rateLimitAbs || abs(z) > flags.rateLimitAbs)
                    return false;
            }

            return true;
        }

        float angularError(float current, float target)
        {
            float error = target - current;

            if (error > PI)
                error -= PI * 2;
            else if (error < -PI)
                error += PI * 2;

            return error;
        }

    public:
        // Control Data struct (contains computational data and caches)
        struct ControlData
        {
            float actual_x = 0; // Roll (x-axis) angle (rad)
            float actual_y = 0; // Pitch (y-axis) angle (rad)
            float actual_z = 0; // Yaw (z-axis) angle (rad)

            float output_x = 0; // Roll (x-axis) torque (Nm)
            float output_y = 0; // Pitch (y-axis) torque (Nm)
            float output_z = 0; // Yaw (z-axis) torque (Nm)

            float target_x = 0; // Roll (x-axis) angle (rad)
            float target_y = 0; // Pitch (y-axis) angle (rad)
            float target_z = 0; // Yaw (z-axis) angle (rad)
        } Orientation, Rate;

        // Input Keeping Container
        Vector3 OrientationInput, RateInput;

        // Control/Safety Flags
        struct ControlFlags : public IAttitude::IAttitudeFlags
        {
            bool useTiltLimit = true;
            bool useRateLimit = true;
            bool holdHeading = false;
            bool useExtremeTiltLimit = true;

            float tiltLimitAbs = 1.0f;        // Absolute angle clamp value (rad)
            float rateLimitAbs = 3.0f;        // Absolute rate clamp value (rad/s)
            float extremeTiltLimitAbs = 1.0f; // Absolute angle clamp value (rad)
        } flags;

        // Angular Acceleration Output
        Vector3 AngularAcceleration;

        // Constructor
        Attitude() : OrientationInput(Vector3()), RateInput(Vector3()), AngularAcceleration(Vector3())
        {
        }

        // Implementations of the IAttitude interface methods
        Attitude &configureOrientationGains(PIDGains gains_x, PIDGains gains_y, PIDGains gains_z) override
        {
            OrientationXPID.gains(gains_x.kp, gains_x.ki, gains_x.kd, gains_x.kf);
            OrientationYPID.gains(gains_y.kp, gains_y.ki, gains_y.kd, gains_y.kf);
            OrientationZPID.gains(gains_z.kp, gains_z.ki, gains_z.kd, gains_z.kf);
            return *this;
        }

        Attitude &configureRateGains(PIDGains gains_x, PIDGains gains_y, PIDGains gains_z) override
        {
            RateXPID.gains(gains_x.kp, gains_x.ki, gains_x.kd, gains_x.kf);
            RateYPID.gains(gains_y.kp, gains_y.ki, gains_y.kd, gains_y.kf);
            RateZPID.gains(gains_z.kp, gains_z.ki, gains_z.kd, gains_z.kf);
            return *this;
        }

        Attitude &updateMeasuredOrientation(float x, float y, float z) override
        {
            Orientation.actual_x = x;
            Orientation.actual_y = y;
            Orientation.actual_z = z;

            // Update the orientation input
            OrientationInput = Vector3(x, y, z);
            return *this;
        }

        Attitude &updateMeasuredRate(float x, float y, float z) override
        {
            Rate.actual_x = x;
            Rate.actual_y = y;
            Rate.actual_z = z;

            // Update the rate input
            RateInput = Vector3(x, y, z);
            return *this;
        }

        Vector3 getMeasuredOrientation()
        {
            return OrientationInput; // Return the current orientation input
        }

        Vector3 getMeasuredRate()
        {
            return RateInput; // Return the current rate input
        }

        Attitude &configureOrientationTarget(float x, float y, float z) override
        {
            Orientation.target_x = x;
            Orientation.target_y = y;
            Orientation.target_z = z;
            return *this;
        }

        Attitude &configureRateTarget(float x, float y, float z) override
        {
            Rate.target_x = x;
            Rate.target_y = y;
            Rate.target_z = z;
            return *this;
        }

        Attitude &setControlFlags(IAttitudeFlags *updFlags) override
        {
            ControlFlags *cf = static_cast<ControlFlags *>(updFlags);
            if (cf != nullptr)
            {
                flags = *cf;
            }
            return *this;
        }

        Vector3 compute(float dt) override
        {
            if (!abnormalAttitudeCheck(Orientation.actual_x, Orientation.actual_y, Orientation.actual_z))
            {
                AngularAcceleration = Vector3();
                return AngularAcceleration;
            }

            if (!abnormalRateCheck(Rate.actual_x, Rate.actual_y, Rate.actual_z))
            {
                AngularAcceleration = Vector3();
                return AngularAcceleration;
            }

            float err_roll = Orientation.target_x - Orientation.actual_x;
            float err_pitch = Orientation.target_y - Orientation.actual_y;

            // Normalize the yaw error to the range [-PI, PI]
            if (!flags.holdHeading)
            {
                // If not holding heading, we do not compute yaw error
                Orientation.target_z = Orientation.actual_z; // Maintain current yaw
            }
            else
            {
                Orientation.target_z = Orientation.target_z; // Maintain target yaw
            }

            float err_yaw = angularError(Orientation.target_z, Orientation.actual_z);

            float rate_target_x = OrientationXPID.compute(err_roll, dt);
            float rate_target_y = OrientationYPID.compute(err_pitch, dt);
            float rate_target_z = OrientationZPID.compute(err_yaw, dt);

            float err_rx = rate_target_x - Rate.actual_x;
            float err_ry = rate_target_y - Rate.actual_y;
            float err_rz = rate_target_z - Rate.actual_z;

            float alpha_x = RateXPID.compute(err_rx, dt);
            float alpha_y = RateYPID.compute(err_ry, dt);
            float alpha_z = RateZPID.compute(err_rz, dt);

            AngularAcceleration = Vector3(alpha_x, alpha_y, alpha_z);

            return AngularAcceleration;
        }

        void reset() override
        {
            OrientationXPID.reset();
            OrientationYPID.reset();
            OrientationZPID.reset();
            RateXPID.reset();
            RateYPID.reset();
            RateZPID.reset();

            // Reset flags
            flags = ControlFlags(); // Reset flags to default values

            Orientation = ControlData(); // Reset attitude data
            Rate = ControlData();        // Reset rate data

            AngularAcceleration = Vector3(); // Reset angular acceleration data
        }
    };

    class Velocity : public IVelocity
    {
    private:
        PID_filtered VelocityXPID, VelocityYPID, VelocityZPID;

        struct ControlData
        {
            float actual_x = 0;
            float actual_y = 0;
            float actual_z = 0;

            float target_x = 0;
            float target_y = 0;
            float target_z = 0;
        } VelocityD;

        Vector3 VelocityInput;

        Vector3 DesiredAcceleration; // Desired accelerations output

        bool abnormalVelocityCheck(float x, float y, float z)
        {
            if (isnan(x) || isnan(y) || isnan(z))
                return false;
            if (flags.useVelocityLimit)
            {
                if (abs(x) > flags.velocityLimitAbs || abs(y) > flags.velocityLimitAbs || abs(z) > flags.velocityLimitAbs)
                    return false;
            }
            return true;
        }

    public:
        Velocity() : VelocityInput(Vector3()), DesiredAcceleration(Vector3()) {}

        struct ControlFlags : public IVelocity::IVelocityFlags
        {
            bool useVelocityLimit = true;
            float velocityLimitAbs = 8.0f;
            float gravity = 9.81f;
            float yaw_reference = 0.0f; // in radians
        } flags;

        Velocity &configureVelocityGains(PIDGains gains_x, PIDGains gains_y, PIDGains gains_z) override
        {
            VelocityXPID.gains(gains_x.kp, gains_x.ki, gains_x.kd, gains_x.kf);
            VelocityYPID.gains(gains_y.kp, gains_y.ki, gains_y.kd, gains_y.kf);
            VelocityZPID.gains(gains_z.kp, gains_z.ki, gains_z.kd, gains_z.kf);
            return *this;
        }

        Velocity &configureAccelerationGains(PIDGains, PIDGains, PIDGains) override
        {
            // Not used in this version
            return *this;
        }

        Velocity &updateMeasuredVelocity(float x, float y, float z) override
        {
            VelocityD.actual_x = x;
            VelocityD.actual_y = y;
            VelocityD.actual_z = z;
            VelocityInput = Vector3(x, y, z);
            return *this;
        }

        Velocity &updateMeasuredAcceleration(float, float, float) override
        {
            // Not used in this version
            return *this;
        }

        Velocity &configureVelocityTarget(float x, float y, float z) override
        {
            VelocityD.target_x = x;
            VelocityD.target_y = y;
            VelocityD.target_z = z;
            return *this;
        }

        Velocity &configureAccelerationTarget(float, float, float) override
        {
            // Not used in this version
            return *this;
        }

        Velocity &setControlFlags(IVelocityFlags *updFlags) override
        {
            ControlFlags *cf = static_cast<ControlFlags *>(updFlags);
            if (cf != nullptr)
            {
                flags = *cf;
            }
            return *this;
        }

        Vector3 compute(float dt) override
        {
            if (!abnormalVelocityCheck(VelocityD.actual_x, VelocityD.actual_y, VelocityD.actual_z))
                return Vector3();

            float err_vx = VelocityD.target_x - VelocityD.actual_x;
            float err_vy = VelocityD.target_y - VelocityD.actual_y;
            float err_vz = VelocityD.target_z - VelocityD.actual_z;

            float a_x = VelocityXPID.compute(err_vx, dt);
            float a_y = VelocityYPID.compute(err_vy, dt);
            float a_z = VelocityZPID.compute(err_vz, dt);

            Vector3 a_cmd(a_x, a_y, a_z + flags.gravity);

            DesiredAcceleration = a_cmd;
            return DesiredAcceleration;
        }

        void reset() override
        {
            VelocityXPID.reset();
            VelocityYPID.reset();
            VelocityZPID.reset();

            VelocityD = ControlData();
            DesiredAcceleration = Vector3();
        }
    };

    class Orchestrator : public IOrchestrator
    {
    private:
        IRigidModel *model_ptr = nullptr;

    public:
        Orchestrator() {}

        using Torque = Vector3;  // Define Torque as a 3D vector type (for syntax clarity)
        Torque controllerOutput; // Output torque vector

        IAttitude *attitudeInterface = nullptr; // Returns desired angular velocity
        IVelocity *velocityInterface = nullptr; // Returns desired acceleration
        IPosition *positionInterface = nullptr; // Returns desired velocity

        Orchestrator &useAttitudeInterface(IAttitude *attitudeCtrl) override
        {
            if (attitudeCtrl != nullptr)
            {
                attitudeInterface = attitudeCtrl;
            }
            return *this;
        }

        Orchestrator &useVelocityInterface(IVelocity *velocityCtrl) override
        {
            if (velocityCtrl != nullptr)
            {
                velocityInterface = velocityCtrl;
            }
            return *this;
        }

        Orchestrator &usePositionInterface(IPosition *positionCtrl) override
        {
            if (positionCtrl != nullptr)
            {
                positionInterface = positionCtrl;
            }
            return *this;
        }

        Orchestrator &useModel(IRigidModel *model) override
        {
            model_ptr = model;
            return *this;
        }

        Vector3 compute(float dt) override
        {
            bool hasAtt = attitudeInterface != nullptr;
            bool hasVel = velocityInterface != nullptr;
            bool hasPos = positionInterface != nullptr;

            // Invalid configurations
            if (!hasAtt) // Attitude is mandatory for all valid modes
            {
                controllerOutput = Torque(0, 0, 0);
                return controllerOutput;
            }

            if (hasPos && !hasVel)
            {
                // Position requires velocity
                controllerOutput = Torque(0, 0, 0);
                return controllerOutput;
            }

            if (hasVel && !hasAtt)
            {
                // Velocity requires attitude (already caught above, redundant safety)
                controllerOutput = Torque(0, 0, 0);
                return controllerOutput;
            }

            // Mode 1: Attitude only
            if (hasAtt && !hasVel && !hasPos)
            {
                Vector3 des_alpha = attitudeInterface->compute(dt);
                Vector3 omega = attitudeInterface->getMeasuredRate();

                if (model_ptr)
                    controllerOutput = model_ptr->computeRequiredTorque(omega, des_alpha);
                else
                    controllerOutput = Torque(des_alpha.x, des_alpha.y, des_alpha.z);

                return controllerOutput;
            }

            // Mode 2: Attitude + Velocity
            if (hasAtt && hasVel && !hasPos)
            {
                Vector3 des_acc = velocityInterface->compute(dt);

                Vector3 orientationTarget = model_ptr->computeRequiredOrientation(des_acc, attitudeInterface->getMeasuredOrientation().z);

                attitudeInterface->configureOrientationTarget(
                    orientationTarget.x, orientationTarget.y, orientationTarget.z);

                Vector3 des_alpha = attitudeInterface->compute(dt);
                Vector3 omega = attitudeInterface->getMeasuredRate();

                if (model_ptr)
                    controllerOutput = model_ptr->computeRequiredTorque(omega, des_alpha);
                else
                    controllerOutput = Torque(des_alpha.x, des_alpha.y, des_alpha.z);

                return controllerOutput;
            }

            // Mode 3: Attitude + Velocity + Position
            if (hasAtt && hasVel && hasPos)
            {
                Vector3 velocityTarget = positionInterface->compute(dt);
                velocityInterface->configureVelocityTarget(
                    velocityTarget.x, velocityTarget.y, velocityTarget.z);

                Vector3 orientationTarget = velocityInterface->compute(dt);
                attitudeInterface->configureOrientationTarget(
                    orientationTarget.x, orientationTarget.y, orientationTarget.z);

                Vector3 des_alpha = attitudeInterface->compute(dt);
                Vector3 omega = attitudeInterface->getMeasuredRate();

                if (model_ptr)
                    controllerOutput = model_ptr->computeRequiredTorque(omega, des_alpha);
                else
                    controllerOutput = Torque(des_alpha.x, des_alpha.y, des_alpha.z);

                return controllerOutput;
            }

            // Fallback: should never reach here if logic is sound
            controllerOutput = Torque(0, 0, 0);
            return controllerOutput;
        }

        void reset() override
        {
            if (attitudeInterface)
            {
                attitudeInterface->reset();
            }
            if (velocityInterface)
            {
                velocityInterface->reset();
            }
            if (positionInterface)
            {
                positionInterface->reset();
            }

            controllerOutput = Torque(0, 0, 0); // Reset output torque
        }
    };
}