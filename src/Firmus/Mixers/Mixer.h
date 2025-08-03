#pragma once

/** © 2025 Keshav Haripersad
 *  A mixer class (mixer<N>) for allocation / distributing torque and thrust to actuators - Mixer.h
 *  | Requires a RigidModel (IRigidModel) to get the allocation matrix.
 *
 *  Licensed under the Apache 2.0 license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/Firmus-SDK
 *  Firmus SDK version: 1.3.4
 */

#include <Arduino.h>

#include "../models/Rigid.h" // Include the RigidModel class for model operations

#include "../tools/matrix2.h" // Include the matrix2 library for matrix operations

struct Vector3; // Forward declaration of Vector3 (Firmus.h for implementation)

template <size_t Actuators>
class Mixer
{
private:
    IRigidModel *model; // Pointer to the RigidModel instance

public:
    // Constructor to initialize the Mixer with a RigidModel instance
    Mixer(IRigidModel *model) : model(model) {}

    struct MixerOutput
    {
        Matrix2::matrix<float, Actuators, 1> motorThrusts = Matrix2::zeros<float, Actuators, 1>(); // Holds the calculated motor thrusts

        // Copy operator to allow easy assignment of MixerOutput
        void set(const Matrix2::matrix<float, Actuators, 1> &thrusts)
        {
            motorThrusts = thrusts; // Set the motor thrusts
        }

        // Method to get the motor thrusts
        float operator[](size_t i) const
        {
            return (i < Actuators) ? motorThrusts(i, 0) : 0.0f; // Return the thrust for the given motor index
        }

        // Method to get the motor thrusts as a matrix
        Matrix2::matrix<float, Actuators, 1> to_matrix() const
        {
            return motorThrusts;
        }

        MixerOutput &map_to_pwm(float min_N, float max_N, float min_pwm, float max_pwm)
        {
            // Map the motor thrusts to PWM values
            for (size_t i = 0; i < Actuators; ++i)
            {
                motorThrusts(i, 0) = constrain(motorThrusts(i, 0), min_N, max_N);
                motorThrusts(i, 0) = min_pwm + ((motorThrusts(i, 0) - min_N) / (max_N - min_N)) * (max_pwm - min_pwm);
            }

            return *this; // Return the MixerOutput instance for possible chaining
        }
    } output; // Output structure to hold the result of the mixing operation

    // Method to mix inputs (torque and thrust) into motor thrusts
    // Takes a reference to a Vector3 for torque and a float for thrust
    MixerOutput mix(Vector3 &torque, float thrust)
    {
        // Ensure the model is valid
        if (!model)
        {
            return MixerOutput(); // Return empty output
        }

        Matrix2::matrix<float, Actuators, 4> A_inv = Matrix2::zeros<float, Actuators, 4>();
        model->getInverseAllocationMatrix(&A_inv); // Get the allocation matrix from the model

        output.set(A_inv * Matrix2::make_matrix<float, 4, 1>({{torque.x}, {torque.y}, {torque.z}, {thrust}})); // Mix inputs into output

        return output; // Return the filled output structure
    }

    void setModel(IRigidModel *model_)
    {
        model = model_;
    }
};