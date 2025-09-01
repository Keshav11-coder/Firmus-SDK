#pragma once

/** © 2025 Keshav Haripersad
 *  RigidModel<N> implementation class - Rigid.h
 *  | abstract.h for virtual table / overview.
 *
 *  Licensed under the Apache 2.0 license (check LICENSE).
 *  Github link: https://github.com/Keshav11-coder/Firmus-SDK
 *  Firmus SDK version: 1.3.3
 */

#include <cmath>
#include "../tools/matrix2.h" // Include the matrix2 library for matrix operations

#include "abstract.h" // Include the abstract model interface

struct Vector3; // Forward declaration of Vector3 (Firmus.h for implementation)

template <size_t Actuators>
class RigidModel : public IRigidModel
{
private:
    // Geometry, inertia & distribution
    float _l[Actuators] = {};
    float _theta[Actuators] = {};
    float _alpha[Actuators] = {};
    float _mass = 0;

    Matrix2::matrix<float, 4, Actuators> _allocationMatrix = Matrix2::zeros<float, 4, Actuators>();
    Matrix2::matrix<float, Actuators, 4> _inverseAllocationMatrix = Matrix2::zeros<float, Actuators, 4>();

    Matrix2::matrix<float, 3, 3> _inertiaMatrix = Matrix2::identity<float, 3>();

    double _operatingVoltage = 0;
    int _motorKv[Actuators] = {};

    float _propellerDiameter = 0;
    float _propellerPitch = 0;

public:
    RigidModel() {}

    RigidModel(float l[], float theta[], float alpha[], float mass, Matrix2::matrix<float, 4, Actuators> A_tau, Matrix2::matrix<float, 6, Actuators> A_w, Matrix2::matrix<float, 3, 3> I, double V, int motorKv[], float d, float p)
        : _l{l}, _theta{theta}, _alpha{alpha}, _mass(mass), _allocationMatrix(A_tau), _inverseAllocationMatrix(A_tau.pseudoinverse()), _inertiaMatrix(I), _operatingVoltage(V), _motorKv{motorKv}, _propellerDiameter(d), _propellerPitch(p) {}

    // === Inherited virtual methods ===
    float getMomentArm(size_t i) override
    {
        return (i >= Actuators) ? -1 : _l[i];
    }

    float getActuatorAngle(size_t i) override
    {
        return (i >= Actuators) ? -1 : _theta[i];
    }

    float getActuatorTilt(size_t i) override
    {
        return (i >= Actuators) ? -1 : _alpha[i];
    }

    float getMass() override
    {
        return _mass;
    }

    double getOperatingVoltage() override
    {
        return _operatingVoltage;
    }

    int getMotorKv(size_t i) override
    {
        return (i >= Actuators) ? -1 : _motorKv[i];
    }

    float getPropellerDiameter() override
    {
        return _propellerDiameter;
    }

    float getPropellerPitch() override
    {
        return _propellerPitch;
    }

    size_t getActuatorCount() override
    {
        return Actuators;
    }

    void getAllocationMatrix(void *out) override
    {
        *reinterpret_cast<Matrix2::matrix<float, 4, Actuators> *>(out) = _allocationMatrix;
    }

    void getInverseAllocationMatrix(void *out) override
    {
        *reinterpret_cast<Matrix2::matrix<float, Actuators, 4> *>(out) = _inverseAllocationMatrix;
    }

    void getInertiaMatrix(void *out) override
    {
        *reinterpret_cast<Matrix2::matrix<float, 3, 3> *>(out) = _inertiaMatrix;
    }

    // === Setters ===
    template <typename... Args>
    RigidModel &setMomentArm(Args... args)
    {
        static_assert(sizeof...(args) == Actuators, "Number of moments must match the number of actuators.");
        float temp[] = {static_cast<float>(args)...};
        for (size_t i = 0; i < Actuators; ++i)
        {
            _l[i] = temp[i];
        }
        return *this;
    }

    template <typename... Args>
    RigidModel &setActuatorAngles(Args... args)
    {
        static_assert(sizeof...(args) == Actuators, "Number of angles must match the number of actuators.");
        float temp[] = {static_cast<float>(args)...};
        for (size_t i = 0; i < Actuators; ++i)
        {
            _theta[i] = temp[i];
        }
        return *this;
    }

    template <typename... Args>
    RigidModel &setActuatorTilts(Args... args)
    {
        static_assert(sizeof...(args) == Actuators, "Number of tilts must match the number of actuators.");
        float temp[] = {static_cast<float>(args)...};
        for (size_t i = 0; i < Actuators; ++i)
        {
            _alpha[i] = temp[i];
        }
        return *this;
    }

    RigidModel &setAllocationMatrix(Matrix2::matrix<float, 4, Actuators> A_tau)
    {
        _allocationMatrix = A_tau;
        _inverseAllocationMatrix = A_tau.pseudoinverse();
        return *this;
    }

    RigidModel &setInverseAllocationMatrix(Matrix2::matrix<float, Actuators, 4> A_tau_inv)
    {
        _inverseAllocationMatrix = A_tau_inv;
        return *this;
    }

    RigidModel &setInertiaMatrix(Matrix2::matrix<float, 3, 3> I)
    {
        _inertiaMatrix = I;
        return *this;
    }

    RigidModel &setMass(float m)
    {
        _mass = m;
        return *this;
    }

    RigidModel &setOperatingVoltage(double V)
    {
        _operatingVoltage = V;
        return *this;
    }

    template <typename... Args>
    RigidModel &setMotorKv(Args... args)
    {
        static_assert(sizeof...(args) == Actuators, "Number of Kv ratings must match the number of actuators.");
        int temp[] = {static_cast<int>(args)...};
        for (size_t i = 0; i < Actuators; ++i)
        {
            _motorKv[i] = temp[i];
        }
        return *this;
    }

    RigidModel &setPropellerDiameter(float d)
    {
        _propellerDiameter = d;
        return *this;
    }

    RigidModel &setPropellerPitch(float p)
    {
        _propellerPitch = p;
        return *this;
    }

    // Physical computation
    Vector3 computeRequiredTorque(Vector3 omega, Vector3 des_alpha) override
    {
        // Use model to compute full torque
        auto omega_ = Matrix2::make_matrix<float, 3, 1>({{omega.x}, {omega.y}, {omega.z}});
        auto I = _inertiaMatrix;
        auto L = I * omega_; // Angular Momentum
        auto des_alpha_ = Matrix2::make_matrix<float, 3, 1>({{des_alpha.x}, {des_alpha.y}, {des_alpha.z}});
        auto tau_vec = I * des_alpha_ + Matrix2::cross_matrix3(omega_) * L;

        return Vector3(tau_vec(0, 0), tau_vec(1, 0), tau_vec(2, 0));
    }

    Vector3 computeRequiredOrientation(Vector3 des_acc, float yaw_reference) override
    {
        auto T_vec = Matrix2::make_matrix<float, 3, 1>({{des_acc.x}, {des_acc.y}, {des_acc.z}}) * _mass;
        float T = T_vec.norm();
        auto T_hat = Matrix2::make_matrix<float, 3, 1>({{T_vec(0, 0) / T}, {T_vec(1, 0) / T}, {T_vec(2, 0) / T}});

        float pitch = asin(T_hat(0, 0));               // pitch from x component
        float roll = atan2(-T_hat(1, 0), T_hat(2, 0)); // roll from y,z components
        float yaw = yaw_reference;                     // pass yaw through as is

        return Vector3(roll, pitch, yaw);
    }
};