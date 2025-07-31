// config.h

#pragma once
#include <Firmus.h>
#include <Firmus/Controllers/Cascade.h>
#include <Firmus/Mixers/Mixer.h>
#include <Firmus/mpu6050.h>

using namespace Firmus;
using namespace Matrix2;

// Rotor layout
constexpr float theta_1 = PI / 4;
constexpr float theta_2 = (3 * PI) / 4;
constexpr float theta_3 = (5 * PI) / 4;
constexpr float theta_4 = (7 * PI) / 4;
constexpr float l = 0.125;
constexpr float k_yaw = 1;
constexpr float k_F = 1;

// Loop
constexpr unsigned long target_period_us = 10000;

// Controller strategies
inline Cascade::FrameCorrectionStrategy orientation_strategy = {
  4.5, 0.02, 0.05, 15,
  4.5, 0.02, 0.05, 15,
  5.0, 0.10, 0.05, 10
};

inline Cascade::FrameCorrectionStrategy rate_strategy = {
  2.5, 0.05, 0.02, 10,
  2.5, 0.05, 0.02, 10,
  3.5, 0.02, 0.01, 10
};

// Model builder
inline RigidModel<4> buildModel4() {
  return RigidModel<4>()
    .setMomentArm(l, l, l, l)
    .setActuatorAngles(theta_1, theta_2, theta_3, theta_4)
    .setAllocationMatrix({
      { l * sin(theta_1), l * sin(theta_2), l * sin(theta_3), l * sin(theta_4) },
      { l * cos(theta_1), l * cos(theta_2), l * cos(theta_3), l * cos(theta_4) },
      { -k_yaw,  k_yaw, -k_yaw, k_yaw },
      {  k_F,    k_F,    k_F,   k_F }
    })
    .setInertiaMatrix(diagonal<float, 3>({ 0.125f, 0.125f, 0.10f }))
    .setMass(0.8)
    .setOperatingVoltage(11.1)
    .setMotorKv(2300, 2300, 2300, 2300)
    .setPropellerDiameter(0.1016)
    .setPropellerPitch(0.1270);
}
