#pragma once

#include <Firmus.h>
#include <Firmus/models/Rigid.h>
#include <Firmus/hardware/mpu6050.h>
#include <Firmus/mpc/mpc.h>

using namespace FMat;

#define PLOT_TORQUE 0

/* Timing & Feedback */
constexpr unsigned long target_period_us = 10000;
unsigned long last_time = 0;

/* Hardware */
mpu6050 imu(19, 23);

/* Control */
auto Q = FMat::diagonal<float, 6>({ 10, 10, 10, 1, 1, 1 });
auto R = FMat::diagonal<float, 3>(0.1);
auto Qf = Q;

MPC::BodyAttitudeMPCPolicy<4> mpc(Q, R, Qf);

/* Model */
RigidModel<4> model4;

RigidModel<4> buildModel4() {
  constexpr float theta_1 = PI / 4;
  constexpr float theta_2 = (3 * PI) / 4;
  constexpr float theta_3 = (5 * PI) / 4;
  constexpr float theta_4 = (7 * PI) / 4;
  constexpr float l = 0.125;
  constexpr float k_yaw = 1;
  constexpr float k_F = 1;

  return RigidModel<4>()
    .setMomentArm(l, l, l, l)
    .setActuatorAngles(theta_1, theta_2, theta_3, theta_4)
    .setAllocationMatrix({ { l * sin(theta_1), l * sin(theta_2), l * sin(theta_3), l * sin(theta_4) },
                           { l * cos(theta_1), l * cos(theta_2), l * cos(theta_3), l * cos(theta_4) },
                           { -k_yaw, k_yaw, -k_yaw, k_yaw },
                           { k_F, k_F, k_F, k_F } })
    .setInertiaMatrix(diagonal<float, 3>({ 0.125f, 0.125f, 0.10f }))
    .setMass(0.8)
    .setOperatingVoltage(11.1)
    .setMotorKv(2300, 2300, 2300, 2300)
    .setPropellerDiameter(0.1016)
    .setPropellerPitch(0.1270);
}
