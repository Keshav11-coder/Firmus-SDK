// © 2025 Keshav Haripersad
//
// Footprint:
// - Heap Usage: ~<00.0> KB used / 303 KB total (~00.0% utilization)
// - CPU Usage: ~0.0% (single-core approximation, 50 Hz loop rate)
// - MCU: ESP32 (dual-core, 240 MHz)
// - Sensors: MPU6050 (I2C interface)
// - Control: Cascade PID attitude controller with RigidModel<4>
// - Loop Frequency: ~50 Hz (timing based on micros())
// - Features: Real-time orientation & rate feedback, torque computation, mixer for rotor force distribution
// - Notes: Efficient static allocation, minimal dynamic memory usage
// - Limitations: CPU usage measured only on Core 0; does not include multi-core workload
// - Usage: Initialize MPU6050, calibrate sensor, call controller compute(dt) each loop
// - Logging: Serial prints CPU usage every second; disable or adapt for higher frequencies (100Hz)
//
// References:
// - SDK v1.3.3
// - ESP-IDF timers and FreeRTOS primitives for CPU measurement
// - MPU6050 datasheet and driver documentation

#include <Firmus.h>
#include <Firmus/mpu6050.h>
#include <Firmus/Controllers/Cascade.h> // Uses RigidModel+Matrix2 by default
#include <Firmus/Mixers/Mixer.h> // Uses RigidModel+Matrix2 by default

using namespace Firmus;
using namespace Matrix2;

const float theta_1 = PI / 4;
const float theta_2 = (3 * PI) / 4;
const float theta_3 = (5 * PI) / 4;
const float theta_4 = (7 * PI) / 4;

const float l = 0.125;

const float k_yaw = 1;
const float k_F = 1;

auto model4 = RigidModel<4>()
  .setMomentArm(l, l, l, l)
  .setActuatorAngles(theta_1, theta_2, theta_3, theta_4)
  .setAllocationMatrix({
    { l * sin(theta_1), l * sin(theta_2), l * sin(theta_3), l * sin(theta_4) }, // torque x
    { l * cos(theta_1), l * cos(theta_2), l * cos(theta_3), l * cos(theta_4) }, // torque y
    { -k_yaw,  k_yaw,  -k_yaw,   k_yaw }, // torque z
    {  k_F,   k_F,    k_F,    k_F  } // thrust T
  })
  .setInertiaMatrix(diagonal<float, 3>({ 0.125f, 0.125f, 0.10f }))
  .setMass(0.8)
  .setOperatingVoltage(11.1)
  .setMotorKv(2300, 2300, 2300, 2300)
  .setPropellerDiameter(0.1016)
  .setPropellerPitch(0.1270);

auto attitudeCtrl = Cascade::Attitude()
  .configureOrientationGains(PIDGains{.kp = 4.5, .ki = 0.02, .kd = 0.05, .kf = 15},
                             PIDGains{.kp = 4.5, .ki = 0.02, .kd = 0.05, .kf = 15},
                             PIDGains{.kp = 5.0, .ki = 0.10, .kd = 0.05, .kf = 10})
  .configureRateGains(PIDGains{.kp = 2.5, .ki = 0.05, .kd = 0.02, .kf = 10},
                      PIDGains{.kp = 2.5, .ki = 0.05, .kd = 0.02, .kf = 10},
                      PIDGains{.kp = 3.5, .ki = 0.02, .kd = 0.01, .kf = 10});

auto velocityCtrl = Cascade::Velocity()
  .configureVelocityGains(PIDGains{.kp = 0.8, .ki = 0.00, .kd = 0.20, .kf = 10},
                          PIDGains{.kp = 0.8, .ki = 0.00, .kd = 0.20, .kf = 10},
                          PIDGains{.kp = 1.0, .ki = 0.05, .kd = 0.20, .kf = 10});

auto resolver = Cascade::Orchestrator()
  .useModel(&model4)
  .useVelocityInterface(&velocityCtrl)
  .useAttitudeInterface(&attitudeCtrl);

Mixer<4> mixer(&model4);
mpu6050 mpu(19, 23);

void setup() {
  Serial.begin(115200);

  mpu.begin();
  mpu.calibrate();

  Cascade::Attitude::ControlFlags flagsAtt;
  flagsAtt.useTiltLimit=true;
  flagsAtt.useRateLimit=true;
  flagsAtt.holdHeading=false;
  flagsAtt.tiltLimitAbs=PI / 4;
  flagsAtt.rateLimitAbs=3.0f;

  Cascade::Velocity::ControlFlags flagsVel;
  flagsVel.useVelocityLimit=true;
  flagsVel.velocityLimitAbs=8.0f;
  flagsVel.gravity=9.81f; // Do NOT negate
  flagsVel.yaw_reference=0;

  attitudeCtrl.setControlFlags(&flagsAtt);
  velocityCtrl.setControlFlags(&flagsVel);
}

unsigned long last_time = 0;
float dt = 0;

constexpr unsigned long target_period_us = 10000; // In micros, (1 / Hz) * 1e6

void loop()
{
  unsigned long now = micros();
  dt = (now - last_time) / 1e6f;

  mpu.read();

  attitudeCtrl.updateMeasuredOrientation(mpu.pitch_rad, mpu.roll_rad, mpu.yaw_rad);
  attitudeCtrl.updateMeasuredRate(mpu.gx, mpu.gy, mpu.gz);

  velocityCtrl.updateMeasuredVelocity(0.4, 0, 0);
  velocityCtrl.configureVelocityTarget(0.4, 0, 0);

  Cascade::Orchestrator::Torque torque = resolver.compute(dt);

  auto thrusts = mixer.mix(torque, model4.getMass()*9.81f).map_to_pwm(0.00f, 9.09f, 1000, 2000);

  Serial.print("Motor 1: ");
  Serial.print(thrusts[0]);
  Serial.print(" Motor 2: ");
  Serial.print(thrusts[1]);
  Serial.print(" Motor 3: ");
  Serial.print(thrusts[2]);
  Serial.print(" Motor 4: ");
  Serial.println(thrusts[3]);

//  Serial.println(1.0f / dt);

  // Wait for remaining time in period
  unsigned long loop_duration = micros() - now;
  if (loop_duration < target_period_us) {
    delayMicroseconds(target_period_us - loop_duration);
  }

  last_time = now;
}

// Sketch Promotion Checklist (v1.3.3) — Diluted by Criticality

// --- Critical (Promotion Blockers) ---
// [X] dt computation accurate with micros(), no rollover or drift at 50–100 Hz
// [X] No NaN/inf in controller outputs across full input domain
// [ ] Controller compute() completes within <10 ms at 50 Hz (CPU-bound check)
// [ ] Stable PID tuning for attitude and rate layers under nominal conditions
// [ ] Allocation matrix produces bounded, realistic actuator commands
// [ ] All actuator outputs finite, clamped if necessary
// [ ] Heap usage stable under peak and idle load, no leaks (<40kB target)
// [ ] Architecture compliant with IRigidModel, IAttitude, IController interfaces
// [ ] Mixer logic (if present) correct, invertible, non-singular
// [ ] Core control flow deterministic and non-blocking
// [ ] Loop jitter <5% of target period; no missed frames

// --- High Priority (Robustness Enablers) ---
// [ ] Failsafe and saturation handling present in control layer
// [ ] Control flags (tilt, rate limit) active and enforced
// [ ] Cascade::Attitude and ::Controller modules verified against regression baselines
// [ ] BLE input bridge (if enabled) gracefully handles disconnect and loss
// [ ] Torque range within expected physical limits; logs inspected
// [ ] Modularization of controller path — no hard dependencies or inline overrides
// [ ] IMixer base interface introduced and operational if mixer used
// [ ] Code compiles warning-free on SDK 1.3.2, across toolchain versions

// --- Medium Priority (Development Support) ---
// [ ] Telemetry/logging decoupled from control loop; can be toggled off
// [ ] BLE/control inputs abstracted (e.g., InputBridge layer)
// [ ] Unit-test scaffolds available for controller and mixer
// [ ] Sensor readings (orientation, rate) time-synced with control compute()
// [ ] Modular failsafe definitions injected via control policy, not hardcoded

// --- Low Priority (Optional Enhancements) ---
// [ ] Support extended to alternate vehicle topologies (RM6, tricopter)
// [ ] Mixer supports constraint-aware redistribution (soft saturation)
// [ ] Onboard gain tuning interface or field parameter loading
// [ ] Loop jitter and timing stats loggable during flight
// [ ] Dynamic gain scheduling or adaptation framework stubbed in

// --- Finalization ---
// [ ] Final engineering lead sign-off prior to SDK promotion
// Promotion Path: SDK 1.3.3 -> 1.3.4
