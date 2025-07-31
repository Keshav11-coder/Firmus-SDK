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
// - SDK v1.3.4 beta
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

Cascade::FrameCorrectionStrategy orientation_strategy(4.5, 0.02, 0.05, 15,
                                                      4.5, 0.02, 0.05, 15,
                                                      5.0, 0.10, 0.05, 10);
Cascade::FrameCorrectionStrategy rate_strategy(2.5, 0.05, 0.02, 10,
                                               2.5, 0.05, 0.02, 10,
                                               3.5, 0.02, 0.01, 10);

Cascade::FrameOrientationControlLayer orientation;
Cascade::FrameRateControlLayer rate;

Cascade::FrameControlOrchestrator orchestrator(3);

Mixer<4> mixer(&model4);
mpu6050 mpu(19, 23);

// Create frame initial state and Snapshot
Snapshot frame = Snapshot{
  .position = Vector3{.x=0,.y=0,.z=0},
  .velocity = Vector3{.x=0,.y=0,.z=0},
  .acceleration = Vector3{.x=0,.y=0,.z=0},
  .orientation = Vector3{.x=mpu.pitch_rad,.y=mpu.roll_rad,.z=mpu.yaw_rad},
  .angular_velocity = Vector3{.x=mpu.gx,.y=mpu.gy,.z=mpu.gz},
  .angular_acceleration = Vector3{.x=0,.y=0,.z=0},

  .torque = Vector3{.x=0,.y=0,.z=0}
};

// Frame static target
Snapshot target = Snapshot{
  .position = Vector3{.x=0,.y=0,.z=0},
  .velocity = Vector3{.x=0,.y=0,.z=0},
  .acceleration = Vector3{.x=0,.y=0,.z=0},
  .orientation = Vector3{.x=0,.y=0,.z=0}
};

void setup() {
  Serial.begin(115200);

  mpu.begin();
  mpu.calibrate();

  orientation.setCorrectionStrategy(&orientation_strategy);
  rate.setCorrectionStrategy(&rate_strategy);

  orchestrator.useModel(&model4)
              .addControlLayer(&orientation)
              .addControlLayer(&rate);
}

unsigned long last_time = 0;
float dt = 0;

constexpr unsigned long target_period_us = 10000; // In micros, (1 / Hz) * 1e6

void loop()
{
  unsigned long now = micros();
  dt = (now - last_time) / 1e6f;

  mpu.read();

  // Update frame state
  frame = Snapshot{
    .position = Vector3{.x=0,.y=0,.z=0},
    .velocity = Vector3{.x=0,.y=0,.z=0},
    .acceleration = Vector3{.x=0,.y=0,.z=0},
    .orientation = Vector3{.x=mpu.pitch_rad,.y=mpu.roll_rad,.z=mpu.yaw_rad},
    .angular_velocity = Vector3{.x=mpu.gx,.y=mpu.gy,.z=mpu.gz},
  };

  frame.timestamp = now;
  frame.loop_freq = 1.0f / dt;

  Snapshot correction = orchestrator.process(frame, target, dt);

  Serial.print("Torque X: ");
  Serial.print(correction.torque.x);
  Serial.print(" Torque Y: ");
  Serial.print(correction.torque.y);
  Serial.print(" Torque Z: ");
  Serial.println(correction.torque.z);

//  Serial.println(1.0f / dt);

  // Wait for remaining time in period
  unsigned long loop_duration = micros() - now;
  if (loop_duration < target_period_us) {
    delayMicroseconds(target_period_us - loop_duration);
  }

  last_time = now;
}

// Sketch Promotion Checklist (v1.3.4) — Diluted by Criticality

// --- todo

// --- Finalization ---
// [ ] Final engineering lead sign-off prior to SDK promotion
// Promotion Path: SDK 1.3.3 -> 1.3.4
