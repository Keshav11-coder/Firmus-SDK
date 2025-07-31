// main.cpp

#include "config.h"

Cascade::FrameOrientationControlLayer orientation;
Cascade::FrameRateControlLayer rate;
Cascade::FrameControlOrchestrator orchestrator(3);

mpu6050 mpu(19, 23);
Mixer<4> mixer(nullptr);
RigidModel<4> model4;

Snapshot frame{};
Snapshot target = {
  .position = {0, 0, 0},
  .velocity = {0, 0, 0},
  .acceleration = {0, 0, 0},
  .orientation = {0, 0, 0}
};

void setup() {
  Serial.begin(115200);
  mpu.begin();
  mpu.calibrate();

  model4 = buildModel4();
  mixer.setModel(&model4);

  orientation.setCorrectionStrategy(&orientation_strategy);
  rate.setCorrectionStrategy(&rate_strategy);

  orchestrator.useModel(&model4)
              .addControlLayer(&orientation)
              .addControlLayer(&rate);
}

unsigned long last_time = 0;

void loop() {
  unsigned long now = micros();
  float dt = (now - last_time) / 1e6f;
  last_time = now;

  mpu.read();

  frame.orientation = {mpu.pitch_rad, mpu.roll_rad, mpu.yaw_rad};
  frame.angular_velocity = {mpu.gx, mpu.gy, mpu.gz};
  frame.timestamp = now;
  frame.loop_freq = 1.0f / dt;

  Snapshot correction = orchestrator.process(frame, target, dt);

//  Serial.print("Torque X: "); Serial.print(correction.torque.x);
//  Serial.print(" Torque Y: "); Serial.print(correction.torque.y);
//  Serial.print(" Torque Z: "); Serial.println(correction.torque.z);
  Serial.printf("%u/%u\n", ESP.getHeapSize()-ESP.getFreeHeap(), ESP.getHeapSize());

  unsigned long loop_duration = micros() - now;
  if (loop_duration < target_period_us) {
    delayMicroseconds(target_period_us - loop_duration);
  }
}
