#include "config.h"

Cascade::BodyOrientationLayer orientation;
Cascade::BodyRateLayer rate;
Cascade::BodyController controller(2);  // Two control layers

mpu6050 mpu(19, 23);
RigidModel<4> model4;

Snapshot frame{};
Snapshot target = {
  .position = { 0, 0, 0 },
  .velocity = { 0, 0, 0 },
  .acceleration = { 0, 0, 0 },
  .orientation = { 0, 0, 0 }
};

unsigned long last_time = 0;

void setup() {
  Serial.begin(115200);

  mpu.begin();
  mpu.calibrate();

  model4 = buildModel4();

  orientation.setCorrectionPolicy(&orientation_plc);
  rate.setCorrectionPolicy(&rate_plc);

  controller.useModel(&model4)
    .addControlLayer(&orientation)
    .addControlLayer(&rate);
}

void loop() {
  unsigned long now = micros();
  float dt = (now - last_time) / 1e6f;
  last_time = now;

  mpu.read();

  frame.orientation = { mpu.pitch_rad, mpu.roll_rad, mpu.yaw_rad };
  frame.angular_velocity = { mpu.gx, mpu.gy, mpu.gz };
  frame.timestamp = now;
  frame.loop_freq = 1.0f / dt;

  Snapshot body_target = controller.process(frame, target, dt);

  Serial.printf("Torque X: %.4f  Torque Y: %.4f  Torque Z: %.4f\n", body_target.torque.x, body_target.torque.y, body_target.torque.z);

  unsigned long loop_duration = micros() - now;
  if (loop_duration < target_period_us) {
    delayMicroseconds(target_period_us - loop_duration);
  }
}
