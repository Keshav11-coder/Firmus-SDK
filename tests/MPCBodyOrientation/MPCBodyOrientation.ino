#include "config.h"

Snapshot frame{};
Snapshot target = {
  .orientation = { 0.01, 0.01, 0 }
};

void setup() {
  Serial.begin(115200);

  model4 = buildModel4();

  imu.begin();
  imu.calibrate();

  auto Sx = mpc.getSx();

  FMat::print(Sx);
}

void loop() {
  unsigned long now = micros();
  float dt = (now - last_time) / 1e6f;
  last_time = now;

  imu.read();

  frame.orientation = Vector3(imu.pitch_rad, imu.roll_rad, imu.yaw_rad);
  frame.angular_velocity = Vector3(imu.gx, imu.gy, imu.gz);

  Snapshot result = mpc.compute(frame, target, dt);

  // Convert to torque using model
  // result.torque = model4.computeRequiredTorque(frame.angular_velocity, result.angular_acceleration);

  Serial.printf("Acc X: %.4f  Acc Y: %.4f  Acc Z: %.4f\n", result.angular_velocity.x, result.angular_velocity.y, result.angular_velocity.z);
}