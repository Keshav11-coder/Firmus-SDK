# Firmus-SDK

**Version:** `v1.3.5`  
**Recommended Board:** ESP32 v3.1.0  

---

## 📖 Overview

**Firmus** is a high-performance, modular flight control SDK built around **clarity**, **composability**, and **physics-accurate modeling**.  

It provides a **clean interface** for building both simple cascaded controllers and complex high-level control architectures, now with **Snapshot-based policies** and experimental **Model Predictive Control (MPC)**.  

---

## 🧩 Key Principles

- **Modular Control Layers** – Each layer is swappable and composable.  
- **Frame-Level Operation** – All quantities are defined in the **body (local) reference frame**.  
- **Deterministic Abstractions** – Predictable interfaces, no hidden state.  
- **Hardware-Agnostic** – Core logic is separated from hardware. Hardware drivers now live in `/hardware` (MPU6050, ESC, etc.).  

---

## 🛠 Architecture Summary

Traditional **Cascade Control**:

```
[ Sensor / Hardware Layer ]
      ↓
[ Rigid Model ]
      ↓
[ Orientation Layer ]  ← BodyPID or other policy
      ↓
[ Rate Layer ]
      ↓
[ Torque Computation via Model ]
      ↓
[ Actuators / Mixer ]
```

Experimental **MPC Control**:

```
[ Sensor / Hardware Layer ]
      ↓
[ Snapshot State ]
      ↓
[ MPC Optimizer ]  ← Multi-step prediction horizon
      ↓
[ Desired Angular Acceleration ]
      ↓
[ Rigid Model: Compute Torque ]
      ↓
[ Actuators / Mixer ]
```

> **Note:** The new MPC flow uses Snapshots end-to-end, allowing the policy to see **full body state**, not just a `Vector3`.  

---

## 🧩 Core Interfaces

### `Vector3`

3D vector structure for all spatial and angular quantities.

```cpp
struct Vector3 {
    float x, y, z;
};
```

### `Snapshot`

Encapsulates the **complete dynamic state** of a body at a given moment. Central to the new Snapshot-based policies.

```cpp
struct Snapshot {
    Vector3 position;
    Vector3 velocity;
    Vector3 acceleration;
    Vector3 orientation;
    Vector3 angular_velocity;
    Vector3 angular_acceleration;
    uint64_t timestamp;
    uint64_t loop_freq;
    Vector3 torque;
};
```

---

## 🧩 Control Stack Interfaces

### `IBodyCorrectionPolicy`  

Computes corrections between measured and target states.  

- **New:** Can operate on either `Vector3` **or** `Snapshot`.  
- Both methods are **pure virtual**, so if you don’t implement one, it **must** be stubbed out.

```cpp
Vector3 compute(const Vector3 &target, const Vector3 &measured, float dt);
Snapshot compute(const Snapshot &target, const Snapshot &measured, float dt);
void reset();
```

---

### `IBodyControlLayer`

Processes a single stage of the control cascade using a correction policy.

```cpp
IBodyControlLayer& setCorrectionPolicy(IBodyCorrectionPolicy*);
Snapshot process(const Snapshot &state, const Snapshot &target, float dt);
void reset();
```

---

### `IBodyController`

Orchestrates multiple `IBodyControlLayer`s and computes final torque using a rigid body model.

```cpp
IBodyController& addControlLayer(IBodyControlLayer*);
IBodyController& removeControlLayer(IBodyControlLayer*);
IBodyController& useModel(IRigidModel*);
Snapshot process(const Snapshot &state, const Snapshot &target, float dt);
```

---

## 🧩 Cascade Control Implementations

Default PID-based control layers, now Snapshot-aware.

### `BodyPIDPolicy`

```cpp
Vector3 compute(const Vector3 &target, const Vector3 &measured, float dt);
Snapshot compute(const Snapshot &target, const Snapshot &measured, float dt);
void reset();
```

- Angular wraparound on Z (yaw)  
- Tunable gains per axis  

---

### `BodyOrientationLayer`

Computes angular velocity from orientation error.

```cpp
Snapshot process(const Snapshot &state, const Snapshot &target, float dt);
```

---

### `BodyRateLayer`

Computes angular acceleration from angular velocity error.

```cpp
Snapshot process(const Snapshot &state, const Snapshot &target, float dt);
```

---

### `BodyController`

Manages layers and computes torque using the rigid body model.

```cpp
Snapshot process(const Snapshot &state, const Snapshot &target, float dt);
```

---

## 🛠 Hardware Layer

Hardware interfaces are now **isolated** in `/hardware`, including:  

- `mpu6050.h`  
- `esc.h`  
- Other sensor drivers  

> This ensures the **control logic stays hardware-independent**.  

---

## 🤖 Model Predictive Control (MPC) — Experimental

Basic MPC implementation is included in `/mpc`.  

```cpp
#include <Firmus/mpc/mpc.h>

MPC::BodyAttitudeMPCPolicy<4> mpc(Q, R, Qf);
Snapshot result = mpc.compute(frame, target, dt);
```

- Currently WIP, but functional for **multi-step attitude optimization**.  
- Works directly with `RigidModel` to compute torque.  

**Sample usage with MPU6050:**

```cpp
frame.orientation = Vector3(imu.pitch_rad, imu.roll_rad, imu.yaw_rad);
frame.angular_velocity = Vector3(imu.gx, imu.gy, imu.gz);

Snapshot result = mpc.compute(frame, target, dt);
result.torque = model4.computeRequiredTorque(frame.angular_velocity, result.angular_acceleration);
```

---

## 📚 Notes

- **All quantities are in SI units:** meters, m/s, radians, rad/s, Nm  
- **Reset policies** before reusing or switching control modes  
- **Control loop frequency** tracked manually via `Snapshot.timestamp` and `Snapshot.loop_freq`  

---

## 🧩 Upcoming Features

- Full MPC pipeline with constraints and prediction horizon  
- Neural Network correction policies  
- STM32 and native C++ support  
- Offline simulation and model introspection tools  
