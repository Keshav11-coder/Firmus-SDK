# Firmus-SDK

**Version:** `v1.3.4`  
**Recommended Board:** ESP32 v3.1.0

---

## Overview

Firmus is a high-performance, modular flight control SDK built around **clarity**, **composability**, and **physics-accurate modeling**. It offers a clean interface for building both simple cascaded controllers and complex high-level control architectures.

---

## Key Principles

- **Modular Control Layers**  
  Each controller layer is swappable and composable.

- **Frame-Level Operation**  
  All quantities are defined in the body (local) reference frame.

- **Deterministic Abstractions**  
  Predictable interfaces, no hidden state.

- **Hardware-Agnostic**  
  Runs on Arduino ESP32, with future support for STM32 and native C++.

---

## Architecture Summary

```
[ Sensor Array ]
      ↓
[ Rigid Model ] 
      ↓
[ Control Layers: Orientation → Rate → ... ]
      ↓
[ Final Torque Computation via Model ]
      ↓
[ Output (e.g., Mixer, Actuators) ]
```

---

## Core Interfaces

### `Vector3`

3D vector structure for all spatial and angular quantities (position, velocity, orientation, torque, etc).

```cpp
struct Vector3 {
    float x, y, z;
};
```

### `Snapshot`

Encapsulates the complete dynamic state of a body at a given moment. Plays a key role in flow-of-data.

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

## Modern Control Stack (1.3.4+)

### `IBodyCorrectionPolicy`

Defines the contract for computing corrections between a measured and target vector.

```cpp
Vector3 compute(const Vector3 &target, const Vector3 &measured, float dt);
void reset();
```

**Examples:** PID, MPC, LQR, NN.

---

### `IBodyControlLayer`

Processes one stage of the control cascade. Applies a correction strategy to produce a new control target.

```cpp
IBodyControlLayer& setCorrectionPolicy(IBodyCorrectionPolicy*);
Snapshot process(const Snapshot &state, const Snapshot &target, float dt);
void reset();
```

---

### `IBodyController`

Orchestrates a stack of `IBodyControlLayer`s and computes final torque using a rigid body model.

```cpp
IBodyController& addControlLayer(IBodyControlLayer*);
IBodyController& removeControlLayer(IBodyControlLayer*);
IBodyController& useModel(IRigidModel*);
Snapshot process(const Snapshot &state, const Snapshot &target, float dt);
```

---

## Cascade Control Implementation

These implementations provide a default control pipeline using PID-based correction.

### `BodyPIDPolicy`

Implements PID correction on 3 axes independently.

```cpp
Vector3 compute(const Vector3 &target, const Vector3 &measured, float dt);
void reset();
```

- Uses angular wraparound on Z (yaw).
- Tunable gains per axis.
- Stateless except for internal PID memory.

---

### `BodyOrientationLayer`

Computes angular velocity from orientation error.

```cpp
Snapshot process(const Snapshot &state, const Snapshot &target, float dt);
```

- Assumes orientation vectors in radians.
- Outputs angular velocity in `Snapshot.angular_velocity`.

---

### `BodyRateLayer`

Computes angular acceleration from angular velocity error.

```cpp
Snapshot process(const Snapshot &state, const Snapshot &target, float dt);
```

- Assumes inputs are in `rad/s`.
- Outputs angular acceleration in `Snapshot.angular_acceleration`.

---

### `BodyController`

Manages control layers and computes torque using the model.

```cpp
Snapshot process(const Snapshot &state, const Snapshot &target, float dt);
```

- Applies layers in order.
- Uses `IRigidModel` to compute torque from angular acceleration.

---

## Legacy Interfaces (Deprecated)

The following are still supported but discouraged:

- `IAttitude`
- `IVelocity`
- `IPosition`
- `IOrchestrator`

Use the new cascade-based structure instead.

---

## Notes

- **All quantities are in SI units**:  
  - Position: meters  
  - Velocity: m/s  
  - Orientation: radians  
  - Angular rates: rad/s  
  - Torque: Nm

- **Reset policies** before reusing or switching control modes.

- **Control loop frequency** should be tracked manually using `timestamp` and `loop_freq` in `Snapshot`.

---

## Upcoming Features

- Template implementations for LQR, MPC, and Neural Network controllers.
- STM32 and native C++ support.
- Full model introspection and offline simulation tools.
