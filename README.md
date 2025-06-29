# Firmus-SDK

**Current stable version: v1.3.3**

**Runs optimally (and tested) on esp32 boards v3.1.0**

---

## 📘 Introduction

**Firmus** is a high-performance, modular flight control SDK designed with clarity and composability in mind. It was born out of frustration with many popular drone SDKs — tools that offer beautiful syntax and developer-friendly APIs, yet are weighed down by bloated, opaque, and often inefficient internals.

Firmus addresses these shortcomings by offering:

- A fully **composable control stack**, where each layer is modular and interchangeable  
- **Strict but expressive control templates** that promote robust, deterministic control  
- **Readable, flow-oriented syntax** that feels intuitive for developers  
- Support for both experienced control engineers and those who "just want to fly"  
- **Physics-accurate models and algorithms** grounded in real-world drone dynamics  

Firmus is currently built for the **ESP32 platform**, leveraging the Arduino environment for development. Porting to other Arduino-compatible targets (e.g. AVR) is possible with some adaptation to the internals.

Whether you're building experimental flight software, writing your own controller, or just need a clean and flexible starting point, **Firmus** is designed to help you focus on control — not complexity.

## 🏗️ Architecture Overview

A High-performance and robust control stack requires a careful set of abstract module layers that reflect the fundamentals of control theory. Below is a visualization of the most basic abstract control flow.

```
                                                          [Position]
                   [ Sensor Array ]                      /          \
[ Sketch (.ino) ] -------------------> [ Orchestrator ] — [Velocity] — -> [ Mixer ] -> [ Output ]
                   [ Rigid Model ]                       \          /
                                                          [Attitude]
```

Core interfaces:
- `IRigidModel` (vehicle dynamics)
- `IOrchestrator`, `IPosition`, `IVelocity`, `IAttitude` (control layers)

Implementations of interfaces:
- `RigidModel<N>` (vehicle model by actuators)
- `Cascade::Orchestrator`, `Position`, `Velocity`, `Attitude` (Cascaded by orchestrator, full filtered PID implementation, rigid body dynamics)

Other useful assets:
- `Mixer<N>` (optional, needs to be combined with the rigid model)
- `mpu6050` and some other built-in, extensive sensors
- `Matrix2`, `matrix<T, r, c>` along with some allocation utilities — custom, lightweight and robust matrix library. Used a lot by implementations, but not mandatory
- `Vector3{}` (mandatory, universal spatial vector, used and returned by all abstract systems)
- `PIDGains{}` (mandatory for PID configuration)

## 🧩 Core SDK Overview
The Firmus SDK is built on composable and deterministic abstractions. These base interfaces represent the “contract” between major modules and enable layered, flexible control. All interfaces share similar semantics for consistent usage and development.

Each section below describes briefly:
- **What the iterface/module does**
- **Its standard public methods**
- **Implementation notes**
- **Data conventions and units**





### **`IOrchestrator` (Controller/Orchestrator Interface)**
**Purpose:**
Computes the desired torque from an interchangeable control set. Combinations can be limited from here. All computations converge toward this point.

**Key Methods:**
```cpp
void useModel(IRigidModel*);
void usePositionInterface(IPosition*);
void useVelocityInterface(IVelocity*);
void useAttitudeInterface(IAttitude*);

Vector3 compute(float dt);
```

**Conventions/Units:**
- Returns torques in *Nm*

**Notes:**
- The order of controllers can be programmatically changed per orchestrator. The converging properties as well.







### **`IAttitude` (Attitude Controller Interface)**
**Purpose:**
Computes the desired angular acceleration for a given attitude.

**Key Methods:**
```cpp
void configureOrientationGains(PIDGains, PIDGains, PIDGains);
void configureRateGains(PIDGains, PIDGains, PIDGains);
void setControlFlags(IAttitudeFlags*);

void updateMeasuredOrientation(float x, float y, float z);
void updateMeasuredRate(float x, float y, float z);
void configureOrientationTarget(float x, float y, float z);
void configureRateTarget(float x, float y, float z);

Vector3 compute(float dt);
```

**Conventions/Units:**
- All angles are in *radians*
- Rate inputs are in *rad/s*
- When returning torque: `Vector3` in *Nm*
- When returning angular accelerations: `Vector3` in *rad/s^2*

**Notes:**
- The typical attitude controller should not compute the torque directly, which is why it is promoted to compute the angular accelerations instead, and compute the final torque via the model. This is to maintain structural integrity, because a controller may not have two major priorities.








### **`IVelocity` (Velocity Controller Interface)**
**Purpose:**
Computes the required acceleration (Vector3) from a velocity target and current (measured) velocity.

**Key Methods:**
```cpp
void configureVelocityGains(PIDGains, PIDGains, PIDGains);
void setControlFlags(IVelocityFlags*);

void updateMeasuredVelocity(float x, float y, float z);
void configureVelocityTarget(float x, float y, float z);

Vector3 compute(float dt);
```

**Conventions/Units:**
- Velocity in *m/s*
- Acceleration in *m/s^2*
- Returns **frame-aligned acceleration vector**

### **`IPosition` (Position Controller Interface)**
**Purpose:**
Computes the required velocity (Vector3) from a position target and current (estimated and measured) position.

**Key Methods:**
```cpp
void configurePositionGains(PIDGains, PIDGains, PIDGains);
void setControlFlags(IPositionFlags*);

void updateMeasuredPosition(float x, float y, float z);
void configurePositionTarget(float x, float y, float z);

Vector3 compute(float dt);
```

**Conventions/Units:**
- Position in *meters*
- Velocity in *m/s*
- Returns **frame-aligned velocity vector**
