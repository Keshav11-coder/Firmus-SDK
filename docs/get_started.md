# Getting Started with Firmus SDK

Welcome to the **Firmus SDK** — a modular, high-performance flight control stack for drones and robotics, designed for clarity, composability, and real-world control. This guide will help you set up, test, and contribute to the SDK, whether you're a veteran or a new developer.

---

## In case you don't know —

Firmus is a modern, open-source flight control SDK for the ESP32 platform (Arduino environment), focused on:
- **Composable, layered control** (Attitude, Velocity, Position)
- **Physics-accurate models** and robust PID algorithms
- **Readable, flow-oriented C++ API**
- **Easy extensibility** for new aerial vehicles, sensors, and control strategies since it's based on frame-level control

> **Note:** Firmus is not a full autopilot stack (like PX4), but a flexible, modular control core for research, rapid prototyping, and advanced use. It's mainly focused on modelling and frame-level control.

---


## Suggested Hardware

The below image is an example of a testing unit, containing basic computing chips and sensors on a perfboard in a compact way.

![ESP32, MPU6050, BMP280 soldered onto a perf board](https://github.com/user-attachments/assets/ec9f5331-23cf-4b4b-a458-055016f137ea)

This is, of course, not required. The components can also simply be placed on a breadboard. The key components (most basic) you need are listed below.

- **MCU:** ESP32 DEVKIT V1 (recommended) (tested on ESP32 boards, v3.1.0 recommended)
- **IMU:** MPU6050 (I2C) -> there's full support for this line-up without requiring third-party libraries.

---

### Suggested Software
- Arduino IDE 1.8.x or 2.x, PlatformIO not available at the moment
- **ESP32 Board Support**
    - Install via Arduino Board Manager: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
    - Select version 3.1.0 for best compatibility
- **Firmus SDK** (this repo)
- **MPU6050 driver** (included in SDK)

> **Tip:** Other Arduino-compatible MCUs may work with adaptation, but ESP32 is the reference platform.

---

## Installation & Setup

While you can clone this repository via the terminal, the best way to add it to Arduino is by downloading it as a zip from GitHub.

### 1. Downlaod the zip from GitHub

### 2. Add to Arduino IDE
- Open Arduino IDE
- Go to **Sketch > Include Library > Add .ZIP Library...** or similar
- Restart Arduino IDE (optional, but often required for keyword highlighting)

### 3. Select Correct Board & Port
- **Tools > Board:** Select your ESP32 board (e.g., "ESP32 DEVKIT V1")
- **Tools > Port:** Select the correct COM port

### 4. Open Example Project
- Go to `Firmus-SDK/examples/BasicBodyAttitude/`
- Open `BasicBodyAttitude.ino`

or

- **File > Examples > Firmus > BasicBodyAttitude** in Arduino IDE

---

## Quickstart: Running the Example

1. **Wire up your hardware:**
    - Connect MPU6050 to ESP32 (I2C: default SDA=19, SCL=23 in example)
2. **Open `BasicBodyAttitude.ino` in Arduino IDE**
3. **Upload to your ESP32**
4. **Open Serial Monitor** (`115200 baud`)
5. **Observe output:**
    - Real-time motor PWM values
    - Control loop runs at ~50Hz or can be configured to ~100Hz

> **Note:** The example uses a cascaded PID controller (Orientation + Rate) and a quadrotor rigid model. Tuning parameters are in the sketch.

---

## Environment & Version Matrix

| Component         | Version      | Notes                        |
|-------------------|--------------|------------------------------|
| ESP32 Board Core  | 3.1.0        | Arduino Board Manager        |
| Arduino IDE       | 1.8.x / 2.x  | PlatformIO **NOT** supported |
| Firmus SDK        | 1.3.4        | See `README.md`              |
| MPU6050           | Built-in     | No extra library needed      |

> **Tip:** If you encounter build errors, check your ESP32 core version and board selection first.

---

## Extending & Contributing

### How to Contribute
- **Fork the repo** on GitHub
- **Create a feature branch**
- **Submit a pull request** with a clear description
- **Discuss on Discord or open an issue**

### Where to Start
- `src/` contains all core modules (controllers, models, mixers, assets)
- `examples/` for usage patterns
- `docs/` for architecture and control theory

### Templates
**Feature/bug PR template:**
```md
### What does this PR do?
- [ ] Feature
- [ ] Bugfix
- [ ] Refactor

#### Description
<!-- Briefly describe your changes -->

#### Testing
<!-- How did you test this? -->

#### Checklist
- [ ] Code compiles
- [ ] Example runs
- [ ] Docs updated (if needed)
```

---

## 📚 Further Reading
- [README.md](../README.md): Full architecture, API, and design notes
- [GitHub Repo](https://github.com/Keshav11-coder/Firmus-SDK)
- **GitHub Issues:** [Firmus-SDK Issues](https://github.com/Keshav11-coder/Firmus-SDK/issues)

> **Welcome!** Your feedback, bug reports, and contributions are highly valued.
