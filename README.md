# Rust GNC (Guidance, Navigation, and Control)

A high-assurance, `no_std` library for flight control systems. Designed for performance, safety, and readability.

## Features
- **Type-Safe Physics:** Custom units for `Radians`, `Position`, and `Velocity` to prevent unit-mismatch bugs.
- **Modular Control:** Generic `AxisProcessor` allowing any `Filter` implementation (Complementary, Kalman, etc.).
- **Safety-First:** Built-in `ArmingState` to prevent integral windup and accidental motor spins.
- **Shortest-Path Yaw:** Automatic angle normalization for circular navigation.

## Architecture
The library is organized by physical responsibility:
- `units/`: Core physical primitives and operator overloading.
- `filters/`: Signal processing (Sensor Fusion).
- `control/`: PID loops and Motor Mixing.

## 🛠 Supported Configurations

* **Airframe:** Quadcopter (X-Configuration).
* **Mixing:** Standard 4-motor thrust distribution.
* **Coordinate System:** NED (North-East-Down) frame.

## Quick Start
```rust
let mut stabilizer = Stabilizer::new(...);
stabilizer.set_armed(ArmingState::Armed);
let motors = stabilizer.tick(roll_in, pitch_in, yaw_in, target, throttle, dt);
```

# 🚀 Roadmap

The development of **Rust GNC** (Guidance, Navigation, and Control) is divided into four strategic phases, moving from basic stability to autonomous intelligence.

---

### Phase 1: Mission Safety & Reliability
* **[ ] Failsafe System:** State-machine logic for Signal Loss (RX), Battery Low, and IMU Failure.
* **[ ] Emergency Protocols:** Automated "Land-in-Place" and "Return-to-Home" (RTH) routines.
* **[ ] Telemetry Logging:** High-speed, `no_std` compatible binary logging for post-flight black-box analysis.

### Phase 2: Advanced Control & Navigation
* **[ ] Acro (Rate) Mode:** A high-performance controller mapping inputs to angular velocity targets ($deg/s$).
* **[ ] Position Hold:** Implementation of a cascaded outer-loop for GPS-based coordinate maintenance.
* **[ ] Vertical Control:** Altitude hold using Barometer and Z-axis Accelerometer fusion.



### Phase 3: Signal Processing Suite
* **[ ] Extended Kalman Filter (EKF):** Sophisticated 6-DOF/9-DOF fusion to replace simple complementary filters.
* **[ ] Dynamic Notch Filters:** FFT-based vibration suppression to isolate motor noise from control signals.
* **[ ] Alternative Controllers:** Support for **LQR** (Linear Quadratic Regulator) and **Sliding Mode Control**.



### Phase 4: Intelligent Flight (AI/ML)
* **[ ] Neural Attitude Controller:** Augmenting PID loops with Reinforcement Learning (RL) agents for non-linear flight regimes.
* **[ ] System Identification:** Automated "Auto-Tune" utility to calculate vehicle moments of inertia and optimal gains.
* **[ ] Obstacle Avoidance:** Integration of LiDAR/ToF sensor arrays for autonomous path planning.