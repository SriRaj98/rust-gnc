//! # Motor Mixing
//! 
//! This module implements the "Mixer" logic, which maps normalized control 
//! signals from the PID controllers to individual motor output levels.
//! 
//! This module defines the interface for mapping control demands (Roll, Pitch, Yaw, Throttle)
//! to physical actuator outputs.

/// Common interface for different airframe geometries.
pub trait Mixer {
    /// The specific signal type produced by this mixer (e.g., 4 signals for a quad).
    type Output;

    /// Maps normalized control demands [-1.0, 1.0] and throttle [0.0, 1.0] to motor signals.
    fn mix(&self, roll: f32, pitch: f32, yaw: f32, throttle: f32) -> Self::Output;
}

/// A collection of normalized pulse-width modulation (PWM) signals for 
/// a quadcopter's Electronic Speed Controllers (ESCs).
/// 
/// Range: 0.0 (Stopped) to 1.0 (Full Power).
#[derive(Debug, PartialEq, Clone, Copy, Default)]
pub struct QuadMotorSignals {
    /// Signal for the Front-Left motor.
    pub front_left: f32,
    /// Signal for the Front-Right motor.
    pub front_right: f32,
    /// Signal for the Rear-Left motor.
    pub rear_left: f32,
    /// Signal for the Rear-Right motor.
    pub rear_right: f32,
}

/// A stateless utility for calculating motor power distribution.
pub struct QuadXMixer;

impl Mixer for QuadXMixer {
    type Output = QuadMotorSignals;

    /// Mixes axis control signals and throttle into individual motor outputs.
    /// 
    /// This follows the standard Quad-X mixing matrix. The signs (+/-) for each 
    /// component are determined by the motor's position relative to the 
    /// center of mass and its rotational direction.
    /// 
    /// ### Parameters
    /// * `roll` - Normalized roll correction [-1.0, 1.0].
    /// * `pitch` - Normalized pitch correction [-1.0, 1.0].
    /// * `yaw` - Normalized yaw correction [-1.0, 1.0].
    /// * `throttle` - Base power level [0.0, 1.0].
    /// 
    /// ### Returns
    /// A `MotorSignals` struct with values clamped to the safe operating range [0.0, 1.0].
    fn mix(&self, roll: f32, pitch: f32, yaw: f32, throttle: f32) -> Self::Output {
        // Mixing Matrix Calculation:
        // FL = Throttle + Roll + Pitch - Yaw
        // FR = Throttle - Roll + Pitch + Yaw
        // RL = Throttle + Roll - Pitch + Yaw
        // RR = Throttle - Roll - Pitch - Yaw
        
        let fl = throttle - roll - pitch - yaw;
        let fr = throttle + roll - pitch + yaw;
        let rl = throttle - roll + pitch + yaw;
        let rr = throttle + roll + pitch - yaw;

        QuadMotorSignals {
            front_left: fl.clamp(0.0, 1.0),
            front_right: fr.clamp(0.0, 1.0),
            rear_left: rl.clamp(0.0, 1.0),
            rear_right: rr.clamp(0.0, 1.0),
        }
    }
}