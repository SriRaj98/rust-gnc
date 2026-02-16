//! # Angular Units
//! 
//! This module provides type-safe representations for angular measurements.
//! It handles the circular logic required for navigation, specifically 
//! the transition across the ±π (180°) boundary.

/// A type-safe wrapper for angular measurements in radians.
/// 
/// Range: Usually normalized to (-π, π] for flight dynamics.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Radians(pub f32);

impl Radians {
    /// Normalizes the angle to the range (-π, π].
    /// 
    /// This is essential for preventing "the long way around" maneuvers 
    /// and ensuring the PID controller receives the smallest possible error.
    /// 
    /// ### Performance
    /// This implementation uses a deterministic approach to ensure consistent 
    /// execution time in real-time flight loops.
    pub fn normalize(&self) -> Self {
        let pi = core::f32::consts::PI;
        let two_pi = 2.0 * pi;

        // Industry Standard: Use a non-looping normalization for O(1) performance.
        // This prevents timing jitter on microcontrollers if the input is very large.
        let mut angle = self.0;
        if angle <= -pi || angle > pi {
            angle = angle - two_pi * libm::floorf((angle + pi) / two_pi);
        }
        Radians(angle)
     }

    /// Calculates the shortest angular distance to a target.
    /// 
    /// Returns a value in the range (-π, π]. A positive result indicates 
    /// a clockwise rotation, while a negative result indicates counter-clockwise.
    /// 
    /// ### Example
    /// Moving from 179° to -179° will return a distance of 2° (0.035 rad) 
    /// instead of -358°.
     pub fn shortest_distance_to(&self, target: Radians) -> f32 {
        let delta = target.0 - self.0;
        Radians(delta).normalize().0
     }
}

/// A type-safe wrapper for angular measurements in degrees.
/// 
/// Primarily used for human-readable telemetry, logging, and configuration.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Degrees(pub f32);

impl From<Radians> for Degrees {
    /// Converts Radians to Degrees using the standard constant π.
    fn from(radians: Radians) -> Self {
        Degrees(radians.0 * 180.0 / core::f32::consts::PI)
    }
}

impl From<Degrees> for Radians {
    /// Converts Degrees to Radians. Used for ingesting user configuration 
    /// into the flight-ready physics engine.
    fn from(degrees: Degrees) -> Self {
        Radians(degrees.0 * core::f32::consts::PI / 180.0)
    }
}

#[cfg(test)]
mod tests {
  use super::*;

    #[test]
    fn test_normalization_boundaries() {
        // Test wrap-around at PI
        assert!((Radians(3.2).normalize().0 - (-3.0831)).abs() < 0.001);
        // Test wrap-around at -PI
        assert!((Radians(-3.2).normalize().0 - 3.0831).abs() < 0.001);
        // Test identity
        assert_eq!(Radians(1.0).normalize().0, 1.0);
    }
}