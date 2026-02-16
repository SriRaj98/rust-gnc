//! # Complementary Filter
//! 
//! This module implements a first-order Complementary Filter, a computationally 
//! efficient sensor fusion algorithm used to estimate vehicle attitude.
//! 
//! ### Theory of Operation
//! The filter functions as a combination of a **High-Pass Filter** for the 
//! gyroscope and a **Low-Pass Filter** for the accelerometer:
//! 1. **Gyroscope integration** provides high-frequency tracking but suffers 
//!    from low-frequency bias/drift.
//! 2. **Accelerometer reference** provides stable low-frequency absolute 
//!    orientation but is susceptible to high-frequency vibration/noise.
use crate::filters::{Filter, InertialInput};
use crate::units::Radians;

/// A first-order Complementary Filter for attitude estimation.
/// 
/// The internal state represents the filtered angle, blending integrated 
/// angular rates with an absolute reference.
pub struct ComplementaryFilter {
    alpha: f32, 
    state: Radians,
}

impl ComplementaryFilter {
    /// Alpha is the trust coefficient for the gyroscope measurement. A value of 1.0 means we trust the gyro completely, while 0.0 means we trust the accelerometer completely.
    /// Typically set to 0.98 for flight controllers.
    pub fn new(alpha: f32) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            state: Radians(0.0),
        }
    }

    /// Dynamically updates the trust coefficient.
    /// 
    /// Useful for adaptive filtering where trust in the accelerometer 
    /// might decrease during high-G maneuvers.
    pub fn set_alpha(&mut self, alpha: f32) {
        self.alpha = alpha.clamp(0.0, 1.0);
    }
}

impl Filter for ComplementaryFilter {
    type Input = InertialInput; // (gyro_rate, accel_angle)
    type Output = Radians; // Estimated angle

    fn new(alpha: f32) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            state: Radians(0.0),
        }
    }

    /// Processes a new sensor sample to update the attitude estimate.
    /// 
    /// This method implements the discrete-time fusion equation:
    /// `θ = α * (θ + ω * dt) + (1 - α) * a`
    /// 
    /// ### Mathematical Steps
    /// 1. **Prediction**: Integrate the angular rate (gyro) into the current state.
    /// 2. **Correction**: Apply the reference angle (accel) to cancel integration drift.
    fn update(&mut self, input: Self::Input, dt: f32) -> Self::Output {
        let integrated_rate = self.state.0 + input.rate.0 * dt; // Integrate gyro rate to get angle
        let filtered_angle = self.alpha * integrated_rate + ((1.0 - self.alpha) * input.reference.0); // Blend with accelerometer angle

        self.state = Radians(filtered_angle);
        self.state
    }

    /// Resets the filter's internal state to zero.
    /// 
    /// Should be called during the 'Disarm' phase to ensure the next 
    /// takeoff sequence starts from a clean level-reference.
    fn reset(&mut self) {
        self.state = Radians(0.0);
    }
}