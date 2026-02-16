//! # Signal Processing and Sensor Fusion
//! 
//! This module provides the abstractions and implementations for real-time 
//! filtering. In a GNC (Guidance, Navigation, and Control) system, filters 
//! are used to mitigate sensor noise and combat integration drift.
//!
//! ### Design Pattern
//! We use a Trait-based approach to allow for static dispatch of various 
//! filter implementations. This ensures `no_std` compatibility and 
//! maximum performance on embedded hardware.

use crate::Radians;

pub mod complementary;

/// The core abstraction for all state estimators and signal conditioners.
///
/// A `Filter` takes a stream of noisy inputs and produces a high-fidelity 
/// output by maintaining an internal state across time steps (`dt`).
pub trait Filter {
    /// The raw sensor data or multi-sensor packet to be processed.
    type Input;
    /// The filtered estimate, usually a physical unit like `Radians`.
    type Output;

    /// Factory method to initialize a filter with a smoothing or trust coefficient.
    /// 
    /// ### Parameters
    /// * `alpha` - A coefficient usually between 0.0 and 1.0 that defines the 
    ///   filter's cutoff frequency or trust-weighting.
    fn new(alpha: f32) -> Self where Self: Sized;

    /// Advances the filter state by one time step.
    /// 
    /// ### Parameters
    /// * `input` - The latest measurement.
    /// * `dt` - The time elapsed since the last update in seconds.
    fn update(&mut self, input: Self::Input, dt: f32) -> Self::Output;

    /// Resets the internal state to a neutral default.
    /// 
    /// Critical for safety-critical systems to prevent "state carry-over" 
    /// between different flight phases or after a system fault.
    fn reset(&mut self);
}

// Re-export for ergonomic access: rust_gnc::filters::ComplementaryFilter
pub use complementary::ComplementaryFilter;

/// A standardized input structure for 6-DOF orientation filters.
/// 
/// In GNC, orientation estimation typically involves fusing two distinct signals:
/// 1. **Rate**: High-frequency, low-latency data that is integrated (e.g., Gyroscope).
/// 2. **Reference**: Low-frequency, stable data used to correct drift (e.g., Accelerometer).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InertialInput {
    /// The angular velocity (e.g., Rad/s from a Gyro).
    pub rate: Radians,      
    /// The absolute reference angle (e.g., Tilt from an Accelerometer).
    pub reference: Radians, 
}