//! # Rust GNC
//! 
//! A high-performance, `no_std` Guidance, Navigation, and Control library 
//! designed for unmanned aerial vehicles.

#![no_std]

/// Physical unit abstractions to ensure type-safety (e.g., preventing Degrees vs Radians errors).
pub mod units;

/// Signal processing tools including Complementary, Kalman, and Notch filters.
pub mod filters;

/// Flight control laws ranging from PID loops to LQR and Sliding Mode Control.
pub mod control;

/// High-speed binary logging for post-flight analysis.
pub mod telemetry;

/// Global and local coordinate estimation and GPS fusion logic.
pub mod navigation;

// Re-exports for a cleaner API at the crate root
pub use units::angular::Radians;
pub use units::spatial::{Attitude, Position, Velocity};