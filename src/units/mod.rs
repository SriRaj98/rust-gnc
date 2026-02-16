//! # Units of Measurement
//! 
//! This module provides a type-safe foundation for Guidance, Navigation, and Control (GNC).
//! 
//! ### Design Philosophy
//! By wrapping raw `f32` values in domain-specific types, we leverage the Rust compiler 
//! to prevent common physical unit mismatch errors, such as:
//! - Adding a `Velocity` to a `Radians` value.
//! - Mixing `Degrees` and `Radians` in trigonometric calculations.
//! - Inverting the Z-axis (Altitude) due to coordinate system confusion.
//!
//! ### Submodules
//! - **Angular**: Handles circular math, normalization, and shortest-path logic.
//! - **Spatial**: Handles 3D kinematics in the NED (North-East-Down) frame.
pub mod angular;
pub mod spatial;

// Re-exports: Making the most commonly used types available at the crate root.
// This follows the "Facade Pattern" to keep the public API clean and ergonomic.

#[doc(inline)]
pub use angular::{Degrees, Radians};

#[doc(inline)]
pub use spatial::{Position, Velocity, Attitude};