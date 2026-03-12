//! # Control Systems
//! 
//! This module implements the "Action" phase of the GNC (Guidance, Navigation, 
//! and Control) loop. It consumes the navigation solution and outputs 
//! motor-level actuation commands.
//!
//! ### Architecture
//! - **PID**: The fundamental control algorithm for error correction.
//! - **Axis**: Encapsulates a single degree of freedom (Perception + Control).
//! - **Stabilizer**: Orchestrates multiple axes into a coherent flight state.
//! - **Mixer**: Maps abstract demands (Roll, Pitch, Yaw) to physical motor signals.

pub mod pid;
pub mod mixer;
pub mod axis;
pub mod stabilizer;
pub mod failsafe;

// Re-exports: Providing a clean, flattened API for external crate users.
// This allows for ergonomic usage like `use rust_gnc::control::PidController`.

#[doc(inline)]
pub use pid::{PidController, PidConfig};

#[doc(inline)]
pub use mixer::{Mixer, QuadMotorSignals, QuadXMixer};

#[doc(inline)]
pub use axis::AxisProcessor;

#[doc(inline)]
pub use stabilizer::{Stabilizer, AttitudeController, ArmingState};

#[doc(inline)]
pub use failsafe::{FailsafeMonitor, FailsafeLevel};