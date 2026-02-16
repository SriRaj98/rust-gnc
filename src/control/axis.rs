//! # Axis Processing
//! 
//! This module implements the `AxisProcessor`, which encapsulates the full 
//! control pipeline for a single degree of freedom (e.g., Roll, Pitch, or Yaw).
//!
//! By combining a `Filter` and a `PidController`, the `AxisProcessor` transforms 
//! raw inertial data into a normalized control signal.

use crate::filters::{Filter, InertialInput};
use crate::control::pid::PidController;
use crate::units::Radians;

/// A generic controller for a single physical axis.
/// 
/// The `AxisProcessor` is generic over any `Filter` that accepts `InertialInput` 
/// and produces `Radians`. This allows the same control logic to be used with 
/// Complementary filters, Kalman filters, or simple low-pass filters.
pub struct AxisProcessor<F> 
where 
    F: Filter<Input = InertialInput, Output = Radians> 
{
    /// The PID algorithm responsible for generating the corrective signal.
    pub pid: PidController,
    /// The sensor fusion filter responsible for estimating the current orientation.
    pub filter: F,
}

impl<F> AxisProcessor<F> 
where 
    F: Filter<Input = InertialInput, Output = Radians> 
{
    /// Creates a new `AxisProcessor` with the provided estimation and control components.
    pub fn new(filter: F, pid: PidController) -> Self {
        Self { pid, filter }
    }

    /// Executes one iteration of the control loop for this axis.
    /// 
    /// This method performs two primary steps:
    /// 1. **State Estimation**: Updates the internal filter with new IMU data.
    /// 2. **Control Generation**: Computes the PID output based on the error 
    ///    between the `target_angle` and the estimated `current_angle`.
    /// 
    /// ### Parameters
    /// * `input` - Raw rate and reference data from sensors.
    /// * `target_angle` - The desired orientation for this axis.
    /// * `dt` - Time delta since the last update in seconds.
    /// 
    /// ### Returns
    /// A normalized control signal (f32), typically in the range [-1.0, 1.0].
    pub fn process(&mut self, input: InertialInput, target_angle: Radians, dt: f32) -> f32 {
        // Step 1: Perception (Where are we?)
        let current_angle = self.filter.update(input, dt);
        
        // Step 2: Control (How do we get to where we want to be?)
        self.pid.update(target_angle.0, current_angle.0, dt)
    }
}