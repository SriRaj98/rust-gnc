//! # PID Control Logic
//! 
//! This module implements a standard Proportional-Integral-Derivative (PID) 
//! controller with integrated safety features for real-time systems.
//!
//! ### Implementation Details
//! - **Parallel Form**: The output is the sum of $P$, $I$, and $D$ components.
//! - **Anti-Windup**: Clamping is applied to the integral accumulator to 
//!   prevent saturation during prolonged error states.
//! - **Time Step Robustness**: Protects against $dt \le 0$ to ensure 
//!   mathematical stability on embedded systems.

/// Configuration parameters for a PID controller.
/// 
/// These gains determine the responsiveness and stability of the system.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidConfig {
    /// Proportional gain ($K_p$). Immediate correction based on current error.
    pub kp: f32,
    /// Integral gain ($K_i$). Corrects for steady-state error over time.
    pub ki: f32,
    /// Derivative gain ($K_d$). Dampens oscillations by reacting to error rate.
    pub kd: f32, 
    /// Maximum absolute value for the integral accumulator (Anti-windup).
    pub max_integral: f32, 
}

/// A stateful PID controller.
pub struct PidController {
    config: PidConfig,
    /// The accumulated error sum (Integral term).
    integral: f32,
    /// The error value from the previous time step (for Derivative calculation).
    last_error: f32,
}

/// A stateful PID controller.
impl PidController {
    /// Initializes a new controller with the provided gains and zeroed state.
    pub fn new(config: PidConfig) -> Self {
        Self {
            config,
            integral: 0.0,
            last_error: 0.0,
        }
    }

    /// Calculates the corrective signal based on a setpoint and measurement.
    /// 
    /// ### Parameters
    /// * `setpoint` - The desired value (Target).
    /// * `measurement` - The current estimated value (Feedback).
    /// * `dt` - Time delta since the last update in seconds.
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;
        self.update_with_error(error, dt)
    }

    /// Primary calculation engine for the PID signal.
    /// 
    /// This method performs the following:
    /// 1. **Proportional**: $K_p \times error$
    /// 2. **Integral**: Accumulates $error \times dt$, clamped by `max_integral`.
    /// 3. **Derivative**: $K_d \times \frac{\Delta error}{dt}$
    pub fn update_with_error(&mut self, error: f32, dt: f32) -> f32 {

        let proportional = self.config.kp * error;

        if dt <= 0.0 {
            return proportional; // Avoid division by zero or negative time step
        }

        // Integral Term with Anti-Windup Clamping
        // This prevents the drone from "overshooting" aggressively after being held.
        self.integral += error * dt;
        self.integral = self.integral.clamp(-self.config.max_integral, self.config.max_integral);
        let integral = self.config.ki * self.integral;

        let derivative = self.config.kd * (error - self.last_error) / dt;

        self.last_error = error;

        proportional + integral + derivative
    }

    /// Resets the controller's internal memory (Integral and Last Error).
    /// 
    /// **CRITICAL**: This should be called whenever the controller is 
    /// re-engaged (Armed) to prevent "jumpy" initial behavior from stale data.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_pid_anti_windup() {
        let config = PidConfig { kp: 1.0, ki: 1.0, kd: 0.0, max_integral: 5.0 };
        let mut pid = PidController::new(config);

        // Simulate a massive error over a long time (50 seconds)
        // Without clamping, integral would be 50.0. With clamping, it should be 5.0.
        for _ in 0..50 {
            pid.update(10.0, 0.0, 1.0);
        }
        
        // Final output should be P (10) + I (5) = 15.0
        let output = pid.update(10.0, 0.0, 1.0);
        assert_eq!(output, 15.0);
    }
}