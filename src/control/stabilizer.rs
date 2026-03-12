//! # Flight Stabilization Orchestrator
//! 
//! This module implements the `Stabilizer`, the central coordinator of the flight 
//! control stack. It manages the lifecycle of the aircraft (Arming/Disarming) and 
//! executes the primary control loop (Guidance -> Navigation -> Control).
use crate::filters::{Filter, InertialInput};
use crate::control::{AxisProcessor, FailsafeLevel, FailsafeMonitor, Mixer, PidConfig, PidController};
use crate::units::{Radians, Attitude};

/// Defines the operational safety state of the vehicle.
#[derive(PartialEq, Debug)]
pub enum ArmingState {
    Disarmed,
    Armed,
}

/// A container for the three primary rotational control axes.
pub struct AttitudeController<F> where F: Filter<Input = InertialInput, Output = Radians> {
    pub roll_processor: AxisProcessor<F>,
    pub pitch_processor: AxisProcessor<F>,
    pub yaw_processor: AxisProcessor<F>,
}

/// The top-level interface for the flight control system.
/// 
/// The `Stabilizer` manages state estimation and PID calculation for all axes, 
/// eventually producing motor signals for the mixer.
pub struct Stabilizer<F, M>
    where 
        F: Filter<Input = InertialInput, Output = Radians>,
        M: Mixer 
{
    pub attitude_controller: AttitudeController<F>,
    pub arming_state: ArmingState,
    pub mixer: M,
    pub failsafe_monitor: FailsafeMonitor,
}

impl<F, M> Stabilizer<F, M> where 
    F: Filter<Input = InertialInput, Output = Radians>, 
    M: Mixer 
{
    pub fn new(
        roll_cfg: PidConfig,
        pitch_cfg: PidConfig,
        yaw_cfg: PidConfig,
        alpha: f32,
        mixer: M,
        failsafe_timeout: f32
    ) -> Self {
        Self {
            arming_state: ArmingState::Disarmed,
            attitude_controller: AttitudeController {
                roll_processor: AxisProcessor::new(F::new(alpha), PidController::new(roll_cfg)),
                pitch_processor: AxisProcessor::new(F::new(alpha), PidController::new(pitch_cfg)),
                yaw_processor: AxisProcessor::new(F::new(alpha), PidController::new(yaw_cfg)),
            },
            mixer: mixer,
            failsafe_monitor: FailsafeMonitor::new(failsafe_timeout),
        }
    }

    /// Transitions the flight state and handles critical hardware resets.
    /// 
    /// When disarming, all PID integrals and filters are cleared to prevent 
    /// "jump-on-arm" behavior caused by stale data accumulation (I-term windup).
    pub fn set_armed(&mut self, state: ArmingState) {
        self.arming_state = state;
        if self.arming_state == ArmingState::Disarmed {
            // CRITICAL: Reset all PID integrals and Filter states
            // This prevents "jump-on-arm" behavior.
            self.attitude_controller.pitch_processor.pid.reset();
            self.attitude_controller.roll_processor.pid.reset();
            self.attitude_controller.yaw_processor.pid.reset();
            
            self.attitude_controller.pitch_processor.filter.reset();
            self.attitude_controller.roll_processor.filter.reset();
            self.attitude_controller.yaw_processor.filter.reset();
        }
    }

    /// Executes a single iteration of the flight control loop.
    /// 
    /// This is the "Heartbeat" of the aircraft, typically running at 400Hz to 1kHz.
    /// It coordinates safety checks, sensor fusion, and actuator mixing.
    /// 
    /// ### Logic Flow
    /// 1. **Failsafe Evaluation**: Checks the system health/heartbeat against `current_time`.
    /// 2. **Command Override**: If a failsafe is active, pilot inputs are overridden with 
    ///    emergency values (e.g., leveling out and reducing throttle).
    /// 3. **Arming Interlock**: Ensures zero motor output if the aircraft is disarmed.
    /// 4. **Axis Processing**: Filters raw IMU data and calculates PID corrections for Roll/Pitch.
    /// 5. **Heading Control**: Calculates the shortest-path error for Yaw (circular normalization).
    /// 6. **Actuator Mixing**: Maps 3-axis demands and throttle into airframe-specific motor signals.
    ///
    /// ### Returns
    /// Returns `M::Output`, which for a Quadcopter is `QuadMotorSignals`.
    pub fn tick(
        &mut self, 
        roll_input: InertialInput, 
        pitch_input: InertialInput, 
        yaw_input: InertialInput,
        target: Attitude,
        throttle: f32,
        dt: f32,
        current_time: f32
    ) -> M:: Output {

        let failsafe_level = self.failsafe_monitor.check(current_time);

        let (effective_throttle, effective_target) = match failsafe_level {
            FailsafeLevel::None => (throttle, target),
            FailsafeLevel::Land => (throttle * 0.5, Attitude::default()), // 50% power & Level
            FailsafeLevel::Kill => (0.0, Attitude::default()),            // Cut power
        };

        // Safeguard: If disarmed, return zero motor outputs
        if self.arming_state == ArmingState::Disarmed {
            return self.mixer.mix(0.0, 0.0, 0.0, 0.0);
        }

        // Process Roll and Pitch axes
        let roll_output = self.attitude_controller.roll_processor.process(roll_input, effective_target.roll, dt);
        let pitch_output = self.attitude_controller.pitch_processor.process(pitch_input, effective_target.pitch, dt);
        
        let current_yaw = self.attitude_controller.yaw_processor.filter.update(yaw_input, dt);
        let yaw_error = current_yaw.shortest_distance_to(effective_target.yaw);
        let yaw_output = self.attitude_controller.yaw_processor.pid.update_with_error(yaw_error, dt);

        self.mixer.mix(roll_output, pitch_output, yaw_output, effective_throttle)
    }
}