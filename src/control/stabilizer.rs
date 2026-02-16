//! # Flight Stabilization Orchestrator
//! 
//! This module implements the `Stabilizer`, the central coordinator of the flight 
//! control stack. It manages the lifecycle of the aircraft (Arming/Disarming) and 
//! executes the primary control loop (Guidance -> Navigation -> Control).
use crate::filters::{Filter, InertialInput};
use crate::control::{AxisProcessor, MotorMixer, MotorSignals, PidConfig, PidController};
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
pub struct Stabilizer<F> where F: Filter<Input = InertialInput, Output = Radians> {
    pub attitude_controller: AttitudeController<F>,
    pub arming_state: ArmingState,
}

impl<F> Stabilizer<F> where F: Filter<Input = InertialInput, Output = Radians> {

    pub fn new(
        roll_cfg: PidConfig,
        pitch_cfg: PidConfig,
        yaw_cfg: PidConfig,
        alpha: f32
    ) -> Self {
        Self {
            arming_state: ArmingState::Disarmed,
            attitude_controller: AttitudeController {
                roll_processor: AxisProcessor::new(F::new(alpha), PidController::new(roll_cfg)),
                pitch_processor: AxisProcessor::new(F::new(alpha), PidController::new(pitch_cfg)),
                yaw_processor: AxisProcessor::new(F::new(alpha), PidController::new(yaw_cfg)),
            },
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
    /// 
    /// ### Logic Flow
    /// 1. **Safety Check**: Verify arming status.
    /// 2. **Axis Processing**: Filter sensor data and update PIDs for Roll and Pitch.
    /// 3. **Yaw Normalization**: Handle circular wrap-around for heading control.
    /// 4. **Mixing**: Combine axis demands with throttle for final motor signals.
    pub fn tick(
        &mut self, 
        roll_input: InertialInput, 
        pitch_input: InertialInput, 
        yaw_input: InertialInput,
        target: Attitude,
        throttle: f32,
        dt: f32
    ) -> MotorSignals {

        // Safeguard: If disarmed, return zero motor outputs
        if self.arming_state == ArmingState::Disarmed {
            return MotorSignals::default();
        }

        // Process Roll and Pitch axes
        let roll_output = self.attitude_controller.roll_processor.process(roll_input, target.roll, dt);
        let pitch_output = self.attitude_controller.pitch_processor.process(pitch_input, target.pitch, dt);
        
        let current_yaw = self.attitude_controller.yaw_processor.filter.update(yaw_input, dt);
        let yaw_error = current_yaw.shortest_distance_to(target.yaw);
        let yaw_output = self.attitude_controller.yaw_processor.pid.update_with_error(yaw_error, dt);

        MotorMixer::mix(roll_output, pitch_output, yaw_output, throttle)
    }
}