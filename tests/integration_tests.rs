use rust_gnc::control::{FailsafeMonitor, QuadMotorSignals};
use rust_gnc::control::mixer::QuadXMixer;
// tests/integration_tests.rs
use rust_gnc::units::{Radians, Attitude};
use rust_gnc::control::stabilizer::{Stabilizer, AttitudeController, ArmingState};
use rust_gnc::control::axis::AxisProcessor;
use rust_gnc::control::pid::{PidController, PidConfig};
use rust_gnc::filters::{ComplementaryFilter, InertialInput};

#[test]
fn test_safety_disarm_isolation() {
    // Setup a standard controller
    let mixer = QuadXMixer;
    let mut stabilizer = Stabilizer {
        arming_state: ArmingState::Disarmed,
        attitude_controller: AttitudeController {
            roll_processor: AxisProcessor::new(ComplementaryFilter::new(0.98), PidController::new(PidConfig { kp: 1.0, ki: 1.0, kd: 0.0, max_integral: 10.0 })),
            pitch_processor: AxisProcessor::new(ComplementaryFilter::new(0.98), PidController::new(PidConfig { kp: 1.0, ki: 1.0, kd: 0.0, max_integral: 10.0 })),
            yaw_processor: AxisProcessor::new(ComplementaryFilter::new(0.98), PidController::new(PidConfig { kp: 1.2, ki: 1.0, kd: 0.0, max_integral: 10.0 })),
        },
        mixer: mixer,
        failsafe_monitor: FailsafeMonitor::new(1.0)
    };

    // Simulate sensor input while disarmed
    let input = InertialInput { rate: Radians(1.0), reference: Radians(1.0) };
    let target = Attitude::default();
    
    let outputs = stabilizer.tick(input, input, input, target, 0.5, 0.01, 1.0);

    // Verify all motors are DEAD zero regardless of input or throttle
    assert_eq!(outputs.front_left, 0.0);
    assert_eq!(outputs.rear_right, 0.0);
}

#[test]
fn test_yaw_shortest_path_logic() {
    let start = Radians(3.1);   // ~177 degrees
    let target = Radians(-3.1);  // ~ -177 degrees
    
    let dist = start.shortest_distance_to(target);
    
    // We care that the distance is SMALL (0.08), not the large way around (6.2)
    assert!(dist.abs() < 0.1, "Distance was {}, expected a small value", dist);
    
    // In this specific case, 3.1 + 0.08 = 3.18. 
    // 3.18 normalized is -3.103. So the distance SHOULD be positive.
    assert!(dist > 0.0, "Shortest path should be positive (crossing the PI boundary)");
}

/// Helper to create a tuned stabilizer for testing
fn setup_test_stabilizer() -> Stabilizer<ComplementaryFilter, QuadXMixer> {
    let cfg = PidConfig { kp: 1.0, ki: 0.1, kd: 0.05, max_integral: 10.0 };
    Stabilizer::new(cfg.clone(), cfg.clone(), cfg.clone(), 0.98, QuadXMixer, 0.5)
}

#[test]
fn test_disarm_safety_guarantee() {
    let mut stabilizer = setup_test_stabilizer();
    stabilizer.set_armed(ArmingState::Disarmed);

    let input = InertialInput { rate: Radians(1.0), reference: Radians(1.0) };
    let target = Attitude::default();
    
    // Even with max throttle and high error, output must be 0.0
    let motors = stabilizer.tick(input, input, input, target, 1.0, 0.01, 1.0);
    
    assert_eq!(motors.front_left, 0.0);
    assert_eq!(motors.rear_right, 0.0);
}

#[test]
fn test_state_reset_on_rearm() {
    let mut stabilizer = setup_test_stabilizer();
    stabilizer.set_armed(ArmingState::Armed);
    
    // 1. Induce an integral windup
    let error = InertialInput { rate: Radians(0.0), reference: Radians(1.0) };
    for _ in 0..100 {
        stabilizer.tick(error, error, error, Attitude::default(), 0.5, 0.01, 1.0);
    }

    // 2. Disarm (This should trigger a reset)
    stabilizer.set_armed(ArmingState::Disarmed);
    
    // 3. Re-arm and check first tick
    stabilizer.set_armed(ArmingState::Armed);
    let motors = stabilizer.tick(error, error, error, Attitude::default(), 0.5, 0.01, 1.0);
    
    // If reset worked, the motor output should be based on P-term only, 
    // not the 100 iterations of I-term accumulation.
    assert!(motors.front_left < 1.0, "Integral was not reset on re-arming");
}

#[test]
fn test_armed_response_to_tilt() {
    let mut stabilizer = setup_test_stabilizer();
    
    stabilizer.set_armed(ArmingState::Armed);
    assert_eq!(stabilizer.arming_state, ArmingState::Armed);

    // Drone is tilted Right (Positive Roll). Target is Level (0.0).
    let roll_tilt = InertialInput { rate: Radians(0.0), reference: Radians(0.5) };
    let level = InertialInput { rate: Radians(0.0), reference: Radians(0.0) };
    let target = Attitude::default();

    let mut motors = QuadMotorSignals::default();
    for _ in 0..10 {
        motors = stabilizer.tick(roll_tilt, level, level, target, 0.5, 0.01, 0.1);
    }

    // If we are tilted Right, FL and RL motors must increase to push the left side up.
    assert!(motors.front_left > 0.5, "FL motor should be above base throttle (0.5)");
    assert!(motors.front_right < 0.5, "FR motor should be below base throttle (0.5)");
}

#[test]
fn test_failsafe_timeout_cuts_throttle() {
    let mut stabilizer = setup_test_stabilizer(); // Now includes FailsafeMonitor::new(0.5)
    stabilizer.set_armed(ArmingState::Armed);
    
    let input = InertialInput { rate: Radians(0.0), reference: Radians(0.0) };
    let target = Attitude::default();

    // 1. Initial heartbeat at T=0.0
    stabilizer.failsafe_monitor.feed_heartbeat(0.0);

    // 2. Tick at T=0.1 (Healthy)
    let motors = stabilizer.tick(input, input, input, target, 0.8, 0.01, 0.1);
    assert!(motors.front_left > 0.1); // Motor is spinning

    // 3. Tick at T=1.0 (Timed out! Threshold was 0.5)
    let motors_failsafe = stabilizer.tick(input, input, input, target, 0.8, 0.01, 1.0);
    
    // Throttle should be reduced/cut (depending on your Land logic)
    assert!(motors_failsafe.front_left < motors.front_left);
}