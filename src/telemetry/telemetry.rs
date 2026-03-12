//! # Telemetry
//! 
//! Handles the serialization and transmission of flight data snapshots.
//! Optimized for high-frequency binary logging in `no_std` environments.

use crate::{Attitude, Position};
use crate::control::mixer::QuadMotorSignals;

/// A packed, fixed-size snapshot of the vehicle state.
/// 
/// ### Binary Layout
/// This struct uses `#[repr(C, packed)]` to ensure a stable, predictable memory layout
/// across different platforms. It is exactly 37 bytes (36 for data + 1 for flags).
/// 
/// | Field | Type | Offset |
/// | :--- | :--- | :--- |
/// | `timestamp_ms` | `u32` | 0 |
/// | `roll`..`pos_z` | `f32` x 4 | 4-19 |
/// | `motor_fl`..`rr` | `f32` x 4 | 20-35 |
/// | `status_flags` | `u8` | 36 |
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct TelemetryPacket {
    /// System uptime in milliseconds.
    pub timestamp_ms: u32,
    /// Current Euler angles (Radians).
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    /// Vertical position (Altitude) in meters (NED frame).
    pub pos_z: f32,
    /// Normalized motor output [0.0 - 1.0].
    pub motor_fl: f32,
    pub motor_fr: f32,
    pub motor_rl: f32,
    pub motor_rr: f32,
    /// Bitmask for system status:
    /// - Bit 0: Armed state (1 = Armed, 0 = Disarmed)
    /// - Bit 1: Failsafe active
    /// - Bit 2: Battery critical
    pub status_flags: u8, 
}

impl TelemetryPacket {
    /// Constructs a new telemetry snapshot from high-level GNC types.
    pub fn new(
        timestamp_ms: u32,
        attitude: Attitude,
        pos: Position,
        motors: QuadMotorSignals,
        armed: bool
    ) -> Self {
        Self {
            timestamp_ms,
            roll: attitude.roll.0,
            pitch: attitude.pitch.0,
            yaw: attitude.yaw.0,
            pos_z: pos.z,
            motor_fl: motors.front_left,
            motor_fr: motors.front_right,
            motor_rl: motors.rear_left,
            motor_rr: motors.rear_right,
            status_flags: if armed { 1 } else { 0 },
        }
    }

    /// Views the packet as a byte slice for transmission.
    /// 
    /// # Safety
    /// This uses `unsafe` to cast the struct directly to a byte slice. 
    /// This is safe as long as:
    /// 1. The struct is `#[repr(C, packed)]`.
    /// 2. The slice does not outlive the struct instance.
    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                (self as *const Self) as *const u8,
                core::mem::size_of::<Self>(),
            )
        }
    }
}