//! # Navigation and State Estimation
//! 
//! This module defines the "Perception" layer of the flight stack. It aggregates 
//! data from various estimators to provide a high-fidelity representation of 
//! the vehicle's physical state.
//!
//! ### Coordinate System
//! All spatial data (Position, Velocity) is expressed in the **NED (North-East-Down)** //! frame. Attitudes are expressed as Tait-Bryan Euler angles.

use crate::{Attitude, units::{Position, Radians, Velocity}};

/// Represents the instantaneous rotational velocity of the aircraft.
/// 
/// These values represent the "Body Rate" of the vehicle, typically sourced 
/// directly from a filtered 3-axis Gyroscope.
#[derive(Debug, Clone, Copy, Default)]
pub struct AngularRate {
    /// Rotational velocity around the longitudinal axis (X).
    pub roll_rate: Radians,
    /// Rotational velocity around the lateral axis (Y).
    pub pitch_rate: Radians,
    /// Rotational velocity around the vertical axis (Z).
    pub yaw_rate: Radians,
}

/// The complete navigational state of the vehicle.
/// 
/// Known as the **Navigation Solution**, this struct is the "Single Source of Truth" 
/// for the Control and Guidance modules. It must be updated at a high frequency 
/// to ensure stability.
#[derive(Debug, Clone, Copy)]
pub struct DroneState {
    /// Current angular velocities, used primarily for D-term (Derivative) control.
    pub angular_rate: AngularRate,
    /// Current Euler angles, used for P-term (Proportional) attitude correction.
    pub attitude: Attitude,
    /// Linear velocity vector, essential for position hold and GPS missions.
    pub velocity: Velocity,
    /// 3D coordinates relative to the home point or global datum.
    pub position: Position,
}