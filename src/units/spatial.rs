//! # Spatial Units and Kinematics
//! 
//! This module defines the 3D spatial primitives used for navigation.
//! 
//! ### Coordinate Convention
//! We use the **NED (North-East-Down)** convention:
//! - **X-axis**: Points North.
//! - **Y-axis**: Points East.
//! - **Z-axis**: Points Down (aligned with gravity).

use core::ops::{Add, Mul};
use crate::units::angular::Radians;

/// Represents linear velocity in 3D space.
/// Units are typically Meters per Second (m/s).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Velocity {
    /// Velocity component along the North axis.
    pub x: f32,
    /// Velocity component along the East axis.
    pub y: f32,
    /// Velocity component along the Down axis (positive is descending).
    pub z: f32,
}

impl Mul<f32> for Velocity {
    type Output = Self;

    /// Scales velocity by a scalar value. 
    /// Commonly used to calculate displacement (Velocity * dt).
    fn mul(self, rhs: f32) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

/// Represents a 3D position in the NED frame.
/// Units are typically Meters (m).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Position {
    /// Distance North of the origin.
    pub x: f32,
    /// Distance East of the origin.
    pub y: f32,
    /// Altitude relative to origin (positive is below the origin/sea level).
    pub z: f32,
}

impl Add<Velocity> for Position {
    type Output = Self;

    /// Adds a velocity vector (scaled by time) to a position.
    /// Implements the basic kinematic equation: P_new = P_old + V.
    fn add(self, rhs: Velocity) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl Position {
    /// Performs a first-order Euler integration to predict the next position.
    /// 
    /// ### Arguments
    /// * `velocity` - The current velocity vector.
    /// * `dt` - The time delta (seconds).
    pub fn predict_next(&self, velocity: Velocity, dt: f32) -> Self {
        *self +velocity * dt
    }
}

/// Represents the aircraft orientation using Euler angles.
/// 
/// Follows the standard Tait-Bryan convention (Z-Y-X).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Attitude {
    /// Rotation about the X-axis (longitudinal).
    pub roll: Radians,
    /// Rotation about the Y-axis (lateral).
    pub pitch: Radians,
    /// Rotation about the Z-axis (vertical/heading).
    pub yaw: Radians,
}

impl Default for Attitude {
    /// Returns a level attitude (zeroed on all axes).
    fn default() -> Self {
        Self {
            roll: Radians(0.0),
            pitch: Radians(0.0),
            yaw: Radians(0.0),
        }
    }
}