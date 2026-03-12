//! # Failsafe System
//! 
//! Monitors the health of the system and triggers emergency responses.

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum FailsafeLevel {
    None,       // All systems nominal
    // Warning,    // Low battery or weak signal (Alert only)
    Land,       // Critical failure, immediate controlled descent
    Kill,       // Total failure, stop motors immediately (Safety)
}

pub struct FailsafeMonitor {
    last_heartbeat: f32, // Timestamp of last valid input
    timeout_threshold: f32,
}

impl FailsafeMonitor {
    pub fn new(timeout: f32) -> Self {
        Self {
            last_heartbeat: 0.0,
            timeout_threshold: timeout,
        }
    }

    /// Evaluates system health based on current time.
    pub fn check(&self, current_time: f32) -> FailsafeLevel {
        if current_time - self.last_heartbeat > self.timeout_threshold {
            FailsafeLevel::Land
        } else {
            FailsafeLevel::None
        }
    }

    pub fn feed_heartbeat(&mut self, current_time: f32) {
        self.last_heartbeat = current_time;
    }
}