/// One connected component of the contact + joint graph.
/// Bodies in different islands share no constraints and can be solved independently.
#[derive(Debug, Default)]
pub struct Island {
    /// Global body slot indices of all bodies in this island.
    pub bodies:   Vec<u32>,

    /// Indices into ContactPool::manifolds for all contacts in this island.
    pub contacts: Vec<usize>,

    /// JointHandles for all joints in this island.
    pub joints:   Vec<JointHandle>,

    /// True when every body in the island has been still long enough to sleep.
    pub is_sleeping: bool,

    /// Accumulator: how long all bodies have been below the sleep threshold.
    pub sleep_timer: f32,
}

use crate::JointHandle;

impl Island {
    pub fn new(bodies: Vec<u32>, contacts: Vec<usize>, joints: Vec<JointHandle>) -> Self {
        Self {
            bodies,
            contacts,
            joints,
            is_sleeping: false,
            sleep_timer: 0.0,
        }
    }
}