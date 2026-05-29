use physics_math::Vec2;
use crate::body::BodyStorage;

#[derive(Debug, Clone, Copy, Default)]
pub struct BodyPosition {
    pub position: Vec2,
    pub angle: f32,
    pub inv_mass: f32,
    pub inv_inertia: f32,
}

impl BodyPosition {
    pub fn from_storage(bodies: &BodyStorage, slot: usize) -> Self {
        Self {
            position: bodies.position[slot],
            angle: bodies.angle[slot],
            inv_mass: bodies.inv_mass[slot],
            inv_inertia: bodies.inv_inertia[slot],
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct BodyVelocity {
    pub linear: Vec2,
    pub angular: f32,
}

impl BodyVelocity {
    pub fn from_storage(bodies: &BodyStorage, slot: usize) -> Self {
        Self {
            linear: bodies.linear_velocity[slot],
            angular: bodies.angular_velocity[slot],
        }
    }

    pub fn apply_impulse(&mut self, j_lin: Vec2, j_ang: f32, lambda: f32, inv_mass: f32, inv_inertia: f32) {
        self.linear += j_lin * (lambda * inv_mass);
        self.angular += j_ang * (lambda * inv_inertia);
    }
}
