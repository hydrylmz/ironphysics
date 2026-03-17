use crate::math::vec2::Vector2;

pub struct RigidBody {
    pub position: Vector2,
    pub velocity: Vector2,
    pub mass: f32,
    pub net_force: Vector2,
    pub forces: Vec<(Vector2, ForceType)>,
    pub gravity_enabled: bool,
    pub drag_coefficient: f32,
}

pub enum ForceType {
    Gravity,
    External,
    Collision,
}

impl RigidBody {
    pub fn new(position: Vector2, velocity: Vector2, mass: f32) -> Self {
        Self {
            position,
            velocity,
            mass,
            net_force: Vector2::new(0.0, 0.0),
            forces: vec![],
            gravity_enabled: true,
            drag_coefficient: 0.1,
        }
    }

    // RigidBody update function
    pub fn update(&mut self, dt: f32) {
        if self.gravity_enabled {
            self.forces
                .push((Vector2::new(0.0, 9.8) * self.mass, ForceType::Gravity));
        }

        self.forces.push((
            (self.velocity * self.velocity.length()) * -self.drag_coefficient,
            ForceType::External,
        ));

        self.net_force = self
            .forces
            .iter()
            .filter(|(_, ftype)| match ftype {
                ForceType::Gravity => self.gravity_enabled,
                _ => true,
            })
            .fold(Vector2::new(0.0, 0.0), |acc, (v, _)| acc + *v);

        self.velocity = self.velocity + (self.net_force * (1.0 / self.mass)) * dt;
        self.position.add(&(self.velocity * dt));

        self.net_force.x = 0.0;
        self.net_force.y = 0.0;
        self.forces.clear();
    }

    pub fn apply_force(&mut self, force: Vector2, force_type: ForceType) {
        if let ForceType::Gravity = force_type {
            if !self.gravity_enabled {
                return;
            }
        }

        self.forces.push((force, force_type));
    }
}
