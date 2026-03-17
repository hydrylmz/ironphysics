use std::{collections::hash_set::Difference, default};

use crate::math::vec2::Vector2;

#[derive(Debug, Default)]
pub struct RigidBody {
    pub position: Vector2,
    pub velocity: Vector2,
    pub mass: f32,
    pub net_force: Vector2,
    pub forces: Vec<(Vector2, ForceType)>,
    pub gravity_enabled: bool,
    pub drag_coefficient: f32,
    pub radius: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
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
            radius: 1.0,
        }
    }

    // RigidBody update function
    pub fn update(&mut self, dt: f32) {
        if self.gravity_enabled {
            let gravity_vec = Vector2::new(0.0, -9.8) * self.mass;
            self.forces.push((gravity_vec, ForceType::Gravity));
        }

        if self.velocity.length() > f32::EPSILON {
            let drag_mag = self.velocity.length() * self.velocity.length() * self.drag_coefficient;
            let drag_vec = self.velocity.normalize_copy() * -drag_mag; // Oppose velocity
            self.forces.push((drag_vec, ForceType::External));
        }

        self.net_force = self
            .forces
            .iter()
            .fold(Vector2::new(0.0, 0.0), |acc, (v, _)| acc + *v);

        let inv_mass = if self.mass > 0.0 {
            1.0 / self.mass
        } else {
            0.0
        };
        self.velocity = self.velocity + (self.net_force * inv_mass) * dt;

        let displacement = self.velocity * dt;
        self.position.add(&displacement);

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

    pub fn is_colliding(a: &RigidBody, b: &RigidBody) -> bool {
        let difference = b.position - a.position;
        difference.dot(&difference) <= (a.radius + b.radius) * (a.radius + b.radius)
    }

    pub fn normal(a: &RigidBody, b: &RigidBody) -> Vector2 {
        let difference = b.position - a.position;
        let len = difference.length();
        if len > f32::EPSILON {
            difference.normalize_copy()
        } else {
            Vector2::new(1.0, 0.0)
        }
    }

    pub fn resolve_penetration(a: &mut RigidBody, b: &mut RigidBody) {
        let difference = b.position - a.position;
        let distance = difference.length();
        let mut normal = Vector2::new(1.0, 0.0);
        if distance > f32::EPSILON {
            normal = Self::normal(a, b);
        }
        let penetration = (a.radius + b.radius) - distance;
        a.position = a.position - normal * penetration / 2.0;
        b.position = b.position + normal * penetration / 2.0;
    }

    pub fn resolve_velocity(a: &mut RigidBody, b: &mut RigidBody, restitution: f32) {
        let relative_velocity = b.velocity - a.velocity;
        let normal = &Self::normal(a, b);
        let along_normal = relative_velocity.dot(normal);
        if along_normal <= 0.0 {
            let j = -(1.0 + restitution) * along_normal / ((1.0 / a.mass) + (1.0 / b.mass));
            a.velocity = a.velocity - (*normal * (j / a.mass));
            b.velocity = b.velocity + (*normal * (j / b.mass));
        }
    }
}
