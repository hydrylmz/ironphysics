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
    pub friction: f32,
    pub static_friction: f32,
    pub dynamic_friction: f32,
    pub angle: f32,
    pub angular_velocity: f32,
    pub moment_of_inertia: f32,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ForceType {
    Gravity,
    External,
    Collision,
}

impl RigidBody {
    pub fn new(position: Vector2, velocity: Vector2, mass: f32) -> Self {
        let r = 1.0;
        Self {
            position,
            velocity,
            mass,
            net_force: Vector2::new(0.0, 0.0),
            forces: vec![],
            gravity_enabled: true,
            drag_coefficient: 0.1,
            radius: r,
            friction: 0.8,
            static_friction: 0.5,
            dynamic_friction: 0.3,
            angle: 0.0,
            angular_velocity: 0.0,
            moment_of_inertia: 0.5 * mass * r * r,
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
            let drag_vec = self.velocity.normalize_copy() * -drag_mag;
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

    pub fn resolve_collision(a: &mut RigidBody, b: &mut RigidBody, restitution: f32) {
        let relative_velocity = b.velocity - a.velocity;
        let normal = Self::normal(a, b);
        let along_normal = relative_velocity.dot(&normal);

        if along_normal > 0.0 {
            return;
        }

        let inv_mass_sum = (1.0 / a.mass) + (1.0 / b.mass);
        let j = -(1.0 + restitution) * along_normal / inv_mass_sum;

        let inv_mass_a = 1.0 / a.mass;
        let inv_mass_b = 1.0 / b.mass;

        a.velocity = a.velocity - normal * (j * inv_mass_a);
        b.velocity = b.velocity + normal * (j * inv_mass_b);

        let mut tangent = relative_velocity - (normal * relative_velocity.dot(&normal));
        if tangent.length() > f32::EPSILON {
            tangent = tangent.normalize_copy();

            let tangent_velocity = (b.velocity - a.velocity).dot(&tangent);
            let jt = -tangent_velocity / inv_mass_sum;

            let mu = (a.friction * b.friction).sqrt();

            let mu_static = (a.static_friction * b.static_friction).sqrt();
            let mu_dynamic = (a.dynamic_friction * b.dynamic_friction).sqrt();
            let jt_clamped = if jt.abs() < j.abs() * mu_static {
                jt
            } else {
                -j.abs() * mu_dynamic * jt.signum()
            };
            a.velocity = a.velocity - tangent * (jt_clamped * inv_mass_a);
            b.velocity = b.velocity + tangent * (jt_clamped * inv_mass_b);

            let r_a = normal * a.radius;
            let r_b = -normal * b.radius;

            let r_a_cross_jt = r_a.x * tangent.y - r_a.y * tangent.x;
            let r_b_cross_jt = r_b.x * tangent.y - r_b.y * tangent.x;

            a.angular_velocity -= r_a_cross_jt / a.moment_of_inertia;
            b.angular_velocity += r_b_cross_jt / b.moment_of_inertia;
        }
    }
}
