use crate::shape::{Shape, ShapeType, MassProperties};
use physics_math::{Transform, Vec2, Aabb};

pub struct Capsule {
    pub half_length: f32,  
    pub radius:      f32,
}

impl Shape for Capsule {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Capsule
    }

    fn compute_aabb(&self, transform: &Transform) -> Aabb {
        let top_local = Vec2::new(0.0, self.half_length);
        let bottom_local = Vec2::new(0.0, -self.half_length);
        let world_top = transform.apply(top_local);
        let world_bottom = transform.apply(bottom_local);  
        let aabb_top = Aabb::from_center_half_extents(world_top, Vec2::splat(self.radius));
        let aabb_bottom = Aabb::from_center_half_extents(world_bottom, Vec2::splat(self.radius));
        aabb_top.merge(&aabb_bottom)
    }

    fn compute_mass_properties(&self, density: f32) -> MassProperties {
        let area_rect = 4.0 * self.radius * self.half_length;
        let area_circle = std::f32::consts::PI * self.radius * self.radius;
        let area = area_rect + area_circle;
        let mass = density * area;
        let inertia_rect = (mass * area_rect / area) / 3.0 * (self.radius * self.radius + self.half_length * self.half_length);
        let inertia_circle = (mass * area_circle / area) * (0.5 * self.radius * self.radius + self.half_length * self.half_length);
        MassProperties {
            mass,
            inv_mass: if mass > 0.0 { 1.0 / mass } else { 0.0 },
            inertia: inertia_rect + inertia_circle,
            inv_inertia: if inertia_rect + inertia_circle > 0.0 { 1.0 / (inertia_rect + inertia_circle) } else { 0.0 },
            local_centroid: Vec2::zero(),
        }
    }

    fn support(&self, direction: Vec2) -> Vec2 {
        let a = Vec2::new(0.0,  self.half_length);
        let b = Vec2::new(0.0, -self.half_length);
        let segment_support = if a.dot(direction) >= b.dot(direction) { a } else { b };
        segment_support + direction.normalize_or_zero() * self.radius
    }

    fn as_any(&self) -> &dyn std::any::Any { self }

    fn local_centroid(&self) -> Vec2 {
        Vec2::zero()
    }
}