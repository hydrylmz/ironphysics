use crate::shape::{Shape, ShapeType, MassProperties};
use physics_math::{Transform, Vec2, Aabb};

pub struct Circle {
    pub radius: f32,
}

impl Circle {
    pub fn new(radius: f32) -> Self {
        Self { radius }
    }
}

impl Shape for Circle {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Circle
    }

    fn compute_aabb(&self, transform: &Transform) -> Aabb {
        let center = transform.position;
        let half = Vec2::splat(self.radius);
        Aabb::from_center_half_extents(center, half)
    }

    fn compute_mass_properties(&self, density: f32) -> MassProperties {
        let area = std::f32::consts::PI * self.radius * self.radius;
        let mass = density * area;
        let inertia = 0.5 * mass * self.radius * self.radius;
        MassProperties {
            mass,
            inv_mass: if mass > 0.0 { 1.0 / mass } else { 0.0 },
            inertia,
            inv_inertia: if inertia > 0.0 { 1.0 / inertia } else { 0.0 },
            local_centroid: Vec2::zero(),
        }
    }

    fn support(&self, direction: Vec2) -> Vec2 {
        let n = direction.normalize_or_zero();
        if n == Vec2::zero() {
            return Vec2::zero();
        }
        n * self.radius
    }

    fn as_any(&self) -> &dyn std::any::Any { self }

    fn local_centroid(&self) -> Vec2 {
        Vec2::zero()
    }
}