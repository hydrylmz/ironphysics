use physics_math::{Aabb, Transform, Vec2};
use crate::shape::{Shape, ShapeType, MassProperties};

pub struct BoxShape {
    pub half_extents: Vec2,
}

impl BoxShape {
    pub fn new(half_width: f32, half_height: f32) -> Self {
        debug_assert!(half_width > 0.0, "BoxShape half_width must be positive");
        debug_assert!(half_height > 0.0, "BoxShape half_height must be positive");
        Self {
            half_extents: Vec2::new(half_width, half_height),
        }
    }

    pub fn vertices_local(&self) -> [Vec2; 4] {
        let hx = self.half_extents.x;
        let hy = self.half_extents.y;
        [
            Vec2::new(-hx, -hy), 
            Vec2::new( hx, -hy),
            Vec2::new( hx,  hy), 
            Vec2::new(-hx,  hy),
        ]
    }

    pub fn face_normals_local() -> [Vec2; 4] {
        [
            Vec2::new( 1.0,  0.0), // right
            Vec2::new( 0.0,  1.0), // top
            Vec2::new(-1.0,  0.0), // left
            Vec2::new( 0.0, -1.0), // bottom
        ]
    }   
}

impl Shape for BoxShape {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Box
    }

    fn compute_aabb(&self, transform: &Transform) -> Aabb {
        let rot = transform.rotation_mat();
        let abs_rot = physics_math::mat2::Mat2 {
            cols: [
                physics_math::Vec2::new(rot.cols[0].x.abs(), rot.cols[0].y.abs()),
                physics_math::Vec2::new(rot.cols[1].x.abs(), rot.cols[1].y.abs()),
            ],
        };
        let world_half = abs_rot.mul_vec(self.half_extents);
        let center = transform.position;
        Aabb::from_center_half_extents(center, world_half)
    }

    fn compute_mass_properties(&self, density: f32) -> MassProperties {
        let area = 4.0 * self.half_extents.x * self.half_extents.y;
        let mass = density * area;
        let inertia = (mass / 3.0) * (self.half_extents.x * self.half_extents.x + self.half_extents.y * self.half_extents.y);
        MassProperties {
            mass,
            inv_mass: if mass > 0.0 { 1.0 / mass } else { 0.0 },
            inertia,
            inv_inertia: if inertia > 0.0 { 1.0 / inertia } else { 0.0 },
            local_centroid: Vec2::zero(),
        }
    }

    fn support(&self, direction: Vec2) -> Vec2 {
        Vec2 {
            x: if direction.x >= 0.0 { self.half_extents.x } else { -self.half_extents.x },
            y: if direction.y >= 0.0 { self.half_extents.y } else { -self.half_extents.y },
        }
    }

    fn as_any(&self) -> &dyn std::any::Any { self }

    fn local_centroid(&self) -> Vec2 {
        Vec2::zero()
    }
}