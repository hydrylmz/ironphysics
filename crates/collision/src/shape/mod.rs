pub mod circle;
pub mod box_shape;
pub mod convex_polygon;
pub mod capsule;
use physics_math::{Transform, Vec2};
use physics_math::aabb::Aabb;
use physics_core::{BodyHandle, Material};
use crate::filter::CollisionFilter;

pub fn support_world(shape: &dyn Shape, xf: &Transform, direction: Vec2) -> Vec2 {
    let local_dir = xf.apply_inv(xf.position + direction);
    let local_support = shape.support(local_dir);
    xf.apply(local_support)
}

pub use circle::Circle;
pub use box_shape::BoxShape;
pub use convex_polygon::ConvexPolygon;
pub use capsule::Capsule;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ShapeType {
    Circle   = 0,
    Box      = 1,
    Capsule  = 2,
    Polygon  = 3,
}

pub trait Shape: Send + Sync {
    fn shape_type(&self) -> ShapeType;

    fn compute_aabb(&self, transform: &Transform) -> Aabb;

    fn compute_mass_properties(&self, density: f32) -> MassProperties;

    fn support(&self, direction: Vec2) -> Vec2;

    fn local_centroid(&self) -> Vec2;
    fn as_any(&self) -> &dyn std::any::Any;
}

#[derive(Debug, Clone, Copy)]
pub struct MassProperties {
    pub mass:           f32,
    pub inv_mass:       f32,    
    pub inertia:        f32, 
    pub inv_inertia:    f32,
    pub local_centroid: Vec2,
}

/// Descriptor for creating a collider — passed to World::add_collider.
pub struct ColliderDesc {
    pub shape:            Box<dyn Shape>,
    pub material:         Material,         // defined in physics_core
    pub local_transform:  Transform,        // offset from body origin
    pub filter:           CollisionFilter,
    pub is_sensor:        bool,             // true = events only, no impulse response
}

/// Collider state, stored alongside body state.
/// Each collider is attached to exactly one body.
pub struct ColliderStorage {
    pub body_handle:      Vec<BodyHandle>,
    pub shape:            Vec<Box<dyn Shape>>,
    pub local_transform:  Vec<Transform>,
    pub world_transform:  Vec<Transform>,   // derived: body_transform * local_transform
    pub world_aabb:       Vec<Aabb>,        // derived from shape + world_transform
    pub filter:           Vec<CollisionFilter>,
    pub is_sensor:        Vec<bool>,
    pub density:          Vec<f32>,
    pub restitution:      Vec<f32>,
    pub friction:         Vec<f32>,
    pub generation:       Vec<u32>,
    pub len:              usize,
}