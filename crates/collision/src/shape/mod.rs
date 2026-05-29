pub mod circle;
pub mod box_shape;
pub mod convex_polygon;
pub mod capsule;
use physics_math::{Transform, Vec2};
use physics_math::aabb::Aabb;
use crate::{BodyHandle, Material};
use crate::filter::CollisionFilter;
use rayon::prelude::*;
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

impl Default for ColliderStorage {
    fn default() -> Self {
        Self::new()
    }
}

fn _assert_shape_send_sync()
where
    Box<dyn Shape>: Send + Sync,
{}

impl ColliderStorage {
    pub fn new() -> Self {
        Self {
            body_handle: Vec::new(),
            shape: Vec::new(),
            local_transform: Vec::new(),
            world_transform: Vec::new(),
            world_aabb: Vec::new(),
            filter: Vec::new(),
            is_sensor: Vec::new(),
            density: Vec::new(),
            restitution: Vec::new(),
            friction: Vec::new(),
            generation: Vec::new(),
            len: 0,
        }
    }

    pub fn with_capacity(cap: usize) -> Self {
        Self {
            body_handle: Vec::with_capacity(cap),
            shape: Vec::with_capacity(cap),
            local_transform: Vec::with_capacity(cap),
            world_transform: Vec::with_capacity(cap),
            world_aabb: Vec::with_capacity(cap),
            filter: Vec::with_capacity(cap),
            is_sensor: Vec::with_capacity(cap),
            density: Vec::with_capacity(cap),
            restitution: Vec::with_capacity(cap),
            friction: Vec::with_capacity(cap),
            generation: Vec::with_capacity(cap),
            len: 0,
        }
    }
    pub fn update_world_transforms<F>(&mut self, get_transform: F)
    where
        F: Fn(BodyHandle) -> Transform,
    {
        for i in 0..self.len {
            let body_handle = self.body_handle[i];
            let body_xf = get_transform(body_handle);
            let local = self.local_transform[i];
            self.world_transform[i] = body_xf.combine(&local);
            self.world_aabb[i] = self.shape[i].compute_aabb(&self.world_transform[i]);
        }
    }

    pub fn push(&mut self, body: BodyHandle, desc: ColliderDesc) -> u32 {
        let slot = self.len as u32;
        self.body_handle.push(body);
        self.shape.push(desc.shape);
        self.local_transform.push(desc.local_transform);
        self.world_transform.push(Transform::identity());
        self.world_aabb.push(Aabb::new(Vec2::zero(), Vec2::zero()));
        self.filter.push(desc.filter);
        self.is_sensor.push(desc.is_sensor);
        self.density.push(desc.material.density);
        self.restitution.push(desc.material.restitution);
        self.friction.push(desc.material.friction);
        self.generation.push(0);
        self.len += 1;
        slot
    }

    pub fn recompute_aabbs_parallel(&mut self) {
    // Recompute world_aabb for every collider in parallel.
    // Does NOT touch world_transform (already synced in update_world_transforms).
    // Does NOT touch the BVH (caller updates it serially after this call).
    //
    // Algorithm:
    //   Use rayon::iter::IndexedParallelIterator over a zip of
    //   (shape, world_transform, world_aabb) arrays.
    //
    //   The zip requires parallel mutable access to world_aabb while
    //   shape and world_transform are read-only.
    //
    //   Safe approach — par_iter_mut on world_aabb with index:
    //     self.world_aabb
    //         .par_iter_mut()
    //         .enumerate()
    //         .for_each(|(i, aabb)| {
    //             *aabb = self.shape[i].compute_aabb(&self.world_transform[i])
    //         })
    //
    //   Borrow checker note: self.world_aabb is mutably borrowed; self.shape and
    //   self.world_transform are read-only borrows of OTHER Vec fields.
    //   Rust allows borrowing separate fields of a struct simultaneously, but
    //   the closure captures &self implicitly. Work around this by using
    //   split-field borrows before the par_iter:
    //
    //     let shapes     = &self.shape;
    //     let transforms = &self.world_transform;
    //     self.world_aabb
    //         .par_iter_mut()
    //         .enumerate()
    //         .for_each(|(i, aabb)| {
    //             *aabb = shapes[i].compute_aabb(&transforms[i])
    //         })
    //
    //   This is sound: shapes[i] and transforms[i] are read-only, and each
    //   par_iter_mut element is a DIFFERENT mutable slot — no aliasing.
    //
    // Expected speedup:
    //   At 10K colliders, compute_aabb has non-trivial cost (cos/sin per BoxShape).
    //   Linear scan on 1 core ≈ 0.5ms; on 8 cores ≈ 0.07ms.
        let shapes     = &self.shape;
        let transforms = &self.world_transform;
        self.world_aabb
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, aabb)| {
                *aabb = shapes[i].compute_aabb(&transforms[i])
            });
        
    }

}
