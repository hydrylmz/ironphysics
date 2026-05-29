pub mod filter;
pub mod shape;
pub mod broadphase;
pub mod narrowphase;
pub mod pool;
pub mod handle;
pub mod collider_handle;
pub mod material;

pub use filter::{CollisionFilter, combined_friction, combined_restitution};
pub use shape::{Shape, ShapeType, Circle, BoxShape, ConvexPolygon, Capsule, ColliderStorage, ColliderDesc};
pub use broadphase::bvh::DynamicAabbTree;
pub use narrowphase::manifold::{ContactManifold, ContactPoint, ContactFeatureId,
                                ContactFeatureKind, MAX_MANIFOLD_POINTS};
pub use narrowphase::dispatch::{dispatch_narrowphase, run_narrowphase_parallel};
pub use pool::ContactPool;

pub use handle::BodyHandle;
pub use collider_handle::ColliderHandle;
pub use material::Material;

// Re-export commonly used math types from the physics_math crate
pub use physics_math::vec2::Vec2;
pub use physics_math::aabb::Aabb;
pub use physics_math::transform::Transform;

