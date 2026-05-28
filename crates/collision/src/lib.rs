pub mod filter;
pub mod shape;
pub mod broadphase;
pub mod narrowphase;
pub mod pool;

pub use filter::{CollisionFilter, combined_friction, combined_restitution};
pub use shape::{Shape, ShapeType, Circle, BoxShape, ConvexPolygon, Capsule, ColliderStorage};
pub use broadphase::bvh::DynamicAabbTree;
pub use narrowphase::manifold::{ContactManifold, ContactPoint, ContactFeatureId,
                                ContactFeatureKind, MAX_MANIFOLD_POINTS};
pub use narrowphase::dispatch::dispatch_narrowphase;
pub use pool::ContactPool;

// Re-export ColliderHandle from physics_core
pub use physics_core::ColliderHandle;

// Re-export commonly used math types from the physics_math crate
pub use physics_math::vec2::Vec2;
pub use physics_math::aabb::Aabb;
pub use physics_math::transform::Transform;

