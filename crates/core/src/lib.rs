pub mod arena;
pub mod body;
pub mod config;
pub mod handle;
pub mod world;

pub use arena::GenerationalArena;
pub use body::BodyStorage;
pub use config::WorldConfig;
pub use handle::BodyHandle;
pub use world::World;

// Re-export commonly used math types from the physics_math crate so
// `crate::Vec2`, `crate::Aabb`, `crate::Transform`, and `crate::EPSILON`
// are available to other modules within this crate.
pub use physics_math::vec2::Vec2;
pub use physics_math::aabb::Aabb;
pub use physics_math::transform::Transform;
pub use physics_math::scalar::EPSILON;
