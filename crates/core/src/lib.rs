pub mod world;

pub use world::World;

// Re-export types from physics_collision
pub use physics_collision::{
    BodyHandle, ColliderHandle, Material, ColliderDesc,
    Shape, ShapeType, Circle, BoxShape, ConvexPolygon, Capsule,
    CollisionFilter,
};

// Re-export types from physics_dynamics
pub use physics_dynamics::{
    GenerationalArena, BodyStorage, WorldConfig, BodyDesc, BodyType,
    MassProperties, BodyView, BodyViewMut, JointHandle, JointKind,
    DistanceJoint, RevoluteJoint, PrismaticJoint,
};

// Re-export commonly used math types from the physics_math crate so
// `crate::Vec2`, `crate::Aabb`, `crate::Transform`, and `crate::EPSILON`
// are available to other modules within this crate.
pub use physics_math::vec2::Vec2;
pub use physics_math::aabb::Aabb;
pub use physics_math::transform::Transform;
pub use physics_math::scalar::EPSILON;


/// Diagnostics collected during one call to World::step().
/// Zero-cost to carry when all timings are 0 — fields are filled by
/// the instrumentation wrappers in T-10.
#[derive(Debug, Clone, Default)]
pub struct StepStats {
    pub bodies_active:   u32,   // Dynamic + awake
    pub bodies_sleeping: u32,   // Dynamic + asleep
    pub bodies_static:   u32,   // Static (never simulated)

    pub broadphase_pairs:   u32,   // candidate pairs after BVH traversal
    pub narrowphase_hits:   u32,   // pairs that produced a ContactManifold
    pub contacts_total:     u32,   // total contact points this frame

    pub islands_active:   u32,
    pub islands_sleeping: u32,

    pub time_integrate_us:    u64,
    pub time_broadphase_us:   u64,
    pub time_narrowphase_us:  u64,
    pub time_island_build_us: u64,
    pub time_solve_us:        u64,
    pub time_position_us:     u64,
    pub time_sleep_us:        u64,
    pub time_total_us:        u64,
}

impl StepStats {
    pub fn reset(&mut self) {
        *self = StepStats {
            bodies_active: 0,
            bodies_sleeping: 0,
            bodies_static: 0,
            broadphase_pairs: 0,
            narrowphase_hits: 0,
            contacts_total: 0,
            islands_active: 0,
            islands_sleeping: 0,
            time_integrate_us: 0,
            time_broadphase_us: 0,
            time_narrowphase_us: 0,
            time_island_build_us: 0,
            time_solve_us: 0,
            time_position_us: 0,
            time_sleep_us: 0,
            time_total_us: 0,
        }
    }
}

/// Runs `f`, records elapsed microseconds into `*slot`, and returns f's result.
/// Usage: let pairs = timed(&mut stats.time_broadphase_us, || { ... });
#[inline]
pub fn timed<T, F: FnOnce() -> T>(slot: &mut u64, f: F) -> T {

    // Instant::now() is chosen by LORD CLAUDE because: its portable and safe.
    let start = std::time::Instant::now();
    let result = f();
    *slot = start.elapsed().as_micros() as u64;
    result
}