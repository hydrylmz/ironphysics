use physics_math::Vec2;
use crate::{BodyHandle, ColliderHandle};

pub const MAX_MANIFOLD_POINTS: usize = 2;

/// Identifies which geometric feature (vertex or face) generated a contact point.
/// Used to match contact points across frames for warm-starting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ContactFeatureId {
    pub index_a: u8,   // vertex or edge index on shape A
    pub index_b: u8,   // vertex or edge index on shape B
    pub kind:    ContactFeatureKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ContactFeatureKind {
    #[default]
    Vertex,
    Face,
}

/// One point of contact between two shapes.
#[derive(Debug, Clone, Copy, Default)]
pub struct ContactPoint {
    /// World-space position of the contact.
    pub point:           Vec2,

    /// Penetration depth (positive = shapes are overlapping).
    pub depth:           f32,

    pub normal_impulse:  f32,

    pub tangent_impulse: f32,

    pub id:              ContactFeatureId,
}

/// The result of a narrowphase test between two colliders.
/// Contains up to MAX_MANIFOLD_POINTS contact points.
#[derive(Debug, Clone)]
pub struct ContactManifold {
    /// Collision normal in world space, pointing from A toward B.
    /// (Impulse applied to A in -normal direction, to B in +normal direction.)
    pub normal: Vec2,

    pub points: [ContactPoint; MAX_MANIFOLD_POINTS],

    /// Number of valid entries in `points`. Either 1 or 2.
    pub count:  usize,

    /// Handles of the two bodies involved.
    pub body_a:  BodyHandle,
    pub body_b:  BodyHandle,

    /// Collider handles for filtering and event dispatch.
    pub collider_a: ColliderHandle,
    pub collider_b: ColliderHandle,

    /// Combined material properties (computed from both colliders).
    pub friction:    f32,
    pub restitution: f32,
}

impl ContactManifold {
    pub fn swapped(&self) -> Self {
        let mut swapped = self.clone();
        swapped.normal = -self.normal;
        std::mem::swap(&mut swapped.body_a, &mut swapped.body_b);
        std::mem::swap(&mut swapped.collider_a, &mut swapped.collider_b);
        for point in &mut swapped.points {
            std::mem::swap(&mut point.id.index_a, &mut point.id.index_b);
        }
        swapped
    }
}