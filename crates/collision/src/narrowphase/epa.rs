const EPA_MAX_ITER:     usize = 32;
const EPA_TOLERANCE:    f32   = 1e-4; //i am still amazed that f32 has 1e-4 thingy
use physics_math::{Transform, Vec2};
use crate::{BodyHandle, ColliderHandle};
use crate::narrowphase::manifold::{ContactManifold, ContactPoint, ContactFeatureId};
use crate::shape::Shape;
use crate::narrowphase::gjk::{gjk_intersection, Simplex};
use crate::shape::support_world;

pub fn epa_penetration(
    simplex:  &Simplex,
    shape_a:  &dyn Shape, xf_a: &Transform,
    shape_b:  &dyn Shape, xf_b: &Transform,
) -> (Vec2, f32) {
    let mut polytope = simplex.points[..simplex.count].to_vec();
    // Ensure CCW winding:
    if (polytope[1] - polytope[0]).perp().dot(polytope[2] - polytope[0]) < 0.0 {
        polytope.swap(1, 2);
    }
    let mut best_normal = Vec2::zero();
    let mut best_distance = f32::INFINITY;
    for _ in 0..EPA_MAX_ITER {
        // Find closest edge to origin:
        let mut closest_index = 0;
        for i in 0..polytope.len() {
            let a = polytope[i];
            let b = polytope[(i + 1) % polytope.len()];
            let edge = b - a;
            let edge_normal = edge.perp().normalize_or_zero();
            let distance = a.dot(edge_normal);
            if distance < best_distance {
                best_distance = distance;
                best_normal = edge_normal;
                closest_index = i;
            }
        }
        // Support point in edge normal direction:
        let support = support_world(shape_a, xf_a, best_normal) - support_world(shape_b, xf_b, -best_normal);
        let new_distance = support.dot(best_normal);
        if new_distance - best_distance < EPA_TOLERANCE {
            // Converged:
            return (best_normal, best_distance);
        }
        // Insert support into polytope:
        polytope.insert(closest_index + 1, support);
    }
    (best_normal, best_distance)
}


pub fn gjk_epa_manifold(
    shape_a: &dyn Shape, xf_a: &Transform,
    shape_b: &dyn Shape, xf_b: &Transform,
) -> Option<ContactManifold> {
    if let Some(simplex) = gjk_intersection(shape_a, xf_a, shape_b, xf_b) {
        let (normal, depth) = epa_penetration(&simplex, shape_a, xf_a, shape_b, xf_b);
        let support_a = support_world(shape_a, xf_a,  normal);
        let support_b = support_world(shape_b, xf_b, -normal);
        let contact_point = (support_a + support_b) * 0.5;
        Some(ContactManifold {
            normal,
            points: [ContactPoint {
                point: contact_point,
                depth,
                normal_impulse: 0.0,
                tangent_impulse: 0.0,
                id: ContactFeatureId::default(),
            }, ContactPoint::default()],
            count: 1,
            body_a: BodyHandle::default(),
            body_b: BodyHandle::default(),
            collider_a: ColliderHandle::default(),
            collider_b: ColliderHandle::default(),
            friction: 0.0,
            restitution: 0.0,
        })
    } else {
        None
    }

}