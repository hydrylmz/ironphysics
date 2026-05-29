use physics_math::{Transform, Vec2};
use crate::{BodyHandle, ColliderHandle};
use crate::narrowphase::manifold::{ContactManifold, ContactPoint, ContactFeatureId};
use crate::shape::{BoxShape, Circle};



pub fn circle_vs_circle(
    ca: &Circle, xf_a: &Transform,
    cb: &Circle, xf_b: &Transform,
) -> Option<ContactManifold> {
    let center_a = xf_a.position;
    let center_b = xf_b.position;
    let delta = center_b - center_a;
    let dist_sq = delta.len_sq();
    let radius_sum = ca.radius + cb.radius;
    if dist_sq >= radius_sum * radius_sum {
        return None; // no collision
    }
    let dist = dist_sq.sqrt();
    let normal = if dist > 1e-6 {
        delta / dist
    } else {
        Vec2::new(0.0, 1.0) // degenerate case: circles at same position
    };
    let depth = radius_sum - dist;
    let point = center_a + normal * (ca.radius - depth * 0.5);
    Some(ContactManifold {
        normal,
        points: [ContactPoint {
            point,
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
}

pub fn circle_vs_box(
    circle: &Circle, xf_circle: &Transform,
    b:      &BoxShape, xf_box: &Transform,
) -> Option<ContactManifold> {
    let local_center = xf_box.apply_inv(xf_circle.position);
    let closest = Vec2::new(local_center.x.clamp(-b.half_extents.x, b.half_extents.x), local_center.y.clamp(-b.half_extents.y, b.half_extents.y));
    let inside = closest == local_center;
    let diff = local_center - closest;
    let dist_sq = diff.len_sq();
    if !inside && dist_sq >= circle.radius * circle.radius {
        return None; // no collision
    }
    let normal = if inside {
        let dx = b.half_extents.x - local_center.x.abs();
        let dy = b.half_extents.y - local_center.y.abs();
        if dx < dy {
            if local_center.x > 0.0 { Vec2::new(1.0, 0.0) } else { Vec2::new(-1.0, 0.0) }
        } else if local_center.y > 0.0 {
            Vec2::new(0.0, 1.0)
        } else {
            Vec2::new(0.0, -1.0)
        }
    } else {
        diff.normalize_or_zero()
    };
    let depth = if inside { circle.radius + dist_sq.sqrt() } else { circle.radius - dist_sq.sqrt() };
    let point = xf_box.apply(closest);
    let world_normal = xf_box.apply_vec(normal);

    Some(ContactManifold {
        normal: world_normal,
        points: [ContactPoint {
            point,
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
}


#[cfg(test)]
mod tests {
    use super::*;
    use physics_math::{Vec2, Transform};
    use crate::shape::{Circle, BoxShape};

    #[test]
    fn circle_circle_overlap() {
        // GIVEN: Circle A at (0,0) r=1, Circle B at (1,0) r=1
        //        overlap = 2 - 1 = 1 unit
        // THEN:  returns Some(manifold)
        //        normal ≈ (1, 0)  (A→B direction)
        //        depth  ≈ 1.0
        //        count  == 1
        let circle_a = Circle { radius: 1.0 };
        let circle_b = Circle { radius: 1.0 };
        let xf_a = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_b = Transform::new(Vec2::new(1.0, 0.0), 0.0);
        let manifold = circle_vs_circle(&circle_a, &xf_a, &circle_b, &xf_b).expect("Expected collision");
        assert!(manifold.normal.x > 0.9 && manifold.normal.x < 1.1, "Expected normal ≈ (1, 0)");
        assert!(manifold.normal.y > -0.1 && manifold.normal.y < 0.1, "Expected normal ≈ (1, 0)");
        assert!(manifold.points[0].depth > 0.9 && manifold.points[0].depth < 1.1, "Expected depth ≈ 1.0");
        assert_eq!(manifold.count, 1, "Expected count == 1");

    }

    #[test]
    fn circle_circle_separated() {
        // GIVEN: Circle A at (0,0) r=1, Circle B at (3,0) r=1
        //        gap = 3 - 2 = 1 unit
        // THEN:  returns None
        let circle_a = Circle { radius: 1.0 };
        let circle_b = Circle { radius: 1.0 };
        let xf_a = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_b = Transform::new(Vec2::new(3.0, 0.0), 0.0);
        let manifold = circle_vs_circle(&circle_a, &xf_a, &circle_b, &xf_b);
        assert!(manifold.is_none(), "Expected no collision");

    }

    #[test]
    fn circle_circle_touching_edge() {
        // GIVEN: Circle A at (0,0) r=1, Circle B at (2,0) r=1
        //        dist = 2.0 == radius_sum = 2.0  (exactly touching)
        // THEN:  returns None  (dist >= radius_sum → no overlap)
        let circle_a = Circle { radius: 1.0 };
        let circle_b = Circle { radius: 1.0 };
        let xf_a = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_b = Transform::new(Vec2::new(2.0, 0.0), 0.0);
        let manifold = circle_vs_circle(&circle_a, &xf_a, &circle_b, &xf_b);
        assert!(manifold.is_none(), "Expected no collision");

    }

    #[test]
    fn circle_circle_concentric_no_panic() {
        // GIVEN: Two circles at exactly the same position
        //        (degenerate case — avoid division by zero)
        // THEN:  returns Some(manifold) with an arbitrary stable normal
        //        does NOT panic or return NaN
        let circle_a = Circle { radius: 1.0 };
        let circle_b = Circle { radius: 1.0 };
        let xf_a = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_b = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let manifold = circle_vs_circle(&circle_a, &xf_a, &circle_b, &xf_b).expect("Expected collision");
        assert!(manifold.normal.x > -0.1 && manifold.normal.x < 0.1, "Expected normal.x ≈ 0");
        assert!(manifold.normal.y > 0.9 && manifold.normal.y < 1.1, "Expected normal.y ≈ 1.0");
        assert!(manifold.points[0].depth > 1.9 && manifold.points[0].depth < 2.1, "Expected depth ≈ 2.0");
        assert_eq!(manifold.count, 1, "Expected count == 1");

    }

    #[test]
    fn circle_box_overlap_face() {
        // GIVEN: BoxShape half_extents=(1,1) at origin
        //        Circle r=1 at (1.5, 0)  (approaching from the right)
        //        overlap = 1 + 1 - 1.5 = 0.5
        // THEN:  Some(manifold)
        //        normal ≈ (1, 0)   (pointing right, from box toward circle)
        //        depth  ≈ 0.5
        let box_shape = BoxShape { half_extents: Vec2::new(1.0, 1.0) };
        let circle = Circle { radius: 1.0 };
        let xf_box = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_circle = Transform::new(Vec2::new(1.5, 0.0), 0.0);
        let manifold = circle_vs_box(&circle, &xf_circle, &box_shape, &xf_box).expect("Expected collision");
        assert!(manifold.normal.x > 0.9 && manifold.normal.x < 1.1, "Expected normal ≈ (1, 0)");
        assert!(manifold.normal.y > -0.1 && manifold.normal.y < 0.1, "Expected normal ≈ (1, 0)");
        assert!(manifold.points[0].depth > 0.4 && manifold.points[0].depth < 0.6, "Expected depth ≈ 0.5");
        assert_eq!(manifold.count, 1, "Expected count == 1");
        
    }

    #[test]
    fn circle_box_separated() {
        // GIVEN: BoxShape at origin, Circle r=0.5 at (3, 0)
        // THEN:  None
        let box_shape = BoxShape { half_extents: Vec2::new(1.0, 1.0) };
        let circle = Circle { radius: 0.5 };
        let xf_box = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_circle = Transform::new(Vec2::new(3.0, 0.0), 0.0);
        let manifold = circle_vs_box(&circle, &xf_circle, &box_shape, &xf_box);
        assert!(manifold.is_none(), "Expected no collision");
    }

    #[test]
    fn circle_box_center_inside_box() {
        // GIVEN: BoxShape half_extents=(2,2) at origin
        //        Circle r=0.5 center at (0, 0)  (fully inside)
        // THEN:  Some(manifold) — inside case must not panic
        //        normal is one of the 4 face normals
        //        depth > circle.radius  (must push circle completely out)
        let box_shape = BoxShape { half_extents: Vec2::new(2.0, 2.0) };
        let circle = Circle { radius: 0.5 };
        let xf_box = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_circle = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let manifold = circle_vs_box(&circle, &xf_circle, &box_shape, &xf_box).expect("Expected collision");
        let expected_normals = [
            Vec2::new( 1.0,  0.0), // right
            Vec2::new( 0.0,  1.0), // top
            Vec2::new(-1.0,  0.0), // left
            Vec2::new( 0.0, -1.0), // bottom
        ];  
        let mut normal_matches = false;
        for expected in &expected_normals {
            if manifold.normal.x > expected.x - 0.1 && manifold.normal.x < expected.x + 0.1 &&
               manifold.normal.y > expected.y - 0.1 && manifold.normal.y < expected.y + 0.1 {
                normal_matches = true;
                break;
            }
        }
        assert!(normal_matches, "Expected normal to match one of the face normals");
        assert!(manifold.points[0].depth > 0.4 && manifold.points[0].depth < 0.6, "Expected depth > circle.radius");
        assert_eq!(manifold.count, 1, "Expected count == 1");
    }
}