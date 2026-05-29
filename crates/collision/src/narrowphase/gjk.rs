use physics_math::{Transform, Vec2};
use crate::shape::{Shape, support_world};


/// Simplex is apparently the evolving set of support points in Minkowski space. (I love this Minkowski guy)
/// In 2D, a simplex has at most 3 points (triangle).
pub struct Simplex {
    pub points: [Vec2; 3],
    pub count:  usize,
}

pub fn gjk_intersection(
    shape_a: &dyn Shape, xf_a: &Transform,
    shape_b: &dyn Shape, xf_b: &Transform,
) -> Option<Simplex> {
    let mut simplex = Simplex {
        points: [Vec2::zero(), Vec2::zero(), Vec2::zero()],
        count:  0,
    };
    let mut d = xf_b.position - xf_a.position;
    if d.len_sq() < 1e-6 {
        d = Vec2::new(1.0, 0.0); // arbitrary direction if centers are very close
    }
    simplex.points[0] = support_world(shape_a, xf_a, d) - support_world(shape_b, xf_b, -d);
    simplex.count = 1;
    if simplex.points[0].dot(d) < 0.0 {
        return None; // no collision
    }
    Some(simplex)
}


#[cfg(test)]
mod tests {
    use super::*;
    use physics_math::{Vec2, Transform};
    use crate::shape::{Circle, ConvexPolygon};

    #[test]
    fn gjk_two_polygons_overlapping() {
        // GIVEN: Two unit squares as ConvexPolygon at (0,0) and (0.5,0)
        // THEN:  gjk_intersection returns Some(_)
        let verts = smallvec::SmallVec::from_slice(&[Vec2::new(-1.0, -1.0), Vec2::new(1.0, -1.0), Vec2::new(1.0, 1.0), Vec2::new(-1.0, 1.0)]);
        let square_a = ConvexPolygon::new(verts.clone());
        let square_b = ConvexPolygon::new(verts);
        let xf_a = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_b = Transform::new(Vec2::new(0.5, 0.0), 0.0);
        let simplex = gjk_intersection(&square_a, &xf_a, &square_b, &xf_b);
        assert!(simplex.is_some(), "Expected collision");   
    }

    #[test]
    fn gjk_two_polygons_separated() {
        // GIVEN: Two unit squares at (0,0) and (3,0)
        // THEN:  gjk_intersection returns None
        let verts = smallvec::SmallVec::from_slice(&[Vec2::new(-1.0, -1.0), Vec2::new(1.0, -1.0), Vec2::new(1.0, 1.0), Vec2::new(-1.0, 1.0)]);
        let square_a = ConvexPolygon::new(verts.clone());
        let square_b = ConvexPolygon::new(verts);
        let xf_a = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_b = Transform::new(Vec2::new(3.0, 0.0), 0.0);
        let simplex = gjk_intersection(&square_a, &xf_a, &square_b, &xf_b);
        assert!(simplex.is_none(), "Expected no collision");
    }

    #[test]
    fn gjk_two_circles_via_support() {
        // GIVEN: Two Circle shapes with correct support functions
        //        at positions where they overlap
        // THEN:  gjk_intersection returns Some(_)
        //        (Validates support function is called correctly in world space)
        let circle_a = Circle { radius: 1.0 };
        let circle_b = Circle { radius: 1.0 };
        let xf_a = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_b = Transform::new(Vec2::new(1.0, 0.0), 0.0);
        let simplex = gjk_intersection(&circle_a, &xf_a, &circle_b, &xf_b);
        assert!(simplex.is_some(), "Expected collision");

    }
}