
use physics_core::{BodyHandle, ColliderHandle};
use physics_math::{Transform, Vec2};
use crate::shape::BoxShape;
use crate::narrowphase::manifold::{ContactManifold, ContactPoint, ContactFeatureId};

const EPSILON: f32 = 1e-6;

fn axis_overlap(
    a: &BoxShape, xf_a: &Transform,
    b: &BoxShape, xf_b: &Transform,
    axis: Vec2,
) -> f32 {
    let axis = axis.normalize_or_zero();
    let a_x = xf_a.apply_vec(Vec2::new(1.0, 0.0));
    let a_y = xf_a.apply_vec(Vec2::new(0.0, 1.0));
    let b_x = xf_b.apply_vec(Vec2::new(1.0, 0.0));
    let b_y = xf_b.apply_vec(Vec2::new(0.0, 1.0));

    let ra = a.half_extents.x * axis.dot(a_x).abs()
           + a.half_extents.y * axis.dot(a_y).abs();
    let rb = b.half_extents.x * axis.dot(b_x).abs()
           + b.half_extents.y * axis.dot(b_y).abs();
    let center_proj = (xf_b.position - xf_a.position).dot(axis).abs();

    ra + rb - center_proj
}

fn world_vertices(shape: &BoxShape, xf: &Transform) -> [Vec2; 4] {
    let local = shape.vertices_local();
    [
        xf.apply(local[0]),
        xf.apply(local[1]),
        xf.apply(local[2]),
        xf.apply(local[3]),
    ]
}

fn select_face_vertices(vertices: &[Vec2; 4], normal: Vec2) -> Vec<Vec2> {
    let mut best = f32::NEG_INFINITY;
    let mut verts = Vec::new();

    for &vertex in vertices.iter() {
        let projection = vertex.dot(normal);
        if projection > best + EPSILON {
            best = projection;
            verts.clear();
            verts.push(vertex);
        } else if (projection - best).abs() <= EPSILON {
            verts.push(vertex);
        }
    }

    if verts.len() == 1 {
        verts.push(verts[0]);
    }
    verts
}

fn clip_polygon_to_plane(points: &[Vec2], plane_normal: Vec2, plane_offset: f32) -> Vec<Vec2> {
    let mut output = Vec::new();
    if points.is_empty() {
        return output;
    }

    for i in 0..points.len() {
        let current = points[i];
        let next = points[(i + 1) % points.len()];
        let current_dist = plane_normal.dot(current) - plane_offset;
        let next_dist = plane_normal.dot(next) - plane_offset;
        let current_inside = current_dist <= 0.0;
        let next_inside = next_dist <= 0.0;

        if current_inside {
            output.push(current);
        }

        if current_inside ^ next_inside {
            let t = current_dist / (current_dist - next_dist);
            let intersection = current + (next - current) * t;
            output.push(intersection);
        }
    }

    output
}

fn unique_points(points: &[Vec2]) -> Vec<Vec2> {
    let mut unique = Vec::new();

    for &point in points.iter() {
        if !unique.iter().any(|existing: &Vec2| {
            (existing.x - point.x).abs() < EPSILON && (existing.y - point.y).abs() < EPSILON
        }) {
            unique.push(point);
        }
    }

    unique
}

pub fn sat_box_vs_box(
    a: &BoxShape, xf_a: &Transform,
    b: &BoxShape, xf_b: &Transform,
) -> Option<ContactManifold> {
    let axes_a = [
        xf_a.apply_vec(Vec2::new(1.0, 0.0)).normalize_or_zero(),
        xf_a.apply_vec(Vec2::new(0.0, 1.0)).normalize_or_zero(),
    ];
    let axes_b = [
        xf_b.apply_vec(Vec2::new(1.0, 0.0)).normalize_or_zero(),
        xf_b.apply_vec(Vec2::new(0.0, 1.0)).normalize_or_zero(),
    ];

    let mut min_penetration = f32::INFINITY;
    let mut reference_normal = Vec2::new(1.0, 0.0);
    let mut reference_is_a = true;

    for &axis in axes_a.iter() {
        let penetration = axis_overlap(a, xf_a, b, xf_b, axis);
        if penetration <= 0.0 {
            return None;
        }
        if penetration < min_penetration {
            min_penetration = penetration;
            let dir = xf_b.position - xf_a.position;
            reference_normal = if axis.dot(dir) < 0.0 { -axis } else { axis };
            reference_is_a = true;
        }
    }

    for &axis in axes_b.iter() {
        let penetration = axis_overlap(a, xf_a, b, xf_b, axis);
        if penetration <= 0.0 {
            return None;
        }
        if penetration < min_penetration {
            min_penetration = penetration;
            let dir = xf_a.position - xf_b.position;
            reference_normal = if axis.dot(dir) < 0.0 { -axis } else { axis };
            reference_is_a = false;
        }
    }

    let (reference_shape, reference_xf, incident_shape, incident_xf) = if reference_is_a {
        (a, xf_a, b, xf_b)
    } else {
        (b, xf_b, a, xf_a)
    };

    let incident_face_normal = BoxShape::face_normals_local()
        .iter()
        .map(|&local_normal| incident_xf.apply_vec(local_normal))
        .min_by(|x, y| x.dot(reference_normal).partial_cmp(&y.dot(reference_normal)).unwrap())
        .unwrap_or(Vec2::new(1.0, 0.0));

    let incident_vertices = world_vertices(incident_shape, incident_xf);
    let mut clipped_points = select_face_vertices(&incident_vertices, incident_face_normal);

    let reference_vertices = world_vertices(reference_shape, reference_xf);
    let reference_face_vertices = select_face_vertices(&reference_vertices, reference_normal);
    let reference_face_center = (reference_face_vertices[0] + reference_face_vertices[1]) * 0.5;

    let tangent = (reference_face_vertices[1] - reference_face_vertices[0]).normalize_or_zero();
    let t0 = tangent.dot(reference_face_vertices[0]);
    let t1 = tangent.dot(reference_face_vertices[1]);
    let min_t = t0.min(t1);
    let max_t = t0.max(t1);

    clipped_points = clip_polygon_to_plane(&clipped_points, tangent, max_t);
    clipped_points = clip_polygon_to_plane(&clipped_points, -tangent, -min_t);
    clipped_points = clip_polygon_to_plane(
        &clipped_points,
        reference_normal,
        reference_normal.dot(reference_face_center),
    );

    let clipped_points = unique_points(&clipped_points);
    let mut contact_points = Vec::new();
    for point in clipped_points.into_iter() {
        let depth = reference_normal.dot(reference_face_center - point);
        if depth >= 0.0 {
            contact_points.push(ContactPoint {
                point,
                depth,
                normal_impulse: 0.0,
                tangent_impulse: 0.0,
                id: ContactFeatureId::default(),
            });
        }
        if contact_points.len() == 2 {
            break;
        }
    }

    if contact_points.is_empty() {
        return None;
    }

    let mut points = [ContactPoint::default(), ContactPoint::default()];
    for (i, contact_point) in contact_points.into_iter().enumerate().take(2) {
        points[i] = contact_point;
    }

    let normal = if reference_is_a {
        reference_normal
    } else {
        -reference_normal
    };

    let count = if points[1].depth > 0.0 { 2 } else { 1 };

    Some(ContactManifold {
        normal,
        points,
        count,
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
    use crate::shape::BoxShape;

    #[test]
    fn box_box_axis_aligned_overlap() {
        // GIVEN: Box A [-1,1]×[-1,1]  Box B [0.5,2.5]×[-1,1]
        //        X overlap = 0.5
        // THEN:  Some(manifold)
        //        normal ≈ (1,0) or (-1,0)
        //        depth  ≈ 0.5
        //        count  == 1 or 2  (edge-edge → 2 contact points)
        let box_a = BoxShape { half_extents: Vec2::new(1.0, 1.0) };
        let box_b = BoxShape { half_extents: Vec2::new(1.0, 1.0) };
        let xf_a = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_b = Transform::new(Vec2::new(1.5, 0.0), 0.0);
        let manifold = sat_box_vs_box(&box_a, &xf_a, &box_b, &xf_b).expect("Expected collision");
        assert!(manifold.normal.x.abs() > 0.9 && manifold.normal.y.abs() < 0.1, "Expected normal ≈ (±1, 0)");
        assert!(manifold.points[0].depth > 0.4 && manifold.points[0].depth < 0.6, "Expected depth ≈ 0.5");
        assert!(manifold.count == 1 || manifold.count == 2, "Expected count == 1 or 2");

    }

    #[test]
    fn box_box_axis_aligned_separated() {
        // GIVEN: Box A [-1,1]×[-1,1]  Box B [2,4]×[-1,1]
        // THEN:  None
        let box_a = BoxShape { half_extents: Vec2::new(1.0, 1.0) };
        let box_b = BoxShape { half_extents: Vec2::new(1.0, 1.0) };
        let xf_a = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_b = Transform::new(Vec2::new(3.0, 0.0), 0.0);
        let manifold = sat_box_vs_box(&box_a, &xf_a, &box_b, &xf_b);
        assert!(manifold.is_none(), "Expected no collision");

    }

    #[test]
    fn box_box_rotated_45_overlap() {
        let box_a = BoxShape { half_extents: Vec2::new(1.0, 1.0) };
        let box_b = BoxShape { half_extents: Vec2::new(1.0, 1.0) };
        let xf_a = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_b = Transform::new(Vec2::new(1.2, 0.0), std::f32::consts::FRAC_PI_4);
        let manifold = sat_box_vs_box(&box_a, &xf_a, &box_b, &xf_b).expect("Expected collision");
        let valid_normals = [
            xf_a.apply_vec(Vec2::new(1.0, 0.0)).normalize_or_zero(),
            xf_a.apply_vec(Vec2::new(0.0, 1.0)).normalize_or_zero(),
            xf_b.apply_vec(Vec2::new(1.0, 0.0)).normalize_or_zero(),
            xf_b.apply_vec(Vec2::new(0.0, 1.0)).normalize_or_zero(),
        ];
        assert!(valid_normals.iter().any(|&n| (manifold.normal - n).len() < 0.1), "Expected normal to be one of the face normals");
        assert!(manifold.points[0].depth > 0.1 && manifold.points[0].depth < 0.3, "Expected depth ≈ 0.2");
    }

    #[test]
    fn box_box_touching_edge_no_overlap() {
        // GIVEN: Box A [-1,1]×[-1,1]  Box B [1,3]×[-1,1]  (touching at x=1)
        // THEN:  None  (zero overlap → no collision reported)
        let box_a = BoxShape { half_extents: Vec2::new(1.0, 1.0) };
        let box_b = BoxShape { half_extents: Vec2::new(1.0, 1.0) };
        let xf_a = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_b = Transform::new(Vec2::new(2.0, 0.0), 0.0);
        let manifold = sat_box_vs_box(&box_a, &xf_a, &box_b, &xf_b);
        assert!(manifold.is_none(), "Expected no collision");
    }

    #[test]
    fn box_box_contact_has_correct_count() {
        let box_a = BoxShape { half_extents: Vec2::new(1.0, 1.0) };
        let box_b = BoxShape { half_extents: Vec2::new(1.0, 1.0) };
        let xf_a = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let xf_b = Transform::new(Vec2::new(0.0, 0.0), 0.0);
        let manifold = sat_box_vs_box(&box_a, &xf_a, &box_b, &xf_b).expect("Expected collision");
        assert_eq!(manifold.count, 2, "Expected count == 2 for full edge-edge contact");

    }
}