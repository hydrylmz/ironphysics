use physics_math::Transform;
use crate::shape::{Shape, ShapeType};
use crate::narrowphase::manifold::ContactManifold;
use crate::narrowphase::analytic::{circle_vs_circle, circle_vs_box};
use crate::narrowphase::sat::sat_box_vs_box;
use crate::narrowphase::epa::gjk_epa_manifold;

pub fn dispatch_narrowphase(
    shape_a: &dyn Shape, xf_a: &Transform,
    shape_b: &dyn Shape, xf_b: &Transform,
) -> Option<ContactManifold> {
    let type_a = shape_a.shape_type();
    let type_b = shape_b.shape_type();
    if type_a > type_b {
        let manifold = dispatch_narrowphase(shape_b, xf_b, shape_a, xf_a)?;
        return Some(manifold.swapped());
    }
    match (type_a, type_b) {
        (ShapeType::Circle, ShapeType::Circle) => {
            let ca = shape_a.as_any().downcast_ref().unwrap();
            let cb = shape_b.as_any().downcast_ref().unwrap();
            circle_vs_circle(ca, xf_a, cb, xf_b)
        },
        (ShapeType::Circle, ShapeType::Box) => {
            let ca = shape_a.as_any().downcast_ref().unwrap();
            let cb = shape_b.as_any().downcast_ref().unwrap();
            circle_vs_box(ca, xf_a, cb, xf_b)
        },
        (ShapeType::Box, ShapeType::Box) => {
            let ca = shape_a.as_any().downcast_ref().unwrap();
            let cb = shape_b.as_any().downcast_ref().unwrap();
            sat_box_vs_box(ca, xf_a, cb, xf_b)
        },
        _ => gjk_epa_manifold(shape_a, xf_a, shape_b, xf_b),
    }
}
