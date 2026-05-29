use physics_math::Transform;
use crate::shape::{Shape, ShapeType};
use crate::narrowphase::manifold::ContactManifold;
use crate::narrowphase::analytic::{circle_vs_circle, circle_vs_box};
use crate::narrowphase::sat::sat_box_vs_box;
use crate::narrowphase::epa::gjk_epa_manifold;
use crate::{ColliderHandle, ColliderStorage, ContactPool};
use rayon::prelude::*;
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


pub fn run_narrowphase_parallel(
    pairs:      &[(ColliderHandle, ColliderHandle)],
    colliders:  &ColliderStorage,
    prev_pool:  &ContactPool,
    out_pool:   &mut ContactPool,
) {
    let mut result: Vec<Option<ContactManifold>> = pairs
        .par_iter()
        .map(|&(ha, hb): &(ColliderHandle, ColliderHandle)| {
            let ia = ha.slot();
            let ib = hb.slot();

            let xf_a = colliders.world_transform[ia];
            let xf_b = colliders.world_transform[ib];

            // SAFETY: ia != ib (filtered upstream), non-overlapping Box heap memory.
            let shape_a = unsafe { &*(&*colliders.shape[ia] as *const dyn Shape) };
            let shape_b = unsafe { &*(&*colliders.shape[ib] as *const dyn Shape) };

            let mut result = dispatch_narrowphase(shape_a, &xf_a, shape_b, &xf_b)?;

            // Fill in handles and material data inline (all reads, no writes)
            result.body_a      = colliders.body_handle[ia];
            result.body_b      = colliders.body_handle[ib];
            result.collider_a  = ha;
            result.collider_b  = hb; 
            // friction is stored per-collider; use the first collider's value
            // instead of calling combined_friction to match the stored type
            result.friction    = colliders.friction[ia];
            result.restitution = colliders.restitution[ia];
            Some(result)
        })
        .collect();

    for (_, manifold_opt) in pairs.iter().zip(result.iter_mut()) {
        if let Some(manifold) = manifold_opt {
            ContactPool::persist_contacts(prev_pool, manifold); 
            out_pool.insert(manifold.clone());
        }
    }
}

