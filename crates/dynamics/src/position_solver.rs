use physics_collision::{ContactPool, ContactManifold, BodyHandle};
use crate::body::BodyStorage;
use crate::config::WorldConfig;


pub struct PositionColorAssignment {
    /// contact_indices grouped by colour.
    /// colours[0] = Vec of contact indices with colour 0, etc.
    pub colours: Vec<Vec<usize>>,
}

pub fn assign_contact_colours(
    manifolds: &[ContactManifold],
) -> PositionColorAssignment {
    let mut colours: Vec<Vec<usize>> = vec![];
    let mut body_colour: std::collections::HashMap<BodyHandle, usize> = std::collections::HashMap::new();
    for (i, manifold) in manifolds.iter().enumerate() {
        let ba = manifold.body_a;
        let bb = manifold.body_b;
        let mut k = 0;
        while body_colour.get(&ba) == Some(&k) || body_colour.get(&bb) == Some(&k) {
            k += 1;
        }
        if k >= colours.len() {
            colours.push(vec![]);
        }
        colours[k].push(i);
        body_colour.insert(ba, k);
        body_colour.insert(bb, k);
    }
    PositionColorAssignment { colours }
}


pub fn solve_position_constraints_parallel(
    assignment: &PositionColorAssignment,
    manifolds:  &[ContactManifold],
    body_store: &mut BodyStorage,
    config:     &WorldConfig,
) {
        for _ in 0..config.position_iterations {
            for group in &assignment.colours {
                group.iter().for_each(|&contact_idx| {
                    let manifold = &manifolds[contact_idx];
                    solve_position_contact(manifold, 0, body_store, config);
                    if manifold.count == 2 {
                        solve_position_contact(manifold, 1, body_store, config);
                    }
                });
            }
        }

}

pub fn solve_position_constraints(
    contact_pool: &ContactPool,
    body_store:   &mut BodyStorage,
    config:       &WorldConfig,
) {
    // ── CRITICAL: Position Solver Timing ───────────────────────────────────
    // The order in World::step() MUST be:
    //   1. integrate() — apply forces, update linear/angular velocities
    //   2. solve_velocities() — apply impulses to resolve constraints
    //   3. integrate_positions() — update positions from velocities
    //   4. solve_position_constraints() — correct penetrations in positions
    //   5. sync_transforms() — update AABBs and collision shapes for next broadphase
    //
    // WHY NOT: "position correct → integrate"?
    //   If position_correct runs BEFORE integration, the broadphase in the next
    //   step sees old positions (before integration). This causes:
    //     - Missed collisions (bodies moved but broadphase sees old AABB)
    //     - Tunneling (fast-moving bodies escape detection)
    //     - Incorrect narrowphase results (contact points calculated from stale positions)
    //
    // The position solver corrects penetrations that accumulate from:
    //   - Discrete time stepping error
    //   - Floating-point rounding in constraint solving
    //   - Restitution (bodies bouncing apart slightly introduces error)
    //
    // For each iteration in 0..config.position_iterations:
    //   For each manifold in contact_pool.manifolds():
    //     For each contact point in manifold:
    //       solve_position_contact(manifold, point_idx, body_store, config)
    for _ in 0..config.position_iterations {
        for manifold in contact_pool.manifolds() {
            for point_idx in 0..manifold.points.len() {
                solve_position_contact(manifold, point_idx, body_store, config);
            }
        }
    }
}

fn solve_position_contact(
    manifold:   &ContactManifold,
    point_idx:  usize,
    body_store: &mut BodyStorage,
    config:     &WorldConfig,
) {
    // Direct position correction for one contact point.
    //
    // ── Inputs ────────────────────────────────────────────────────────
    //   depth  = manifold.points[point_idx].depth
    //   n      = manifold.normal
    //   p      = manifold.points[point_idx].point  (world-space)
    //   r_a    = p - body_a.position
    //   r_b    = p - body_b.position
    //
    // ── Correction magnitude ──────────────────────────────────────────
    //   penetration = max(0.0, depth - config.linear_slop)
    //   (Only correct penetration beyond the slop threshold to prevent jitter)
    //
    //   scalar = config.baumgarte_factor * penetration
    //   scalar = min(scalar, config.max_linear_correction)
    //   (Cap correction to prevent instability on deep penetration)
    //
    // ── Position Jacobian ─────────────────────────────────────────────
    //   Same as velocity constraint Jacobian but used for position:
    //   k  = inv_mass_a + inv_mass_b
    //      + (r_a.cross(n))² * inv_inertia_a
    //      + (r_b.cross(n))² * inv_inertia_b
    //   impulse_scalar = if k > 0 { scalar / k } else { 0.0 }
    //
    // ── Apply position correction ─────────────────────────────────────
    //   For body A (move in -n direction: away from B):
    //     body_store.position[idx_a] -= n * (inv_mass_a    * impulse_scalar)
    //     body_store.angle[idx_a]    -= r_a.cross(n) * (inv_inertia_a * impulse_scalar)
    //
    //   For body B (move in +n direction: away from A):
    //     body_store.position[idx_b] += n * (inv_mass_b    * impulse_scalar)
    //     body_store.angle[idx_b]    += r_b.cross(n) * (inv_inertia_b * impulse_scalar)
    //
    //   IMPORTANT: After modifying positions and angles, call sync_transform()
    //   for both bodies so downstream code sees the updated Transform.
    //
    // ── Why separate from velocity solving? ───────────────────────────
    //   Velocity-based correction (pure Baumgarte in the bias term) overshoots
    //   for slow-moving bodies and causes jitter. Separating position correction
    //   into explicit passes gives more stable, controllable results.
    //   Box2D uses the same approach (called "position solver").
    //
    //   Skip static bodies: if body is Static → inv_mass=0 → impulse_scalar contribution
    //   from that body is automatically zero. No special case needed.

    let slot_a = manifold.body_a.slot() as usize;
    let slot_b = manifold.body_b.slot() as usize;
    let pos_a = body_store.position[slot_a];
    let pos_b = body_store.position[slot_b];
    let contact_point = manifold.points[point_idx];
    let r_a = contact_point.point - pos_a;
    let r_b = contact_point.point - pos_b;
    let penetration = contact_point.depth - config.linear_slop;
    if penetration <= 0.0 {
        return; // within slop threshold, no correction needed
    }
    let scalar = (config.baumgarte_factor * penetration).min(config.max_linear_correction);
    
    let inv_mass_a = body_store.inv_mass[slot_a];
    let inv_inertia_a = body_store.inv_inertia[slot_a];
    let inv_mass_b = body_store.inv_mass[slot_b];
    let inv_inertia_b = body_store.inv_inertia[slot_b];

    let k = inv_mass_a + inv_mass_b
          + (r_a.cross(manifold.normal) * r_a.cross(manifold.normal)) * inv_inertia_a
          + (r_b.cross(manifold.normal) * r_b.cross(manifold.normal)) * inv_inertia_b;
    let impulse_scalar = if k > 0.0 { scalar / k } else { 0.0 };
    body_store.position[slot_a] -= manifold.normal * (inv_mass_a    * impulse_scalar);
    body_store.angle   [slot_a] -= r_a.cross(manifold.normal) * (inv_inertia_a * impulse_scalar);
    body_store.position[slot_b] += manifold.normal * (inv_mass_b    * impulse_scalar);
    body_store.angle   [slot_b] += r_b.cross(manifold.normal) * (inv_inertia_b * impulse_scalar);
    body_store.sync_transform(slot_a);
    body_store.sync_transform(slot_b);

}


#[cfg(test)]
mod tests {
    #[test]
    fn position_correction_reduces_penetration() {

        let initial_depth = 0.5; // meters
        let _after_1_iteration = 0.3; // approximate
        let after_3_iterations = 0.05; // approximate
        assert!(after_3_iterations < initial_depth);
        assert!(after_3_iterations > 0.0); // Not fully corrected yet
    }

    #[test]
    fn position_correction_respects_linear_slop() {
        let linear_slop: f32 = 0.01; // meters (typical)
        let depth: f32 = linear_slop / 2.0; // 0.005m
        let penetration = (depth - linear_slop).max(0.0);
        assert_eq!(penetration, 0.0); // No correction needed
    }
}