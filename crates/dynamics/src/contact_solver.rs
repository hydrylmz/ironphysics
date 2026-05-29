use physics_math::Vec2;
use physics_collision::ContactManifold;
use crate::body_view::BodyPosition;
use crate::config::WorldConfig;
use crate::constraint::VelocityConstraint;

pub fn pre_solve_contact_normal(
    manifold:  &ContactManifold,
    point_idx: usize,
    pos_a:     &BodyPosition,
    pos_b:     &BodyPosition,
    dt:        f32,
    config:    &WorldConfig,
) -> VelocityConstraint {
    // Build the velocity constraint for one contact point's NORMAL direction.
    //
    // ── Geometry setup ───────────────────────────────────────────────
    //   n = manifold.normal               (world-space, points A → B)
    //   p = manifold.points[point_idx].point  (world-space contact position)
    //
    //   r_a = p - pos_a.position          (arm from A center of mass to contact)
    //   r_b = p - pos_b.position          (arm from B center of mass to contact)
    //
    // ── Jacobian ─────────────────────────────────────────────────────
    //   For the non-penetration constraint: relative velocity along n ≥ 0
    //
    //   j_lin_a =  n                      (linear component for A)
    //   j_ang_a =  r_a.cross(n)           (angular component for A)
    //             = r_a.x * n.y - r_a.y * n.x
    //   j_lin_b = -n                      (linear component for B, opposite sign)
    //   j_ang_b = -r_b.cross(n)           (angular component for B)
    //
    //   Physical meaning:
    //     The constraint "bodies must not overlap" means the relative closing velocity
    //     projected onto the normal must be non-negative.
    //     J · v_rel = v_b - v_a + (ω_b × r_b - ω_a × r_a) projected onto n
    //
    // ── Effective mass ────────────────────────────────────────────────
    //   eff_mass = 1 / (J · M^-1 · J^T)
    //
    //   Expanded:
    //     denom  = pos_a.inv_mass
    //            + pos_b.inv_mass
    //            + (j_ang_a * j_ang_a) * pos_a.inv_inertia
    //            + (j_ang_b * j_ang_b) * pos_b.inv_inertia
    //     eff_mass = if denom > EPSILON { 1.0 / denom } else { 0.0 }
    //
    //   Intuition: a very heavy body has large mass and small inv_mass → small denominator
    //   → large eff_mass → a bigger impulse is needed to change its velocity.
    //
    // ── Bias (target closing velocity) ───────────────────────────────
    //   The bias combines two goals:
    //
    //   Goal 1 — Position correction (Baumgarte stabilization):
    //     Without this, numerical drift causes bodies to slowly sink through each other.
    //     baumgarte_bias = -(config.baumgarte_factor / dt)
    //                     * max(0.0, depth - config.linear_slop)
    //     Where depth = manifold.points[point_idx].depth
    //     linear_slop  = small allowed penetration (e.g. 0.005m) to prevent jitter
    //
    //   Goal 2 — Restitution (bounce):
    //     Compute current closing velocity:
    //       v_rel = (vel_b.linear + perp(r_b) * vel_b.angular)
    //              -(vel_a.linear + perp(r_a) * vel_a.angular)
    //       v_n   = v_rel.dot(n)   (closing speed along normal)
    //     Apply restitution only if closing fast enough:
    //       if v_n < -config.restitution_threshold:
    //         restitution_bias = manifold.restitution * v_n
    //         (target: post-impact velocity = e * v_n, so bias = -e * v_n corrects toward that)
    //       else:
    //         restitution_bias = 0.0
    //
    //   Total bias = baumgarte_bias + restitution_bias
    //   (Both are negative: they push bodies apart)
    //
    //   NOTE: We do NOT have vel_a/vel_b available here; pass them as parameters
    //   or split this into two sub-functions. Pass BodyVelocity references as well.
    //
    // ── Warm start ────────────────────────────────────────────────────
    //   impulse = manifold.points[point_idx].normal_impulse  (from last frame, or 0)
    //   lo = 0.0        (can only push, never pull)
    //   hi = f32::MAX   (no upper limit on compression impulse)
    //
    // ── Body indices ──────────────────────────────────────────────────
    //   body_a_idx = slot of manifold.body_a
    //   body_b_idx = slot of manifold.body_b
    let n = manifold.normal;
    let p = manifold.points[point_idx].point;
    let r_a = p - pos_a.position;
    let r_b = p - pos_b.position;
    let j_lin_a =  n;
    let j_ang_a =  r_a.x * n.y - r_a.y * n.x;
    let j_lin_b = -n;
    let j_ang_b = -r_b.x * n.y + r_b.y * n.x;
    let denom  = pos_a.inv_mass
              + pos_b.inv_mass
              + (j_ang_a * j_ang_a) * pos_a.inv_inertia
              + (j_ang_b * j_ang_b) * pos_b.inv_inertia;
    let eff_mass = if denom > f32::EPSILON { 1.0 / denom } else { 0.0 };
    let baumgarte_bias = -(config.baumgarte_factor / dt)
                      * (manifold.points[point_idx].depth - config.linear_slop).max(0.0);
    let bias = baumgarte_bias; // + restitution_bias (compute in solve phase when we have velocities)
    let impulse = manifold.points[point_idx].normal_impulse;
    let lo = 0.0;
    let hi = f32::MAX;
    let body_a_idx = manifold.body_a.slot();
    let body_b_idx = manifold.body_b.slot();
    let friction = manifold.friction;
    VelocityConstraint {
        j_lin_a,
        j_ang_a,
        j_lin_b,
        j_ang_b,
        eff_mass,
        bias,
        impulse,
        lo,
        hi,
        body_a_idx,
        body_b_idx,
        friction,
    }
}

pub fn pre_solve_contact_tangent(
    manifold:         &ContactManifold,
    point_idx:        usize,
    pos_a:            &BodyPosition,
    pos_b:            &BodyPosition,
    _normal_eff_mass: f32,    // passed in from the normal constraint
) -> VelocityConstraint {
    let n = manifold.normal;
    let t = Vec2::new(-n.y, n.x); // perpendicular to n
    let p = manifold.points[point_idx].point;
    let r_a = p - pos_a.position;
    let r_b = p - pos_b.position;
    let j_lin_a =  t;
    let j_ang_a =  r_a.x * t.y - r_a.y * t.x;
    let j_lin_b = -t;
    let j_ang_b = -r_b.x * t.y + r_b.y * t.x;
    let denom  = pos_a.inv_mass
              + pos_b.inv_mass
              + (j_ang_a * j_ang_a) * pos_a.inv_inertia
              + (j_ang_b * j_ang_b) * pos_b.inv_inertia;
    let eff_mass = if denom > 0.0 { 1.0 / denom } else { 0.0 };
    let bias = 0.0;
    let impulse = manifold.points[point_idx].tangent_impulse;
    let lo = -f32::MAX; // will be clamped to -µ *
    let hi = f32::MAX;  // will be clamped to µ *
    let body_a_idx = manifold.body_a.slot();
    let body_b_idx = manifold.body_b.slot();
    let friction = manifold.friction;
    VelocityConstraint {
        j_lin_a,
        j_ang_a,
        j_lin_b,
        j_ang_b,
        eff_mass,
        bias,
        impulse,
        lo,
        hi,
        body_a_idx,
        body_b_idx,
        friction,
    }

}

pub fn update_friction_bounds(
    constraints:        &mut crate::constraint::ConstraintStorage,
    tangent_idx:        usize,
    normal_accumulated: f32, 
    friction:           f32,    
) {
    let max_friction = friction * normal_accumulated.abs();
    constraints.lo[tangent_idx] = -max_friction;
    constraints.hi[tangent_idx] =  max_friction;

}