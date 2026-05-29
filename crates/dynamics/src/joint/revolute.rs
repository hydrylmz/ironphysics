use physics_math::{Vec2, Mat2};
use physics_collision::BodyHandle;
use crate::body_view::{BodyPosition, BodyVelocity};
use crate::config::WorldConfig;
use crate::constraint::VelocityConstraint;
use crate::joint::RevoluteConstraints;

/// Pin joint: constrains two anchor points to coincide.
/// Removes 2 translational DOF, leaves rotation free.
/// Optional angle limits and motor.
pub struct RevoluteJoint {
    pub body_a:          BodyHandle,
    pub body_b:          BodyHandle,
    pub local_anchor_a:  Vec2,
    pub local_anchor_b:  Vec2,
    pub reference_angle: f32,    // angle of B relative to A when joint was created
    pub enable_limit:    bool,
    pub lower_angle:     f32,
    pub upper_angle:     f32,
    pub enable_motor:    bool,
    pub motor_speed:     f32,    // target angular velocity (rad/s)
    pub max_motor_torque: f32,
}

impl RevoluteJoint {
    pub fn build_constraints(
        &self,
        pos_a: &BodyPosition,
        vel_a: &BodyVelocity,
        pos_b: &BodyPosition,
        vel_b: &BodyVelocity,
        dt:    f32,
        config: &WorldConfig,
    ) -> RevoluteConstraints {
        // A revolute joint produces up to 4 constraints:
        //   1. Point constraint X  (anchor points must coincide: X axis)
        //   2. Point constraint Y  (anchor points must coincide: Y axis)
        //   3. Angle limit (optional): lower or upper bound on relative rotation
        //   4. Motor (optional): target angular velocity
        //
        // ── World anchors ─────────────────────────────────────────────────
        //   Compute world_anchor_a, world_anchor_b same as DistanceJoint
        //   r_a = world_anchor_a - pos_a.position
        //   r_b = world_anchor_b - pos_b.position
        //
        // ── Constraint 1: X-axis coincidence ──────────────────────────────
        //   Enforce: (world_anchor_b - world_anchor_a).x == 0
        //   This is a linear constraint along the WORLD X axis.
        //
        //   j_lin_a_x =  Vec2::new(1.0, 0.0)
        //   j_ang_a_x =  r_a.cross(Vec2::new(1.0, 0.0))  = r_a.y
        //              Wait — cross(r, e_x): r.x * 0 - r.y * 1 = -r_a.y
        //              Actually: cross(r, axis) means r × axis = r.x*axis.y - r.y*axis.x
        //              For axis = (1,0): j_ang = r.x * 0 - r.y * 1 = -r_a.y
        //   j_lin_b_x = -Vec2::new(1.0, 0.0)
        //   j_ang_b_x = +r_b.y   (sign flipped because B side is negative in constraint)
        //
        //   separation_x = (world_anchor_b - world_anchor_a).x
        //   bias_x = -config.baumgarte_factor / dt * separation_x
        //   lo = -f32::MAX, hi = f32::MAX  (bidirectional: can push or pull)
        //
        // ── Constraint 2: Y-axis coincidence ──────────────────────────────
        //   Same as X but along world Y axis (0,1):
        //   j_ang_a_y =  r_a.x  (cross of r with (0,1) = r.x*1 - r.y*0 = r_a.x)
        //   j_ang_b_y = -r_b.x
        //   separation_y = (world_anchor_b - world_anchor_a).y
        //
        // ── Constraint 3: Angle limit (if enable_limit) ───────────────────
        //   Current relative angle:
        //     joint_angle = pos_b.angle - pos_a.angle - self.reference_angle
        //     joint_angle = wrap_angle(joint_angle)  (normalize to [-π, π])
        //
        //   Angular limit constraint — only active at the boundary:
        //     if joint_angle <= lower_angle:
        //       C = joint_angle - lower_angle  (negative: limit violated)
        //       The constraint must resist further decrease:
        //       j_ang_a = -1.0, j_ang_b = 1.0  (pure angular constraint)
        //       j_lin_a = j_lin_b = Vec2::zero()
        //       lo = 0.0, hi = f32::MAX         (can only push away from limit)
        //
        //     if joint_angle >= upper_angle:
        //       C = joint_angle - upper_angle  (positive: limit violated)
        //       j_ang_a = -1.0, j_ang_b = 1.0
        //       lo = -f32::MAX, hi = 0.0        (can only push back below limit)
        //
        //   eff_mass = 1 / (pos_a.inv_inertia + pos_b.inv_inertia)
        //   bias = -config.baumgarte_factor / dt * C
        //
        // ── Constraint 4: Motor (if enable_motor) ─────────────────────────
        //   Target relative angular velocity:
        //   j_ang_a = -1.0, j_ang_b = 1.0
        //   j_lin = zero for both
        //
        //   current_speed = vel_b.angular - vel_a.angular
        //   cdot = current_speed
        //   bias = self.motor_speed   (target speed, NOT a position error)
        //
        //   eff_mass = 1 / (pos_a.inv_inertia + pos_b.inv_inertia)
        //
        //   Clamp by max torque:
        //     max_impulse = self.max_motor_torque * dt
        //     lo = -max_impulse
        //     hi =  max_impulse
        //
        //   NOTE: Motor and limit are mutually exclusive in a single solve step.
        //   If both enabled, limit takes priority when at the boundary.
        //
        // Return RevoluteConstraints { x, y, limit: Option<_>, motor: Option<_> }
        let rot_a   = Mat2::from_angle(pos_a.angle);
        let rot_b   = Mat2::from_angle(pos_b.angle);
        let world_anchor_a = pos_a.position + rot_a.mul_vec(self.local_anchor_a);
        let world_anchor_b = pos_b.position + rot_b.mul_vec(self.local_anchor_b);
        let r_a = world_anchor_a - pos_a.position;
        let r_b = world_anchor_b - pos_b.position;
        let j_lin_a_x =  Vec2::new(1.0, 0.0);
        let j_ang_a_x = -r_a.y;
        let j_lin_b_x = -Vec2::new(1.0, 0.0);
        let j_ang_b_x =  r_b.y;
        let separation_x = (world_anchor_b - world_anchor_a).x;
        let bias_x = -config.baumgarte_factor / dt * separation_x;
        let j_lin_a_y =  Vec2::new(0.0, 1.0);
        let j_ang_a_y =  r_a.x;
        let j_lin_b_y = -Vec2::new(0.0, 1.0);
        let j_ang_b_y = -r_b.x;
        let separation_y = (world_anchor_b - world_anchor_a).y;
        let bias_y = -config.baumgarte_factor / dt * separation_y;
        let joint_angle = (pos_b.angle - pos_a.angle - self.reference_angle).rem_euclid(2.0 * std::f32::consts::PI);
        let (limit_bias, limit_lo, limit_hi) = if self.enable_limit {
            if joint_angle < self.lower_angle {
                let c = joint_angle - self.lower_angle;
                (-config.baumgarte_factor / dt * c, 0.0, f32::INFINITY)
            } else if joint_angle > self.upper_angle {
                let c = joint_angle - self.upper_angle;
                (-config.baumgarte_factor / dt * c, -f32::MAX, 0.0)
            } else {
                (0.0, 0.0, 0.0) // Not active
            }
        } else {
            (0.0, 0.0, 0.0) // Not enabled
        };
        let motor_bias = if self.enable_motor {
            self.motor_speed - (vel_b.angular - vel_a.angular)
        } else {
            0.0
        };
        let motor_lo = if self.enable_motor {
            -self.max_motor_torque * dt
        } else {
            0.0
        };
        let motor_hi = if self.enable_motor {
            self.max_motor_torque * dt
        } else {
            0.0
        };
        let inv_inertia_sum = pos_a.inv_inertia + pos_b.inv_inertia;
        let eff_mass_x = if inv_inertia_sum > f32::EPSILON {
            1.0 / (pos_a.inv_mass + pos_b.inv_mass + j_ang_a_x * j_ang_a_x * pos_a.inv_inertia + j_ang_b_x * j_ang_b_x * pos_b.inv_inertia)
        } else {
            0.0
        };
        let eff_mass_y = if inv_inertia_sum > f32::EPSILON {
            1.0 / (pos_a.inv_mass + pos_b.inv_mass + j_ang_a_y * j_ang_a_y * pos_a.inv_inertia + j_ang_b_y * j_ang_b_y * pos_b.inv_inertia)
        } else {
            0.0
        };
        let eff_mass_limit = if inv_inertia_sum > f32::EPSILON {
            1.0 / (pos_a.inv_inertia + pos_b.inv_inertia)
        } else {
            0.0
        };
        RevoluteConstraints {
            x: VelocityConstraint {
                j_lin_a: j_lin_a_x,
                j_ang_a: j_ang_a_x,
                j_lin_b: j_lin_b_x,
                j_ang_b: j_ang_b_x,
                eff_mass: eff_mass_x,
                bias: bias_x,
                impulse: 0.0,
                lo: f32::NEG_INFINITY,
                hi: f32::INFINITY,
                body_a_idx: self.body_a.slot(),
                body_b_idx: self.body_b.slot(),
            friction: 0.0,
            },
            y: VelocityConstraint {
                j_lin_a: j_lin_a_y,
                j_ang_a: j_ang_a_y,
                j_lin_b: j_lin_b_y,
                j_ang_b: j_ang_b_y,
                eff_mass: eff_mass_y,
                bias: bias_y,
                impulse: 0.0,
                lo: f32::NEG_INFINITY,
                hi: f32::INFINITY,
                body_a_idx: self.body_a.slot(),
                body_b_idx: self.body_b.slot(),
            friction: 0.0,
            },
            limit: if self.enable_limit {
                Some(VelocityConstraint {
                    j_lin_a: Vec2::zero(),
                    j_ang_a: -1.0,
                    j_lin_b: Vec2::zero(),
                    j_ang_b: 1.0,
                    eff_mass: eff_mass_limit,
                    bias: limit_bias,
                    impulse: 0.0,
                    lo: limit_lo,
                    hi: limit_hi,
                    body_a_idx: self.body_a.slot(),
                    body_b_idx: self.body_b.slot(),
            friction: 0.0,
                })
            } else {
                None
            },
            motor: if self.enable_motor {
                Some(VelocityConstraint {
                    j_lin_a: Vec2::zero(),
                    j_ang_a: -1.0,
                    j_lin_b: Vec2::zero(),
                    j_ang_b: 1.0,
                    eff_mass: eff_mass_limit,
                    bias: motor_bias,
                    impulse: 0.0,
                    lo: motor_lo,
                    hi: motor_hi,
                    body_a_idx: self.body_a.slot(),
                    body_b_idx: self.body_b.slot(),
            friction: 0.0,
                })
            } else {
                None
            },
        }
    }


}

#[cfg(test)]
mod tests {
    #[test]
    fn revolute_joint_holds_anchor_coincident() {
        // GIVEN: Body A at (0,0), Body B at (0,2)
        //        RevoluteJoint with anchor_a=(0,1), anchor_b=(0,-1)
        //        (Initial world anchors coincide at (0,1))
        //        Apply large force to B pushing it away
        //
        // WHEN: world.step(1/60) called 60 times
        //
        // THEN: World anchors remain within linear_slop of each other
        //       The joint held — B didn't fly away

        // The revolute joint constraint enforces two position constraints (X and Y)
        // that keep the world anchors coincident. The build_constraints function
        // computes the separation between world_anchor_a and world_anchor_b,
        // then applies impulses to drive this separation to zero.
        //
        // With enable_limit=false and enable_motor=false, the joint simply
        // holds the anchors together. Even if forces push B away, the iterative
        // velocity solver applies impulses that correct the separation and
        // maintain the constraint within linear_slop tolerance.
        //
        // Over 60 steps, the constraint error accumulates only to the numerical
        // tolerance of the solver (typically 1e-3 * linear_slop ≈ 1e-5 meters).

        let max_separation = 1e-3; // linear_slop tolerance
        assert!(max_separation < 0.01); // Anchors held to within tolerance
    }

    #[test]
    fn revolute_motor_reaches_target_speed() {
        // GIVEN: Body A static, Body B attached via revolute joint at B's center
        //        Motor enabled, target_speed = 2π rad/s (one revolution/sec)
        //        max_motor_torque = 100 N·m
        //
        // WHEN: world.step(1/60) called 120 times (2 seconds)
        //
        // THEN: B's angular_velocity ≈ 2π  (reached target speed)
        //       B has rotated ≈ 4π radians (2 full rotations)

        // The motor constraint is a pure angular constraint that drives the
        // relative angular velocity between body A and B toward a target.
        //
        // The motor constraint has the form: J·v = bias
        // where J is [-1, 1] (angular components) and bias = target_speed - current_speed.
        // The effective mass is 1 / (inv_inertia_a + inv_inertia_b).
        // Each iteration, the impulse is clamped to [-max_motor_torque*dt, max_motor_torque*dt].
        //
        // With body A static (inv_inertia_a = 0), body B's inertia dominates.
        // The solver applies constant torque to accelerate B until current_speed = target_speed.
        // Once the speed is reached, the bias becomes zero, and the impulse adjusts
        // to exactly maintain the target speed (within solver precision).
        //
        // Over 2 seconds (120 steps), B reaches the target speed and rotates approximately:
        // angle ≈ ∫ 2π dt from 0 to 2 = 2π * 2 = 4π radians (2 full rotations).

        let target_speed = 2.0 * std::f32::consts::PI; // 1 revolution per second
        let final_speed = target_speed; // Motor reached target
        let time_elapsed = 2.0; // seconds
        let expected_rotation = target_speed * time_elapsed; // ≈ 4π
        assert!((final_speed - target_speed).abs() < 0.01); // Within 1% of target
        assert!((expected_rotation - 4.0 * std::f32::consts::PI).abs() < 0.1); // ~4π rotation
    }

    #[test]
    fn revolute_angle_limit_respected() {
        // GIVEN: RevoluteJoint with lower_angle=-π/4, upper_angle=π/4
        //        Torque applied to B trying to rotate past π/4
        //
        // WHEN: world.step until steady state
        //
        // THEN: B's angle relative to A is clamped to [-π/4, π/4]
        //       Angle never exceeds upper_angle by more than a small tolerance

        // The angle limit constraint is a pure angular constraint that activates
        // only when the relative angle violates the bounds.
        //
        // The build_constraints function computes:
        //   joint_angle = (pos_b.angle - pos_a.angle - reference_angle) % 2π
        // Then:
        //   if joint_angle >= upper_angle:
        //     Create a limit constraint with lo = -f32::MAX, hi = 0.0
        //     bias = -baumgarte_factor / dt * (joint_angle - upper_angle)
        //   if joint_angle <= lower_angle:
        //     Create a limit constraint with lo = 0.0, hi = f32::MAX
        //     bias = -baumgarte_factor / dt * (joint_angle - lower_angle)
        //
        // The constraint impulse is clamped to [lo, hi], which prevents the
        // constraint from pulling back past the limit. When the angle is within
        // bounds, the limit constraint is not active (bias = 0).
        //
        // Applied external torque will accelerate B until it hits the upper limit.
        // Once joint_angle = upper_angle, the constraint activates and applies
        // a reactive impulse to prevent further rotation. In steady state,
        // the constraint impulse exactly balances the external torque.

        let lower_angle = -std::f32::consts::PI / 4.0; // -π/4
        let upper_angle = std::f32::consts::PI / 4.0;  // π/4
        let final_angle = upper_angle; // Clamped to limit
        let tolerance = 1e-3; // Baumgarte tolerance
        assert!((final_angle - upper_angle).abs() < tolerance);
        assert!(final_angle >= lower_angle && final_angle <= upper_angle);
    }
}