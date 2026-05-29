use physics_math::{Vec2, Mat2};
use physics_collision::BodyHandle;
use crate::body_view::{BodyPosition, BodyVelocity};
use crate::config::WorldConfig;
use crate::constraint::VelocityConstraint;
use crate::joint::PrismaticConstraints;

/// Slider joint: constrains motion to a single axis.
/// Removes 1 translational DOF and 1 rotational DOF (relative rotation is locked).
pub struct PrismaticJoint {
    pub body_a:              BodyHandle,
    pub body_b:              BodyHandle,
    pub local_anchor_a:      Vec2,
    pub local_anchor_b:      Vec2,
    pub local_axis_a:        Vec2,    // slide direction in A's local space (normalized)
    pub reference_angle:     f32,    // relative angle when joint was created
    pub enable_limit:        bool,
    pub lower_translation:   f32,
    pub upper_translation:   f32,
    pub enable_motor:        bool,
    pub motor_speed:         f32,
    pub max_motor_force:     f32,
}

impl PrismaticJoint {
    pub fn build_constraints(
        &self,
        pos_a: &BodyPosition,
        pos_b: &BodyPosition,
        vel_a: &BodyVelocity,
        vel_b: &BodyVelocity,
        dt:    f32,
        config: &WorldConfig,
    ) -> PrismaticConstraints {

        let rot_a   = Mat2::from_angle(pos_a.angle);
        let rot_b   = Mat2::from_angle(pos_b.angle);
        let world_anchor_a = pos_a.position + rot_a.mul_vec(self.local_anchor_a);
        let world_anchor_b = pos_b.position + rot_b.mul_vec(self.local_anchor_b);
        let d = world_anchor_b - world_anchor_a;
        let axis = rot_a.mul_vec(self.local_axis_a);
        let perp = Vec2::new(-axis.y, axis.x);
        
        let r_a = world_anchor_a - pos_a.position;
        let r_b = world_anchor_b - pos_b.position;

        let mut constraints = Vec::new();
        let separation_perp = d.dot(perp);
        constraints.push(VelocityConstraint {
            j_lin_a: -perp,
            j_ang_a: -(r_a + d).x * perp.y + (r_a + d).y * perp.x,
            j_lin_b:  perp,
            j_ang_b:  r_b.x * perp.y - r_b.y * perp.x,
            eff_mass: 1.0 / (pos_a.inv_mass + pos_b.inv_mass
                        + (r_a + d).dot(perp) * (r_a + d).dot(perp) * pos_a.inv_inertia
                        + r_b.dot(perp) * r_b.dot(perp) * pos_b.inv_inertia),
            bias: -config.baumgarte_factor / dt * separation_perp,
            impulse: 0.0,
            lo: f32::NEG_INFINITY,
            hi: f32::INFINITY,
            body_a_idx: self.body_a.slot(),
            body_b_idx: self.body_b.slot(),
            friction: 0.0,
        });
        let relative_angle = pos_b.angle - pos_a.angle - self.reference_angle;
        constraints.push(VelocityConstraint {
            j_lin_a: Vec2::zero(),
            j_ang_a: -1.0,
            j_lin_b: Vec2::zero(),
            j_ang_b:  1.0,
            eff_mass: 1.0 / (pos_a.inv_inertia + pos_b.inv_inertia),
            bias: -config.baumgarte_factor / dt * relative_angle,
            impulse: 0.0,
            lo: f32::NEG_INFINITY,
            hi: f32::INFINITY,
            body_a_idx: self.body_a.slot(),
            body_b_idx: self.body_b.slot(),
            friction: 0.0,
        });
        if self.enable_limit {
            let translation = d.dot(axis);
            if translation < self.lower_translation {
                let c = translation - self.lower_translation;
                constraints.push(VelocityConstraint {
                    j_lin_a: -axis,
                    j_ang_a: -(r_a + d).x * axis.y + (r_a + d).y * axis.x,
                    j_lin_b:  axis,
                    j_ang_b:  r_b.x * axis.y - r_b.y * axis.x,
                    eff_mass: 1.0 / (pos_a.inv_mass + pos_b.inv_mass
                                + (r_a + d).dot(axis) * (r_a + d).dot(axis) * pos_a.inv_inertia
                                + r_b.dot(axis) * r_b.dot(axis) * pos_b.inv_inertia),
                    bias: -config.baumgarte_factor / dt * c,
                    impulse: 0.0,
                    lo: 0.0,
                    hi: f32::INFINITY,
                    body_a_idx: self.body_a.slot(),
                    body_b_idx: self.body_b.slot(),
            friction: 0.0,
                });
            } else if translation > self.upper_translation {
                let c = translation - self.upper_translation;
                constraints.push(VelocityConstraint {
                    j_lin_a: -axis,
                    j_ang_a: -(r_a + d).x * axis.y + (r_a + d).y * axis.x,
                    j_lin_b:  axis,
                    j_ang_b:  r_b.x * axis.y - r_b.y * axis.x,
                    eff_mass: 1.0 / (pos_a.inv_mass + pos_b.inv_mass
                                + (r_a + d).dot(axis) * (r_a + d).dot(axis) * pos_a.inv_inertia
                                + r_b.dot(axis) * r_b.dot(axis) * pos_b.inv_inertia),
                    bias: -config.baumgarte_factor / dt * c,
                    impulse: 0.0,
                    lo: f32::NEG_INFINITY,
                    hi: 0.0,
                    body_a_idx: self.body_a.slot(),
                    body_b_idx: self.body_b.slot(),
            friction: 0.0,
                });
            }
        }
        if self.enable_motor {
            let current_velocity = (vel_b.linear - vel_a.linear).dot(axis)
                                + vel_b.angular * r_b.x * axis.y - vel_b.angular * r_b.y * axis.x
                                - vel_a.angular * (r_a + d).x * axis.y + vel_a.angular * (r_a + d).y * axis.x;
            let max_impulse = self.max_motor_force * dt;
            constraints.push(VelocityConstraint {
                j_lin_a: -axis,
                j_ang_a: -(r_a + d).x * axis.y + (r_a + d).y * axis.x,
                j_lin_b:  axis,
                j_ang_b:  r_b.x * axis.y - r_b.y * axis.x,
                eff_mass: 1.0 / (pos_a.inv_mass + pos_b.inv_mass
                            + (r_a + d).dot(axis) * (r_a + d).dot(axis) * pos_a.inv_inertia
                            + r_b.dot(axis) * r_b.dot(axis) * pos_b.inv_inertia),
                bias: self.motor_speed - current_velocity,
                impulse: 0.0,
                lo: -max_impulse,
                hi: max_impulse,
                body_a_idx: self.body_a.slot(),
                body_b_idx: self.body_b.slot(),
            friction: 0.0,
            });
        }
        PrismaticConstraints { constraints }
    }
}