use physics_math::{Vec2, Mat2};
use physics_collision::BodyHandle;
use crate::body_view::{BodyPosition, BodyVelocity};
use crate::constraint::VelocityConstraint;

/// Maintains a target distance between two anchor points.
/// Optionally acts as a spring (stiffness > 0) or rigid link (stiffness = 0).
pub struct DistanceJoint {
    pub body_a:      BodyHandle,
    pub body_b:      BodyHandle,
    pub anchor_a:    Vec2,       // local-space anchor on body A
    pub anchor_b:    Vec2,       // local-space anchor on body B
    pub min_length:  f32,        // minimum allowed distance
    pub max_length:  f32,        // maximum allowed distance
    pub stiffness:   f32,        // spring constant (0 = rigid)
    pub damping:     f32,        // spring damper coefficient
}

impl DistanceJoint {
    pub fn build_constraint(
        &self,
        pos_a: &BodyPosition,
        _vel_a: &BodyVelocity,
        pos_b: &BodyPosition,
        _vel_b: &BodyVelocity,
        dt:    f32,
    ) -> Option<VelocityConstraint> {
        let rot_a = Mat2::from_angle(pos_a.angle);
        let rot_b = Mat2::from_angle(pos_b.angle);

        let world_anchor_a = pos_a.position + rot_a.mul_vec(self.anchor_a);
        let world_anchor_b = pos_b.position + rot_b.mul_vec(self.anchor_b);

        let r_a = world_anchor_a - pos_a.position;
        let r_b = world_anchor_b - pos_b.position;

        let d = world_anchor_b - world_anchor_a;
        let l = d.len();

        if l < 1e-6 {
            return None;
        }

        let u = d * (1.0 / l);

        let (lo, hi, c) = if (self.max_length - self.min_length).abs() < 1e-4 {
            // Rigid
            (-f32::MAX, f32::MAX, l - self.max_length)
        } else if l < self.min_length {
            // Violated min
            (0.0, f32::MAX, l - self.min_length)
        } else if l > self.max_length {
            // Violated max
            (-f32::MAX, 0.0, l - self.max_length)
        } else {
            return None;
        };

        let mut baumgarte = 0.2;
        let mut compliance = 0.0;
        if self.stiffness > 0.0 {
            let k = self.stiffness;
            let d_val = self.damping;
            let denom = d_val + dt * k;
            if denom > 1e-6 {
                compliance = 1.0 / (dt * denom);
                baumgarte = dt * k / denom;
            }
        }

        let j_lin_a = u;
        let j_ang_a = r_a.cross(u);
        let j_lin_b = -u;
        let j_ang_b = -r_b.cross(u);

        let k_mass = pos_a.inv_mass + pos_b.inv_mass
            + j_ang_a * j_ang_a * pos_a.inv_inertia
            + j_ang_b * j_ang_b * pos_b.inv_inertia
            + compliance;

        if k_mass < 1e-6 {
            return None;
        }

        let eff_mass = 1.0 / k_mass;
        let bias = -baumgarte / dt * c;

        Some(VelocityConstraint {
            j_lin_a,
            j_ang_a,
            j_lin_b,
            j_ang_b,
            eff_mass,
            bias,
            impulse: 0.0,
            lo,
            hi,
            body_a_idx: self.body_a.slot(),
            body_b_idx: self.body_b.slot(),
            friction: 0.0,
        })
    }
}