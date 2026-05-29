pub mod distance;
pub mod revolute;
pub mod prismatic;

use crate::constraint::VelocityConstraint;

pub use distance::DistanceJoint;
pub use revolute::RevoluteJoint;
pub use prismatic::PrismaticJoint;

pub struct JointDesc;

/// Constraints produced by a revolute (pin) joint.
pub struct RevoluteConstraints {
    pub x:     VelocityConstraint,
    pub y:     VelocityConstraint,
    pub limit: Option<VelocityConstraint>,
    pub motor: Option<VelocityConstraint>,
}

/// Constraints produced by a prismatic (slider) joint.
pub struct PrismaticConstraints {
    pub constraints: Vec<VelocityConstraint>,
}
