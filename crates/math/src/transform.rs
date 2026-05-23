#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct Transform {
    pub position: Vec2,
    pub rotation: f32,  
}
use crate::vec2::Vec2;
use crate::mat2::Mat2;
impl Transform {
    pub fn identity() -> Self {
        Transform { position: Vec2::zero(), rotation: 0.0 }
    }

    pub fn new(position: Vec2, rotation: f32) -> Self {
        Transform { position, rotation }
    }

    pub fn rotation_mat(&self) -> Mat2 {
        Mat2::from_angle(self.rotation)
    }

    pub fn apply(&self, local_point: Vec2) -> Vec2 {
        self.rotation_mat().mul_vec(local_point) + self.position
    }

    pub fn apply_inv(&self, world_point: Vec2) -> Vec2 {
        let delta = world_point - self.position;
        let rot = self.rotation_mat();
        let rot_inv = rot.transpose();
        rot_inv.mul_vec(delta)
    }

    pub fn apply_vec(&self, local_vec: Vec2) -> Vec2 {
        self.rotation_mat().mul_vec(local_vec)
    }

    pub fn combine(&self, child: &Transform) -> Transform {
        let new_rotation = self.rotation + child.rotation;
        let new_position = self.apply(child.position);
        Transform { position: new_position, rotation: new_rotation }
    }
}

impl Default for Transform {
    fn default() -> Self {
        Transform::identity()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vec2::Vec2;
    use crate::scalar::EPSILON;

    #[test]
    fn transform_apply_zero_rotation() {
        // GIVEN: t = Transform::new(Vec2::new(5.0, 3.0), 0.0)
        //        p = Vec2::new(1.0, 0.0)
        // WHEN:  world = t.apply(p)
        // THEN:  world == Vec2::new(6.0, 3.0)  (only translation applied)
        let t = Transform::new(Vec2::new(5.0, 3.0), 0.0);
        let p = Vec2::new(1.0, 0.0);
        let world = t.apply(p);
        assert_eq!(world, Vec2::new(6.0, 3.0));

    }

    #[test]
    fn transform_apply_inv_roundtrip() {
        // GIVEN: t = Transform::new(Vec2::new(2.0, -1.0), 0.8)
        //        p = Vec2::new(3.0, 5.0)
        // WHEN:  world = t.apply(p)
        //        local = t.apply_inv(world)
        // THEN:  local ≈ p  (apply then apply_inv is identity, within EPSILON)
        let t = Transform::new(Vec2::new(2.0, -1.0), 0.8);
        let p = Vec2::new(3.0, 5.0);
        let world = t.apply(p);
        let local = t.apply_inv(world);
        assert!((local.x - p.x).abs() < EPSILON);
        assert!((local.y - p.y).abs() < EPSILON);

    }

    #[test]
    fn transform_combine_two_translations() {
        // GIVEN: parent = Transform::new(Vec2::new(1.0, 0.0), 0.0)
        //        child  = Transform::new(Vec2::new(2.0, 0.0), 0.0)
        // WHEN:  combined = parent.combine(&child)
        // THEN:  combined.position ≈ Vec2::new(3.0, 0.0)  (translations add)
        //        combined.rotation ≈ 0.0
        let parent = Transform::new(Vec2::new(1.0, 0.0), 0.0);
        let child = Transform::new(Vec2::new(2.0, 0.0), 0.0);
        let combined = parent.combine(&child);
        assert!((combined.position.x - 3.0).abs() < EPSILON);
        assert!((combined.position.y - 0.0).abs() < EPSILON);
        assert!((combined.rotation - 0.0).abs() < EPSILON);

    }

    #[test]
    fn transform_identity_apply_is_noop() {
        // GIVEN: t = Transform::identity()
        //        p = Vec2::new(7.0, -2.0)
        // THEN:  t.apply(p) == p
        //        t.apply_inv(p) == p
        let t = Transform::identity();
        let p = Vec2::new(7.0, -2.0);
        let world = t.apply(p);
        let local = t.apply_inv(p);
        assert_eq!(world, p);
        assert_eq!(local, p);
        
    }
}
