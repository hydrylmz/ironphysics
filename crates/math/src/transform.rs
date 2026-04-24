#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct Transform {
    pub position: Vec2,
    pub rotation: f32,  
}

pub fn identity() -> Self {
    position = Vec2::zero();
    rotation = 0.0;
}

pub fn new(position: Vec2, rotation: f32) -> Self {
    Transform { position, rotation }
}

pub fn rotation_mat(&self) -> Mat2 {
    // NOTE: This calls cos/sin every time and itss NOT cached.
    // This'll prolly be a bottleneck if we have a lot of Transforms, so we should consider
    // add a cached rot: Mat2 field to transform and set dirty flag.
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

