#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Material {
    pub friction: f32,
    pub restitution: f32,
    pub density: f32,
}

impl Default for Material {
    fn default() -> Self {
        Self {
            friction: 0.5,
            restitution: 0.0,
            density: 1.0,
        }
    }
}
