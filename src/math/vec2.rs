pub struct Vector2 {
    pub x: f32,
    pub y: f32,
}

impl Vector2 {
    // Creates a new instance of Vector2 which takes x and y floating points for arguments
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
    // Returns the length of the given Vector2 reference
    pub fn length(&self) -> f32 {
        ((self.x * self.x) + (self.y * self.y)).sqrt()
    }
    // Vector2 summation
    // Given an another Vector2 reference, it adds its values to the caller Vector2
    pub fn add(&mut self, other: &Vector2) -> &Self {
        self.x += other.x;
        self.y += other.y;
        self
    }
    // Vector2 subtraction
    // Given an another Vector2 reference, it subtracts its values to the caller Vector2
    pub fn subtract(&mut self, other: &Vector2) -> &Self {
        self.x -= other.x;
        self.y -= other.y;
        self
    }
    // Vector2 scalar operation
    // It takes the scale floating point as an argument, it scales the given Vector2
    pub fn scale(&mut self, other: f32) -> &Self {
        self.x *= other;
        self.y *= other;
        self
    }
    // Vector2 dot operation
    // It returns the dot of given Vector2 references
    pub fn dot(&self, other: &Vector2) -> f32 {
        (self.x * other.x) + (self.y * other.y)
    }
    // Vector2 normalization operation
    // It normalizes the given Vector2
    pub fn normalize(&mut self) -> &Self {
        let len = self.length();
        if len != 0.0 {
            self.x /= len;
            self.y /= len;
        }
        self
    }
}
