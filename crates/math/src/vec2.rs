#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}
impl Vec2 {
    pub fn new(x: f32, y: f32) -> Self {
        Vec2 { x, y }
    }

    pub fn zero() -> Self {
        Vec2 { x: 0.0, y: 0.0 }
    }

    pub fn splat(v: f32) -> Self {
        Vec2 { x: v, y: v }
    }

    pub fn dot(self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y
    }

    pub fn cross(self, rhs: Self) -> f32 {
        self.x * rhs.y - self.y * rhs.x
    }

    pub fn perp(self) -> Self {
        Vec2 { x: -self.y, y: self.x }
    }

    pub fn len_sq(self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    pub fn len(self) -> f32 {
        self.len_sq().sqrt()
    }

    pub fn normalize(self) -> Self {
        if self.x == 0.0 && self.y == 0.0 {
            panic!("Cannot normalize the zero vector");
        }
        let len = self.len();
        let inv_len = 1.0 / len;
        Vec2 {
            x: self.x * inv_len,
            y: self.y * inv_len,
        }
    }

    pub fn normalize_or_zero(self) -> Self {
        if self.len_sq() < super::scalar::EPSILON {
            Vec2::zero()
        } else {
            self.normalize()
        }
    }

    pub fn lerp(self, rhs: Self, t: f32) -> Self {
        Vec2 {
            x: self.x + t * (rhs.x - self.x),
            y: self.y + t * (rhs.y - self.y),
        }
    }

    pub fn abs(self) -> Self {
        Vec2 { x: self.x.abs(), y: self.y.abs() }
    }

    pub fn min_comp(self, rhs: Self) -> Self {
        Vec2 { x: self.x.min(rhs.x), y: self.y.min(rhs.y) }
    }

    pub fn max_comp(self, rhs: Self) -> Self {
        Vec2 { x: self.x.max(rhs.x), y: self.y.max(rhs.y) }
    }
}

impl std::ops::Add for Vec2 {
    type Output = Vec2;
    fn add(self, rhs: Vec2) -> Vec2 {
        Vec2 { x: self.x + rhs.x, y: self.y + rhs.y }
    }
}

impl std::ops::Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, rhs: Vec2) -> Vec2 {
        Vec2 { x: self.x - rhs.x, y: self.y - rhs.y }
    }
}

impl std::ops::Mul<f32> for Vec2 {
    type Output = Vec2;
    fn mul(self, scalar: f32) -> Vec2 {
        Vec2 { x: self.x * scalar, y: self.y * scalar }
    }
}

impl std::ops::Mul<Vec2> for f32 {
    type Output = Vec2;
    fn mul(self, v: Vec2) -> Vec2 {
        v * self
    }
}

impl std::ops::Div<f32> for Vec2 {
    type Output = Vec2;
    fn div(self, scalar: f32) -> Vec2 {
        // NOTE: does NOT guard against scalar == 0; caller's responsibility :3
        let inv = 1.0 / scalar;
        Vec2 { x: self.x * inv, y: self.y * inv }
    }
}

impl std::ops::Neg for Vec2 {
    type Output = Vec2;
    fn neg(self) -> Vec2 {
        Vec2 { x: -self.x, y: -self.y}
    }
}

impl std::ops::AddAssign for Vec2 {
    fn add_assign(&mut self, rhs: Vec2) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl std::ops::SubAssign for Vec2 {
    fn sub_assign(&mut self, rhs: Vec2) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl std::ops::MulAssign<f32> for Vec2 {
    fn mul_assign(&mut self, scalar: f32) {
        self.x *= scalar;
        self.y *= scalar;
    }
}

impl Default for Vec2 {
    fn default() -> Self {
        Vec2::zero()    
    }
}



#[cfg(test)]
mod tests {
    use super::*;
    use crate::scalar::EPSILON;

    #[test]
    fn vec2_add() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(3.0, 4.0);
        let c = a + b;
        assert_eq!(c.x, 4.0);
        assert_eq!(c.y, 6.0);
    }

    #[test]
    fn vec2_dot_perpendicular() {

        let a = Vec2::new(1.0, 0.0);
        let b = Vec2::new(0.0, 1.0);
        let d = a.dot(b);
        assert_eq!(d, 0.0);
    }

    #[test]
    fn vec2_dot_parallel() {
        let a = Vec2::new(2.0, 0.0);
        let b = Vec2::new(3.0, 0.0);
        let d = a.dot(b);
        assert_eq!(d, 6.0);
    }

    #[test]
    fn vec2_cross_ccw() {
        let a = Vec2::new(1.0, 0.0);
        let b = Vec2::new(0.0, 1.0);
        let c = a.cross(b);
        assert_eq!(c, 1.0);
    }

    #[test]
    fn vec2_cross_cw() {

        let a = Vec2::new(0.0, 1.0);
        let b = Vec2::new(1.0, 0.0);
        let c = a.cross(b);
        
        assert_eq!(c, -1.0);
    }

    #[test]
    fn vec2_perp_is_unit_for_unit_input() {

        let a = Vec2::new(1.0, 0.0);
        let p = a.perp();
        assert_eq!(p, Vec2::new(0.0, 1.0));
        assert!((p.len() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn vec2_normalize_unit_length() {

        let v = Vec2::new(3.0, 4.0);
        let n = v.normalize();
        assert!((n.len() - 1.0).abs() < EPSILON);
        assert!((n.x - 0.6).abs() < EPSILON);
        assert!((n.y - 0.8).abs() < EPSILON);

    }

    #[test]
    fn vec2_normalize_or_zero_on_zero_vec() {

        let v = Vec2::zero();
        let n = v.normalize_or_zero();
        assert_eq!(n, Vec2::zero());

    }

    #[test]
    fn vec2_len_sq_matches_len_squared() {

        let v = Vec2::new(3.0, 4.0);
        assert!((v.len_sq() - v.len() * v.len()).abs() < EPSILON);
        assert!((v.len_sq() - 25.0).abs() < EPSILON);
        assert!((v.len() - 5.0).abs() < EPSILON);

    }
}