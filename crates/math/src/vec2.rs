#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

#[inline]
pub fn new(x: f32, y: f32) -> Self {
    Vec2 { x, y }
}

#[inline]
pub fn zero() -> Self {
    Vec2 { x: 0.0, y: 0.0 }
}

#[inline]
pub fn splat(v: f32) -> Self {
    Vec2 { x: v, y: v }
}

#[inline]
pub fn dot(self, rhs: Self) -> f32 {
    self.x * rhs.x + self.y * rhs.y
}

#[inline]
pub fn cross(self, rhs: Self) -> f32 {
    self.x * rhs.y - self.y * rhs.x
}

#[inline]
pub fn perp(self) -> Self {
    Vec2 { x: -self.y, y: self.x }
}

#[inline]
pub fn len_sq(self) -> f32 {
    self.x * self.x + self.y * self.y
}

#[inline]
pub fn len(self) -> f32 {
    self.len_sq().sqrt()
}

#[inline]
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

#[inline]
pub fn normalize_or_zero(self) -> Self {
    if len_sq(self) < super::scalar::EPSILON {
        Vec2::zero()
    } else {
        self.normalize()
    }
}

#[inline]
pub fn lerp(self, rhs: Self, t: f32) -> Self {
    Vec2 {
        x: self.x + t * (rhs.x - self.x),
        y: self.y + t * (rhs.y - self.y),
    }
}

#[inline]
pub fn abs(self) -> Self {
    Vec2 { x: self.x.abs(), y: self.y.abs() }
}

#[inline]
pub fn min_comp(self, rhs: Self) -> Self {
    Vec2 { x: self.x.min(rhs.x), y: self.y.min(rhs.y) }
}

#[inline]
pub fn max_comp(self, rhs: Self) -> Self {
    Vec2 { x: self.x.max(rhs.x), y: self.y.max(rhs.y) }
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

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct Mat2 {
    pub cols: [Vec2; 2],
}

pub fn identity() -> Self {
    Mat2 {
        cols: [Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)],
    }
}

pub fn from_angle(theta: f32) -> Self {
    let c = theta.cos();
    let s = theta.sin();
    Mat2 {
        cols: [Vec2::new(c, s), Vec2::new(-s, c)],
    }
}

pub fn transpose(self) -> Self {
    Mat2 {
        cols: [
            Vec2::new(self.cols[0].x, self.cols[1].x),
            Vec2::new(self.cols[0].y, self.cols[1].y),
        ],
    }
}

pub fn det(self) -> f32 {
    self.cols[0].x * self.cols[1].y - self.cols[1].x * self.cols[0].y
}

pub fn mul_vec(self, v: Vec2) -> Vec2 {
    self.cols[0] * v.x + self.cols[1] * v.y
}

impl std::ops::Mul for Mat2 {
    type Output = Mat2;
    fn mul(self, rhs: Mat2) -> Mat2 {
        Mat2 {
            cols: [
                self.mul_vec(rhs.cols[0]),
                self.mul_vec(rhs.cols[1]),
            ],
        }
    }
}