pub const EPSILON: f32 = 1e-6;
pub const PI: f32 = std::f32::consts::PI;
pub const TWO_PI: f32 = 2.0 * PI;
pub const DEG_TO_RAD: f32 = PI / 180.0;
pub const RAD_TO_DEG: f32 = 180.0 / PI;

#[inline]
pub fn clamp(v: f32, lo: f32, hi: f32) -> f32 {
    v.max(lo).min(hi)
}

#[inline]
pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    // NOTE: does NOT clamp t bcz callers are responsible
    a + t * (b - a)

}

#[inline]
pub fn almost_zero(v: f32) -> bool {
    // This exists to avoid f32 == 0.0 comparisons in code
    v.abs() < EPSILON
}

#[inline]
pub fn almost_equal(a: f32, b: f32) -> bool {
    // Equivalent to almost_zero(a - b)
    (a - b).abs() < EPSILON
}

#[inline]
pub fn sign(v: f32) -> f32 {
    v.signum()
}

pub fn wrap_angle(angle: f32) -> f32 {
    (angle + PI).rem_euclid(TWO_PI) - PI
}

#[inline]
pub fn min_f32(a: f32, b: f32) -> f32 {
    f32::min(a, b)
}

#[inline]
pub fn max_f32(a: f32, b: f32) -> f32 {
    f32::max(a, b)
}