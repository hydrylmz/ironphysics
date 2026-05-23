/// Column-major 2×2 matrix.
/// cols[0] = first column = [m00, m10]
/// cols[1] = second column = [m01, m11]
///
/// Full layout:
///   | cols[0].x   cols[1].x |   | m00  m01 |
///   | cols[0].y   cols[1].y | = | m10  m11 |
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct Mat2 {
    pub cols: [Vec2; 2],
}
use crate::vec2::Vec2;
use crate::scalar::EPSILON;
 
impl Mat2 {
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

    pub fn inverse(self) -> Option<Self> {
        let det = self.det();
        if det.abs() < EPSILON {
            None
        } else {
            let inv_det = 1.0 / det;
            Some(Mat2 {
                cols: [
                    Vec2::new(self.cols[1].y * inv_det, -self.cols[0].y * inv_det),
                    Vec2::new(-self.cols[1].x * inv_det, self.cols[0].x * inv_det),
                ],
            })
        }
    }

    pub fn mul_vec(self, v: Vec2) -> Vec2 {
        self.cols[0] * v.x + self.cols[1] * v.y
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vec2::Vec2;
    use crate::scalar::EPSILON;

    #[test]
    fn mat2_identity_times_vec_is_identity() {

        let v = Vec2::new(3.0, 7.0);
        let result = Mat2::identity().mul_vec(v);
        assert_eq!(result, v);

    }

    #[test]
    fn mat2_rotate_90_ccw() {

        let v = Vec2::new(1.0, 0.0);
        let rot = Mat2::from_angle(std::f32::consts::FRAC_PI_2);
        let result = rot.mul_vec(v);
        assert!((result.x - 0.0).abs() < EPSILON);
        assert!((result.y - 1.0).abs() < EPSILON);

    }

    #[test]
    fn mat2_rotate_180() {

        let v = Vec2::new(1.0, 0.0);
        let rot = Mat2::from_angle(std::f32::consts::PI);
        let result = rot.mul_vec(v);
        assert!((result.x + 1.0).abs() < EPSILON);
        assert!((result.y - 0.0).abs() < EPSILON);

    }

    #[test]
    fn mat2_transpose_of_rotation_is_inverse() {

        let theta = 0.7;
        let r = Mat2::from_angle(theta);
        let r_inv = r.transpose();
        let v = Vec2::new(2.0, 3.0);
        let forward = r.mul_vec(v);
        let recovered = r_inv.mul_vec(forward);
        assert!((recovered.x - v.x).abs() < EPSILON);
        assert!((recovered.y - v.y).abs() < EPSILON);

    }

    #[test]
    fn mat2_det_of_rotation_is_one() {
  
        let theta = 0.7; 
        let r = Mat2::from_angle(theta);
        let det = r.det();
        assert!((det - 1.0).abs() < EPSILON);

    }
}