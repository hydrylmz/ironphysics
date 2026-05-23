
use crate::vec2::Vec2;

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct Aabb {
    pub min: Vec2,
    pub max: Vec2,
}
impl Aabb {
    pub fn new(min: Vec2, max: Vec2) -> Self {
        debug_assert!(min.x <= max.x && min.y <= max.y, "Invalid AABB: min must be less than or equal to max");
        Aabb { min, max }
    }

    pub fn from_center_half_extents(center: Vec2, half: Vec2) -> Self {
        Aabb {
            min: center - half,
            max: center + half,
        }
    }

    pub fn center(&self) -> Vec2 {
        (self.min + self.max) * 0.5
    }

    pub fn half_extents(&self) -> Vec2 {
        (self.max - self.min) * 0.5
    }

    pub fn area(&self) -> f32 {
        let size = self.max - self.min;
        size.x * size.y
    }

    pub fn overlaps(&self, other: &Aabb) -> bool {
        self.min.x <= other.max.x
            && other.min.x <= self.max.x
            && self.min.y <= other.max.y
            && other.min.y <= self.max.y
    }

    pub fn contains_point(&self, p: Vec2) -> bool {
        self.min.x <= p.x && p.x <= self.max.x && self.min.y <= p.y && p.y <= self.max.y
    }

    pub fn contains_aabb(&self, other: &Aabb) -> bool {
        self.min.x <= other.min.x
            && other.max.x <= self.max.x
            && self.min.y <= other.min.y
            && other.max.y <= self.max.y
    }

    pub fn merge(&self, other: &Aabb) -> Aabb {
        Aabb {
            min: self.min.min_comp(other.min),
            max: self.max.max_comp(other.max),
        }
    }

    pub fn fatten(&self, margin: f32) -> Aabb {
        let margin_vec = Vec2 { x: margin, y: margin };
        Aabb {
            min: self.min - margin_vec,
            max: self.max + margin_vec,
        }
    }

    pub fn translate(&self, offset: Vec2) -> Aabb {
        Aabb {
            min: self.min + offset,
            max: self.max + offset,
        }
    }
}

impl Default for Aabb {
    fn default() -> Self {
        Aabb::new(Vec2::zero(), Vec2::zero())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vec2::Vec2;
    use crate::scalar::EPSILON;

    #[test]
    fn aabb_overlaps_true() {
        // GIVEN: a = Aabb { min:(0,0), max:(2,2) }
        //        b = Aabb { min:(1,1), max:(3,3) }
        // THEN:  a.overlaps(&b) == true  (overlap region: (1,1)..(2,2))
        let a = Aabb { min: Vec2::new(0.0, 0.0), max: Vec2::new(2.0, 2.0) };
        let b = Aabb { min: Vec2::new(1.0, 1.0), max: Vec2::new(3.0, 3.0) };
        assert!(a.overlaps(&b));

    }

    #[test]
    fn aabb_overlaps_false_x_separated() {
        // GIVEN: a = Aabb { min:(0,0), max:(1,1) }
        //        b = Aabb { min:(2,0), max:(3,1) }
        // THEN:  a.overlaps(&b) == false  (gap on X axis)
        let a = Aabb { min: Vec2::new(0.0, 0.0), max: Vec2::new(1.0, 1.0) };
        let b = Aabb { min: Vec2::new(2.0, 0.0), max: Vec2::new(3.0, 1.0) };
        assert!(!a.overlaps(&b));

    }

    #[test]
    fn aabb_overlaps_false_y_separated() {
        // GIVEN: a = Aabb { min:(0,0), max:(1,1) }
        //        b = Aabb { min:(0,2), max:(1,3) }
        // THEN:  a.overlaps(&b) == false  (gap on Y axis)
        let a = Aabb { min: Vec2::new(0.0, 0.0), max: Vec2::new(1.0, 1.0) };
        let b = Aabb { min: Vec2::new(0.0, 2.0), max: Vec2::new(1.0, 3.0) };
        assert!(!a.overlaps(&b));

    }

    #[test]
    fn aabb_overlaps_touching_edge() {
        // GIVEN: a = Aabb { min:(0,0), max:(1,1) }
        //        b = Aabb { min:(1,0), max:(2,1) }
        // THEN:  a.overlaps(&b) == true  (shared edge at x=1 counts as overlap)
        let a = Aabb { min: Vec2::new(0.0, 0.0), max: Vec2::new(1.0, 1.0) };
        let b = Aabb { min: Vec2::new(1.0, 0.0), max: Vec2::new(2.0, 1.0) };
        assert!(a.overlaps(&b));

    }

    #[test]
    fn aabb_merge_contains_both() {
        // GIVEN: a = Aabb { min:(0,0), max:(1,1) }
        //        b = Aabb { min:(2,2), max:(3,3) }
        // WHEN:  m = a.merge(&b)
        // THEN:  m.min == (0,0) AND m.max == (3,3)
        //        m.contains_aabb(&a) == true
        //        m.contains_aabb(&b) == true
        let a = Aabb { min: Vec2::new(0.0, 0.0), max: Vec2::new(1.0, 1.0) };
        let b = Aabb { min: Vec2::new(2.0, 2.0), max: Vec2::new(3.0, 3.0) };
        let m = a.merge(&b);
        assert_eq!(m.min, Vec2::new(0.0, 0.0));
        assert_eq!(m.max, Vec2::new(3.0, 3.0));
        assert!(m.contains_aabb(&a));
        assert!(m.contains_aabb(&b));
    }

    #[test]
    fn aabb_fatten_expands_uniformly() {
        // GIVEN: a = Aabb::from_center_half_extents(Vec2::zero(), Vec2::splat(1.0))
        //        → min=(-1,-1), max=(1,1)
        // WHEN:  f = a.fatten(0.5)
        // THEN:  f.min ≈ (-1.5, -1.5)
        //        f.max ≈ ( 1.5,  1.5)
        let a = Aabb::from_center_half_extents(Vec2::zero(), Vec2::splat(1.0));
        let f = a.fatten(0.5);
        assert!((f.min.x - (-1.5)).abs() < EPSILON);
        assert!((f.min.y - (-1.5)).abs() < EPSILON);
        assert!((f.max.x - 1.5).abs() < EPSILON);
        assert!((f.max.y - 1.5).abs() < EPSILON);
        
    }
}