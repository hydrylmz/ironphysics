
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct Aabb {
    pub min: Vec2,
    pub max: Vec2,
}

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