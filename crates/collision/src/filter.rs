pub struct CollisionFilter {

    pub category_bits: u32,
    pub mask_bits: u32,

    /// Group index shortcut:
    /// `> 0`  always collide with same positive group (overrides bits)
    /// `< 0`  never collide with same negative group (overrides bits)
    /// `= 0`  use category/mask bits only
    pub group_index: i32,
}

impl CollisionFilter {
    pub fn should_collide(a: &CollisionFilter, b: &CollisionFilter) -> bool {
        if a.group_index != 0 && a.group_index == b.group_index {
            return a.group_index > 0;
        }
        (a.category_bits & b.mask_bits) != 0 && (b.category_bits & a.mask_bits) != 0
    }
}

/// Combine friction coefficients from two materials using the geometric mean.
pub fn combined_friction(_filter_a: &CollisionFilter, _filter_b: &CollisionFilter) -> f32 {
    // This is a simple default implementation.
    // In practice, you might store friction values in the filter or material,
    // but for now we'll return a default value.
    0.5
}

/// Combine restitution (bounciness) from two materials.
pub fn combined_restitution(_filter_a: &CollisionFilter, _filter_b: &CollisionFilter) -> f32 {
    // This is a simple default implementation.
    // In practice, you might store restitution values in the filter or material,
    // but for now we'll return a default value.
    0.0
}
