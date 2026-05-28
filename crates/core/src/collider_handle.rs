/// Opaque handle to a collider.
/// Stores a simple u32 slot index for O(1) array access.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub struct ColliderHandle(pub u32);

impl ColliderHandle {
    #[inline]
    pub fn from_slot(slot: usize) -> Self {
        ColliderHandle(slot as u32)
    }

    #[inline]
    pub fn slot(&self) -> usize {
        self.0 as usize
    }

    #[inline]
    pub fn null() -> Self {
        ColliderHandle(u32::MAX)
    }

    #[inline]
    pub fn is_null(&self) -> bool {
        self.0 == u32::MAX
    }
}
