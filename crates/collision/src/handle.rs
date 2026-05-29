/// Opaque handle to a rigid body.
/// Stores (slot: u32, generation: u32) packed into a single u64
/// for cheap Copy and hash.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BodyHandle(pub u64);

impl BodyHandle {
    #[inline]
    pub fn new(slot: u32, gen: u32) -> Self {
        let value = ((gen as u64) << 32) | (slot as u64);
        BodyHandle(value)
    }

    #[inline]
    pub fn slot(&self) -> u32 {
        self.0 as u32
    }

    #[inline]
    pub fn generation(&self) -> u32 {
        (self.0 >> 32) as u32
    }

    #[inline]
    pub fn is_valid(&self) -> bool {
        self.0 != u64::MAX
    }

    #[inline]
    pub fn null() -> Self {
        BodyHandle(u64::MAX)
    }
}

impl Default for BodyHandle {
    fn default() -> Self {
        Self::null()
    }
}
