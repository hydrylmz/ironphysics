use physics_math::Vec2;

/// One scalar velocity constraint in the form: J · v = b
/// Solved by iteratively applying impulses until Cdot ≈ bias.
#[derive(Debug, Clone, Copy, Default)]
pub struct VelocityConstraint {
    // Jacobian rows for body A and B
    // The full Jacobian is: J = [j_lin_a, j_ang_a, -j_lin_b, -j_ang_b]
    // (negative sign on B because impulse is equal-and-opposite)
    pub j_lin_a: Vec2,
    pub j_ang_a: f32,
    pub j_lin_b: Vec2,
    pub j_ang_b: f32,

    // ── Precomputed scalars (set in pre_solve, read every iteration) ──
    /// Effective mass = 1 / (J · M^-1 · J^T)
    /// Scalar that converts velocity error → impulse magnitude
    pub eff_mass: f32,

    /// Bias velocity: combines Baumgarte position correction + restitution target
    pub bias: f32,

    // Accumulated impulse 
    /// Sum of all impulses applied to this constraint this frame.
    /// Warm-started from the previous frame. Clamped each iteration.
    pub impulse: f32,

    // Impulse bounds
    /// Minimum allowed accumulated impulse (e.g. 0 for non-penetration = no pull)
    pub lo: f32,
    /// Maximum allowed accumulated impulse (e.g. f32::MAX for joints with no upper limit)
    pub hi: f32,

    // Body indices into BodyStorage
    pub body_a_idx: u32,
    pub body_b_idx: u32,
    
    // Material property for friction bounds
    pub friction: f32,
}

/// SoA layout for all velocity constraints in one island.
/// Built fresh each frame from ContactManifold + Joint data.
pub struct ConstraintStorage {
    pub j_lin_a:    Vec<Vec2>,
    pub j_ang_a:    Vec<f32>,
    pub j_lin_b:    Vec<Vec2>,
    pub j_ang_b:    Vec<f32>,
    pub eff_mass:   Vec<f32>,
    pub bias:       Vec<f32>,
    pub impulse:    Vec<f32>,
    pub lo:         Vec<f32>,
    pub hi:         Vec<f32>,
    pub body_a_idx: Vec<u32>,
    pub body_b_idx: Vec<u32>,
    pub friction:   Vec<f32>,
    pub len:        usize,
}

impl ConstraintStorage {
    pub fn new() -> Self {
        Self::with_capacity(0)
    }

    pub fn with_capacity(cap: usize) -> Self {
        Self {
            j_lin_a:    Vec::with_capacity(cap),
            j_ang_a:    Vec::with_capacity(cap),
            j_lin_b:    Vec::with_capacity(cap),
            j_ang_b:    Vec::with_capacity(cap),
            eff_mass:   Vec::with_capacity(cap),
            bias:       Vec::with_capacity(cap),
            impulse:    Vec::with_capacity(cap),
            lo:         Vec::with_capacity(cap),
            hi:         Vec::with_capacity(cap),
            body_a_idx: Vec::with_capacity(cap),
            body_b_idx: Vec::with_capacity(cap),
            friction:   Vec::with_capacity(cap),
            len:        0,
        }
    }

    pub fn push(&mut self, c: VelocityConstraint) -> usize {
        let idx = self.len;
        self.j_lin_a.push(c.j_lin_a);
        self.j_ang_a.push(c.j_ang_a);
        self.j_lin_b.push(c.j_lin_b);
        self.j_ang_b.push(c.j_ang_b);
        self.eff_mass.push(c.eff_mass);
        self.bias.push(c.bias);
        self.impulse.push(c.impulse);
        self.lo.push(c.lo);
        self.hi.push(c.hi);
        self.body_a_idx.push(c.body_a_idx);
        self.body_b_idx.push(c.body_b_idx);
        self.friction.push(c.friction);
        self.len += 1;
        debug_assert_eq!(self.j_lin_a.len(), self.len);
        debug_assert_eq!(self.j_ang_a.len(), self.len);
        debug_assert_eq!(self.j_lin_b.len(), self.len);
        debug_assert_eq!(self.j_ang_b.len(), self.len);
        debug_assert_eq!(self.eff_mass.len(), self.len);
        debug_assert_eq!(self.bias.len(), self.len);
        debug_assert_eq!(self.impulse.len(), self.len);
        debug_assert_eq!(self.lo.len(), self.len);
        debug_assert_eq!(self.hi.len(), self.len);
        debug_assert_eq!(self.body_a_idx.len(), self.len);
        debug_assert_eq!(self.body_b_idx.len(), self.len);
        debug_assert_eq!(self.friction.len(), self.len);
        idx
    }

    pub fn clear(&mut self) {
        self.j_lin_a.clear();
        self.j_ang_a.clear();
        self.j_lin_b.clear();
        self.j_ang_b.clear();
        self.eff_mass.clear();
        self.bias.clear();
        self.impulse.clear();
        self.lo.clear();
        self.hi.clear();
        self.body_a_idx.clear();
        self.body_b_idx.clear();
        self.friction.clear();
        self.len = 0;
    }
}

impl Default for ConstraintStorage {
    fn default() -> Self {
        Self::new()
    }
}