use physics_math::Vec2;
use physics_collision::{
    ColliderStorage, ColliderHandle, DynamicAabbTree, ContactPool,
    CollisionFilter, ColliderDesc, run_narrowphase_parallel,
};
use physics_dynamics::{
    ImpulseSolver, IslandManager, JointStorage, JointKind, WorldConfig,
    BodyStorage, GenerationalArena, BodyType, BodyDesc, BodyView, BodyViewMut,
    solve_position_constraints, JointHandle, compute_island_sleep_decisions_parallel,
};
use physics_collision::BodyHandle;
use physics_dynamics::body::MassProperties;
use crate::{StepStats, timed};


pub struct World {
    pub gravity: Vec2,
    pub config: WorldConfig,
    pub stats: StepStats,
    
    pub bodies: BodyStorage,
    body_arena: GenerationalArena<()>,
    step_count: u64,

    pub colliders: ColliderStorage,
    collider_arena: GenerationalArena<()>,
    broadphase: DynamicAabbTree,
    pub contact_pool: ContactPool,
    prev_pool: ContactPool,

    island_manager: IslandManager,
    pub joint_store: JointStorage,
    impulse_solver: ImpulseSolver,

    candidate_pairs:     Vec<(ColliderHandle, ColliderHandle)>, 


}
impl World {
    pub fn new(config: WorldConfig) -> Self {
        Self {
            gravity: Vec2::new(0.0, -9.81),
            config,
            bodies: BodyStorage::with_capacity(256),
            body_arena: GenerationalArena::with_capacity(256),
            step_count: 0,
            colliders: ColliderStorage::with_capacity(256),
            collider_arena: GenerationalArena::with_capacity(256),
            broadphase: DynamicAabbTree::new(),
            contact_pool: ContactPool::new(256),
            prev_pool: ContactPool::new(256),
            island_manager: IslandManager::new(),
            joint_store: JointStorage::new(),
            impulse_solver: ImpulseSolver::new(),
            candidate_pairs: Vec::new(),
            stats: StepStats::default(),
        }
    }


    pub fn add_body(&mut self, desc: BodyDesc) -> BodyHandle {
        let mass_props = MassProperties { mass: 1.0, inertia: 1.0 };
        let slot = self.bodies.push(&desc, mass_props);
        let (arena_slot, gen) = self.body_arena.insert(());
        debug_assert!(arena_slot == slot, "Arena slot and BodyStorage slot must stay in sync");
        BodyHandle::new(slot, gen)
    }

    pub fn add_collider(&mut self, body: BodyHandle, desc: ColliderDesc) -> ColliderHandle {
        let density = desc.material.density;
        let shape_mass_props = desc.shape.compute_mass_properties(density);

        let body_slot = body.slot() as usize;
        let m = shape_mass_props.mass;
        let inertia = shape_mass_props.inertia;
        if self.bodies.body_type[body_slot] == BodyType::Dynamic {
            self.bodies.inv_mass[body_slot] = if m > 0.0 { 1.0 / m } else { 0.0 };
            self.bodies.inv_inertia[body_slot] = if self.bodies.fixed_rotation[body_slot] {
                0.0
            } else if inertia > 0.0 {
                1.0 / inertia
            } else {
                0.0
            };
        }

        let slot = self.colliders.push(body, desc);
        let (arena_slot, _gen) = self.collider_arena.insert(());
        debug_assert!(arena_slot == slot);
        ColliderHandle::from_slot(slot as usize)
    }

    pub fn add_joint(&mut self, kind: JointKind) -> JointHandle {
        let (body_a, body_b) = match &kind {
            JointKind::Distance(j) => (j.body_a.slot(), j.body_b.slot()),
            JointKind::Revolute(j) => (j.body_a.slot(), j.body_b.slot()),
            JointKind::Prismatic(j) => (j.body_a.slot(), j.body_b.slot()),
        };
        let slot = self.joint_store.push(kind, body_a, body_b);
        JointHandle::new(slot, 0)
    }

    pub fn remove_body(&mut self, handle: BodyHandle) {
        self.body_arena.remove(handle.slot(), handle.generation());
        let slot = handle.slot() as usize;
        if slot < self.bodies.len {
            self.bodies.is_awake[slot] = false;
            self.bodies.body_type[slot] = BodyType::Static;
            self.bodies.generation[slot] = handle.generation().wrapping_add(1);
        }
    }

    pub fn body(&self, handle: BodyHandle) -> Option<BodyView<'_>> {
        self.body_arena.get(handle.slot(), handle.generation()).map(|_| {
            let slot = handle.slot() as usize;
            BodyView {
                position: &self.bodies.position[slot],
                linear_velocity: &self.bodies.linear_velocity[slot],
                angle: &self.bodies.angle[slot],
                angular_velocity: &self.bodies.angular_velocity[slot],
                force: &self.bodies.force[slot],
                torque: &self.bodies.torque[slot],
                inv_mass: &self.bodies.inv_mass[slot],
                inv_inertia: &self.bodies.inv_inertia[slot],

                transform: &self.bodies.transform[slot],
                aabb: &self.bodies.aabb[slot],

                body_type: &self.bodies.body_type[slot],
                gravity_scale: &self.bodies.gravity_scale[slot],
                linear_damping: &self.bodies.linear_damping[slot],
                angular_damping: &self.bodies.angular_damping[slot],
                is_awake: &self.bodies.is_awake[slot],
                fixed_rotation: &self.bodies.fixed_rotation[slot],
                user_data: &self.bodies.user_data[slot],
            }
        })
    }

    pub fn body_mut(&mut self, handle: BodyHandle) -> Option<BodyViewMut<'_>> {
        self.body_arena.get(handle.slot(), handle.generation()).map(|_| {
            let slot = handle.slot() as usize;
            BodyViewMut {
                position: &mut self.bodies.position[slot],
                linear_velocity: &mut self.bodies.linear_velocity[slot],
                angle: &mut self.bodies.angle[slot],
                angular_velocity: &mut self.bodies.angular_velocity[slot],
                force: &mut self.bodies.force[slot],
                torque: &mut self.bodies.torque[slot],
                inv_mass: &mut self.bodies.inv_mass[slot],
                inv_inertia: &mut self.bodies.inv_inertia[slot],

                transform: &mut self.bodies.transform[slot],
                aabb: &mut self.bodies.aabb[slot],

                body_type: &mut self.bodies.body_type[slot],
                gravity_scale: &mut self.bodies.gravity_scale[slot],
                linear_damping: &mut self.bodies.linear_damping[slot],
                angular_damping: &mut self.bodies.angular_damping[slot],
                is_awake: &mut self.bodies.is_awake[slot],
                fixed_rotation: &mut self.bodies.fixed_rotation[slot],
                user_data: &mut self.bodies.user_data[slot],
            }
        })
    }

        pub fn step(&mut self, dt: f32) {
        // Reset stats at the top of every step
        self.stats.reset();
        let step_start = std::time::Instant::now();

        // ── STEP 1+2: INTEGRATE ────────────────────────────────────────────
        timed(&mut self.stats.time_integrate_us, || {
            self.bodies.integrate_parallel(self.gravity, dt)
        });

        // ── STEP 3: CLEAR FORCES ──────────────────────────────────────────
        // (serial — trivially fast, not worth parallelising)
        for i in 0..self.bodies.len {
            self.bodies.force[i]  = Vec2::zero();
            self.bodies.torque[i] = 0.0;
        }

        // ── STEP 4: UPDATE COLLIDER TRANSFORMS + PARALLEL AABB RECOMPUTE ──
        timed(&mut self.stats.time_broadphase_us, || {
            // world_transform = body_transform * local_transform (serial — depends on body transforms)
            self.colliders.update_world_transforms(|h| self.bodies.transform[h.slot() as usize]);

            // AABB recompute: parallel (pure function per collider)
            self.colliders.recompute_aabbs_parallel();

            // BVH node updates: serial (shared mutable tree)
            for i in 0..self.colliders.len {
                let handle = ColliderHandle::from_slot(i);
                self.broadphase.update(handle, self.colliders.world_aabb[i]);
            }

            // Pair collection: serial (tree traversal with shared read)
            self.candidate_pairs.clear();
            self.broadphase.collect_pairs(&mut self.candidate_pairs);

            // Filter
            self.candidate_pairs.retain(|(ha, hb)| {
                let ia = ha.slot();
                let ib = hb.slot();
                self.colliders.body_handle[ia] != self.colliders.body_handle[ib]
                    && CollisionFilter::should_collide(
                        &self.colliders.filter[ia],
                        &self.colliders.filter[ib],
                    )
            });

            self.stats.broadphase_pairs = self.candidate_pairs.len() as u32;
        });

        // ── STEP 5: NARROWPHASE (PARALLEL) ────────────────────────────────
        timed(&mut self.stats.time_narrowphase_us, || {
            std::mem::swap(&mut self.contact_pool, &mut self.prev_pool);
            self.contact_pool.begin_frame();

            run_narrowphase_parallel(
                &self.candidate_pairs,
                &self.colliders,
                &self.prev_pool,
                &mut self.contact_pool,
            );

            self.stats.narrowphase_hits = self.contact_pool.manifolds().len() as u32;
            self.stats.contacts_total   = self.contact_pool.manifolds()
                .iter().map(|m| m.count as u32).sum();
        });

        // ── STEP 6: WAKE BODIES ────────────────────────────────────────────
        // (serial — tiny, runs on new-contact detection only)
        let wake_slots: Vec<(u32, u32)> = self.contact_pool
            .manifolds().iter()
            .filter(|m| ContactPool::get_previous(
                &self.prev_pool, m.collider_a, m.collider_b).is_none())
            .map(|m| (m.body_a.slot(), m.body_b.slot()))
            .collect();
        for (sa, sb) in wake_slots {
            self.bodies.is_awake[sa as usize] = true;
            self.bodies.is_awake[sb as usize] = true;
        }

        // ── STEP 7: ISLAND CONSTRUCTION ───────────────────────────────────
        timed(&mut self.stats.time_island_build_us, || {
            self.island_manager.build_islands(
                &self.bodies, &self.contact_pool, &self.joint_store);

            let islands = self.island_manager.islands();
            self.stats.islands_active   = islands.iter().filter(|i| !i.is_sleeping).count() as u32;
            self.stats.islands_sleeping = islands.iter().filter(|i|  i.is_sleeping).count() as u32;
        });

        // ── STEP 8: PARALLEL ISLAND SOLVE ─────────────────────────────────
        timed(&mut self.stats.time_solve_us, || {
            let results = self.impulse_solver.solve_all_islands_parallel(
                self.island_manager.islands(),
                self.contact_pool.manifolds(),
                &self.joint_store,
                &self.bodies,
                &self.config,
                dt,
            );
            ImpulseSolver::apply_island_results(
                results, &mut self.bodies, &mut self.contact_pool);
        });

        // ── STEP 9: POSITION CORRECTION ───────────────────────────────────
        timed(&mut self.stats.time_position_us, || {
            // Phase 4: still serial (see T-08 for parallel version design)
            solve_position_constraints(
                &self.contact_pool, &mut self.bodies, &self.config);
            for i in 0..self.bodies.len {
                if self.bodies.body_type[i] != BodyType::Static {
                    self.bodies.sync_transform(i);
                }
            }
        });

        // ── STEP 10: SLEEP UPDATE ─────────────────────────────────────────
        timed(&mut self.stats.time_sleep_us, || {
            if self.config.allow_sleeping {
                let decisions = compute_island_sleep_decisions_parallel(
                    self.island_manager.islands(), &self.bodies, &self.config);

                let mut bodies_to_sleep: Vec<Vec<u32>> = Vec::new();

                for (isl, &decision) in self.island_manager.islands_mut().iter_mut().zip(decisions.iter()) {
                    if decision { isl.sleep_timer += dt; }
                    else        { isl.sleep_timer  = 0.0; }
                    if !isl.is_sleeping
                        && isl.sleep_timer >= self.config.sleep_time_required
                    {
                        isl.is_sleeping = true;
                        bodies_to_sleep.push(isl.bodies.clone());
                    }
                }

                for body_list in bodies_to_sleep {
                    for b in body_list {
                        let b = b as usize;
                        self.bodies.linear_velocity[b]  = Vec2::zero();
                        self.bodies.angular_velocity[b] = 0.0;
                        self.bodies.is_awake[b]         = false;
                    }
                }
            }
        });

        // ── STEP 11: FINAL TRANSFORM SYNC ─────────────────────────────────
        for i in 0..self.bodies.len {
            if self.bodies.body_type[i] != BodyType::Static {
                self.bodies.sync_transform(i);
            }
        }

        // ── STEP 12: BODY COUNT STATS + STEP COUNTER ──────────────────────
        self.stats.bodies_active   = (0..self.bodies.len)
            .filter(|&i| self.bodies.body_type[i] == BodyType::Dynamic
                    && self.bodies.is_awake[i]).count() as u32;
        self.stats.bodies_sleeping = (0..self.bodies.len)
            .filter(|&i| self.bodies.body_type[i] == BodyType::Dynamic
                    && !self.bodies.is_awake[i]).count() as u32;
        self.stats.bodies_static   = (0..self.bodies.len)
            .filter(|&i| self.bodies.body_type[i] == BodyType::Static).count() as u32;
        self.stats.time_total_us   = step_start.elapsed().as_micros() as u64;

        self.step_count += 1;
    }

    pub fn configure_thread_pool(num_threads: Option<usize>) {
        // Call this in main() BEFORE the first World::new(), not inside World::new(),
        // so the pool is warmed up before the first step and doesn't count toward
        // the first frame's timing.
        let threads = num_threads.unwrap_or_else(|| {
            (num_cpus::get().saturating_sub(1)).max(1)
        });
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .stack_size(2 * 1024 * 1024)   // 2 MiB — enough for deep GJK/EPA recursion
            .build_global()
            .unwrap_or(());                 // silently ignore "already initialized"

    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use physics_math::{EPSILON, Aabb, Transform};
    #[test]
    fn single_body_free_fall() {

        let mut world = World::new(WorldConfig::default());
        let body_desc = BodyDesc {
            body_type: BodyType::Dynamic,
            position: Vec2::new(0.0, 10.0),
            linear_velocity: Vec2::zero(),
            angle: 0.0,
            angular_velocity: 0.0,
            force: Vec2::zero(),
            torque: 0.0,
            inv_mass: 1.0,
            inv_inertia: 1.0,
            transform: Transform::default(),
            aabb: Aabb::default(),
            gravity_scale: 1.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            is_awake: true,
            fixed_rotation: false,
            user_data: None,
        };
        let handle = world.add_body(body_desc);
        let dt = 1.0 / 60.0;
        world.step(dt);
        let body = world.body(handle).unwrap();
        assert!(body.position.y < 10.0);
        assert!(body.linear_velocity.y < 0.0);
        assert!((body.linear_velocity.x).abs() < EPSILON);

    }

    #[test]
    fn static_body_does_not_move() {

        let mut world = World::new(WorldConfig::default());
        let body_desc = BodyDesc {
            body_type: BodyType::Static,
            position: Vec2::new(5.0, 5.0),
            linear_velocity: Vec2::zero(),
            angle: 0.0,
            angular_velocity: 0.0,
            force: Vec2::zero(),
            torque: 0.0,
            inv_mass: 0.0,
            inv_inertia: 0.0,
            transform: Transform::default(),
            aabb: Aabb::default(),
            gravity_scale: 1.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            is_awake: true,
            fixed_rotation: false,
            user_data: None,
        };
        let handle = world.add_body(body_desc);
        let dt = 1.0 / 60.0;
        for _ in 0..100 {
            world.step(dt);
        }
        let body = world.body(handle).unwrap();
        assert_eq!(*body.position, Vec2::new(5.0, 5.0));
        assert_eq!(*body.linear_velocity, Vec2::zero());

    }

    #[test]
    fn gravity_scale_zero_means_no_gravity() {

        let mut world = World::new(WorldConfig::default());
        let body_desc = BodyDesc {
            body_type: BodyType::Dynamic,
            position: Vec2::new(0.0, 10.0),
            linear_velocity: Vec2::zero(),
            angle: 0.0,
            angular_velocity: 0.0,
            force: Vec2::zero(),
            torque: 0.0,
            inv_mass: 1.0,
            inv_inertia: 1.0,
            transform: Transform::default(),
            aabb: Aabb::default(),
            gravity_scale: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            is_awake: true,
            fixed_rotation: false,
            user_data: None,
        };
        let handle = world.add_body(body_desc);
        let dt = 1.0 / 60.0;
        world.step(dt);
        let body = world.body(handle).unwrap();
        assert!((body.linear_velocity.y).abs() < EPSILON);
        assert_eq!(*body.position, Vec2::new(0.0, 10.0));

    }

    #[test]
    fn stale_handle_returns_none() {

        let mut world = World::new(WorldConfig::default());
        let body_desc = BodyDesc {
            body_type: BodyType::Dynamic,
            position: Vec2::zero(),
            linear_velocity: Vec2::zero(),
            angle: 0.0,
            angular_velocity: 0.0,
            force: Vec2::zero(),
            torque: 0.0,
            inv_mass: 1.0,
            inv_inertia: 1.0,
            transform: Transform::default(),
            aabb: Aabb::default(),
            gravity_scale: 1.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            is_awake: true,
            fixed_rotation: false,
            user_data: None,
        };
        let handle = world.add_body(body_desc);
        world.remove_body(handle);
        let result = world.body(handle);
        assert!(result.is_none());

    }

    #[test]
    fn damping_reduces_velocity() {

        let mut world = World::new(WorldConfig::default());
        let body_desc = BodyDesc {
            body_type: BodyType::Dynamic,
            position: Vec2::zero(),
            linear_velocity: Vec2::new(100.0, 0.0),
            angle: 0.0,
            angular_velocity: 0.0,
            force: Vec2::zero(),
            torque: 0.0,
            inv_mass: 1.0,
            inv_inertia: 1.0,
            transform: Transform::default(),
            aabb: Aabb::default(),
            gravity_scale: 0.0,
            linear_damping: 1.0,
            angular_damping: 0.0,
            is_awake: true,
            fixed_rotation: false, 
            user_data: None,
        };
        let handle = world.add_body(body_desc);
        let dt = 1.0 / 60.0;
        world.step(dt);
        let body = world.body(handle).unwrap();
        assert!(body.linear_velocity.x < 100.0);
        assert!(body.linear_velocity.x >= 0.0);
        
    }
}
