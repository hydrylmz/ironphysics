use crate::{Vec2, BodyHandle, WorldConfig, GenerationalArena, BodyStorage, Transform};
use crate::body::{BodyDesc, BodyType, MassProperties, BodyView, BodyViewMut};

pub struct World {
    pub gravity:        Vec2,
    pub config:         WorldConfig,

    bodies:             BodyStorage,
    body_arena:         GenerationalArena<()>, 

    step_count:         u64,

    // TODO: Collision integration
    // colliders:    ColliderStorage,
    // broadphase:   DynamicAabbTree,
    // contact_pool: ContactPool,
    // prev_pool:    ContactPool,
    // collider_arena: GenerationalArena<()>,
}
impl World {
    pub fn new(config: WorldConfig) -> Self {
        Self {
            gravity: Vec2::new(0.0, -9.81),
            config,
            bodies: BodyStorage::with_capacity(256),
            body_arena: GenerationalArena::with_capacity(256),
            step_count: 0,
            // TODO: Initialize collision components
        }
    }

    pub fn add_body(&mut self, desc: BodyDesc) -> BodyHandle {
        let mass_props = MassProperties { mass: 1.0, inertia: 1.0 };
        let slot = self.bodies.push(&desc, mass_props);
        let (arena_slot, gen) = self.body_arena.insert(());
        debug_assert!(arena_slot == slot, "Arena slot and BodyStorage slot must stay in sync");
        BodyHandle::new(slot, gen)
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
        if dt <= 0.0 {
            return;
        }
        // STEP 1 — APPLY FORCES & INTEGRATE VELOCITIES
        for i in 0..self.bodies.len {
            if self.bodies.is_awake[i] && self.bodies.body_type[i] == BodyType::Dynamic {
                let gravity = self.gravity * self.bodies.gravity_scale[i];
                let total_force = self.bodies.force[i] + gravity * (1.0 / self.bodies.inv_mass[i]);
                let total_torque = self.bodies.torque[i];
                self.bodies.linear_velocity[i] += total_force * self.bodies.inv_mass[i] * dt;
                self.bodies.angular_velocity[i] += total_torque * self.bodies.inv_inertia[i] * dt;
                self.bodies.linear_velocity[i] = self.bodies.linear_velocity[i] * (1.0 / (1.0 + dt * self.bodies.linear_damping[i]));
                self.bodies.angular_velocity[i] *= 1.0 / (1.0 + dt * self.bodies.angular_damping[i]);
            }
        }   
        // STEP 2 — INTEGRATE POSITIONS
        for i in 0..self.bodies.len {
            if self.bodies.is_awake[i] && self.bodies.body_type[i] == BodyType::Dynamic {
                self.bodies.position[i] += self.bodies.linear_velocity[i] * dt;
                self.bodies.angle[i] += self.bodies.angular_velocity[i] * dt;
            }
        }
        // STEP 3 — CLEAR FORCE ACCUMULATORS
        for i in 0..self.bodies.len {
            self.bodies.force[i] = Vec2::zero();
            self.bodies.torque[i] = 0.0;
        }

        // TODO: STEP 4 — UPDATE COLLIDER WORLD TRANSFORMS
        // STEP 5 — BROADPHASE UPDATE  
        // STEP 6 — FILTER PAIRS
        // STEP 7 — NARROWPHASE
        // STEP 8 — SYNC TRANSFORMS
        // STEP 9 — INCREMENT STEP COUNTER

        // STEP 8 — SYNC TRANSFORMS (moved here for now)
        for i in 0..self.bodies.len {
            self.bodies.transform[i] = Transform::new(self.bodies.position[i], self.bodies.angle[i]);
        }
        // STEP 9 — INCREMENT STEP COUNTER
        self.step_count += 1;
    }
}

#[cfg(test)]
mod tests {
    use crate::{World, WorldConfig, Vec2, EPSILON, Aabb, Transform};
    use crate::body::{BodyDesc, BodyType};
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
