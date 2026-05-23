use crate::{Vec2, BodyHandle, WorldConfig, GenerationalArena, BodyStorage};
use crate::body::{BodyDesc, BodyType, MassProperties, BodyView, BodyViewMut};

pub struct World {
    pub gravity:        Vec2,
    pub config:         WorldConfig,

    bodies:             BodyStorage,
    body_arena:         GenerationalArena<()>, 

    step_count:         u64,
}
impl World {
    pub fn new(config: WorldConfig) -> Self {
        Self {
            gravity: Vec2::new(0.0, -9.81),
            config,
            bodies: BodyStorage::with_capacity(256),
            body_arena: GenerationalArena::with_capacity(256),
            step_count: 0,
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

        let bodies = &mut self.bodies;
    let num_bodies = bodies.len();
    for i in 0..num_bodies {
            if !bodies.is_active(i) || !bodies.is_awake[i] || bodies.inv_mass[i] == 0.0 {
                continue;
            }

            let mass = 1.0 / bodies.inv_mass[i];
            let gravity_force = self.gravity * bodies.gravity_scale[i] * mass;
            bodies.force[i] += gravity_force;

            let linear_accel = bodies.force[i] * bodies.inv_mass[i];
            bodies.linear_velocity[i] += linear_accel * dt;

            let angular_accel = bodies.torque[i] * bodies.inv_inertia[i];
            bodies.angular_velocity[i] += angular_accel * dt;

            let damping_factor = (1.0 - bodies.linear_damping[i] * dt).max(0.0_f32);
            bodies.linear_velocity[i] *= damping_factor;

            let ang_damp_factor = (1.0 - bodies.angular_damping[i] * dt).max(0.0_f32);
            bodies.angular_velocity[i] *= ang_damp_factor;

            bodies.position[i] += bodies.linear_velocity[i] * dt;
            bodies.angle[i] += bodies.angular_velocity[i] * dt;
        }

    for i in 0..num_bodies {
            if !bodies.is_active(i) {
                continue;
            }

            bodies.force[i] = Vec2::zero();
            bodies.torque[i] = 0.0;

            bodies.sync_transform(i);
    }

    self.step_count += 1;
    }

    pub fn apply_force(&mut self, handle: BodyHandle, force: Vec2) {
    if !handle.is_valid() {
        return;
    }

    let slot = handle.slot() as usize;
    let bodies = &mut self.bodies;

    if !bodies.is_active(slot) || bodies.generation[slot] != handle.generation() {
        return;
    }

    if bodies.inv_mass[slot] == 0.0 {
        return;
    }

    bodies.force[slot] += force;

    bodies.is_awake[slot] = true;
    }

    pub fn apply_force_at_point(&mut self, handle: BodyHandle, force: Vec2, world_point: Vec2) {
    if !handle.is_valid() {
        return;
    }

    let r = world_point - self.bodies.position[handle.slot() as usize];
    self.bodies.force[handle.slot() as usize] += force;
    let torque = r.cross(force);
    self.bodies.torque[handle.slot() as usize] += torque;
    }

    pub fn apply_impulse(&mut self, handle: BodyHandle, impulse: Vec2) {
    if !handle.is_valid() {
        return;
    }

    if let Some(body) = self.body_mut(handle) {
        if *body.inv_mass == 0.0 {
            return;
        }

        *body.linear_velocity += impulse * *body.inv_mass;
        *body.is_awake = true;
    }
    }

    pub fn apply_torque(&mut self, handle: BodyHandle, torque: f32) {
    if !handle.is_valid() {
        return;
    }

    if let Some(body) = self.body_mut(handle) {
        if *body.inv_mass == 0.0 {
            return;
        }

        *body.torque += torque;
        *body.is_awake = true;
    }
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
    fn applied_force_accelerates_body() {

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
            gravity_scale: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            is_awake: true,
            fixed_rotation: false,
            user_data: None,
        };
        let handle = world.add_body(body_desc);
        let force = Vec2::new(10.0, 0.0);
        world.apply_force(handle, force);
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
