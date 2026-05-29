use physics_math::{Aabb, Transform, Vec2};
use rayon::prelude::*;
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BodyType {
    Static,
    Kinematic,
    Dynamic,
}

#[derive(Debug, Clone, Copy)]
pub struct MassProperties {
    pub mass: f32,
    pub inertia: f32,
}

pub struct BodyDesc {
    pub body_type: BodyType,
    pub position: Vec2,
    pub linear_velocity: Vec2,
    pub angle: f32,
    pub angular_velocity: f32,
    pub force: Vec2,
    pub torque: f32,
    pub inv_mass: f32,
    pub inv_inertia: f32,
    pub transform: Transform,
    pub aabb: Aabb,
    pub gravity_scale: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub is_awake: bool,
    pub fixed_rotation: bool,
    pub user_data: Option<u64>,
}

pub struct BodyStorage {
    pub position:          Vec<Vec2>,    
    pub linear_velocity:   Vec<Vec2>,    
    pub angle:             Vec<f32>,     
    pub angular_velocity:  Vec<f32>,     
    pub force:             Vec<Vec2>,    
    pub torque:            Vec<f32>,     
    pub inv_mass:          Vec<f32>,     
    pub inv_inertia:       Vec<f32>,    

    pub transform:         Vec<Transform>,  
    pub aabb:              Vec<Aabb>,        

    pub body_type:         Vec<BodyType>,
    pub gravity_scale:     Vec<f32>,
    pub linear_damping:    Vec<f32>,
    pub angular_damping:   Vec<f32>,
    pub is_awake:          Vec<bool>,
    pub fixed_rotation:    Vec<bool>,
    pub user_data:         Vec<Option<u64>>,
    pub generation:        Vec<u32>,      

    pub len: usize,
}

impl BodyStorage {
    pub fn new() -> Self {
        Self {
            position:          Vec::new(),
            linear_velocity:   Vec::new(),
            angle:             Vec::new(),
            angular_velocity:  Vec::new(),
            force:             Vec::new(),
            torque:            Vec::new(),
            inv_mass:          Vec::new(),
            inv_inertia:       Vec::new(),

            transform:         Vec::new(),
            aabb:              Vec::new(),

            body_type:         Vec::new(),
            gravity_scale:     Vec::new(),
            linear_damping:    Vec::new(),
            angular_damping:   Vec::new(),
            is_awake:          Vec::new(),
            fixed_rotation:    Vec::new(),
            user_data:         Vec::new(),
            generation:        Vec::new(),

            len: 0,
        }
    }

    pub fn with_capacity(cap: usize) -> Self {
        Self {
            position:          Vec::with_capacity(cap),
            linear_velocity:   Vec::with_capacity(cap),
            angle:             Vec::with_capacity(cap),
            angular_velocity:  Vec::with_capacity(cap),
            force:             Vec::with_capacity(cap),
            torque:            Vec::with_capacity(cap),
            inv_mass:          Vec::with_capacity(cap),
            inv_inertia:       Vec::with_capacity(cap),

            transform:         Vec::with_capacity(cap),
            aabb:              Vec::with_capacity(cap),

            body_type:         Vec::with_capacity(cap),
            gravity_scale:     Vec::with_capacity(cap),
            linear_damping:    Vec::with_capacity(cap),
            angular_damping:   Vec::with_capacity(cap),
            is_awake:          Vec::with_capacity(cap),
            fixed_rotation:    Vec::with_capacity(cap),
            user_data:         Vec::with_capacity(cap),
            generation:        Vec::with_capacity(cap),

            len: 0,
        }
    }

    pub fn push(&mut self, desc: &BodyDesc, mass_props: MassProperties) -> u32 {
        self.position.push(desc.position);
        self.linear_velocity.push(desc.linear_velocity);
        self.angle.push(desc.angle);
        self.angular_velocity.push(desc.angular_velocity);
        self.force.push(Vec2::zero());
        self.torque.push(0.0);
        match desc.body_type {
            BodyType::Static => {
                self.inv_mass.push(0.0);
                self.inv_inertia.push(0.0);
            }
            BodyType::Kinematic => {
                self.inv_mass.push(0.0);
                self.inv_inertia.push(0.0);
            }
            BodyType::Dynamic => {
                self.inv_mass.push(1.0 / mass_props.mass);
                self.inv_inertia.push(if desc.fixed_rotation {
                    0.0
                } else {
                    1.0 / mass_props.inertia
                });
            }
        }
        self.transform.push(Transform::new(desc.position, desc.angle));
        self.aabb.push(Aabb::new(Vec2::zero(), Vec2::zero()));
        self.body_type.push(desc.body_type);
        self.gravity_scale.push(desc.gravity_scale);
        self.linear_damping.push(desc.linear_damping);
        self.angular_damping.push(desc.angular_damping);
        self.is_awake.push(true);
        self.fixed_rotation.push(desc.fixed_rotation);
        self.user_data.push(desc.user_data);
        self.generation.push(0);
        let slot = self.len as u32;
        self.len += 1;
        debug_assert_eq!(self.position.len(), self.linear_velocity.len());
        debug_assert_eq!(self.position.len(), self.angle.len());
        debug_assert_eq!(self.position.len(), self.angular_velocity.len());
        debug_assert_eq!(self.position.len(), self.force.len());
        debug_assert_eq!(self.position.len(), self.torque.len());
        debug_assert_eq!(self.position.len(), self.inv_mass.len());
        debug_assert_eq!(self.position.len(), self.inv_inertia.len());
        slot
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn is_active(&self, idx: usize) -> bool {
        idx < self.len
    }

    pub fn sync_transform(&mut self, slot: usize) {
        self.transform[slot] = Transform::new(self.position[slot], self.angle[slot]);
    }

    pub fn iter(&self) -> BodyStorageIter<'_> {
        BodyStorageIter {
            storage: self,
            index: 0,
        }
    }

    pub fn integrate_parallel(
        &mut self,
        gravity:  Vec2,
        dt:       f32,
    ) {
        // 1. Collect indices of dynamic + awake bodies sequentially
        let active_indices: Vec<usize> = (0..self.len)
            .filter(|&i| self.body_type[i] == BodyType::Dynamic && self.is_awake[i])
            .collect();

        // 2. Parallel phase — COMPUTE new velocities and positions on multiple threads
        let updates: Vec<(Vec2, f32, Vec2, f32)> = active_indices
            .par_iter() // <--- Re-enabled parallelism!
            .map(|&i| {
                let lin_accel: Vec2 = gravity * self.gravity_scale[i]
                            + self.force[i] * self.inv_mass[i];
                let ang_accel: f32 = self.torque[i] * self.inv_inertia[i];

                let mut lv: Vec2 = self.linear_velocity[i]  + lin_accel * dt;
                let mut av: f32 = self.angular_velocity[i] + ang_accel * dt;

                // Using the f32::max fix here keeps Rayon completely happy
                lv *= f32::max(1.0_f32 - self.linear_damping[i] * dt, 0.0_f32);
                av *= f32::max(1.0_f32 - self.angular_damping[i] * dt, 0.0_f32);

                let new_pos   = self.position[i] + lv * dt;
                let new_angle = self.angle[i]    + av * dt;

                (lv, av, new_pos, new_angle)
            })
            .collect();

        // 3. Serial write-back phase
        for (idx, (lv, av, pos, angle)) in active_indices.iter().zip(updates.iter()) {
            self.linear_velocity[*idx]  = *lv;
            self.angular_velocity[*idx] = *av;
            self.position[*idx]         = *pos;
            self.angle[*idx]            = *angle;
        }
    }
}

pub struct BodyStorageIter<'a> {
    storage: &'a BodyStorage,
    index: usize,
}

impl<'a> Iterator for BodyStorageIter<'a> {
    type Item = BodyView<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.storage.len {
            let slot = self.index;
            self.index += 1;
            Some(BodyView {
                position: &self.storage.position[slot],
                linear_velocity: &self.storage.linear_velocity[slot],
                angle: &self.storage.angle[slot],
                angular_velocity: &self.storage.angular_velocity[slot],
                force: &self.storage.force[slot],
                torque: &self.storage.torque[slot],
                inv_mass: &self.storage.inv_mass[slot],
                inv_inertia: &self.storage.inv_inertia[slot],
                transform: &self.storage.transform[slot],
                aabb: &self.storage.aabb[slot],
                body_type: &self.storage.body_type[slot],
                gravity_scale: &self.storage.gravity_scale[slot],
                linear_damping: &self.storage.linear_damping[slot],
                angular_damping: &self.storage.angular_damping[slot],
                is_awake: &self.storage.is_awake[slot],
                fixed_rotation: &self.storage.fixed_rotation[slot],
                user_data: &self.storage.user_data[slot],
            })
        } else {
            None
        }
    }
}

impl Default for BodyStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for BodyDesc {
    fn default() -> Self {
        Self {
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
        }
    }
}

pub struct BodyView<'a> {
    pub position: &'a Vec2,
    pub linear_velocity: &'a Vec2,
    pub angle: &'a f32,
    pub angular_velocity: &'a f32,
    pub force: &'a Vec2,
    pub torque: &'a f32,
    pub inv_mass: &'a f32,
    pub inv_inertia: &'a f32,

    pub transform: &'a Transform,
    pub aabb: &'a Aabb,

    pub body_type: &'a BodyType,
    pub gravity_scale: &'a f32,
    pub linear_damping: &'a f32,
    pub angular_damping: &'a f32,
    pub is_awake: &'a bool,
    pub fixed_rotation: &'a bool,
    pub user_data: &'a Option<u64>,
}

impl<'a> BodyView<'a> {
    pub fn is_static(&self) -> bool {
        *self.body_type == BodyType::Static
    }

    pub fn is_sleeping(&self) -> bool {
        !*self.is_awake
    }
}

pub struct BodyViewMut<'a> {
    pub position: &'a mut Vec2,
    pub linear_velocity: &'a mut Vec2,
    pub angle: &'a mut f32,
    pub angular_velocity: &'a mut f32,
    pub force: &'a mut Vec2,
    pub torque: &'a mut f32,
    pub inv_mass: &'a mut f32,
    pub inv_inertia: &'a mut f32,

    pub transform: &'a mut Transform,
    pub aabb: &'a mut Aabb,

    pub body_type: &'a mut BodyType,
    pub gravity_scale: &'a mut f32,
    pub linear_damping: &'a mut f32,
    pub angular_damping: &'a mut f32,
    pub is_awake: &'a mut bool,
    pub fixed_rotation: &'a mut bool,
    pub user_data: &'a mut Option<u64>,
}
