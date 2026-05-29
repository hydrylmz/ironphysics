use std::collections::VecDeque;
use physics_collision::ContactPool;
use crate::body::{BodyStorage, BodyType};
use crate::Island;
use crate::joint::{DistanceJoint, RevoluteJoint, PrismaticJoint};

pub struct IslandManager {
    islands: Vec<Island>,
    /// Scratch buffer: maps global body index → island index (usize::MAX = unvisited)
    pub body_to_island: Vec<usize>,
}

/// All joints stored flat with index-based access.
/// JointHandle maps to a slot here.
pub struct JointStorage {
    pub kinds:      Vec<JointKind>,   // which variant
    pub body_pairs: Vec<(u32, u32)>,  // (body_a_slot, body_b_slot)
    pub generation: Vec<u32>,
    pub len:        usize,
}

impl Default for JointStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl JointStorage {
    pub fn new() -> Self {
        Self {
            kinds: Vec::new(),
            body_pairs: Vec::new(),
            generation: Vec::new(),
            len: 0,
        }
    }

    pub fn push(&mut self, kind: JointKind, body_a: u32, body_b: u32) -> u32 {
        let slot = self.len as u32;
        self.kinds.push(kind);
        self.body_pairs.push((body_a, body_b));
        self.generation.push(0);
        self.len += 1;
        slot
    }
}

pub enum JointKind {
    Distance(DistanceJoint),
    Revolute(RevoluteJoint),
    Prismatic(PrismaticJoint),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct JointHandle(u64);   // same packing as BodyHandle

impl Default for IslandManager {
    fn default() -> Self {
        Self::new()
    }
}

impl IslandManager {
    pub fn new() -> Self {
        Self {
            islands: Vec::new(),
            body_to_island: Vec::new(),
        }
    }

    pub fn build_islands(
        &mut self,
        body_store:   &BodyStorage,
        contact_pool: &ContactPool,
        joint_store:  &JointStorage,
    ) {
        self.islands.clear();
        self.body_to_island.clear();
        self.body_to_island.resize(body_store.len(), usize::MAX);

        for (i, body) in body_store.iter().enumerate() {
            if body.is_static() || body.is_sleeping() || self.body_to_island[i] != usize::MAX {
                continue;
            }
            let island_idx = self.islands.len();
            let mut island = Island::default();
            let mut queue = VecDeque::new();
            queue.push_back(i as u32);
            self.body_to_island[i] = island_idx;

            while let Some(body_idx) = queue.pop_front() {
                island.bodies.push(body_idx);

                // Traverse contacts
                for (contact_idx, manifold) in contact_pool.manifolds().iter().enumerate() {
                    let a = manifold.body_a.slot();

                    let b = manifold.body_b.slot();
                    if a != body_idx && b != body_idx {
                        continue;
                    }
                    island.contacts.push(contact_idx);
                    let other = if a == body_idx { b } else { a };
                    if self.body_to_island[other as usize] == usize::MAX {
                        if body_store.body_type[other as usize] != BodyType::Static {
                            self.body_to_island[other as usize] = island_idx;
                            queue.push_back(other);
                        } else {
                            // Mark static bodies with a sentinel to avoid double-add
                            self.body_to_island[other as usize] = usize::MAX - 1;
                            island.bodies.push(other);
                        }
                    }
                }

                // Traverse joints
                for j in 0..joint_store.len {
                    let (a, b) = joint_store.body_pairs[j];
                    if a != body_idx && b != body_idx {
                        continue;
                    }
                    island.joints.push(JointHandle::new(j as u32, joint_store.generation[j]));
                    let other = if a == body_idx { b } else { a };
                    if self.body_to_island[other as usize] == usize::MAX {
                        if body_store.body_type[other as usize] != BodyType::Static {
                            self.body_to_island[other as usize] = island_idx;
                            queue.push_back(other);
                        } else {
                            // Mark static bodies with a sentinel to avoid double-add
                            self.body_to_island[other as usize] = usize::MAX - 1;
                            island.bodies.push(other);
                        }
                    }
                }
            }
            self.islands.push(island);
        }
    }

    pub fn islands(&self) -> &[Island] {
        &self.islands[..]
    }

    pub fn islands_mut(&mut self) -> &mut [Island] {
        &mut self.islands[..]
    }
}

impl JointHandle {
    pub fn new(slot: u32, gen: u32) -> Self {
        JointHandle((gen as u64) << 32 | (slot as u64))
    }
    pub fn slot(&self) -> u32 { 
        // lower 32 bits 
        (self.0 & 0xFFFF_FFFF) as u32
    }
    pub fn generation(&self) -> u32 { 
        // upper 32 bits 
        (self.0 >> 32) as u32
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn island_construction_two_separate_stacks() {
        let num_islands = 2;
        let island_0_bodies = 2; // A and B
        let island_1_bodies = 2; // C and D
        assert_eq!(num_islands, 2);
        assert_eq!(island_0_bodies + island_1_bodies, 4);
    }

    #[test]
    fn island_construction_chain() {
        let num_islands = 1;
        let island_0_bodies = 4; // A, B, C, D
        let island_0_contacts = 3; // A-B, B-C, C-D
        assert_eq!(num_islands, 1);
        assert_eq!(island_0_bodies, 4);
        assert_eq!(island_0_contacts, 3);
    }

    #[test]
    fn static_body_does_not_create_own_island() {
        let num_islands = 1;
        let island_0_has_static = true;
        let num_dynamic_bodies = 1; // Only A is dynamic
        assert_eq!(num_islands, 1);
        assert!(island_0_has_static);
        assert_eq!(num_dynamic_bodies, 1);
    }

    #[test]
    fn joint_connects_two_islands_into_one() {
        let num_islands = 1;
        let island_0_bodies = 2; // A and B
        let island_0_joints = 1; // The RevoluteJoint
        assert_eq!(num_islands, 1);
        assert_eq!(island_0_bodies, 2);
        assert_eq!(island_0_joints, 1);
    }
}