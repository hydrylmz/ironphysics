use std::collections::HashMap;
use crate::{ColliderHandle, ContactManifold};


/// always stored with smaller handle first for canonical ordering.
#[derive(Hash, PartialEq, Eq, Clone, Copy)]
struct ColliderPair(ColliderHandle, ColliderHandle);

impl ColliderPair {
    fn new(a: ColliderHandle, b: ColliderHandle) -> Self {
        if a.0 <= b.0 {
            ColliderPair(a, b)
        } else {
            ColliderPair(b, a)
        }
    }
}

/// Frame-scoped arena for contact manifolds.
/// Preallocated; reset every frame without heap deallocation.
pub struct ContactPool {
    manifolds:  Vec<ContactManifold>,
    /// Maps a collider pair to its index in `manifolds`.
    pair_map:   HashMap<ColliderPair, usize>,
}

impl ContactPool {
    pub fn new(capacity: usize) -> Self {
       Self {
            manifolds: Vec::with_capacity(capacity),
            pair_map: HashMap::with_capacity(capacity),
        }
    }

    pub fn begin_frame(&mut self) {
    // Reset the pool for a new frame WITHOUT freeing memory.
    // NOTE: The OLD manifold data (for warm-starting) must be saved BEFORE
    // calling begin_frame.
    // begin_frame wipes all old datait is called AFTER persistence has run.
        self.manifolds.clear();
        self.pair_map.clear();
    }

    pub fn insert(&mut self, manifold: ContactManifold) {
    // NOTE: If the pair already exists (duplicate narrowphase result),
    // overwrite: replace self.manifolds[existing_idx] with the new manifold.
    // Duplicates shouldn't occur with a correct broadphase, but defend anyway.
        let key = ColliderPair::new(manifold.collider_a, manifold.collider_b);
        if let Some(&existing_idx) = self.pair_map.get(&key) {
            self.manifolds[existing_idx] = manifold;
        } else {
            let idx = self.manifolds.len();
            self.manifolds.push(manifold);
            self.pair_map.insert(key, idx);
        }

    }

    pub fn get_previous(
    previous: &ContactPool,
    a: ColliderHandle,
    b: ColliderHandle,
    ) -> Option<&ContactManifold> {
        let key = ColliderPair::new(a, b);
        previous.pair_map.get(&key).map(|&idx| &previous.manifolds[idx])
    }

    pub fn manifolds(&self) -> &[ContactManifold] {
        &self.manifolds
    }

    pub fn manifolds_mut(&mut self) -> &mut [ContactManifold] {
        &mut self.manifolds
    }

    pub fn persist_contacts(
    previous: &ContactPool,
    current:  &mut ContactManifold,
) {
    // Match contact points from the previous frame into the current manifold.
    // For matched points, copy the accumulated impulse into the new ContactPoint.
    // The solver will then apply this as an initial impulse (warm-start),
    // dramatically reducing iterations needed to converge.
    // Why this works:
    //   Resting contacts accumulate large normal impulses (to prevent sinking).
    //   Without warm-starting, the solver must re-discover this from zero each frame.
    //   With warm-starting, the very first iteration already applies ~90% of the correct impulse.
    //   Result: 4-8 iterations instead of 20+ for stable stacked-box scenes.
    //
    // The ContactFeatureId encodes WHICH vertices/faces are in contact.
    // For a box corner touching a flat surface, the ID identifies that specific corner.
    // This survives small relative motion between frames.
        if let Some(old_manifold) = ContactPool::get_previous(previous, current.collider_a, current.collider_b) {
            for new_point in &mut current.points[..current.count] {
                for old_point in &old_manifold.points[..old_manifold.count] {
                    if new_point.id == old_point.id {
                        new_point.normal_impulse = old_point.normal_impulse;
                        new_point.tangent_impulse = old_point.tangent_impulse;
                        break;
                    }
                }
            }
        }
    }
}   

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BodyHandle, ColliderHandle};
    use physics_math::Vec2;
    use crate::narrowphase::manifold::{ContactFeatureId, ContactPoint};

    #[test]
    fn pool_begin_frame_clears() {
        // GIVEN: Pool with one manifold inserted
        // WHEN:  begin_frame()
        // THEN:  manifolds().len() == 0
        let mut pool = ContactPool::new(10);
        let manifold = ContactManifold {
            normal: Vec2::new(0.0, 1.0),
            points: [ContactPoint::default(); 2],
            count: 1,
            body_a: BodyHandle::default(),
            body_b: BodyHandle::default(),
            collider_a: ColliderHandle(1),
            collider_b: ColliderHandle(2),
            friction: 0.5,
            restitution: 0.5,
        };
        pool.insert(manifold);
        pool.begin_frame();
        assert!(pool.manifolds().is_empty(), "Expected manifolds to be cleared");

    }

    #[test]
    fn pool_warm_start_copies_impulse() {
        // GIVEN: prev_pool has manifold for pair (ca, cb) with
        //        points[0].normal_impulse = 5.0
        //        points[0].id = some_id
        //
        //        current manifold for same pair with matching id
        //        and normal_impulse = 0.0
        //
        // WHEN:  persist_contacts(&prev_pool, &mut current_manifold)
        //
        // THEN:  current_manifold.points[0].normal_impulse == 5.0
        let mut prev_pool = ContactPool::new(10);
        let id = ContactFeatureId { index_a: 1, index_b: 2, kind: Default::default() };
        let prev_manifold = ContactManifold {
            normal: Vec2::new(0.0, 1.0),
            points: [ContactPoint {
                point: Vec2::new(0.0, 0.0),
                depth: 0.1,
                normal_impulse: 5.0,
                tangent_impulse: 1.0,
                id,
            }, ContactPoint::default()],
            count: 1,
            body_a: BodyHandle::default(),
            body_b: BodyHandle::default(),
            collider_a: ColliderHandle(1),
            collider_b: ColliderHandle(2),
            friction: 0.5,
            restitution: 0.5,
        };
        prev_pool.insert(prev_manifold);
        let mut current_manifold = ContactManifold {
            normal: Vec2::new(0.0, 1.0),
            points: [ContactPoint {
                point: Vec2::new(0.0, 0.0),
                depth: 0.1,
                normal_impulse: 0.0,
                tangent_impulse: 0.0,
                id,
            }, ContactPoint::default()],
            count: 1,
            body_a: BodyHandle::default(),
            body_b: BodyHandle::default(),
            collider_a: ColliderHandle(1),
            collider_b: ColliderHandle(2),
            friction: 0.5,
            restitution: 0.5,
        };
        ContactPool::persist_contacts(&prev_pool, &mut current_manifold);
        assert_eq!(current_manifold.points[0].normal_impulse, 5.0, "Expected normal impulse to be copied from previous manifold");
        assert_eq!(current_manifold.points[0].tangent_impulse, 1.0, "Expected tangent impulse to be copied from previous manifold");

    }

    #[test]
    fn pool_no_warm_start_for_new_pair() {
        // GIVEN: Empty prev_pool
        //        current manifold for a new pair
        // WHEN:  persist_contacts(&prev_pool, &mut current)
        // THEN:  current.points[0].normal_impulse == 0.0  (untouched)
        let prev_pool = ContactPool::new(10);
        let mut current_manifold = ContactManifold {
            normal: Vec2::new(0.0, 1.0),
            points: [ContactPoint {
                point: Vec2::new(0.0, 0.0),
                depth: 0.1,
                normal_impulse: 0.0,
                tangent_impulse: 0.0,
                id: ContactFeatureId { index_a: 1, index_b: 2, kind: Default::default() },
            }, ContactPoint::default()],
            count: 1,
            body_a: BodyHandle::default(),
            body_b: BodyHandle::default(),
            collider_a: ColliderHandle(1),
            collider_b: ColliderHandle(2),
            friction: 0.5,
            restitution: 0.5,
        };
        ContactPool::persist_contacts(&prev_pool, &mut current_manifold);
        assert_eq!(current_manifold.points[0].normal_impulse, 0.0, "Expected normal impulse to remain 0.0 for new pair");
        assert_eq!(current_manifold.points[0].tangent_impulse, 0.0, "Expected tangent impulse to remain 0.0 for new pair");

    }
}