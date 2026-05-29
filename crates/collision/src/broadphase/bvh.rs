use std::collections::HashMap;
use crate::ColliderHandle;
use physics_math::Aabb;

const AABB_EXTENSION: f32 = 0.1; // Fatness factor for faster tree rebalancing
const NULL_NODE: u32 = u32::MAX;    // sentinel for "no node"

struct TreeNode {
    aabb:        Aabb,
    parent:      u32,           // NULL_NODE if root
    left:        u32,           // NULL_NODE if leaf
    right:       u32,           // NULL_NODE if leaf
    height:      i32,           // 0 for leaves, -1 for free nodes
    /// leaf only, which collider this node represents.
    /// NULL for internal nodes.
    handle:      ColliderHandle,
    /// Next free node index (used when node is in free list).
    next_free:   u32,
}

pub struct DynamicAabbTree {
    nodes:      Vec<TreeNode>,
    root:       u32,            // NULL_NODE when tree is empty
    free_list:  u32,            // head of the free-node linked list
    /// leaf node index for O(1) lookup.
    handle_map: HashMap<ColliderHandle, u32>,
}

impl Default for DynamicAabbTree {
    fn default() -> Self {
        Self::new()
    }
}

impl DynamicAabbTree {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            root: NULL_NODE,
            free_list: NULL_NODE,
            handle_map: HashMap::new(),
        }
    }

    /// Allocate a node from the free list or by expanding the nodes vector.
    fn allocate_node(&mut self) -> u32 {
        if self.free_list != NULL_NODE {
            let node_index = self.free_list;
            self.free_list = self.nodes[node_index as usize].next_free;
            node_index
        } else {
            let node_index = self.nodes.len() as u32;
            self.nodes.push(TreeNode {
                aabb: Aabb::default(),
                parent: NULL_NODE,
                left: NULL_NODE,
                right: NULL_NODE,
                height: -1,
                handle: ColliderHandle::null(),
                next_free: NULL_NODE,
            });
            node_index
        }
    }

    fn free_node(&mut self, node_idx: u32) {
        let node = &mut self.nodes[node_idx as usize];
        node.height = -1;
        node.next_free = self.free_list;
        self.free_list = node_idx;
    }

    pub fn insert(&mut self, handle: ColliderHandle, aabb: Aabb) {
        // Until now, I had some clue about what is written in the code. But now I have no idea what is going on.
        // I will just copy the code and hope for the best
        let fat_aabb = aabb.fatten(AABB_EXTENSION);
        let leaf_idx = self.allocate_node();
        {
            let leaf = &mut self.nodes[leaf_idx as usize];
            leaf.aabb = fat_aabb;
            leaf.height = 0;
            leaf.handle = handle;
            leaf.left = NULL_NODE;
            leaf.right = NULL_NODE;
        }
        if self.root == NULL_NODE {
            // Tree was empty, new leaf becomes root.
            self.root = leaf_idx;
        } else {
            let sibling_idx = self.find_best_sibling(leaf_idx);
            let sibling_aabb = self.nodes[sibling_idx as usize].aabb;
            let new_parent_aabb = fat_aabb.merge(&sibling_aabb);
            let new_parent_idx = self.allocate_node();
            {
                let sibling_height = self.nodes[sibling_idx as usize].height;
                let new_parent = &mut self.nodes[new_parent_idx as usize];
                new_parent.aabb = new_parent_aabb;
                new_parent.height = sibling_height + 1;
                new_parent.left = sibling_idx;
                new_parent.right = leaf_idx;
            }
            let sibling_parent_idx = self.nodes[sibling_idx as usize].parent;
            if sibling_parent_idx == NULL_NODE {
                // Sibling was root, now new parent is root.
                self.root = new_parent_idx;
                self.nodes[sibling_idx as usize].parent = new_parent_idx;
                self.nodes[leaf_idx as usize].parent = new_parent_idx;
            } else {
                // Link new parent to sibling's old parent.
                if self.nodes[sibling_parent_idx as usize].left == sibling_idx {
                    self.nodes[sibling_parent_idx as usize].left = new_parent_idx;
                } else {
                    self.nodes[sibling_parent_idx as usize].right = new_parent_idx;
                }
                self.nodes[new_parent_idx as usize].parent = sibling_parent_idx;
                self.nodes[sibling_idx as usize].parent = new_parent_idx;
                self.nodes[leaf_idx as usize].parent = new_parent_idx;
            }
            // Refit ancestors.
            self.refit_ancestors(new_parent_idx);
        }
        // Register in handle_map for O(1) lookup.
        self.handle_map.insert(handle, leaf_idx);
    }

    fn find_best_sibling(&self, leaf_idx: u32) -> u32 {
        let leaf_aabb = self.nodes[leaf_idx as usize].aabb;
        let mut best_cost = leaf_aabb.merge(&self.nodes[self.root as usize].aabb).area();
        let mut best_sibling = self.root;
        let mut stack = vec![(self.root, 0.0)];
        while let Some((node_idx, inherited_cost)) = stack.pop() {
            let node = &self.nodes[node_idx as usize];
            let direct_cost = leaf_aabb.merge(&node.aabb).area();
            let total_cost = direct_cost + inherited_cost;
            if total_cost < best_cost {
                best_cost = total_cost;
                best_sibling = node_idx;
            }
            if node.left != NULL_NODE {
                let child_inherited = inherited_cost + direct_cost - node.aabb.area();
                let lower_bound = leaf_aabb.area() + child_inherited;
                if lower_bound < best_cost {
                    stack.push((node.left, child_inherited));
                    stack.push((node.right, child_inherited));
                }
            }
        }
        best_sibling
    }

    pub fn remove(&mut self, handle: ColliderHandle) {

        let leaf_idx = match self.handle_map.remove(&handle) {
            Some(idx) => idx,
            None => return, // handle not found, nothing to remove
        };
        let leaf = &self.nodes[leaf_idx as usize];
        if leaf.parent == NULL_NODE {
            // Leaf is root.
            self.root = NULL_NODE;
            self.free_node(leaf_idx);
            return;
        }
        let parent_idx = leaf.parent;
        let sibling_idx = if self.nodes[parent_idx as usize].left == leaf_idx {
            self.nodes[parent_idx as usize].right
        } else {
            self.nodes[parent_idx as usize].left
        };
        if self.nodes[parent_idx as usize].parent == NULL_NODE {
            // Parent is root, sibling becomes new root.
            self.root = sibling_idx;
            self.nodes[sibling_idx as usize].parent = NULL_NODE;
        } else {
            // Link sibling to grandparent.
            let grandparent_idx = self.nodes[parent_idx as usize].parent;
            if self.nodes[grandparent_idx as usize].left == parent_idx {
                self.nodes[grandparent_idx as usize].left = sibling_idx;
            } else {
                self.nodes[grandparent_idx as usize].right = sibling_idx;
            }
            self.nodes[sibling_idx as usize].parent = grandparent_idx;
        }
        self.free_node(parent_idx);
        self.free_node(leaf_idx);
        // Refit ancestors starting from the sibling's new parent (which is either grandparent or NULL if sibling is new root).
        let refit_start = if self.nodes[parent_idx as usize].parent == NULL_NODE {
            NULL_NODE
        } else {
            self.nodes[parent_idx as usize].parent
        };
        self.refit_ancestors(refit_start);
    }

    pub fn update(&mut self, handle: ColliderHandle, new_aabb: Aabb) {
        // The fatten + contains_aabb check is the key performance optimization.
        // Most frames, 90%+ of bodies don't move enough to exit their fat AABB,
        // so this function is nearly free for them.
        let leaf_idx = match self.handle_map.get(&handle) {
            Some(&idx) => idx,
            None => return, // handle not found, nothing to update
        };
        let stored_fat_aabb = self.nodes[leaf_idx as usize].aabb;
        if stored_fat_aabb.contains_aabb(&new_aabb) {
            return; // no update needed
        }
        self.remove(handle);
        let fat_new = new_aabb.fatten(AABB_EXTENSION);
        self.insert(handle, fat_new);
    }

    fn refit_ancestors(&mut self, mut node_idx: u32) {
        while node_idx != NULL_NODE {
            let left_idx = self.nodes[node_idx as usize].left;
            let right_idx = self.nodes[node_idx as usize].right;
            let left_aabb = self.nodes[left_idx as usize].aabb;
            let right_aabb = self.nodes[right_idx as usize].aabb;
            self.nodes[node_idx as usize].aabb = left_aabb.merge(&right_aabb);
            self.nodes[node_idx as usize].height = 1 + i32::max(self.nodes[left_idx as usize].height, self.nodes[right_idx as usize].height);
            node_idx = self.nodes[node_idx as usize].parent;
        }
    }

    pub fn collect_pairs(&self, pairs: &mut Vec<(ColliderHandle, ColliderHandle)>) {
        // This is the broadphase output called once per frame.
        // Note: pairs.clear() should be called by the caller before this call.
        if self.root == NULL_NODE || self.nodes[self.root as usize].height == 0 {
            return; // empty tree or single leaf = no pairs
        }
        let mut stack = vec![(self.root, self.root)];
        while let Some((a_idx, b_idx)) = stack.pop() {
            if a_idx == b_idx {
                // Same subtree — expand it.
                let node = &self.nodes[a_idx as usize];
                if node.height == 0 {
                    continue; // leaf has no children to pair with itself
                }
                stack.push((node.left, node.left));
                stack.push((node.right, node.right));
                stack.push((node.left, node.right));
            } else {
                // Two different subtrees.
                let a_node = &self.nodes[a_idx as usize];
                let b_node = &self.nodes[b_idx as usize];
                if !a_node.aabb.overlaps(&b_node.aabb) {
                    continue; // no overlap possible in this subtree
                }
                if a_node.height == 0 && b_node.height == 0 {
                    // Both are leaves → emit pair with canonical ordering.
                    let handle_a = a_node.handle;
                    let handle_b = b_node.handle;
                    if handle_a.0 < handle_b.0 {
                        pairs.push((handle_a, handle_b));
                    } else {
                        pairs.push((handle_b, handle_a));
                    }
                } else if a_node.height == 0 {
                    // a is leaf, b is internal → descend into b.
                    stack.push((a_idx, b_node.left));
                    stack.push((a_idx, b_node.right));
                } else if b_node.height == 0 {
                    // a is internal, b is leaf → descend into a.
                    stack.push((a_node.left, b_idx));
                    stack.push((a_node.right, b_idx));
                } else {
                    // Both are internal → descend into both.
                    stack.push((a_node.left,  b_node.left));
                    stack.push((a_node.left,  b_node.right));
                    stack.push((a_node.right, b_node.left));
                    stack.push((a_node.right, b_node.right));
                }
            }
        }

    }

    pub fn query_aabb(&self, query: Aabb, results: &mut Vec<ColliderHandle>) {
        if self.root == NULL_NODE {
            return; // empty tree
        }
        let mut stack = vec![self.root];
        while let Some(node_idx) = stack.pop() {
            if node_idx == NULL_NODE {
                continue;
            }
            let node = &self.nodes[node_idx as usize];
            if !node.aabb.overlaps(&query) {
                continue; // skip this subtree
            }
            if node.height == 0 {
                results.push(node.handle);
            } else {
                stack.push(node.left);
                stack.push(node.right);
            }
        }
    }



}

#[cfg(test)]
mod tests {
    use super::*;
    use physics_math::{Vec2, aabb::Aabb};

    #[test]
    fn bvh_insert_and_query() {
        // GIVEN: Empty tree
        //        Insert handle_0 with aabb [(0,0),(1,1)]
        //        Insert handle_1 with aabb [(0.5,0.5),(1.5,1.5)]
        //        Insert handle_2 with aabb [(5,5),(6,6)]
        //
        // WHEN:  collect_pairs into vec
        //
        // THEN:  (handle_0, handle_1) is in pairs   ← overlapping
        //        (handle_0, handle_2) NOT in pairs   ← separated
        //        (handle_1, handle_2) NOT in pairs   ← separated
        let mut tree = DynamicAabbTree::new();
        let handle_0 = ColliderHandle(0);
        let handle_1 = ColliderHandle(1);
        let handle_2 = ColliderHandle(2);
        tree.insert(handle_0, Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0)));
        tree.insert(handle_1, Aabb::new(Vec2::new(0.5, 0.5), Vec2::new(1.5, 1.5)));
        tree.insert(handle_2, Aabb::new(Vec2::new(5.0, 5.0), Vec2::new(6.0, 6.0)));
        let mut pairs = Vec::new();
        tree.collect_pairs(&mut pairs);
        assert!(pairs.contains(&(handle_0, handle_1)) || pairs.contains(&(handle_1, handle_0)), "Expected (handle_0, handle_1) in pairs");
        assert!(!pairs.contains(&(handle_0, handle_2)) && !pairs.contains(&(handle_2, handle_0)), "Expected (handle_0, handle_2) NOT in pairs");
        assert!(!pairs.contains(&(handle_1, handle_2)) && !pairs.contains(&(handle_2, handle_1)), "Expected (handle_1, handle_2) NOT in pairs");

    }

    #[test]
    fn bvh_remove_stops_pairing() {
        // GIVEN: Tree with handle_0 and handle_1 overlapping → 1 pair
        //        remove(handle_0)
        // WHEN:  collect_pairs
        // THEN:  pairs is empty
        let mut tree = DynamicAabbTree::new();
        let handle_0 = ColliderHandle(0);
        let handle_1 = ColliderHandle(1);
        tree.insert(handle_0, Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0)));
        tree.insert(handle_1, Aabb::new(Vec2::new(0.5, 0.5), Vec2::new(1.5, 1.5)));
        let mut pairs = Vec::new();
        tree.collect_pairs(&mut pairs);
        assert!(!pairs.is_empty(), "Expected pairs before removal");
        tree.remove(handle_0);
        pairs.clear();
        tree.collect_pairs(&mut pairs);
        assert!(pairs.is_empty(), "Expected no pairs after removal");

    }

    #[test]
    fn bvh_update_inside_fat_aabb_no_reinsert() {
        // GIVEN: handle_0 inserted at aabb [(0,0),(1,1)]
        //        Tiny movement: update handle_0 to [(0.01,0),(1.01,1)]
        // THEN:  Tree structure unchanged (stored fat aabb still contains new aabb)
        //        (Verify by checking node count didn't change)
        let mut tree = DynamicAabbTree::new();
        let handle_0 = ColliderHandle(0);
        tree.insert(handle_0, Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0)));
        let initial_node_count = tree.nodes.len();
        tree.update(handle_0, Aabb::new(Vec2::new(0.01, 0.0), Vec2::new(1.01, 1.0)));
        let final_node_count = tree.nodes.len();
        assert_eq!(initial_node_count, final_node_count, "Expected no new nodes allocated for update inside fat AABB");

    }

    #[test]
    fn bvh_query_aabb() {
        // GIVEN: Tree with 3 non-overlapping handles
        //        query_aabb with a small box that only overlaps handle_1
        // THEN:  results contains only handle_1
        let mut tree = DynamicAabbTree::new();
        let handle_0 = ColliderHandle(0);
        let handle_1 = ColliderHandle(1);
        let handle_2 = ColliderHandle(2);
        tree.insert(handle_0, Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0)));
        tree.insert(handle_1, Aabb::new(Vec2::new(0.5, 0.5), Vec2::new(1.5, 1.5)));
        tree.insert(handle_2, Aabb::new(Vec2::new(5.0, 5.0), Vec2::new(6.0, 6.0)));
        let mut results = Vec::new();
        tree.query_aabb(Aabb::new(Vec2::new(1.2, 1.2), Vec2::new(1.3, 1.3)), &mut results);
        assert_eq!(results.len(), 1, "Expected exactly one result from query");
        assert_eq!(results[0], handle_1, "Expected result to be handle_1");

    }
}