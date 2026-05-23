# Core Module (`physics_core`)

The `physics_core` crate manages the primary simulation components: memory allocation, data layout, external forces, solver settings, and step integrations.

---

## 1. Generational Arena (`arena.rs`)

The `GenerationalArena<T>` is a pre-allocated collection that provides safe, fast, and continuous memory indexing. 

```rust
pub struct GenerationalArena<T> {
    entries:   Vec<ArenaEntry<T>>,
    free_list: Vec<u32>,             
    len:       usize,                
}

enum ArenaEntry<T> {
    Occupied { generation: u32, value: T },
    Free     { generation: u32 },    
}
```

### Allocation & Recycling Pipeline

1.  **Insertion**:
    *   If `free_list` is not empty, pop a slot index, increment the slot's `generation` wrapping-counter, write the value, and transition to `Occupied`.
    *   If `free_list` is empty, push a new `Occupied` entry with `generation = 0` at the end of the `entries` vector.
2.  **Removal**:
    *   Verify the handle's generation matches the active entry.
    *   Extract the value, transition to `Free`, increment the `generation` counter again (to immediately invalidate any outstanding handles), and push the slot index onto `free_list`.
3.  **Lookup**:
    *   $O(1)$ constant-time lookup by slot index. Checks if the slot is `Occupied` and its generation matches the query handle.

---

## 2. Opaque Handle Packing (`handle.rs`)

The `BodyHandle` is a lightweight token returned to the user upon body creation. It hides the underlying memory layout while ensuring absolute lookup safety.

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BodyHandle(u64);
```

### Bit-Packing Mechanics
The slot index (`u32`) and generation counter (`u32`) are bitwise packed into a single `u64`. This makes handles extremely cheap to copy, compare, and hash:

```rust
// Packing: Slot occupies lower 32 bits, Generation occupies upper 32 bits
let value = ((generation as u64) << 32) | (slot as u64);
```

To extract coordinates:
*   **Slot**: `self.0 as u32`
*   **Generation**: `(self.0 >> 32) as u32`
*   **Null Handle**: Represented as `u64::MAX`.

---

## 3. Data-Oriented Body Storage (`body.rs`)

`BodyStorage` stores all rigid body variables in a cache-friendly Struct-of-Arrays (SoA) layout.

```rust
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
    pub len:               usize,
}
```

### Rigid Body Classifications (`BodyType`)
*   **`Static`**: Stationary bodies (e.g. ground, walls). Mass and inertia are treated as infinite ($\text{inv\_mass} = 0.0$, $\text{inv\_inertia} = 0.0$). They do not move in response to gravity, forces, or impulses.
*   **`Kinematic`**: Bodies animated via scripts or direct velocity. Like static bodies, they have infinite mass. However, they integrate velocity to update position.
*   **`Dynamic`**: Fully simulated bodies. They respond to gravity, dampings, collisions, user-applied forces, and impulses.

### Direct Borrow Views (`BodyView` & `BodyViewMut`)
To keep the API ergonomic while retaining SoA memory speed, the engine provides aggregated reference wrappers. This allows you to inspect or modify a body using a clean struct interface:

```rust
// Ergonomic view wrapper returned by world.body(handle)
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
    pub is_awake: &'a bool,
    pub user_data: &'a Option<u64>,
}
```

---

## 4. Solver Settings (`config.rs`)

The `WorldConfig` structure stores physical parameters and solver thresholds for the integration pipeline:

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `velocity_iterations` | `u32` | `8` | Number of solver iterations resolving velocity constraints. |
| `position_iterations` | `u32` | `3` | Number of solver iterations resolving overlapping positions. |
| `warm_starting_factor` | `f32` | `0.9` | Ratio of accumulated impulses carried over to the next step. |
| `linear_slop` | `f32` | `0.005` | Penetration threshold below which position correction is skipped. |
| `baumgarte_factor` | `f32` | `0.2` | Position correction factor to resolve overlaps gently. |
| `restitution_threshold` | `f32` | `1.0` | Velocity threshold below which elastic collisions become inelastic. |
| `allow_sleeping` | `bool` | `true` | Enables sleeping to put idle bodies to rest, saving CPU cycles. |
| `sleep_time_required` | `f32` | `0.5` | Seconds of continuous immobility required to trigger sleep. |

---

## 5. Main Simulation Coordinator (`world.rs`)

`World` is the central engine class, orchestrating the rigid bodies, gravity, sleeping states, and time integration step.

```rust
pub struct World {
    pub gravity:        Vec2,
    pub config:         WorldConfig,
    bodies:             BodyStorage,
    body_arena:         GenerationalArena<()>, 
    step_count:         u64,
}
```

### Physics Step Loop (`World::step`)
When `world.step(dt)` is called, it performs:

1.  **DOD Contiguous Filtering**: Skips static, sleeping, or inactive slots.
2.  **External Accelerations**: Adds gravity to active force buffers:
    $$\vec{F}_{\text{total}} \gets \vec{F}_{\text{accumulated}} + \vec{g} \cdot s_{\text{gravity}} \cdot m$$
3.  **Euler semi-implicit integration**:
    $$\vec{v} \gets \vec{v} + \vec{a} \cdot \Delta t$$
    $$\vec{x} \gets \vec{x} + \vec{v} \cdot \Delta t$$
4.  **Damping**: Multiplies velocities by linear/angular damping factors.
5.  **Accumulator Cleanup**: Clears force and torque arrays back to zero.
6.  **Transform Sync**: Updates the cached positional `Transform` structures to optimize collision detection.

### User Physics APIs

```rust
// Force Application (Accumulates over the step, wakes up sleeping bodies)
pub fn apply_force(&mut self, handle: BodyHandle, force: Vec2);

// Direct Impulse Application (Immediately updates linear velocity)
pub fn apply_impulse(&mut self, handle: BodyHandle, impulse: Vec2);

// Angular Torque (Wakes body up and adds rotational force)
pub fn apply_torque(&mut self, handle: BodyHandle, torque: f32);

// Force at offset point (Applies linear force and computes corresponding torque)
pub fn apply_force_at_point(&mut self, handle: BodyHandle, force: Vec2, world_point: Vec2);
```
