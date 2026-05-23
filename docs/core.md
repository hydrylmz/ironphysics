# physics_core

The `physics_core` namespace contains all components responsible for the rigid-body solver, generational memory allocation, and force integration.

---

## `World`
**Struct** in `physics_core`

The primary coordinator of the physics simulation. Manages bodies, applies forces, and advances the simulation in discrete time steps.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `gravity` | `Vec2` | The global gravity vector applied to all active bodies. Default: `(0.0, -9.81)`. |
| `config` | `WorldConfig` | The physical limits, iterations, and stabilization settings of the world. |

### Constructors

#### `new`
```rust
pub fn new(config: WorldConfig) -> Self
```
Initializes a new `World` with the given configuration settings and a pre-allocated capacity for 256 bodies.

### Public Methods

#### `add_body`
```rust
pub fn add_body(&mut self, desc: BodyDesc) -> BodyHandle
```
Registers a new rigid body in the physics world using the provided `BodyDesc`. Returns an opaque `BodyHandle` to reference the body later.

#### `remove_body`
```rust
pub fn remove_body(&mut self, handle: BodyHandle)
```
De-registers a rigid body, freeing its memory slot for future recycling and incrementing its generation to invalidate old handles.

#### `body`
```rust
pub fn body(&self, handle: BodyHandle) -> Option<BodyView<'_>>
```
Returns a read-only view (`BodyView`) of the specified body. Returns `None` if the handle is stale or null.

#### `body_mut`
```rust
pub fn body_mut(&mut self, handle: BodyHandle) -> Option<BodyViewMut<'_>>
```
Returns a mutable view (`BodyViewMut`) of the specified body. Returns `None` if the handle is stale or null.

#### `step`
```rust
pub fn step(&mut self, dt: f32)
```
Advances the physics simulation by `dt` seconds (Delta Time). This executes Symplectic Euler integrations, collision responses, and updates all active transforms.

#### `apply_force`
```rust
pub fn apply_force(&mut self, handle: BodyHandle, force: Vec2)
```
Applies a continuous linear force to the body's center of mass. Awakens the body if sleeping.

#### `apply_force_at_point`
```rust
pub fn apply_force_at_point(&mut self, handle: BodyHandle, force: Vec2, world_point: Vec2)
```
Applies a force at an offset `world_point`, computing and applying both the linear force and the resulting angular torque.

#### `apply_impulse`
```rust
pub fn apply_impulse(&mut self, handle: BodyHandle, impulse: Vec2)
```
Applies an instantaneous velocity change (impulse) to the body.

#### `apply_torque`
```rust
pub fn apply_torque(&mut self, handle: BodyHandle, torque: f32)
```
Applies a continuous angular torque to the body.

---

## `BodyDesc`
**Struct** in `physics_core`

A configuration blueprint used when spawning a new body via `World::add_body()`.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `body_type` | `BodyType` | Specifies if the body is `Static`, `Kinematic`, or `Dynamic`. |
| `position` | `Vec2` | The starting position. |
| `linear_velocity` | `Vec2` | The starting linear velocity. |
| `angle` | `f32` | The starting rotation (radians). |
| `angular_velocity` | `f32` | The starting angular velocity. |
| `inv_mass` | `f32` | The inverse of the mass. A value of $0.0$ signifies infinite mass. |
| `inv_inertia` | `f32` | The inverse of the inertia. A value of $0.0$ signifies infinite rotational inertia. |
| `gravity_scale` | `f32` | A multiplier for the world's gravity (e.g., $0.0$ disables gravity for this body). |
| `linear_damping` | `f32` | Air resistance factor applied to linear velocity per step. |
| `angular_damping` | `f32` | Resistance factor applied to angular velocity per step. |
| `is_awake` | `bool` | Whether the body is initially participating in the simulation solver. |
| `fixed_rotation` | `bool` | If `true`, the body cannot be rotated by forces (forces `inv_inertia` to 0.0 internally). |
| `user_data` | `Option<u64>` | A custom field for linking this body to your external game entities. |

---

## `BodyType`
**Enum** in `physics_core`

Categorizes a rigid body's interaction with the simulation.

| Enum Value | Description |
| :--- | :--- |
| `Static` | Immovable geometry (e.g. walls, floors). Has infinite mass and does not move. |
| `Kinematic` | Unaffected by forces and collisions, but moves according to its `linear_velocity` (e.g. moving platforms). |
| `Dynamic` | A fully simulated body that reacts to gravity, impulses, and collisions. |

---

## `WorldConfig`
**Struct** in `physics_core`

Global configuration thresholds for the `World` solver.

| Name | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `velocity_iterations` | `u32` | `8` | Number of iterations used to resolve velocity constraints. |
| `position_iterations` | `u32` | `3` | Number of iterations used to resolve position constraints (overlaps). |
| `warm_starting_factor` | `f32` | `0.9` | Ratio of accumulated impulses carried between frames. |
| `allow_sleeping` | `bool` | `true` | Allows bodies to freeze computations when inactive. |
| `sleep_time_required` | `f32` | `0.5` | Required continuous seconds of inactivity before sleeping. |
| `linear_slop` | `f32` | `0.005` | Margin of penetration ignored by the position solver. |
| `baumgarte_factor` | `f32` | `0.2` | Intensity ratio for correcting body overlaps over time. |

---

## `BodyHandle`
**Struct** in `physics_core`

An opaque $64$-bit token returned upon body creation. It packs a 32-bit slot index and a 32-bit generation counter.

### Public Methods

#### `is_valid`
```rust
pub fn is_valid(&self) -> bool
```
Returns `true` if this handle does not match the null representation.

#### `null`
```rust
pub fn null() -> Self
```
Returns a safely invalid handle.
