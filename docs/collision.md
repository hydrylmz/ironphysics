# physics_collision

The `physics_collision` namespace contains all resources for collision detection, query primitives, shapes, bounding volume hierarchies (broadphase), and narrowphase contact generation.

---

## `CollisionFilter`
**Struct** in `physics_collision`

Defines bitmasks and group indices to filter collisions between game objects.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `category_bits` | `u32` | Bitmask representing the collision categories this object belongs to. |
| `mask_bits` | `u32` | Bitmask representing the categories this object will collide with. |
| `group_index` | `i32` | Overriding collision group index. Positive values always collide, negative values never collide, zero defaults to bitmask checking. |

### Public Methods

#### `should_collide`
```rust
pub fn should_collide(a: &CollisionFilter, b: &CollisionFilter) -> bool
```
Determines if two filters allow a collision to take place.

---

## Combined Material Helpers
Global friction and restitution combining rules.

#### `combined_friction`
```rust
pub fn combined_friction(filter_a: &CollisionFilter, filter_b: &CollisionFilter) -> f32
```
Combines friction coefficients from two colliding objects. Default value: `0.5`.

#### `combined_restitution`
```rust
pub fn combined_restitution(filter_a: &CollisionFilter, filter_b: &CollisionFilter) -> f32
```
Combines restitution coefficients from two colliding objects. Default value: `0.0`.

---

## `Shape`
**Trait** in `physics_collision`

Exposed trait for geometric collision primitives.

### Public Methods

#### `shape_type`
```rust
fn shape_type(&self) -> ShapeType
```
Returns the type category of this shape.

#### `compute_aabb`
```rust
fn compute_aabb(&self, transform: &Transform) -> Aabb
```
Computes the Axis-Aligned Bounding Box (AABB) of the shape under a given coordinate transform.

#### `compute_mass_properties`
```rust
fn compute_mass_properties(&self, density: f32) -> MassProperties
```
Computes physical mass and rotational inertia given a specific material density.

#### `support`
```rust
fn support(&self, direction: Vec2) -> Vec2
```
Returns the furthest point of the shape along a local direction vector (used in GJK/EPA).

#### `local_centroid`
```rust
fn local_centroid(&self) -> Vec2
```
Returns the local center of mass coordinates of the shape.

---

## `ShapeType`
**Enum** in `physics_collision`

Lists supported geometric shapes.

| Enum Value | Value | Description |
| :--- | :--- | :--- |
| `Circle` | `0` | A circle shape. |
| `Box` | `1` | An oriented bounding box shape. |
| `Capsule` | `2` | A capsule shape (line segment extended by a radius). |
| `Polygon` | `3` | A convex polygon shape. |

---

## `Circle`
**Struct** in `physics_collision` (Implements `Shape`)

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `radius` | `f32` | Radius of the circle. |

### Constructors

#### `new`
```rust
pub fn new(radius: f32) -> Self
```
Creates a new circle shape.

---

## `BoxShape`
**Struct** in `physics_collision` (Implements `Shape`)

An oriented rectangle defined by half extents.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `half_extents` | `Vec2` | Half width and half height coordinates. |

### Constructors

#### `new`
```rust
pub fn new(half_width: f32, half_height: f32) -> Self
```
Creates a new box shape from positive half width and half height extents.

### Public Methods

#### `vertices_local`
```rust
pub fn vertices_local(&self) -> [Vec2; 4]
```
Returns the local-space coordinates of the 4 box corners.

#### `face_normals_local`
```rust
pub fn face_normals_local() -> [Vec2; 4]
```
Returns the static local face normals.

---

## `Capsule`
**Struct** in `physics_collision` (Implements `Shape`)

A capsule defined by a vertical line segment and a radius.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `half_length` | `f32` | Half of the vertical spine length. |
| `radius` | `f32` | Radius of the capsule caps. |

---

## `ConvexPolygon`
**Struct** in `physics_collision` (Implements `Shape`)

A convex polygon with up to 8 vertices for optimal cache performance.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `vertices` | `SmallVec<[Vec2; 8]>` | Wound vertices ordered counter-clockwise. |
| `normals` | `SmallVec<[Vec2; 8]>` | Normals corresponding to each edge. |
| `centroid` | `Vec2` | Computed local center of mass. |

### Constructors

#### `new`
```rust
pub fn new(vertices: SmallVec<[Vec2; 8]>) -> Self
```
Winds vertices, verifies convex integrity, computes edge normals, and returns a new `ConvexPolygon`.

---

## `MassProperties`
**Struct** in `physics_collision`

The mass values calculated for a given shape.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `mass` | `f32` | Scaled total mass of the object. |
| `inv_mass` | `f32` | Reciprocal mass ($1 / \text{mass}$), where $0.0$ represents infinite mass. |
| `inertia` | `f32` | Moment of inertia. |
| `inv_inertia` | `f32` | Reciprocal moment of inertia ($1 / \text{inertia}$). |
| `local_centroid` | `Vec2` | Center of mass offset in local space. |

---

## `ColliderDesc`
**Struct** in `physics_collision`

Blueprint template passed to the world for attaching new collision shapes to rigid bodies.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `shape` | `Box<dyn Shape>` | The collision geometry. |
| `material` | `Material` | Friction, restitution, and density properties. |
| `local_transform` | `Transform` | Offset coordinate transform relative to the body center. |
| `filter` | `CollisionFilter` | Mask and bit configurations for collision filtering. |
| `is_sensor` | `bool` | True if this collider should act as a sensor (fires events but generates no physical impulses). |

---

## `ColliderStorage`
**Struct** in `physics_collision`

Structure of Arrays (SoA) layout mapping and holding active collider states in contiguous system memory.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `body_handle` | `Vec<BodyHandle>` | Owner rigid body handles. |
| `shape` | `Vec<Box<dyn Shape>>` | Contiguous array of collider shapes. |
| `local_transform` | `Vec<Transform>` | Body-space offsets. |
| `world_transform` | `Vec<Transform>` | Derived world transforms ($T_{\text{body}} \times T_{\text{local}}$). |
| `world_aabb` | `Vec<Aabb>` | Axis-Aligned bounding boxes in world space. |
| `filter` | `Vec<CollisionFilter>` | Stored bitmask filters. |
| `is_sensor` | `Vec<bool>` | Array of sensor flags. |
| `density` | `Vec<f32>` | Densities. |
| `restitution` | `Vec<f32>` | Bounce properties. |
| `friction` | `Vec<f32>` | Sliding friction properties. |
| `generation` | `Vec<u32>` | Generation tracking. |
| `len` | `usize` | Total active collider count. |

---

## `DynamicAabbTree`
**Struct** in `physics_collision`

Broadphase Axis-Aligned Bounding Box (AABB) tree utilizing node fattening ($0.1$ slop) to limit rebalance operations for high-performance pair generation.

### Constructors

#### `new`
```rust
pub fn new() -> Self
```
Initializes an empty broadphase tree.

### Public Methods

#### `insert`
```rust
pub fn insert(&mut self, handle: ColliderHandle, aabb: Aabb)
```
Inserts a new leaf node representing a collider.

#### `remove`
```rust
pub fn remove(&mut self, handle: ColliderHandle)
```
Deletes an active leaf node from the broadphase hierarchy.

#### `update`
```rust
pub fn update(&mut self, handle: ColliderHandle, new_aabb: Aabb)
```
Updates a leaf's position. Does nothing if the new AABB remains within its fat slop boundaries.

#### `collect_pairs`
```rust
pub fn collect_pairs(&self, pairs: &mut Vec<(ColliderHandle, ColliderHandle)>)
```
Collects all overlapping leaf node pairs using canonical ordering.

#### `query_aabb`
```rust
pub fn query_aabb(&self, query: Aabb, results: &mut Vec<ColliderHandle>)
```
Finds all colliders whose bounding box overlaps with the given query area.

---

## `ContactManifold`
**Struct** in `physics_collision`

Stores results of narrowphase intersections between overlapping shapes.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `normal` | `Vec2` | Collision normal pointing from A toward B. |
| `points` | `[ContactPoint; 2]` | Contact coordinates (up to two points for face-to-face contacts). |
| `count` | `usize` | Number of active contact points (1 or 2). |
| `body_a` | `BodyHandle` | Owner of the first collider. |
| `body_b` | `BodyHandle` | Owner of the second collider. |
| `collider_a` | `ColliderHandle` | Reference handle for collider A. |
| `collider_b` | `ColliderHandle` | Reference handle for collider B. |
| `friction` | `f32` | Combined friction coefficient. |
| `restitution` | `f32` | Combined bounce coefficient. |

### Public Methods

#### `swapped`
```rust
pub fn swapped(&self) -> Self
```
Inverts normal vector direction, swaps collider and body references, and swaps contact point identifiers.

---

## `ContactPoint`
**Struct** in `physics_collision`

Specific point coordinates where shapes are touching.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `point` | `Vec2` | Touch coordinate in world space. |
| `depth` | `f32` | Overlap distance (penetration depth). |
| `normal_impulse` | `f32` | Total accumulated physical impulse along the normal vector. |
| `tangent_impulse` | `f32` | Total accumulated friction impulse along the tangent plane. |
| `id` | `ContactFeatureId` | Geometric signature of the contact, used to warm-start solver across multiple frames. |

---

## `ContactFeatureId`
**Struct** in `physics_collision`

Wired signature for matching contact coordinates between steps to allow robust warm-starting.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `index_a` | `u8` | Feature index (vertex or edge) on shape A. |
| `index_b` | `u8` | Feature index (vertex or edge) on shape B. |
| `kind` | `ContactFeatureKind` | Vertex or Face type. |

---

## `ContactFeatureKind`
**Enum** in `physics_collision`

Specifies feature types creating a contact point.

| Enum Value | Description |
| :--- | :--- |
| `Vertex` | Intersection produced by a corner vertex. |
| `Face` | Intersection produced by a flat surface face. |

---

## Narrowphase Solver
Core manifold generation function.

#### `dispatch_narrowphase`
```rust
pub fn dispatch_narrowphase(
    shape_a: &dyn Shape, xf_a: &Transform,
    shape_b: &dyn Shape, xf_b: &Transform,
) -> Option<ContactManifold>
```
Dispatches two shapes to correct solver algorithms (e.g. Circle-Circle, Circle-Box, SAT Box-Box, or GJK/EPA) to calculate exact manifold details.

---

## `ContactPool`
**Struct** in `physics_collision`

Contiguous frame-scoped preallocated arena hosting generated contacts. Reuses memory each frame without reallocations.

### Constructors

#### `new`
```rust
pub fn new(capacity: usize) -> Self
```
Creates a new contact pool with the preallocated capacity.

### Public Methods

#### `begin_frame`
```rust
pub fn begin_frame(&mut self)
```
Clears and resets the pool buffers. Run at the beginning of each frame.

#### `insert`
```rust
pub fn insert(&mut self, manifold: ContactManifold)
```
Appends a generated contact manifold to the current frame's pool.

#### `get_previous`
```rust
pub fn get_previous(
    previous: &ContactPool,
    a: ColliderHandle,
    b: ColliderHandle,
) -> Option<&ContactManifold>
```
Retrieves a matching contact manifold from the previous frame's pool.

#### `manifolds`
```rust
pub fn manifolds(&self) -> &[ContactManifold]
```
Returns a read-only slice of all manifolds generated in the current frame.

#### `persist_contacts`
```rust
pub fn persist_contacts(
    previous: &ContactPool,
    current: &mut ContactManifold,
)
```
Matches contact features from the previous frame to the current frame's manifold to carry over accumulated impulses for warm-starting.
