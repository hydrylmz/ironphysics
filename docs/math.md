# physics_math

The `physics_math` namespace contains all 2D linear algebra and geometric primitives used by the Iron Physics engine.

---

## `Vec2`
**Struct** in `physics_math`

A 2D column vector used for positions, velocities, forces, and spatial directions.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `x` | `f32` | The X coordinate of the vector. |
| `y` | `f32` | The Y coordinate of the vector. |

### Constructors

#### `new`
```rust
pub fn new(x: f32, y: f32) -> Self
```
Creates a new `Vec2` with the specified `x` and `y` components.

#### `zero`
```rust
pub fn zero() -> Self
```
Shorthand for `Vec2::new(0.0, 0.0)`.

#### `splat`
```rust
pub fn splat(v: f32) -> Self
```
Creates a vector with both `x` and `y` initialized to `v`.

### Public Methods

#### `dot`
```rust
pub fn dot(self, rhs: Self) -> f32
```
Returns the dot product ($\vec{a} \cdot \vec{b}$) of this vector and `rhs`.

#### `cross`
```rust
pub fn cross(self, rhs: Self) -> f32
```
Returns the 2D cross product magnitude ($\vec{a} \times \vec{b}$) of this vector and `rhs`.

#### `perp`
```rust
pub fn perp(self) -> Self
```
Returns the 2D counter-clockwise perpendicular vector (rotated 90 degrees).

#### `len`
```rust
pub fn len(self) -> f32
```
Returns the magnitude (length) of the vector.

#### `len_sq`
```rust
pub fn len_sq(self) -> f32
```
Returns the squared magnitude of the vector (faster than `len()` as it avoids the square root).

#### `normalize`
```rust
pub fn normalize(self) -> Self
```
Returns this vector with a magnitude of 1.0. **Panics** if the vector has a length of zero.

#### `normalize_or_zero`
```rust
pub fn normalize_or_zero(self) -> Self
```
Returns a normalized vector. If the length is near zero (below `EPSILON`), returns `Vec2::zero()`.

#### `lerp`
```rust
pub fn lerp(self, rhs: Self, t: f32) -> Self
```
Linearly interpolates between this vector and `rhs` by interpolation factor `t`.

### Operators
`Vec2` supports standard arithmetic operators:
*   **Math**: `+`, `-`, `*` (scalar), `/` (scalar), `-` (negate)
*   **Assignment**: `+=`, `-=`, `*=` (scalar)

---

## `Mat2`
**Struct** in `physics_math`

A column-major $2 \times 2$ transformation matrix used for 2D rotations.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `cols` | `[Vec2; 2]` | Array of 2 column vectors representing the matrix data. |

### Constructors

#### `identity`
```rust
pub fn identity() -> Self
```
Returns the Identity matrix.

#### `from_angle`
```rust
pub fn from_angle(theta: f32) -> Self
```
Creates a rotation matrix for a given angle `theta` in radians.

### Public Methods

#### `transpose`
```rust
pub fn transpose(self) -> Self
```
Returns the transposed matrix (columns swapped to rows).

#### `det`
```rust
pub fn det(self) -> f32
```
Returns the determinant of the matrix.

#### `inverse`
```rust
pub fn inverse(self) -> Option<Self>
```
Returns the inverted matrix, or `None` if the matrix is singular (determinant is near zero).

#### `mul_vec`
```rust
pub fn mul_vec(self, v: Vec2) -> Vec2
```
Multiplies the matrix by the given column vector `v`.

---

## `Transform`
**Struct** in `physics_math`

A combination of translation and rotation, defining a coordinate frame.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `position` | `Vec2` | The origin point of the coordinate frame in the parent space. |
| `rotation` | `f32` | The rotation of the coordinate frame in radians. |

### Constructors

#### `identity`
```rust
pub fn identity() -> Self
```
Returns a transform with `position = (0,0)` and `rotation = 0.0`.

#### `new`
```rust
pub fn new(position: Vec2, rotation: f32) -> Self
```
Creates a new transform from the specified position and rotation.

### Public Methods

#### `rotation_mat`
```rust
pub fn rotation_mat(&self) -> Mat2
```
Returns the $2 \times 2$ rotation matrix representing this transform's angle.

#### `apply`
```rust
pub fn apply(&self, local_point: Vec2) -> Vec2
```
Transforms a point from this local space into the parent (world) space.

#### `apply_inv`
```rust
pub fn apply_inv(&self, world_point: Vec2) -> Vec2
```
Transforms a point from the parent (world) space back into this local space.

#### `combine`
```rust
pub fn combine(&self, child: &Transform) -> Transform
```
Combines two transformations, effectively computing $T_{\text{self}} \circ T_{\text{child}}$.

---

## `Aabb`
**Struct** in `physics_math`

An Axis-Aligned Bounding Box. Used heavily in broadphase collision culling.

### Properties

| Name | Type | Description |
| :--- | :--- | :--- |
| `min` | `Vec2` | The minimum X and Y coordinates (bottom-left). |
| `max` | `Vec2` | The maximum X and Y coordinates (top-right). |

### Constructors

#### `new`
```rust
pub fn new(min: Vec2, max: Vec2) -> Self
```
Creates an AABB with the specified min and max bounds.

#### `from_center_half_extents`
```rust
pub fn from_center_half_extents(center: Vec2, half: Vec2) -> Self
```
Constructs an AABB using a center point and half-size dimensions.

### Public Methods

#### `overlaps`
```rust
pub fn overlaps(&self, other: &Aabb) -> bool
```
Returns `true` if this bounding box intersects with `other`.

#### `contains_point`
```rust
pub fn contains_point(&self, p: Vec2) -> bool
```
Returns `true` if the vector `p` lies inside the bounding box.

#### `merge`
```rust
pub fn merge(&self, other: &Aabb) -> Aabb
```
Returns a new AABB that is the union of this box and `other`.

#### `fatten`
```rust
pub fn fatten(&self, margin: f32) -> Aabb
```
Expands the bounding box outward by `margin` on all sides.

---

## Scalar Utilities

Global float constants and precision mathematical functions provided by `physics_math`.

| Identifier | Type | Description |
| :--- | :--- | :--- |
| `EPSILON` | `f32` | $1 \times 10^{-6}$. The standard floating-point tolerance. |
| `PI` | `f32` | $\pi \approx 3.14159$ |
| `DEG_TO_RAD` | `f32` | $\frac{\pi}{180.0}$ |
| `RAD_TO_DEG` | `f32` | $\frac{180.0}{\pi}$ |

#### `clamp`
```rust
pub fn clamp(v: f32, lo: f32, hi: f32) -> f32
```
Restricts `v` between `lo` and `hi`.

#### `lerp`
```rust
pub fn lerp(a: f32, b: f32, t: f32) -> f32
```
Linearly interpolates between `a` and `b`.

#### `almost_zero`
```rust
pub fn almost_zero(v: f32) -> bool
```
Returns `true` if $|v| < \text{EPSILON}$.

#### `almost_equal`
```rust
pub fn almost_equal(a: f32, b: f32) -> bool
```
Returns `true` if $|a - b| < \text{EPSILON}$.

#### `wrap_angle`
```rust
pub fn wrap_angle(angle: f32) -> f32
```
Wraps an angle so that it stays perfectly within the $[-\pi, \pi]$ range.
