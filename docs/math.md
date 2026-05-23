# Mathematics Module (`physics_math`)

The `physics_math` crate is a self-contained, low-overhead linear algebra library designed specifically for 2D rigid-body simulations. By avoiding heavy, generic math frameworks, it achieves optimal compile times and maximal optimization potential.

---

## 1. 2D Vectors (`vec2.rs`)

The core workhorse of geometric calculations is `Vec2`. It is stored as a standard C-compatible structure of two floating-point values:

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}
```

### Operator Overloads
`Vec2` implements standard arithmetic operators via `std::ops`, enabling clean mathematical expressions:
*   **Addition / Subtraction**: `a + b`, `a - b`, `a += b`, `a -= b`
*   **Scalar Multiplication / Division**: `v * s`, `s * v`, `v / s`, `v *= s`
*   **Negation**: `-v`

### Vector Operations & Formulas

#### Dot Product
The dot product computes the scalar projection of one vector onto another:
$$\vec{a} \cdot \vec{b} = a_x b_x + a_y b_y$$
```rust
pub fn dot(self, rhs: Self) -> f32;
```
*   **Application**: Used to calculate projection lengths, relative speeds along contact normals, and angle cosines between vectors.

#### Cross Product (2D)
In 2D, the cross product is a scalar representing the signed magnitude of the 3D cross product perpendicular to the 2D plane:
$$\vec{a} \times \vec{b} = a_x b_y - a_y b_x$$
```rust
pub fn cross(self, rhs: Self) -> f32;
```
*   **Application**: Used to check rotational directions (Clockwise vs. Counter-Clockwise), compute perpendicular torques, and resolve angular impulses ($\vec{r} \times \vec{F}$).

#### Perpendicular Vector
Computes the vector rotated 90 degrees counter-clockwise:
$$\vec{a}^{\perp} = \begin{bmatrix} -a_y \\ a_x \end{bmatrix}$$
```rust
pub fn perp(self) -> Self;
```
*   **Application**: Quick perpendicular projection, useful when computing contact tangent directions.

#### Normalization
Scaling a vector to have a unit length of $1.0$:
$$\hat{u} = \frac{\vec{u}}{\|\vec{u}\|} = \frac{\vec{u}}{\sqrt{u_x^2 + u_y^2}}$$
*   `normalize()`: Normalizes the vector, panicking if it is a zero vector.
*   `normalize_or_zero()`: Safely returns `Vec2::zero()` if the length squared is less than `EPSILON`.

---

## 2. 2D Matrices (`mat2.rs`)

The `Mat2` struct represents a column-major $2 \times 2$ transformation matrix:

$$\mathbf{A} = \begin{bmatrix} m_{00} & m_{01} \\ m_{10} & m_{11} \end{bmatrix} = \begin{bmatrix} \text{cols}[0].x & \text{cols}[1].x \\ \text{cols}[0].y & \text{cols}[1].y \end{bmatrix}$$

```rust
pub struct Mat2 {
    pub cols: [Vec2; 2],
}
```

### Key Matrix Formulas

#### Rotation Matrix
Constructs a rotation matrix around the origin from an angle $\theta$ in radians:
$$\mathbf{R}(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$$
```rust
pub fn from_angle(theta: f32) -> Self;
```

#### Transposition
Swaps columns and rows:
$$\mathbf{A}^T = \begin{bmatrix} a_{00} & a_{10} \\ a_{01} & a_{11} \end{bmatrix}$$
```rust
pub fn transpose(self) -> Self;
```
*   **Physics Detail**: For orthonormal rotation matrices, transposition is equivalent to inversion ($\mathbf{R}^T = \mathbf{R}^{-1}$). This provides an extremely cheap $O(1)$ inverse transform step!

#### Determinant
Computes the matrix determinant:
$$\det(\mathbf{A}) = a_{00} a_{11} - a_{01} a_{10}$$
```rust
pub fn det(self) -> f32;
```

#### Inversion
Computes the mathematical inverse of a matrix:
$$\mathbf{A}^{-1} = \frac{1}{\det(\mathbf{A})} \begin{bmatrix} a_{11} & -a_{01} \\ -a_{10} & a_{00} \end{bmatrix}$$
```rust
pub fn inverse(self) -> Option<Self>;
```
*   Returns `None` if the matrix is singular ($\det(\mathbf{A}) \approx 0$).

#### Vector Multiplication
Multiplies a matrix by a column vector:
$$\mathbf{A}\vec{v} = v_x \cdot \mathbf{A}_{\text{col}0} + v_y \cdot \mathbf{A}_{\text{col}1} = \begin{bmatrix} a_{00} v_x + a_{01} v_y \\ a_{10} v_x + a_{11} v_y \end{bmatrix}$$
```rust
pub fn mul_vec(self, v: Vec2) -> Vec2;
```

---

## 3. Coordinate Transformations (`transform.rs`)

The `Transform` structure acts as a compact pose representation, combining a translation vector and a rotational scalar:

```rust
pub struct Transform {
    pub position: Vec2,
    pub rotation: f32, // Angle in radians
}
```

### Transforming Coordinates

#### Local to World (Forward Apply)
Transforms a coordinate from a body's local coordinate system into the global world space:
$$P_{\text{world}} = \mathbf{R}(\theta) \cdot P_{\text{local}} + \vec{x}$$
```rust
pub fn apply(&self, local_point: Vec2) -> Vec2;
```

#### World to Local (Inverse Apply)
Transforms a global coordinate back into the body's local space:
$$P_{\text{local}} = \mathbf{R}(\theta)^T \cdot \left(P_{\text{world}} - \vec{x}\right)$$
```rust
pub fn apply_inv(&self, world_point: Vec2) -> Vec2;
```

#### Compounding (Transform Composition)
Combines two transforms (e.g. parent-to-child composition):
$$T_{\text{combined}} = T_{\text{parent}} \circ T_{\text{child}}$$
$$\vec{x}_{\text{new}} = T_{\text{parent}}.\text{apply}(T_{\text{child}}.\vec{x})$$
$$\theta_{\text{new}} = T_{\text{parent}}.\theta + T_{\text{child}}.\theta$$
```rust
pub fn combine(&self, child: &Transform) -> Transform;
```

---

## 4. Axis-Aligned Bounding Boxes (`aabb.rs`)

The `Aabb` struct defines a rectangular boundary aligned with the global coordinate axes, widely used in Broadphase collision detection to quickly prune non-colliding objects:

```rust
pub struct Aabb {
    pub min: Vec2,
    pub max: Vec2,
}
```

### AABB Utilities

*   **Overlaps Check**: Determines if two bounding boxes intersect:
    $$\text{overlaps} \iff (A_{\min.x} \le B_{\max.x} \land B_{\min.x} \le A_{\max.x}) \land (A_{\min.y} \le B_{\max.y} \land B_{\min.y} \le A_{\max.y})$$
*   **Merge**: Computes the union bounding box that completely encloses both AABBs.
*   **Fatten**: Expands the AABB boundary uniformly by a safety margin. In collision pipelines, this acts as a cache, preventing rebuilding broadphase structures on tiny micro-movements.

---

## 5. Floating-Point Tolerances (`scalar.rs`)

To prevent numerical instabilities in rigid-body joints and contact constraints, the engine implements epsilon-based float checks:

*   `EPSILON = 1e-6`: The default floating-point tolerance limit.
*   `almost_zero(v: f32) -> bool`: Checks if $|v| < \epsilon$, replacing unsafe `v == 0.0` comparisons.
*   `almost_equal(a: f32, b: f32) -> bool`: Checks if $|a - b| < \epsilon$.
*   `wrap_angle(angle: f32) -> f32`: Normalizes any rotation angle to fit into the standard range $[-\pi, \pi]$:
    $$\theta_{\text{wrapped}} = ((\theta + \pi) \pmod{2\pi}) - \pi$$
