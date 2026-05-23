# Iron Physics Engine: The Definitive Guide

This document is a deep-dive technical manual for the Iron Physics Engine. It is intended for developers who want to understand exactly how the engine works from the bit-level up to the high-level architecture.

---

## I. Project Philosophy and Structure

### Why a Workspace?
The project is organized as a **Cargo Workspace**. This is a Rust feature that allows multiple packages (crates) to share a single `Cargo.lock` and output directory.
- **`physics_math`**: A standalone crate for linear algebra. It has zero dependencies on physics logic. This makes it easier to test and potentially reuse in other projects (like a renderer).
- **`physics_core`**: The engine itself. It depends on `physics_math`.
- **`ironphysics` (Root)**: The binary entry point that orchestrates the two.

### Data-Oriented Design (DOD)
Most engines use Object-Oriented Programming (OOP), where each `Body` is a struct. We use **Structure of Arrays (SoA)**.
- **OOP (Array of Structures)**: `[Body, Body, Body]` where each Body contains `pos, vel, mass`.
- **DOD (Structure of Arrays)**: One array for all `pos`, one for all `vel`, one for all `mass`.
- **Why?** Modern CPUs are fast, but RAM is slow. CPUs use "prefetching" to grab contiguous memory. In OOP, when updating velocity, the CPU also loads position and mass into the cache (wasted space). In SoA, the CPU loads *only* velocities, fitting more relevant data in the cache and significantly speeding up the simulation.

---

## II. The `physics_math` Crate

### 1. `Vec2`: 2D Vectors
The foundation of all motion.

#### Rust Syntax & Features:
- **`#[derive(Debug, Clone, Copy, PartialEq)]`**: Automatically implements standard traits. `Copy` is crucial here; it allows vectors to be passed by value without moving ownership, just like an `f32`.
- **`#[repr(C)]`**: Tells Rust to lay out the struct in memory exactly like C. This is important for potential FFI (Foreign Function Interface) or GPU interoperability.
- **Operator Overloading**: We implement `std::ops::Add`, `Sub`, `Mul`, etc. This allows us to write `let c = a + b;` instead of `let c = a.add(b);`.

#### Key Math:
- **Dot Product (`a · b`)**: `x1*x2 + y1*y2`. Returns a scalar.
  - If `0`, vectors are perpendicular.
  - If `> 0`, they point in the same general direction.
- **Cross Product (2D)**: In 3D, a cross product returns a vector. In 2D, we treat it as the "Z" component of a 3D cross product: `x1*y2 - y1*x2`. This represents the "signed area" of the parallelogram formed by the vectors and is the basis for **Torque**.
- **Normalization**: Scaling a vector to length `1.0`. `v / v.length()`. We use `normalize_or_zero` to prevent crashes when the length is zero.

### 2. `Mat2`: 2D Rotation Matrix
We use a **Column-Major** layout.
```rust
pub struct Mat2 {
    pub cols: [Vec2; 2],
}
```
- **Column-Major**: The first `Vec2` is the first column. This is standard in OpenGL/DirectX.
- **Rotation Math**: To rotate a point by angle `θ`, we multiply it by the matrix:
  ```
  [ cosθ  -sinθ ] [ x ]   [ x*cosθ - y*sinθ ]
  [ sinθ   cosθ ] [ y ] = [ x*sinθ + y*cosθ ]
  ```

### 3. `Transform`
Combines a position and a rotation.
- **`apply(point)`**: Moves a point from "Local Space" (relative to the body) to "World Space".
- **`apply_inv(point)`**: The reverse. Useful for mouse interaction (converting mouse click to local body coordinates). It uses the **Transpose** of the rotation matrix, which for rotation matrices is conveniently equal to the inverse.

---

## III. The `physics_core` Crate: Memory & Storage

### 1. `GenerationalArena<T>`
This is our "Manual Memory Manager". In Rust, owning objects in a graph or list while deleting them is hard because of the Borrow Checker.

- **The Problem**: If you have an index `5` to a body, and that body is deleted, and a new body is created at index `5`, your old reference now points to the *wrong* body.
- **The Solution**: Every slot has a **Generation** number.
  - Slot 5, Generation 0: "Old Body".
  - Body deleted -> Slot 5 becomes Generation 1.
  - New body created -> Slot 5, Generation 1: "New Body".
- **Handles**: A `BodyHandle` contains `(index, generation)`. When you try to access a body, we check if the handle's generation matches the arena's current generation for that slot. If not, the access is denied. This is **100% Memory Safe**.

### 2. `BodyStorage`
The SoA implementation.
```rust
pub struct BodyStorage {
    pub position: Vec<Vec2>,
    pub linear_velocity: Vec<Vec2>,
    // ... many more vectors ...
}
```
Each vector is kept in sync. When we add a body, we push to *all* vectors. This ensures that `position[5]` always corresponds to `linear_velocity[5]`.

### 3. `Aabb` (Axis-Aligned Bounding Boxes)
Used for the **Broad-phase** of collision detection.
- **Math**: Defined by `min` and `max` points.
- **Overlap Test**: `(a.min.x <= b.max.x && a.max.x >= b.min.x) && (a.min.y <= b.max.y && a.max.y >= b.min.y)`.
- **Fattening**: We often "fatten" AABBs by a small margin. This prevents the need to update the broad-phase every time a body moves a microscopic amount.

---

## IV. The Physics System (The "World")

### 1. The Integration Step (`World::step`)
We use **Semi-Implicit Euler**. The order of operations is vital for stability.

#### Step-by-Step Logic:
1.  **Force Accumulation**: Gravity is applied to the `force` buffer: `force += gravity * mass`.
2.  **Velocity Update**:
    - `acceleration = force / mass`
    - `velocity += acceleration * dt`
3.  **Damping**: We reduce velocity slightly to simulate friction/air resistance: `velocity *= 1.0 - (damping * dt)`.
4.  **Position Update**:
    - `position += velocity * dt`
    - *Crucial*: We use the **new** velocity calculated in step 2. This makes the simulation "Symplectic," meaning it conserves energy much better than updating position with the *old* velocity.
5.  **Cleanup**: Reset `force` and `torque` to zero for the next frame. Sync the `Transform` and `Aabb`.

### 2. Angular Physics (Rotations)
- **Inertia**: The rotational equivalent of mass. A high inertia makes a body harder to spin.
- **Torque**: The rotational equivalent of force.
- **Apply Force at Point**:
  - When you hit a box at its corner, it moves *and* spins.
  - Linear Force: `force`
  - Torque: `(impact_point - center_of_mass) CROSS force`
  - This cross product gives us exactly how much "leverage" the force has to rotate the body.

---

## V. Advanced Rust Patterns Used

### 1. The View Pattern (`BodyView` and `BodyViewMut`)
Since our data is split across 15 different vectors in `BodyStorage`, how do we provide a nice API to the user?
We use "Proxy Structs".

```rust
pub struct BodyView<'a> {
    pub position: &'a Vec2,
    pub linear_velocity: &'a Vec2,
    // ...
}
```
When you call `world.body(handle)`, we:
1. Validate the handle using the Generational Arena.
2. If valid, we grab a **reference** to the data in every single array.
3. Package those references into a `BodyView`.

This uses **Rust Lifetimes (`'a`)**. The `BodyView` cannot live longer than the `World`. This prevents "Use-After-Free" bugs.

### 2. Generational Syncing
We maintain two arenas:
1. `body_arena`: A `GenerationalArena<()>` that tracks which slots are alive and what their generation is.
2. `bodies`: The `BodyStorage` (SoA) that holds the actual data.
They are kept perfectly in sync by index. This allows the high-performance SoA storage to be accessed via the safe Generational Handles.

### 3. Enums and Pattern Matching
In `arena.rs`, we use:
```rust
enum ArenaEntry<T> {
    Occupied { generation: u32, value: T },
    Free     { generation: u32 },
}
```
This is a **Tagged Union**. Rust forces us to handle both the `Occupied` and `Free` cases using `match` statements. This makes it impossible to accidentally access a "Free" slot as if it were "Occupied", providing compile-time safety that C++ cannot offer.

### 4. Functional Patterns
The engine uses functional-style methods like `.map()` and `.filter_map()` to handle `Option` types.
Example: `self.body_arena.get(...).map(|_| { ... })`.
- If the arena returns `None` (invalid handle), the `map` block is skipped entirely, and the function returns `None`.
- This eliminates "null pointer" checks and makes the code more expressive.

---

## VI. Summary of Math Applications

| Concept | Application | Formula |
| :--- | :--- | :--- |
| **Dot Product** | Collision Normal Projection | `depth = delta.dot(normal)` |
| **Cross Product** | Torque Calculation | `τ = r × F` |
| **Integration** | Moving bodies | `v = v + a*dt`, `p = p + v*dt` |
| **Mat2** | Rotation | `p' = R * p` |
| **AABB** | Broad-phase check | `min_a < max_b && min_b < max_a` |

---

## VII. Conclusion
The Iron Physics Engine combines **Low-Level Performance** (SoA, Manual Memory Management) with **High-Level Safety** (Generational Handles, Rust Borrow Checker, View Pattern). By separating the math into its own crate and focusing on DOD, it provides a foundation that is both fast and robust.
