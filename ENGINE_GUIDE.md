# Iron Physics Engine Guide

Welcome to the comprehensive guide for the Iron Physics Engine. This document explains the inner workings, architectural decisions, and mathematical foundations of the engine.

---

## 1. Low-Level Mechanics

### Semi-Implicit Euler Integration
The engine's heart is the `World::step` method, which uses **Semi-Implicit Euler** integration. Unlike standard Euler integration, we update velocity *before* position:

1.  **Accumulate Forces**: Sum all forces (gravity, user-applied forces).
2.  **Update Velocity**: `v_next = v_curr + (force / mass) * dt`
3.  **Update Position**: `p_next = p_curr + v_next * dt` (using the *new* velocity)

This provides better stability for oscillatory systems (like springs) and conserves energy better than explicit Euler.

### Data-Oriented Design (SoA)
Internally, the engine uses a **Structure of Arrays (SoA)** pattern via `BodyStorage`. Instead of an array of `Body` objects, we have separate arrays for `position`, `velocity`, `mass`, etc.
- **Why?** This maximizes CPU cache hits during the physics step. When we iterate to update positions, the CPU loads a contiguous block of positions into the cache, rather than jumping between large, fragmented object structures.

---

## 2. Rust-wise Implementation

### Generational Arena & Safety
To manage bodies without the risks of raw pointers or the overhead of `Arc<Mutex<T>>`, the engine uses a **Generational Arena**.
- **Handles**: When you add a body, you receive a `BodyHandle`. This contains an `index` (u32) and a `generation` (u32).
- **ABA Prevention**: If a body is removed and a new one is created at the same index, the generation increments. The old handle becomes invalid, preventing bugs where you accidentally modify the wrong object (the "ABA problem").
- **Syncing**: The `World` maintains a `GenerationalArena<()>` in parallel with `BodyStorage`. This ensures that every storage slot is guarded by a generation check.

### The View Pattern
Because data is stored in SoA format, accessing a "Body" as a single object is tricky. We solve this with the **View Pattern**:
- `BodyView`: A temporary struct holding references to various arrays in `BodyStorage`.
- `BodyViewMut`: The mutable version.
This allows the API to feel object-oriented while keeping the underlying data high-performance and contiguous.

### Crate Workspace
The project is split into two main crates:
- `physics_math`: Pure, zero-dependency linear algebra.
- `physics_core`: The physics simulation logic that depends on `physics_math`.

---

## 3. Architecture

### The World
The `World` struct is the central coordinator. It owns the `BodyStorage` and the `GenerationalArena`. It handles the lifecycle of bodies and orchestrates the simulation steps.

### Separation of Concerns
1.  **Math Layer**: Handles vectors, matrices, and transforms.
2.  **Storage Layer**: Manages raw memory and SoA layouts.
3.  **Logic Layer**: Implements integration, force application, and (eventually) collision resolution.

---

## 4. Mathematics

### Vector2 Operations
Our `Vec2` implementation supports standard linear algebra:
- **Dot Product**: `a.x * b.x + a.y * b.y`. Used for projections and determining angles.
- **Cross Product (2D)**: `a.x * b.y - a.y * b.x`. Result is a scalar representing the "z-component" of a 3D cross product. Essential for torque: `torque = r.cross(force)`.
- **Perpendicular Vector**: `(-y, x)`. Rotates a vector 90 degrees counter-clockwise.

### Matrices and Transforms
- **Mat2**: A column-major 2x2 matrix. We use it primarily for rotation:
  ```rust
  [ cosθ  -sinθ ]
  [ sinθ   cosθ ]
  ```
- **Transform**: Combines a `Vec2` position and a rotation angle. It provides `apply` and `apply_inv` methods to move points between local and world space using the formula: `world_point = (RotationMatrix * local_point) + translation`.

### AABBs
Axis-Aligned Bounding Boxes are used for **Broad-phase Collision Detection**. They provide a fast way to check if two objects *might* be colliding before running expensive per-pixel or per-vertex checks.

---

## 5. Current Math Applications

### Motion Integration
Used in every frame to advance the simulation.
- `linear_velocity += (force * inv_mass) * dt`
- `angular_velocity += (torque * inv_inertia) * dt`

### Force at Point
Calculates how a force applied at an arbitrary point on a body affects both its linear and angular motion:
- Linear effect: `force`
- Angular effect (Torque): `(impact_point - center).cross(force)`

### Damping
Simulates air resistance or friction by scaling velocities down slightly each frame:
- `velocity *= (1.0 - damping * dt)`

### Transform Synchronization
After updating positions and angles, the engine recalculates the `Mat2` rotation matrices so that rendering and collision logic have access to the latest orientation data.
