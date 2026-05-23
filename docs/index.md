# Iron Physics Scripting API

Welcome to the **Iron Physics Scripting API Reference**. 

This section of the documentation is structured for developers writing code that interacts with the Iron Physics engine. It provides an exhaustive listing of every public struct, method, and field exposed by the engine's modular crates.

---

## Namespaces / Crates

The Iron Physics API is divided into two primary crates:

### [physics_math](math.md)
The foundational 2D math library handling all linear algebra, vectors, matrices, bounds, and geometric transformations.

**Classes / Structs:**
*   [`Vec2`](math.md#vec2): 2D Vector algebra.
*   [`Mat2`](math.md#mat2): 2D Column-major $2 \times 2$ matrices.
*   [`Transform`](math.md#transform): Positional and rotational coordinates.
*   [`Aabb`](math.md#aabb): Axis-Aligned Bounding Box.

**Globals / Scalars:**
*   [`EPSILON`](math.md#scalar-utilities): Global floating-point tolerance ($1e-6$).
*   [`almost_zero`](math.md#scalar-utilities), [`almost_equal`](math.md#scalar-utilities), [`wrap_angle`](math.md#scalar-utilities): Scalar mathematical helpers.

---

### [physics_core](core.md)
The core physical solver, structural memory allocators, force integrators, and broadphase mechanics.

**Classes / Structs:**
*   [`World`](core.md#world): The main engine solver and coordinator.
*   [`BodyDesc`](core.md#bodydesc): The description template used to spawn a new rigid body.
*   [`WorldConfig`](core.md#worldconfig): The global configuration settings for stepping and integrations.
*   [`BodyHandle`](core.md#bodyhandle): Opaque token representing a safe reference to a body.
*   [`BodyView`](core.md#bodyview), [`BodyViewMut`](core.md#bodyviewmut): Read-only and mutable reference proxies to an active body.
*   [`BodyStorage`](core.md#bodystorage): The underlying contiguous SoA memory layout for all bodies.
*   [`GenerationalArena<T>`](core.md#generationalarenat): The type-safe memory allocator that mitigates dangling pointers.

**Enums:**
*   [`BodyType`](core.md#bodytype): Classifies the rigid body as `Static`, `Kinematic`, or `Dynamic`.
