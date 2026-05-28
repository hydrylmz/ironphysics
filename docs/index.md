# Iron Physics Scripting API

Welcome to the **Iron Physics Scripting API Reference**. 

This section of the documentation is structured for developers writing code that interacts with the Iron Physics engine. It provides an exhaustive listing of every public struct, method, and field exposed by the engine's modular crates.

---

## Namespaces / Crates

The Iron Physics API is divided into three primary crates:

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
The core physical solver, structural memory allocators, force integrators, and basic handles.

**Classes / Structs:**
*   [`World`](core.md#world): The main engine solver and coordinator.
*   [`BodyDesc`](core.md#bodydesc): The description template used to spawn a new rigid body.
*   [`WorldConfig`](core.md#worldconfig): The global configuration settings for stepping and integrations.
*   [`BodyHandle`](core.md#bodyhandle): Opaque token representing a safe reference to a body.
*   [`ColliderHandle`](core.md#colliderhandle): Opaque token representing a safe reference to a collider.
*   [`Material`](core.md#material): Holds physical material properties (friction, restitution, density).
*   [`BodyView`](core.md#bodyview), [`BodyViewMut`](core.md#bodyviewmut): Read-only and mutable reference proxies to an active body.
*   [`BodyStorage`](core.md#bodystorage): The underlying contiguous SoA memory layout for all bodies.
*   [`GenerationalArena<T>`](core.md#generationalarenat): The type-safe memory allocator that mitigates dangling pointers.

**Enums:**
*   [`BodyType`](core.md#bodytype): Classifies the rigid body as `Static`, `Kinematic`, or `Dynamic`.

---

### [physics_collision](collision.md)
The collision detection module housing shape geometry, spatial indexing (broadphase AABB trees), narrowphase dispatcher algorithms, and frame-scoped contact persistence.

**Classes / Structs:**
*   [`CollisionFilter`](collision.md#collisionfilter): Group-based and bitmask-based collision filtering rules.
*   [`Circle`](collision.md#circle): A circle primitive.
*   [`BoxShape`](collision.md#boxshape): A rectangle/oriented box primitive.
*   [`Capsule`](collision.md#capsule): A capsule cap primitive.
*   [`ConvexPolygon`](collision.md#convexpolygon): A convex polygon with up to 8 vertices.
*   [`MassProperties`](collision.md#massproperties): Calculated mass, inertia, and center of mass centroids.
*   [`ColliderDesc`](collision.md#colliderdesc): Blueprint descriptor passed to the world to create colliders.
*   [`ColliderStorage`](collision.md#colliderstorage): Structure of Arrays holding active collider allocations.
*   [`DynamicAabbTree`](collision.md#dynamicaabbtree): The broadphase tree utilizing fat AABB extensions for fast pairing.
*   [`ContactManifold`](collision.md#contactmanifold): Exact intersection result holding contact points.
*   [`ContactPoint`](collision.md#contactpoint): Touch location, depth, and persistent impulse tracking.
*   [`ContactFeatureId`](collision.md#contactfeatureid): Feature coordinate ID for warm-start persistence.
*   [`ContactPool`](collision.md#contactpool): Continuous preallocated arena recycling contact instances between steps.

**Enums:**
*   [`ShapeType`](collision.md#shapetype): Geometric categories (`Circle`, `Box`, `Capsule`, `Polygon`).
*   [`ContactFeatureKind`](collision.md#contactfeaturekind): Touch kinds (`Vertex`, `Face`).

**Traits:**
*   [`Shape`](collision.md#shape): Common trait interface implemented by all collision primitives.
