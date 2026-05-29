# Rigid-bodies

The real-time simulation of rigid-bodies subjected to forces and contacts is the main feature of a physics engine for video games, robotics, or animation. Rigid-bodies are typically used to simulate the dynamics of non-deformable solids as well as to integrate the trajectory of solids whose velocities are controlled by the user (e.g. moving platforms).

Note that rigid-bodies are only responsible for the dynamics and kinematics of the solid. A rigid-body without a collider attached to it will not be affected by contacts (because there is no shape to compute contacts against). See the [Collision](collision.md) guide for adding shapes to your bodies.

---

## Creation and insertion

A rigid-body is described using a `BodyDesc` structure and then inserted into the `World` via `World::add_body()`. The insertion returns a `BodyHandle` — an opaque token you keep to reference the body later.

!!! info
    The following examples show several fields that can be set on `BodyDesc`. The values used are illustrative; a real simulation would use values appropriate to your scene's scale.

```rust
use ironphysics::physics_core::*;
use ironphysics::physics_math::Vec2;

// Create the world.
let mut world = World::new(WorldConfig::default());

// Describe a dynamic rigid-body at position (0, 10).
let body_desc = BodyDesc {
    body_type: BodyType::Dynamic,
    position: Vec2::new(0.0, 10.0),
    linear_velocity: Vec2::new(1.0, 0.0),
    angle: 0.0,
    angular_velocity: 0.0,
    inv_mass: 1.0,          // mass = 1 kg
    inv_inertia: 1.0,
    gravity_scale: 1.0,
    linear_damping: 0.1,
    angular_damping: 0.05,
    is_awake: true,
    fixed_rotation: false,
    user_data: None,
};

// Insert the body and keep the handle.
let handle: BodyHandle = world.add_body(body_desc);
```

The `BodyHandle` bundles a **slot index** and a **generation counter** into a single 64-bit value. This means that if a body is removed and its slot is recycled, any old handles pointing to it will be recognized as stale automatically.

---

## Rigid-body type

The `body_type` field of `BodyDesc` controls how the body interacts with the simulation.

| Type | Description |
| :--- | :--- |
| `BodyType::Dynamic` | Fully simulated. Reacts to gravity, forces, impulses, and collisions. |
| `BodyType::Kinematic` | Not affected by forces or collisions. Moves only via its assigned velocity, useful for moving platforms or animated obstacles. |
| `BodyType::Static` | Immovable. Acts as infinite-mass geometry that dynamic bodies collide against, such as floors and walls. |

!!! tip
    Use `Static` for world geometry and `Dynamic` for objects that should respond to physics. `Kinematic` bodies are ideal for player-controlled or scripted objects that still need to push `Dynamic` bodies around.

```rust
// A static floor.
let floor = world.add_body(BodyDesc {
    body_type: BodyType::Static,
    position: Vec2::new(0.0, 0.0),
    ..BodyDesc::default()
});

// A kinematic platform that moves horizontally.
let platform = world.add_body(BodyDesc {
    body_type: BodyType::Kinematic,
    position: Vec2::new(-5.0, 3.0),
    linear_velocity: Vec2::new(2.0, 0.0),
    ..BodyDesc::default()
});

// A dynamic ball that falls and bounces.
let ball = world.add_body(BodyDesc {
    body_type: BodyType::Dynamic,
    position: Vec2::new(0.0, 10.0),
    gravity_scale: 1.0,
    ..BodyDesc::default()
});
```

---

## Mass and inertia

IronPhysics works with **inverse mass** (`inv_mass`) and **inverse inertia** (`inv_inertia`) rather than mass directly. This avoids a division on every solver iteration and naturally represents infinite mass with the value `0.0`.

| `inv_mass` | Effective mass |
| :--- | :--- |
| `1.0 / m` | Mass of `m` kg |
| `0.0` | Infinite mass (body cannot be linearly displaced) |

!!! warning
    Setting `inv_mass = 0.0` on a `Dynamic` body will mean forces and impulses have no effect on its linear velocity. This is normally only appropriate for `Static` bodies.

### Locking rotation

If you want a body to move but never rotate — common for top-down characters or voxel objects — set `fixed_rotation: true`. IronPhysics will internally force `inv_inertia` to `0.0`, preventing any torque from spinning the body.

```rust
// A character capsule that slides but never tips over.
let character = world.add_body(BodyDesc {
    body_type: BodyType::Dynamic,
    position: Vec2::new(0.0, 5.0),
    inv_mass: 1.0 / 70.0,   // 70 kg character
    fixed_rotation: true,    // cannot be rotated by forces
    ..BodyDesc::default()
});
```

---

## Damping

Damping simulates energy loss due to air resistance or friction with the environment. It is applied every step as a multiplicative factor that gradually reduces velocity.

| Field | Effect |
| :--- | :--- |
| `linear_damping` | Reduces linear velocity each step. |
| `angular_damping` | Reduces angular velocity each step. |

A value of `0.0` means no damping. Higher values make the body slow down faster.

!!! info
    Damping is **not** the same as friction. Friction is a contact response computed during collision resolution. Damping is always applied, even when the body is in free flight.

```rust
// A body with high air resistance — floaty, like a balloon.
let balloon = world.add_body(BodyDesc {
    body_type: BodyType::Dynamic,
    position: Vec2::new(0.0, 2.0),
    inv_mass: 1.0 / 0.05,  // very light
    gravity_scale: -0.5,    // floats upward slightly
    linear_damping: 0.8,    // heavy drag
    angular_damping: 0.9,
    ..BodyDesc::default()
});
```

---

## Sleeping

IronPhysics can automatically **sleep** bodies that have been nearly still for a period of time. A sleeping body is excluded from the solver, saving CPU time with no visible difference in the simulation.

Sleeping is configured globally on `WorldConfig`:

| Field | Default | Description |
| :--- | :--- | :--- |
| `allow_sleeping` | `true` | Enables the sleep system globally. |
| `sleep_time_required` | `0.5 s` | How many consecutive seconds of inactivity before a body is put to sleep. |

Bodies wake up automatically when a force, impulse, or contact is applied to them. You can also control the initial state of a body using the `is_awake` field on `BodyDesc`.

!!! tip
    Sleeping is almost always a net win. Only disable it (`allow_sleeping: false`) if you observe bodies incorrectly sleeping — for example, a body resting on a very-slowly-moving kinematic platform.

```rust
let config = WorldConfig {
    allow_sleeping: true,
    sleep_time_required: 0.5,
    ..WorldConfig::default()
};
let mut world = World::new(config);

// Spawn a body that starts asleep (e.g. pre-placed scenery).
let debris = world.add_body(BodyDesc {
    body_type: BodyType::Dynamic,
    position: Vec2::new(4.0, 1.0),
    is_awake: false,
    ..BodyDesc::default()
});
```

---

## Applying forces and impulses

Once a body is in the world, you can drive its motion by applying forces, impulses, and torques through the `World` struct.

### Forces (continuous)

Forces are accumulated across the step and cause a gradual acceleration. Use them for things like thrust, wind, or buoyancy that act over time.

```rust
// Apply a constant upward thrust to a rocket body.
world.apply_force(rocket_handle, Vec2::new(0.0, 500.0));

// Apply a force at a world-space point (also generates torque).
let wing_tip = Vec2::new(3.0, 5.0);
world.apply_force_at_point(plane_handle, Vec2::new(0.0, 200.0), wing_tip);
```

### Impulses (instantaneous)

Impulses produce an immediate change in velocity. Use them for explosions, jumps, or sudden collisions resolved outside the physics pipeline.

```rust
// Make the character jump.
let jump_impulse = Vec2::new(0.0, 8.0);
world.apply_impulse(character_handle, jump_impulse);
```

### Torque (angular force)

`apply_torque` works like `apply_force` but for rotation — it accumulates angular acceleration over the step.

```rust
// Spin a wheel continuously.
world.apply_torque(wheel_handle, 50.0); // positive = counter-clockwise
```

!!! info
    All force/impulse/torque calls **wake up** a sleeping body automatically if the body is currently asleep.

---

## Reading body state

After stepping the simulation, you can query a body's current transform, velocity, and other properties using `World::body()`.

```rust
// Advance the simulation by one frame (16ms).
world.step(1.0 / 60.0);

// Read back the body's state.
if let Some(view) = world.body(handle) {
    println!("Position : {:?}", view.position);
    println!("Velocity : {:?}", view.linear_velocity);
    println!("Angle    : {:.3} rad", view.angle);
}
```

Use `World::body_mut()` when you need to modify the body's state directly (e.g., teleporting it or zeroing its velocity).

```rust
if let Some(mut view) = world.body_mut(handle) {
    // Teleport the body.
    view.position = Vec2::new(0.0, 20.0);
    // Zero out velocity after teleport.
    view.linear_velocity = Vec2::ZERO;
}
```

---

## Removing bodies

Call `World::remove_body()` when a body is no longer needed. The underlying memory slot is recycled and the generation counter is incremented, so any existing `BodyHandle`s pointing to that slot become stale and will return `None` on subsequent lookups.

```rust
world.remove_body(handle);

// Old handle is now stale — returns None.
assert!(world.body(handle).is_none());
```
