# Quickstart & Usage Guide

This guide walks you through setting up Iron Physics in your Rust projects, spawning bodies, stepping simulations, and interacting with rigid bodies programmatically.

---

## 1. Installation

Add `ironphysics` to your project's `Cargo.toml`. Since Iron Physics is structured as a multi-crate workspace, you can reference it directly from your repository:

```toml
[dependencies]
ironphysics = { git = "https://github.com/hydrylmz/ironphysics.git" }
```

---

## 2. Spawning a Physics World

The `World` is the central coordinator of the simulation. You must initialize it with a `WorldConfig` configuration.

```rust
use ironphysics::{World, WorldConfig, Vec2};

fn main() {
    // 1. Create a configuration (contains iteration, sleep, and stabilization settings)
    let mut config = WorldConfig::default();
    
    // Customize configuration settings if desired
    config.allow_sleeping = true;
    config.velocity_iterations = 8;
    
    // 2. Spawn the physics world
    let mut world = World::new(config);
    
    // 3. Define the global gravity vector (e.g. downward gravity of -9.81 m/s²)
    world.gravity = Vec2::new(0.0, -9.81);
}
```

---

## 3. Spawning Rigid Bodies

Rigid bodies are spawned by filling out a `BodyDesc` (Body Description) structure and passing it to the world. The world processes the description, registers the body inside the data-oriented arrays, and returns a safe, copyable `BodyHandle`.

### Spawning a Static Ground Body

Static bodies have infinite mass ($\text{inv\_mass} = 0.0$) and act as stationary boundaries (like ground, floors, or heavy structural columns).

```rust
use ironphysics::{BodyDesc, BodyType, Vec2, Transform, Aabb};

let ground_desc = BodyDesc {
    body_type: BodyType::Static,
    position: Vec2::new(0.0, -2.0), // Center the ground 2 meters down
    linear_velocity: Vec2::zero(),
    angle: 0.0,
    angular_velocity: 0.0,
    force: Vec2::zero(),
    torque: 0.0,
    inv_mass: 0.0,      // Infinite mass
    inv_inertia: 0.0,   // Infinite inertia
    transform: Transform::default(),
    aabb: Aabb::default(),
    gravity_scale: 0.0, // Unaffected by gravity
    linear_damping: 0.0,
    angular_damping: 0.0,
    is_awake: true,
    fixed_rotation: true,
    user_data: Some(101), // Optional metadata tag
};

let ground_handle = world.add_body(ground_desc);
```

### Spawning a Dynamic Falling Box

Dynamic bodies are fully simulated objects that fall under gravity, slide, rotate, collide, and react to external forces.

```rust
let box_desc = BodyDesc {
    body_type: BodyType::Dynamic,
    position: Vec2::new(0.0, 10.0), // Start 10 meters in the air
    linear_velocity: Vec2::new(2.0, 0.0), // Give it an initial sideways velocity of 2 m/s
    angle: 45.0 * std::f32::consts::PI / 180.0, // Start rotated at 45 degrees
    angular_velocity: 0.5, // Tilted spin
    force: Vec2::zero(),
    torque: 0.0,
    inv_mass: 1.0 / 5.0,     // 5.0 kg mass -> inv_mass = 0.2
    inv_inertia: 1.0 / 2.0,  // 2.0 kg*m² inertia -> inv_inertia = 0.5
    transform: Transform::default(),
    aabb: Aabb::default(),
    gravity_scale: 1.0,      // Responds fully to gravity
    linear_damping: 0.05,    // Subtle air resistance
    angular_damping: 0.05,
    is_awake: true,
    fixed_rotation: false,   // Allow the box to spin freely
    user_data: Some(102),
};

let box_handle = world.add_body(box_desc);
```

---

## 4. Stepping the Simulation & Rendering

To run the physics simulation, you step the world at a fixed timestep (typically 60 Hz, meaning $\Delta t = 1/60$ seconds). 

After stepping the world, retrieve coordinates using `world.body(handle)` to update your game sprites or console drawers.

```rust
let dt = 1.0 / 60.0; // Fixed timestep

loop {
    // 1. Advance the physics simulation
    world.step(dt);
    
    // 2. Fetch the updated state of our box
    if let Some(body) = world.body(box_handle) {
        let position = body.position;
        let angle = body.angle;
        
        println!(
            "Draw box at x: {:.3}, y: {:.3} with rotation: {:.3} rad", 
            position.x, position.y, angle
        );
        
        // 3. Break the loop if the box hits the ground
        if position.y <= -2.0 {
            println!("The box has landed!");
            break;
        }
    }
}
```

---

## 5. Applying Forces & Impulses

You can interact with active bodies in real-time. The engine supports two methods of force application:

### Continuous Forces (`apply_force`)
Forces accumulate over the duration of a timestep. Use forces for continuous effects like rocket thrusters, magnets, gravity fields, or strong wind.

```rust
// Apply a continuous upward force (e.g. thruster pushing with 15 Newtons)
let thruster_force = Vec2::new(0.0, 15.0);
world.apply_force(box_handle, thruster_force);
```

### Instantaneous Impulses (`apply_impulse`)
Impulses immediately alter the linear velocity of a body, bypassing the integration accumulator. Use impulses for instant events like explosions, impacts, or jumping character controls.

$$\vec{v}_{\text{new}} = \vec{v}_{\text{old}} + \vec{P} \cdot m_{\text{inv}}$$

```rust
// Apply an instantaneous forward blast impulse of 10 N·s
let blast_impulse = Vec2::new(10.0, 0.0);
world.apply_impulse(box_handle, blast_impulse);
```

### Rotational Forces & Torque
*   **Torque**: Accumulate continuous rotational torque over the step using `apply_torque(handle, torque)`.
*   **Force at Offset Point**: Use `apply_force_at_point(handle, force, world_point)` to apply a force at an offset position relative to the body's center of mass. This automatically calculates and applies the resulting rotational torque:
    $$\tau_{\text{applied}} = (\vec{P}_{\text{world}} - \vec{x}_{\text{body}}) \times \vec{F}$$

---

## 6. Advanced Customizations

### Character Controller (Fixed Rotation)
For character controllers, you usually want to prevent players from tipping or rolling over. You can lock rotation by setting `fixed_rotation = true`. This forces `inv_inertia` to `0.0` internally, keeping the body upright:

```rust
let mut player_desc = BodyDesc {
    body_type: BodyType::Dynamic,
    position: Vec2::new(0.0, 0.0),
    fixed_rotation: true, // Will not rotate under external forces
    // ...
};
```

### Custom Gravity Scales
Customize gravity responses per body. For example, make helium balloons float upwards, or feathers drift down slowly:

```rust
// Floating Balloon
balloon_desc.gravity_scale = -0.2; // Gravity pulls upward

// Zero-Gravity Satellite
satellite_desc.gravity_scale = 0.0; // Completely floats, unaffected by world gravity
```
