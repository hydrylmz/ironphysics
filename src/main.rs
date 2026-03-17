mod math;
mod physics;
use math::vec2::Vector2;
use physics::rigid_body::ForceType;
use physics::rigid_body::RigidBody;
use std::thread::sleep;
use std::time::Duration;

fn main() {
    // 1. Create a "Physics World" with 3 different objects
    let mut bodies = vec![
        // Body 0: A heavy ball falling from high up
        RigidBody::new(Vector2::new(0.0, 10.0), Vector2::new(1.0, 0.0), 2.0),
        // Body 1: A light ball moving fast
        RigidBody::new(Vector2::new(5.0, 5.0), Vector2::new(-5.0, 2.0), 0.5),
        // Body 2: A medium ball
        RigidBody::new(Vector2::new(-3.0, 8.0), Vector2::new(0.0, 0.0), 1.0),
    ];

    // Customize individual properties
    bodies[0].radius = 0.8;
    bodies[1].drag_coefficient = 0.05; // Low drag
    bodies[2].drag_coefficient = 0.8; // High drag (Parachute effect)

    let restitution = 0.7;
    let dt = 0.016;

    println!("--- Running Iron Physics Simulation ---");

    for step in 0..50 {
        println!("\n[Frame {}]", step);

        // --- PHASE 1: Update each body (Gravity, Drag, Movement) ---
        for body in bodies.iter_mut() {
            body.update(dt);
        }

        // --- PHASE 2: Handle World Bounds (Floor & Walls) ---
        for body in bodies.iter_mut() {
            // Floor Check (y = 0)
            if body.position.y - body.radius < 0.0 {
                body.position.y = body.radius; // Snap to surface
                body.velocity.y *= -restitution; // Bounce!
                body.velocity.x *= 0.9; // Friction
            }
            // Side Walls (x = -10 to 10)
            if body.position.x.abs() + body.radius > 10.0 {
                body.position.x = 10.0 * body.position.x.signum();
                body.velocity.x *= -restitution;
            }
        }

        // --- PHASE 3: Handle Object-to-Object Collisions ---
        let len = bodies.len();
        for i in 0..len {
            for j in (i + 1)..len {
                // Safe split to get two mutable references
                let (left, right) = bodies.split_at_mut(j);
                let obj1 = &mut left[i];
                let obj2 = &mut right[0];

                if RigidBody::is_colliding(obj1, obj2) {
                    println!("  💥 Collision: Obj {} + Obj {}", i, j);
                    RigidBody::resolve_penetration(obj1, obj2);
                    RigidBody::resolve_velocity(obj1, obj2, restitution);
                }
            }
        }

        // --- PHASE 4: Print Status of the first two objects ---
        for (i, b) in bodies.iter().enumerate().take(2) {
            println!(
                "  Obj {}: Pos({:>6.2}, {:>6.2}) | Vel({:>6.2}, {:>6.2})",
                i, b.position.x, b.position.y, b.velocity.x, b.velocity.y
            );
        }
    }
}
