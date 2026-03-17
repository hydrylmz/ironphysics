use macroquad::prelude::*;
mod math;
mod physics;
use math::vec2::Vector2;
use physics::rigid_body::RigidBody;

// THIS MAIN FUNCTION WAS ENTIRELY WRITTEN BY AI, TO VISUALIZE THE CURRENT SYSTEM
fn conf() -> Conf {
    Conf {
        window_title: "Iron Physics".to_owned(),
        window_width: 800,
        window_height: 600,
        ..Default::default()
    }
}

#[macroquad::main(conf)]
async fn main() {
    let mut bodies = vec![
        RigidBody::new(Vector2::new(3.0, 40.0), Vector2::new(2.0, 0.0), 2.0),
        RigidBody::new(Vector2::new(5.0, 5.0), Vector2::new(-5.0, 2.0), 0.5),
        RigidBody::new(Vector2::new(-7.0, 8.0), Vector2::new(0.0, 0.0), 1.0),
        RigidBody::new(Vector2::new(-15.0, 15.0), Vector2::new(4.0, -2.0), 1.5),
    ];

    bodies[0].radius = 0.8;
    bodies[1].drag_coefficient = 0.05;
    bodies[2].drag_coefficient = 0.8;
    bodies[3].radius = 1.2;

    let restitution = 0.7;
    let world_width = 20.0;
    let world_height = 20.0;

    loop {
        clear_background(BLACK);

        let dt = get_frame_time();
        // Limit dt to avoid instability if window is moved or frame drops
        let dt = dt.min(0.033);

        // --- PHASE 1: Update ---
        for body in bodies.iter_mut() {
            body.update(dt);
        }

        // --- PHASE 2: World Bounds ---
        for body in bodies.iter_mut() {
            // Floor (y = 0)
            if body.position.y - body.radius < 0.0 {
                body.position.y = body.radius;
                body.velocity.y *= -restitution;
                body.velocity.x *= 0.95; // Ground friction
            }
            // Side Walls (x = -world_width/2 to world_width/2)
            let half_width = world_width / 2.0;
            if body.position.x.abs() + body.radius > half_width {
                body.position.x = (half_width - body.radius) * body.position.x.signum();
                body.velocity.x *= -restitution;
            }
        }

        // --- PHASE 3: Collisions ---
        let len = bodies.len();
        for i in 0..len {
            for j in (i + 1)..len {
                let (left, right) = bodies.split_at_mut(j);
                let obj1 = &mut left[i];
                let obj2 = &mut right[0];

                if RigidBody::is_colliding(obj1, obj2) {
                    RigidBody::resolve_penetration(obj1, obj2);
                    RigidBody::resolve_velocity(obj1, obj2, restitution);
                }
            }
        }

        // --- PHASE 4: Draw ---
        let screen_w = screen_width();
        let screen_h = screen_height();
        let scale = f32::min(screen_w / world_width, screen_h / world_height);
        let scale_x = scale;
        let scale_y = scale;

        // Ground line
        let ground_y = screen_h - (0.0 * scale_y);
        draw_line(0.0, ground_y, screen_w, ground_y, 2.0, GRAY);

        for (i, body) in bodies.iter().enumerate() {
            // Transform physics coords to screen coords
            // Physics: x is centered, y grows up from 0
            // Screen: x starts left, y grows down from top
            let sx = (body.position.x + world_width / 2.0) * scale_x;
            let sy = screen_h - (body.position.y * scale_y);
            let s_radius = body.radius * scale_x;

            let color = match i % 4 {
                0 => RED,
                1 => GREEN,
                2 => BLUE,
                _ => YELLOW,
            };

            draw_circle(sx, sy, s_radius, color);

            // Draw a small line to show velocity direction
            if body.velocity.length() > 0.1 {
                let vel_dir = body.velocity.normalize_copy();
                draw_line(
                    sx,
                    sy,
                    sx + vel_dir.x * 20.0,
                    sy - vel_dir.y * 20.0,
                    1.0,
                    WHITE,
                );
            }
        }

        draw_text("Iron Physics Testing Visual", 20.0, 30.0, 30.0, WHITE);
        draw_text(&format!("FPS: {}", get_fps()), 20.0, 60.0, 20.0, GREEN);

        next_frame().await
    }
}
