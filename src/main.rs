mod math;
mod physics;
use math::vec2::Vector2;
use physics::rigid_body::ForceType;
use physics::rigid_body::RigidBody;

fn main() {
    let mut rb = RigidBody::new(Vector2::new(0.0, 0.0), Vector2::new(0.0, 0.0), 1.0);

    let dt = 0.1;

    for step in 0..10 {
        rb.apply_force(Vector2::new(10.0, 0.0), ForceType::External);

        rb.update(dt);

        println!(
            "Step {} -> Pos: ({:.2}, {:.2}) Vel: ({:.2}, {:.2})",
            step, rb.position.x, rb.position.y, rb.velocity.x, rb.velocity.y
        );
    }
}
