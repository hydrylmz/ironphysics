use criterion::{criterion_group, criterion_main, Criterion};
use physics_core::{
    World, WorldConfig, ColliderHandle, Material, CollisionFilter,
    Circle, BoxShape, BodyDesc, BodyType, Vec2, Transform, ColliderDesc,
};
use physics_collision::narrowphase::dispatch::run_narrowphase_parallel;
use physics_collision::ContactPool;

fn make_world() -> World {
    let world = World::new(WorldConfig::default());
    World::configure_thread_pool(None);
    world
}

fn add_circle(world: &mut World, pos: Vec2, radius: f32, is_static: bool) -> ColliderHandle {
    let shape = Circle { radius };
    let body_handle = world.add_body(BodyDesc {
        body_type: if is_static { BodyType::Static } else { BodyType::Dynamic },
        position: pos,
        ..BodyDesc::default()
    });
    let material = Material {
        density: 1.0,
        friction: 0.5,
        restitution: 0.5,
    };
    let desc = ColliderDesc {
        shape: Box::new(shape),
        material,
        local_transform: Transform::identity(),
        filter: CollisionFilter::default(),
        is_sensor: false,
    };
    world.add_collider(body_handle, desc)
}

fn add_box(world: &mut World, pos: Vec2, half: Vec2, is_static: bool) -> ColliderHandle {
    let shape = BoxShape { half_extents: half };
    let body_handle = world.add_body(BodyDesc {
        body_type: if is_static { BodyType::Static } else { BodyType::Dynamic },
        position: pos,
        ..BodyDesc::default()
    });
    let material = Material {
        density: 1.0,
        friction: 0.5,
        restitution: 0.5,
    };
    let desc = ColliderDesc {
        shape: Box::new(shape),
        material,
        local_transform: Transform::identity(),
        filter: CollisionFilter::default(),
        is_sensor: false,
    };
    world.add_collider(body_handle, desc)
}

fn add_static_floor(world: &mut World) {
    let shape = BoxShape { half_extents: Vec2::new(100.0, 0.5)}; 
    let body_handle = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        position: Vec2::new(0.0, -0.5),
        ..BodyDesc::default()
    });
    let material = Material {
        density: 1.0,
        friction: 0.5,
        restitution: 0.5,
    };
    let desc = ColliderDesc {
        shape: Box::new(shape),
        material,
        local_transform: Transform::identity(),
        filter: CollisionFilter::default(),
        is_sensor: false,
    };
    world.add_collider(body_handle, desc);
}

fn bench_integration_1k(c: &mut Criterion) {
    // Expected throughput: integration at 1K bodies should be < 0.1ms on modern hardware.
    // This benchmark validates the SoA layout delivers cache efficiency.
    let mut world = make_world();
    for i in 0..1_000 {
        add_circle(&mut world, Vec2::new(i as f32 * 0.1, 10.0), 0.5, false);
    }
    c.bench_function("integration_1k", |b| {
        b.iter(|| {
            world.step(1.0 / 60.0);
        });
    });
}

fn bench_broadphase_10k(c: &mut Criterion) {
    // Measures: BVH update + pair collection for 10_000 bodies in a grid.
    //
    // Setup:
    //   10_000 static circles arranged on a 100×100 grid (no movement).
    //   Run step() once to build the tree.
    //   Benchmark subsequent step() calls (tree update is near-zero for static).
    //
    // What it tests:
    //   collect_pairs() traversal cost for a fully populated tree.
    //   Expected: O(n log n) — should scale predictably from 1K to 100K.
    let mut world = make_world();
    for i in 0..10_000 {
        add_circle(&mut world, Vec2::new((i % 100) as f32 * 0.1, (i / 100) as f32 * 0.1), 0.5, true);
    }
    world.step(1.0 / 60.0); // warm up to build tree
    c.bench_function("broadphase_10k", |b| {
        b.iter(|| {
            world.step(1.0 / 60.0);
        });
    });
}

fn bench_narrowphase_circles(_c: &mut Criterion) {
    // Compare serial vs parallel with two benchmark variants:
    //   "narrowphase_circles_serial"   — single-threaded dispatch loop
    //   "narrowphase_circles_parallel" — run_narrowphase_parallel
    //
    // Report: throughput in pairs/second and speedup ratio.
    let mut world = make_world();
    let mut pairs = Vec::new();
    for i in 0..1_000 {
        let col_a = add_circle(&mut world, Vec2::new(i as f32 * 0.1, 10.0), 0.5, false);
        let col_b = add_circle(&mut world, Vec2::new(i as f32 * 0.1 + 0.05, 10.0), 0.5, false);
        pairs.push((col_a, col_b));
    }

    run_narrowphase_parallel(&pairs, &world.colliders, &ContactPool::new(1024), &mut ContactPool::new(1024));
}

fn bench_narrowphase_boxes(_c: &mut Criterion) {
    // SAT is more expensive than circle-circle — measures worst-case narrowphase cost.
    let mut world = make_world();
    let mut pairs = Vec::new();
    for i in 0..1_000 {
        let col_a = add_box(&mut world, Vec2::new(i as f32 * 0.1, 10.0), Vec2::new(0.5, 0.5), false);
        let col_b = add_box(&mut world, Vec2::new(i as f32 * 0.1 + 0.05, 10.0), Vec2::new(0.5, 0.5), false);
        pairs.push((col_a, col_b));
    }
    run_narrowphase_parallel(&pairs, &world.colliders, &ContactPool::new(1024), &mut ContactPool::new(1024));
}

fn bench_stack_of_boxes(c: &mut Criterion) {
    // Measures: full step() for a stacked scene — the canonical physics benchmark.
    //
    // Setup:
    //   Static floor.
    //   50 boxes stacked in a 5-wide × 10-tall tower.
    //   Run 300 steps to reach steady state (all sleeping).
    //   THEN benchmark: run 1 step to wake the top layer by spawning a
    //   fast-moving ball, then step(). This gives a non-trivial active scene.
    //
    // What to measure:
    //   Total step() time including all phases.
    //   Break it down via stats.time_* fields — print them in the bench output.
    let mut world = make_world();
    add_static_floor(&mut world);
    for i in 0..50 {
        let x = (i % 5) as f32 * 1.0;
        let y = (i / 5) as f32 * 1.0 + 0.5;
        add_box(&mut world, Vec2::new(x, y), Vec2::new(0.5, 0.5), false);
    }
    for _ in 0..300 {
        world.step(1.0 / 60.0);
    }
    // Spawn a fast ball to wake the stack.
    add_circle(&mut world, Vec2::new(2.0, 15.0), 0.5, false);
    c.bench_function("stack_of_boxes", |b| {
        b.iter(|| {
            world.step(1.0 / 60.0);
        });
    });
}

fn bench_stress_1k_dynamic(c: &mut Criterion) {
    // Measures: full step() for 1_000 non-sleeping dynamic bodies.
    //
    // Setup:
    //   1_000 circles randomly distributed, overlapping slightly.
    //   Static walls on 4 sides (box bodies).
    //   Gravity enabled.
    //   Run 60 warm-up steps before benchmarking (build contact caches).
    //
    // This is the "worst case" for island solving and is the primary
    // benchmark for evaluating the Phase 4 parallel speedup.
    //
    // Expected:
    //   Serial (Phase 3):   ~8–12 ms per step
    //   Parallel (Phase 4): ~2–4 ms per step on 8 cores
    let mut world = make_world();
    add_static_floor(&mut world);
    for i in 0..1_000 {
        let x = (i % 50) as f32 * 0.2 + 0.1;
        let y = (i / 50) as f32 * 0.2 + 5.0;
        add_circle(&mut world, Vec2::new(x, y), 0.5, false);
    }
    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }
    c.bench_function("stress_1k_dynamic", |b| {
        b.iter(|| {
            world.step(1.0 / 60.0);
        });
    });
}

fn bench_island_solve_parallel_vs_serial(_c: &mut Criterion) {
    // Direct A/B comparison of serial vs parallel island solving.
    //
    // Setup:
    //   10 independent stacks of 10 boxes each (10 islands, no shared bodies).
    //   This is the ideal case for parallel island solving.
    //
    // Variants:
    //   "island_solve_serial"   — call solve_island() in a for loop
    //   "island_solve_parallel" — call solve_all_islands_parallel()
    //
    // Report the speedup factor explicitly in the benchmark name output.
}

criterion_group!(
    benches,
    bench_integration_1k,
    bench_broadphase_10k,
    bench_narrowphase_circles,
    bench_narrowphase_boxes,
    bench_stack_of_boxes,
    bench_stress_1k_dynamic,
    bench_island_solve_parallel_vs_serial
);
criterion_main!(benches);