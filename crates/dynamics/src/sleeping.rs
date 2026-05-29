use crate::body::BodyStorage;
use crate::config::WorldConfig;
use crate::island::Island;
use crate::island_manager::IslandManager;
use physics_math::Vec2;
use rayon::prelude::*;


pub fn update_island_sleep(
    island:     &mut Island,
    body_store: &mut BodyStorage,
    config:     &WorldConfig,
    dt:         f32,
) {
    let all_slow = island.bodies.iter().all(|&body_idx| {
        let v = body_store.linear_velocity[body_idx as usize].len_sq();
        let ω = body_store.angular_velocity[body_idx as usize].abs();
        v <= config.linear_sleep_threshold * config.linear_sleep_threshold
            && ω <= config.angular_sleep_threshold
    });
    if all_slow {
        island.sleep_timer += dt;
    } else {
        island.sleep_timer = 0.0;
    }
    if island.sleep_timer >= config.sleep_time_required {
        island.is_sleeping = true;
        for &body_idx in &island.bodies {
            body_store.linear_velocity[body_idx as usize]  = Vec2::zero();
            body_store.angular_velocity[body_idx as usize] = 0.0;
            body_store.is_awake[body_idx as usize]         = false;
        }
    }

}

pub fn wake_body(
    body_idx:   u32,
    body_store: &mut BodyStorage,
    islands:    &mut [Island],
    island_manager: &IslandManager,
) {
    body_store.is_awake[body_idx as usize] = true;
    let island_idx = island_manager.body_to_island[body_idx as usize];
    if island_idx == usize::MAX {
        return;
    }
    let island = &mut islands[island_idx];
    island.is_sleeping = false;
    island.sleep_timer = 0.0;
    for &b in &island.bodies {
        body_store.is_awake[b as usize] = true;
    }

}

pub fn compute_island_sleep_decisions_parallel(
    islands:    &[Island],
    body_store: &BodyStorage,
    config:     &WorldConfig,
) -> Vec<bool> {
    islands
        .par_iter()
        .map(|island| {
            let lin_thresh_sq = config.linear_sleep_threshold
                              * config.linear_sleep_threshold;
            island.bodies.iter().all(|&slot| {
                let s = slot as usize;
                body_store.linear_velocity[s].len_sq()  < lin_thresh_sq
                && body_store.angular_velocity[s].abs() < config.angular_sleep_threshold
            })
        })
        .collect()
    
}