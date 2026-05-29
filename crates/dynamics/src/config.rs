pub struct WorldConfig {
    pub velocity_iterations: u32,
    pub position_iterations: u32,

    pub warm_starting_factor: f32,

    pub linear_slop:            f32,
    pub max_linear_correction:  f32,
    pub baumgarte_factor:       f32, 
    pub restitution_threshold:  f32,

    pub allow_sleeping:           bool, 
    pub linear_sleep_threshold:   f32,   
    pub angular_sleep_threshold:  f32,   
    pub sleep_time_required:      f32,  

    pub aabb_extension:   f32,
    pub aabb_multiplier:  f32,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            velocity_iterations: 8,
            position_iterations: 3,
            warm_starting_factor: 0.9,
            linear_slop: 0.005,
            max_linear_correction: 0.2,
            baumgarte_factor: 0.2,
            restitution_threshold: 1.0,
            allow_sleeping: true,
            linear_sleep_threshold: 0.01,
            angular_sleep_threshold: 0.01,
            sleep_time_required: 0.5,
            aabb_extension: 0.1,
            aabb_multiplier: 4.0,
        }
    }
}
