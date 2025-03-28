use bevy::prelude::*;
use std::f32;

pub fn launch_velocity(
    start_pos: Vec3,
    target_pos: Vec3,
    initial_velocity: f32,
    gravity: f32,
) -> Option<(Vec3, Vec3)> {
    // Calculate displacement
    let delta = target_pos - start_pos;

    // Calculate horizontal distance
    let horizontal_dist = (delta.x * delta.x + delta.z * delta.z).sqrt();

    // Calculate terms for the quadratic formula
    let v_squared = initial_velocity * initial_velocity;
    let discriminant = v_squared * v_squared
        - gravity * (gravity * horizontal_dist * horizontal_dist + 2.0 * delta.y * v_squared);

    if discriminant < 0.0 {
        return None;
    }

    // Calculate pitch angles
    let term1 = v_squared / (gravity * horizontal_dist);
    let term2 = discriminant.sqrt() / (gravity * horizontal_dist);

    let pitch_high = (term1 + term2).atan();
    let pitch_low = (term1 - term2).atan();

    // Calculate yaw angle
    let yaw = delta.z.atan2(delta.x);

    let pitch = pitch_low;

    let dir_x = pitch.cos() * yaw.cos();
    let dir_y = pitch.sin();
    let dir_z = pitch.cos() * yaw.sin();

    let result_low = Vec3::new(
        initial_velocity * dir_x,
        initial_velocity * dir_y,
        initial_velocity * dir_z,
    );

    let pitch = pitch_high;

    let dir_x = pitch.cos() * yaw.cos();
    let dir_y = pitch.sin();
    let dir_z = pitch.cos() * yaw.sin();

    let result_high = Vec3::new(
        initial_velocity * dir_x,
        initial_velocity * dir_y,
        initial_velocity * dir_z,
    );

    Some((result_low, result_high))
}

// see https://www.forrestthewoods.com/blog/solving_ballistic_trajectories/
pub fn launch_velocity_lateral(
    proj_pos: Vec3,
    lateral_speed: f32,
    target_pos: Vec3,
    max_height: f32,
) -> Option<(Vec3, f32)> {
    let diff = target_pos - proj_pos;
    let diff_xz = Vec3::new(diff.x, 0., diff.z);

    let lateral_distance = diff_xz.length();
    if lateral_distance <= 0.001 {
        return None;
    }

    let time = lateral_distance / lateral_speed;

    let mut fire_velocity = diff_xz.normalize() * lateral_speed;

    let a = proj_pos.y; // initial
    let b = max_height; // peak
    let c = target_pos.y; // final

    let gravity = -4. * (a - 2. * b + c) / (time * time);
    fire_velocity.y = -(3. * a - 4. * b + c) / time;

    Some((fire_velocity, gravity))
}
