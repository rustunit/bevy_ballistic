use bevy::prelude::*;

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
