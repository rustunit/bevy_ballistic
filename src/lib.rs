//! Simple launch velocity calculation for game projectiles
//! to hit a specified target based on given gravity. Only
//! depending on `bevy_math` for vector math.
//!
//! Based on this [article](https://www.forrestthewoods.com/blog/solving_ballistic_trajectories/) by Forrest Smith

use bevy_math::Vec3;
use std::f32::{EPSILON, consts::FRAC_PI_4};

/// Caluculate range of a projectile being fired from `initial_height`
/// with `speed` and only affected by `gravity`.
pub fn ballistic_range(speed: f32, gravity: f32, initial_height: f32) -> f32 {
    // Derivation
    //   (1) x = speed * time * cos O
    //   (2) y = initial_height + (speed * time * sin O) - (.5 * gravity*time*time)
    //   (3) via quadratic: t = (speed*sin O)/gravity + sqrt(speed*speed*sin O + 2*gravity*initial_height)/gravity    [ignore smaller root]
    //   (4) solution: range = x = (speed*cos O)/gravity * sqrt(speed*speed*sin O + 2*gravity*initial_height)    [plug t back into x=speed*time*cos O]
    let angle = FRAC_PI_4; // no air resistence, so 45 degrees provides maximum range
    let cos = angle.cos();
    let sin = angle.sin();

    (speed * cos / gravity)
        * (speed * sin + (speed * speed * sin * sin + 2. * gravity * initial_height).sqrt())
}

/// Calculates a `low` and `high` angle for projectile to start at `start_pos` to hit `target_pos` with a given `initial_velocity`.
///
/// # Returns
/// If solvable returns two vectors `low` and `high`
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

    let pitch = move |pitch: f32| {
        let dir_x = pitch.cos() * yaw.cos();
        let dir_y = pitch.sin();
        let dir_z = pitch.cos() * yaw.sin();

        Vec3::new(
            initial_velocity * dir_x,
            initial_velocity * dir_y,
            initial_velocity * dir_z,
        )
    };

    Some((pitch(pitch_low), pitch(pitch_high)))
}

/// Solve for arc with a fixed lateral speed. Vertical speed and gravity varies.
/// This leads to a visually pleasing arc.
///
/// Notes: its important to apply the returned gravity to lead to hit target and an arc with `max_height`.
pub fn launch_velocity_lateral(
    proj_pos: Vec3,
    lateral_speed: f32,
    target_pos: Vec3,
    max_height: f32,
) -> Option<(Vec3, f32)> {
    let diff = target_pos - proj_pos;
    let diff_xz = Vec3::new(diff.x, 0., diff.z);

    let lateral_distance = diff_xz.length();
    if lateral_distance <= EPSILON {
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
