//! Simple launch velocity calculation for game projectiles
//! to hit a specified target based on given gravity. Only
//! depending on `bevy_math` for vector math.
//!
//! Based on this [article](https://www.forrestthewoods.com/blog/solving_ballistic_trajectories/) by Forrest Smith

use bevy_math::Vec3;
use std::{
    cmp::Ordering,
    f32::{EPSILON, consts::FRAC_PI_4},
    f64::{self, consts::PI},
};

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

fn is_zero(d: f64) -> bool {
    const EPS: f64 = 1e-9;
    return d > -EPS && d < EPS;
}

fn get_cubic_root(value: f64) -> f64 {
    if value > 0.0 {
        value.powf(1.0 / 3.0)
    } else if value < 0.0 {
        -(-value.powf(1.0 / 3.0))
    } else {
        0.0
    }
}

// Solve quadratic equation: c0*x^2 + c1*x + c2.
// Returns number of solutions.
fn solve_quadric(c0: f64, c1: f64, c2: f64, s0: &mut f64, s1: &mut f64) -> i32 {
    *s0 = f64::NAN;
    *s1 = f64::NAN;

    let p: f64;
    let q: f64;
    let d: f64;

    /* normal form: x^2 + px + q = 0 */
    p = c1 / (2. * c0);
    q = c2 / c0;

    d = p * p - q;

    if is_zero(d) {
        *s0 = -p;
        return 1;
    } else if d < 0. {
        return 0;
    } else
    /* if (D > 0) */
    {
        let sqrt_d = d.sqrt();

        *s0 = sqrt_d - p;
        *s1 = -sqrt_d - p;
        return 2;
    }
}

// Solve cubic equation: c0*x^3 + c1*x^2 + c2*x + c3.
// Returns number of solutions.
fn solve_cubic(
    c0: f64,
    c1: f64,
    c2: f64,
    c3: f64,
    s0: &mut f64,
    s1: &mut f64,
    s2: &mut f64,
) -> i32 {
    *s0 = f64::NAN;
    *s1 = f64::NAN;
    *s2 = f64::NAN;

    let num: i32;
    let sub: f64;
    let (a, b, c): (f64, f64, f64);
    let (sq_a, p, q): (f64, f64, f64);
    let (cb_p, d): (f64, f64);

    /* normal form: x^3 + Ax^2 + Bx + C = 0 */
    a = c1 / c0;
    b = c2 / c0;
    c = c3 / c0;

    /*  substitute x = y - A/3 to eliminate quadric term:  x^3 +px + q = 0 */
    sq_a = a * a;
    p = 1.0 / 3. * (-1.0 / 3. * sq_a + b);
    q = 1.0 / 2. * (2.0 / 27. * a * sq_a - 1.0 / 3. * a * b + c);

    /* use Cardano's formula */
    cb_p = p * p * p;
    d = q * q + cb_p;

    if is_zero(d) {
        if is_zero(q)
        /* one triple solution */
        {
            *s0 = 0.;
            num = 1;
        } else
        /* one single and one double solution */
        {
            let u = get_cubic_root(-q);
            *s0 = 2. * u;
            *s1 = -u;
            num = 2;
        }
    } else if d < 0.
    /* Casus irreducibilis: three real solutions */
    {
        let phi = 1.0 / 3. * (-q / (-cb_p).sqrt()).acos();
        let t = 2. * (-p).sqrt();

        *s0 = t * (phi).cos();
        *s1 = -t * (phi + PI / 3.).cos();
        *s2 = -t * (phi - PI / 3.).cos();
        num = 3;
    } else
    /* one real solution */
    {
        let sqrt_d = d.sqrt();
        let u = get_cubic_root(sqrt_d - q);
        let v = -get_cubic_root(sqrt_d + q);

        *s0 = u + v;
        num = 1;
    }

    /* resubstitute */
    sub = 1.0 / 3. * a;

    if num > 0 {
        *s0 -= sub;
    }
    if num > 1 {
        *s1 -= sub;
    }
    if num > 2 {
        *s2 -= sub;
    }

    return num;
}

// Solve quartic function: c0*x^4 + c1*x^3 + c2*x^2 + c3*x + c4.
// Returns number of solutions.
fn solve_quartic(
    c0: f64,
    c1: f64,
    c2: f64,
    c3: f64,
    c4: f64,
    s0: &mut f64,
    s1: &mut f64,
    s2: &mut f64,
    s3: &mut f64,
) -> i32 {
    *s0 = f64::NAN;
    *s1 = f64::NAN;
    *s2 = f64::NAN;
    *s3 = f64::NAN;

    let mut coeffs: [f64; 4] = [0.; 4];
    let (z, mut u, mut v, sub): (f64, f64, f64, f64);
    let (a, b, c, d): (f64, f64, f64, f64);
    let (sq_a, p, q, r): (f64, f64, f64, f64);
    let mut num: i32;

    /* normal form: x^4 + Ax^3 + Bx^2 + Cx + D = 0 */
    a = c1 / c0;
    b = c2 / c0;
    c = c3 / c0;
    d = c4 / c0;

    /*  substitute x = y - A/4 to eliminate cubic term: x^4 + px^2 + qx + r = 0 */
    sq_a = a * a;
    p = -3.0 / 8. * sq_a + b;
    q = 1.0 / 8. * sq_a * a - 1.0 / 2. * a * b + c;
    r = -3.0 / 256. * sq_a * sq_a + 1.0 / 16. * sq_a * b - 1.0 / 4. * a * c + d;

    if is_zero(r) {
        /* no absolute term: y(y^3 + py + q) = 0 */

        coeffs[3] = q;
        coeffs[2] = p;
        coeffs[1] = 0.;
        coeffs[0] = 1.;

        num = solve_cubic(coeffs[0], coeffs[1], coeffs[2], coeffs[3], s0, s1, s2);
    } else {
        /* solve the resolvent cubic ... */
        coeffs[3] = 1.0 / 2. * r * p - 1.0 / 8. * q * q;
        coeffs[2] = -r;
        coeffs[1] = -1.0 / 2. * p;
        coeffs[0] = 1.;

        solve_cubic(coeffs[0], coeffs[1], coeffs[2], coeffs[3], s0, s1, s2);

        /* ... and take the one real solution ... */
        z = *s0;

        /* ... to build two quadric equations */
        u = z * z - r;
        v = 2. * z - p;

        if is_zero(u) {
            u = 0.;
        } else if u > 0. {
            u = u.sqrt();
        } else {
            return 0;
        }

        if is_zero(v) {
            v = 0.;
        } else if v > 0. {
            v = v.sqrt();
        } else {
            return 0;
        }

        coeffs[2] = z - u;
        coeffs[1] = if q < 0. { -v } else { v };
        coeffs[0] = 1.;

        num = solve_quadric(coeffs[0], coeffs[1], coeffs[2], s0, s1);

        coeffs[2] = z + u;
        coeffs[1] = if q < 0. { v } else { -v };
        coeffs[0] = 1.;

        if num == 0 {
            num += solve_quadric(coeffs[0], coeffs[1], coeffs[2], s0, s1);
        } else if num == 1 {
            num += solve_quadric(coeffs[0], coeffs[1], coeffs[2], s1, s2);
        } else if num == 2 {
            num += solve_quadric(coeffs[0], coeffs[1], coeffs[2], s2, s3);
        }
    }

    /* resubstitute */
    sub = 1.0 / 4. * a;

    if num > 0 {
        *s0 -= sub;
    }
    if num > 1 {
        *s1 -= sub;
    }
    if num > 2 {
        *s2 -= sub;
    }
    if num > 3 {
        *s3 -= sub;
    }

    return num;
}

pub fn launch_velocity_moving_target(
    proj_pos: Vec3,
    proj_speed: f32,
    target_pos: Vec3,
    target_velocity: Vec3,
    gravity: f32,
) -> Option<(Vec3, Vec3)> {
    // Derivation
    //
    //  For full derivation see: blog.forrestthewoods.com
    //  Here is an abbreviated version.
    //
    //  Four equations, four unknowns (solution.x, solution.y, solution.z, time):
    //
    //  (1) proj_pos.x + solution.x*time = target_pos.x + target_vel.x*time
    //  (2) proj_pos.y + solution.y*time + .5*G*t = target_pos.y + target_vel.y*time
    //  (3) proj_pos.z + solution.z*time = target_pos.z + target_vel.z*time
    //  (4) proj_speed^2 = solution.x^2 + solution.y^2 + solution.z^2
    //
    //  (5) Solve for solution.x and solution.z in equations (1) and (3)
    //  (6) Square solution.x and solution.z from (5)
    //  (7) Solve solution.y^2 by plugging (6) into (4)
    //  (8) Solve solution.y by rearranging (2)
    //  (9) Square (8)
    //  (10) Set (8) = (7). All solution.xyz terms should be gone. Only time remains.
    //  (11) Rearrange 10. It will be of the form a*^4 + b*t^3 + c*t^2 + d*t * e. This is a quartic.
    //  (12) Solve the quartic using SolveQuartic.
    //  (13) If there are no positive, real roots there is no solution.
    //  (14) Each positive, real root is one valid solution
    //  (15) Plug each time value into (1) (2) and (3) to calculate solution.xyz
    //  (16) The end.

    let g = gravity as f64;

    let a = proj_pos.x as f64;
    let b = proj_pos.y as f64;
    let c = proj_pos.z as f64;
    let m = target_pos.x as f64;
    let n = target_pos.y as f64;
    let o = target_pos.z as f64;
    let p = target_velocity.x as f64;
    let q = target_velocity.y as f64;
    let r = target_velocity.z as f64;
    let s = proj_speed as f64;

    let h = m - a;
    let j = o - c;
    let k = n - b;
    let l = -0.5 * g;

    // Quartic Coeffecients
    let c0 = l * l;
    let c1 = -2. * q * l;
    let c2 = q * q - 2. * k * l - s * s + p * p + r * r;
    let c3 = 2. * k * q + 2. * h * p + 2. * j * r;
    let c4 = k * k + h * h + j * j;

    // Solve quartic
    let (mut times, num_times) = {
        let (mut times0, mut times1, mut times2, mut times3): (f64, f64, f64, f64) =
            (0., 0., 0., 0.);
        let num_times = solve_quartic(
            c0,
            c1,
            c2,
            c3,
            c4,
            &mut times0,
            &mut times1,
            &mut times2,
            &mut times3,
        ) as usize;

        ([times0, times1, times2, times3], num_times)
    };

    // Sort so faster collision is found first
    times.sort_by(|a, b| {
        if a < b {
            Ordering::Less
        } else if a > b {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    });

    // Plug quartic solutions into base equations
    // There should never be more than 2 positive, real roots.
    let mut solutions: [Vec3; 2] = [Vec3::ZERO; 2];
    let mut num_solutions = 0;

    for t in times {
        if num_solutions >= 2 {
            break;
        }

        if t <= 0. || t.is_nan() {
            continue;
        }

        solutions[num_solutions].x = ((h + p * t) / t) as f32;
        solutions[num_solutions].y = ((k + q * t - l * t * t) / t) as f32;
        solutions[num_solutions].z = ((j + r * t) / t) as f32;
        num_solutions += 1;
    }

    println!("num solutions: {num_solutions} / times: {num_times}");

    if num_solutions == 0 {
        None
    } else {
        Some((solutions[0], solutions[1]))
    }
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

pub fn launch_velocity_lateral_moving_target(
    proj_pos: Vec3,
    lateral_speed: f32,
    target: Vec3,
    target_velocity: Vec3,
    max_height_offset: f32,
) -> Option<(Vec3, f32)> {
    // Initialize output variables

    // Ground plane terms
    let target_vel_xz = Vec3::new(target_velocity.x, 0., target_velocity.z);
    let mut diff_xz = target - proj_pos;
    diff_xz.y = 0.;

    // Derivation
    //   (1) Base formula: |P + V*t| = S*t
    //   (2) Substitute variables: |diffXZ + targetVelXZ*t| = S*t
    //   (3) Square both sides: Dot(diffXZ,diffXZ) + 2*Dot(diffXZ, targetVelXZ)*t + Dot(targetVelXZ, targetVelXZ)*t^2 = S^2 * t^2
    //   (4) Quadratic: (Dot(targetVelXZ,targetVelXZ) - S^2)t^2 + (2*Dot(diffXZ, targetVelXZ))*t + Dot(diffXZ, diffXZ) = 0
    let c0 = Vec3::dot(target_vel_xz, target_vel_xz) - lateral_speed * lateral_speed;
    let c1 = 2. * Vec3::dot(diff_xz, target_vel_xz);
    let c2 = Vec3::dot(diff_xz, diff_xz);
    let (mut t0, mut t1): (f64, f64) = (0., 0.);
    let n = solve_quadric(c0 as f64, c1 as f64, c2 as f64, &mut t0, &mut t1);

    // pick smallest, positive time
    let valid0 = n > 0 && t0 > 0.;
    let valid1 = n > 1 && t1 > 0.;

    let t: f32;
    if !valid0 && !valid1 {
        return None;
    } else if valid0 && valid1 {
        t = t0.min(t1) as f32;
    } else {
        t = if valid0 { t0 } else { t1 } as f32;
    }

    // Calculate impact point
    let impact_point = target + (target_velocity * t);

    // Calculate fire velocity along XZ plane
    let dir = impact_point - proj_pos;
    let mut fire_velocity = Vec3::new(dir.x, 0., dir.z).normalize() * lateral_speed;

    // Solve system of equations. Hit max_height at t=.5*time. Hit target at t=time.
    //
    // peak = y0 + vertical_speed*halfTime + .5*gravity*halfTime^2
    // end = y0 + vertical_speed*time + .5*gravity*time^s
    // Wolfram Alpha: solve b = a + .5*v*t + .5*g*(.5*t)^2, c = a + vt + .5*g*t^2 for g, v
    let a = proj_pos.y; // initial
    let b = proj_pos.y.max(impact_point.y) + max_height_offset; // peak
    let c = impact_point.y; // final

    let gravity = -4. * (a - 2. * b + c) / (t * t);
    fire_velocity.y = -(3. * a - 4. * b + c) / t;

    Some((fire_velocity, gravity))
}
