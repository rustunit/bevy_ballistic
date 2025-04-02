use avian3d::prelude::*;
use bevy::{
    color::palettes::{basic::SILVER, css::WHITE},
    prelude::*,
    render::{
        render_asset::RenderAssetUsages,
        render_resource::{Extent3d, TextureDimension, TextureFormat},
    },
};
use bevy_ballistic::{ballistic_range, launch_velocity, launch_velocity_lateral};
use bevy_egui::{EguiContexts, EguiPlugin, egui};
use bevy_firework::{
    bevy_utilitarian::prelude::{RandF32, RandValue, RandVec3},
    core::{BlendMode, ParticleSpawner},
    curve::{FireworkCurve, FireworkGradient},
    emission_shape::EmissionShape,
    plugin::ParticleSystemPlugin,
};
use bevy_flycam::prelude::*;
use std::{f32::consts::PI, time::Duration};

#[derive(Component)]
struct Shooter;

#[derive(Component)]
struct Target;

#[derive(Component)]
struct Projectile;

#[derive(Resource)]
struct Shooting {
    timer: Timer,
    material: Handle<StandardMaterial>,
    mesh: Handle<Mesh>,
}

#[derive(Resource, Clone)]
struct Controls {
    x: f32,
    z: f32,
    vel: f32,
    lateral_vel: f32,
    lateral_height: f32,
    lateral: bool,
    show_range: bool,
}

#[derive(Event, Clone)]
struct SpawnParticle(Vec3);

fn main() {
    let mut app = App::new();

    app.add_plugins((DefaultPlugins.set(ImagePlugin::default_nearest()),))
        .add_plugins(NoCameraPlayerPlugin);

    app.add_plugins(PhysicsPlugins::default());
    app.add_plugins(PhysicsDebugPlugin::default());
    app.add_plugins(ParticleSystemPlugin::default());

    if !app.is_plugin_added::<EguiPlugin>() {
        app.add_plugins(EguiPlugin);
    }

    app.add_systems(Startup, setup);
    app.add_systems(Update, (update, ui, controls, gismos));
    app.add_systems(PostUpdate, collisions);
    app.add_observer(on_particle);

    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let debug_material = materials.add(StandardMaterial {
        base_color_texture: Some(images.add(uv_debug_texture())),
        ..default()
    });

    commands.insert_resource(Shooting {
        timer: Timer::new(Duration::from_millis(200), TimerMode::Repeating),
        material: debug_material.clone(),
        mesh: meshes.add(Sphere::default().mesh().ico(5).unwrap()),
    });

    let controls = Controls {
        x: 5.,
        z: 20.,
        vel: 15.,
        lateral_vel: 12.,
        lateral_height: 4.,
        lateral: false,
        show_range: false,
    };

    commands.insert_resource(controls.clone());

    commands.spawn((
        Shooter,
        Mesh3d(meshes.add(Cuboid::default())),
        MeshMaterial3d(debug_material.clone()),
        Transform::from_xyz(-5.0, 0.6, 20.0),
    ));

    commands.spawn((
        Name::new("target"),
        Target,
        Mesh3d(meshes.add(Cuboid::default())),
        MeshMaterial3d(debug_material.clone()),
        Transform::from_xyz(controls.x, 0.6, controls.z),
        RigidBody::Kinematic,
        Collider::cuboid(1., 1., 1.),
        CollisionLayers::new(LayerMask(0b001), LayerMask::ALL),
    ));

    commands.spawn((
        PointLight {
            shadows_enabled: true,
            intensity: 10_000_000.,
            range: 100.0,
            shadow_depth_bias: 0.2,
            ..default()
        },
        Transform::from_xyz(8.0, 26.0, 8.0),
    ));

    // ground plane
    commands.spawn((
        Name::new("ground"),
        Mesh3d(
            meshes.add(
                Plane3d::default()
                    .mesh()
                    .size(150.0, 150.0)
                    .subdivisions(10),
            ),
        ),
        MeshMaterial3d(materials.add(Color::from(SILVER))),
        RigidBody::Static,
        Collider::half_space(Vec3::Y),
        CollisionLayers::new(LayerMask(0b010), LayerMask::ALL),
    ));

    commands.spawn((
        Camera3d::default(),
        FlyCam,
        Transform::from_xyz(0.0, 4., 0.0).looking_at(Vec3::new(0., 1., 10.), Vec3::Y),
    ));
}

fn gismos(
    mut gizmos: Gizmos,
    controls: Res<Controls>,
    shooter: Query<&Transform, (With<Shooter>, Without<Target>)>,
) {
    if !controls.show_range {
        return;
    }
    let Ok(shooter) = shooter.get_single() else {
        return;
    };

    let range = ballistic_range(controls.vel, 9.81, 0.5);
    gizmos.sphere(
        Isometry3d::from_translation(shooter.translation),
        range,
        WHITE,
    );
}

fn ui(mut contexts: EguiContexts, mut res: ResMut<Controls>) {
    res.show_range = false;
    egui::Window::new("Controls").show(contexts.ctx_mut(), |ui| {
        let dragged_x = ui
            .add(egui::Slider::new(&mut res.x, 2.0..=10.0).text("x"))
            .dragged();
        let dragged_z = ui
            .add(egui::Slider::new(&mut res.z, 15.0..=30.0).text("z"))
            .dragged();
        let dragged_vel = ui
            .add(egui::Slider::new(&mut res.vel, 10.0..=25.0).text("vel"))
            .dragged();

        if dragged_vel || dragged_x || dragged_z {
            res.show_range = true;
        }

        ui.checkbox(&mut res.lateral, "lateral");

        if res.lateral {
            ui.add(egui::Slider::new(&mut res.lateral_vel, 1.0..=15.0).text("lateral vel"));
            ui.add(egui::Slider::new(&mut res.lateral_height, 1.0..=15.0).text("lateral height"));
        }
    });
}

fn controls(
    res: Res<Controls>,
    mut target: Query<&mut Transform, (With<Target>, Without<Shooter>)>,
) {
    let Ok(mut target) = target.get_single_mut() else {
        return;
    };

    target.translation.z = res.z;
    target.translation.x = res.x;
}

fn update(
    mut commands: Commands,
    mut shooting: ResMut<Shooting>,
    controls: Res<Controls>,
    time: Res<Time>,
    shooter: Query<&Transform, (With<Shooter>, Without<Target>)>,
    target: Query<&Transform, (With<Target>, Without<Shooter>)>,
) {
    shooting.timer.tick(time.delta());

    if shooting.timer.just_finished() {
        let Ok(shooter) = shooter.get_single() else {
            return;
        };
        let Ok(target) = target.get_single() else {
            return;
        };

        if controls.lateral {
            let Some((vel, gravity)) = launch_velocity_lateral(
                shooter.translation,
                controls.lateral_vel,
                target.translation,
                controls.lateral_height,
            ) else {
                warn!("cannot reach target");
                return;
            };

            commands.spawn((
                Name::new("projectile"),
                Projectile,
                Mesh3d(shooting.mesh.clone()),
                MeshMaterial3d(shooting.material.clone()),
                Collider::sphere(0.5),
                GravityScale(gravity / 9.81),
                RigidBody::Dynamic,
                LinearVelocity(vel),
                Visibility::default(),
                Transform::from_translation(shooter.translation).with_scale(Vec3::splat(0.4)),
                CollisionLayers::new(LayerMask(0b100), LayerMask(0b011)),
            ));
        } else {
            let Some((vel_low, vel_high)) =
                launch_velocity(shooter.translation, target.translation, controls.vel, 9.81)
            else {
                warn!("cannot reach target");
                return;
            };

            for vel in [vel_low, vel_high] {
                commands.spawn((
                    Name::new("projectile"),
                    Projectile,
                    Mesh3d(shooting.mesh.clone()),
                    MeshMaterial3d(shooting.material.clone()),
                    Collider::sphere(0.5),
                    RigidBody::Dynamic,
                    LinearVelocity(vel),
                    Visibility::default(),
                    Transform::from_translation(shooter.translation).with_scale(Vec3::splat(0.4)),
                    CollisionLayers::new(LayerMask(0b100), LayerMask(0b011)),
                ));
            }
        }
    }
}

fn collisions(
    mut commands: Commands,
    mut collision_event_reader: EventReader<CollisionStarted>,
    projectile: Query<(&Projectile, &Transform)>,
) {
    for CollisionStarted(e1, e2) in collision_event_reader.read() {
        let mut projectiles = Vec::new();

        if projectile.contains(*e1) {
            projectiles.push(*e1);
        }
        if projectile.contains(*e2) {
            projectiles.push(*e2);
        }

        for e in projectiles {
            let Ok((_, t)) = projectile.get(e) else {
                continue;
            };

            commands.trigger(SpawnParticle(t.translation));

            commands.entity(e).despawn();
        }
    }
}

/// Creates a colorful test pattern
fn uv_debug_texture() -> Image {
    const TEXTURE_SIZE: usize = 8;

    let mut palette: [u8; 32] = [
        255, 102, 159, 255, 255, 159, 102, 255, 236, 255, 102, 255, 121, 255, 102, 255, 102, 255,
        198, 255, 102, 198, 255, 255, 121, 102, 255, 255, 236, 102, 255, 255,
    ];

    let mut texture_data = [0; TEXTURE_SIZE * TEXTURE_SIZE * 4];
    for y in 0..TEXTURE_SIZE {
        let offset = TEXTURE_SIZE * y * 4;
        texture_data[offset..(offset + TEXTURE_SIZE * 4)].copy_from_slice(&palette);
        palette.rotate_right(4);
    }

    Image::new_fill(
        Extent3d {
            width: TEXTURE_SIZE as u32,
            height: TEXTURE_SIZE as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        &texture_data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    )
}

fn on_particle(trigger: Trigger<SpawnParticle>, mut commands: Commands) {
    commands.spawn((
        Transform::from_translation(trigger.event().0),
        ParticleSpawner {
            one_shot: true,
            rate: 100.0,
            emission_shape: EmissionShape::Sphere(0.4),
            lifetime: RandF32::constant(0.1),
            initial_velocity: RandVec3 {
                magnitude: RandF32 { min: 0., max: 10. },
                direction: Vec3::Y,
                spread: 30. / 180. * PI,
            },
            initial_scale: RandF32 {
                min: 0.02,
                max: 0.08,
            },
            scale_curve: FireworkCurve::constant(1.),
            color: FireworkGradient::even_samples(vec![
                LinearRgba::new(1., 0., 0., 1.),
                LinearRgba::new(1., 1., 0., 1.),
            ]),
            blend_mode: BlendMode::Blend,
            linear_drag: 0.1,
            pbr: false,
            ..default()
        },
    ));
}
