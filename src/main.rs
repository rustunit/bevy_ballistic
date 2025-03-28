mod ballistic;

use avian3d::prelude::*;
use ballistic::launch_velocity;
use bevy::{
    color::palettes::basic::SILVER,
    prelude::*,
    render::{
        render_asset::RenderAssetUsages,
        render_resource::{Extent3d, TextureDimension, TextureFormat},
    },
};
use bevy_flycam::prelude::*;
use std::time::Duration;

#[derive(Component)]
struct Shooter;

#[derive(Component)]
struct Target;

#[derive(Resource)]
struct Shooting {
    timer: Timer,
    material: Handle<StandardMaterial>,
    mesh: Handle<Mesh>,
}

fn main() {
    let mut app = App::new();

    app.add_plugins((DefaultPlugins.set(ImagePlugin::default_nearest()),))
        .add_plugins(NoCameraPlayerPlugin);

    app.add_plugins(bevy_inspector_egui::quick::WorldInspectorPlugin::new());
    app.add_plugins(PhysicsDebugPlugin::default());

    app.add_systems(Startup, setup);
    app.add_systems(Update, update);

    app.add_plugins(PhysicsPlugins::default());

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
        timer: Timer::new(Duration::from_secs(1), TimerMode::Repeating),
        material: debug_material.clone(),
        mesh: meshes.add(Sphere::default().mesh().ico(5).unwrap()),
    });

    commands.spawn((
        Shooter,
        Mesh3d(meshes.add(Cuboid::default())),
        MeshMaterial3d(debug_material.clone()),
        Transform::from_xyz(-5.0, 0.5, 20.0),
    ));

    commands.spawn((
        Target,
        Mesh3d(meshes.add(Cuboid::default())),
        MeshMaterial3d(debug_material.clone()),
        Transform::from_xyz(5.0, 0.5, 20.0),
    ));

    commands.spawn((
        PointLight {
            shadows_enabled: true,
            intensity: 10_000_000.,
            range: 100.0,
            shadow_depth_bias: 0.2,
            ..default()
        },
        Transform::from_xyz(8.0, 16.0, 8.0),
    ));

    // ground plane
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(50.0, 50.0).subdivisions(10))),
        MeshMaterial3d(materials.add(Color::from(SILVER))),
    ));

    commands.spawn((
        Camera3d::default(),
        FlyCam,
        Transform::from_xyz(0.0, 4., 0.0).looking_at(Vec3::new(0., 1., 10.), Vec3::Y),
    ));
}

fn update(
    mut commands: Commands,
    mut shooting: ResMut<Shooting>,
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

        let Some((vel_low, vel_high)) =
            launch_velocity(shooter.translation, target.translation, 20., 9.81)
        else {
            warn!("cannot reach target");
            return;
        };

        commands.spawn((
            Mesh3d(shooting.mesh.clone()),
            MeshMaterial3d(shooting.material.clone()),
            Collider::sphere(0.5),
            RigidBody::Dynamic,
            LinearVelocity(vel_low),
            Visibility::default(),
            Transform::from_translation(shooter.translation).with_scale(Vec3::splat(0.4)),
        ));

        commands.spawn((
            Mesh3d(shooting.mesh.clone()),
            MeshMaterial3d(shooting.material.clone()),
            Collider::sphere(0.5),
            RigidBody::Dynamic,
            LinearVelocity(vel_high),
            Visibility::default(),
            Transform::from_translation(shooter.translation).with_scale(Vec3::splat(0.4)),
        ));
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
