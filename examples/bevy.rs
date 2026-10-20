// macOS ld notes the debug binary's >16MB __eh_frame; harmless.
#![allow(linker_messages)]

use avian3d::prelude::*;
use bevy::{
    asset::RenderAssetUsages,
    camera_controller::free_camera::{FreeCamera, FreeCameraPlugin},
    color::palettes::{basic::SILVER, css::WHITE},
    feathers::{
        FeathersPlugins,
        containers::{pane, pane_body, pane_header},
        controls::{FeathersCheckbox, FeathersSlider},
        dark_theme::create_dark_theme,
        display::{caption, label},
        theme::UiTheme,
    },
    prelude::*,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
    ui::Checked,
    ui_widgets::{SliderDragState, SliderPrecision, SliderValue, ValueChange},
};
use bevy_ballistic::{
    ballistic_range, launch_velocity, launch_velocity_lateral,
    launch_velocity_lateral_moving_target, launch_velocity_moving_target,
};
use bevy_sprinkles::prelude::{
    DrawPassMaterial, EmissionShape, EmitterColors, EmitterData, EmitterDrawPass, EmitterEmission,
    EmitterScale, EmitterTime, EmitterVelocities, GradientStop, ParticleGradient, ParticleRange,
    Particles3d, ParticlesAsset, ParticlesDimension, SprinklesPlugin, StandardParticleMaterial,
};
use std::time::Duration;

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
    impact_effect: Handle<ParticlesAsset>,
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
    animate: bool,
    extrapolated_aim: bool,
}

#[derive(Event, Clone)]
struct SpawnParticle(Vec3);

#[derive(Event, Clone)]
struct SpawnProjectile {
    gravity: f32,
    pos: Vec3,
    vel: Vec3,
}

fn main() {
    let mut app = App::new();

    app.add_plugins((DefaultPlugins.set(ImagePlugin::default_nearest()),))
        .add_plugins(FreeCameraPlugin);

    app.add_plugins(PhysicsPlugins::default());
    app.add_plugins(PhysicsDebugPlugin);
    app.add_plugins(SprinklesPlugin);
    app.add_plugins(FeathersPlugins);
    app.insert_resource(UiTheme(create_dark_theme()));

    app.add_systems(Startup, (setup, controls_panel.spawn()));
    app.add_systems(
        Update,
        (
            update,
            controls,
            gismos,
            animate,
            show_range_while_dragging,
            sync_widgets
                .after(controls)
                .run_if(resource_changed::<Controls>),
        ),
    );

    app.add_observer(on_particle);
    app.add_observer(on_spawn);

    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut effects: ResMut<Assets<ParticlesAsset>>,
) {
    let debug_material = materials.add(StandardMaterial {
        base_color_texture: Some(images.add(uv_debug_texture())),
        ..default()
    });

    commands.insert_resource(Shooting {
        timer: Timer::new(Duration::from_millis(200), TimerMode::Repeating),
        material: debug_material.clone(),
        mesh: meshes.add(Sphere::default().mesh().ico(5).unwrap()),
        impact_effect: effects.add(impact_effect()),
    });

    let controls = Controls {
        x: 2.,
        z: 15.,
        vel: 15.,
        lateral_vel: 12.,
        lateral_height: 4.,
        lateral: false,
        show_range: false,
        animate: true,
        extrapolated_aim: true,
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
            shadow_maps_enabled: true,
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
        ColliderConstructor::ConvexHullFromMesh,
        CollisionLayers::new(LayerMask(0b010), LayerMask::ALL),
    ));

    commands.spawn((
        Camera3d::default(),
        FreeCamera::default(),
        Transform::from_xyz(10.0, 10., 5.0).looking_at(Vec3::new(2., 0., 25.), Vec3::Y),
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
    let Ok(shooter) = shooter.single() else {
        return;
    };

    let range = ballistic_range(controls.vel, 9.81, 0.5);
    gizmos.sphere(
        Isometry3d::from_translation(shooter.translation),
        range,
        WHITE,
    );
}

/// The `Controls` field a slider edits.
#[derive(Component, Clone, Copy, Default)]
enum ControlSlider {
    #[default]
    X,
    Z,
    Vel,
    LateralVel,
    LateralHeight,
}

impl ControlSlider {
    fn get(self, c: &Controls) -> f32 {
        match self {
            Self::X => c.x,
            Self::Z => c.z,
            Self::Vel => c.vel,
            Self::LateralVel => c.lateral_vel,
            Self::LateralHeight => c.lateral_height,
        }
    }

    fn get_mut(self, c: &mut Controls) -> &mut f32 {
        match self {
            Self::X => &mut c.x,
            Self::Z => &mut c.z,
            Self::Vel => &mut c.vel,
            Self::LateralVel => &mut c.lateral_vel,
            Self::LateralHeight => &mut c.lateral_height,
        }
    }
}

/// The `Controls` field a checkbox edits.
#[derive(Component, Clone, Copy, Default)]
enum ControlToggle {
    #[default]
    Animate,
    ExtrapolatedAim,
    Lateral,
}

impl ControlToggle {
    fn get(self, c: &Controls) -> bool {
        match self {
            Self::Animate => c.animate,
            Self::ExtrapolatedAim => c.extrapolated_aim,
            Self::Lateral => c.lateral,
        }
    }

    fn get_mut(self, c: &mut Controls) -> &mut bool {
        match self {
            Self::Animate => &mut c.animate,
            Self::ExtrapolatedAim => &mut c.extrapolated_aim,
            Self::Lateral => &mut c.lateral,
        }
    }
}

#[derive(Component, Clone, Copy, Default)]
enum ShowIf {
    #[default]
    Always,
    Animate,
    Lateral,
    NotLateral,
}

impl ShowIf {
    fn visible(self, c: &Controls) -> bool {
        match self {
            Self::Always => true,
            Self::Animate => c.animate,
            Self::Lateral => c.lateral,
            Self::NotLateral => !c.lateral,
        }
    }
}

fn controls_panel() -> impl Scene {
    bsn! {
        @pane()
        Node {
            position_type: PositionType::Absolute,
            left: px(8),
            top: px(8),
            width: px(300),
        }
        Children [
            @pane_header()
            Children [ @label("Controls") ]
            --
            @pane_body()
            Children [
                @slider_row("x", ControlSlider::X, 2., 10., ShowIf::Always)
                --
                @slider_row("z", ControlSlider::Z, 15., 30., ShowIf::Always)
                --
                @slider_row("vel", ControlSlider::Vel, 10., 25., ShowIf::NotLateral)
                --
                @checkbox("animate", ControlToggle::Animate, ShowIf::Always)
                --
                @checkbox("extrapolated aim", ControlToggle::ExtrapolatedAim, ShowIf::Animate)
                --
                @checkbox("lateral", ControlToggle::Lateral, ShowIf::Always)
                --
                @slider_row("lateral vel", ControlSlider::LateralVel, 1., 15., ShowIf::Lateral)
                --
                @slider_row("lateral height", ControlSlider::LateralHeight, 1., 15., ShowIf::Lateral)
            ]
        ]
    }
}

fn slider_row(
    text: &'static str,
    field: ControlSlider,
    min: f32,
    max: f32,
    show_if: ShowIf,
) -> impl Scene {
    bsn! {
        Node {
            align_items: AlignItems::Center,
            column_gap: px(8),
        }
        show_if
        Children [
            @FeathersSlider { @min: min, @max: max }
            SliderPrecision(2)
            field
            on(on_slider)
            --
            @label(text)
            Node { width: px(100) }
        ]
    }
}

fn checkbox(text: &'static str, field: ControlToggle, show_if: ShowIf) -> impl Scene {
    bsn! {
        @FeathersCheckbox { @caption: bsn! { @caption(text) } }
        field
        show_if
        on(on_toggle)
    }
}

fn on_slider(
    change: On<ValueChange<f32>>,
    sliders: Query<&ControlSlider>,
    mut controls: ResMut<Controls>,
) {
    if let Ok(field) = sliders.get(change.source) {
        *field.get_mut(&mut controls) = change.value;
    }
}

fn on_toggle(
    change: On<ValueChange<bool>>,
    toggles: Query<&ControlToggle>,
    mut controls: ResMut<Controls>,
) {
    if let Ok(field) = toggles.get(change.source) {
        *field.get_mut(&mut controls) = change.value;
    }
}

/// Widgets are controlled: `Controls` is the source of truth, also written by `controls()`.
fn sync_widgets(
    mut commands: Commands,
    controls: Res<Controls>,
    sliders: Query<(Entity, &ControlSlider, &SliderValue)>,
    toggles: Query<(Entity, &ControlToggle, Has<Checked>)>,
    mut rows: Query<(&ShowIf, &mut Node)>,
) {
    for (entity, field, value) in &sliders {
        let new_value = field.get(&controls);
        if value.0 != new_value {
            commands.entity(entity).insert(SliderValue(new_value));
        }
    }

    for (entity, field, checked) in &toggles {
        match (field.get(&controls), checked) {
            (true, false) => {
                commands.entity(entity).insert(Checked);
            }
            (false, true) => {
                commands.entity(entity).remove::<Checked>();
            }
            _ => {}
        }
    }

    for (show_if, mut node) in &mut rows {
        let display = if show_if.visible(&controls) {
            Display::Flex
        } else {
            Display::None
        };
        if node.display != display {
            node.display = display;
        }
    }
}

fn show_range_while_dragging(
    sliders: Query<(&ControlSlider, &SliderDragState)>,
    mut controls: ResMut<Controls>,
) {
    let dragging = sliders.iter().any(|(field, drag)| {
        drag.dragging
            && matches!(
                field,
                ControlSlider::X | ControlSlider::Z | ControlSlider::Vel
            )
    });
    if controls.show_range != dragging {
        controls.show_range = dragging;
    }
}

fn controls(
    mut res: ResMut<Controls>,
    mut target: Query<&mut Transform, (With<Target>, Without<Shooter>)>,
) {
    let Ok(mut target) = target.single_mut() else {
        return;
    };

    if res.animate {
        res.z = target.translation.z;
        res.x = target.translation.x;
    } else {
        target.translation.z = res.z;
        target.translation.x = res.x;
    }
}

fn animate(
    controls: Res<Controls>,
    time: Res<Time>,
    mut target: Query<&mut LinearVelocity, (With<Target>, Without<Shooter>)>,
) {
    let direction = (time.elapsed_secs() * 0.5).sin();
    let direction: f32 = if direction > 0. { 1. } else { -1. };

    for mut vel in target.iter_mut() {
        if controls.animate {
            vel.x = 1.5 * direction;
            vel.z = 2. * 1.5 * direction;
        } else {
            *vel = LinearVelocity::ZERO;
        }
    }
}

fn update(
    mut commands: Commands,
    mut shooting: ResMut<Shooting>,
    controls: Res<Controls>,
    time: Res<Time>,
    shooter: Query<&Transform, With<Shooter>>,
    target: Query<(&Transform, &LinearVelocity), With<Target>>,
) {
    shooting.timer.tick(time.delta());

    if shooting.timer.just_finished() {
        let Ok(shooter) = shooter.single() else {
            return;
        };
        let Ok((target, LinearVelocity(target_vel))) = target.single() else {
            return;
        };

        if controls.lateral {
            let launch_vel = if controls.extrapolated_aim {
                launch_velocity_lateral_moving_target(
                    shooter.translation,
                    controls.lateral_vel,
                    target.translation,
                    *target_vel,
                    controls.lateral_height,
                )
            } else {
                launch_velocity_lateral(
                    shooter.translation,
                    controls.lateral_vel,
                    target.translation,
                    controls.lateral_height,
                )
            };
            let Some((vel, gravity)) = launch_vel else {
                warn!("cannot reach target");
                return;
            };

            // info!("launch vel: {launch_vel:?}");
            commands.trigger(SpawnProjectile {
                gravity,
                vel,
                pos: shooter.translation,
            });
        } else {
            let launch_vel = if controls.extrapolated_aim {
                launch_velocity_moving_target(
                    shooter.translation,
                    controls.vel,
                    target.translation,
                    *target_vel,
                    9.81,
                )
            } else {
                launch_velocity(shooter.translation, target.translation, controls.vel, 9.81)
            };

            let Some((vel_low, vel_high)) = launch_vel else {
                warn!("cannot reach target");
                return;
            };

            for vel in [vel_low, vel_high] {
                commands.trigger(SpawnProjectile {
                    gravity: 9.81,
                    vel,
                    pos: shooter.translation,
                });
            }
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

fn on_spawn(trigger: On<SpawnProjectile>, mut commands: Commands, shooting: Res<Shooting>) {
    commands
        .spawn((
            Name::new("projectile"),
            Projectile,
            Mesh3d(shooting.mesh.clone()),
            MeshMaterial3d(shooting.material.clone()),
            Collider::sphere(0.5),
            RigidBody::Dynamic,
            GravityScale(trigger.gravity / 9.81),
            LinearVelocity(trigger.vel),
            Visibility::default(),
            Transform::from_translation(trigger.pos).with_scale(Vec3::splat(0.4)),
            CollisionLayers::new(LayerMask(0b100), LayerMask(0b011)),
            CollisionEventsEnabled,
        ))
        .observe(
            |trigger: On<CollisionStart>, mut cmds: Commands, query: Query<&Transform>| -> Result {
                let t = query.get(trigger.collider1)?;
                cmds.trigger(SpawnParticle(t.translation));
                cmds.entity(trigger.collider1).try_despawn();
                Ok(())
            },
        );
}

fn on_particle(spawn: On<SpawnParticle>, mut commands: Commands, shooting: Res<Shooting>) {
    commands.spawn((
        Transform::from_translation(spawn.event().0),
        Particles3d(shooting.impact_effect.clone()),
    ));
}

fn impact_effect() -> ParticlesAsset {
    ParticlesAsset::new(
        "impact".into(),
        ParticlesDimension::D3,
        default(),
        vec![EmitterData {
            time: EmitterTime {
                lifetime: 0.1,
                one_shot: true,
                explosiveness: 1.,
                ..default()
            },
            draw_pass: EmitterDrawPass {
                material: DrawPassMaterial::Standard(StandardParticleMaterial {
                    unlit: true,
                    ..default()
                }),
                shadow_caster: false,
                ..default()
            },
            emission: EmitterEmission {
                shape: EmissionShape::Sphere { radius: 0.4 },
                particles_amount: 100,
                ..default()
            },
            scale: EmitterScale {
                range: ParticleRange::new(0.02, 0.08),
                ..default()
            },
            colors: EmitterColors {
                color_over_lifetime: ParticleGradient {
                    stops: vec![
                        GradientStop {
                            color: [1., 0., 0., 1.],
                            position: 0.,
                        },
                        GradientStop {
                            color: [1., 1., 0., 1.],
                            position: 1.,
                        },
                    ],
                    ..default()
                },
                ..default()
            },
            velocities: EmitterVelocities {
                initial_direction: Vec3::Y,
                spread: 30.,
                initial_velocity: ParticleRange::new(0., 10.),
                ..default()
            },
            ..default()
        }],
        vec![],
        true,
        default(),
    )
}
