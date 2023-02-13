mod walls;
use libm::atan2f;
use walls::*;
use std::{time::Duration};

use bevy::{prelude::*, render::camera::RenderTarget};
use bevy_rapier2d::{prelude::*, na::{Vector2, vector}};
use iyes_loopless::prelude::*;
use bevy_prototype_lyon::{prelude::{*, FillMode}, entity::ShapeBundle};

const WALL_COLOR: Color = Color::rgb(0.8, 0.8, 0.8);
const BALL_COLOR: Color = Color::rgb(1.0, 0.5, 0.5);
const BACKGROUND_COLOR: Color = Color::rgb(0.9, 0.9, 0.9);
const WALL_THICKNESS: f32 = 10.0;
// x coordinates
const LEFT_WALL: f32 = -450.;
const RIGHT_WALL: f32 = 450.;
// y coordinates
const BOTTOM_WALL: f32 = -300.;
const TOP_WALL: f32 = 300.;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(40.0))
        .add_plugin(RapierDebugRenderPlugin::default())
        // .add_plugin(LogDiagnosticsPlugin::default())

        .add_plugin(ShapePlugin)
        // .add_plugin(FrameTimeDiagnosticsPlugin::default())
        .insert_resource(ClearColor(BACKGROUND_COLOR))
        .insert_resource(Msaa { samples: 4})
        .add_startup_system(setup_graphics)
        .add_startup_system(setup_physics)
        .add_system(weapon_system)
        .add_system(camera_system)
        .add_system(rocket_system)
        .add_fixed_timestep(Duration::from_millis(17), "physics_timestep")
        .add_fixed_timestep_system("physics_timestep", 0, control_player)
        .add_fixed_timestep_system("physics_timestep", 0, jump_reset)
        // .add_fixed_timestep_system("physics_timestep", 0, rocket_system)

        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(( Camera2dBundle::default(), MainCamera ));
}

fn setup_physics(
    mut commands: Commands,
) {
    commands.spawn(PlayerBundle::new());

    commands.spawn(WeaponBundle::new());

    commands.spawn(WallBundle::new(WallLocation::Left));
    commands.spawn(WallBundle::new(WallLocation::Right));
    commands.spawn(WallBundle::new(WallLocation::Bottom));
    commands.spawn(WallBundle::new(WallLocation::Top));

}

fn jump_reset(
    mut query: Query<(Entity, &mut Jumper), With<Player>>,
    mut collision_events: EventReader<CollisionEvent>,
) {
    for e in collision_events.iter() {
        for (entity, mut jumper) in query.iter_mut() {
            jump_check(entity, &mut jumper, e);
        }
    }

}

fn jump_check(entity: Entity, jumper: &mut Jumper, event: &CollisionEvent) {
    if let CollisionEvent::Started(h1, h2, _) = event {
        if *h1 == entity || *h2 == entity {
            jumper.is_jumping = false;

        }
    }
}

fn control_player(
    keys: Res<Input<KeyCode>>,
    mut query: Query<(&mut Jumper, &mut Velocity), With<Player>>,
) {
    let   (mut jumper, mut velocity) = query.single_mut();
    if keys.just_released(KeyCode::Space) && !jumper.is_jumping {
        velocity.linvel.y += jumper.jump_impulse * jumper.jump_multiplier;
        jumper.is_jumping = true;
        jumper.jump_multiplier = 1.;
    }

    if keys.pressed(KeyCode::A) {

        velocity.angvel += jumper.roll_impulse;

        if jumper.is_jumping && velocity.linvel.x > -100.{
            velocity.linvel.x -= 3.;
        }

    }

    if keys.pressed(KeyCode::D) {

        velocity.angvel -= jumper.roll_impulse;

        if jumper.is_jumping && velocity.linvel.x < 100. {
            velocity.linvel.x += 3.;
        }
    }    if keys.pressed(KeyCode::Space) {

        velocity.angvel = 0.0;
        if velocity.linvel.x.abs() < 2. {
            jumper.jump_multiplier = 1.
        } else if jumper.jump_multiplier <= 1.6 {
            jumper.jump_multiplier += 0.01;
        }
    }

}

fn camera_system(
    mut camq: Query<( &mut Transform, With<Camera2d> ), Without<Player>>,
    // playerq: Query<( &Transform, With<Player> ), Without<Camera2d>>,
    playerq: Query<( &Transform, With<Player> ), Without<Camera2d>>,

) {

    let mut cam = camq.get_single_mut().unwrap();
    let transform = playerq.get_single().unwrap();
    cam.0.translation.x = transform.0.translation.x;
    cam.0.translation.y = transform.0.translation.y + 200.;
}

fn weapon_system(

    mut weapq: Query<( &mut Transform, With<Weapon>), Without<Player>>,
    playerq: Query<( &Transform, With<Player>), Without<Weapon>>,
    // need to get window dimensions
    wnds: Res<Windows>,
    // query to get camera transform
    q_camera: Query<(&Camera, &GlobalTransform), With<MainCamera>>,

    buttons: Res<Input<MouseButton>>,

    mut commands:Commands,

) {

    let (camera, camera_transform) = q_camera.single();
    let cursor_location = get_cursor_location(wnds, camera, camera_transform);


    let  mut weap = weapq.single_mut();
    let player = playerq.single();
    weap.0.translation.x = player.0.translation.x;
    weap.0.translation.y = player.0.translation.y + 20.;
    let angle = Vec2::new(weap.0.translation.x - cursor_location.x, weap.0.translation.y - cursor_location.y);
    let rads = atan2f(angle.y, angle.x);
    let rot = Quat::from_euler(EulerRot::XYZ, 0., 0., rads);
    weap.0.rotation = rot;

    if buttons.just_pressed(MouseButton::Left) {
        commands.spawn(RocketBundle::new(weap.0.translation, cursor_location));

    }
}

#[derive(Bundle)]
struct PlayerBundle {
    player: Player,
    rigidbody: RigidBody,
    shape: ShapeBundle,
    collider: Collider,
    restitution: Restitution,
    active_events: ActiveEvents,
    jumper: Jumper,
    velocity: Velocity,
    friction: Friction,

}

#[derive(Component)]
struct Player;
impl PlayerBundle {
    fn new(

    ) -> PlayerBundle {

        let shape = shapes::Circle {
            radius: 20.,
            ..default()
        };
        PlayerBundle {
            player: Player,
            rigidbody: RigidBody::Dynamic,
            shape: GeometryBuilder::build_as(&shape, DrawMode::Outlined {
                fill_mode: FillMode::color(Color::hex("66567A").unwrap()),
                outline_mode: StrokeMode::new(Color::BLACK, 1.0),
            },
                Transform::default()
            ),
            collider: Collider::ball(20.),
            restitution: Restitution::coefficient(0.0),
            active_events: ActiveEvents::COLLISION_EVENTS,
            jumper: Jumper {
                jump_impulse: 100.,
                roll_impulse: 0.2, 
                is_jumping: false,
                jump_multiplier: 1.
            },
            velocity: Velocity {
                linvel: Vec2::new(0.0, 0.0),
                angvel: 0.2,
            },
            friction: Friction::coefficient(2.0),
        }

    }
}



#[derive(Component)]
struct Jumper {
    jump_impulse: f32,
    roll_impulse: f32,
    is_jumping: bool,
    jump_multiplier: f32,
}
#[derive(Bundle)]
struct WeaponBundle {
    shape: ShapeBundle,
    weapon: Weapon,
}

#[derive(Component)]
struct Weapon;


impl WeaponBundle {
    fn new(

    ) -> WeaponBundle {        

        let shape = shapes::Rectangle {
            extents: Vec2::new(50., 10.),
            ..default()
        };
        WeaponBundle {
            shape: GeometryBuilder::build_as(&shape, DrawMode::Outlined {
                fill_mode: FillMode::color(Color::hex("000000").unwrap()),
                outline_mode: StrokeMode::new(Color::BLACK, 1.0),
            },
                Transform::default()
            ),
            weapon: Weapon,



        }
    }
}



fn get_cursor_location(
    // need to get window dimensions
    wnds: Res<Windows>,
    // query to get camera transform
    // q_camera: Query<(&Camera, &GlobalTransform), With<MainCamera>>
    camera: &Camera,
    camera_transform: &GlobalTransform
) -> Vec3 {
    // get the camera info and transform
    // assuming there is exactly one main camera entity, so query::single() is OK
    // let (camera, camera_transform) = q_camera.single();

    // get the window that the camera is displaying to (or the primary window)
    let wnd = if let RenderTarget::Window(id) = camera.target {
        wnds.get(id).unwrap()
    } else {
        wnds.get_primary().unwrap()
    };

    // check if the cursor is inside the window and get its position
    if let Some(screen_pos) = wnd.cursor_position() {
        // get the size of the window
        let window_size = Vec2::new(wnd.width() as f32, wnd.height() as f32);

        // convert screen position [0..resolution] to ndc [-1..1] (gpu coordinates)
        let ndc = (screen_pos / window_size) * 2.0 - Vec2::ONE;

        // matrix for undoing the projection and camera transform
        let ndc_to_world = camera_transform.compute_matrix() * camera.projection_matrix().inverse();

        // use it to convert ndc to world-space coordinates
        let world_pos = ndc_to_world.project_point3(ndc.extend(-1.0));

        // reduce it to a 2D value
        let world_pos: Vec2 = world_pos.truncate();

        Vec3::new(world_pos.x, world_pos.y, 0.)
    } else {

        Vec3::new(1., 1., 0.)
    }
}
#[derive(Component)]
struct MainCamera;


#[derive(Component)]
struct Rocket;

#[derive(Bundle)]
struct RocketBundle {
    shape: ShapeBundle,
    rocket: Rocket,
    collider: Collider,
    sensor: Sensor,
    active_events: ActiveEvents,
    rigid_body: RigidBody,
    velocity: Velocity,


}

impl RocketBundle {
    fn new(
        player_position: Vec3,
        cursor_position: Vec3,

    ) -> RocketBundle {
        let velo = get_rocket_velocity(player_position, cursor_position);
        let shape = shapes::Circle {
            radius: 5.,
            ..default()
        };
        RocketBundle {
            rocket: Rocket,
            collider: Collider::ball(5.0),
            sensor: Sensor,
            active_events: ActiveEvents::COLLISION_EVENTS,
            rigid_body: RigidBody::KinematicVelocityBased,
            velocity: Velocity::linear(velo),
            shape: GeometryBuilder::build_as(&shape, DrawMode::Outlined {
                fill_mode: FillMode::color(Color::CYAN)
                ,
                outline_mode: StrokeMode::new(Color::BLACK, 1.0),
            },
                Transform::from_xyz(player_position.x, player_position.y+10., 0.)
            ),
        }
    }
}

fn get_rocket_velocity(player_position: Vec3, cursor_position: Vec3) -> Vec2 {
    let delta_y = cursor_position.y - player_position.y;
    let delta_x = cursor_position.x - player_position.x;
    let vec: Vector2<f32> = vector![delta_x, delta_y];
    let normalized = vec.normalize();
    return Vec2::new(normalized.x * 300., normalized.y * 300.)
}

fn rocket_system(
    rocketq: Query<(Entity), With<Rocket>>,
    mut commands: Commands,

    mut rapier_context: Res<RapierContext>,
) {
    for rocket in rocketq.iter() {

        for (h1, h2, intersecting) in rapier_context.intersections_with(rocket) {

            if h1 == rocket || h2 == rocket {
                println!("in event");
                commands.entity(rocket).despawn();

            }
        }


    }

}
