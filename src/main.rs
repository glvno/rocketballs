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
const WALL_THICKNESS: f32 = BALL_RADIUS;
// x coordinates
const LEFT_WALL: f32 = -1000.;
const RIGHT_WALL: f32 = 1000.;
// y coordinates
const BOTTOM_WALL: f32 = -1000.;
const TOP_WALL: f32 = 1000.;
const BALL_RADIUS: f32 = 40.;
const ROCKET_VELOCITY: f32 = 25. * BALL_RADIUS;
const ROCKET_KNOCKBACK: f32 = 25. * BALL_RADIUS;
const ROCKET_KNOCKBACK_RADIUS: f32 = 2.5 * BALL_RADIUS;
const WEAPON_LENGTH: f32 = 2.5 * BALL_RADIUS;
const WEAPON_WIDTH: f32 = 0.5 * BALL_RADIUS;
const ROCKET_RADIUS: f32 = 0.25 * BALL_RADIUS;
const WEAPON_OFFSET: f32 = 1.25 * BALL_RADIUS;
const CAMERA_HEIGHT: f32 = 200.;
const PIXELS_PER_METER: f32 = 2. * BALL_RADIUS;
const AIRCONTROL_CAP: f32 = -1. * 5. * BALL_RADIUS;
const AIRCONTROL: f32 = BALL_RADIUS/7.;
const PLAYER_ANGVEL: f32 = 0.05;
const JUMP_IMPULSE: f32 = 5. * BALL_RADIUS;
const KNOCKBACK_DIVISOR: f32 = BALL_RADIUS - 1.;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(PIXELS_PER_METER))
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
        // .add_system(rocket_system)
        .add_fixed_timestep(Duration::from_millis(17), "physics_timestep")
        .add_fixed_timestep_system("physics_timestep", 0, control_player)
        .add_fixed_timestep_system("physics_timestep", 0, jump_reset)
        .add_fixed_timestep_system("physics_timestep", 0, rocket_system)

        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn(( Camera2dBundle::default(), MainCamera ));
}

fn setup_physics(
    mut commands: Commands,
) {
    commands.spawn(BallBundle::new()).insert(Player);
    commands.spawn(BallBundle::new());

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
    mut query: Query<(Entity, &mut Jumper, &mut Velocity), With<Player>>,
    mut commands: Commands
) {
    let   (entity, mut jumper, mut velocity) = query.single_mut();
    let mut impulse = ExternalImpulse {
        impulse: Vec2::new(0.0, 0.0),
        torque_impulse: 0.0
    };
    if keys.just_released(KeyCode::Space) && !jumper.is_jumping {
        impulse.impulse = Vec2::new(0., JUMP_IMPULSE * jumper.jump_multiplier);
        jumper.is_jumping = true;
        jumper.jump_multiplier = 1.;
    }

    if keys.pressed(KeyCode::A) {

        impulse.torque_impulse = jumper.roll_impulse;

        if jumper.is_jumping && velocity.linvel.x > AIRCONTROL_CAP{
            velocity.linvel.x -= AIRCONTROL;
        }

    }

    if keys.pressed(KeyCode::D) {

        impulse.torque_impulse = 0. - jumper.roll_impulse;

        if jumper.is_jumping && velocity.linvel.x < AIRCONTROL_CAP {
            velocity.linvel.x += AIRCONTROL;
        }
    }    
    if keys.pressed(KeyCode::Space) {

        velocity.angvel = 0.0;
        if velocity.linvel.x.abs() < 2. {
            jumper.jump_multiplier = 1.
        } else if jumper.jump_multiplier <= 1.6 {
            jumper.jump_multiplier += 0.01;
        }
    }
    if velocity.angvel <  -30. {
        velocity.angvel = -30.;
    } else if velocity.angvel > 30. {
        velocity.angvel = 30.
    }

    commands.entity(entity).insert(impulse);

}

fn camera_system(
    mut camq: Query<( &mut Transform, With<Camera2d> ), Without<Player>>,
    // playerq: Query<( &Transform, With<Player> ), Without<Camera2d>>,
    playerq: Query<( &Transform, With<Player> ), Without<Camera2d>>,

) {

    let mut cam = camq.get_single_mut().unwrap();
    let transform = playerq.get_single().unwrap();
    cam.0.translation.x = transform.0.translation.x;
    cam.0.translation.y = transform.0.translation.y + CAMERA_HEIGHT;
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
    weap.0.translation.y = player.0.translation.y + WEAPON_OFFSET;
    let angle = Vec2::new(weap.0.translation.x - cursor_location.x, weap.0.translation.y - cursor_location.y);
    let rads = atan2f(angle.y, angle.x);
    let rot = Quat::from_euler(EulerRot::XYZ, 0., 0., rads);
    weap.0.rotation = rot;

    if buttons.just_pressed(MouseButton::Left) {
        commands.spawn(RocketBundle::new(weap.0.translation, cursor_location));

    }
}

#[derive(Bundle)]
struct BallBundle {
    ball: Ball,
    rigidbody: RigidBody,
    shape: ShapeBundle,
    collider: Collider,
    restitution: Restitution,
    active_events: ActiveEvents,
    jumper: Jumper,
    velocity: Velocity,
    friction: Friction,
    gravity_scale: GravityScale

}

#[derive(Component)]
struct Player;
impl BallBundle {
    fn new(

    ) -> BallBundle {

        let shape = shapes::Circle {
            radius: BALL_RADIUS,
            ..default()
        };
        BallBundle {
            ball: Ball,
            rigidbody: RigidBody::Dynamic,
            shape: GeometryBuilder::build_as(&shape, DrawMode::Outlined {
                fill_mode: FillMode::color(Color::hex("66567A").unwrap()),
                outline_mode: StrokeMode::new(Color::BLACK, 1.0),
            },
                Transform::from_xyz(0., 0., 0.)
            ),
            collider: Collider::ball(BALL_RADIUS),
            restitution: Restitution::coefficient(0.1),
            active_events: ActiveEvents::COLLISION_EVENTS,
            jumper: Jumper {
                jump_impulse: JUMP_IMPULSE,
                roll_impulse: PLAYER_ANGVEL, 
                is_jumping: false,
                jump_multiplier: 1.
            },
            velocity: Velocity {
                linvel: Vec2::new(0.0, 0.0),
                angvel: 0.2,
            },
            friction: Friction::coefficient(2.0),
            gravity_scale: GravityScale(5.0)
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
            extents: Vec2::new(WEAPON_LENGTH, WEAPON_WIDTH),
            ..default()
        };
        WeaponBundle {
            shape: GeometryBuilder::build_as(&shape, DrawMode::Outlined {
                fill_mode: FillMode::color(Color::hex("000000").unwrap()),
                outline_mode: StrokeMode::new(Color::BLACK, 1.0),
            },
                Transform::from_xyz(0., 0., 1.)
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
            radius: ROCKET_RADIUS,
            ..default()
        };
        RocketBundle {
            rocket: Rocket,
            collider: Collider::ball(ROCKET_RADIUS),
            sensor: Sensor,
            active_events: ActiveEvents::COLLISION_EVENTS,
            rigid_body: RigidBody::KinematicVelocityBased,
            velocity: Velocity::linear(velo),
            shape: GeometryBuilder::build_as(&shape, DrawMode::Outlined {
                fill_mode: FillMode::color(Color::CYAN)
                ,
                outline_mode: StrokeMode::new(Color::BLACK, 1.0),
            },
                Transform::from_xyz(player_position.x, player_position.y, 0.)
            ),
        }
    }
}

fn get_rocket_velocity(player_position: Vec3, cursor_position: Vec3) -> Vec2 {
    let delta_y = cursor_position.y - player_position.y;
    let delta_x = cursor_position.x - player_position.x;
    let vec: Vector2<f32> = vector![delta_x, delta_y];
    let normalized = vec.normalize();
    return Vec2::new(normalized.x * ROCKET_VELOCITY, normalized.y * ROCKET_VELOCITY)
}

fn rocket_system(
    rocketq: Query<(Entity, &Transform), With<Rocket>>,

    playerq: Query<(Entity), With<Player>>,

    mut ballq: Query<(Entity, &Transform), With<Ball>>,
    mut commands: Commands,

    mut rapier_context: Res<RapierContext>,
) {
    let player = playerq.single();
    for (rocket, rocket_transform) in rocketq.iter() {

        for (h1, h2, intersecting) in rapier_context.intersections_with(rocket) {

            if h1 == player || h2 == player {
                continue
            } else {
                commands.entity(rocket).despawn();

            }
            for (balle, mut ball_transform) in ballq.iter_mut() {
                let dist = rocket_transform.translation.distance(ball_transform.translation);
                if dist <= ROCKET_KNOCKBACK_RADIUS {
                    let dx =  ball_transform.translation.x - rocket_transform.translation.x;
                    let dy = ball_transform.translation.y - rocket_transform.translation.y;
                    let vec: Vector2<f32> = vector![dx, dy];
                    let normalized = vec.normalize();
                    commands.entity(balle).insert(ExternalImpulse {
                        impulse: Vec2::new(normalized.x * ROCKET_KNOCKBACK * dist / ROCKET_KNOCKBACK_RADIUS, normalized.y * ROCKET_KNOCKBACK * dist/ROCKET_KNOCKBACK_RADIUS),
                        ..default()
                    });
                }


            }
        }


    }

}
#[derive(Component)]
struct Ball;
