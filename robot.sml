structure Robot =
struct

open Types

val columnCount = 5
val rowCount = 16

val xs = Array.fromList [0.0, ~10.0, ~5.0, 5.0, 10.0]

fun init world =
    let
        val ground_body = BDD.World.create_body (world,
                                                 {typ = BDD.Body.Static,
                                                  position = BDDMath.vec2 (0.0, ~0.01),
                                                  angle = 0.0,
                                                  linear_velocity = BDDMath.vec2_zero,
                                                  angular_velocity = 0.0,
                                                  linear_damping = 0.0,
                                                  angular_damping = 0.0,
                                                  allow_sleep = true,
                                                  awake = true,
                                                  fixed_rotation = false,
                                                  bullet = false,
                                                  active = true,
                                                  data = (),
                                                  inertia_scale = 1.0
                                                })
        val ground_shape = BDDShape.Polygon (BDDPolygon.box (40.0, 0.01))
        val ground_fixture = BDD.Body.create_fixture_default
                             (ground_body, ground_shape, (), 1.0)

        val base_body = BDD.World.create_body (world,
                                                 {typ = BDD.Body.Dynamic,
                                                  position = BDDMath.vec2 (0.0, 1.0),
                                                  angle = 0.0,
                                                  linear_velocity = BDDMath.vec2_zero,
                                                  angular_velocity = 0.0,
                                                  linear_damping = 0.0,
                                                  angular_damping = 0.0,
                                                  allow_sleep = false,
                                                  awake = true,
                                                  fixed_rotation = false,
                                                  bullet = false,
                                                  active = true,
                                                  data = (),
                                                  inertia_scale = 1.0
                                                })

        val base_shape = BDDShape.Polygon (BDDPolygon.box (4.0, 0.5))
        val base_fixture = BDD.Body.create_fixture_default
                               (base_body, base_shape, (), 5.0)

        val v = BDDMath.vec2 (0.0, 0.0)
        val axis = BDDMath.vec2normalized (BDDMath.vec2 (1.0, 0.0))
        val j = BDD.World.create_joint
                 (world, {typ = BDD.Joint.PrismaticDef
                                    {local_anchor_a = BDD.Body.get_local_point (ground_body, v),
                                     local_anchor_b = BDD.Body.get_local_point (base_body, v),
                                     reference_angle = 0.0,
                                     local_axis_a = BDD.Body.get_local_point (ground_body, axis),
                                     lower_translation = ~10.0,
                                     upper_translation = 10.0,
                                     enable_limit = true,
                                     max_motor_force = 10000.0,
                                     motor_speed = 10.0,
                                     enable_motor = false
                                    },
                          user_data = (),
                          body_a = ground_body,
                          body_b = base_body,
                          collide_connected = false
                 })

        val segment1_length = 6.0
        val segment1_body = BDD.World.create_body (world,
                                                 {typ = BDD.Body.Dynamic,
                                                  position = BDDMath.vec2 (0.0, 3.0),
                                                  angle = 0.0,
                                                  linear_velocity = BDDMath.vec2_zero,
                                                  angular_velocity = 0.0,
                                                  linear_damping = 0.0,
                                                  angular_damping = 0.0,
                                                  allow_sleep = false,
                                                  awake = true,
                                                  fixed_rotation = false,
                                                  bullet = false,
                                                  active = true,
                                                  data = (),
                                                  inertia_scale = 1.0
                                                })


        val segment1_shape = BDDShape.Polygon (BDDPolygon.box (0.5, segment1_length / 2.0))
        val segment1_fixture = BDD.Body.create_fixture_default
                                   (segment1_body, segment1_shape, (), 5.0)

        val j1 = BDD.World.create_joint
                 (world, {typ = BDD.Joint.RevoluteDef
                                    {local_anchor_a = BDDMath.vec2(0.0, 0.0),
                                     local_anchor_b = BDDMath.vec2(0.0, ~segment1_length / 2.0),
                                     reference_angle = 0.0,
                                     lower_angle = ~0.25 * Math.pi,
                                     upper_angle = 0.0 * Math.pi,
                                     enable_limit = false,
                                     max_motor_torque = 0.0,
                                     motor_speed = 0.0,
                                     enable_motor = false
                                    },
                          user_data = (),
                          body_a = base_body,
                          body_b = segment1_body,
                          collide_connected = false
                 })


        val segment2_length = segment1_length
        val segment2_body = BDD.World.create_body (world,
                                                 {typ = BDD.Body.Dynamic,
                                                  position = BDDMath.vec2 (0.0, 10.0),
                                                  angle = 0.0,
                                                  linear_velocity = BDDMath.vec2_zero,
                                                  angular_velocity = 0.0,
                                                  linear_damping = 0.0,
                                                  angular_damping = 0.0,
                                                  allow_sleep = false,
                                                  awake = true,
                                                  fixed_rotation = false,
                                                  bullet = false,
                                                  active = true,
                                                  data = (),
                                                  inertia_scale = 1.0
                                                })


        val segment2_shape = BDDShape.Polygon (BDDPolygon.box (0.5, segment2_length / 2.0))
        val segment2_fixture = BDD.Body.create_fixture_default
                                   (segment2_body, segment2_shape, (), 5.0)


        val j2 = BDD.World.create_joint
                 (world, {typ = BDD.Joint.RevoluteDef
                                    {local_anchor_a = BDDMath.vec2(0.0, segment1_length / 2.0),
                                     local_anchor_b = BDDMath.vec2(0.0, ~segment2_length / 2.0),
                                     reference_angle = 0.0,
                                     lower_angle = ~0.25 * Math.pi,
                                     upper_angle = 0.0 * Math.pi,
                                     enable_limit = false,
                                     max_motor_torque = 0.0,
                                     motor_speed = 0.0,
                                     enable_motor = false
                                    },
                          user_data = (),
                          body_a = segment1_body,
                          body_b = segment2_body,
                          collide_connected = false
                 })



    in
        ()
    end


fun bullet world =
  let val body = BDD.World.create_body (world,
                                        {typ = BDD.Body.Dynamic,
                                         position = BDDMath.vec2(~31.0, 5.0),
                                         angle = 0.0,
                                         linear_velocity = BDDMath.vec2(400.0, 0.0),
                                         angular_velocity = 0.0,
                                         linear_damping = 0.0,
                                         angular_damping = 0.0,
                                         allow_sleep = true,
                                         awake = true,
                                         fixed_rotation = false,
                                         bullet = true,
                                         active = true,
                                         data = (),
                                         inertia_scale = 1.0
                                       })
      val shape = BDDShape.Circle {radius = 0.25,
                                   p = BDDMath.vec2_zero}
      val fixture = BDD.Body.create_fixture_default
                        (body, shape, (), 20.0)
      val () = BDD.Fixture.set_restitution (fixture, 0.05)
  in ()
  end

fun handle_event world (SDL.E_KeyDown {sym = SDL.SDLK_COMMA}) = bullet world
  | handle_event world _ = ()

val test = Test {init = init,
                 handle_event = handle_event,
                 tick = ignore}

end
