structure Robot =
struct

open Types
open BDDOps
infix 6 :+: :-: %-% %+% +++
infix 7 *: *% +*: +*+ #*% @*:

type robot = {
     base_body : BDD.Body.body,
     segment1_length : real,
     segment2_length : real,
     get_joint1_angle : unit -> real,
     get_joint2_angle : unit -> real,
     set_base_motor : real -> unit,
     set_joint1_motor : real -> unit,
     set_joint2_motor : real -> unit,
     goal : BDDMath.vec2 ref
}

val home_pos = BDDMath.vec2 (10.0, 15.0)
val home1_pos = BDDMath.vec2 (7.0, 15.0)
val home2_pos = BDDMath.vec2 (13.0, 15.0)

fun arrow_pos Up = BDDMath.vec2 (10.0, 20.0)
  | arrow_pos Down = BDDMath.vec2 (10.0, 10.0)
  | arrow_pos Left = BDDMath.vec2 (5.0, 15.0)
  | arrow_pos Right = BDDMath.vec2 (15.0, 15.0)

fun standby_pos Up = BDDMath.vec2 (10.0, 17.0)
  | standby_pos Down = BDDMath.vec2 (10.0, 13.0)
  | standby_pos Left = BDDMath.vec2 (8.0, 15.0)
  | standby_pos Right = BDDMath.vec2 (12.0, 15.0)



fun make_robot world ground_body start_pos =
    let
        val base_body = BDD.World.create_body (world,
                                                 {typ = BDD.Body.Dynamic,
                                                  position = start_pos,
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
                               (base_body, base_shape, RobotFixture, 2.0)

        val v = start_pos
        val axis = BDDMath.vec2normalized (BDDMath.vec2 (1.0, 0.0))
        val j = BDD.World.create_joint
                 (world, {typ = BDD.Joint.PrismaticDef
                                    {local_anchor_a = BDD.Body.get_local_point (ground_body, v),
                                     local_anchor_b = BDD.Body.get_local_point (base_body, v),
                                     reference_angle = 0.0,
                                     local_axis_a = BDD.Body.get_local_point (ground_body, axis),
                                     lower_translation = ~10.0,
                                     upper_translation = 25.0,
                                     enable_limit = true,
                                     max_motor_force = 10000.0,
                                     motor_speed = 0.0,
                                     enable_motor = true
                                    },
                          user_data = (),
                          body_a = ground_body,
                          body_b = base_body,
                          collide_connected = false
                 })

        val {set_motor_speed = set_base_motor, ... } =
            case BDD.Joint.get_typ j of
                SOME (BDD.Joint.Prismatic p) => p
              | _ => raise Fail "impossible"

        val segment1_length = 10.0
        val segment1_body = BDD.World.create_body (world,
                                                 {typ = BDD.Body.Dynamic,
                                                  position = start_pos :+:
                                                             BDDMath.vec2 (0.0, 3.0),
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
                                   (segment1_body, segment1_shape, RobotFixture, 1.0)
        val () = BDD.Fixture.set_filter (segment1_fixture,
                                         BDD.Fixture.filter_list {categories = [2],
                                                                  mask = [0],
                                                                  group_index = 0})

        val j1 = BDD.World.create_joint
                 (world, {typ = BDD.Joint.RevoluteDef
                                    {local_anchor_a = BDDMath.vec2(0.0, 0.0),
                                     local_anchor_b = BDDMath.vec2(0.0, ~segment1_length / 2.0),
                                     reference_angle = 0.0,
                                     lower_angle = ~0.25 * Math.pi,
                                     upper_angle = 0.0 * Math.pi,
                                     enable_limit = false,
                                     max_motor_torque = 300000.0,
                                     motor_speed = 0.0,
                                     enable_motor = true
                                    },
                          user_data = (),
                          body_a = base_body,
                          body_b = segment1_body,
                          collide_connected = false
                 })

        val {get_joint_angle = get_joint1_angle,
             set_motor_speed = set_joint1_motor,
             ...} = case BDD.Joint.get_typ j1 of
                        SOME (BDD.Joint.Revolute r) => r
                      | _ => raise Fail "impossible"


        val segment2_length = segment1_length
        val segment2_body = BDD.World.create_body (world,
                                                 {typ = BDD.Body.Dynamic,
                                                  position = start_pos :+:
                                                             BDDMath.vec2 (0.0, 10.0),
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
                                   (segment2_body, segment2_shape, RobotFixture, 0.3)

        val () = BDD.Fixture.set_filter (segment2_fixture,
                                         BDD.Fixture.filter_list {categories = [2],
                                                                  mask = [0],
                                                                  group_index = 0})

        val end_shape = BDDShape.Polygon
                            (BDDPolygon.rotated_box
                                 (0.25, 0.25,
                                  BDDMath.vec2(0.0, segment2_length / 2.0), 0.0))
        val foot_fixture = BDD.Body.create_fixture_default
                              (segment2_body, end_shape, RobotFootFixture, 0.0)

        val () = BDD.Fixture.set_filter (foot_fixture,
                                         BDD.Fixture.filter_list {categories = [2],
                                                                  mask = [0],
                                                                  group_index = 0})


        val j2 = BDD.World.create_joint
                 (world, {typ = BDD.Joint.RevoluteDef
                                    {local_anchor_a = BDDMath.vec2(0.0, segment1_length / 2.0),
                                     local_anchor_b = BDDMath.vec2(0.0, ~segment2_length / 2.0),
                                     reference_angle = 0.0,
                                     lower_angle = ~0.25 * Math.pi,
                                     upper_angle = 0.0 * Math.pi,
                                     enable_limit = false,
                                     max_motor_torque = 10000.0,
                                     motor_speed = 0.0,
                                     enable_motor = true
                                    },
                          user_data = (),
                          body_a = segment1_body,
                          body_b = segment2_body,
                          collide_connected = false
                 })

        val {get_joint_angle = get_joint2_angle,
             set_motor_speed = set_joint2_motor,
             ...} = case BDD.Joint.get_typ j2 of
                        SOME (BDD.Joint.Revolute r) => r
                      | _ => raise Fail "impossible"

    in
        {
         base_body = base_body,
         segment1_length = segment1_length,
         segment2_length = segment2_length,
         get_joint1_angle = get_joint1_angle,
         get_joint2_angle = get_joint2_angle,
         set_base_motor = set_base_motor,
         set_joint1_motor = set_joint1_motor,
         set_joint2_motor = set_joint2_motor,
         goal = ref (start_pos :+: BDDMath.vec2 (0.0, segment1_length + segment2_length))
        }
    end

fun make_arrow_body world dir =
    let
        val (x, y) = BDDMath.vec2xy (arrow_pos dir)
        val arrow_body = BDD.World.create_body (world,
                                                 {typ = BDD.Body.Static,
                                                  position = BDDMath.vec2 (x, y),
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

        val arrow_shape = BDDShape.Polygon (BDDPolygon.box (2.0, 2.0))
        val arrow_fixture = BDD.Body.create_fixture
                              (arrow_body,
                                   {shape = arrow_shape,
                                    data = ArrowFixture {direction = dir,
                                                         touching = ref 0},
                                    friction = 0.0,
                                    restitution = 0.0,
                                    density = 0.0,
                                    is_sensor = true,
                                    filter = BDD.Fixture.default_filter})
    in
        arrow_body
    end

val robot1 = ref NONE
val robot2 = ref NONE

fun init world =
    let
        val ground_body = BDD.World.create_body (world,
                                                 {typ = BDD.Body.Static,
                                                  position = BDDMath.vec2 (0.0, 0.0),
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

        val () = robot1 := SOME (make_robot world ground_body ( BDDMath.vec2 (0.0, 0.0)))
        val () = robot2 := SOME (make_robot world ground_body ( BDDMath.vec2 (20.0, 0.0)))

        val uparrow_body = make_arrow_body world Up
        val downarrow_body = make_arrow_body world Down
        val leftarrow_body = make_arrow_body world Left
        val righttarrow_body = make_arrow_body world Right

    in
        ()
    end


fun bullet world ticks =
    if ticks < (ticks_per_second * 88) orelse (Int.mod (ticks, 120) <> 0)
    then ()
    else
  let val y = 10.0 + Math.sin (Real.fromInt ticks) * 10.0
      val body = BDD.World.create_body (world,
                                        {typ = BDD.Body.Dynamic,
                                         position = BDDMath.vec2(~32.0, y),
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
                        (body, shape, GenericFixture, 15.0)
      val () = BDD.Fixture.set_restitution (fixture, 0.05)
  in ()
  end

fun handle_event world _ = ()

val max_angular_speed = 1.1

fun control {base_body : BDD.Body.body,
             segment1_length : real,
             segment2_length : real,
             get_joint1_angle : unit -> real,
             get_joint2_angle : unit -> real,
             set_base_motor : real -> unit,
             set_joint1_motor : real -> unit,
             set_joint2_motor : real -> unit,
             goal : BDDMath.vec2 ref } =
    let
        val base_pos = BDD.Body.get_position base_body
        val base_x = BDDMath.vec2x base_pos
        val angle1 = get_joint1_angle()
        val angle2 = get_joint2_angle() + angle1
        val rot1 = BDDMath.mat22angle angle1
        val rot1' = BDDMath.mat22angle (angle1 + Math.pi / 2.0)
        val rot2 = BDDMath.mat22angle angle2
        val rot2' = BDDMath.mat22angle (angle2 + Math.pi / 2.0)
        val s1 = BDDMath.vec2 (0.0, segment1_length)
        val s2 = BDDMath.vec2 (0.0, segment2_length)
        val p = base_pos :+: ( rot1 +*: s1) :+: (rot2 +*: s2)
        val err = !goal :-: p

        val factor = if BDDMath.vec2length err < 0.25
                     then 0.0
                     else if BDDMath.vec2length err < 1.0
                     then 0.05
                     else if BDDMath.vec2length err < 3.0
                     then 0.3
                     else 0.6
        val d_base_x = factor * 10.0 * BDDMath.dot2 (BDDMath.vec2(1.0, 0.0), err)
        val d_theta1 = BDDMath.clampr (factor * BDDMath.dot2
                                                    (rot1' +*: s1 :+:
                                                           rot2' +*: s2,
                                                     err),
                                       ~max_angular_speed,
                                       max_angular_speed)
        val d_theta2 = BDDMath.clampr (factor * BDDMath.dot2 (rot2' +*: s2, err),
                                       ~max_angular_speed,
                                       max_angular_speed)

    in
        set_base_motor (d_base_x);
        set_joint1_motor (d_theta1);
        set_joint2_motor (d_theta2)
    end

(* goal positions and the times to start going towards them. *)
val robot1_plan : (int * BDDMath.vec2) Queue.queue ref = ref (Queue.empty ())
val robot1_plan_last : (int * BDDMath.vec2) ref = ref (0, home_pos)

val robot2_plan : (int * BDDMath.vec2) Queue.queue ref = ref (Queue.empty ())
val robot2_plan_last : (int * BDDMath.vec2) ref = ref (0, home_pos)

val ticks_processed = ref 0

fun plantos p =
    String.concat (List.map (fn (i, v) => Int.toString i ^ ":  " ^ vtos v ^ "\n")
                            (Queue.tolist p))

fun plan ticks moves =
    let
        val moves' = discard_old_moves (!ticks_processed) moves
        fun process mvs =
            case Queue.deq mvs of
                (NONE, _) => ()
              | (SOME (tk, dir), mvs') =>
                let
                    val goal_pos = arrow_pos dir

                    val (last_tk1, last_pos1) = !robot1_plan_last
                    val (last_tk2, last_pos2) = !robot2_plan_last

                    val dist1 = BDDMath.vec2length (last_pos1 :-: goal_pos)
                    val dist2 = BDDMath.vec2length (last_pos2 :-: goal_pos)

                    fun goodness dist tks =
                       (* (~ dist) *) 0.0 - 10.0 * Real.fromInt (tks)

                    val (last_tk, last_pos, plan, last) =
                        if goodness dist1 last_tk1 > goodness dist2 last_tk2
                        then (last_tk1, last_pos1, robot1_plan, robot1_plan_last)
                        else (last_tk2, last_pos2, robot2_plan, robot2_plan_last)
                    (* when to start the maneuver *)
                    val standby = if last_tk < ticks
                                  then ticks
                                  else last_tk + 1
                    val hit_it = tk + leading_ticks - 5
                    val retreat = tk + leading_ticks + 1
                    val pp_standby = (standby, standby_pos dir)
                    val pp_hit_it = (hit_it, arrow_pos dir)
                    val pp_retreat = (retreat, home_pos)
                in
                    if hit_it > last_tk
                    then (
                        last := pp_retreat;
                        plan := (Queue.enq (pp_standby, !plan ));
                        plan := (Queue.enq (pp_hit_it, !plan ));
                        plan := (Queue.enq (pp_retreat, !plan ))
                        )
                    else ();
                    process mvs'
                end

        val () = process moves'
    in
        ()
    end

fun update_goal rb ticks plan =
    let
        val () = plan := discard_old_moves (ticks - 1) (!plan)
        val () = case Queue.peek (!plan) of
                     NONE => ()
                   | SOME (tk, pos) =>
                     if tk = ticks
                     then
                       (#goal rb) := pos
                     else ()
    in
        ()
    end

fun tick world ticks moves =
    let

        val () = bullet world ticks

        val horizon = leading_ticks div 4
        val () = if ticks > !ticks_processed + horizon
                 then (plan ticks moves;
                       ticks_processed := ticks)
                 else ()

        val r1 = valOf (!robot1)
        val r2 = valOf (!robot2)

        val () = update_goal r1 ticks robot1_plan
        val () = update_goal r2 ticks robot2_plan

        val () = control r1

        val () = control r2

    in
        ()
    end

val test = Test {init = init,
                 handle_event = handle_event,
                 tick = tick}

end
