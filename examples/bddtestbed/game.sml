structure Game :> GAME =
struct
  open Types
  open GL
  open BDDOps
  infix 6 :+: :-: %-% %+% +++
  infix 7 *: *% +*: +*+ #*% @*:

  type state = game_state
  type screen = SDL.surface

  (* Constant parameters *)
  val width = 640
  val height = 480
  val use_gl = true

  (* These will be mutated to the corners of the screen *)
  val lower_left = BDDMath.vec2 (0.0, 0.0)
  val upper_right = BDDMath.vec2 (0.0, 0.0)

  fun screen_to_world (x, y) =
      let val (left, bottom) = BDDMath.vec2xy lower_left
          val (right, top) = BDDMath.vec2xy upper_right
      in BDDMath.vec2
          ((Real.fromInt x  / Real.fromInt width) * (right - left) + left,
           (Real.fromInt (height - y)  / Real.fromInt height) * (top - bottom) + bottom)
      end

  fun body_color b =
      if not (BDD.Body.get_active b)
      then RGB (0.5, 0.5, 0.3)
      else case BDD.Body.get_type b of
               BDD.Body.Static => RGB (0.5, 0.9, 0.5)
             | BDD.Body.Kinematic => RGB (0.5, 0.5, 0.9)
             | BDD.Body.Dynamic =>
               if not (BDD.Body.get_awake b)
               then RGB (0.6, 0.6, 0.6)
               else RGB (0.9, 0.7, 0.7)

  datatype contact_point = CP of {position : BDDMath.vec2,
                                  state : BDDTypes.point_state}

  (* Track the contact points with a presolve callback in order to
  catch any whose life is shorter than a step. *)

  val contact_points = (ref []) : contact_point list ref

  fun pre_solve (contact, old_manifold) =
      let val manifold = BDD.Contact.get_manifold contact
          val (state1, state2) = BDDCollision.get_point_states (old_manifold, manifold)

          (* ??? *)
          val world_manifold = {normal = BDDMath.vec2 (~999.0, ~999.0),
                                points = Array.fromList
                                [ BDDMath.vec2 (~111.0, ~111.0),
                                  BDDMath.vec2 (~222.0, ~222.0)]}
          val () = BDD.Contact.get_world_manifold (world_manifold, contact)

          val points = #points world_manifold
          fun addpoint (i, p) =
              let val cp = CP {position = p, state = Array.sub (state2, i)}
              in contact_points := (cp :: (!contact_points))
              end
          val () = Array.appi addpoint points
      in
          ()
      end


  fun init_test (test as Test {init, ...}) =
      let val gravity = BDDMath.vec2 (0.0, ~10.0)
          val world = BDD.World.world (gravity, true)
          val () = BDD.World.set_pre_solve (world, pre_solve)
          val () = init world
      in GS { test = test, mouse_joint = NONE, world = world }
      end

  val initstate = init_test VerticalStack.test

  fun initscreen screen =
      (

       glBlendFunc GL_SRC_ALPHA GL_ONE_MINUS_SRC_ALPHA;
       glEnable GL_BLEND;

       glEnable GL_TEXTURE_2D;

       glClearColor 0.0 0.0 0.0 1.0;
       glClearDepth 1.0;
       glViewport 0 0 width height;
       glClear GL_COLOR_BUFFER_BIT;
       glMatrixMode GL_PROJECTION;
       glLoadIdentity();

       let val viewCenter = BDDMath.vec2 (0.0, 20.0)
           val ratio = (Real.fromInt width) / (Real.fromInt height)
           val extents = BDDMath.vec2 (ratio * 25.0, 25.0)
           val lower = viewCenter :-: extents
           val upper = viewCenter :+: extents
           val (lx, ly) = BDDMath.vec2xy lower
           val (ux, uy) = BDDMath.vec2xy upper
           val () = BDDMath.vec2set (lower_left, lx, ly)
           val () = BDDMath.vec2set (upper_right, ux, uy)
       in
           glOrtho lx ux ly uy 5.0 ~5.0
       end;

       glMatrixMode GL_MODELVIEW;

       glLoadIdentity();
       ()
      )

  fun drawcontactpoint (CP {position, state, ...}) =
      case state of
          BDDTypes.AddState =>
          Render.draw_point position 10.0 (RGB (0.3, 0.95, 0.3))
        | BDDTypes.PersistState =>
          Render.draw_point position 5.0 (RGB (0.3, 0.3, 0.95))
        | _ => ()

  fun drawfixture color tf fix =
      case BDD.Fixture.shape fix of
          BDDShape.Polygon p =>
          let val n = BDDPolygon.get_vertex_count p
              val vl = List.tabulate (n, fn ii => tf @*: (BDDPolygon.get_vertex(p, ii)))
          in Render.draw_solid_polygon vl color
          end
        | BDDShape.Circle {radius, p} =>
          let val center = tf @*: p
              val axis = (BDDMath.transformr tf) +*: (BDDMath.vec2 (1.0, 0.0))
          in Render.draw_solid_circle center radius axis color
          end

  fun drawbody b =
      let val pos = BDD.Body.get_position b
          val theta = BDD.Body.get_angle b
          val fl = BDD.Body.get_fixtures b
          val color = body_color b
          val tf = BDD.Body.get_transform b
      in
          oapp BDD.Fixture.get_next (drawfixture color tf) fl
      end

  fun drawjoint j =
      let
          val body_a = BDD.Joint.get_body_a j
          val body_b = BDD.Joint.get_body_b j
          val xf1 = BDD.Body.get_transform body_a
          val xf2 = BDD.Body.get_transform body_b
          val x1 = BDDMath.transformposition xf1
          val x2 = BDDMath.transformposition xf2
          val p1 = BDD.Joint.get_anchor_a j
          val p2 = BDD.Joint.get_anchor_b j
          val color = RGB (0.5, 0.8, 0.8)
      in
          case BDD.Joint.get_typ j of
              (SOME (BDD.Joint.Mouse _)) => ()
            | _ =>
              ( Render.draw_segment x1 p1 color;
                Render.draw_segment p1 p2 color;
                Render.draw_segment x2 p2 color
              )
      end

  fun drawmousejoint NONE = ()
    | drawmousejoint (SOME ({get_target, ...}, j)) =
      let val p1 = BDD.Joint.get_anchor_b j
          val p2 = get_target ()
          val (p1x, p1y) = BDDMath.vec2xy p1
          val (p2x, p2y) = BDDMath.vec2xy p2
      in
          glPointSize 4.0;
          glColor3d 0.0 1.0 0.0;
          glBegin GL_POINTS;
          glVertex2d p1x p1y;
          glVertex2d p2x p2y;
          glEnd();
          glPointSize 1.0;

          glColor3d 0.8 0.8 0.8;
          glBegin GL_LINES;
          glVertex2d p1x p1y;
          glVertex2d p2x p2y;
          glEnd()
      end

  fun render screen (GS {world, mouse_joint, ...}) =
  let in
   glClear(GL_COLOR_BUFFER_BIT + GL_DEPTH_BUFFER_BIT);
   glLoadIdentity();

   oapp BDD.Body.get_next drawbody (BDD.World.get_body_list world);
   oapp BDD.Joint.get_next drawjoint (BDD.World.get_joint_list world);
   drawmousejoint mouse_joint;

   List.map drawcontactpoint (!contact_points);
   contact_points := [];

   glFlush();
   SDL.glflip();
   ()
  end

  fun mouse_motion (s as GS {world, mouse_joint = NONE, test}) p = SOME s
    | mouse_motion (s as GS {world, mouse_joint = SOME ({set_target, ...}, _), test}) p =
      let
          val () = set_target p
      in
          SOME s
      end

  fun mouse_up (s as GS {world, mouse_joint = NONE, test}) p = SOME s
    | mouse_up (s as GS {world, mouse_joint = SOME (mj, j), test}) p =
      let val () = BDD.World.destroy_joint (world, j)
      in SOME (GS {world = world, mouse_joint = NONE, test = test})
      end

  fun mouse_down (s as GS {world, mouse_joint, test}) p =
      let val d = BDDMath.vec2 (0.001, 0.001)
          val aabb = { lowerbound = p :-: d,
                       upperbound = p :+: d
                     }
          val result_fixture = (ref NONE) : BDD.fixture option ref
          fun one_fixture f =
              case BDD.Body.get_type (BDD.Fixture.get_body f) of
                  BDD.Body.Dynamic =>
                  if BDD.Fixture.test_point (f, p)
                  then (result_fixture := SOME f; false)
                  else true
                | _ => true
          val () = BDD.World.query_aabb (world, one_fixture, aabb)
          val mbe_new_joint = case !result_fixture of
              NONE => NONE
            | SOME f => let val body = BDD.Fixture.get_body f
                            val mass = BDD.Body.get_mass body
                            val gb = BDD.World.create_body
                                     (world,
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

                             val j = BDD.World.create_joint
                                    (world, {
                                             typ = BDD.Joint.MouseDef
                                                       {target = p,
                                                        max_force = 1000.0 * mass,
                                                        frequency_hz = 5.0,
                                                        damping_ratio = 0.7
                                                       },
                                             user_data = (),
                                             body_a = gb,
                                             body_b = body,
                                             collide_connected = false
                                            })
                            val () = BDD.Body.set_awake (body, true)
                        in case BDD.Joint.get_typ j of
                               SOME (BDD.Joint.Mouse mj) => SOME (mj, j)
                             | _ => raise Fail "expected a mouse joint"
                        end
      in SOME (GS {world = world, mouse_joint = mbe_new_joint, test = test})
      end

  fun handle_event (SDL.E_KeyDown {sym = SDL.SDLK_ESCAPE}) s = NONE
    | handle_event SDL.E_Quit s = NONE
    | handle_event (SDL.E_KeyDown {sym = SDL.SDLK_0}) s =
      SOME (init_test VerticalStack.test)
    | handle_event (SDL.E_KeyDown {sym = SDL.SDLK_1}) s =
      SOME (init_test VaryingRestitution.test)
    | handle_event (SDL.E_KeyDown {sym = SDL.SDLK_2}) s =
      SOME (init_test BulletTest.test)
    | handle_event (SDL.E_KeyDown {sym = SDL.SDLK_3}) s =
      SOME (init_test Revolute.test)
    | handle_event (SDL.E_KeyDown {sym = SDL.SDLK_4}) s =
      SOME (init_test Prismatic.test)
    | handle_event (SDL.E_MouseDown {button, x, y}) s =
      mouse_down s (screen_to_world (x, y))
    | handle_event (SDL.E_MouseUp {button, x, y}) s =
      mouse_up s (screen_to_world (x, y))
    | handle_event (SDL.E_MouseMotion {which, state, x, y, xrel, yrel}) s =
      mouse_motion s (screen_to_world (x, y))
    | handle_event e (s as GS {world, test = Test {handle_event = he, ... }, ...})  =
      (he world e; SOME s)

  val ticks_per_second = 60.0

  fun dophysics world =
      let val timestep = 1.0 / ticks_per_second
          val () = BDD.World.step (world, timestep, 8, 3)
      in () end

  fun tick (s as GS {world, ...}) =
    let val () = dophysics world
    in
        SOME s
    end
end

structure Main =
struct
  structure S = RunGame (Game)
end
