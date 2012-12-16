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

  val steel = Graphics.requireimage "media/graphics/steel.png"
  val uparrow = Graphics.requireimage "media/graphics/uparrow.png"
  val downarrow = Graphics.requireimage "media/graphics/downarrow.png"
  val leftarrow = Graphics.requireimage "media/graphics/leftarrow.png"
  val rightarrow = Graphics.requireimage "media/graphics/rightarrow.png"

  val groove = SDLMusic.load "media/audio/groove.mp3"

  fun surface_metadata surface =
      let
          val w = SDL.surface_width surface
          val h = SDL.surface_height surface
          val format = case (SDL.get_bytes_per_pixel surface,
                             SDL.is_rgb surface) of
                           (4, true) => GL_RGBA
                         | (4, false) => GL_BGRA
                         | (_, true) => GL_RGB
                         | (_, false) => GL_BGR
      in
          {width = w, height = h, format = format}
      end

  fun blit surface =
      let
          val {width, height, format} = surface_metadata surface
      in
          glDrawPixels width height format GL_UNSIGNED_BYTE (SDL.getpixels surface)
      end

  fun make_message text =
      let val pixel_width = (Font.Huge.width - Font.Huge.overlap)
                            * String.size text
          val pixel_height = Font.Huge.height
          val surf = SDL.makesurface (pixel_width, pixel_height)
          val () = SDL.clearsurface (surf, SDL.color(0w255, 0w255, 0w255, 0w255))
          val () = Font.Huge.draw (surf, 0, 0, text)
      in surf
      end


  val init_message_string = "         0"
  val score_message = make_message init_message_string
  val game_over_message = make_message "the robot scored:"


  fun prepare_score_message score =
      let
          val score' = Int.abs(score)
          val ss = (if score < 0 then "-" else "") ^ Int.toString score'
          val df = String.size init_message_string - String.size ss
          val spaces = List.tabulate (df, fn _ => " ")
          val ms = String.concat spaces ^ ss
      in
          SDL.clearsurface (score_message, SDL.color(0w255, 0w255, 0w255, 0w255));
          Font.Huge.draw (score_message, 0, 0, ms)
      end


  fun glGenSingleTexture () =
      let val arr = Array.array (1, 0)
          val () = glGenTextures 1 arr
      in Array.sub (arr, 0)
      end

  fun load_texture surface =
      let
          val {width, height, format} = surface_metadata surface
          val texture = glGenSingleTexture ()
      in
          glBindTexture GL_TEXTURE_2D texture;
          glTexParameteri GL_TEXTURE_2D GL_TEXTURE_MIN_FILTER GL_NEAREST;
          glTexParameteri GL_TEXTURE_2D GL_TEXTURE_MAG_FILTER GL_NEAREST;
          glTexImage2D GL_TEXTURE_2D 0 4 width height
                       0 format GL_UNSIGNED_BYTE (SDL.getpixels surface);
          texture
      end


  fun screen_to_world (x, y) (View {center, zoom, ...}) =
      let val u = Real.fromInt x / Real.fromInt width
          val v = Real.fromInt (height - y) / Real.fromInt height
          val ratio = (Real.fromInt width) / (Real.fromInt height)
          val extents = BDDMath.vec2 (ratio * zoom * 25.0, zoom * 25.0)
          val lower = center :-: extents
          val upper = center :+: extents
          val (lx, ly) = BDDMath.vec2xy lower
          val (ux, uy) = BDDMath.vec2xy upper
          val px = (1.0 - u) * lx + u * ux
          val py = (1.0 - v) * ly + v * uy
      in BDDMath.vec2 (px, py)
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
               else RGB (1.0, 0.8, 0.9)


  fun resize (v as View {center, zoom, needs_resize = false}) = v
    | resize (View {center, zoom, needs_resize = true}) =
       let
           val ratio = (Real.fromInt width) / (Real.fromInt height)
           val extents = BDDMath.vec2 (ratio * zoom * 25.0, zoom * 25.0)
           val lower = center :-: extents
           val upper = center :+: extents
           val (lx, ly) = BDDMath.vec2xy lower
           val (ux, uy) = BDDMath.vec2xy upper
       in
        glMatrixMode GL_PROJECTION;
        glLoadIdentity();
        glOrtho lx ux ly uy 5.0 ~5.0;
        glMatrixMode GL_MODELVIEW;
        glLoadIdentity();
        View {center = center, zoom = zoom,
              needs_resize = false}
       end

  val up_touched = ref false
  val down_touched = ref false
  val left_touched = ref false
  val right_touched = ref false

  fun make_touching Up = up_touched := true
    | make_touching Down = down_touched := true
    | make_touching Right = right_touched := true
    | make_touching Left = left_touched := true

  fun stop_touching Up = up_touched := false
    | stop_touching Down = down_touched := false
    | stop_touching Right = right_touched := false
    | stop_touching Left = left_touched := false

  fun is_touching Up = !up_touched
    | is_touching Down = !down_touched
    | is_touching Right = !right_touched
    | is_touching Left = !left_touched

  fun begin_contact contact =
      let
          val (fA, fB) = BDD.Contact.get_fixtures contact
      in
          case (BDD.Fixture.get_data fA, BDD.Fixture.get_data fB) of
              (ArrowFixture {direction, touching}, RobotFootFixture) =>
              (touching := !touching + 1;
               make_touching direction)
            | (RobotFootFixture, ArrowFixture {direction, touching}) =>
              (touching := !touching + 1;
               make_touching direction)
            | _ => ()
      end

  fun end_contact contact =
      let
          val (fA, fB) = BDD.Contact.get_fixtures contact
      in
          case (BDD.Fixture.get_data fA, BDD.Fixture.get_data fB) of
              (ArrowFixture {direction, touching}, RobotFootFixture) =>
              (touching := !touching - 1;
               if !touching = 0
               then stop_touching direction
               else ())
            | (RobotFootFixture, ArrowFixture {direction, touching}) =>
              (touching := !touching - 1;
               if !touching = 0
               then stop_touching direction
               else ())
            | _ => ()
      end


  fun init_test (test as Test {init, ...}) =
      let val gravity = BDDMath.vec2 (0.0, ~10.0)
          val world = BDD.World.world (gravity, true)
          val () = BDD.World.set_begin_contact (world, begin_contact)
          val () = BDD.World.set_end_contact (world, end_contact)
          val () = init world
          val center = BDDMath.vec2 (0.0, 20.0)
          val zoom = 1.0
          val view = View {center = center, zoom = zoom,
                           needs_resize = true}
          val settings = {draw_contacts = ref false,
                          paused = ref false,
                          profile = ref NONE}

          val () = SDLMusic.loop (valOf groove)
      in GS { test = test, mouse_joint = NONE, world = world,
              view = view, settings = settings, ticks = 0, score = 0,
              moves = Queue.empty() }
      end

  val initstate = init_test Robot.test

  val steel_texture = ref 0
  val uparrow_texture = ref 0
  val downarrow_texture = ref 0
  val leftarrow_texture = ref 0
  val rightarrow_texture = ref 0

  fun arrow_texture Up = !uparrow_texture
    | arrow_texture Down = !downarrow_texture
    | arrow_texture Left = !leftarrow_texture
    | arrow_texture Right = !rightarrow_texture

  fun initscreen screen =
      (
       glBlendFunc GL_SRC_ALPHA GL_ONE_MINUS_SRC_ALPHA;
       glEnable GL_BLEND;
       glEnable GL_TEXTURE_2D;
       glClearColor 0.0 0.0 0.0 1.0;
       glClearDepth 1.0;
       glViewport 0 0 width height;
       glClear GL_COLOR_BUFFER_BIT;
       glMatrixMode GL_MODELVIEW;
       glLoadIdentity();


       steel_texture := load_texture steel;
       uparrow_texture := load_texture uparrow;
       downarrow_texture := load_texture downarrow;
       leftarrow_texture := load_texture leftarrow;
       rightarrow_texture := load_texture rightarrow;
       ()
      )

  fun drawfixture color tf fix =
      case BDD.Fixture.shape fix of
          BDDShape.Polygon p =>
          let val n = BDDPolygon.get_vertex_count p
              val vl = List.tabulate (n, fn ii => (BDDPolygon.get_vertex(p, ii)))
          in
              case BDD.Fixture.get_data fix of
                  RobotFixture =>
                  Render.draw_textured_polygon vl tf (!steel_texture)
                | RobotFootFixture =>
                  Render.draw_solid_polygon (List.map (fn v => tf @*: v) vl)
                                            (RGB (0.7, 1.0, 0.7))
                                            1.0
                | ArrowFixture {direction, touching} =>
                  let val vl' = List.map (fn v => tf @*: v) vl
                  in
                      if !touching > 0
                      then Render.draw_solid_polygon vl' (RGB (0.7, 0.9, 1.0)) 0.5
                      else ();
                      Render.draw_sprite vl' (arrow_texture direction)
                  end
                | _ => ()
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

  val target_box_y = 40.0

  fun drawmove current_ticks (tick, dir) =
      let
          val ticks_past = current_ticks - tick
          val w = 3.0
          val h = 3.0
          val x = case dir of
                      Left => ~28.0
                    | Down => ~24.0
                    | Up => ~20.0
                    | Right => ~16.0
          val y = target_box_y * (Real.fromInt ticks_past / Real.fromInt leading_ticks)
          val v1 = BDDMath.vec2 (x - w / 2.0, y - h / 2.0)
          val v2 = BDDMath.vec2 (x + w / 2.0, y - h / 2.0)
          val v3 = BDDMath.vec2 (x + w / 2.0, y + h / 2.0)
          val v4 = BDDMath.vec2 (x - w / 2.0, y + h / 2.0)
      in
          Render.draw_sprite [v1, v2, v3, v4] (arrow_texture dir)
      end



  fun draw_target_box () =
      let
          val x = ~22.0
          val y = target_box_y
          val w = 16.0
          val h = 4.0
          val v1 = BDDMath.vec2 (x - w / 2.0, y - h / 2.0)
          val v2 = BDDMath.vec2 (x + w / 2.0, y - h / 2.0)
          val v3 = BDDMath.vec2 (x + w / 2.0, y + h / 2.0)
          val v4 = BDDMath.vec2 (x - w / 2.0, y + h / 2.0)

          val c = RGB (0.7, 0.9, 1.0)
      in
       Render.draw_polygon [v1, v2, v3, v4] (RGB (1.0, 1.0, 1.0));
       if is_touching Left
       then Render.draw_solid_polygon [v1, (0.75 *: v1) :+: (0.25 *: v2),
                                       (0.75 *: v4) :+: (0.25 *: v3), v4]
                                      c 0.5
       else ();
       if is_touching Down
       then Render.draw_solid_polygon [(0.75 *: v1) :+: (0.25 *: v2),
                                       (0.5 *: v1) :+: (0.5 *: v2),
                                       (0.5 *: v4) :+: (0.5 *: v3),
                                       (0.75 *: v4) :+: (0.25 *: v3)]
                                      c 0.5
       else ();
       if is_touching Up
       then Render.draw_solid_polygon [(0.5 *: v1) :+: (0.5 *: v2),
                                       (0.75 *: v2) :+: (0.25 *: v1),
                                       (0.75 *: v3) :+: (0.25 *: v4),
                                       (0.5 *: v4) :+: (0.5 *: v3)]
                                      c 0.5
       else ();
       if is_touching Right
       then Render.draw_solid_polygon [(0.75 *: v2) :+: (0.25 *: v1), v2, v3,
                                       (0.75 *: v3) :+: (0.25 *: v4)]
                                      c 0.5
       else ()
      end

  fun render screen (GS {world, mouse_joint, settings, moves, ticks, score, ...}) =
  let in
   glClear(GL_COLOR_BUFFER_BIT + GL_DEPTH_BUFFER_BIT);
   glLoadIdentity();

   oapp BDD.Body.get_next drawbody (BDD.World.get_body_list world);
   drawmousejoint mouse_joint;

   Queue.app (drawmove ticks) moves;
   draw_target_box();

   (* draw the score *)
   glEnable GL_BLEND;
   prepare_score_message score;
   glDisable GL_TEXTURE_2D;
   glColor3f 1.0 1.0 1.0;
   glRasterPos2d 0.0 42.0;
   glPixelZoom 1.0 ~1.0;
   blit score_message;


   glFlush();
   SDL.glflip();
   ()
  end
    | render screen (GameOver score) =
      let in
          glClear(GL_COLOR_BUFFER_BIT + GL_DEPTH_BUFFER_BIT);
          glLoadIdentity();

          (* draw the score *)
          glColor3f 1.0 1.0 1.0;
          glRasterPos2d ~20.0 30.0;
          glPixelZoom 1.0 ~1.0;
          blit game_over_message;


          prepare_score_message score;
          glDisable GL_TEXTURE_2D;
          glColor3f 1.0 1.0 1.0;
          glRasterPos2d ~12.0 20.0;
          glPixelZoom 1.0 ~1.0;
          blit score_message;

          glFlush();
          SDL.glflip();
          ()
      end


  fun dophysics world =
      let val timestep = 1.0 / (Real.fromInt ticks_per_second)
          val () = BDD.World.step (world, timestep, 12, 10)
      in () end

  fun compute_score moves ticks =
      let
          local
              val up_close = ref 0
              val down_close = ref 0
              val left_close = ref 0
              val right_close = ref 0
          in
           fun close Up = up_close
             | close Down = down_close
             | close Left = left_close
             | close Right = right_close
          end


          val d = 4
          fun tally (tk, dir) =
              if Int.abs (tk + leading_ticks - ticks) < 3
              then (close dir) := !(close dir) + 1
              else ()
          val () = Queue.app tally moves

          fun get_score dir =
              case (is_touching dir, !(close dir) > 0) of
                  (true, true) => 10
                | (false, true) => ~1
                | (true, false) => ~5
                | (false, false) => 0
      in
          get_score Up + get_score Down + get_score Right + get_score Left
      end

  fun dotick (s as GS {world, view, test, mouse_joint, settings, ticks, score, moves}) =
    let
        val score' = score + compute_score moves ticks
        val Test {tick = robot_tick, ...} = test
        val () = robot_tick world ticks moves
        val () = dophysics world
        val moves' = discard_old_moves (ticks - 2 * leading_ticks) moves
    in
        if ticks > 144 * ticks_per_second
        then SOME (GameOver score')
        else
        SOME (GS {world = world, view = view, test = test, ticks = ticks + 1, score = score',
                       mouse_joint = mouse_joint, settings = settings, moves = moves'})
    end
    | dotick s = SOME s

  fun tick (s as GS {world, view, test, mouse_joint, settings, ticks, moves, score}) =
      let val view' = resize view
          val s' = GS {world = world, view = view', test = test, ticks = ticks, score = score,
                       mouse_joint = mouse_joint, settings = settings, moves = moves}
      in
          if not (!(#paused settings))
          then dotick s'
          else SOME s'
      end
    | tick (s as GameOver score) = SOME s

  fun mouse_motion (s as GS {world, mouse_joint = NONE, test, ...}) p = SOME s
    | mouse_motion (s as GS {world, mouse_joint = SOME ({set_target, ...}, _),
                             test, ...}) p =
      let
          val () = set_target p
      in
          SOME s
      end

  fun mouse_up (s as GS {world, mouse_joint = NONE, test, ...}) p = SOME s
    | mouse_up (s as GS {world, mouse_joint = SOME (mj, j), test, view, settings,
                         ticks, moves, score}) p =
      let val () = BDD.World.destroy_joint (world, j)
      in SOME (GS {world = world, mouse_joint = NONE, ticks = ticks, moves = moves,
                   score = score, test = test, view = view, settings = settings})
      end

  fun mouse_down (s as GS {world, mouse_joint, test, view, settings, ticks, score, moves}) p =
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
                                                        max_force = 100000.0 * mass,
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
      in SOME (GS {world = world, mouse_joint = mbe_new_joint, ticks = ticks, score = score,
                   test = test, view = view, settings = settings, moves = moves})
      end

  fun update_view (GS {world, mouse_joint, test, settings, ticks, moves, score,
                       view = View {center, zoom, ...}}) v s =
      SOME (GS {world = world, mouse_joint = mouse_joint, test = test,
                settings = settings, ticks = ticks, moves = moves, score = score,
                view = View {center = center :+: v,
                             zoom = zoom * s,
                             needs_resize = true}})


  fun add_move (GS {world, mouse_joint, test, settings, ticks, moves, score,
                    view}) dir =
      let
          val moves' = Queue.enq ((ticks, dir), moves)
      in
      SOME (GS {world = world, mouse_joint = mouse_joint, test = test,
                settings = settings, ticks = ticks, moves = moves',
                score = score, view = view})
      end
    | add_move s dir = SOME s

  fun handle_event (SDL.E_KeyDown {sym = SDL.SDLK_ESCAPE}) s = NONE
    | handle_event SDL.E_Quit s = NONE

    | handle_event (SDL.E_KeyDown {sym = SDL.SDLK_SPACE}) (GameOver s) =
      NONE

    | handle_event _ (GameOver s) =
      SOME (GameOver s)

    | handle_event (SDL.E_KeyDown {sym = sym as SDL.SDLK_LEFT}) s =
      add_move s Left
    | handle_event (SDL.E_KeyDown {sym = sym as SDL.SDLK_RIGHT}) s =
      add_move s Right
    | handle_event (SDL.E_KeyDown {sym = sym as SDL.SDLK_UP}) s =
      add_move s Up
    | handle_event (SDL.E_KeyDown {sym = sym as SDL.SDLK_DOWN}) s =
      add_move s Down

    | handle_event (SDL.E_MouseDown {button, x, y}) (s as (GS gs)) =
      mouse_down s (screen_to_world (x, y) (#view gs))
    | handle_event (SDL.E_MouseUp {button, x, y}) (s as (GS gs)) =
      mouse_up s (screen_to_world (x, y) (#view gs))
    | handle_event (SDL.E_MouseMotion {which, state, x, y, xrel, yrel}) (s as (GS gs)) =
      mouse_motion s (screen_to_world (x, y) (#view gs))


    | handle_event (SDL.E_KeyDown {sym = SDL.SDLK_z}) s =
      update_view s (BDDMath.vec2 (0.0, 0.0)) 1.1
    | handle_event (SDL.E_KeyDown {sym = SDL.SDLK_x}) s =
      update_view s (BDDMath.vec2 (0.0, 0.0)) 0.9

    | handle_event e (s as GS {world, test = Test {handle_event = he, ... }, ...})  =
      (he world e; SOME s)

end

structure Main =
struct
  structure S = RunGame (Game)
end
