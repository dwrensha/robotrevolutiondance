structure Types =
struct

datatype spec = RGB of GL.GLdouble * GL.GLdouble * GL.GLdouble;

datatype direction = Up | Down | Left | Right

datatype fixture_data = RobotFixture
                      | RobotFootFixture
                      | ArrowFixture of
                        {direction : direction,
                         touching : int ref}
                      | GenericFixture

structure BDD = BDDWorld(
                struct type fixture_data = fixture_data
                       type body_data = unit
                       type joint_data = unit
                end
                )

datatype view = View of
         {center : BDDMath.vec2,
          zoom : real,
          needs_resize : bool
         }

datatype test = Test of
         {init : BDD.world -> unit,
          handle_event : BDD.world -> SDL.event -> unit,
          tick : BDD.world -> int -> (int * direction) Queue.queue -> unit
         }

type profile_data = { step_count : int,
                      total : BDDDynamicsTypes.profile,
                      max : BDDDynamicsTypes.profile
                    }

fun new_profile_data () =
    let fun new_profile () = {step = Time.zeroTime,
                              collide = Time.zeroTime,
                              solve = Time.zeroTime,
                              solve_toi = Time.zeroTime}
    in
        {step_count = 0, total = new_profile(), max = new_profile()}
    end

type settings =
         { draw_contacts : bool ref,
           paused : bool ref,
           profile : profile_data option ref
         }

type mouse_joint = {get_target : unit -> BDDMath.vec2,
                    set_target : BDDMath.vec2 -> unit
                   }

datatype game_state = GS of {world : BDD.world,
                             mouse_joint : (mouse_joint * BDD.joint) option,
                             test : test,
                             view : view,
                             ticks : int,
                             moves : (int * direction) Queue.queue,
                             settings : settings,
                             score : int
                            }
                    | GameOver of int

val ticks_per_second = 60
val leading_ticks = ticks_per_second * 4


(* Get rid of everything less than or equal to |ticks_threshold| *)
fun discard_old_moves ticks_threshold moves =
    case Queue.peek moves of
        NONE => moves
      | SOME (t, d) => if t > ticks_threshold
                       then moves
                       else discard_old_moves ticks_threshold (#2 (Queue.deq moves))




end
