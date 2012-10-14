structure Types =
struct

datatype spec = RGB of real * real * real;

structure BDD = BDDWorld(
                struct type fixture_data = unit
                       type body_data = unit
                       type joint_data = unit
                end
                )


datatype constants = CONST of {width : int,
                               height : int,
                               left : real,
                               right : real,
                               bottom : real,
                               top : real,
                               gravity : BDDMath.vec2}

datatype test = Test of
         {init : BDD.world -> unit,
          handle_event : BDD.world -> SDL.event -> unit
         }

type mouse_joint = {get_target : unit -> BDDMath.vec2,
                    set_target : BDDMath.vec2 -> unit
                   }

datatype game_state = GS of {world : BDD.world,
                             mouse_joint : (mouse_joint * BDD.joint) option,
                             test : test
                            }



end
