functor BDDJoint(Arg:
                 sig
                   type fixture_data
                   type body_data
                   type joint_data
                 end) : BDDJOINT =
struct
  open Arg
  open BDDSettings
  open BDDTypes
  open BDDMath
  open BDDOps
  infix 6 :+: :-: %-% %+% +++
  infix 7 *: *% +*: +*+ #*% @*:

  exception BDDJoint of string

  structure D = BDDDynamics
  structure DT = BDDDynamicsTypes(Arg)
  open DT
  type filter = D.filter

  open D.J

  fun !! (SOME x) = x
    | !! NONE = raise BDDJoint "Expected non-NONE value, like Box2D does"

  fun get_anchor_a joint =
      (#get_anchor_a (!!(get_dispatch joint))) ()

  fun get_anchor_b joint =
      (#get_anchor_b (!!(get_dispatch joint))) ()
end
