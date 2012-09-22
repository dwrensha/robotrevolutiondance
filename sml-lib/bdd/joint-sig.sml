signature BDDJOINT =
sig

  (* Supplied by functor argument *)
  type fixture_data
  type body_data
  type joint_data

  type body
  type fixture
  type joint
  type world
  type contact
  type filter
  type contactedge
  type jointedge

  datatype joint_type =
      Revolute
    | Prismatic
    | Distance
    | Pulley
    | Mouse
    | Gear
    | Line
    | Weld
    | Friction


  val get_next : joint -> joint option

end
