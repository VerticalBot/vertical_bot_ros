# External axes: rails, gantries, positioners

Any robot can be mounted on another robot: drag the arm onto the rail/gantry/turntable in the station tree.
The arm's base then hangs on the carrier's flange and follows its joints.

## Programming with external axes

- **Jog** the carrier from its own properties panel; the arm follows.
- ***Robot › Move with external axes (rail / gantry)…*** solves the carrier and the arm together for a
  target (combined-chain IK with pre-positioning of the carrier and preference for small carrier motion), and
  records the carrier joints on the target (`carrierJoints` parameter).
- **Curve follow / machining** accept a carrier (RoboDK "optimise external axes"): the carrier is moved to keep
  the arm in a comfortable configuration along the path.
- **Positioners** (welding turntables) work the same way with the part attached to the positioner's flange.

## Post processors

Carrier joints are emitted as external axes in the posts that support them (KUKA `E1…E6`, ABB external
joints, Fanuc extended axes) and as additional joints in the ROS 2 and JSON posts.

## Simulation

Combined trajectories respect both mechanisms' speed limits; collision checking includes the carrier.
