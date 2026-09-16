# Robot drivers (online programming)

Drivers run on the studio server and translate generic commands (`moveJ`, `moveL`, `setDO`, `getDI`,
`runScript`, `stop`) into vendor protocols, reporting joint state back:

| Driver | Protocol | Notes |
|---|---|---|
| `UR` | URScript over the primary/secondary interface (TCP 30001/30002), RTDE-style state | Universal Robots CB3 / e-Series |
| `ABB_RWS` | Robot Web Services (HTTP) | IRC5 / OmniCore with RWS enabled |
| `KUKA_KVP` | KUKAVARPROXY (TCP 7000) | KRC4 with KVP installed; moves through `$POS_ACT`/`E6POS` variables |
| `ROS2` | rosbridge / ROS 2 topics and actions | `joint_states`, `JointTrajectory`, Nav2 for mobile bases |

## Using drivers

- **UI** — *Connect › Studio server…* shows the connection; robot properties hold IP/port; the API run mode
  selects simulation vs. real robot.
- **API** — `robot.setConnectionParams(ip, port, driver)`, `robot.Connect()`, `RDK.setRunMode(RUNMODE_RUN_ROBOT)`;
  moves are then sent to the controller and the simulation follows the reported joints.
- **HTTP/WS** — the server exposes driver commands (`list`, `connect`, `state`, `moveJ`, `moveL`, `setDO`,
  `getDI`, `runScript`, `stop`).

## ROS 2 digital twin

*Connect › ROS 2 via rosbridge…* connects the browser to `rosbridge_server`: publish the simulated state to
`/joint_states` (drive real robots through `ros2_control`), follow the real `/joint_states` (digital twin), or
both. Mobile robots follow `/odom` / TF.

## Safety

Drivers move real machines. Validate programs in simulation, keep the controller in T1/reduced speed for the
first runs and use the controller's safety configuration — the studio does not replace it.
