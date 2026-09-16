# Generating robot programs

The studio turns the simulated program into controller code with a **post processor** — exactly the concept
used by RoboDK. Each robot has a default post (properties › *Kinematics › Post*), and *Program › Export with post
processor…* lets you pick another one, preview the code and download the files.

## Built-in post processors

| Post | Output |
|---|---|
| KUKA KRC4 (KRL) / KUKA KRC2 | `.src` + `.dat` with `$BASE`, `$TOOL`, PTP/LIN/CIRC, `$VEL`, `$APO`, IO |
| ABB RAPID (IRC5 / OmniCore) / S4C legacy | `.mod` module with tooldata/wobjdata, MoveJ/L/C, zones, IO |
| Fanuc R-30iA/iB (LS) / RJ3 | `.ls` with positions in Cartesian/joint, CNT, DO/WAIT |
| Universal Robots (URScript) | `.script` with movej/movel/movec, blend radius, IO |
| Yaskawa Motoman (INFORM JBI) | `.JBI` |
| Stäubli VAL3 | `.pgx` |
| Doosan Robotics (DRL) | `.drl` |
| Mecademic (Python) | Python using the Mecademic API |
| Denso RC8, Kawasaki AS, Nachi SLIM, Comau PDL2, Epson SPEL+, Techman/Omron TM, Hanwha Rodi, Kinova Kortex, Mitsubishi MELFA, AUBO, JAKA, Elite, Dobot | vendor formats |
| ROS 2 | rclpy node with a `JointTrajectory` + JSON trajectory |
| RoboDK API (Python script) | Rebuilds the program inside RoboDK through its API |
| JSON, Generic CSV | Neutral formats for custom controllers |

Original RoboDK Python post processors can be imported and run unmodified ({doc}`../programming/robodk-posts`).

## What the post receives

The program is compiled into a stream of events: program start, frame and tool poses, speed changes, moves
(with pose in the active frame, joints and configuration), pauses, IO, code, messages, sub-program calls,
program end. Poses are converted to each controller's convention automatically (KUKA ABC, Fanuc WPR, ABB
quaternion, UR rotation vector). Programs longer than the controller's limits are split according to the post.

## Sending programs to robots

- Download the file and load it on the controller as usual.
- Run directly through a **driver** from the studio server (UR, ABB RWS, KUKA KVP, ROS 2): see
  {doc}`../api/drivers`.
- Use the RoboDK API run mode `RUNMODE_RUN_ROBOT` from Python scripts.
