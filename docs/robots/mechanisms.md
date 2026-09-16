# Mechanisms and custom robots

Besides serial arms the studio models rails, gantries, turntables, positioners, grippers, conveyors and
parallel mechanisms.

## Mechanism builder

The API `BuildMechanism(type, list_obj, parameters, joints_build, joints_home, joints_senses, joints_lim_low,
joints_lim_high, base, tool, name, robot)` mirrors RoboDK's robot builder: 1R/1T/2R/2T/3R/3T/4R/4T, 6-DOF,
7-DOF and SCARA mechanisms are assembled from objects (one per link) and joint definitions. The result is a
regular robot item that can carry other robots.

## Modified DH and robot parameters

`setRobotParams(robot, dhm, poseBase, poseTool)` sets modified DH parameters; the *Parameters…* dialog shows
and edits the table. Standard DH import/export is available through `.dh` files.

## Grippers and tools with axes

A gripper with moving fingers is a mechanism with prismatic or revolute joints; attach it to the arm's flange by
dropping it onto the robot in the tree. `mimic` joints (from URDF) keep coupled fingers in sync.

## Conveyors and process components

Conveyors move objects along a direction at a speed; they are process components with a world-clock
behaviour ({doc}`../process/components`), not kinematic mechanisms, and interact with robots through
attach/detach events and signals.

## Calibration of custom mechanisms

Robot DH identification from measured poses (`Calibrate_Robot`) refines link parameters of any serial chain;
see {doc}`../programming/calibration`.
