# Calibration and accuracy

The studio implements the calibration workflows of RoboDK with simulated measurement devices, so procedures can
be developed and validated before hardware is available.

| Feature | API | Notes |
|---|---|---|
| TCP calibration | `CalibrateTool(poses_joints, method)` | By points (`CALIBRATE_TCP_BY_POINT`) or by line/axis (`CALIBRATE_TCP_BY_LINE`); returns TCP, mean and max error |
| Reference frame calibration | `Calibrate_Reference(points, method, use_joints, robot)` | 3-point (P1 origin / P1 on X), 6-point, turntable |
| Robot calibration | `Calibrate_Robot(measurements)` | DH identification by Gauss–Newton from tracker measurements; accuracy on/off with `setAccuracyActive` |
| ISO 9283 | `Popup_ISO9283_CubeProgram` | Cube test program and pose accuracy / repeatability statistics |
| Ballbar | `BallbarProgram` | Circular test with radius error statistics |
| Measurement devices | `LaserTracker_Measure`, `StereoCamera_Measure`, `MeasurePose` | Simulated with configurable noise |

Results are shown in the Log and returned to scripts; calibrated tools and frames update the station.
