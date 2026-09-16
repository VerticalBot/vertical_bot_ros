# Using RoboDK post processors

RoboDK distributes its post processors as Python files in the `Posts/` folder of the installation
(`C:/RoboDK/Posts` on Windows). The studio executes those files **unmodified** in the browser:

1. *Program › Import RoboDK post processors (.py)…* and select one or many files (the whole folder works).
2. The posts appear in the export dialog and in each robot's post selector as “*Name (RoboDK post)*”. They are
   stored in the browser (localStorage) and restored at the next start.
3. Export as usual: the post runs in Pyodide (Python compiled to WebAssembly, downloaded once, ≈10 MB) with the
   official Apache-2.0 `robodk.robomath` and `robofileio` modules and a headless `robodialogs` — message boxes
   and file dialogs are logged instead of shown, so posts run unattended.

## What is supported

- `from robodk import *`, `from robodk.robomath import *`, `from robolink import *` imports (legacy and current).
- `RobotPost(robotpost, robotname, robot_axes, **kwargs)` constructors with keyword parameters
  (`axes_type`, `native_name`, `ip_com`, `api_port`, `prog_ptr`, `robot_ptr`, `pose_turntable`, `pose_rail`…).
- `ProgStart`, `ProgFinish`, `ProgSave` (files are collected instead of written), `MoveJ`, `MoveL`, `MoveC`,
  `setFrame`, `setTool`, `Pause`, `setSpeed`, `setAcceleration`, `setSpeedJoints`, `setAccelerationJoints`,
  `setZoneData`, `setDO`, `setAO`, `waitDI`, `RunCode`, `RunMessage`, external axes in the joint list.
- `PROG_EXT`, `PROG` as list or string, `LOG` output shown in the export dialog.

## What is not

Posts that open sockets to the controller (`ip_com`) or launch external programs cannot do so from a browser;
run those through the studio server instead (`python/post_shim.py` uses the system Python).

## Licence note

RoboDK's posts are yours by your RoboDK licence; the studio does not redistribute them.
