# Built-in robot library

```{image} ../_static/screens/03-robot-library-dialog.png
:alt: Robot library dialog
:class: screenshot
:width: 620px
```

*Add › Robot from library…* opens the local library — robots defined by DH/URDF-style chains that load
instantly and need no network:

- **Collaborative** — UR3e, UR5e, UR10e, UR16e, UR20 (official DH parameters), Doosan M1013.
- **Industrial** — KUKA KR 6 R900, KR 16 R2010, KR 210 R2700; ABB IRB 120, IRB 1200, IRB 2600, IRB 6700;
  Fanuc LR Mate 200iD, M-20iA, M-410iC; Yaskawa GP12; Stäubli TX2-60; Mecademic Meca500.
- **Mechanisms** — generic palletizer (parallelogram), SCARA, XYZ gantry, 7-axis telescopic harvesting arm.

Robots marked *approx.* use representative link geometry (correct reach and joint limits, simplified
shapes). For exact geometry and vendor meshes use the {doc}`online-library` or import the vendor URDF.

Each entry defines brand, payload, reach, joint limits, home position, speed limits and the default post
processor; these appear in the properties panel and can be edited per robot.
