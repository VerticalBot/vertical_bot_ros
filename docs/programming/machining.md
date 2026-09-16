# Robot machining, cutting and 3D printing

The studio covers RoboDK's *Robot machining project*, *Curve follow*, *Point follow* and *3D printing* projects.

## From NC / G-code

```{image} ../_static/screens/16-machining-dialog.png
:alt: Robot machining project dialog
:class: screenshot
:width: 620px
```

1. Drop a `.nc`, `.gcode`, `.ngc`, `.tap`, `.cnc` or `.gco` file onto the 3D view. The parser understands
   G0/G1/G2/G3 (arcs tessellated in the active plane), G17/18/19, G20/21 (inch/mm), G90/91, feed rates (F),
   spindle M3/M4/M5, coolant, tool changes (T / M6) and extrusion (E axis with absolute/relative modes and
   retraction detection for FDM printing). The result is an object with *rapid* and *cut* segments and metadata
   (feed, spindle, tool, extruding); the log summarises lengths and feeds.
2. Position the object (the part frame) where the workpiece is.
3. ***Robot › Robot machining project (NC / G-code / 3D print)…*** and choose:
   - approach/retract distance, rapid speed, optional cutting-speed override (otherwise NC feed in mm/min),
   - spindle/laser digital output and extruder output (switched per segment),
   - tool Z direction: −Z of the part (milling, printing) or along curve normals (surface following),
   - free rotation about the tool axis (symmetric tools), resampling step, rounding radius.
4. The generated program contains home, approach, per-segment speed changes and outputs, all cutting and rapid
   moves as targets in the part frame, retract and return. The dialog reports points, segments, cut and rapid
   lengths, estimated cycle time and unreachable points.

## From curves and points

```{image} ../_static/screens/17-machining-program.png
:alt: Generated machining program on the tutorial table
:class: screenshot
```

*Robot › Follow curve / points of an object…* generates programs along object curves (welding seams, glue
beads, pruning cuts, spraying along canopies) or through points (drilling, planting, pollinating) with
approach/retract, normals, tool-Z optimisation, preferred configuration, IO switching and optional external
axes. Curves come from NC files, RoboDK projects, the API (`AddCurve`, `AddPoints`, `ProjectPoints`) or CSV.

## API

```python
proj = RDK.AddMachiningProject("Pocket", robot)
prog, status = proj.setMachiningParameters(ncfile="", part=part_object,
    params="Approach=50 RapidSpeed=250 SpindleIO=Spindle ExtruderIO=E Rounding=1")
```

`Update()` regenerates the program after moving the part; `MachiningParameters()` returns the settings.

## RoboDK machining projects

Machining projects of `.rdk` stations are transferred by the converter with robot / part / program links and
the part's curves and points; the generated program comes along, or is regenerated here when missing
({doc}`../interop/robodk`).
