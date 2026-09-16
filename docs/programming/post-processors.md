# Post processors

A post processor converts the simulated program into a controller's language. The studio ships 28 posts
(see {doc}`../basic-guide/export`) written in TypeScript, and runs RoboDK's Python posts unmodified.

## Choosing a post

The robot's default post is set in properties › *Kinematics › Post*; the export dialog lets you override it
per export and previews the generated code.

## Writing a post (TypeScript)

Posts implement `PostProcessor` from `src/posts/base.ts`: `id`, `name`, `brand`, `extension` and
`generate(program: PostProgram): PostFile[]`. `PostProgram` provides the compiled event stream
(`progStart`, `setFrame`, `setTool`, `speed`, `moveJ`, `moveL`, `moveC`, `pause`, `setDO`, `waitDI`, `runCode`,
`comment`, `message`, `callProgram`, `gripper`, `progFinish`) with poses as 4×4 matrices relative to the active
frame, joints, configuration flags, robot name and DOF. Helpers convert poses to KUKA/Fanuc/ABB/UR conventions.
Register with `registerPost(post)` and import the module from `src/posts/index.ts`.

## Python posts

Any file defining `class RobotPost` with the RoboDK method set (`ProgStart`, `MoveJ`, `MoveL`, `MoveC`,
`setFrame`, `setTool`, `Pause`, `setDO`, `waitDI`, `RunCode`, `RunMessage`, `ProgFinish`, `ProgSave`) can be
imported through *Program › Import RoboDK post processors (.py)…* and behaves like a built-in post. Details in
{doc}`robodk-posts`.

## Headless generation

On the studio server, `python/post_shim.py <post.py> <events.json>` runs a Python post against an event dump,
and the RoboDK API `MakeProgram` / `ProgramStart` sequence works from any client.
