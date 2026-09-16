# Cameras, vision and spray simulation

## Cameras

Add a camera item to any item (robot flange, frame, mobile robot) with the API `Cam2D_Add(item, params)` or
*Tools › Camera parameters…*; parameters follow RoboDK (`FOV`, `SIZE=WxH`, `NEAR_LENGTH`, `FAR_LENGTH`, `DEPTH`).
The **Camera** tab shows the live render; `Cam2D_Snapshot` returns a PNG (data URL in the browser).

Simulated **fruit / object detection** (`src/agri/vision.ts`) projects visible fruit into the camera image with
occlusion and noise and yields detections usable by picking programs and missions.

## Spray / painting

`Spray_Add(item_tool, item_object, params)` attaches a spray model (cone/particle) to a tool; while programs
run, deposition on the object is accumulated and `Spray_GetStats` returns coverage statistics
(`Spray_SetState`, `Spray_Clear`). Useful for spraying orchards, painting and coating.

## Video and screenshots

*Tools › Record video of the 3D view (WebM)*, *File › Export screenshot (PNG)*, camera renders from `Cam2D`.
