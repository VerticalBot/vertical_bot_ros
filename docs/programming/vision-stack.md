# Machine vision stack (СТЗ): sensors, models, VLM / VLA, point clouds

A perception system for a robot is a **stack**: a sensor (mono, stereo, RGB-D, ToF, 3D LiDAR, thermal…), a
compute target (Jetson, industrial PC with a GPU, an in-camera accelerator, a cloud GPU) and one model per task —
detection, tracking, classification, segmentation, keypoints, 6D pose / 3D position, depth, point-cloud
detection and segmentation, grasp generation, vision-language queries (VLM) and vision-language-action policies
(VLA). The studio holds a catalogue of all of these, recommends a stack for the task and the environment,
**runs the pipeline on the station's cameras** (simulated ground truth or real models plugged in) and exports a
ROS 2 perception package with the same configuration.

```{image} ../_static/screens/36-vision-wizard.png
:alt: Machine vision stack wizard
:class: screenshot
:width: 780px
```

## Adding a camera and choosing a stack

1. Select the item that carries the sensor (a tool on the flange for eye-in-hand, a mobile robot, a frame in
   the cell) and use *Add › Camera / vision sensor*. The camera item looks along its **+Z** axis, X right, Y down
   (the optical convention of ROS `camera_optical_frame`).
2. *Tools › Machine vision stack (camera)…* (or *Select vision stack…* in the camera properties) opens the wizard:
   - **Tasks** — detection, tracking, classification, segmentation, keypoints (stems, cutting points),
     6D pose / 3D position, grasp generation, depth estimation, point-cloud detection / segmentation, measurement,
     following, VLM query, VLA policy.
   - **Environment**, **sensor modality** (or *any — recommend*), **compute** (or *any*), working distance,
     required rate, cost budget, class list (also the prompt for open-vocabulary models), 3D needed, night,
     moving vehicle, open vocabulary, connectivity.
   - The ranked list shows for every candidate sensor: the best compute target, one model per task, the depth
     error at the working distance (σ from the sensor's error model), the estimated pipeline latency, reasons
     and warnings (“depth degrades in direct sunlight”, “rolling shutter on a moving vehicle”, “no depth: 3D
     positions only from a size prior”) and the processing pipeline with the ROS message types.
   - **Runtime & endpoints** decide where the models execute (see below).
3. *Apply stack* stores the configuration on the camera (`params.visionStack`, saved with the station), sets the
   camera kind / field of view / resolution from the sensor sheet and opens the **Vision** tab.

## Catalogue

| Group | Entries |
|---|---|
| Sensors | USB / MIPI rolling-shutter RGB, global-shutter GigE (Basler / FLIR), Stereolabs ZED X / 2i, Luxonis OAK-D Pro, Intel RealSense D455 / D435i, Orbbec Gemini / Femto, ToF (Azure Kinect / Basler blaze), Zivid / Photoneo structured light, Velodyne VLP-16, Ouster OS1 / OS0, Livox Mid-360, Hesai XT32, FLIR Boson thermal, MicaSense multispectral, Prophesee event camera — with resolution, FOV, depth range, depth error model σ(z) = a + b·z², shutter, sunlight robustness, night capability, cost, IP rating, ROS 2 driver |
| Compute | Coral / Hailo modules, in-camera Myriad X, Raspberry Pi 5 + Hailo, Jetson Orin Nano / NX / AGX / Thor, x86 + RTX, Intel IPC (OpenVINO), cloud GPU — TOPS, power, runtimes |
| Detection | YOLOv8/11 n·s·m, YOLOv10, RT-DETR / D-FINE, YOLO-World (open vocabulary), Grounding DINO / OWLv2 |
| Segmentation | YOLOv8-seg, FastSAM / MobileSAM, SAM 2 (video), Mask2Former / SegFormer (semantic: soil, row, canopy, path) |
| Classification | EfficientNet / MobileNet, YOLOv8-cls, CLIP / SigLIP (zero-shot) |
| Tracking | ByteTrack (built in), BoT-SORT / DeepSORT (re-ID), OC-SORT |
| Pose / keypoints | YOLOv8-pose (custom keypoints), FoundationPose, MegaPose / CosyPose, DOPE, depth-centroid position, mono size-prior position |
| Depth | Depth Anything V2, RAFT-Stereo / FoundationStereo / Isaac ESS, OpenCV SGBM |
| Point clouds | PCL / Open3D pipeline (voxel → RANSAC ground → Euclidean clusters → rows), PointPillars / CenterPoint, PointNet++ / KPConv, Patchwork++ ground segmentation |
| Grasping | GraspNet / Contact-GraspNet / AnyGrasp, approach-vector grasp (built in) |
| VLM | Florence-2, PaliGemma 2, Qwen2.5-VL (Ollama / vLLM), LLaVA / Moondream / SmolVLM, hosted GPT-4o-class models |
| VLA | OpenVLA / OpenVLA-OFT, π0 / π0-FAST (openpi), Octo, NVIDIA GR00T, ACT / Diffusion Policy (LeRobot) |

Every model entry carries its tasks, inputs (image / depth / point cloud / text / proprioception), parameter
count, latency per compute class, a normalised quality figure, runtimes and formats, licence, source repository,
agriculture suitability and the adapter the studio uses to execute it. The recommender scores models per task by
quality, latency against the required rate, open-vocabulary needs, and modality fit; sensors by depth error at
the working distance, sunlight / night / motion robustness and cost.

## Running the pipeline — the Vision tab

```{image} ../_static/screens/37-vision-tab.png
:alt: Vision tab with detections on the camera image
:class: screenshot
```

*Run* (or *live*) captures a frame from the selected camera and runs the configured tasks:

1. **Capture** — the rendered camera image (for real models) and, in simulation, the ground truth of what the
   camera sees: fruit (from the crop rows, with occlusion), trunks and canopies, objects (bounding boxes), mobile
   robots. Depth cameras get a simulated depth image, 3D LiDARs a simulated point cloud ray-cast against the
   scene (ground, trunks, canopies, object boxes, vehicles) with the sensor's channels, FOV and range noise.
2. **Infer** — one adapter per task (below). The simulated adapter turns ground truth into detections with the
   statistics of the chosen model: recall falls with object size in pixels and with the model's quality, false
   positives appear on background, boxes jitter, ripeness classification is confused proportionally to
   (1 − accuracy), masks and keypoints (stem = top of the fruit) are generated.
3. **Track** — ByteTrack: high/low score association, constant-velocity prediction, stable ids and velocities.
4. **Locate** — 3D position per detection: median depth in the box / mask for stereo, RGB-D and ToF (with the
   sensor's σ(z)), the object-size prior for mono cameras (*z = f·D/w*), cluster centroids for LiDAR; then camera
   → world → robot frame.
5. **Act** — *Targets → station* creates a *grasp* and an *approach* target per detection (tool Z along the
   camera ray, approach 100 mm back) under a “Vision ‹camera›” frame, ready for MoveJ/MoveL in a program.
   *Follow* steers the selected mobile robot to keep the first tracked object centred at the working distance
   (visual servoing for a mobile base; `servoDelta` gives the TCP correction for an arm camera). With a VLA
   configured, *Ask / step* sends the image, the instruction and the joint state to the policy and moves the robot
   TCP by the returned action (scaled mm / deg per step).

The overlay shows boxes, masks, keypoints, track ids and velocities, ranges and ripeness; *truth* draws the ground
truth boxes for comparison. The status line reports precision, recall, mean 3D position error and latency
against the ground truth, so a stack can be compared against another one on the same scene before hardware is
bought. LiDAR / depth clouds appear in the bird's-eye view with ground removed, clusters (trunk, canopy, person,
low obstacle, wall) and fitted crop-row lines; *Export cloud (.pcd)* saves the current cloud.

## Plugging in real models

The **Runtime & endpoints** section of the wizard selects where inference happens; the same configuration
drives the Vision tab, the API and the ROS 2 export.

| Runtime | What runs | Configuration |
|---|---|---|
| Simulated | ground truth + model statistics (default; no weights needed) | seed, confidence, IoU |
| ONNX Runtime Web | any Ultralytics export (`yolo export format=onnx`: YOLOv5/8/10/11 detect, -seg, -pose, -cls; RT-DETR; YOLO-World with the vocabulary baked in) **in the browser** (WebGPU when available, WASM otherwise) | model URL or path served next to the app, input size, class names |
| Studio server | `POST /vision/infer` → `python/vision_infer.py` using **ultralytics** (`.pt/.onnx/.engine`) or **onnxruntime**; `STUDIO_VISION_MODELS` folder, `STUDIO_VISION_URL` to forward to an external inference server (Triton, Roboflow inference, TorchServe) with the same JSON contract | model name / path |
| VLM endpoint | OpenAI-compatible `chat/completions` with the image: Ollama (`qwen2.5vl`, `llava`, `moondream`), vLLM (PaliGemma, Qwen2-VL), hosted APIs. Prompts for detection (JSON boxes in a 0–1000 frame), classification, counting and free questions are built in; answers with `box_2d`, PaliGemma `<loc####>` tokens or JSON labels are parsed into boxes / labels | base URL, model, API key |
| VLA server | openpi (`/infer`, π0), OpenVLA (`/act`), the studio JSON contract (`/act`: LeRobot ACT / Diffusion Policy, custom) → 7-DoF action chunk (Δx Δy Δz Δroll Δpitch Δyaw gripper, normalised) applied in the tool frame | URL, checkpoint, format, instruction, mm / deg per step |
| ROS 2 | `vision_msgs/Detection2DArray` from a rosbridge topic (yolo_ros, Isaac ROS, your own node) | topic |

Detections from any adapter go through the same tracker, 3D localisation and target creation. The HTTP adapter
also understands Roboflow (`predictions[]` with centre/width/height) and Ultralytics JSON, polygons become masks,
`position`/`z` fields become 3D points.

## Point clouds

`.pcd` (ascii / binary) and `.ply` (ascii / binary) files dropped into the studio become objects with a
decimated point display; the full cloud is kept for *Analyse imported cloud* in the Vision tab: voxel
down-sampling, RANSAC ground plane, Euclidean clustering with 3D boxes and shape classes, a trunk slice
(0.2–0.9 m above ground) for row detection, PCA row lines, canopy metrics (volume, height, width). The same
algorithms run on the simulated LiDAR / depth clouds and are exported as a ROS 2 node (`cloud_pipeline.py`,
Open3D with a numpy fallback). Clouds are written back with *Export cloud (.pcd)*; the API exposes them through
`src/vision/pointcloud.ts` (`parsePCD`, `parsePLY`, `writePCD`, `writePLY`, `voxelDownsample`, `fitGroundPlane`,
`euclideanCluster`, `detectRows`, `simulateLidar3D`, `simulateDepthCamera`).

## Camera geometry and calibration

`src/vision/camera_model.ts` provides pinhole intrinsics from the camera item or a sensor sheet, projection /
back-projection, depth error models per modality (stereo σz = z²·σd / (f·B), ToF, LiDAR, mono size prior), the
target pose builder (grasp + approach) and **rigid 3D–3D registration** (Horn's method) for camera-to-robot
calibration: touch N points with the TCP, measure them with the camera, `rigidTransform(cameraPts, robotPts)`
returns the camera pose in the robot base and the RMS residual. The hand-eye pose (eye-in-hand: camera in the
flange; eye-to-hand: camera in the base) is written into the ROS 2 package as a static transform.

## ROS 2 perception package

*Export ROS 2 package* (camera properties, Vision tab or *Tools › Export ROS 2 perception package…*) writes an
`ament_python` package:

- `launch/perception.launch.py` — sensor driver (`realsense2_camera`, `zed_wrapper`, `depthai_ros_driver`,
  `orbbec_camera`, `usb_cam`, `pylon`, `velodyne`, `ouster_ros`, `livox_ros_driver2`, `hesai`…), static hand-eye
  and optical-frame transforms, `yolo_ros` detector / tracker / 3D nodes, the point-cloud pipeline, VLM and VLA
  bridges as configured.
- `config/detector.yaml` (model, task, device, thresholds, input size, classes), `tracker.yaml` (ByteTrack /
  BoT-SORT), `detect_3d.yaml`, `hand_eye.yaml`, `cloud_pipeline.yaml`, `vlm.yaml`, `vla.yaml`.
- `scripts/cloud_pipeline.py` (Open3D / numpy), `scripts/vlm_bridge.py` (asks the VLM periodically or on
  `~/question`, publishes the answer and grounded boxes), `scripts/vla_bridge.py` (policy server →
  `TwistStamped` for `moveit_servo` + gripper command).
- `README.md` with install steps, topics and the hand-eye refinement procedure (`easy_handeye2`).

## API and programs

```python
from robodk.robolink import Robolink
RDK = Robolink()
cam = RDK.Item('Camera 1')
cam.setParam('visionStack', {...})          # same JSON as the wizard writes
```

In the browser console (`app` is the studio): `import('/src/vision/pipeline.ts')` exposes `VisionRuntime`,
`captureTruth`, `createTargetsFromOutput`, `followCommand`, `servoDelta`; `src/vision/models.ts` exposes the
adapters (`OnnxYoloModel`, `HttpModel`, `VlmOpenAIModel`, `VlaHttpModel`, `Ros2VisionModel`, `ByteTracker`) for
custom pipelines and plugins. Programs consume the created targets like any taught target; missions use the
harvest detections through the fleet manager.

## Limits

- Real-model inference in the browser needs the model served over HTTP (CORS) and WebGPU/WASM memory for
  the chosen size; TensorRT engines run only through the server or ROS 2.
- The simulated detector models statistics, not appearance: lighting, motion blur and occlusion by leaves are
  approximated (canopy occlusion probability, pixel-size recall), not rendered.
- 6D orientation from depth uses the camera ray / surface normal; full 6D pose networks (FoundationPose,
  MegaPose) run outside the browser (ROS 2 / HTTP adapters).
- VLM answers take seconds and are meant for decisions, reports and mission planning, not for servo loops.
