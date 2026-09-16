# Integration: ROS 2, custom nodes, protocols and ports

This page is the wiring diagram of the studio: which process speaks which protocol on which port, what the
messages look like, and how to connect ROS 2 nodes, your own (non-ROS) nodes, model servers and fleet software
to the navigation and machine-vision stacks. Everything listed is implemented and covered by tests
(`tests/vision_protocols.test.ts`, `tests/vda5050*.test.ts`, `tests/scenarios.test.ts`).

## Processes and ports

```
 browser (studio UI, simulation, perception runtime)
   │  ws://host:20500  (relay: the studio server forwards API calls to this tab)
   │  ws://host:9090   rosbridge_server  ──►  ROS 2 graph (topics below)
   │  http  VLM / VLA / inference endpoints (direct from the browser, CORS)
   ▼
 studio server  (npm run server, Node.js)                 ROS 2 machine / robot
   20500/tcp  WebSocket JSON-RPC  RoboDK-compatible API    9090   rosbridge websocket
   20501/tcp  TCP JSON-lines      same API (MATLAB, PLC)   yolo_ros / Nav2 / drivers: native DDS
   20500/tcp  HTTP                /health /station.json     
                                  /convert/rdk             model servers
                                  /vda5050/*  (MQTT bridge) 11434  Ollama (VLM, OpenAI-compatible)
                                  /vision/infer /vision/models 8000 vLLM · openpi policy server
   → 1883/8883 MQTT               VDA 5050 broker           8500  custom inference (example)
   → 30001-30003, 7000, 80/443    robot controllers (UR, KUKA KVP, ABB RWS)
```

| Port | Protocol | Who listens | Purpose |
|---|---|---|---|
| **20500** | WebSocket, JSON-RPC (`{id, method, params}`) | studio server (`STUDIO_PORT`) | RoboDK-compatible API (`Robolink` from Python/C#/C++/MATLAB), relay to the browser tab opened with `?server=ws://host:20500` |
| **20501** | TCP, newline-delimited JSON, same objects | studio server | API for tools without WebSocket (MATLAB `tcpclient`, PLCs) |
| **20500** | HTTP | studio server | `GET /health`, `GET/POST /station.json`, `POST /convert/rdk`, `GET/POST /vda5050/*`, `POST /vision/infer`, `GET /vision/models` |
| **9090** | WebSocket, rosbridge v2 JSON (`op: advertise / publish / subscribe / call_service`) | `rosbridge_server` on the ROS 2 machine | browser ↔ ROS 2 topics (twin, navigation, perception) |
| **1883 / 8883** | MQTT (TLS on 8883) | your broker (Mosquitto, EMQX, KUKA Fleet) | VDA 5050 `order` / `state` / `instantActions` / `connection` / `factsheet` |
| **11434** | HTTP, OpenAI `chat/completions` | Ollama | VLM queries (`qwen2.5vl`, `llava`, `moondream`) |
| **8000** | HTTP | vLLM (`/v1`), openpi policy server (`/infer`), OpenVLA server (`/act`) | VLM / VLA endpoints configured in the vision wizard |
| **8500** | HTTP | `python/examples/custom_inference_server.py` | your own detector behind the studio inference contract |
| **8765** | HTTP | `python/examples/vision_webhook_sink.py` | receives the JSON summary of every perception run (webhook) |
| 30001–30003 · 7000 · 80/443 · 9090 | vendor | UR · KUKA KVP · ABB RWS · ROS 2 | robot drivers on the server ({doc}`../api/drivers`) |

Security: nothing above authenticates. Keep the server, rosbridge and model endpoints on a trusted network or
behind a reverse proxy with TLS and access control.

## Three ways to get data out of (and into) the studio

| Path | Transport | Best for |
|---|---|---|
| **ROS 2 topics** through rosbridge | `Connect › ROS 2 via rosbridge…` (browser → `ws://ros-host:9090`) | RViz, Nav2, MoveIt, `vision_msgs` consumers, recording bags |
| **API params** (`visionLast`, `navEstimate`) | Robolink `Item.getParam()` over 20500/20501 | Python/C#/MATLAB supervisors without ROS, PLC gateways, tests |
| **Webhook** | HTTP POST JSON to a URL you set in the vision wizard | custom nodes, MQTT/OPC UA bridges, cloud dashboards |

Into the studio: `subscribe` on rosbridge (`/joint_states`, `/odom` mirror real robots), `vision_msgs`
detections from a real detector (`Runtime: ROS 2 topic`), HTTP inference / VLM / VLA answers, VDA 5050 `state`
messages from real AGVs, driver feedback (joint states).

## ROS 2 topics

Enable the twin with *Connect › ROS 2 via rosbridge…* (`rate` Hz, mode publish / follow / both). Names are
prefixed by the item's ROS namespace (robot properties). Message types are ROS 2 (`pkg/msg/Type`).

### Arms and mobile bases (existing twin)

| Topic | Type | Direction |
|---|---|---|
| `<ns>/cmd_joint_state` | `sensor_msgs/msg/JointState` | studio → ROS (simulated joints, rad / m) |
| `<ns>/tcp_pose` | `geometry_msgs/msg/PoseStamped` | studio → ROS (TCP in `base_link`, m) |
| `<ns>/cmd_point` | `std_msgs/msg/Float32MultiArray` | studio → ROS (TCP xyz, m) |
| `<ns>/cmd_vel` · `<ns>/robot_pose` · `<ns>/battery_state` | `geometry_msgs/msg/Twist` · `geometry_msgs/msg/Pose` · `sensor_msgs/msg/BatteryState` | studio → ROS (mobile robots) |
| `/joint_states` · `/odom` | `sensor_msgs/msg/JointState` · `nav_msgs/msg/Odometry` | ROS → studio (the simulation follows the real robot) |

### Navigation & SLAM stack (per mobile robot with a simulated stack)

| Topic | Type | Content |
|---|---|---|
| `<ns>/odom_estimate` | `nav_msgs/msg/Odometry` | the localization **estimate** (frame `map`, child `base_link`), covariance from the current error |
| `<ns>/ground_truth` | `nav_msgs/msg/Odometry` | the true simulated pose |
| `<ns>/scan` | `sensor_msgs/msg/LaserScan` | simulated 2D LiDAR (frame `laser`) |
| `<ns>/slam_map` | `nav_msgs/msg/OccupancyGrid` | incremental SLAM map (`-1` unknown, 0–100), every 2 s |

Compare estimate and truth in RViz, feed `odom_estimate` to your own EKF or Nav2 (`robot_localization`
`odom0`), or record bags to replay localisation failures. The exported navigation package
({doc}`../mobile/navigation-slam`) contains the real stack (Nav2 + SLAM + EKF) with matching frame names.

### Machine vision (per camera with "publish to ROS 2" enabled in the wizard)

| Topic | Type | Content |
|---|---|---|
| `<ns>/vision/<camera>/detections` | `vision_msgs/msg/Detection2DArray` | boxes (pixels), `class_id`, `score`, track id in `id` |
| `<ns>/vision/<camera>/detections_3d` | `vision_msgs/msg/Detection3DArray` | 3D positions in `map` (m), box size |
| `<ns>/vision/<camera>/targets` | `geometry_msgs/msg/PoseArray` | grasp poses (`map`), same order as the detections with a position |
| `<ns>/vision/<camera>/cloud` | `sensor_msgs/msg/PointCloud2` | float32 `x y z [intensity]`, little-endian, base64 in rosbridge |
| `<ns>/vision/<camera>/image/compressed` | `sensor_msgs/msg/CompressedImage` | the rendered camera image (jpeg) |
| `<ns>/vision/<camera>/vlm/answer` | `std_msgs/msg/String` | VLM answer text |

These are the same topic names and message layouts the exported ROS 2 perception package produces from real
sensors (`yolo_ros` `detections` / `detections_3d`, `cloud_pipeline.py`, `vlm_bridge.py`), so a consumer node
written against the simulation runs unchanged on the robot.

```python
# python/examples/ros2_vision_consumer.py — nearest detection of a class → /target_pose (PoseStamped)
ros2 run rosbridge_server rosbridge_websocket           # on the ROS 2 machine, port 9090
python3 python/examples/ros2_vision_consumer.py --camera camera_1 --class apple
```

## JSON contracts (HTTP)

### Inference server — `POST /vision/infer`

Used by the browser runtime *Studio server*, by the studio server itself (forwarding to `STUDIO_VISION_URL`)
and by anything else that wants to plug a model in.

```json
{"model": "yolov8n.pt | path/model.onnx | vlm:qwen2.5vl", "task": "detect|segment|keypoints|classify",
 "image": "data:image/jpeg;base64,...", "confidence": 0.4, "iou": 0.5, "classes": ["apple", "trunk"]}
```

```json
{"detections": [{"x": 12, "y": 40, "w": 80, "h": 60, "score": 0.91, "class": "apple",
                 "track_id": 3, "keypoints": [[52, 40, 0.9]], "points": [[12, 40], [92, 40], [92, 100]],
                 "position": [1230, -80, 410]}],
 "labels": [{"class": "ripe", "score": 0.8}], "text": "optional VLM text", "backend": "ultralytics", "latencyMs": 18}
```

Also accepted by the studio: Roboflow (`predictions[{x, y, width, height, class, confidence}]`, centre-based) and
Ultralytics JSON (`[{name, confidence, box:{x1,y1,x2,y2}}]`). `python/vision_infer.py` implements the contract with
ultralytics / onnxruntime; `python/examples/custom_inference_server.py` is a 60-line template for your own model.

### VLM — OpenAI-compatible `POST <url>/chat/completions`

```json
{"model": "qwen2.5vl:7b", "temperature": 0, "max_tokens": 800,
 "messages": [{"role": "user", "content": [{"type": "text", "text": "Detect every apple ... JSON ... 0-1000 frame"},
                                            {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,..."}}]}]}
```

`Authorization: Bearer <key>` when an API key is set. Answers are parsed from `choices[0].message.content`: JSON
`detections[{label, box_2d:[x1,y1,x2,y2], confidence, polygon?}]` (0–1000 or pixel frame), `labels[]`,
`counts{}`, PaliGemma `<loc####>` tokens, or free text.

### VLA — policy servers

| Format | Request | Response |
|---|---|---|
| `openpi` (π0 / π0-FAST) | `POST <url>/infer` `{"observation": {"image": dataURL, "state": [joints…, gripper], "prompt": "pick the apple"}}` | `{"actions": [[dx,dy,dz,droll,dpitch,dyaw,gripper], …]}` |
| `openvla` | `POST <url>/act` `{"image": dataURL, "instruction": "...", "unnorm_key": "<dataset>"}` | `{"action": [7 values]}` |
| `studio` (LeRobot ACT / Diffusion Policy / custom) | `POST <url>/act` `{"model": "...", "image": dataURL, "instruction": "...", "proprio": {"joints": [...], "tcp": [16], "gripper": 0}}` | `{"action": [...]}` or `{"actions": [[...], …]}` |

Actions are normalised to [-1, 1] and scaled by the wizard's mm / deg per step; deltas are applied in the tool
(camera) frame. The exported `vla_bridge.py` does the same on the robot and publishes `TwistStamped` for
`moveit_servo`.

### Webhook — `POST <your url>` after every perception run

The body is the JSON summary also stored in `camera.params.visionLast`:

```text
{"camera": "Camera 1", "time": 12.4, "frame": "map", "units": "mm",
 "detections": [{"cls": "apple", "score": 0.87, "id": 5, "box": [312, 140, 22, 22], "z": 1480, "p": [45210, 9812, 1710], "attr": {"ripe": 1}}],
 "targets": [{"name": "apple#5", "cls": "apple", "p": [45210, 9812, 1710], "grasp": [16 column-major], "approach": [16], "sigmaMm": 19}],
 "cloud": {"points": 8123, "clusters": [{"id": 1, "shape": "trunk", "centroid": [...], "size": [...], "points": 40}], "rows": [{"point": [x, y], "dir": [dx, dy], "length": 40000}]},
 "text": null, "vla": null, "warnings": []}
```

`python/examples/vision_webhook_sink.py` receives it (port 8765) — replace its `do_POST` with your MQTT / OPC UA /
database code.

### API params — Robolink `getParam`

```python
from robodk.robolink import Robolink
RDK = Robolink()                                   # ws://localhost:20500 (or ROBODK_URL); TCP 20501 also works
last = RDK.Item('Camera 1').getParam('visionLast')    # dict as above
est = RDK.Item('Harvest platform 1').getParam('navEstimate')
# {'x','y','theta','error','rmse','maxError','lost','lostEvents','fixes','gnss','distanceSinceFix','scale','method','truth':{...},'slamCoverage'}
```

`python/examples/robolink_poll_vision_nav.py` polls both. `setParam('visionStack', {...})` /
`setParam('navStack', {...})` configure the stacks from a script with the same JSON the wizards write.

## VDA 5050 (fleet software)

MQTT topics `<prefix>/<version>/<manufacturer>/<serial>/{order,state,instantActions,connection,factsheet}`, JSON per
VDA 5050 v2. The studio server is the **master** (dispatches orders to real AGVs and mirrors their `state`) or the
**AGV bridge** (each simulated robot appears as a vehicle to KUKA Fleet / MiR Fleet / your master). HTTP control on
the server: `POST /vda5050/connect`, `GET /vda5050/status`, `POST /vda5050/order`, `POST /vda5050/instantAction`
— see {doc}`../mobile/vda5050`.

## Bringing your own node

1. **ROS 2 node** — subscribe to the topics above (simulation) and to the same names from the exported
   perception / navigation packages (hardware). Example: `python/examples/ros2_vision_consumer.py`.
2. **Custom node without ROS** — either receive the webhook (`vision_webhook_sink.py`), poll the API params
   (`robolink_poll_vision_nav.py`), or talk to the server's JSON-RPC directly:
   ```json
   {"id": 1, "method": "Item", "params": ["Camera 1"]}                → {"id": 1, "result": {"$item": "<item id>"}}
   {"id": 2, "method": "getParam", "params": ["visionLast"], "target": "<item id>"}
   ```
   (`target` selects the item; one line per request on TCP 20501, the same object as a WebSocket text frame on
   20500 — see `src/api/rpc.ts` and `python/robodk/robolink.py` for the full method list).
3. **Your own model** — implement `/vision/infer` (`custom_inference_server.py`), export
   `STUDIO_VISION_URL=http://host:8500/vision/infer` for the studio server, select *Runtime: Studio server* in the
   wizard. Or export the model to ONNX and select *ONNX Runtime Web* to run it in the browser.
4. **Your own policy** — serve `/infer` or `/act` (formats above) and select it under *VLA policy server*.
5. **Fleet / MES** — VDA 5050 over MQTT for vehicles; Robolink API (`RDK.Command`, programs, `setParam`) for the
   cell.

## Verification

- `npm test` — 157 unit and integration tests, including mock servers for every HTTP contract on this page
  (`tests/vision_protocols.test.ts`), an embedded MQTT broker for VDA 5050 and a real `onnxruntime-web` run.
- `npm run scenarios` — the demo scenarios of {doc}`../reference/scenarios` (every localization method, every
  vision task) with the published results.
- `python3 python/vision_infer.py --check` — which inference backends the server machine has.
