# Installation

VerticalBot Studio is a web application. There is nothing to install for the simulation, programming and
export features: open the hosted build in a browser, or run it locally from the repository.

## Requirements

| Component | Requirement |
|---|---|
| Browser | Chrome / Edge / Firefox / Safari with WebGL 2. Chrome or Edge recommended for large stations. |
| Local development | Node.js 20+ and npm |
| Studio server (optional) | Node.js 20+; Python 3.9+ for the `robodk` drop-in package and RoboDK conversion |
| RoboDK conversion (optional) | A RoboDK installation on the server machine and `pip install robodk` |
| Fleet interface (optional) | An MQTT broker (Mosquitto, EMQX, HiveMQ…) reachable from the server |

## Run from the repository

```bash
git clone https://github.com/VerticalBot/vertical_bot_ros.git
cd vertical_bot_ros/studio
npm install
npm run dev            # development server: http://localhost:5173
```

Production build and static hosting:

```bash
npm run build          # -> studio/dist (static files, host them anywhere)
npm run preview        # serve the build locally on http://localhost:4173
```

## The studio server

The server is optional. It adds what a browser cannot do alone: the RoboDK-compatible API for Python / C# /
C++ / MATLAB clients, live robot drivers, the RoboDK file converter and the VDA 5050 fleet interface.

```bash
cd studio
npm run server                                  # ws://localhost:20500 (API), TCP 20501, HTTP endpoints
```

Then open the studio with `?server=ws://localhost:20500` so the browser relays to it, for example
`http://localhost:5173/?server=ws://localhost:20500`.

Environment variables:

| Variable | Meaning |
|---|---|
| `STUDIO_PORT` | WebSocket/HTTP port (default 20500; TCP JSON-lines on port+1) |
| `STUDIO_STATION` | `.vbstation` to load at start (headless simulation) |
| `STUDIO_ROBODK_PYTHON` | Python with the `robodk` package next to a RoboDK installation — enables `.rdk` conversion |
| `STUDIO_MQTT_URL`, `STUDIO_VDA_PREFIX`, `STUDIO_VDA_ROLE`, `STUDIO_VDA_MANUFACTURER` | Auto-connect the VDA 5050 interface at start |

## Python API package

```bash
pip install robodk            # optional: the official package; the studio is API-compatible
# or use the drop-in package shipped with the studio:
export PYTHONPATH=$PWD/studio/python
python studio/python/examples/hello_studio.py
```

See {doc}`../api/robolink`.

## Blender add-on

Install `studio/blender/vertical_bot_studio.py` through *Edit › Preferences › Add-ons › Install…* in
Blender 3.x/4.x. See {doc}`../interop/blender-ros-vc`.
