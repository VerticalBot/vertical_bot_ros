# Studio server

`npm run server` in `studio/` starts a Node.js process that:

- serves the **RoboDK-compatible API** on WebSocket JSON-RPC (`ws://host:20500`) and TCP JSON-lines (port
  20501) for Python, C#, C++ and MATLAB clients;
- either **relays** to a browser session (open the studio with `?server=ws://host:20500` — the browser is the
  “RoboDK window”, scripts drive it live) or runs a **headless station** (`STUDIO_STATION=file.vbstation`) for
  CI, batch post-processing and fleet simulations without a browser;
- hosts **live robot drivers** ({doc}`drivers`), the **RoboDK file converter** (`POST /convert/rdk`, needs a
  RoboDK installation with `STUDIO_ROBODK_PYTHON`) and the **VDA 5050** interface (`/vda5050/*`);
- exposes `GET /health`, `GET/POST /station.json` (read or replace the headless station).

## Python example

```python
from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
RDK = Robolink()                       # ws://localhost:20500 by default (ROBODK_URL / ROBODK_HOST overrides)
robot = RDK.Item('', ITEM_TYPE_ROBOT)
print(robot.Name(), robot.Joints().list())
prog = RDK.AddProgram('FromPython', robot)
prog.MoveJ(robot.Joints())
```

Set `RDK.setRunMode(RUNMODE_RUN_ROBOT)` and connect a driver to execute moves on the real controller.

## Deployment

The server has no authentication: run it on a trusted network or behind a reverse proxy with TLS and access
control. A Docker image and auth are on the {doc}`../roadmap`.
