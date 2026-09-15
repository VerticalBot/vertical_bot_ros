"""robolink drop-in for VerticalBot Studio (RoboDK API compatible subset).

    from robodk.robolink import *
    from robodk.robomath import *
    RDK = Robolink()                       # connects to ws://localhost:20500
    robot = RDK.Item('', ITEM_TYPE_ROBOT)
    robot.MoveJ([0, -90, 90, -90, -90, 0])
    prog = RDK.AddProgram('Hello', robot)
    prog.MoveJ(RDK.AddTarget('T1', robot.Parent(), robot))

Transport: WebSocket JSON-RPC. Uses the `websocket-client` package if installed, otherwise a
tiny built-in RFC 6455 client (no dependencies).
"""
import json
import os
import socket
import struct
import threading
import base64
import itertools

from .robomath import Mat

# ---- constants (RoboDK values) -----------------------------------------------------------
ITEM_TYPE_ANY = -1
ITEM_TYPE_STATION = 1
ITEM_TYPE_ROBOT = 2
ITEM_TYPE_FRAME = 3
ITEM_TYPE_TOOL = 4
ITEM_TYPE_OBJECT = 5
ITEM_TYPE_TARGET = 6
ITEM_TYPE_CURVE = 7
ITEM_TYPE_PROGRAM = 8
ITEM_TYPE_INSTRUCTION = 9
ITEM_TYPE_PROGRAM_PYTHON = 10
ITEM_TYPE_MACHINING = 11
ITEM_TYPE_FOLDER = 17
ITEM_TYPE_ROBOT_ARM = 18
ITEM_TYPE_CAMERA = 19
ITEM_TYPE_MOBILE_ROBOT = 100

INSTRUCTION_CALL_PROGRAM = 0
INSTRUCTION_INSERT_CODE = 1
INSTRUCTION_START_THREAD = 2
INSTRUCTION_COMMENT = 3
INSTRUCTION_SHOW_MESSAGE = 4

RUNMODE_SIMULATE = 1
RUNMODE_QUICKVALIDATE = 2
RUNMODE_MAKE_ROBOTPROG = 3
RUNMODE_RUN_ROBOT = 6

ROBOTCOM_READY = 2
ROBOTCOM_DISCONNECTED = 0


class _MiniWebSocket:
    """Minimal RFC 6455 client (text frames only) used when websocket-client is missing."""

    def __init__(self, url, timeout=30):
        assert url.startswith("ws://"), "only ws:// is supported by the built-in client"
        rest = url[5:]
        hostport, _, path = rest.partition("/")
        host, _, port = hostport.partition(":")
        self.sock = socket.create_connection((host, int(port or 80)), timeout=timeout)
        key = base64.b64encode(os.urandom(16)).decode()
        req = (f"GET /{path} HTTP/1.1\r\nHost: {hostport}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
               f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n")
        self.sock.sendall(req.encode())
        resp = b""
        while b"\r\n\r\n" not in resp:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("websocket handshake failed")
            resp += chunk
        if b" 101 " not in resp.split(b"\r\n")[0]:
            raise ConnectionError("websocket handshake rejected: " + resp.decode(errors="ignore")[:200])
        self.buf = resp.split(b"\r\n\r\n", 1)[1]

    def send(self, text):
        data = text.encode()
        header = bytearray([0x81])
        n = len(data)
        if n < 126:
            header.append(0x80 | n)
        elif n < 65536:
            header.append(0x80 | 126)
            header += struct.pack(">H", n)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", n)
        mask = os.urandom(4)
        header += mask
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        self.sock.sendall(bytes(header) + masked)

    def _read(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("websocket closed")
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def recv(self):
        while True:
            b1, b2 = self._read(2)
            opcode = b1 & 0x0F
            n = b2 & 0x7F
            if n == 126:
                n = struct.unpack(">H", self._read(2))[0]
            elif n == 127:
                n = struct.unpack(">Q", self._read(8))[0]
            mask = self._read(4) if b2 & 0x80 else None
            payload = self._read(n)
            if mask:
                payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
            if opcode == 0x1:
                return payload.decode()
            if opcode == 0x9:  # ping -> pong
                self.sock.sendall(bytes([0x8A, 0x80]) + os.urandom(4))
            if opcode == 0x8:
                raise ConnectionError("websocket closed by server")

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def _connect(url):
    try:
        import websocket  # type: ignore

        ws = websocket.create_connection(url, timeout=60)
        return ws
    except ImportError:
        return _MiniWebSocket(url)


class Robolink:
    """Connection to VerticalBot Studio (browser via relay, or headless server)."""

    def __init__(self, robodk_ip="localhost", port=20500, args=None, robodk_path=None, close_std_out=False, quit_on_close=False, com_object=None, skipstatus=False):
        url = os.environ.get("STUDIO_URL") or robodk_ip
        if not url.startswith("ws"):
            url = f"ws://{robodk_ip}:{port}"
        self.url = url
        self._ws = _connect(url)
        self._lock = threading.Lock()
        self._ids = itertools.count(1)
        self.run_mode = RUNMODE_SIMULATE

    # -- transport -------------------------------------------------------------
    def _call(self, method, params=None, target=None):
        req = {"id": next(self._ids), "method": method, "params": [_encode(p) for p in (params or [])], "target": target}
        with self._lock:
            self._ws.send(json.dumps(req))
            while True:
                res = json.loads(self._ws.recv())
                if res.get("id") == req["id"]:
                    break
        if res.get("error"):
            raise Exception(res["error"])
        return _decode(self, res.get("result"))

    def Disconnect(self):
        self._ws.close()

    Finish = Disconnect

    # -- station -----------------------------------------------------------------
    def Item(self, name, itemtype=ITEM_TYPE_ANY):
        return self._call("Item", [name, itemtype])

    def ItemList(self, filter=ITEM_TYPE_ANY, list_names=False):
        return self._call("ItemList", [filter, list_names])

    def ItemUserPick(self, message="Pick one item", itemtype=ITEM_TYPE_ANY):
        return self._call("ItemUserPick", [message, itemtype])

    def ActiveStation(self):
        return self._call("ActiveStation")

    def AddStation(self, name="New Station"):
        return self._call("AddStation", [name])

    def AddFrame(self, name, itemparent=None):
        return self._call("AddFrame", [name, itemparent])

    def AddFolder(self, name, itemparent=None):
        return self._call("AddFolder", [name, itemparent])

    def AddTarget(self, name, itemparent=None, itemrobot=None):
        return self._call("AddTarget", [name, itemparent, itemrobot])

    def AddProgram(self, name, itemrobot=None):
        return self._call("AddProgram", [name, itemrobot])

    def AddRobot(self, library_name, itemparent=None):
        return self._call("AddRobot", [library_name, itemparent])

    def AddMobileRobot(self, name):
        return self._call("AddMobileRobot", [name])

    def AddFile(self, filename, parent=None):
        return self._call("AddFile", [filename, parent])

    def AddShape(self, triangle_points, add_to=None, override_shapes=False, name="Shape"):
        return self._call("AddShape", [triangle_points, add_to, name])

    def AddCurve(self, curve_points, reference_object=None, add_to_ref=False, projection_type=0):
        return self._call("AddCurve", [curve_points, reference_object, add_to_ref])

    def AddPoints(self, points, reference_object=None, add_to_ref=False, projection_type=0):
        return self._call("AddPoints", [points, reference_object])

    def ShowMessage(self, message, popup=True):
        return self._call("ShowMessage", [message, popup])

    def Render(self, always_render=True):
        return self._call("Render", [always_render])

    def Update(self):
        return self._call("Update")

    def setRunMode(self, run_mode=RUNMODE_SIMULATE):
        self.run_mode = run_mode
        return self._call("setRunMode", [run_mode])

    def RunMode(self):
        return self._call("RunMode")

    def setSimulationSpeed(self, speed):
        return self._call("setSimulationSpeed", [speed])

    def SimulationSpeed(self):
        return self._call("SimulationSpeed")

    def Save(self, filename, itemsave=None):
        return self._call("Save", [filename, itemsave])

    def getParam(self, param="PATH_OPENSTATION"):
        return self._call("getParam", [param])

    def setParam(self, param, value):
        return self._call("setParam", [param, value])

    def Command(self, cmd, value=""):
        return self._call("Command", [cmd, value])

    def Version(self):
        return self._call("Version")

    def License(self):
        return self._call("License")

    def Selection(self):
        return self._call("Selection")

    def setSelection(self, list_items):
        return self._call("setSelection", [list_items])

    def Collisions(self):
        return self._call("Collisions")

    def Delete(self, item_list):
        return self._call("Delete", [item_list])

    def Cam2D_Snapshot(self, file_save_img="", cam_handle=None):
        return self._call("Cam2D_Snapshot", [file_save_img, cam_handle])

    def App(self, path, *args):
        """VerticalBot extension: call an application function (e.g. 'startWorld')."""
        return self._call("__app__", [path, *args])


class Item:
    """Proxy to a station item (robot, frame, target, program, tool, object...)."""

    def __init__(self, link, item_id, name="", itemtype=-1):
        self.link = link
        self.item = item_id
        self._name = name
        self._type = itemtype

    def __repr__(self):
        return f"Item({self._name!r}, {self._type})"

    def __eq__(self, other):
        return isinstance(other, Item) and other.item == self.item

    def __hash__(self):
        return hash(self.item)

    def _c(self, method, *params):
        return self.link._call(method, list(params), target=self.item)

    def RDK(self):
        return self.link

    def Valid(self, check_deleted=False):
        return self.item is not None and self._c("Valid")

    def Name(self):
        return self._c("Name")

    def setName(self, name):
        self._name = name
        return self._c("setName", name)

    def Type(self):
        return self._c("Type")

    def Parent(self):
        return self._c("Parent")

    def Childs(self):
        return self._c("Childs")

    def Delete(self):
        return self._c("Delete")

    def Visible(self):
        return self._c("Visible")

    def setVisible(self, visible, visible_frame=None):
        return self._c("setVisible", visible)

    def setParent(self, parent):
        return self._c("setParent", parent)

    def setParentStatic(self, parent):
        return self._c("setParentStatic", parent)

    def Pose(self):
        return self._c("Pose")

    def setPose(self, pose):
        return self._c("setPose", pose)

    def PoseAbs(self):
        return self._c("PoseAbs")

    def setPoseAbs(self, pose):
        return self._c("setPoseAbs", pose)

    def PoseTool(self):
        return self._c("PoseTool")

    def setPoseTool(self, tool):
        return self._c("setPoseTool", tool)

    def PoseFrame(self):
        return self._c("PoseFrame")

    def setPoseFrame(self, frame):
        return self._c("setPoseFrame", frame)

    def Joints(self):
        return self._c("Joints")

    def setJoints(self, joints):
        return self._c("setJoints", list(joints))

    def JointsHome(self):
        return self._c("JointsHome")

    def JointLimits(self):
        return self._c("JointLimits")

    def setJointLimits(self, lower, upper):
        return self._c("setJointLimits", list(lower), list(upper))

    def SolveFK(self, joints, tool=None, reference=None):
        return self._c("SolveFK", list(joints))

    def SolveIK(self, pose, joints_approx=None, tool=None, reference=None):
        return self._c("SolveIK", pose, list(joints_approx) if joints_approx else None)

    def SolveIK_All(self, pose, tool=None, reference=None):
        return self._c("SolveIK_All", pose)

    def Connect(self, robot_ip=""):
        return self._c("Connect", robot_ip)

    def ConnectedState(self):
        return self._c("ConnectedState")

    def setSpeed(self, speed_linear, speed_joints=-1, accel_linear=-1, accel_joints=-1):
        return self._c("setSpeed", speed_linear, speed_joints, accel_linear, accel_joints)

    def setRounding(self, rounding_mm):
        return self._c("setRounding", rounding_mm)

    setZoneData = setRounding

    def MoveJ(self, target, blocking=True):
        return self._c("MoveJ", target, blocking)

    def MoveL(self, target, blocking=True):
        return self._c("MoveL", target, blocking)

    def MoveC(self, target1, target2, blocking=True):
        return self._c("MoveC", target1, target2)

    def Pause(self, time_ms=-1):
        return self._c("Pause", time_ms)

    def setDO(self, io_var, io_value):
        return self._c("setDO", str(io_var), io_value)

    def waitDI(self, io_var, io_value, timeout_ms=-1):
        return self._c("waitDI", str(io_var), io_value, timeout_ms)

    def RunInstruction(self, code, run_type=INSTRUCTION_CALL_PROGRAM):
        return self._c("RunInstruction", code, run_type)

    RunCodeCustom = RunInstruction

    def ShowInstructions(self, show=True):
        return self._c("ShowInstructions", show)

    def ShowTargets(self, show=True):
        return self._c("ShowTargets", show)

    def InstructionCount(self):
        return self._c("InstructionCount")

    def Instruction(self, ins_id=-1):
        r = self._c("Instruction", ins_id)
        return r["name"], r["type"], r["moveType"], r["isJointTarget"], r["pose"], r["joints"]

    def InstructionList(self):
        return self._c("InstructionList")

    def InstructionDelete(self, ins_id=0):
        return self._c("InstructionDelete", ins_id)

    def setRobot(self, robot):
        return self._c("setRobot", robot)

    def getLink(self, type_linked=ITEM_TYPE_ROBOT):
        return self._c("getLink", type_linked)

    def AddTool(self, tool_pose, tool_name="New TCP"):
        return self._c("AddTool", tool_pose, tool_name)

    def AddFrame(self, name):
        return self._c("AddFrame", name)

    def AddTarget(self, name):
        return self._c("AddTarget", name)

    def setAsCartesianTarget(self):
        return self._c("setAsCartesianTarget")

    def setAsJointTarget(self):
        return self._c("setAsJointTarget")

    def isJointTarget(self):
        return self._c("isJointTarget")

    def Update(self, check_collisions=0, timeout_sec=3600, mm_step=-1, deg_step=-1):
        return tuple(self._c("Update"))

    def RunProgram(self, prog_parameters=None):
        return self._c("RunProgram")

    RunCode = RunProgram

    def MakeProgram(self, folder_path="", run_mode=RUNMODE_MAKE_ROBOTPROG, post=None):
        return tuple(self._c("MakeProgram", folder_path, post))

    def setParam(self, param, value=""):
        return self._c("setParam", param, value)

    def getParam(self, param):
        return self._c("getParam", param)

    def setColor(self, tocolor, fromcolor=None, tolerance=0.1):
        return self._c("setColor", tocolor)

    Recolor = setColor

    def setGeometryPose(self, pose):
        return self._c("setGeometryPose", pose)

    def Busy(self):
        return self._c("Busy")

    def Stop(self):
        return self._c("Stop")

    def WaitMove(self, timeout=360000):
        return self._c("WaitMove")

    def DOF(self):
        return self._c("DOF")


# ---- (de)serialisation ------------------------------------------------------------
def _encode(v):
    if isinstance(v, Item):
        return {"$item": v.item}
    if isinstance(v, Mat):
        return {"$pose": v.tolist()}
    if isinstance(v, (list, tuple)):
        return [_encode(x) for x in v]
    if isinstance(v, dict):
        return {k: _encode(x) for k, x in v.items()}
    return v


def _decode(link, v):
    if isinstance(v, dict):
        if "$item" in v:
            return Item(link, v["$item"], v.get("name", ""), v.get("type", -1))
        if "$pose" in v:
            return Mat(v["$pose"])
        return {k: _decode(link, x) for k, x in v.items()}
    if isinstance(v, list):
        return [_decode(link, x) for x in v]
    return v
