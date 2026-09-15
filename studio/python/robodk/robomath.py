"""Minimal robomath compatible with robodk.robomath (poses as Mat, angles in degrees at the API)."""
import math

pi = math.pi


class Mat:
    """4x4 homogeneous matrix stored as a list of rows (like robodk.robomath.Mat)."""

    def __init__(self, rows=None, ncols=None):
        if rows is None:
            self.rows = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
        elif isinstance(rows, Mat):
            self.rows = [list(r) for r in rows.rows]
        elif isinstance(rows, int) and ncols is not None:
            self.rows = [[0.0] * ncols for _ in range(rows)]
        else:
            self.rows = [list(map(float, r)) for r in rows]

    # -- basic ops -------------------------------------------------------
    def __mul__(self, other):
        if isinstance(other, Mat):
            n, m, k = len(self.rows), len(other.rows[0]), len(other.rows)
            out = [[sum(self.rows[i][t] * other.rows[t][j] for t in range(k)) for j in range(m)] for i in range(n)]
            return Mat(out)
        if isinstance(other, (list, tuple)):
            v = list(other) + [1.0] * (4 - len(other))
            r = [sum(self.rows[i][j] * v[j] for j in range(4)) for i in range(4)]
            return r[: len(other)]
        return Mat([[x * other for x in r] for r in self.rows])

    def __getitem__(self, idx):
        if isinstance(idx, tuple):
            return self.rows[idx[0]][idx[1]]
        return self.rows[idx]

    def __setitem__(self, idx, value):
        if isinstance(idx, tuple):
            self.rows[idx[0]][idx[1]] = value
        else:
            self.rows[idx] = list(value)

    def __repr__(self):
        return "Mat(\n" + "\n".join("  [" + ", ".join("%9.3f" % v for v in r) + "]" for r in self.rows) + "\n)"

    def tolist(self):
        return [list(r) for r in self.rows]

    def tr(self):
        return Mat([[self.rows[j][i] for j in range(len(self.rows))] for i in range(len(self.rows[0]))])

    def inv(self):
        R = [[self.rows[j][i] for j in range(3)] for i in range(3)]
        t = [self.rows[i][3] for i in range(3)]
        nt = [-sum(R[i][j] * t[j] for j in range(3)) for i in range(3)]
        return Mat([[R[0][0], R[0][1], R[0][2], nt[0]], [R[1][0], R[1][1], R[1][2], nt[1]], [R[2][0], R[2][1], R[2][2], nt[2]], [0, 0, 0, 1]])

    def Pos(self):
        return [self.rows[0][3], self.rows[1][3], self.rows[2][3]]

    def setPos(self, p):
        for i in range(3):
            self.rows[i][3] = p[i]
        return self

    def VX(self):
        return [self.rows[i][0] for i in range(3)]

    def VY(self):
        return [self.rows[i][1] for i in range(3)]

    def VZ(self):
        return [self.rows[i][2] for i in range(3)]

    def isHomogeneous(self):
        return len(self.rows) == 4 and len(self.rows[0]) == 4

    def Offset(self, x, y, z, rx=0, ry=0, rz=0):
        return self * TxyzRxyz_2_Pose([x, y, z, rx * pi / 180, ry * pi / 180, rz * pi / 180])

    def Rot33(self):
        return [r[:3] for r in self.rows[:3]]


def eye(n=4):
    return Mat()


def transl(x, y=None, z=None):
    if isinstance(x, (list, tuple)):
        x, y, z = x
    m = Mat()
    m.rows[0][3], m.rows[1][3], m.rows[2][3] = x, y, z
    return m


def rotx(rx):
    c, s = math.cos(rx), math.sin(rx)
    return Mat([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])


def roty(ry):
    c, s = math.cos(ry), math.sin(ry)
    return Mat([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])


def rotz(rz):
    c, s = math.cos(rz), math.sin(rz)
    return Mat([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def TxyzRxyz_2_Pose(v):
    """[x,y,z,rx,ry,rz] (radians) -> pose = transl * rotx * roty * rotz."""
    return transl(v[0], v[1], v[2]) * rotx(v[3]) * roty(v[4]) * rotz(v[5])


def Pose_2_TxyzRxyz(H):
    r02 = H[0, 2]
    if abs(r02) < 1 - 1e-9:
        b = math.asin(r02)
        a = math.atan2(-H[1, 2], H[2, 2])
        c = math.atan2(-H[0, 1], H[0, 0])
    else:
        b = math.pi / 2 if r02 > 0 else -math.pi / 2
        a = math.atan2(H[1, 0], H[1, 1])
        c = 0.0
    return [H[0, 3], H[1, 3], H[2, 3], a, b, c]


def KUKA_2_Pose(v):
    """[x,y,z,A,B,C] degrees -> pose = transl * rotz(A) * roty(B) * rotx(C)."""
    d = pi / 180
    return transl(v[0], v[1], v[2]) * rotz(v[3] * d) * roty(v[4] * d) * rotx(v[5] * d)


def Pose_2_KUKA(H):
    r20 = H[2, 0]
    if abs(r20) < 1 - 1e-9:
        b = math.asin(-r20)
        a = math.atan2(H[1, 0], H[0, 0])
        c = math.atan2(H[2, 1], H[2, 2])
    else:
        b = math.pi / 2 if r20 < 0 else -math.pi / 2
        a = 0.0
        c = math.atan2(-H[1, 2], H[1, 1])
    r = 180 / pi
    return [H[0, 3], H[1, 3], H[2, 3], a * r, b * r, c * r]


def Fanuc_2_Pose(v):
    return KUKA_2_Pose([v[0], v[1], v[2], v[5], v[4], v[3]])


def Pose_2_Fanuc(H):
    x, y, z, a, b, c = Pose_2_KUKA(H)
    return [x, y, z, c, b, a]


def Motoman_2_Pose(v):
    return Fanuc_2_Pose(v)


def Pose_2_Motoman(H):
    return Pose_2_Fanuc(H)


def quaternion_2_pose(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return Mat([[w * w + x * x - y * y - z * z, 2 * (x * y - w * z), 2 * (x * z + w * y), 0],
                [2 * (x * y + w * z), w * w - x * x + y * y - z * z, 2 * (y * z - w * x), 0],
                [2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z, 0],
                [0, 0, 0, 1]])


def pose_2_quaternion(H):
    tr = H[0, 0] + H[1, 1] + H[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1) * 2
        return [0.25 * s, (H[2, 1] - H[1, 2]) / s, (H[0, 2] - H[2, 0]) / s, (H[1, 0] - H[0, 1]) / s]
    if H[0, 0] > H[1, 1] and H[0, 0] > H[2, 2]:
        s = math.sqrt(1 + H[0, 0] - H[1, 1] - H[2, 2]) * 2
        return [(H[2, 1] - H[1, 2]) / s, 0.25 * s, (H[0, 1] + H[1, 0]) / s, (H[0, 2] + H[2, 0]) / s]
    if H[1, 1] > H[2, 2]:
        s = math.sqrt(1 + H[1, 1] - H[0, 0] - H[2, 2]) * 2
        return [(H[0, 2] - H[2, 0]) / s, (H[0, 1] + H[1, 0]) / s, 0.25 * s, (H[1, 2] + H[2, 1]) / s]
    s = math.sqrt(1 + H[2, 2] - H[0, 0] - H[1, 1]) * 2
    return [(H[1, 0] - H[0, 1]) / s, (H[0, 2] + H[2, 0]) / s, (H[1, 2] + H[2, 1]) / s, 0.25 * s]


def ABB_2_Pose(v):
    m = quaternion_2_pose(v[3:7])
    return m.setPos(v[:3])


def Pose_2_ABB(H):
    return H.Pos() + pose_2_quaternion(H)


def UR_2_Pose(v):
    x, y, z, rx, ry, rz = v
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-12:
        return transl(x, y, z)
    kx, ky, kz = rx / angle, ry / angle, rz / angle
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    m = Mat([[t * kx * kx + c, t * kx * ky - s * kz, t * kx * kz + s * ky, x],
             [t * kx * ky + s * kz, t * ky * ky + c, t * ky * kz - s * kx, y],
             [t * kx * kz - s * ky, t * ky * kz + s * kx, t * kz * kz + c, z],
             [0, 0, 0, 1]])
    return m


def Pose_2_UR(H):
    w, x, y, z = pose_2_quaternion(H)
    s = math.sqrt(x * x + y * y + z * z)
    angle = 2 * math.atan2(s, w)
    if s < 1e-12:
        return H.Pos() + [0, 0, 0]
    k = angle / s
    return H.Pos() + [x * k, y * k, z * k]


def distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def normalize3(v):
    n = norm(v) or 1.0
    return [x / n for x in v]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def angle3(a, b):
    return math.acos(max(-1.0, min(1.0, dot(normalize3(a), normalize3(b)))))


def pause(seconds):
    import time
    time.sleep(seconds)


def tic():
    import time
    global _TIC
    _TIC = time.time()


def toc():
    import time
    return time.time() - _TIC


_TIC = 0.0
