/**
 * RoboDK-compatible scripting API (JavaScript). Mirrors robodk.robolink.Robolink / Item so that
 * RoboDK scripts port with minimal changes. Used by the in-app console, the WebSocket server and
 * the Python drop-in client (python/robodk/robolink.py).
 */
import { Station, Item as CoreItem, ItemType, Frame, Target, Tool, SceneObject, Folder } from '../core/items/item';
import { Robot } from '../core/items/robot';
import { Program, Instruction, InstructionType } from '../core/items/program';
import { createRobotFromLibrary, ROBOT_LIBRARY } from '../core/items/library';
import { Mat4, identity, multiply, invert, transl, rotx, roty, rotz, xyzrpwToPose, poseToXyzrpw, kukaToPose, poseToKuka, fanucToPose, poseToFanuc, urToPose, poseToUr, quatToPose, poseToQuat, toRows, fromRows, DEG } from '../core/math/pose';
import { ProgramSimulator } from '../core/motion/simulator';
import { compileForPost, getPost } from '../posts/index';
import { MobileRobot } from '../mobile/items';

export const ITEM_TYPE_ANY = -1, ITEM_TYPE_STATION = 1, ITEM_TYPE_ROBOT = 2, ITEM_TYPE_FRAME = 3, ITEM_TYPE_TOOL = 4, ITEM_TYPE_OBJECT = 5, ITEM_TYPE_TARGET = 6, ITEM_TYPE_PROGRAM = 8, ITEM_TYPE_INSTRUCTION = 9, ITEM_TYPE_PROGRAM_PYTHON = 10, ITEM_TYPE_FOLDER = 17, ITEM_TYPE_CAMERA = 19;
export const INSTRUCTION_CALL_PROGRAM = 0, INSTRUCTION_INSERT_CODE = 1, INSTRUCTION_START_THREAD = 2, INSTRUCTION_COMMENT = 3, INSTRUCTION_SHOW_MESSAGE = 4;
export const RUNMODE_SIMULATE = 1, RUNMODE_QUICKVALIDATE = 2, RUNMODE_MAKE_ROBOTPROG = 3, RUNMODE_RUN_ROBOT = 6;
export const ROBOTCOM_READY = 2, ROBOTCOM_DISCONNECTED = 0;

/** robodk.robomath-like Mat (row-major nested arrays API over our column-major Mat4). */
export class Mat {
  constructor(public m: Mat4 = identity()) {}
  static fromRows(rows: number[][]): Mat { return new Mat(fromRows(rows)); }
  rows(): number[][] { return toRows(this.m); }
  toString(): string { return this.rows().map((r) => r.map((v) => v.toFixed(3)).join(', ')).join('\n'); }
  mul(o: Mat): Mat { return new Mat(multiply(this.m, o.m)); }
  inv(): Mat { return new Mat(invert(this.m)); }
  Pos(): number[] { return [this.m[12], this.m[13], this.m[14]]; }
  setPos(p: number[]): Mat { this.m[12] = p[0]; this.m[13] = p[1]; this.m[14] = p[2]; return this; }
  Pose_2_TxyzRxyz(): number[] { return poseToXyzrpw(this.m); }
  Pose_2_KUKA(): number[] { return poseToKuka(this.m); }
  Pose_2_Fanuc(): number[] { return poseToFanuc(this.m); }
  Pose_2_UR(): number[] { return poseToUr(this.m); }
  Pose_2_ABB(): number[] { const [w, x, y, z] = poseToQuat(this.m); return [this.m[12], this.m[13], this.m[14], w, x, y, z]; }
  toArray(): number[] { return Array.from(this.m); }
}
export const transl_ = (x: number, y: number, z: number) => new Mat(transl(x, y, z));
export const robomath = {
  Mat,
  eye: () => new Mat(identity()),
  transl: (x: number, y: number, z: number) => new Mat(transl(x, y, z)),
  rotx: (r: number) => new Mat(rotx(r)),
  roty: (r: number) => new Mat(roty(r)),
  rotz: (r: number) => new Mat(rotz(r)),
  TxyzRxyz_2_Pose: (v: number[]) => new Mat(xyzrpwToPose(v[0], v[1], v[2], v[3], v[4], v[5])),
  KUKA_2_Pose: (v: number[]) => new Mat(kukaToPose(v[0], v[1], v[2], v[3], v[4], v[5])),
  Fanuc_2_Pose: (v: number[]) => new Mat(fanucToPose(v[0], v[1], v[2], v[3], v[4], v[5])),
  UR_2_Pose: (v: number[]) => new Mat(urToPose(v[0], v[1], v[2], v[3], v[4], v[5])),
  ABB_2_Pose: (v: number[]) => new Mat(quatToPose(v[0], v[1], v[2], [v[3], v[4], v[5], v[6]])),
  Pose_2_TxyzRxyz: (m: Mat) => m.Pose_2_TxyzRxyz(),
  Pose_2_KUKA: (m: Mat) => m.Pose_2_KUKA(),
  Pose_2_UR: (m: Mat) => m.Pose_2_UR(),
  pi: Math.PI,
  DEG,
};

const toMat4 = (p: Mat | Mat4 | number[] | number[][]): Mat4 => {
  if (p instanceof Mat) return p.m;
  if (p instanceof Float64Array) return p;
  if (Array.isArray(p) && Array.isArray(p[0])) return fromRows(p as number[][]);
  return Float64Array.from(p as number[]);
};

export class RobolinkItem {
  constructor(readonly rdk: Robolink, public item: CoreItem | null) {}
  Valid(): boolean { return !!this.item && (this.item.type === ItemType.STATION || !!this.item.parent); }
  private req<T extends CoreItem>(): T { if (!this.item) throw new Error('Invalid item'); return this.item as T; }
  Name(): string { return this.item?.name ?? ''; }
  setName(n: string): this { this.req().setName(n); return this; }
  Type(): number { return this.item?.type ?? -1; }
  Parent(): RobolinkItem { return new RobolinkItem(this.rdk, this.item?.parent ?? null); }
  Childs(): RobolinkItem[] { return (this.item?.children ?? []).map((c) => new RobolinkItem(this.rdk, c)); }
  Delete(): void { this.item?.delete(); this.item = null; }
  Visible(): boolean { return !!this.item?.visible; }
  setVisible(v: boolean): this { this.req().setVisible(v); return this; }
  setParent(p: RobolinkItem): this { this.req().setParent(p.req()); return this; }
  setParentStatic(p: RobolinkItem): this { this.req().setParentStatic(p.req()); return this; }
  Pose(): Mat {
    const it = this.req();
    if (it instanceof Robot) return new Mat(it.poseTCP());
    return new Mat(it.pose());
  }
  setPose(p: Mat | Mat4 | number[] | number[][]): this {
    const it = this.req();
    const m = toMat4(p);
    if (it instanceof Robot) { const r = it.solveIKFrame(m); if (r.ok) it.setJoints(r.joints); else throw new Error('Target not reachable'); }
    else it.setPose(m);
    return this;
  }
  PoseAbs(): Mat { const it = this.req(); return new Mat(it instanceof Robot ? it.poseTCPAbs() : it.poseAbs()); }
  setPoseAbs(p: Mat | Mat4 | number[]): this { this.req().setPoseAbs(toMat4(p)); return this; }
  PoseTool(): Mat { const it = this.req(); return new Mat(it instanceof Robot ? it.poseTool() : it instanceof Tool ? it.poseTool() : identity()); }
  setPoseTool(p: Mat | Mat4 | number[] | RobolinkItem): this {
    const it = this.req();
    if (p instanceof RobolinkItem) { if (it instanceof Robot && p.item instanceof Tool) it.setTool(p.item); if (it instanceof Program) it.toolId = p.item?.id ?? null; }
    else if (it instanceof Robot) it.setPoseTool(toMat4(p));
    else if (it instanceof Tool) it.setPoseTool(toMat4(p));
    return this;
  }
  PoseFrame(): Mat { const it = this.req(); return new Mat(it instanceof Robot ? it.poseFrame() : identity()); }
  setPoseFrame(p: Mat | Mat4 | number[] | RobolinkItem): this {
    const it = this.req();
    if (p instanceof RobolinkItem) { if (it instanceof Robot) it.setFrame(p.item); if (it instanceof Program) it.frameId = p.item?.id ?? null; }
    else if (it instanceof Robot) { const f = new Frame('Frame'); (it.parent ?? this.rdk.station).addChild(f); f.setPose(multiply(it.pose(), toMat4(p))); it.setFrame(f); }
    return this;
  }
  Joints(): number[] { const it = this.req(); if (it instanceof Robot) return it.joints(); if (it instanceof Target) return it.joints ?? []; return []; }
  setJoints(q: number[]): this { const it = this.req(); if (it instanceof Robot) it.setJoints(q); else if (it instanceof Target) it.setJoints(q); return this; }
  JointsHome(): number[] { return this.req<Robot>().jointsHome(); }
  JointLimits(): [number[], number[]] { const l = this.req<Robot>().jointLimits(); return [l.lower, l.upper]; }
  setJointLimits(lo: number[], hi: number[]): this { this.req<Robot>().setJointLimits(lo, hi); return this; }
  SolveFK(q: number[]): Mat { return new Mat(this.req<Robot>().solveFK(q)); }
  SolveIK(p: Mat | Mat4 | number[], jointsApprox?: number[]): number[] { const r = this.req<Robot>().solveIK(toMat4(p), { seed: jointsApprox }); return r.ok ? r.joints : []; }
  SolveIK_All(p: Mat | Mat4 | number[]): number[][] { return this.req<Robot>().solveIKAll(toMat4(p)); }
  Connect(_ip?: string): boolean { return true; }
  ConnectedState(): number { return ROBOTCOM_READY; }
  setSpeed(speedLinear: number, speedJoints = -1, accelLinear = -1, accelJoints = -1): this {
    const it = this.req();
    if (it instanceof Robot) { if (speedLinear > 0) it.motion.speedLinear = speedLinear; if (speedJoints > 0) it.motion.speedJoints = speedJoints; if (accelLinear > 0) it.motion.accelLinear = accelLinear; if (accelJoints > 0) it.motion.accelJoints = accelJoints; }
    else if (it instanceof Program) it.setSpeed(speedLinear > 0 ? speedLinear : undefined, speedJoints > 0 ? speedJoints : undefined, accelLinear > 0 ? accelLinear : undefined, accelJoints > 0 ? accelJoints : undefined);
    return this;
  }
  setRounding(r: number): this { const it = this.req(); if (it instanceof Robot) it.motion.rounding = r; else if (it instanceof Program) it.setRounding(r); return this; }
  setZoneData(r: number): this { return this.setRounding(r); }
  /** MoveJ: on a robot -> move instantly (simulate); on a program -> add an instruction. */
  MoveJ(target: RobolinkItem | number[] | Mat, _blocking = true): this {
    const it = this.req();
    const tgt = target instanceof RobolinkItem ? (target.item as Target) : target instanceof Mat ? target.m : target;
    if (it instanceof Robot) { if (!it.moveJInstant(tgt as any)) throw new Error('MoveJ: target not reachable'); }
    else if (it instanceof Program) it.addMoveJ(tgt as any);
    this.rdk.render();
    return this;
  }
  MoveL(target: RobolinkItem | number[] | Mat, _blocking = true): this {
    const it = this.req();
    const tgt = target instanceof RobolinkItem ? (target.item as Target) : target instanceof Mat ? target.m : target;
    if (it instanceof Robot) { if (!it.moveJInstant(tgt as any)) throw new Error('MoveL: target not reachable'); }
    else if (it instanceof Program) it.addMoveL(tgt as any);
    this.rdk.render();
    return this;
  }
  MoveC(via: RobolinkItem, target: RobolinkItem): this {
    const it = this.req();
    if (it instanceof Program) it.addMoveC(via.item as Target, target.item as Target);
    else if (it instanceof Robot) it.moveJInstant(target.item as Target);
    return this;
  }
  Pause(ms: number): this { const it = this.req(); if (it instanceof Program) it.pause(ms); return this; }
  setDO(io: string, value: string | number | boolean): this { const it = this.req(); if (it instanceof Program) it.setDO(io, typeof value === 'string' ? value !== '0' : value); else if (it instanceof Robot) it.state.io[io] = typeof value === 'string' ? value !== '0' : value; return this; }
  waitDI(io: string, value: string | number | boolean, timeout = -1): this { const it = this.req(); if (it instanceof Program) it.waitDI(io, typeof value === 'string' ? value !== '0' : value, timeout); return this; }
  RunInstruction(code: string, type = INSTRUCTION_CALL_PROGRAM): this {
    const it = this.req();
    if (it instanceof Program) {
      if (type === INSTRUCTION_COMMENT) it.comment(code);
      else if (type === INSTRUCTION_SHOW_MESSAGE) it.showMessage(code);
      else if (type === INSTRUCTION_INSERT_CODE) it.runInstruction(code, false);
      else if (/^(attach|detach|gripper_open|gripper_close)$/i.test(code)) it.event(code.toLowerCase() as any);
      else it.runInstruction(code, true);
    }
    return this;
  }
  RunCodeCustom(code: string, type = INSTRUCTION_CALL_PROGRAM): this { return this.RunInstruction(code, type); }
  ShowInstructions(_v = true): this { return this; }
  ShowTargets(_v = true): this { return this; }
  InstructionCount(): number { return this.req<Program>().instructions().length; }
  Instruction(id: number): { name: string; type: number; moveType: number; isJointTarget: boolean; pose: Mat | null; joints: number[] | null } {
    const ins = this.req<Program>().instructions()[id];
    const d = ins.data;
    if (d.kind === 'move') {
      const t = d.targetId ? (this.rdk.station.findById(d.targetId) as Target | null) : null;
      return { name: ins.name, type: ins.insType, moveType: d.moveType === 'MoveJ' ? 1 : d.moveType === 'MoveL' ? 2 : 3, isJointTarget: !!t?.isJointTarget, pose: t ? new Mat(t.pose()) : d.pose ? new Mat(Float64Array.from(d.pose)) : null, joints: t?.joints ?? d.joints ?? null };
    }
    return { name: ins.name, type: ins.insType, moveType: 0, isJointTarget: false, pose: null, joints: null };
  }
  InstructionList(): Array<ReturnType<RobolinkItem['Instruction']>> { return this.req<Program>().instructions().map((_, i) => this.Instruction(i)); }
  InstructionDelete(id: number): this { this.req<Program>().instructions()[id]?.delete(); return this; }
  setRobot(r: RobolinkItem): this { this.req<Program>().setRobot(r.item); return this; }
  getLink(type: number): RobolinkItem {
    const it = this.req();
    if (it instanceof Program) { const r = it.robot(); return new RobolinkItem(this.rdk, type === ITEM_TYPE_ROBOT ? r : type === ITEM_TYPE_FRAME ? (it.frameId ? this.rdk.station.findById(it.frameId) : (r instanceof Robot ? r.activeFrame() : null)) : type === ITEM_TYPE_TOOL ? (it.toolId ? this.rdk.station.findById(it.toolId) : (r instanceof Robot ? r.activeTool() : null)) : null); }
    if (it instanceof Robot) return new RobolinkItem(this.rdk, type === ITEM_TYPE_TOOL ? it.activeTool() : type === ITEM_TYPE_FRAME ? it.activeFrame() : it);
    if (it instanceof Tool) return new RobolinkItem(this.rdk, it.parent);
    return new RobolinkItem(this.rdk, null);
  }
  AddTool(pose: Mat | Mat4 | number[], name = 'Tool'): RobolinkItem {
    const r = this.req<Robot>();
    const t = new Tool(name);
    t.setPoseTool(toMat4(pose));
    r.addChild(t);
    r.setTool(t);
    return new RobolinkItem(this.rdk, t);
  }
  AddFrame(name: string): RobolinkItem { return this.rdk.AddFrame(name, this); }
  AddTarget(name: string): RobolinkItem { return this.rdk.AddTarget(name, this); }
  setAsCartesianTarget(): this { this.req<Target>().setAsCartesianTarget(); return this; }
  setAsJointTarget(): this { this.req<Target>().setAsJointTarget(); return this; }
  isJointTarget(): boolean { return !!this.req<Target>().isJointTarget; }
  /** Run/validate the program: returns [valid_instructions, program_time, program_distance, valid_ratio, readable_msg]. */
  Update(): [number, number, number, number, string] {
    const p = this.req<Program>();
    const sim = new ProgramSimulator(this.rdk.station);
    const r = sim.compile(p);
    const errs = r.problems.filter((x) => x.severity === 'error');
    return [r.executed - errs.length, r.duration, r.distance, r.executed ? (r.executed - errs.length) / r.executed : 1, errs.map((e) => e.message).join('; ') || 'OK'];
  }
  RunProgram(): number { const p = this.req<Program>(); this.rdk.onRunProgram?.(p); return 0; }
  RunCode(): number { return this.RunProgram(); }
  MakeProgram(path = '', postId?: string): [boolean, string, string] {
    const p = this.req<Program>();
    const robot = p.robot();
    const post = getPost(postId ?? (robot instanceof Robot ? robot.postProcessor : 'Generic')) ?? getPost('Generic')!;
    const files = post.generate(compileForPost(this.rdk.station, p));
    this.rdk.generated.set(p.id, files);
    return [true, files[0]?.content ?? '', `${path}${files[0]?.name ?? ''}`];
  }
  setParam(key: string, value: any): this { this.req().setParam(key, value); return this; }
  getParam(key: string): any { return this.req().getParam(key); }
  setColor(color: string | number[]): this { const it = this.req(); it.color = Array.isArray(color) ? `#${color.slice(0, 3).map((c) => Math.round(c * 255).toString(16).padStart(2, '0')).join('')}` : color; it.notify('color'); return this; }
  Recolor(color: string | number[]): this { return this.setColor(color); }
  setGeometryPose(p: Mat | Mat4 | number[]): this { const it = this.req(); if (it instanceof SceneObject) { for (const g of it.geometry) g.origin = Array.from(toMat4(p)); it.notify('geometry'); } return this; }
  Busy(): boolean { const it = this.req(); return it instanceof Robot ? it.state.moving : false; }
  Stop(): void { this.rdk.onStop?.(); }
  WaitMove(): void { /* simulation is instantaneous in API mode */ }
  ProgramStart(name: string): number { this.rdk.AddProgram(name, this); return 1; }
  DOF(): number { return this.req<Robot>().dof; }
  toString(): string { return `Item(${this.Name()})`; }
}

export class Robolink {
  generated = new Map<string, any>();
  onRender: (() => void) | null = null;
  onRunProgram: ((p: Program) => void) | null = null;
  onStop: (() => void) | null = null;
  onMessage: ((msg: string, popup: boolean) => void) | null = null;
  runMode = RUNMODE_SIMULATE;
  constructor(public station: Station) {}
  render(): void { this.onRender?.(); }
  Render(_v = true): void { this.render(); }
  /** Item by name (exact, else case-insensitive partial match). Empty name returns the first item of the type. */
  Item(name: string, type: number = ITEM_TYPE_ANY): RobolinkItem {
    if (type === ITEM_TYPE_STATION || (name === '' && type === ITEM_TYPE_ANY)) return new RobolinkItem(this, this.station);
    if (name === '') return new RobolinkItem(this, this.station.itemsOfType(type as ItemType)[0] ?? null);
    const exact = this.station.find(name, type as ItemType);
    if (exact) return new RobolinkItem(this, exact);
    const lower = name.toLowerCase();
    const items = type === ITEM_TYPE_ANY ? [...this.station.walk()].slice(1) : this.station.itemsOfType(type as ItemType);
    return new RobolinkItem(this, items.find((i) => i.name.toLowerCase().includes(lower)) ?? null);
  }
  ItemList(type: number = ITEM_TYPE_ANY, listNames = false): any[] {
    const items = type === ITEM_TYPE_ANY ? [...this.station.walk()].slice(1) : this.station.itemsOfType(type as ItemType);
    return listNames ? items.map((i) => i.name) : items.map((i) => new RobolinkItem(this, i));
  }
  ItemUserPick(_msg = '', type: number = ITEM_TYPE_ANY): RobolinkItem {
    const sel = this.station.selection.find((s) => type === ITEM_TYPE_ANY || s.type === type);
    return new RobolinkItem(this, sel ?? (type === ITEM_TYPE_ANY ? null : this.station.itemsOfType(type as ItemType)[0] ?? null));
  }
  ActiveStation(): RobolinkItem { return new RobolinkItem(this, this.station); }
  AddStation(name: string): RobolinkItem { this.station.setName(name); return new RobolinkItem(this, this.station); }
  AddFrame(name: string, parent?: RobolinkItem): RobolinkItem {
    const f = new Frame(name);
    (parent?.item ?? this.station).addChild(f);
    return new RobolinkItem(this, f);
  }
  AddFolder(name: string, parent?: RobolinkItem): RobolinkItem { const f = new Folder(name); (parent?.item ?? this.station).addChild(f); return new RobolinkItem(this, f); }
  AddTarget(name: string, parent?: RobolinkItem, robot?: RobolinkItem): RobolinkItem {
    const t = new Target(name);
    const r = (robot?.item as Robot | null) ?? this.station.itemsOfType<Robot>(ItemType.ROBOT)[0];
    const p = parent?.item ?? r?.activeFrame() ?? this.station;
    p.addChild(t);
    if (r) { t.setPoseAbs(r.poseTCPAbs()); t.setJoints(r.joints()); t.robotId = r.id; }
    return new RobolinkItem(this, t);
  }
  AddProgram(name: string, robot?: RobolinkItem): RobolinkItem {
    const p = new Program(name);
    this.station.addChild(p);
    const r = robot?.item ?? this.station.itemsOfType(ItemType.ROBOT)[0] ?? null;
    if (r) p.setRobot(r);
    return new RobolinkItem(this, p);
  }
  AddRobot(libraryIdOrName: string, parent?: RobolinkItem): RobolinkItem {
    const e = ROBOT_LIBRARY.find((x) => x.id === libraryIdOrName || x.name === libraryIdOrName || x.name.toLowerCase().includes(libraryIdOrName.toLowerCase()));
    if (!e) throw new Error(`Robot ${libraryIdOrName} not in library`);
    const r = createRobotFromLibrary(e.id);
    (parent?.item ?? this.station).addChild(r);
    return new RobolinkItem(this, r);
  }
  AddMobileRobot(name: string): RobolinkItem { const m = new MobileRobot(name); this.station.addChild(m); return new RobolinkItem(this, m); }
  AddFile(_path: string, _parent?: RobolinkItem): RobolinkItem { this.onMessage?.('AddFile: drop the file onto the viewport or use File > Import', false); return new RobolinkItem(this, null); }
  AddShape(triangles: number[][], parent?: RobolinkItem, name = 'Shape'): RobolinkItem {
    const o = new SceneObject(name);
    const pos = new Float32Array(triangles.flat());
    const id = `shape_${o.id}`;
    this.assetsRegister?.(id, pos);
    o.geometry = [{ mesh: id, origin: Array.from(identity()), color: '#8ca0b3' }];
    (parent?.item ?? this.station).addChild(o);
    return new RobolinkItem(this, o);
  }
  assetsRegister: ((id: string, positions: Float32Array) => void) | null = null;
  AddCurve(points: number[][], parent?: RobolinkItem, _addToRef = false): RobolinkItem {
    const o = (parent?.item instanceof SceneObject ? parent.item : null) ?? new SceneObject('Curve');
    if (!o.parent) (parent?.item ?? this.station).addChild(o);
    o.curves.push({ name: `curve ${o.curves.length + 1}`, points });
    o.notify('geometry');
    return new RobolinkItem(this, o);
  }
  AddPoints(points: number[][], parent?: RobolinkItem): RobolinkItem {
    const o = (parent?.item instanceof SceneObject ? parent.item : null) ?? new SceneObject('Points');
    if (!o.parent) (parent?.item ?? this.station).addChild(o);
    points.forEach((p, i) => o.points.push({ name: `p${i + 1}`, point: p }));
    return new RobolinkItem(this, o);
  }
  ShowMessage(msg: string, popup = true): void { this.onMessage?.(msg, popup); }
  setRunMode(m: number): void { this.runMode = m; }
  RunMode(): number { return this.runMode; }
  setSimulationSpeed(_s: number): void { /* handled by the app */ }
  SimulationSpeed(): number { return 1; }
  Save(_file: string, _item?: RobolinkItem): void { this.onMessage?.('Use File > Save station', false); }
  getParam(key: string): any { return this.station.settings[key] ?? this.station.getParam(key); }
  setParam(key: string, value: any): void { this.station.setParam(key, value); }
  Command(cmd: string, value: any = ''): string { this.station.setParam(`cmd:${cmd}`, value); return 'OK'; }
  Version(): string { return 'VerticalBot Studio 0.1 (RoboDK API compatible)'; }
  License(): [string, string] { return ['VerticalBot Studio', 'MIT']; }
  Selection(): RobolinkItem[] { return this.station.selection.map((s) => new RobolinkItem(this, s)); }
  setSelection(items: RobolinkItem[]): void { this.station.setSelection(items.map((i) => i.item!).filter(Boolean)); }
  Update(): void { this.render(); }
  Finish(): void {}
  Disconnect(): void {}
  Connect(): boolean { return true; }
  Collisions(): number { return 0; }
  Delete(items: RobolinkItem[]): void { for (const i of items) i.Delete(); }
  Cam2D_Snapshot(_file = '', _cam?: RobolinkItem): string { return ''; }
  ProjectPoints(points: number[][], _obj?: RobolinkItem): number[][] { return points; }
  IsInside(_a: RobolinkItem, _b: RobolinkItem): number { return 0; }
}

export { ItemType, InstructionType, Robot, Program, Target, Frame, Tool, SceneObject, Instruction };
