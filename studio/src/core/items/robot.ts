import { Mat4, identity, multiply, invert, clone, fromArray } from '../math/pose';
import { Item, ItemType, Frame, Tool, Target, SerializedItem, DeserializeContext, registerItemType } from './item';
import { ChainDef, forwardKinematics, flangePose, homeJoints, jointLimits, actuatedIndices, withinLimits, FKResult } from '../kinematics/chain';
import { inverseKinematics, IKOptions, IKResult, closestConfiguration } from '../kinematics/ik';

export interface RobotMotionParams {
  /** Linear speed mm/s, joint speed deg/s, accelerations. */
  speedLinear: number;
  speedJoints: number;
  accelLinear: number;
  accelJoints: number;
  /** Rounding / blending radius (mm); -1 = fine point. */
  rounding: number;
}

export type RobotBrand = 'Generic' | 'KUKA' | 'ABB' | 'Fanuc' | 'UR' | 'Yaskawa' | 'Staubli' | 'Doosan' | 'Kawasaki' | 'Denso' | 'Mecademic' | 'ROS2' | 'Custom';

export interface RobotState {
  joints: number[];
  moving: boolean;
  /** Fraction of last programmed motion completed. */
  progress: number;
  faults: string[];
  /** Digital I/O */
  io: Record<string, number | boolean>;
}

/**
 * Serial robot arm (RoboDK ITEM_TYPE_ROBOT). The robot pose is its base frame w.r.t. parent.
 * Active tool: a Tool child. Active reference frame: a Frame anywhere in the station.
 */
export class Robot extends Item {
  chain: ChainDef;
  private _joints: number[];
  brand: RobotBrand = 'Generic';
  model = '';
  /** Post-processor id. */
  postProcessor = 'Generic';
  /** Controller IP / ROS namespace for connections. */
  connection: { ip?: string; port?: number; rosNamespace?: string } = {};
  motion: RobotMotionParams = { speedLinear: 500, speedJoints: 90, accelLinear: 2000, accelJoints: 360, rounding: -1 };
  activeToolId: string | null = null;
  activeFrameId: string | null = null;
  state: RobotState = { joints: [], moving: false, progress: 1, faults: [], io: {} };
  /** Reach (mm) for quick feasibility checks, computed from the chain. */
  reach = 0;
  /** External axes / turntable ids synchronised with this robot. */
  externalAxes: string[] = [];

  constructor(name: string, chain: ChainDef, id?: string) {
    super(ItemType.ROBOT, name, id);
    this.chain = chain;
    this._joints = homeJoints(chain);
    this.state.joints = [...this._joints];
    this.reach = estimateReach(chain);
  }

  get dof(): number {
    return actuatedIndices(this.chain).length;
  }

  jointNames(): string[] {
    return actuatedIndices(this.chain).map((i) => this.chain.joints[i].name);
  }

  joints(): number[] {
    return [...this._joints];
  }

  setJoints(q: number[]): this {
    const n = this.dof;
    const v = q.slice(0, n);
    while (v.length < n) v.push(0);
    this._joints = v;
    this.state.joints = [...v];
    this.notify('joints');
    return this;
  }

  jointsHome(): number[] {
    return homeJoints(this.chain);
  }

  jointLimits(): { lower: number[]; upper: number[] } {
    return jointLimits(this.chain);
  }

  setJointLimits(lower: number[], upper: number[]): void {
    actuatedIndices(this.chain).forEach((ji, k) => {
      if (lower[k] !== undefined) this.chain.joints[ji].lower = lower[k];
      if (upper[k] !== undefined) this.chain.joints[ji].upper = upper[k];
    });
    this.notify('limits');
  }

  // -- Tool / frame ---------------------------------------------------------

  tools(): Tool[] {
    return this.children.filter((c) => c instanceof Tool) as Tool[];
  }

  activeTool(): Tool | null {
    if (this.activeToolId) {
      const t = this.station?.findById(this.activeToolId);
      if (t instanceof Tool) return t;
    }
    return this.tools()[0] ?? null;
  }

  setTool(tool: Tool | null): void {
    this.activeToolId = tool?.id ?? null;
    this.notify('tool');
  }

  activeFrame(): Frame | Item | null {
    if (this.activeFrameId) {
      const f = this.station?.findById(this.activeFrameId);
      if (f) return f;
    }
    return this.parent && this.parent.type !== ItemType.STATION ? this.parent : null;
  }

  setFrame(frame: Item | null): void {
    this.activeFrameId = frame?.id ?? null;
    this.notify('frame');
  }

  /** TCP pose relative to the flange (identity if no tool). */
  poseTool(): Mat4 {
    return this.activeTool()?.poseTool() ?? identity();
  }

  /** Active reference frame pose relative to the robot base. */
  poseFrame(): Mat4 {
    const f = this.activeFrame();
    if (!f) return identity();
    return multiply(invert(this.poseAbs()), f.poseAbs());
  }

  // -- Kinematics -----------------------------------------------------------

  fk(q: number[] = this._joints): FKResult {
    return forwardKinematics(this.chain, q);
  }

  /** Flange pose relative to the robot base. */
  solveFKFlange(q: number[] = this._joints): Mat4 {
    return flangePose(this.chain, q);
  }

  /** TCP pose relative to the robot base (RoboDK SolveFK with tool). */
  solveFK(q: number[] = this._joints, tool: Mat4 = this.poseTool()): Mat4 {
    return multiply(flangePose(this.chain, q), tool);
  }

  /** TCP pose w.r.t. the active reference frame (RoboDK Item.Pose for a robot). */
  poseTCP(q: number[] = this._joints): Mat4 {
    return multiply(invert(this.poseFrame()), this.solveFK(q));
  }

  /** Absolute TCP pose. */
  poseTCPAbs(q: number[] = this._joints): Mat4 {
    return multiply(this.poseAbs(), this.solveFK(q));
  }

  /**
   * Inverse kinematics: target = TCP pose relative to the robot base (RoboDK SolveIK).
   * Returns joints or null.
   */
  solveIK(target: Mat4, opts: IKOptions = {}, tool: Mat4 = this.poseTool()): IKResult {
    const flangeTarget = multiply(target, invert(tool));
    const res = inverseKinematics(this.chain, flangeTarget, { seed: this._joints, ...opts });
    if (res.ok) res.joints = closestConfiguration(this.chain, res.joints, opts.seed ?? this._joints);
    return res;
  }

  /** Multiple IK solutions (different seeds), deduplicated. */
  solveIKAll(target: Mat4, tool: Mat4 = this.poseTool(), attempts = 12): number[][] {
    const flangeTarget = multiply(target, invert(tool));
    const { lower, upper } = this.jointLimits();
    const sols: number[][] = [];
    for (let a = 0; a < attempts; a++) {
      const seed = a === 0 ? this._joints : lower.map((lo, k) => lo + (upper[k] - lo) * ((((a * 7919) % 104729) / 104729 + k * 0.37) % 1));
      const r = inverseKinematics(this.chain, flangeTarget, { seed, restarts: 0 });
      if (!r.ok) continue;
      if (!sols.some((s) => s.every((v, i) => Math.abs(v - r.joints[i]) < 1))) sols.push(r.joints);
    }
    return sols;
  }

  /** IK for a pose given in the active reference frame. */
  solveIKFrame(targetInFrame: Mat4, opts: IKOptions = {}): IKResult {
    return this.solveIK(multiply(this.poseFrame(), targetInFrame), opts);
  }

  /** Move immediately (simulation) to a target or joints. */
  moveJInstant(target: Target | number[] | Mat4): boolean {
    const q = this.jointsForTarget(target);
    if (!q) return false;
    this.setJoints(q);
    return true;
  }

  /** Resolve a target / joints / pose (in active frame) into joint values. */
  jointsForTarget(target: Target | number[] | Mat4, opts: IKOptions = {}): number[] | null {
    if (Array.isArray(target)) return target;
    if (target instanceof Float64Array) {
      const r = this.solveIKFrame(target, opts);
      return r.ok ? r.joints : null;
    }
    if (target.isJointTarget && target.joints) return target.joints;
    // Cartesian target: its pose is relative to its parent (a frame). Convert to the robot base.
    const abs = target.poseAbs();
    const inBase = multiply(invert(this.poseAbs()), abs);
    const r = this.solveIK(inBase, { seed: target.joints ?? this._joints, ...opts });
    return r.ok ? r.joints : null;
  }

  /** Check whether joints are inside limits. */
  jointsValid(q: number[]): boolean {
    return withinLimits(this.chain, q);
  }

  setPoseTool(m: Mat4): void {
    const t = this.activeTool();
    if (t) t.setPoseTool(m);
  }

  // -- Serialization ------------------------------------------------------

  protected override serializeExtra() {
    return {
      chain: serializeChain(this.chain),
      joints: this._joints,
      brand: this.brand,
      model: this.model,
      postProcessor: this.postProcessor,
      connection: this.connection,
      motion: this.motion,
      activeToolId: this.activeToolId,
      activeFrameId: this.activeFrameId,
      externalAxes: this.externalAxes,
    };
  }

  override deserializeExtra(d: SerializedItem, _ctx: DeserializeContext) {
    this.chain = deserializeChain(d.chain as any);
    this.reach = estimateReach(this.chain);
    this._joints = (d.joints as number[]) ?? homeJoints(this.chain);
    this.state.joints = [...this._joints];
    this.brand = (d.brand as RobotBrand) ?? 'Generic';
    this.model = (d.model as string) ?? '';
    this.postProcessor = (d.postProcessor as string) ?? 'Generic';
    this.connection = (d.connection as any) ?? {};
    this.motion = { ...this.motion, ...((d.motion as any) ?? {}) };
    this.activeToolId = (d.activeToolId as string | null) ?? null;
    this.activeFrameId = (d.activeFrameId as string | null) ?? null;
    this.externalAxes = (d.externalAxes as string[]) ?? [];
  }
}

export function serializeChain(c: ChainDef): any {
  return {
    name: c.name,
    joints: c.joints.map((j) => ({ ...j, origin: Array.from(j.origin), post: j.post ? Array.from(j.post) : undefined })),
    links: c.links.map((l) => ({ name: l.name, visuals: l.visuals.map((v) => ({ ...v, origin: Array.from(v.origin) })) })),
    flange: Array.from(c.flange),
    dh: c.dh,
    dof: c.dof,
  };
}

export function deserializeChain(d: any): ChainDef {
  return {
    name: d.name,
    joints: d.joints.map((j: any) => ({ ...j, origin: fromArray(j.origin), post: j.post ? fromArray(j.post) : undefined })),
    links: d.links.map((l: any) => ({ name: l.name, visuals: (l.visuals ?? []).map((v: any) => ({ ...v, origin: fromArray(v.origin) })) })),
    flange: fromArray(d.flange),
    dh: d.dh,
    dof: d.dof,
  };
}

/** Rough reach: sum of link offsets. */
export function estimateReach(chain: ChainDef): number {
  let r = 0;
  for (const j of chain.joints) {
    r += Math.hypot(j.origin[12], j.origin[13], j.origin[14]);
    if (j.post) r += Math.hypot(j.post[12], j.post[13], j.post[14]);
    if (j.type === 'prismatic') r += Math.max(Math.abs(j.upper), Math.abs(j.lower));
  }
  r += Math.hypot(chain.flange[12], chain.flange[13], chain.flange[14]);
  return r;
}

// Robot deserialization needs a chain: register a factory that creates a placeholder chain.
registerItemType(ItemType.ROBOT, (n, id) => new Robot(n, { name: 'empty', joints: [], links: [{ name: 'base', visuals: [] }], flange: identity() }, id));

export { clone as clonePose };
