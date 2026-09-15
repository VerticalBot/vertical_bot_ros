/** Application context: station, renderer, simulators, undo/redo, file I/O and high-level commands. */
import { Station, Item, ItemType, Frame, Target, Tool, SceneObject, Folder, Camera } from './core/items/item';
import { Robot } from './core/items/robot';
import { Program, Instruction, PathItem } from './core/items/program';
import { createRobotFromLibrary, ROBOT_LIBRARY } from './core/items/library';
import { ProgramSimulator } from './core/motion/simulator';
import { planMoveJ, planMoveL, Trajectory } from './core/motion/trajectory';
import { AssetStore } from './scene/assets';
import { SceneRenderer } from './scene/renderer';
import { EventBus } from './core/events';
import { Mat4, multiply, invert, transl, identity, getPos } from './core/math/pose';
import { saveStation, loadStation, StationFile } from './io/station-file';
import { robotFromURDF, parseURDF } from './io/urdf/urdf';
import { importProgram, detectLanguage } from './io/programs/import';
import { importTargets } from './io/robodk/targets';
import { importRdkBestEffort } from './io/robodk/rdk_container';
import { parseOBJ } from './io/mesh/obj';
import { parseDHText, robotFromDH } from './io/robodk/dh';
import { stationToRoboDKScript } from './io/robodk/station_script';
import { compileForPost, getPost, listPosts, PostFile } from './posts/index';
import { MobileRobot, MapItem, ZoneItem } from './mobile/items';
import { FleetItem, FleetManager } from './fleet/fleet';
import { ProcessSimulator, Component } from './vc/component';
import { FieldItem, MissionItem, CropRow } from './agri/items';
import { updateMissionProgress } from './agri/missions';
import { toast } from './ui/dom';

export interface AppEvents extends Record<string, unknown> {
  stationChanged: { station: Station };
  activeProgram: { program: Program | null };
  activeRobot: { robot: Robot | null };
  simulation: { playing: boolean; time: number; duration: number };
  log: { text: string; level: 'info' | 'warn' | 'error' };
  undo: { canUndo: boolean; canRedo: boolean };
}

export class App {
  station: Station;
  readonly assets = new AssetStore();
  renderer!: SceneRenderer;
  sim: ProgramSimulator;
  processSim: ProcessSimulator;
  fleets = new Map<string, FleetManager>();
  readonly events = new EventBus<AppEvents>();
  activeProgram: Program | null = null;
  activeRobot: Robot | null = null;
  private undoStack: string[] = [];
  private redoStack: string[] = [];
  private lastSnapshot = '';
  simSpeed = 1;
  /** Global sim clock for fleet & process simulation (independent from program playback). */
  worldRunning = false;
  worldTime = 0;
  logs: Array<{ t: number; text: string; level: 'info' | 'warn' | 'error' }> = [];

  constructor() {
    this.station = new Station('New station');
    this.sim = new ProgramSimulator(this.station);
    this.processSim = new ProcessSimulator(this.station);
  }

  /** Collision checking during program validation. */
  checkCollisions = false;

  mount(host: HTMLElement): void {
    this.renderer = new SceneRenderer(host, this.station, this.assets);
    this.renderer.onSelect = (item, additive) => this.select(item, additive);
    this.renderer.onPoseEdit = (item, pose) => { item.setPoseAbs(pose); if (item instanceof MobileRobot) item.syncStateFromPose(); };
    this.renderer.onAnimate((dt) => this.tick(dt));
    this.snapshot();
  }

  // -- Logging -------------------------------------------------------------
  log(text: string, level: 'info' | 'warn' | 'error' = 'info'): void {
    this.logs.push({ t: Date.now(), text, level });
    if (this.logs.length > 1000) this.logs.shift();
    this.events.emit('log', { text, level });
    if (level !== 'info') toast(text, level === 'warn' ? 'warn' : 'error');
  }

  // -- Station lifecycle ---------------------------------------------------
  setStation(st: Station): void {
    this.station = st;
    this.sim = new ProgramSimulator(st);
    this.processSim = new ProcessSimulator(st);
    this.fleets.clear();
    this.activeProgram = st.itemsOfType<Program>(ItemType.PROGRAM)[0] ?? null;
    this.activeRobot = st.itemsOfType<Robot>(ItemType.ROBOT)[0] ?? null;
    this.renderer?.bindStation(st);
    this.undoStack = [];
    this.redoStack = [];
    this.snapshot();
    this.events.emit('stationChanged', { station: st });
    this.events.emit('activeProgram', { program: this.activeProgram });
    this.events.emit('activeRobot', { robot: this.activeRobot });
    setTimeout(() => this.renderer?.fitAll(), 50);
  }

  newStation(): void {
    this.setStation(new Station('New station'));
  }

  // -- Undo/redo (snapshot based) --------------------------------------------
  snapshot(): void {
    const s = JSON.stringify(this.station.serialize());
    if (s === this.lastSnapshot) return;
    if (this.lastSnapshot) this.undoStack.push(this.lastSnapshot);
    if (this.undoStack.length > 50) this.undoStack.shift();
    this.redoStack = [];
    this.lastSnapshot = s;
    this.events.emit('undo', { canUndo: this.undoStack.length > 0, canRedo: this.redoStack.length > 0 });
  }
  /** Run a mutating command and record an undo step. */
  cmd<T>(fn: () => T): T {
    const r = fn();
    this.snapshot();
    return r;
  }
  undo(): void {
    const s = this.undoStack.pop();
    if (!s) return;
    this.redoStack.push(this.lastSnapshot);
    this.restore(s);
  }
  redo(): void {
    const s = this.redoStack.pop();
    if (!s) return;
    this.undoStack.push(this.lastSnapshot);
    this.restore(s);
  }
  private restore(s: string) {
    this.lastSnapshot = s;
    const st = loadStation(JSON.parse(s), this.assets);
    const progId = this.activeProgram?.id, robId = this.activeRobot?.id;
    this.station = st;
    this.sim = new ProgramSimulator(st);
    this.processSim = new ProcessSimulator(st);
    this.fleets.clear();
    this.renderer?.bindStation(st);
    this.activeProgram = (progId && (st.findById(progId) as Program)) || st.itemsOfType<Program>(ItemType.PROGRAM)[0] || null;
    this.activeRobot = (robId && (st.findById(robId) as Robot)) || st.itemsOfType<Robot>(ItemType.ROBOT)[0] || null;
    this.events.emit('stationChanged', { station: st });
    this.events.emit('activeProgram', { program: this.activeProgram });
    this.events.emit('activeRobot', { robot: this.activeRobot });
    this.events.emit('undo', { canUndo: this.undoStack.length > 0, canRedo: this.redoStack.length > 0 });
  }

  // -- Selection -----------------------------------------------------------
  select(item: Item | null, additive = false): void {
    this.station.select(item, additive);
    const sel = this.station.selection[0] ?? null;
    if (sel instanceof Program) this.setActiveProgram(sel);
    if (sel instanceof Instruction && sel.parent instanceof Program) this.setActiveProgram(sel.parent);
    if (sel instanceof Robot) this.setActiveRobot(sel);
    if (sel instanceof Tool && sel.parent instanceof Robot) this.setActiveRobot(sel.parent);
    this.renderer?.setGizmo(sel && !(sel instanceof Program) && !(sel instanceof Instruction) && !(sel instanceof CropRow) ? sel : null);
  }
  setActiveProgram(p: Program | null): void {
    if (this.activeProgram === p) return;
    this.activeProgram = p;
    this.events.emit('activeProgram', { program: p });
    this.previewProgram();
  }
  setActiveRobot(r: Robot | null): void {
    if (this.activeRobot === r) return;
    this.activeRobot = r;
    this.events.emit('activeRobot', { robot: r });
  }

  // -- Item creation --------------------------------------------------------
  addRobotFromLibrary(id: string, parent: Item = this.station): Robot {
    return this.cmd(() => {
      const r = createRobotFromLibrary(id);
      const n = this.station.itemsOfType(ItemType.ROBOT).length;
      if (n) r.setPose(transl(n * 1500, 0, 0));
      parent.addChild(r);
      const tool = new Tool('Tool 1');
      tool.setPoseTool(transl(0, 0, 100));
      r.addChild(tool);
      r.setTool(tool);
      this.setActiveRobot(r);
      this.select(r);
      return r;
    });
  }
  addFrame(parent: Item = this.station, name?: string): Frame {
    return this.cmd(() => { const f = parent.addChild(new Frame(name ?? `Frame ${this.station.itemsOfType(ItemType.FRAME).length + 1}`)); this.select(f); return f; });
  }
  addTarget(parent?: Item, name?: string): Target | null {
    const robot = this.activeRobot;
    const frame = parent ?? robot?.activeFrame() ?? this.station;
    return this.cmd(() => {
      const t = new Target(name ?? `Target ${this.station.itemsOfType(ItemType.TARGET).length + 1}`);
      if (robot) {
        const tcpAbs = robot.poseTCPAbs();
        frame.addChild(t);
        t.setPoseAbs(tcpAbs);
        t.setJoints(robot.joints());
        t.robotId = robot.id;
      } else frame.addChild(t);
      this.select(t);
      return t;
    });
  }
  addProgram(robot: Robot | MobileRobot | null = this.activeRobot, name?: string): Program {
    return this.cmd(() => {
      const p = this.station.addChild(new Program(name ?? `Prog ${this.station.itemsOfType(ItemType.PROGRAM).length + 1}`));
      if (robot) p.setRobot(robot);
      this.setActiveProgram(p);
      this.select(p);
      return p;
    });
  }
  addObjectPrimitive(kind: 'box' | 'cylinder' | 'sphere', size: number[] = [500, 500, 500], parent: Item = this.station): SceneObject {
    return this.cmd(() => {
      const o = new SceneObject(kind[0].toUpperCase() + kind.slice(1));
      o.geometry = [kind === 'box' ? { primitive: { kind: 'box', size: [size[0], size[1], size[2]] }, origin: Array.from(transl(0, 0, size[2] / 2)), color: '#8ca0b3' } : kind === 'cylinder' ? { primitive: { kind: 'cylinder', radius: size[0], length: size[2] }, origin: Array.from(transl(0, 0, size[2] / 2)), color: '#8ca0b3' } : { primitive: { kind: 'sphere', radius: size[0] }, origin: Array.from(transl(0, 0, size[0])), color: '#8ca0b3' }];
      parent.addChild(o);
      this.select(o);
      return o;
    });
  }
  addMobileRobot(preset: 'amr' | 'tractor' | 'harvester' | 'sprayer' | 'scout' = 'amr'): MobileRobot {
    return this.cmd(() => {
      const m = new MobileRobot(preset === 'amr' ? 'AMR' : preset === 'tractor' ? 'Tractor' : preset === 'harvester' ? 'Harvest platform' : preset === 'sprayer' ? 'Sprayer' : 'Scout rover');
      if (preset === 'tractor') Object.assign(m.kin, { drive: 'ackermann', wheelBase: 2200, track: 1600, wheelRadius: 600, maxSpeed: 3000, maxYawRate: 40, footprint: [3800, 1900, 1800], minTurnRadius: 3500 });
      if (preset === 'harvester') { Object.assign(m.kin, { drive: 'tracked', wheelBase: 1800, track: 1400, wheelRadius: 300, maxSpeed: 1200, footprint: [2600, 1600, 1400] }); m.capabilities = ['harvest', 'transport']; m.color = '#c92a2a'; }
      if (preset === 'sprayer') { Object.assign(m.kin, { drive: 'ackermann', wheelBase: 1800, track: 1300, wheelRadius: 400, maxSpeed: 2500, footprint: [3000, 1500, 1500], minTurnRadius: 3000 }); m.capabilities = ['spray', 'transport']; m.color = '#1971c2'; }
      if (preset === 'scout') { Object.assign(m.kin, { drive: 'differential', wheelBase: 500, track: 450, wheelRadius: 150, maxSpeed: 2000, footprint: [700, 500, 400] }); m.capabilities = ['scout', 'pollinate']; m.color = '#f08c00'; m.battery.capacityWh = m.battery.levelWh = 600; }
      const n = this.station.itemsOfType(ItemType.MOBILE_ROBOT).length;
      m.setPose2D(n * 2500, -3000, 0);
      this.station.addChild(m);
      this.select(m);
      return m;
    });
  }

  deleteItems(items: Item[]): void {
    this.cmd(() => {
      for (const it of items) if (it !== this.station) it.delete();
      if (items.includes(this.activeProgram as Item)) this.setActiveProgram(null);
      if (items.includes(this.activeRobot as Item)) this.setActiveRobot(null);
      this.select(null);
    });
  }

  // -- Program helpers ------------------------------------------------------
  /** Teach current robot position as a new target and append a move to the active program. */
  teach(moveType: 'MoveJ' | 'MoveL' = 'MoveJ'): void {
    const robot = this.activeRobot;
    if (!robot) return this.log('Select a robot first', 'warn');
    const prog = this.activeProgram ?? this.addProgram(robot);
    const t = this.addTarget();
    if (!t) return;
    this.cmd(() => {
      if (moveType === 'MoveJ') prog.addMoveJ(t); else prog.addMoveL(t);
      if (moveType === 'MoveJ') t.setAsJointTarget();
    });
    this.previewProgram();
  }

  /** Compile the active program and show its trajectory. */
  previewProgram(): void {
    const p = this.activeProgram;
    if (!p) { this.renderer?.showTrajectoryPreview(null, []); return; }
    const robot = p.robot();
    if (!(robot instanceof Robot)) { this.renderer?.showTrajectoryPreview(null, []); return; }
    const q0 = robot.joints();
    this.sim.collisionOptions = { enabled: this.checkCollisions, assets: this.assets, sampleStep: 0.1 };
    const res = this.sim.compile(p);
    this.renderer?.setCollisionHighlight(this.sim.collisions.flatMap((c) => c.pairs.flatMap((pr) => [pr.a.item.id, pr.b.item.id])));
    const trajs = this.sim.steps.filter((s) => s.trajectory).map((s) => s.trajectory as Trajectory);
    this.renderer?.showTrajectoryPreview(robot, trajs);
    robot.setJoints(q0);
    this.events.emit('simulation', { playing: this.sim.playing, time: 0, duration: this.sim.duration });
    void res;
  }

  runProgram(p: Program | null = this.activeProgram): void {
    if (!p) return this.log('No program selected', 'warn');
    this.sim.collisionOptions = { enabled: this.checkCollisions, assets: this.assets, sampleStep: 0.1 };
    const res = this.sim.compile(p);
    this.renderer?.setCollisionHighlight(this.sim.collisions.flatMap((c) => c.pairs.flatMap((pr) => [pr.a.item.id, pr.b.item.id])));
    for (const pr of res.problems) this.log(`${p.name}: ${pr.message}`, pr.severity === 'error' ? 'error' : 'warn');
    this.sim.speedFactor = this.simSpeed;
    this.sim.play();
    this.events.emit('simulation', { playing: true, time: 0, duration: this.sim.duration });
  }
  pauseProgram(): void { this.sim.pause(); this.events.emit('simulation', { playing: false, time: this.sim.time, duration: this.sim.duration }); }
  stopProgram(): void { this.sim.stop(); this.events.emit('simulation', { playing: false, time: 0, duration: this.sim.duration }); }
  seekProgram(t: number): void { this.sim.seek(t); this.events.emit('simulation', { playing: this.sim.playing, time: this.sim.time, duration: this.sim.duration }); }

  /** Move the active robot to a target instantly (or preview MoveJ/MoveL path). */
  moveRobotTo(target: Target, linear = false): void {
    const robot = this.activeRobot;
    if (!robot) return;
    const q0 = robot.joints();
    const q1 = robot.jointsForTarget(target);
    if (!q1) return this.log(`${target.name} is unreachable`, 'warn');
    const tr = linear ? planMoveL(robot, q0, robot.solveFK(q1), robot.motion.speedLinear, robot.motion.accelLinear) : planMoveJ(robot, q0, q1, robot.motion.speedJoints, robot.motion.accelJoints);
    if (!tr.ok) this.log(tr.error ?? 'motion failed', 'warn');
    robot.setJoints(q1);
    this.renderer?.showTrajectoryPreview(robot, [tr]);
  }

  /** Check the current static station for collisions and report them. */
  checkStationCollisions(): number {
    const { checkCollisions } = require_collision();
    const pairs = checkCollisions(this.station, { assets: this.assets });
    this.renderer?.setCollisionHighlight(pairs.flatMap((p) => [p.a.item.id, p.b.item.id]));
    if (!pairs.length) this.log('No collisions in the current state');
    for (const p of pairs.slice(0, 10)) this.log(`Collision: ${p.a.item.name}${p.a.part !== p.a.item.name ? '/' + p.a.part : ''} × ${p.b.item.name}${p.b.part !== p.b.item.name ? '/' + p.b.part : ''} (${p.depth.toFixed(0)} mm)`, 'warn');
    return pairs.length;
  }

  // -- World simulation (fleet + process) --------------------------------------
  fleetManager(fleet: FleetItem): FleetManager {
    let fm = this.fleets.get(fleet.id);
    if (!fm || fm.fleet !== fleet) { fm = new FleetManager(this.station, fleet); this.fleets.set(fleet.id, fm); fm.events.on('log', (e) => this.log(`[${fleet.name}] ${e.text}`, e.level)); }
    return fm;
  }
  startWorld(): void { this.worldRunning = true; }
  pauseWorld(): void { this.worldRunning = false; }
  resetWorld(): void {
    this.worldRunning = false;
    this.worldTime = 0;
    this.processSim.reset();
    for (const f of this.station.itemsOfType<FleetItem>(ItemType.FLEET)) { f.tasks = []; this.fleets.delete(f.id); }
    for (const m of this.station.itemsOfType<MobileRobot>(ItemType.MOBILE_ROBOT)) { m.state.path = null; m.state.taskId = null; m.state.status = 'idle'; m.state.v = 0; if (m.home) m.setPose2D(m.home.x, m.home.y, m.home.theta); }
    for (const mi of this.station.itemsOfType<MissionItem>(ItemType.MISSION)) { mi.taskIds = []; mi.progress = 0; mi.status = 'draft'; }
  }

  private tick(dt: number): void {
    if (this.sim.playing) {
      this.sim.speedFactor = this.simSpeed;
      const still = this.sim.tick(dt);
      this.events.emit('simulation', { playing: still, time: this.sim.time, duration: this.sim.duration });
    }
    if (this.worldRunning) {
      const step = dt * this.simSpeed;
      // sub-step for stability at high speed factors
      const n = Math.max(1, Math.ceil(step / 0.05));
      const h = step / n;
      for (let i = 0; i < n; i++) {
        this.worldTime += h;
        for (const f of this.station.itemsOfType<FleetItem>(ItemType.FLEET)) this.fleetManager(f).step(h);
        if (this.station.itemsOfType(ItemType.COMPONENT).length) this.processSim.step(h);
      }
      for (const mi of this.station.itemsOfType<MissionItem>(ItemType.MISSION)) {
        const f = mi.fleetId ? (this.station.findById(mi.fleetId) as FleetItem | null) : this.station.itemsOfType<FleetItem>(ItemType.FLEET)[0];
        if (f) updateMissionProgress(mi, this.fleetManager(f));
      }
    }
  }

  // -- Files ---------------------------------------------------------------
  saveToJSON(): string {
    return JSON.stringify(saveStation(this.station, this.assets));
  }

  async openFiles(files: File[]): Promise<void> {
    // Load meshes first so URDFs can resolve them
    const meshes = files.filter((f) => /\.(stl|obj|glb|gltf)$/i.test(f.name));
    for (const f of meshes) {
      const buf = new Uint8Array(await f.arrayBuffer());
      const ext = f.name.split('.').pop()!.toLowerCase();
      if (ext === 'stl') this.assets.registerRaw(f.name, 'stl', buf, f.name);
      else if (ext === 'obj') this.assets.registerMesh(f.name, parseOBJ(new TextDecoder().decode(buf)), f.name);
    }
    for (const f of files) {
      const name = f.name;
      const ext = name.split('.').pop()?.toLowerCase() ?? '';
      const await_text = ext === 'dh' ? await f.text() : '';
      try {
        if (ext === 'vbstation' || (ext === 'json' && /station/i.test(name))) {
          const data = JSON.parse(await f.text());
          this.setStation(loadStation(data, this.assets));
          this.station.filePath = name;
          this.log(`Opened station ${name}`);
        } else if (ext === 'json') {
          const data = JSON.parse(await f.text());
          if (data.format === 'vbstation' || data.type === ItemType.STATION) { this.setStation(loadStation(data, this.assets)); this.log(`Opened station ${name}`); }
          else if (Array.isArray(data.dh)) { const robot = this.cmd(() => { const r = robotFromDH(data, name.replace(/\.json$/i, '')); this.station.addChild(r); this.select(r); this.setActiveRobot(r); return r; }); this.log(`Imported DH robot ${robot.name} (${robot.dof} DOF)`); }
          else if (data.type === 'FeatureCollection' || data.type === 'Feature') await this.importGeoJSON(data, name);
          else this.log(`Unknown JSON content in ${name}`, 'warn');
        } else if (ext === 'geojson') {
          await this.importGeoJSON(JSON.parse(await f.text()), name);
        } else if (ext === 'urdf' || ext === 'xacro') {
          const text = await f.text();
          const others = new Map(files.map((x) => [x.name, x]));
          const cache = new Map<string, string>();
          for (const [n, x] of others) if (/\.(xacro|urdf)$/i.test(n)) cache.set(n, await x.text());
          const robot = robotFromURDF(text, { resolveInclude: (fn) => cache.get(fn.split('/').pop()!) ?? null });
          this.cmd(() => { this.station.addChild(robot); this.select(robot); this.setActiveRobot(robot); });
          const model = parseURDF(text, { resolveInclude: (fn) => cache.get(fn.split('/').pop()!) ?? null });
          const missing = model.meshes.filter((m) => !this.assets.get(m) && ![...this.assets.entries()].some(([id]) => id.split(/[\\/]/).pop()?.toLowerCase() === m.split(/[\\/]/).pop()?.toLowerCase()));
          this.log(`Imported URDF ${robot.name} (${robot.dof} DOF)${missing.length ? `; ${missing.length} meshes missing — drop the STL files onto the viewport` : ''}`, missing.length ? 'warn' : 'info');
        } else if (ext === 'dh' || (ext === 'json' && false)) {
          const robot = this.cmd(() => { const r = robotFromDH(parseDHText(await_text), name.replace(/\.dh$/i, '')); this.station.addChild(r); this.select(r); this.setActiveRobot(r); return r; });
          this.log(`Imported DH robot ${robot.name} (${robot.dof} DOF)`);
        } else if (ext === 'stl' || ext === 'obj') {
          const a = this.assets.get(name)!;
          this.cmd(() => {
            const o = new SceneObject(name.replace(/\.(stl|obj)$/i, ''));
            o.geometry = [{ mesh: name, origin: Array.from(identity()), color: '#a5b1c2' }];
            if (a.mesh) o.bbox = { min: a.mesh.min, max: a.mesh.max };
            this.station.addChild(o);
            this.select(o);
          });
          this.log(`Imported mesh ${name}`);
        } else if (ext === 'rdk') {
          const { station, report } = await importRdkBestEffort(await f.arrayBuffer(), name.replace(/\.rdk$/i, ''));
          this.setStation(station);
          for (const n of report.notes) this.log(n, 'warn');
          this.log(`RoboDK .rdk best-effort import: ${report.strings.slice(0, 12).join(', ')}${report.strings.length > 12 ? '…' : ''}`);
        } else if (['src', 'mod', 'prg', 'ls', 'script', 'csv', 'txt'].includes(ext)) {
          const text = await f.text();
          const lang = detectLanguage(text, name);
          if (ext === 'csv' || ext === 'txt') {
            if (lang === 'csv' && /^type,name/m.test(text)) { const r = this.cmd(() => importProgram(text, { station: this.station, robot: this.activeRobot, filename: name })); this.setActiveProgram(r.program); this.log(`Imported program ${r.program.name}`); }
            else { const parent = this.activeRobot?.activeFrame() ?? this.station; const n = this.cmd(() => importTargets(text, { parent, namePrefix: name.replace(/\.[^.]+$/, '') })).length; this.log(`Imported ${n} targets from ${name}`); }
          } else {
            const r = this.cmd(() => importProgram(text, { station: this.station, robot: this.activeRobot, filename: name }));
            for (const w of r.warnings) this.log(w, 'warn');
            this.setActiveProgram(r.program);
            this.log(`Imported ${lang.toUpperCase()} program ${r.program.name} (${r.targets.length} targets)`);
          }
        } else if (['obj', 'glb', 'gltf', 'png', 'jpg'].includes(ext)) {
          this.log(`${ext.toUpperCase()} import: convert to STL for now`, 'warn');
        } else this.log(`Unsupported file ${name}`, 'warn');
      } catch (e: any) {
        this.log(`Failed to open ${name}: ${e.message ?? e}`, 'error');
        console.error(e);
      }
    }
    this.previewProgram();
  }

  private async importGeoJSON(data: any, name: string) {
    const { fieldsFromGeoJSON } = await import('./agri/geo');
    const { fields, origin } = fieldsFromGeoJSON(data);
    this.cmd(() => {
      for (const f of fields) {
        const fi = new FieldItem(f.name);
        fi.polygon = f.polygon;
        fi.geoOrigin = origin;
        this.station.addChild(fi);
      }
    });
    this.log(`Imported ${fields.length} field(s) from ${name}`);
  }

  exportProgram(postId: string, program: Program | null = this.activeProgram): PostFile[] {
    if (!program) { this.log('No program to export', 'warn'); return []; }
    const post = getPost(postId);
    if (!post) { this.log(`Unknown post ${postId}`, 'error'); return []; }
    const compiled = compileForPost(this.station, program);
    for (const p of compiled.problems) this.log(p, 'warn');
    return post.generate(compiled);
  }

  exportRoboDKScript(): string {
    return stationToRoboDKScript(this.station);
  }

  posts() {
    return listPosts();
  }
  get library() {
    return ROBOT_LIBRARY;
  }
}

import * as collisionModule from './core/collision/collision';
function require_collision() { return collisionModule; }

export { Item, ItemType, Frame, Target, Tool, SceneObject, Folder, Camera, Robot, Program, Instruction, PathItem, MobileRobot, MapItem, ZoneItem, FleetItem, Component, FieldItem, MissionItem, CropRow, multiply, invert, getPos };
export type { Mat4, StationFile };
