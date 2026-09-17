/**
 * Document model behind the graphical editors: automata (DES documents) and Petri nets edited as graphs and
 * written back to the DSL. Layout (node positions) is part of the document (`layout` lines), so a diagram survives
 * save / load and round-trips through the text editor. Pure functions — the DOM editor in ui/graph_editor.ts calls
 * them and the tests exercise them directly.
 */
import type { AutomatonSpec, Transition } from './des';
import type { PetriNetSpec, PlaceKind } from './petri';
import type { DesDoc, ActionBinding } from './dsl';

export type XY = [number, number];
export type Layout = Record<string, XY>;
export interface PetriDoc { spec: PetriNetSpec; checks: DesDoc['checks']; horizon: number; layout: Layout; actions: ActionBinding[] }

const fmtNum = (v: number) => Number(v.toFixed(1)).toString();
const q = (s: string) => (/\s/.test(s) ? `"${s}"` : s);
const checkLine = (c: { kind: 'ltl' | 'ctl'; formula: string; name?: string }) => `check ${c.kind}${c.name ? ` "${c.name}"` : ''} ${c.formula}`;
const qv = (v: string | number | boolean) => (typeof v === 'string' && /\s/.test(v) ? `"${v}"` : String(v));
export const actionLine = (a: ActionBinding) => `action ${a.target} ${a.kind}=${qv(a.value)}${a.robot ? ` robot=${qv(a.robot)}` : ''}${a.component ? ` component=${qv(a.component)}` : ''}${a.done ? ` done=${a.done}` : ''}${Object.entries(a.args).map(([k, v]) => ` ${k}=${qv(v)}`).join('')}`;
/** Set (or remove with null) the action bound to a state / transition. */
export function setAction(list: ActionBinding[], target: string, binding: Omit<ActionBinding, 'target'> | null): void {
  const i = list.findIndex((a) => a.target === target);
  if (!binding) { if (i >= 0) list.splice(i, 1); return; }
  const b: ActionBinding = { target, ...binding, args: binding.args ?? {} };
  if (i >= 0) list[i] = b; else list.push(b);
}
export const actionOf = (list: ActionBinding[], target: string): ActionBinding | undefined => list.find((a) => a.target === target);
/** Rename the target of bindings (state or transition renamed / removed). */
export function retargetActions(list: ActionBinding[], from: string, to: string | null): void { for (let i = list.length - 1; i >= 0; i--) { if (list[i].target !== from) continue; if (to === null) list.splice(i, 1); else list[i].target = to; } }

/** Fresh identifier `base`, `base1`, `base2`, … not in `existing`. */
export function uniqueName(existing: Iterable<string>, base: string): string {
  const set = new Set(existing); if (!set.has(base)) return base;
  for (let i = 1; ; i++) if (!set.has(`${base}${i}`)) return `${base}${i}`;
}

// ---------------------------------------------------------------------------------------------
// Automata
// ---------------------------------------------------------------------------------------------

/** All states of an automaton in a stable order: initial, declared, then in order of appearance in transitions. */
export function automatonStates(a: AutomatonSpec): string[] {
  const out: string[] = []; const add = (s: string) => { if (s && !out.includes(s)) out.push(s); };
  add(a.initial); for (const s of a.states ?? []) add(s); for (const t of a.transitions) { add(t.from); add(t.to); } for (const s of a.marked) add(s);
  return out;
}

/** Transitions grouped per (from, to) pair with the list of events — one edge of the diagram. */
export function automatonEdges(a: AutomatonSpec): Array<{ from: string; to: string; events: string[] }> {
  const map = new Map<string, { from: string; to: string; events: string[] }>();
  for (const t of a.transitions) { const k = `${t.from}→${t.to}`; let e = map.get(k); if (!e) { e = { from: t.from, to: t.to, events: [] }; map.set(k, e); } if (!e.events.includes(t.event)) e.events.push(t.event); }
  return [...map.values()];
}

export function automatonToDsl(a: AutomatonSpec, keyword: 'automaton' | 'spec' = 'automaton'): string {
  const states = automatonStates(a);
  const inTrans = new Set(a.transitions.flatMap((t) => [t.from, t.to]));
  const out = [`${keyword} ${q(a.name)}`];
  if (a.events.length) out.push(`  events ${a.events.join(' ')}`);
  out.push(`  initial ${a.initial}`);
  if (a.marked.length) out.push(`  marked ${a.marked.join(' ')}`);
  const lonely = states.filter((s) => !inTrans.has(s) && s !== a.initial);
  if (lonely.length) out.push(`  states ${lonely.join(' ')}`);
  for (const e of automatonEdges(a)) out.push(`  ${e.from} -${e.events.join(',')}-> ${e.to}`);
  return out.join('\n');
}

export function desToDsl(doc: DesDoc): string {
  const out = [`des ${doc.name}`];
  for (const a of doc.plant) out.push(automatonToDsl(a, 'automaton'));
  for (const a of doc.specs) out.push(automatonToDsl(a, 'spec'));
  if (doc.uncontrollable.length) out.push(`uncontrollable ${doc.uncontrollable.join(' ')}`);
  if (doc.unobservable.length) out.push(`unobservable ${doc.unobservable.join(' ')}`);
  if (doc.faults.length) out.push(`faults ${doc.faults.join(' ')}`);
  for (const c of doc.checks) out.push(checkLine(c));
  for (const a of doc.actions ?? []) out.push(actionLine(a));
  for (const [auto, L] of Object.entries(doc.layout ?? {})) { const entries = Object.entries(L); if (entries.length) out.push(`layout ${q(auto)} ${entries.map(([s, p]) => `${s}=${fmtNum(p[0])},${fmtNum(p[1])}`).join(' ')}`); }
  return out.join('\n');
}

export function newAutomaton(name: string): AutomatonSpec { return { name, events: [], initial: 'q0', marked: ['q0'], transitions: [], states: ['q0'] }; }

export function addState(a: AutomatonSpec, name?: string): string {
  const s = name ?? uniqueName(automatonStates(a), 'q');
  if (automatonStates(a).includes(s)) throw new Error(`state ${s} exists`);
  (a.states ??= []).push(s); if (!a.initial) { a.initial = s; }
  return s;
}
export function removeState(a: AutomatonSpec, s: string): void {
  a.transitions = a.transitions.filter((t) => t.from !== s && t.to !== s);
  a.marked = a.marked.filter((m) => m !== s); a.states = (a.states ?? []).filter((x) => x !== s);
  if (a.initial === s) { a.initial = ''; a.initial = automatonStates(a)[0] ?? ''; }
}
export function renameState(a: AutomatonSpec, from: string, to: string): void {
  if (from === to) return; if (!/^\S+$/.test(to) || /[-,>]/.test(to)) throw new Error('a state name has no spaces, commas, dashes or >');
  if (automatonStates(a).includes(to)) throw new Error(`state ${to} exists`);
  for (const t of a.transitions) { if (t.from === from) t.from = to; if (t.to === from) t.to = to; }
  a.marked = a.marked.map((m) => (m === from ? to : m)); a.states = (a.states ?? []).map((x) => (x === from ? to : x)); if (a.initial === from) a.initial = to;
}
export function setInitial(a: AutomatonSpec, s: string): void { a.initial = s; }
export function toggleMarked(a: AutomatonSpec, s: string, on?: boolean): void { const has = a.marked.includes(s); const want = on ?? !has; if (want && !has) a.marked.push(s); if (!want && has) a.marked = a.marked.filter((m) => m !== s); }
/** Replace the events of the edge from → to (empty list removes the edge). Events are added to the alphabet. */
export function setEdgeEvents(a: AutomatonSpec, from: string, to: string, events: string[]): void {
  const evs = [...new Set(events.map((e) => e.trim()).filter(Boolean))];
  a.transitions = a.transitions.filter((t) => !(t.from === from && t.to === to));
  for (const e of evs) { if (/[\s,>]/.test(e)) throw new Error(`bad event name '${e}'`); a.transitions.push({ from, event: e, to }); if (!a.events.includes(e)) a.events.push(e); }
}
export function edgeEvents(a: AutomatonSpec, from: string, to: string): string[] { return a.transitions.filter((t) => t.from === from && t.to === to).map((t) => t.event); }
/** The alphabet is the declared `events` line plus every event used by a transition (a spec may declare events it never uses, e.g. E4). */
export function setAlphabet(a: AutomatonSpec, events: string[]): void { const used = new Set(a.transitions.map((t) => t.event)); a.events = [...new Set([...events.map((e) => e.trim()).filter(Boolean), ...used])]; }
export const transitionsOf = (a: AutomatonSpec): Transition[] => a.transitions;

// ---------------------------------------------------------------------------------------------
// Petri nets
// ---------------------------------------------------------------------------------------------

export function petriToDsl(doc: PetriDoc): string {
  const { spec } = doc;
  const out = [`petri ${spec.name}${doc.horizon && doc.horizon !== 300 ? ` horizon=${doc.horizon}` : ''}`];
  for (const p of spec.places) out.push(`place ${p.id}${p.tokens ? ` tokens=${p.tokens}` : ''}${p.kind ? ` kind=${p.kind}` : ''}${p.capacity !== undefined ? ` capacity=${p.capacity}` : ''}${p.label ? ` "${p.label}"` : ''}`);
  for (const t of spec.transitions) out.push(`transition ${t.id}${t.delay !== undefined ? ` delay=${t.delay}` : ''}${t.rate !== undefined ? ` rate=${t.rate}` : ''}${t.immediate ? ' immediate' : ''}${t.weight !== undefined ? ` weight=${t.weight}` : ''}${t.priority !== undefined ? ` priority=${t.priority}` : ''}${t.label ? ` "${t.label}"` : ''}`);
  const isT = new Set(spec.transitions.map((t) => t.id));
  const plain = (a: { weight?: number; inhibitor?: boolean }) => !a.inhibitor && (a.weight === undefined || a.weight === 1);
  for (const t of spec.transitions) {
    const ins = spec.arcs.filter((a) => a.to === t.id && plain(a)).map((a) => a.from); const outs = spec.arcs.filter((a) => a.from === t.id && plain(a)).map((a) => a.to);
    if (ins.length && outs.length) out.push(`arc ${ins.join(', ')} -> ${t.id} -> ${outs.join(', ')}`);
    else if (ins.length) out.push(`arc ${ins.join(', ')} -> ${t.id}`);
    else if (outs.length) out.push(`arc ${t.id} -> ${outs.join(', ')}`);
  }
  for (const a of spec.arcs) if (!plain(a) || (!isT.has(a.from) && !isT.has(a.to))) out.push(`arc ${a.from} ${a.inhibitor ? '-o' : '->'} ${a.to}${a.weight !== undefined && a.weight !== 1 ? ` weight=${a.weight}` : ''}`);
  for (const c of doc.checks) out.push(checkLine(c));
  for (const a of doc.actions ?? []) out.push(actionLine(a));
  const entries = Object.entries(doc.layout ?? {}); if (entries.length) out.push(`layout ${entries.map(([s, p]) => `${s}=${fmtNum(p[0])},${fmtNum(p[1])}`).join(' ')}`);
  return out.join('\n');
}

export const petriIds = (spec: PetriNetSpec): string[] => [...spec.places.map((p) => p.id), ...spec.transitions.map((t) => t.id)];
export const isPlace = (spec: PetriNetSpec, id: string): boolean => spec.places.some((p) => p.id === id);
export const isTransition = (spec: PetriNetSpec, id: string): boolean => spec.transitions.some((t) => t.id === id);

export function addPlace(spec: PetriNetSpec, id?: string, kind?: PlaceKind): string { const p = id ?? uniqueName(petriIds(spec), 'p'); if (petriIds(spec).includes(p)) throw new Error(`${p} exists`); spec.places.push({ id: p, tokens: 0, kind }); return p; }
export function addPetriTransition(spec: PetriNetSpec, id?: string): string { const t = id ?? uniqueName(petriIds(spec), 't'); if (petriIds(spec).includes(t)) throw new Error(`${t} exists`); spec.transitions.push({ id: t }); return t; }
export function removePetriNode(spec: PetriNetSpec, id: string): void { spec.places = spec.places.filter((p) => p.id !== id); spec.transitions = spec.transitions.filter((t) => t.id !== id); spec.arcs = spec.arcs.filter((a) => a.from !== id && a.to !== id); }
export function renamePetriNode(spec: PetriNetSpec, from: string, to: string): void {
  if (from === to) return; if (!/^[A-Za-z_][\w.]*$/.test(to)) throw new Error('an identifier: letters, digits, _ and .'); if (petriIds(spec).includes(to)) throw new Error(`${to} exists`);
  for (const p of spec.places) if (p.id === from) p.id = to; for (const t of spec.transitions) if (t.id === from) t.id = to; for (const a of spec.arcs) { if (a.from === from) a.from = to; if (a.to === from) a.to = to; }
}
/** Add or update an arc; a place must connect to a transition. */
export function setArc(spec: PetriNetSpec, from: string, to: string, opts: { weight?: number; inhibitor?: boolean } = {}): void {
  if (isPlace(spec, from) === isPlace(spec, to)) throw new Error('an arc connects a place and a transition');
  if (opts.inhibitor && !isPlace(spec, from)) throw new Error('an inhibitor arc goes from a place to a transition');
  let a = spec.arcs.find((x) => x.from === from && x.to === to); if (!a) { a = { from, to }; spec.arcs.push(a); }
  a.weight = opts.weight !== undefined && opts.weight !== 1 ? opts.weight : undefined; a.inhibitor = opts.inhibitor || undefined;
}
export function removeArc(spec: PetriNetSpec, from: string, to: string): void { spec.arcs = spec.arcs.filter((a) => !(a.from === from && a.to === to)); }
