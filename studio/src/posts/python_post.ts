/**
 * Runs genuine RoboDK Python post processors (files defining `class RobotPost`) in the browser with Pyodide.
 * The studio's robomath shim and post_shim.py are written into the Pyodide file system; the compiled
 * PostProgram events are passed as JSON. Pyodide (≈10 MB) is loaded on first use from jsDelivr.
 */
import { PostProgram, PostFile, PostProcessor, registerPost } from './base';
import robomathSrc from '../../python/robodk/robomath.py?raw';
import shimSrc from '../../python/post_shim.py?raw';

const PYODIDE_URL = 'https://cdn.jsdelivr.net/pyodide/v0.26.4/full/pyodide.js';
let pyodidePromise: Promise<any> | null = null;

async function loadPyodide(): Promise<any> {
  if (pyodidePromise) return pyodidePromise;
  pyodidePromise = (async () => {
    if (!(window as any).loadPyodide) {
      await new Promise<void>((resolve, reject) => { const s = document.createElement('script'); s.src = PYODIDE_URL; s.onload = () => resolve(); s.onerror = () => reject(new Error('Failed to load Pyodide')); document.head.appendChild(s); });
    }
    const py = await (window as any).loadPyodide({ indexURL: PYODIDE_URL.replace(/pyodide\.js$/, '') });
    py.FS.mkdirTree('/studio/robodk');
    py.FS.writeFile('/studio/robodk/__init__.py', '');
    py.FS.writeFile('/studio/robodk/robomath.py', robomathSrc);
    py.FS.writeFile('/studio/robodk/robolink.py', 'ITEM_TYPE_ROBOT = 2\nclass Robolink:\n    pass\n');
    py.FS.writeFile('/studio/post_shim.py', shimSrc);
    await py.runPythonAsync('import sys\nsys.path.insert(0, "/studio")\nimport post_shim');
    return py;
  })();
  return pyodidePromise;
}

/** Serialize PostEvents for the Python side (poses as column-major 16 arrays). */
export function eventsToJSON(p: PostProgram): string {
  return JSON.stringify(p.events.map((e) => { const o: any = { ...e }; for (const k of ['pose', 'via']) if (o[k] instanceof Float64Array) o[k] = Array.from(o[k]); return o; }));
}

/** Run a Python post source against a compiled program. */
export async function runPythonPost(postSource: string, program: PostProgram): Promise<PostFile[]> {
  const py = await loadPyodide();
  py.globals.set('__post_src', postSource);
  py.globals.set('__events', eventsToJSON(program));
  py.globals.set('__robot', program.robotName);
  py.globals.set('__prog', program.name);
  py.globals.set('__dof', program.dof);
  const result = await py.runPythonAsync('import json, post_shim\nres = post_shim.run_from_json(__post_src, __events, __robot, __prog, __dof)\njson.dumps(res)');
  const r = JSON.parse(result);
  const files: PostFile[] = [{ name: `${program.name.replace(/\W+/g, '_')}.${r.extension || 'txt'}`, content: r.program }];
  if (r.log) files.push({ name: `${program.name.replace(/\W+/g, '_')}.log`, content: r.log });
  for (const sub of program.subprograms) files.push(...(await runPythonPost(postSource, sub)));
  return files;
}

/** Register a user-provided Python post as a selectable post processor (async generation via `generateAsync`). */
export function registerPythonPost(id: string, name: string, source: string): PostProcessor & { generateAsync: (p: PostProgram) => Promise<PostFile[]> } {
  const post = {
    id, name: `${name} (Python post)`, brand: 'Custom', extension: 'txt',
    generate: (_p: PostProgram): PostFile[] => [{ name: 'README.txt', content: 'This is a Python post processor: use generateAsync / the Export dialog (runs in Pyodide).' }],
    generateAsync: (p: PostProgram) => runPythonPost(source, p),
  };
  registerPost(post);
  return post;
}
