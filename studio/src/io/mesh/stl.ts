/** STL (binary + ASCII) parser producing flat Float32Array positions/normals (mm assumed unless scaled). */
export interface MeshData {
  positions: Float32Array;
  normals: Float32Array;
  /** Axis-aligned bounds. */
  min: [number, number, number];
  max: [number, number, number];
  triangles: number;
}

export function parseSTL(buf: ArrayBuffer, scale = 1): MeshData {
  const bytes = new Uint8Array(buf);
  const isAscii = looksAscii(bytes);
  return isAscii ? parseAsciiSTL(new TextDecoder().decode(bytes), scale) : parseBinarySTL(buf, scale);
}

function looksAscii(b: Uint8Array): boolean {
  if (b.length < 84) return true;
  const head = new TextDecoder().decode(b.slice(0, 5)).toLowerCase();
  if (head !== 'solid') return false;
  // binary files may still start with "solid"; check triangle count consistency
  const n = new DataView(b.buffer, b.byteOffset).getUint32(80, true);
  return 84 + n * 50 !== b.length;
}

function parseBinarySTL(buf: ArrayBuffer, scale: number): MeshData {
  const dv = new DataView(buf);
  const n = dv.getUint32(80, true);
  const positions = new Float32Array(n * 9);
  const normals = new Float32Array(n * 9);
  const min: [number, number, number] = [Infinity, Infinity, Infinity];
  const max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  let off = 84;
  for (let i = 0; i < n; i++) {
    const nx = dv.getFloat32(off, true), ny = dv.getFloat32(off + 4, true), nz = dv.getFloat32(off + 8, true);
    off += 12;
    for (let v = 0; v < 3; v++) {
      const x = dv.getFloat32(off, true) * scale, y = dv.getFloat32(off + 4, true) * scale, z = dv.getFloat32(off + 8, true) * scale;
      off += 12;
      const k = i * 9 + v * 3;
      positions[k] = x; positions[k + 1] = y; positions[k + 2] = z;
      normals[k] = nx; normals[k + 1] = ny; normals[k + 2] = nz;
      if (x < min[0]) min[0] = x; if (y < min[1]) min[1] = y; if (z < min[2]) min[2] = z;
      if (x > max[0]) max[0] = x; if (y > max[1]) max[1] = y; if (z > max[2]) max[2] = z;
    }
    off += 2;
  }
  return { positions, normals, min, max, triangles: n };
}

function parseAsciiSTL(text: string, scale: number): MeshData {
  const pos: number[] = [];
  const nor: number[] = [];
  let cur: number[] = [0, 0, 1];
  const min: [number, number, number] = [Infinity, Infinity, Infinity];
  const max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  const re = /(facet\s+normal\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+))|(vertex\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+))/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(text))) {
    if (m[1]) cur = [+m[2], +m[3], +m[4]];
    else {
      const x = +m[6] * scale, y = +m[7] * scale, z = +m[8] * scale;
      pos.push(x, y, z);
      nor.push(cur[0], cur[1], cur[2]);
      if (x < min[0]) min[0] = x; if (y < min[1]) min[1] = y; if (z < min[2]) min[2] = z;
      if (x > max[0]) max[0] = x; if (y > max[1]) max[1] = y; if (z > max[2]) max[2] = z;
    }
  }
  return { positions: new Float32Array(pos), normals: new Float32Array(nor), min, max, triangles: pos.length / 9 };
}

/** Serialize to binary STL (for export). */
export function writeBinarySTL(positions: Float32Array | number[], header = 'VerticalBot Studio'): ArrayBuffer {
  const n = Math.floor(positions.length / 9);
  const buf = new ArrayBuffer(84 + n * 50);
  const dv = new DataView(buf);
  const hb = new TextEncoder().encode(header.slice(0, 80));
  new Uint8Array(buf, 0, 80).set(hb);
  dv.setUint32(80, n, true);
  let off = 84;
  for (let i = 0; i < n; i++) {
    const k = i * 9;
    const ax = positions[k], ay = positions[k + 1], az = positions[k + 2];
    const bx = positions[k + 3], by = positions[k + 4], bz = positions[k + 5];
    const cx = positions[k + 6], cy = positions[k + 7], cz = positions[k + 8];
    const ux = bx - ax, uy = by - ay, uz = bz - az, vx = cx - ax, vy = cy - ay, vz = cz - az;
    let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const l = Math.hypot(nx, ny, nz) || 1;
    nx /= l; ny /= l; nz /= l;
    dv.setFloat32(off, nx, true); dv.setFloat32(off + 4, ny, true); dv.setFloat32(off + 8, nz, true);
    off += 12;
    for (let v = 0; v < 9; v++) { dv.setFloat32(off, positions[k + v], true); off += 4; }
    dv.setUint16(off, 0, true);
    off += 2;
  }
  return buf;
}
