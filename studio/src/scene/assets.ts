/** Mesh asset registry (renderer-agnostic). Stores raw bytes and parsed geometry. */
import { parseSTL, MeshData } from '../io/mesh/stl';

export interface Asset {
  id: string;
  type: 'stl' | 'glb' | 'obj';
  name?: string;
  source?: Uint8Array;
  mesh?: MeshData;
  /** Optional renderer object cache (three.js BufferGeometry). */
  gpu?: unknown;
}

export class AssetStore {
  private assets = new Map<string, Asset>();
  private listeners = new Set<(id: string) => void>();

  entries(): IterableIterator<[string, Asset]> {
    return this.assets.entries();
  }
  get(id: string): Asset | undefined {
    return this.assets.get(id);
  }
  has(id: string): boolean {
    return this.assets.has(id);
  }
  onChange(fn: (id: string) => void): () => void {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  }
  registerRaw(id: string, type: Asset['type'], bytes: Uint8Array, name?: string, scale = 1): Asset {
    const a: Asset = { id, type, name, source: bytes };
    if (type === 'stl') a.mesh = parseSTL(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength) as ArrayBuffer, scale);
    this.assets.set(id, a);
    for (const l of this.listeners) l(id);
    return a;
  }
  registerMesh(id: string, mesh: MeshData, name?: string): Asset {
    const a: Asset = { id, type: 'stl', name, mesh };
    this.assets.set(id, a);
    for (const l of this.listeners) l(id);
    return a;
  }
  remove(id: string): void {
    this.assets.delete(id);
  }
}
