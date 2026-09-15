/**
 * VerticalBot Studio server.
 *
 *  - WebSocket JSON-RPC endpoint (default port 20500, same as RoboDK's API port) speaking the
 *    RoboDK-compatible Robolink API. Python clients use python/robodk/robolink.py (drop-in).
 *  - Relay mode: when a browser tab is connected as "host", every request is forwarded to it and
 *    executed against the live 3D station (what the user sees). Otherwise the server runs a
 *    headless station itself (CI, batch post-processing, fleet simulations on a server).
 *  - HTTP: GET /station.json (headless station), POST /station.json (replace), GET /health.
 *
 * Run: npm run server   (node --experimental-strip-types server/index.ts)
 */
import { WebSocketServer, WebSocket } from 'ws';
import http from 'node:http';
import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { Station } from '../src/core/items/item.ts';
import { Robolink } from '../src/api/robolink.ts';
import { executeRpc, RpcRequest } from '../src/api/rpc.ts';
import { loadStation, saveStation } from '../src/io/station-file.ts';
import '../src/posts/index.ts';

const PORT = Number(process.env.STUDIO_PORT ?? 20500);
const STATION_FILE = process.env.STUDIO_STATION ?? '';

let station = new Station('Headless station');
if (STATION_FILE && existsSync(STATION_FILE)) {
  station = loadStation(JSON.parse(readFileSync(STATION_FILE, 'utf8')));
  console.log(`[studio-server] loaded ${STATION_FILE}`);
}
let RDK = new Robolink(station);
RDK.onMessage = (m) => console.log(`[studio-server] message: ${m}`);

let host: WebSocket | null = null;
const pending = new Map<string, { ws: WebSocket; id: number | string }>();
let seq = 0;

const httpServer = http.createServer((req, res) => {
  if (req.url === '/health') { res.writeHead(200, { 'content-type': 'application/json' }); res.end(JSON.stringify({ ok: true, host: !!host, station: station.name })); return; }
  if (req.url === '/station.json' && req.method === 'GET') { res.writeHead(200, { 'content-type': 'application/json' }); res.end(JSON.stringify(saveStation(station))); return; }
  if (req.url === '/station.json' && req.method === 'POST') {
    let body = '';
    req.on('data', (c) => (body += c));
    req.on('end', () => {
      try { station = loadStation(JSON.parse(body)); RDK = new Robolink(station); res.writeHead(200); res.end('ok'); } catch (e: any) { res.writeHead(400); res.end(String(e.message)); }
    });
    return;
  }
  res.writeHead(404); res.end();
});

const wss = new WebSocketServer({ server: httpServer });
wss.on('connection', (ws) => {
  let role: 'client' | 'host' = 'client';
  ws.on('message', (data) => {
    let msg: any;
    try { msg = JSON.parse(String(data)); } catch { return; }
    if (msg.role === 'host') { role = 'host'; host = ws; console.log('[studio-server] browser host connected'); return; }
    if (role === 'host') {
      // response from the browser for a relayed request
      const key = String(msg.id);
      const p = pending.get(key);
      if (p) { pending.delete(key); p.ws.send(JSON.stringify({ ...msg, id: p.id })); }
      return;
    }
    const req = msg as RpcRequest;
    if (host && host.readyState === WebSocket.OPEN) {
      const key = `r${++seq}`;
      pending.set(key, { ws, id: req.id });
      host.send(JSON.stringify({ ...req, id: key }));
      setTimeout(() => { if (pending.has(key)) { pending.delete(key); ws.send(JSON.stringify({ id: req.id, error: 'host timeout' })); } }, 30000);
    } else {
      const res = executeRpc(RDK, req);
      ws.send(JSON.stringify(res));
      if (STATION_FILE && req.method !== 'Item' && req.method !== 'ItemList') {
        try { writeFileSync(STATION_FILE, JSON.stringify(saveStation(station))); } catch { /* ignore */ }
      }
    }
  });
  ws.on('close', () => { if (ws === host) { host = null; console.log('[studio-server] browser host disconnected'); } });
});

httpServer.listen(PORT, () => console.log(`[studio-server] listening on ws://localhost:${PORT} (RoboDK-compatible API) — open the studio with ?server=ws://localhost:${PORT} to relay to the browser`));
