// Mock NeoPixel Studio device for testing: implements the firmware's WebSocket
// protocol (hello/ack/nack, base64 blobs) and a faithful port of applyLayers()
// from NeoPixelStudio.ino, so the exact bytes the UI pushes can be inspected.
import net from "node:net";
import crypto from "node:crypto";

const PORT = 82;
const MAX_LAYERS = 12;
const LEDS = 16;

const GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
const b64dec = s => Buffer.from(s, "base64");

// ---- faithful port of applyLayers + renderFrame from the .ino ----
let layers = [];   // {type,effect,c1,c2,speedQ,bpmQ,start,end,blend,pixels,hasPixels}
let nacks = [];

function applyLayers(b) {
  const len = b.length;
  if (len < 1) return;
  let count = b[0];
  if (count > MAX_LAYERS) count = MAX_LAYERS;
  const need = 1 + count * 11;
  if (len < need) { nacks.push("layer blob short"); return; }

  layers = [];
  for (let i = 0; i < count; i++) {
    const p = b.subarray(1 + i * 11, 1 + i * 11 + 11);
    const L = {
      type: p[0] > 2 ? 0 : p[0],
      effect: p[1],
      c1: [p[2], p[3], p[4]], c2: [p[5], p[6], p[7]],
      speedQ: p[8] < 1 ? 1 : p[8],
      bpmQ: p[9],
      start: p[10] & 0x0f,
      end: (p[10] >> 4) & 0x0f,
      blend: 0, hasPixels: false,
      pixels: new Uint8Array(LEDS * 3),
    };
    if (L.end >= LEDS) L.end = LEDS - 1;
    if (L.start > L.end) L.start = L.end;
    layers.push(L);
  }
  let off = need;
  if (len > off && b[off] === 0xfe) {
    off++;
    while (off + 2 <= len) {
      const li = b[off++], pc = b[off++];
      if (off + pc * 3 > len) break;
      if (li < layers.length) {
        const s = Math.min(pc, LEDS);
        layers[li].pixels.fill(0);
        layers[li].pixels.set(b.subarray(off, off + s * 3));
        layers[li].hasPixels = true;
      }
      off += pc * 3;
    }
  }
  if (len >= off + 1 && b[off] === 0xfd) {
    off++;
    for (let i = 0; i < layers.length && off + i < len; i++) {
      const m = b[off + i];
      layers[i].blend = m > 3 ? 0 : m;
    }
  }
}

function renderStatic() {
  // static frame: solid + pixels layers only (effects are animated)
  const dst = new Array(LEDS * 3).fill(0);
  for (const L of layers) {
    if (L.type === 1) return "<animated effect: " + L.effect + ">";
    if (L.type === 2 && L.hasPixels) {
      for (let i = L.start; i <= L.end; i++)
        for (let k = 0; k < 3; k++) dst[i * 3 + k] = L.pixels[i * 3 + k];
    } else if (L.type === 0) {
      for (let i = L.start; i <= L.end; i++)
        for (let k = 0; k < 3; k++) dst[i * 3 + k] = L.c1[k];
    }
  }
  for (let li = 0; li < layers.length; li++) {
    const L = layers[li];
    if (L.type === 2 && L.hasPixels) {
      for (let i = 0; i < LEDS; i++) {
        const o = i * 3;
        if (L.pixels[o] | L.pixels[o + 1] | L.pixels[o + 2])
          for (let k = 0; k < 3; k++) dst[o + k] = L.pixels[o + k];
      }
      break;
    }
  }
  return dst.map((v, i) => (i % 3 === 0 ? "\n  led" + i / 3 + ": " : "") + v).join(",");
}

// ---- minimal RFC6455 server ----
const server = net.createServer(sock => {
  let opened = false, buf = Buffer.alloc(0);
  const send = obj => {
    const payload = Buffer.from(JSON.stringify(obj));
    const head = Buffer.from([0x81, payload.length]);
    sock.write(Buffer.concat([head, payload]));
  };
  sock.on("data", d => {
    buf = Buffer.concat([buf, d]);
    if (!opened) {
      const s = buf.toString("latin1");
      const idx = s.indexOf("\r\n\r\n");
      if (idx < 0) return;
      const req = s.slice(0, idx);
      const keyMatch = /sec-websocket-key: (.+)/i.exec(req);
      // not a websocket handshake (health probe etc.) - drop the connection
      if (!keyMatch) { sock.destroy(); return; }
      const key = keyMatch[1].trim();
      const accept = crypto.createHash("sha1").update(key + GUID).digest("base64");
      sock.write("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: " + accept + "\r\n\r\n");
      buf = buf.subarray(idx + 4);
      opened = true;
      send({ ev: "hello", fw: 7, leds: LEDS, pin: 4, wantBrightness: 60, brightness: 60, gspd: 10, fps: 25, on: 1, ip: "127.0.0.1", heap: 20000, steps: 0, layers: layers.length });
      console.log("[dev] hello sent");
      return;
    }
    // parse frames
    while (true) {
      if (buf.length < 2) break;
      const b0 = buf[0], b1 = buf[1];
      const opcode = b0 & 0x0f;
      let len = b1 & 0x7f, off = 2;
      if (len === 126) { if (buf.length < 4) break; len = buf.readUInt16BE(2); off = 4; }
      else if (len === 127) { if (buf.length < 10) break; len = Number(buf.readBigUInt64BE(2)); off = 10; }
      const masked = (b1 & 0x80) !== 0;
      const maskKey = masked ? buf.subarray(off, off + 4) : null;
      if (masked) off += 4;
      if (buf.length < off + len) break;
      let payload = buf.subarray(off, off + len);
      if (masked) { const out = Buffer.alloc(len); for (let i = 0; i < len; i++) out[i] = payload[i] ^ maskKey[i & 3]; payload = out; }
      buf = buf.subarray(off + len);
      if (opcode !== 1) continue;
      const msg = JSON.parse(payload.toString("utf8"));
      if (msg.ev) continue;
      const key = msg.k, id = msg.i;
      if (key === "h") { send({ ev: "hello", fw: 7, leds: LEDS, pin: 4, wantBrightness: 60, brightness: 60, gspd: 10, fps: 25, on: 1, ip: "127.0.0.1", heap: 20000, steps: 0, layers: layers.length }); send({ ev: "ack", i: id, k: key }); continue; }
      if (key === "l") {
        const blob = b64dec(msg.v || "");
        console.log("[dev] 'l' blob", blob.length + "B:", [...blob].map(x => x.toString(16).padStart(2, "0")).join(" "));
        applyLayers(blob);
        console.log("[dev] layers now:", JSON.stringify(layers.map(L => ({ t: L.type, fx: L.effect, s: L.start, e: L.end, hasPx: L.hasPixels, px: L.hasPixels ? [...L.pixels].join(",") : null }))));
        console.log("[dev] rendered static frame:", renderStatic());
        send({ ev: "ack", i: id, k: key });
        send({ ev: "layers", n: layers.length });
        continue;
      }
      console.log("[dev] msg k=" + key, msg.v ? "(" + b64dec(msg.v).length + "B)" : "");
      send({ ev: "ack", i: id, k: key });
    }
  });
  sock.on("error", () => {});
});
server.listen(PORT, "127.0.0.1", () => console.log("[dev] mock listening on ws://127.0.0.1:" + PORT + "/"));
