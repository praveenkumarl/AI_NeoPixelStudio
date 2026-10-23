/*
 * ============================================================================
 *  NeoPixel Studio - ESP8266 (NodeMCU ESP-12E) firmware
 * ============================================================================
 *  Hardware
 *    - NodeMCU ESP-12E (ESP8266)
 *    - WS2812B / NeoPixel strip or ring, 16 pixels
 *    - Data  : GPIO4  (D2 on most NodeMCU boards)
 *    - Power : external 5V supply, common ground with the NodeMCU
 *    - Series 330R resistor in the data line, 1000uF cap across 5V/GND
 *
 *  Features
 *    - Hosts the full web UI from LittleFS (gzip aware)
 *    - WebSocket link for live control + frame streaming
 *    - Same compositing engine as the browser:
 *        base layer(s) -> per-pixel colours -> overlay effect layers
 *      with blend modes: over / add / screen / multiply
 *    - 16 effect generators, each in C++ and mirrored in the web UI
 *    - Two playback paths
 *        1. LIVE  : the device renders the effect stack itself, frame by
 *                   frame at 25 fps. Streamed JSON only carries parameters.
 *        2. SHOW  : the browser renders an exact timeline of 16-pixel frames
 *                   and uploads them (base64 + RLE). The device replays them
 *                   with the exact same durations the user previewed.
 *
 *  Protocol (text frames, single-letter keys, value = base64 blob)
 *    {"i":1,"k":"h"}                       -> hello / status
 *    {"i":2,"k":"c"}                       -> set LED count / data pin
 *    {"i":3,"k":"b"}                       -> set global params
 *    {"i":4,"k":"l"}                       -> set effect layer list
 *    {"i":5,"k":"t"}                       -> set show timeline
 *    {"i":6,"k":"p"}                       -> direct pixel write (live edit)
 *    {"i":7,"k":"s"}                       -> play / stop / pause / step
 *    {"i":8,"k":"S"}                       -> stream-mode toggle
 *    {"i":9,"k":"x"}                       -> save config to flash
 *    {"i":10,"k":"F"}                      -> erase flash config
 *    {"i":11,"k":"A"}                      -> fill all pixels
 *    {"i":12,"k":"B"}                      -> set brightness only
 *
 *  Binary layouts (little endian, decoded from base64)
 *    "b" : [0]=brightness [1]=globalSpeedQ (x10) [2]=fps [3]=masterOn
 *    "l" : [0]=layerCount, then per layer 11 bytes:
 *            type, effect, r, g, b, r2, g2, b2, speedQ, bpmQ, flags
 *          flags bits: 0-3 = range start, 4-7 = range end, 8-9 = blend
 *    "t" : [0..1]=stepCount (u16), then per step 4 bytes:
 *            durationLo, durationHi, pixelCount, blend
 *          then for every step, pixelCount*3 bytes of RLE runs:
 *            count, r, g, b   (run length 1..255, = count+1 pixels)
 *    "p" : [0]=startIndex [1]=pixelCount, then 3 bytes per pixel
 *    "s" : [0]=action (0 stop, 1 play, 2 pause, 3 next step, 4 restart)
 *
 *  Events pushed to the browser
 *    {"ev":"hello",...}   {"ev":"ack","i":N}    {"ev":"nack","i":N,"e":"..."}
 *    {"ev":"step","i":n}  {"ev":"done"}         {"ev":"stats",...}
 *
 *  Message size and fragmentation
 *    One logical message is capped at MAX_MSG_BYTES (~1.5 KB) because ESP8266
 *    DRAM is the binding constraint. The browser sends two extra fields to get
 *    larger payloads across:
 *      "f":1 starts a logical message, "m":1 means more fragments follow
 *      "p":1 starts a multi-message payload, "M":1 means more messages follow
 *    "f"/"m" reassemble base64 into g_b64; "p"/"M" append the DECODED bytes
 *    into g_showbuf, which is how a show larger than one message arrives.
 *    The handler only sees a complete blob, so ack/nack semantics are unchanged.
 * ============================================================================
 */

// ===========================================================================
// Arduino.h FIRST: the constants and data model below use uint8_t/uint16_t,
// so a type header must be included before them.
// ===========================================================================
#include <Arduino.h>

// ===========================================================================
// Sizing constants and data model - deliberately ABOVE the remaining includes.
//
// The Arduino preprocessor hoists every function prototype to just after the
// include block. Any type such a prototype mentions must already be declared
// by then, otherwise the build fails with a misleading error pointing at a
// line where the type is plainly defined (observed as "RGB does not name a
// type"). One include for the base types is the price of that ordering.
// ===========================================================================

// ---------------------------------------------------------------------------
// Sizing / limits the data model depends on
// ---------------------------------------------------------------------------
#define MAX_LEDS          64         // hard ceiling for the pixel buffer
#define MAX_LAYERS        12
#define MAX_STEPS         64

// ---------------------------------------------------------------------------
// Data model
// ---------------------------------------------------------------------------
struct RGB { uint8_t r, g, b; };

struct Layer {
  uint8_t type;      // LayerType
  uint8_t effect;    // EffectId
  RGB     c1;
  RGB     c2;
  uint8_t speedQ;    // speed  x10
  uint8_t bpmQ;      // bpm/10
  uint8_t start;     // range start index
  uint8_t end;       // range end index (inclusive)
  uint8_t blend;     // BlendMode
  uint8_t pixels[MAX_LEDS];   // only used by LT_PIXELS (3 bytes per pixel)
  bool    hasPixels;
};

struct Step {
  uint16_t duration;   // ms
  uint8_t  pixelCount;
  uint8_t  blend;
  uint8_t *runs;       // RLE: count, r, g, b
  uint16_t runsLen;
};

struct Show {
  uint16_t stepCount;
  uint16_t usedBytes;
  Step     steps[MAX_STEPS];
};

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <ctype.h>
#include "ui_embedded.h"

// ---------------------------------------------------------------------------
// User configuration
// ---------------------------------------------------------------------------
#define PIN_NEOPIXEL      4          // GPIO4 = D2
#define DEFAULT_LED_COUNT 16

// WebSocket port.
//
// Deliberately NOT 81: the ESP8266 core can start its own OTA server, and that
// binds port 81 by default. A second listener on the same port fails, which the
// browser reports as a permanently "offline" UI while HTTP on port 80 works
// fine. If you have OTA disabled, 81 is free - but change WS_PORT in web/app.js
// to match whatever you pick.
#define WS_PORT           82
#define HTTP_PORT         80


// ---------------------------------------------------------------------------
// Strip supply voltage and current limiting
//
// WS2812B wants 5 V. Its logic and LED dies are specified from ~3.5 V up, and
// at 3.3 V the BLUE channel is the first to starve - colours come out
// yellow/green-shifted and whites look warm. It will usually still light up,
// just wrong, and it draws less current than at 5 V.
//
// The bigger risk is current. The NodeMCU's on-board 3V3 regulator is typically
// rated ~500 mA and has to power the ESP8266 as well. One WS2812B pixel at full
// white is ~60 mA, so 16 pixels is ~960 mA - far beyond it, which browns out the
// board and reboots it. Hence a hard brightness ceiling in 3.3 V mode.
//
//   LED_SUPPLY_MV  3300 = fed from the board's 3V3 pin (current-capped)
//                  5000 = fed from a separate 5 V supply (full range)
//
// Change this one line to match your wiring.
#define LED_SUPPLY_MV       3300

// Scale a 0..255 brightness by the supply headroom. At 5 V this is a no-op; at
// 3.3 V it caps the global brightness so the regulator is not asked for more
// than it can give. 40% of 255 is ~102, i.e. roughly 16 x 24 mA ~= 380 mA worst
// case at hard white - still generous, and the UI shows the real value.
static inline uint8_t supplyCap(uint8_t v) {
  if (LED_SUPPLY_MV >= 4500) return v;
  return (uint8_t)(((uint16_t)v * 40) / 100);
}
#define DEFAULT_BRIGHTNESS 60        // also stores the user's 0..255 intent


const char *WIFI_SSID = "Praveenvid_EXT";
const char *WIFI_PASS = "lrpvaa8888";
const char *OTA_HOSTNAME = "neopixel-studio";

#define MAX_MSG_BYTES     1536       // ceiling for ONE logical incoming message
#define MAX_SHOW_BYTES    2560       // decoded show payload, spans several messages

// The ESP8266 has ~80 KB of DRAM and the SDK/lwIP/WiFi stacks take most of it,
// so `.bss` has to stay small - an earlier revision overflowed dram0_0_seg with
// ~23 KB of globals. The static_asserts below pin the total.
//
// Every buffer derives from MAX_MSG_BYTES so the set scales together:
//   MAX_MSG_BYTES 1536 -> base64 2056 -> +JSON 2312 -> wire frame 2344
// The browser sends at most FRAG_CHARS (384) base64 chars per fragment and at
// most BLOB_CHUNK (512) decoded bytes per logical message, so:
//   384 base64 chars -> 288 decoded, inside the 1536 message ceiling
//   512 decoded bytes -> 683 base64 chars -> 2 fragments, 433 B wire frames
// Payloads bigger than one message (a show) arrive as several messages that the
// device reassembles into g_showbuf. Keep BLOB_CHUNK/FRAG_CHARS in step with
// MAX_MSG_BYTES; tests/budget_test.js checks the relationship.
#define B64_CHARS         (((MAX_MSG_BYTES * 4) / 3) + 8)   // base64 of the max payload
#define RX_JSON_BYTES     (B64_CHARS + 256)                 // + JSON envelope
#define WS_RX_MAX         (RX_JSON_BYTES + 32)              // single wire-level frame
#define FRAME_MS_DEFAULT  40         // 25 fps
#define MAX_FRAME_MS      250
#define MIN_FRAME_MS      10

// ---------------------------------------------------------------------------
// Effect ids - MUST stay in sync with EFFECTS in app.js
// ---------------------------------------------------------------------------
enum EffectId : uint8_t {
  FX_SOLID = 0, FX_BLINK, FX_BREATHE, FX_RAINBOW, FX_RAINBOW_CYCLE,
  FX_CHASE, FX_COMET, FX_THEATER, FX_WIPE, FX_SPARKLE, FX_FIRE,
  FX_GRADIENT, FX_DUAL_SCAN, FX_STROBE, FX_METEOR_SHOWER, FX_COLOR_FADE,
  FX_COUNT
};

enum LayerType : uint8_t { LT_SOLID = 0, LT_EFFECT = 1, LT_PIXELS = 2 };
enum BlendMode : uint8_t { BM_OVER = 0, BM_ADD = 1, BM_SCREEN = 2, BM_MUL = 3 };

// ---------------------------------------------------------------------------
// Data model
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// DRAM budget guards
//
// The ESP8266 has ~80 KB of DRAM and the SDK/lwIP/WiFi stacks claim most of it,
// leaving only a few KB of .bss for the sketch. Exceeding it produces the
// cryptic linker error "section .bss is not within region dram0_0_seg", so the
// limits are asserted here instead. (These live after the struct definitions
// because sizeof() needs the complete types.)
//
// Measured budget for the shipping configuration:
//   pixel buffers        576 B   (g_base + g_scratch + g_stream)
//   B64_CHARS           2056 B
//   RX_JSON_BYTES       2312 B
//   MAX_MSG_BYTES       1536 B
//   g_showbuf           2560 B   (cross-message reassembly for shows)
//   g_stepOff/Len        256 B
//   Layer[12]           1008 B   (12 x 84)
//   Step[64]             768 B   (64 x 12)
//   NeoWebSocketServer  ~2344 B frame + client descriptors
//   ------------------------------------------
//   total              ~13.4 KB of .bss
//
// Step is 12 bytes, not 8: uint16 + uint8 + uint8 + pointer(4) + uint16 pads to
// three 4-byte words. An earlier assert assumed 8 and would have failed falsely.
// ---------------------------------------------------------------------------
static_assert(sizeof(Layer) * MAX_LAYERS <= 1400,
              "layer array exceeds the DRAM budget - lower MAX_LAYERS");
static_assert(sizeof(Step) * MAX_STEPS <= 900,
              "show steps exceed the DRAM budget - lower MAX_STEPS");
static_assert(MAX_MSG_BYTES <= 1700,
              "message buffer exceeds the DRAM budget - lower MAX_MSG_BYTES");
static_assert(WS_RX_MAX <= 2500,
              "websocket frame buffer exceeds the DRAM budget - lower MAX_MSG_BYTES");
static_assert(MAX_SHOW_BYTES <= 2600,
              "show reassembly buffer exceeds the DRAM budget - lower MAX_SHOW_BYTES");

// One expression for the whole static footprint, so any future size change has
// to keep the total honest rather than silently growing .bss again.
//
// NOT included: UI_EMBEDDED. That lives in .irom0.text (flash) because of
// PROGMEM. Storing it as a plain const char[] put 74 KB into DRAM-backed
// .rodata and pushed .bss 21 KB past dram0_0_seg - see build_embedded_ui.py.
#define STATIC_BSS_BUDGET_BYTES (MAX_LEDS * 3 * 3 + B64_CHARS + RX_JSON_BYTES +  \
                                 MAX_MSG_BYTES + MAX_SHOW_BYTES + 512 +          \
                                 sizeof(Layer) * MAX_LAYERS + sizeof(Step) * MAX_STEPS + \
                                 2400)
static_assert(STATIC_BSS_BUDGET_BYTES <= 18000,
              "total static buffers exceed the DRAM budget for this sketch");

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
ESP8266WebServer   server(HTTP_PORT);
Adafruit_NeoPixel  strip(DEFAULT_LED_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

uint8_t  g_ledCount   = DEFAULT_LED_COUNT;
uint8_t  g_dataPin    = PIN_NEOPIXEL;
uint8_t  g_userBrightness = DEFAULT_BRIGHTNESS;   // what the UI asked for
uint8_t  g_brightness = DEFAULT_BRIGHTNESS;       // what the strip actually gets
uint8_t  g_globalSpdQ = 10;          // 1.0x
uint16_t g_frameMs    = FRAME_MS_DEFAULT;
bool     g_masterOn   = true;

Layer    g_layers[MAX_LAYERS];
uint8_t  g_layerCount = 0;
Show     g_show;

uint8_t  g_base[MAX_LEDS * 3];       // compositing buffer
uint8_t  g_scratch[MAX_LEDS * 3];
uint8_t  g_stream[MAX_LEDS * 3];     // latest streamed frame

uint8_t  g_mode = 0;                 // 0 idle, 1 live, 2 show, 3 stream
bool     g_playing = false;
uint16_t g_stepIdx = 0;
uint32_t g_stepStart = 0;
uint32_t g_fxClock = 0;      // live animation clock; only advances while playing
uint32_t g_lastFxTick = 0;   // wall time of the last clock advance
uint32_t g_lastFrame = 0;
uint8_t  g_streamBlend = BM_OVER;

uint8_t  g_b64[B64_CHARS];
uint8_t  g_rx[RX_JSON_BYTES];
uint8_t  g_msg[MAX_MSG_BYTES];      // reassembled logical message (fragments)
uint8_t  g_showbuf[MAX_SHOW_BYTES]; // decoded show payload, spans several messages
uint16_t g_showLen = 0;             // bytes accumulated in g_showbuf
bool     g_showActive = false;      // a multi-message show upload is in progress
char     g_showKey = 0;             // key of the upload being reassembled
uint16_t g_stepOff[MAX_STEPS];      // reassembled offsets, applied at the end
uint16_t g_stepLen[MAX_STEPS];

// ---------------------------------------------------------------------------
// Minimal WebSocket server (RFC 6455) - no external library.
//
// Dependency-free on purpose: the ESP8266 core does not always ship a
// WebSocket server, and pinning the sketch to a third-party library adds a
// version-matching failure mode for no benefit. This implements exactly what
// the studio needs: the HTTP upgrade handshake, single-frame text messages,
// client frames (masked, as required of clients), ping/pong and close.
//
// It exposes the same shape the rest of the sketch already uses:
//   ws.begin() / ws.loop() / ws.onEvent(cb)
//   ws.sendTXT(num, str) / ws.broadcastTXT(str) / ws.connectedClients()
// ---------------------------------------------------------------------------
#define WS_MAX_CLIENTS 2
#define WS_MAX_FRAME   WS_RX_MAX        // one wire frame, not the whole message

// --- SHA-1, only needed for the handshake's Sec-WebSocket-Accept ------------
static void sha1(const uint8_t *data, size_t len, uint8_t out[20]) {
  static const uint32_t K[4] = { 0x5A827999, 0x6ED9EBA1, 0x8F1BBCDC, 0xCA62C1D6 };
  uint32_t h0 = 0x67452301, h1 = 0xEFCDAB89, h2 = 0x98BADCFE,
           h3 = 0x10325476, h4 = 0xC3D2E1F0;

  const uint32_t totalLen = (uint32_t)len;
  const uint32_t paddedLen = ((totalLen + 8) / 64 + 1) * 64;

  for (uint32_t off = 0; off < paddedLen; off += 64) {
    uint8_t blk[64];
    for (uint8_t i = 0; i < 64; i++) {
      uint32_t p = off + i;
      if (p < totalLen)          blk[i] = data[p];
      else if (p == totalLen)    blk[i] = 0x80;
      else if (p >= paddedLen - 8) {
        uint8_t shift = (uint8_t)((paddedLen - 1 - p) * 8);
        blk[i] = (uint8_t)(((uint64_t)totalLen * 8) >> shift);
      } else                     blk[i] = 0x00;
    }

    uint32_t w[80];
    for (uint8_t i = 0; i < 16; i++) {
      w[i] = ((uint32_t)blk[i * 4] << 24) | ((uint32_t)blk[i * 4 + 1] << 16) |
             ((uint32_t)blk[i * 4 + 2] << 8) | (uint32_t)blk[i * 4 + 3];
    }
    for (uint8_t i = 16; i < 80; i++) {
      uint32_t v = w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16];
      w[i] = (v << 1) | (v >> 31);
    }

    uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
    for (uint8_t i = 0; i < 80; i++) {
      uint32_t f, k;
      if (i < 20)      { f = (b & c) | ((~b) & d);        k = K[0]; }
      else if (i < 40) { f = b ^ c ^ d;                    k = K[1]; }
      else if (i < 60) { f = (b & c) | (b & d) | (c & d);  k = K[2]; }
      else             { f = b ^ c ^ d;                    k = K[3]; }
      uint32_t tmp = ((a << 5) | (a >> 27)) + f + e + k + w[i];
      e = d; d = c; c = (b << 30) | (b >> 2); b = a; a = tmp;
    }
    h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
  }

  uint32_t h[5] = { h0, h1, h2, h3, h4 };
  for (uint8_t i = 0; i < 5; i++) {
    out[i * 4]     = (uint8_t)(h[i] >> 24);
    out[i * 4 + 1] = (uint8_t)(h[i] >> 16);
    out[i * 4 + 2] = (uint8_t)(h[i] >> 8);
    out[i * 4 + 3] = (uint8_t)(h[i]);
  }
}

static const char B64_TAB[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static void base64Encode(const uint8_t *in, size_t len, char *out) {
  size_t o = 0;
  for (size_t i = 0; i < len; i += 3) {
    uint32_t v = (uint32_t)in[i] << 16;
    if (i + 1 < len) v |= (uint32_t)in[i + 1] << 8;
    if (i + 2 < len) v |= (uint32_t)in[i + 2];
    out[o++] = B64_TAB[(v >> 18) & 0x3F];
    out[o++] = B64_TAB[(v >> 12) & 0x3F];
    out[o++] = (i + 1 < len) ? B64_TAB[(v >> 6) & 0x3F] : '=';
    out[o++] = (i + 2 < len) ? B64_TAB[v & 0x3F]        : '=';
  }
  out[o] = 0;
}

// case-insensitive substring search for HTTP header lookup
static const char *stristr_(const char *hay, const char *needle) {
  if (!*needle) return hay;
  for (; *hay; hay++) {
    const char *h = hay, *n = needle;
    while (*h && *n && tolower((unsigned char)*h) == tolower((unsigned char)*n)) { h++; n++; }
    if (!*n) return hay;
  }
  return NULL;
}

// Event kinds. Deliberately plain #defines over uint8_t rather than an
// `enum ... : uint8_t`: the Arduino builder auto-inserts function prototypes
// near the top of the file, ahead of any type defined down here, so a custom
// enum name used in a function parameter reads as "not declared" at that
// point. A macro has no scope, so the injected prototype is always valid.
#define WS_EV_CONNECTED     0
#define WS_EV_DISCONNECTED  1
#define WS_EV_TEXT          2

class NeoWebSocketServer {
  public:
    explicit NeoWebSocketServer(uint16_t port)
      : m_server(port), m_clientCount(0), m_port(port), m_running(false) {
      // WiFiServer has no default constructor - it must be built with the port.
      //
      // DO NOT memset(m_c, ...) here. Client holds a WiFiClient, which is NOT a
      // trivially-copyable type: on ESP8266 it wraps a
      // std::shared_ptr<ClientContext>. Zeroing it with memset leaves a
      // shared_ptr whose control block looks null-but-nonempty, and the next
      // assignment or stop() walks that bogus pointer. That is memory
      // corruption with no exception dump - exactly the "reboots when I click
      // connect" symptom. Set only the plain flags instead; the WiFiClients are
      // already properly default-constructed by their own constructors.
      for (uint8_t i = 0; i < WS_MAX_CLIENTS; i++) {
        m_c[i].active = false;
        m_c[i].handshaken = false;
        m_c[i].fragLen = 0;
        m_c[i].openedAt = 0;
        m_c[i].hdrGot = 0;
        m_c[i].hdrNeed = 2;
        m_c[i].partSince = 0;
        m_c[i].lastRx = 0;
      }
    }

    void begin() {
      m_server.begin();
      m_server.setNoDelay(true);
      m_running = true;
      Serial.printf("[ws] listening on port %u\n", m_port);
    }

    void onEvent(void (*cb)(uint8_t, uint8_t, uint8_t *, size_t)) { m_cb = cb; }

    uint8_t connectedClients() const { return m_clientCount; }

    // remoteIP() is not const on WiFiClient, so this accessor is not either
    IPAddress remoteIP(uint8_t num) {
      if (num >= WS_MAX_CLIENTS || !m_c[num].active) return IPAddress();
      return m_c[num].tcp.remoteIP();
    }

    bool sendTXT(uint8_t num, const char *text) {
      if (num >= WS_MAX_CLIENTS || !m_c[num].active) return false;
      return sendFrame(num, (const uint8_t *)text, strlen(text), 0x1);
    }
    bool sendTXT(uint8_t num, const String &s) { return sendTXT(num, s.c_str()); }

    bool broadcastTXT(const char *text) {
      bool any = false;
      const uint8_t *p = (const uint8_t *)text;
      size_t len = strlen(text);
      for (uint8_t i = 0; i < WS_MAX_CLIENTS; i++)
        if (m_c[i].active) any |= sendFrame(i, p, len, 0x1);
      return any;
    }
    bool broadcastTXT(const String &s) { return broadcastTXT(s.c_str()); }

    void loop() {
      if (!m_running) return;

      // accept new connections
      while (m_clientCount < WS_MAX_CLIENTS) {
        WiFiClient nc = m_server.accept();
        if (!nc) break;
        Serial.printf("[ws] tcp connect, heap %u\n", ESP.getFreeHeap()); Serial.flush();
        int8_t slot = -1;
        for (uint8_t i = 0; i < WS_MAX_CLIENTS; i++) {
          if (!m_c[i].active) { slot = (int8_t)i; break; }
        }
        if (slot < 0) {
          // Every slot is taken. Do not just refuse: check whether a slot holds a
          // dead/half-handshaken socket and reclaim it, otherwise a connection
          // that died mid-handshake eats a slot forever and the device stops
          // accepting anything (the browser then loops on "closed before the
          // connection is established" because nothing can ever connect).
          bool reclaimed = false;
          for (uint8_t i = 0; i < WS_MAX_CLIENTS; i++) {
            if (!isConn(i) || !m_c[i].tcp.connected() ||
                (millis() - m_c[i].openedAt > 20000 && !m_c[i].handshaken)) {
              Serial.printf("[ws] reclaiming stale slot %u (handshaken=%u)\n",
                            i, m_c[i].handshaken ? 1 : 0);
              dropClient(i, true);
              reclaimed = true;
              break;
            }
          }
          if (!reclaimed) { nc.stop(); break; }     // genuine capacity, refuse
          continue;
        }
        nc.setNoDelay(true);
        m_c[slot].tcp = nc;
        m_c[slot].active = true;
        m_c[slot].handshaken = false;
        m_c[slot].fragLen = 0;
        m_c[slot].hdrGot = 0;
        m_c[slot].hdrNeed = 2;
        m_c[slot].partSince = 0;
        m_c[slot].lastRx = millis();
        m_c[slot].openedAt = millis();
        m_clientCount++;
        // the handshake is finished in the same loop so the browser sees the
        // socket open immediately
        if (!doHandshake(slot)) dropClient(slot, false);
      }

      // a slot that was accepted but never handshook must not linger: if the
      // peer vanished, free it so capacity is not silently consumed
      for (uint8_t i = 0; i < WS_MAX_CLIENTS; i++) {
        if (m_c[i].active && !m_c[i].handshaken &&
            (millis() - m_c[i].openedAt > 8000)) {
          Serial.printf("[ws] dropping slot %u: no handshake after 8s\n", i);
          dropClient(i, true);
        }
      }

      for (uint8_t i = 0; i < WS_MAX_CLIENTS; i++) {
        if (!m_c[i].active) continue;
        if (!m_c[i].handshaken) continue;
        // isConn() is the null-safe test (see below), and connected() then
        // polls the real TCP state: a peer that sent FIN is noticed here and
        // the slot freed instead of leaking until a reboot.
        if (!isConn(i) || !m_c[i].tcp.connected()) { dropClient(i, true); continue; }
        // a frame that has sat half-arrived for 5 s means the peer died
        // mid-message; drop rather than wait forever
        if (m_c[i].partSince && millis() - m_c[i].partSince > 5000) {
          Serial.printf("[ws] dropping slot %u: frame stalled mid-message\n", i);
          dropClient(i, true); continue;
        }
        // the UI heartbeats every 5 s; 30 s of total silence on a handshaken
        // client means the peer vanished without a FIN (sleeping laptop, wifi
        // drop). Reclaim the slot - leaking both slots is what made the device
        // refuse every new connection until a power cycle.
        if (m_c[i].lastRx && millis() - m_c[i].lastRx > 30000) {
          Serial.printf("[ws] dropping slot %u: silent for 30 s\n", i);
          dropClient(i, true); continue;
        }
        readFrames(i);
      }
    }

  private:
    struct Client {
      WiFiClient tcp;
      bool active;
      bool handshaken;
      uint16_t fragLen;      // bytes assembled so far
      uint8_t  fragOpcode;   // opcode of the message being assembled
      uint32_t openedAt;     // millis() when this slot was accepted
      uint8_t  hdr[14];      // staged frame header (2 + 8 ext-len + 4 mask)
      uint8_t  hdrGot;       // header bytes consumed so far
      uint8_t  hdrNeed;      // header bytes required before it can be decoded
      uint32_t lastRx;       // millis() of the last received byte
      uint32_t partSince;    // a frame has been half-arrived since this time
    };

    WiFiServer m_server;
    Client     m_c[WS_MAX_CLIENTS];
    uint8_t    m_clientCount;
    uint16_t   m_port;
    bool       m_running;
    void (*m_cb)(uint8_t, uint8_t, uint8_t *, size_t) = nullptr;
    uint8_t    m_frame[WS_MAX_FRAME];
    char       m_req[1024];        // handshake request, kept off the stack

    // Null-safe liveness test for a client slot. NEVER call connected() or any
    // other WiFiClient member directly: a default-constructed WiFiClient has a
    // null ClientContext and every such call faults.
    //
    // NOT const: WiFiClient::operator bool() is a non-const virtual in the
    // ESP8266 core, so a const method here cannot call it.
    bool isConn(uint8_t slot) {
      if (slot >= WS_MAX_CLIENTS) return false;
      return m_c[slot].tcp.operator bool();    // operator bool() is null-safe
    }

    void dropClient(uint8_t slot, bool notify) {
      if (!m_c[slot].active) return;
      if (isConn(slot)) m_c[slot].tcp.stop();  // only touch a live socket
      m_c[slot].active = false;
      m_c[slot].handshaken = false;
      m_c[slot].fragLen = 0;
      m_c[slot].hdrGot = 0;
      m_c[slot].hdrNeed = 2;
      m_c[slot].partSince = 0;
      if (m_clientCount) m_clientCount--;
      if (notify && m_cb) m_cb(slot, WS_EV_DISCONNECTED, nullptr, 0);
    }

    bool doHandshake(uint8_t slot) {
      WiFiClient &c = m_c[slot].tcp;
      // m_req is a member, not a local: a 1 KB stack buffer is a real risk on the
      // ESP8266 (~4 KB stack), and this runs inside the network callback path.
      char *req = m_req;
      const size_t reqCap = sizeof(m_req);
      size_t n = 0;
      const uint32_t deadline = millis() + 3000;
      while (millis() < deadline) {
        while (c.available() && n < reqCap - 1) {
          req[n++] = (char)c.read();
        }
        req[n] = 0;
        if (strstr(req, "\r\n\r\n")) break;
        if (n >= reqCap - 1) return false;
        delay(1);
      }
      if (n == 0) { Serial.println(F("[ws] handshake: no request bytes")); return false; }
      Serial.printf("[ws] handshake: %u bytes\n", (unsigned)n); Serial.flush();

      const char *keyHdr = stristr_(req, "Sec-WebSocket-Key:");
      if (!keyHdr) { Serial.println(F("[ws] handshake: no Sec-WebSocket-Key")); return false; }
      keyHdr += strlen("Sec-WebSocket-Key:");
      while (*keyHdr == ' ') keyHdr++;
      char key[64];
      size_t k = 0;
      while (*keyHdr && *keyHdr != '\r' && *keyHdr != '\n' && k < sizeof(key) - 1)
        key[k++] = *keyHdr++;
      key[k] = 0;
      if (k == 0) return false;

      // accept = base64(sha1(key + GUID))
      static const char *GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
      uint8_t concat[128];
      size_t cl = 0;
      for (size_t i = 0; i < k && cl < sizeof(concat); i++) concat[cl++] = (uint8_t)key[i];
      for (size_t i = 0; GUID[i] && cl < sizeof(concat); i++) concat[cl++] = (uint8_t)GUID[i];

      uint8_t digest[20];
      sha1(concat, cl, digest);
      char accept[40];
      base64Encode(digest, 20, accept);

      // Print the received key and the accept we computed. The browser validates
      // accept == base64(sha1(key + GUID)); if these disagree, the browser
      // abandons the socket right after the handshake, sending an EMPTY close
      // frame (code 0, payload 0) and never sending a single message.
      Serial.printf("[ws] key=\"%s\"\n", key);
      Serial.printf("[ws] accept=\"%s\" (expect s3pPLMBiTxaQ9kYGzzhZRbK+xOo= for the RFC key)\n", accept);
      Serial.flush();

      char resp[256];
      int rl = snprintf(resp, sizeof(resp),
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: %s\r\n\r\n", accept);
      if (rl <= 0) return false;
      c.write((const uint8_t *)resp, (size_t)rl);
      c.flush();

      m_c[slot].handshaken = true;
      Serial.printf("[ws] handshake complete for client %u\n", slot);
      if (m_cb) m_cb(slot, WS_EV_CONNECTED, nullptr, 0);
      return true;
    }

    bool sendFrame(uint8_t slot, const uint8_t *payload, size_t len, uint8_t opcode) {
      if (slot >= WS_MAX_CLIENTS || !m_c[slot].active || !m_c[slot].handshaken) return false;
      WiFiClient &c = m_c[slot].tcp;

      uint8_t hdr[10];
      size_t h = 0;
      hdr[h++] = 0x80 | (opcode & 0x0F);          // FIN + opcode
      if (len < 126) {
        hdr[h++] = (uint8_t)len;
      } else if (len <= 0xFFFF) {
        hdr[h++] = 126;
        hdr[h++] = (uint8_t)(len >> 8);
        hdr[h++] = (uint8_t)(len & 0xFF);
      } else {
        hdr[h++] = 127;
        for (int i = 7; i >= 0; i--) hdr[h++] = (uint8_t)(((uint64_t)len >> (i * 8)) & 0xFF);
      }
      // server frames are never masked

      // one write for small frames keeps the TLS-less TCP stream tight
      if (h + len <= sizeof(m_frame)) {
        memcpy(m_frame, hdr, h);
        memcpy(m_frame + h, payload, len);
        size_t sent = c.write((const uint8_t *)m_frame, h + len);
        return sent == h + len;
      }
      if (c.write(hdr, h) != h) return false;
      return c.write(payload, len) == len;
    }

    void noteStall(uint8_t slot) {
      if (!m_c[slot].partSince) m_c[slot].partSince = millis();
    }

    // Staged, NON-DESTRUCTIVE RFC 6455 reader. Header bytes are consumed only
    // as the header decodes, and the payload only once ALL of it is buffered,
    // so a frame that straddles TCP segments can never leave the parser
    // mid-frame. The previous version consumed the 2 header bytes immediately
    // and then blocked waiting for the rest (up to 3 s); on timeout the tail
    // of that frame was later parsed as a fresh header and every subsequent
    // message on the socket turned to garbage - the "works until I touch X,
    // then even the buttons die, reload fixes it" failure.
    //
    // A logical message may still arrive as a first frame (opcode 1, FIN
    // clear) plus continuation frames (opcode 0), with FIN set on the last;
    // each individual frame stays small, so the per-frame buffer can be ~2 KB
    // while a message can be up to MAX_MSG_BYTES.
    void readFrames(uint8_t slot) {
      WiFiClient &c = m_c[slot].tcp;
      for (;;) {
        Client &cl = m_c[slot];

        // ---- assemble the 2..14 byte header, never touching payload bytes
        while (cl.hdrGot < cl.hdrNeed) {
          if (c.available() == 0) {
            if (cl.hdrGot) noteStall(slot);
            return;
          }
          cl.hdr[cl.hdrGot++] = (uint8_t)c.read();
          cl.lastRx = millis();
          cl.partSince = 0;
        }

        const uint8_t h0 = cl.hdr[0], h1 = cl.hdr[1];
        const uint8_t opcode = h0 & 0x0F;
        const bool fin = (h0 & 0x80) != 0;
        const bool masked = (h1 & 0x80) != 0;
        const uint8_t len7 = h1 & 0x7F;

        uint8_t need = 2;
        if (len7 == 126)      need = 4;           // 16-bit extended length
        else if (len7 == 127) need = 10;          // 64-bit extended length
        if (masked) need += 4;
        if (cl.hdrGot < need) { cl.hdrNeed = need; continue; }

        uint64_t len = len7;
        if (len7 == 126) {
          len = ((uint64_t)cl.hdr[2] << 8) | cl.hdr[3];
        } else if (len7 == 127) {
          len = 0;
          for (uint8_t i = 0; i < 8; i++) len = (len << 8) | cl.hdr[2 + i];
        }
        const uint8_t extBytes = (len7 == 126) ? 2 : (len7 == 127 ? 8 : 0);
        const uint8_t *mask = cl.hdr + 2 + extBytes;

        // control frames may never be fragmented or exceed 125 bytes
        const bool isControl = (opcode & 0x08) != 0;
        if (isControl) {
          if (!fin || len > 125) { dropClient(slot, true); return; }
        } else if (len > WS_MAX_FRAME) {
          dropClient(slot, true);
          return;
        }

        // ---- only consume the payload once every byte of it is buffered
        if ((uint64_t)c.available() < len) { noteStall(slot); return; }
        if (len > 0) {
          size_t got = 0;
          while (got < (size_t)len) {
            int r = c.read(m_frame + got, (size_t)len - got);
            if (r <= 0) break;
            got += (size_t)r;
          }
          if (got < (size_t)len) { dropClient(slot, true); return; }
          cl.lastRx = millis();
          cl.partSince = 0;
          if (masked)
            for (size_t i = 0; i < (size_t)len; i++) m_frame[i] ^= mask[i & 3];
        }
        cl.hdrGot = 0;
        cl.hdrNeed = 2;

        switch (opcode) {
          case 0x8: {                             // close
            // RFC 6455: the first two payload bytes are the status code.
            // 1000 = normal closure; an EMPTY close right after the handshake
            // is the browser rejecting the 101 response.
            unsigned code = 0;
            if (len >= 2) code = ((unsigned)m_frame[0] << 8) | m_frame[1];
            Serial.printf("[ws] close frame from client %u, code %u, payload %u\n",
                          slot, code, (unsigned)len);
            sendFrame(slot, m_frame, 0, 0x8);     // echo the close back, per spec
            dropClient(slot, true);
            return;
          }

          case 0x9: {                             // ping -> pong
            // copy out first: sendFrame() reuses m_frame as its write buffer,
            // so echoing straight from it would overlap src and dst
            uint8_t pong[125];
            size_t pl = (size_t)len;
            if (pl > sizeof(pong)) pl = sizeof(pong);
            memcpy(pong, m_frame, pl);
            sendFrame(slot, pong, pl, 0xA);
            break;
          }

          case 0xA:                               // pong: nothing to do
            break;

          case 0x1:                               // start of a text message
          case 0x0: {                             // continuation
            // a new text message mid-assembly means the previous one was
            // abandoned; start over rather than mixing the two
            if (opcode == 0x1 && cl.fragLen != 0) cl.fragLen = 0;

            if ((uint32_t)cl.fragLen + (uint32_t)len > MAX_MSG_BYTES) {
              cl.fragLen = 0;
              broadcastTXT("{\"ev\":\"nack\",\"e\":\"message too large\"}");
              dropClient(slot, true);
              return;
            }
            memcpy(g_msg + cl.fragLen, m_frame, (size_t)len);
            cl.fragLen += (uint16_t)len;

            if (fin) {
              size_t total = cl.fragLen;
              cl.fragLen = 0;
              if (m_cb && total > 0) {
                g_msg[total < MAX_MSG_BYTES ? total : MAX_MSG_BYTES - 1] = 0;
                m_cb(slot, WS_EV_TEXT, g_msg, total);
              }
            }
            break;
          }

          default:                                // binary: unused
            break;
        }
        // a callback may have dropped this client
        if (!m_c[slot].active) return;
      }
    }
};

NeoWebSocketServer ws(WS_PORT);

// forward declarations - must match the definitions exactly (including the
// static storage class and the parameter list) or the compiler sees them as
// distinct overloads.
static void renderFrame(uint8_t *dst);
static void showPixels(const uint8_t *src);
static void expandStep(const Step &S, uint8_t *dst);
static void sendHello(uint8_t num);
static void sendStats();
static void pushLayers();
static void pushShow();
static void applyStreamFrame(uint8_t *b, size_t len);
static void setPixelDirect(uint8_t *b, size_t len);
static void fillAll(uint8_t *b, size_t len);
static void printDiag();          // called by onWsEvent, defined further down

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
static inline uint8_t clamp8(int v) { return v < 0 ? 0 : (v > 255 ? 255 : (uint8_t)v); }

// HSV -> RGB, h in 0..360, s/v in 0..255
static RGB hsv2rgb(uint16_t h, uint8_t s, uint8_t v) {
  h = h % 360;
  uint8_t region = h / 60;
  uint16_t rem = (h - region * 60) * 255 / 60;
  uint8_t p = (uint16_t)v * (255 - s) / 255;
  uint8_t q = (uint16_t)v * (255 - ((uint16_t)s * rem) / 255) / 255;
  uint8_t t = (uint16_t)v * (255 - ((uint16_t)s * (255 - rem)) / 255) / 255;
  switch (region) {
    case 0:  return { (uint8_t)v, t, p };
    case 1:  return { q, (uint8_t)v, p };
    case 2:  return { p, (uint8_t)v, t };
    case 3:  return { p, q, (uint8_t)v };
    case 4:  return { t, p, (uint8_t)v };
    default: return { (uint8_t)v, p, q };
  }
}

// brightness scaler 0..255
static inline uint8_t scale8(uint8_t v, uint8_t s) {
  return (uint8_t)(((uint16_t)v * (uint16_t)s) >> 8);
}

static void blendPixel(uint8_t *dst, uint8_t r, uint8_t g, uint8_t b, uint8_t mode) {
  uint16_t rr, gg, bb;
  switch (mode) {
    case BM_ADD:
      dst[0] = clamp8(dst[0] + r);
      dst[1] = clamp8(dst[1] + g);
      dst[2] = clamp8(dst[2] + b);
      break;
    case BM_SCREEN:
      rr = 255 - ((255 - dst[0]) * (255 - r)) / 255;
      gg = 255 - ((255 - dst[1]) * (255 - g)) / 255;
      bb = 255 - ((255 - dst[2]) * (255 - b)) / 255;
      dst[0] = (uint8_t)rr; dst[1] = (uint8_t)gg; dst[2] = (uint8_t)bb;
      break;
    case BM_MUL:
      dst[0] = (uint8_t)(((uint16_t)dst[0] * r) / 255);
      dst[1] = (uint8_t)(((uint16_t)dst[1] * g) / 255);
      dst[2] = (uint8_t)(((uint16_t)dst[2] * b) / 255);
      break;
    default: // over
      dst[0] = r; dst[1] = g; dst[2] = b;
      break;
  }
}

// ---------------------------------------------------------------------------
// Effect engine - mirrors renderEffect() in app.js exactly
// ---------------------------------------------------------------------------
static uint32_t fxRand(uint32_t *seed) {
  uint32_t s = *seed;
  s ^= s << 13; s ^= s >> 17; s ^= s << 5;
  *seed = s;
  return s;
}

static void renderEffect(uint8_t effect, RGB c1, RGB c2, float speed, float bpm,
                         int rangeStart, int rangeEnd, uint8_t *out) {
  const int n = g_ledCount;
  // animation time comes from g_fxClock, which only advances while playing,
  // so Pause actually freezes the effects instead of them running on wall time
  const uint32_t t = g_fxClock;
  const float ts = t / 1000.0f;
  const int span = (rangeEnd - rangeStart) + 1;
  uint32_t seed = 0x9E3779B9u + (uint32_t)(t / 7) + (uint32_t)effect * 2654435761u;

  for (int i = 0; i < n; i++) {
    out[i * 3 + 0] = 0; out[i * 3 + 1] = 0; out[i * 3 + 2] = 0;
  }

  for (int i = 0; i < n; i++) {
    if (i < rangeStart || i > rangeEnd) continue;
    const int k = i - rangeStart;          // index inside the range
    RGB px = { 0, 0, 0 };

    switch (effect) {
      case FX_SOLID:
        px = c1;
        break;

      case FX_BLINK: {
        float period = bpm > 0.1f ? (30.0f / bpm) : 0.5f;
        if (period < 0.02f) period = 0.02f;
        bool on = fmodf(ts * speed, period) < (period * 0.5f);
        px = on ? c1 : c2;
        break;
      }

      case FX_BREATHE: {
        float period = bpm > 0.1f ? (60.0f / bpm) : 2.0f;
        if (period < 0.05f) period = 0.05f;
        float ph = fmodf(ts * speed, period) / period;
        float v = 0.5f - 0.5f * cosf(ph * 2.0f * PI);
        px.r = scale8(c1.r, (uint8_t)(v * 255));
        px.g = scale8(c1.g, (uint8_t)(v * 255));
        px.b = scale8(c1.b, (uint8_t)(v * 255));
        break;
      }

      case FX_RAINBOW: {
        uint16_t hue = (uint16_t)fmodf(ts * speed * 60.0f + (float)k * (360.0f / (span > 0 ? span : 1)), 360.0f);
        px = hsv2rgb(hue, 255, 255);
        break;
      }

      case FX_RAINBOW_CYCLE: {
        uint16_t hue = (uint16_t)fmodf(ts * speed * 90.0f + (float)k * 22.5f, 360.0f);
        px = hsv2rgb(hue, 255, 255);
        break;
      }

      case FX_CHASE: {
        int period = span > 0 ? span : 1;
        int head = (int)fmodf(ts * speed * 8.0f, (float)period);
        int d = (k - head + period * 2) % period;
        int fade = period / 3 + 1;
        float v = d < fade ? (1.0f - (float)d / fade) : 0.0f;
        px.r = scale8(c1.r, (uint8_t)(v * 255));
        px.g = scale8(c1.g, (uint8_t)(v * 255));
        px.b = scale8(c1.b, (uint8_t)(v * 255));
        break;
      }

      case FX_COMET: {
        int period = span > 0 ? span : 1;
        int head = (int)fmodf(ts * speed * 6.0f, (float)period);
        int d = (k - head + period * 2) % period;
        int tail = period / 2 + 1;
        float v = d < tail ? powf(0.72f, (float)d) : 0.0f;
        px.r = scale8(c1.r, (uint8_t)(v * 255));
        px.g = scale8(c1.g, (uint8_t)(v * 255));
        px.b = scale8(c1.b, (uint8_t)(v * 255));
        break;
      }

      case FX_THEATER: {
        int step = (int)(ts * speed * 6.0f);
        bool on = (((k + step) % 3) == 0);
        px = on ? c1 : c2;
        break;
      }

      case FX_WIPE: {
        float period = bpm > 0.1f ? (60.0f / bpm) : 2.0f;
        if (period < 0.05f) period = 0.05f;
        float ph = fmodf(ts * speed, period) / period;
        float edge = ph * (float)(span + 1);
        px = ((float)k < edge) ? c1 : c2;
        break;
      }

      case FX_SPARKLE: {
        float rate = speed * (bpm > 0.1f ? bpm / 60.0f : 1.0f);
        float prob = 0.06f * rate;
        if (prob > 0.9f) prob = 0.9f;
        if (prob < 0.0f) prob = 0.0f;
        const uint32_t bucket = t / 60;
        uint32_t s = 0x1234567u + (uint32_t)i * 2654435761u + bucket * 40503u;
        float r = (fxRand(&s) & 0xFFFFFF) / 16777216.0f;
        bool on = r < prob;
        px = on ? c1 : c2;
        break;
      }

      case FX_FIRE: {
        const uint32_t bucket = t / 55;
        uint32_t s = 0xABCDEFu + (uint32_t)i * 2246822519u + bucket * 3266489917u;
        float r = (fxRand(&s) & 0xFFFFFF) / 16777216.0f;
        float wave = 0.5f + 0.5f * sinf(ts * speed * 3.0f + k * 0.9f);
        float v = 0.45f + 0.55f * (0.5f * r + 0.5f * wave);
        if (v > 1.0f) v = 1.0f;
        RGB hot = hsv2rgb((uint16_t)(14.0f * (1.0f - v) + 10.0f), 255, (uint8_t)(v * 255));
        px = hot;
        break;
      }

      case FX_GRADIENT: {
        float u = span > 1 ? (float)k / (float)(span - 1) : 0.0f;
        float w = 0.5f + 0.5f * sinf(ts * speed * 2.0f * PI * 0.5f);
        float mix = u * (1.0f - w) + (1.0f - u) * w;
        px.r = c1.r + (c2.r - c1.r) * mix;
        px.g = c1.g + (c2.g - c1.g) * mix;
        px.b = c1.b + (c2.b - c1.b) * mix;
        break;
      }

      case FX_DUAL_SCAN: {
        int period = span > 0 ? span : 1;
        int head = (int)fmodf(ts * speed * 5.0f, (float)period);
        int a = head, b = period - 1 - head;
        int da = abs(k - a), db = abs(k - b);
        int w = period / 4 + 1;
        int d = da < db ? da : db;
        float v = d < w ? (1.0f - (float)d / w) : 0.0f;
        px.r = scale8(c1.r, (uint8_t)(v * 255));
        px.g = scale8(c1.g, (uint8_t)(v * 255));
        px.b = scale8(c1.b, (uint8_t)(v * 255));
        break;
      }

      case FX_STROBE: {
        float rate = bpm > 0.1f ? bpm / 60.0f : 4.0f;
        float period = 1.0f / (rate * speed);
        if (period < 0.03f) period = 0.03f;
        float duty = 0.12f;
        bool on = fmodf(ts, period) < (period * duty);
        px = on ? c1 : c2;
        break;
      }

      case FX_METEOR_SHOWER: {
        int period = span > 0 ? span : 1;
        const uint32_t bucket = t / 45;
        uint32_t s = 0x5BF03635u + bucket * 2246822519u;
        int m1 = 0, m2 = 0, m3 = 0;
        if (bucket != 0) {
          m1 = (int)(fxRand(&s) % (uint32_t)period);
          m2 = (int)(fxRand(&s) % (uint32_t)period);
          m3 = (int)(fxRand(&s) % (uint32_t)period);
        }
        float v = 0.0f;
        int heads[3] = { m1, m2, m3 };
        for (int h = 0; h < 3; h++) {
          int d = (k - heads[h] + period * 2) % period;
          int tail = period / 3 + 1;
          if (d < tail) {
            float f = powf(0.6f, (float)d);
            if (f > v) v = f;
          }
        }
        v *= speed > 0.35f ? 1.0f : 0.4f;
        RGB col = c2.r + c2.g + c2.b > 30 ? c2 : c1;
        px.r = scale8(col.r, (uint8_t)(v * 255));
        px.g = scale8(col.g, (uint8_t)(v * 255));
        px.b = scale8(col.b, (uint8_t)(v * 255));
        break;
      }

      case FX_COLOR_FADE: {
        float period = bpm > 0.1f ? (120.0f / bpm) : 8.0f;
        if (period < 0.2f) period = 0.2f;
        float ph = fmodf(ts * speed, period) / period;
        float mix = 0.5f - 0.5f * cosf(ph * 2.0f * PI);
        px.r = c1.r + (c2.r - c1.r) * mix;
        px.g = c1.g + (c2.g - c1.g) * mix;
        px.b = c1.b + (c2.b - c1.b) * mix;
        break;
      }

      default:
        px = c1;
        break;
    }

    out[i * 3 + 0] = px.r;
    out[i * 3 + 1] = px.g;
    out[i * 3 + 2] = px.b;
  }
}

// ---------------------------------------------------------------------------
// Compositor - mirrors renderFrame() in app.js
// ---------------------------------------------------------------------------
static void renderFrame(uint8_t *dst) {
  const int n = g_ledCount;
  for (int i = 0; i < n * 3; i++) dst[i] = 0;

  const float gspd = g_globalSpdQ / 10.0f;

  for (uint8_t li = 0; li < g_layerCount; li++) {
    Layer &L = g_layers[li];
    int rs = L.start, re = L.end;
    if (rs >= n) continue;
    if (re >= n) re = n - 1;
    if (re < rs) continue;
    const float spd = (L.speedQ / 10.0f) * gspd;
    const float bpm = L.bpmQ * 10.0f;

    if (L.type == LT_PIXELS) {
      if (!L.hasPixels) continue;
      for (int i = rs; i <= re; i++) {
        int o = i * 3;
        blendPixel(&dst[o], L.pixels[o], L.pixels[o + 1], L.pixels[o + 2], L.blend);
      }
    } else if (L.type == LT_SOLID) {
      for (int i = rs; i <= re; i++) {
        int o = i * 3;
        blendPixel(&dst[o], L.c1.r, L.c1.g, L.c1.b, L.blend);
      }
    } else {
      renderEffect(L.effect, L.c1, L.c2, spd, bpm, rs, re, g_scratch);
      for (int i = rs; i <= re; i++) {
        int o = i * 3;
        blendPixel(&dst[o], g_scratch[o], g_scratch[o + 1], g_scratch[o + 2], L.blend);
      }
    }
  }

  // The browser paints the FIRST pixel layer's colours ON TOP of the stack
  // (renderFrame() in the UI: the S.px overwrite). Mirror that here, or per-LED
  // edits are invisible on the strip whenever an effect layer rides above them.
  for (uint8_t li = 0; li < g_layerCount; li++) {
    const Layer &L = g_layers[li];
    if (L.type != LT_PIXELS || !L.hasPixels) continue;
    for (int i = 0; i < n; i++) {
      const int o = i * 3;
      if (L.pixels[o] | L.pixels[o + 1] | L.pixels[o + 2]) {
        dst[o] = L.pixels[o]; dst[o + 1] = L.pixels[o + 1]; dst[o + 2] = L.pixels[o + 2];
      }
    }
    break;    // only the first pixel layer is the "direct paint" layer
  }
}

static void showPixels(const uint8_t *src) {
  if (!g_masterOn) {
    for (int i = 0; i < g_ledCount; i++) strip.setPixelColor(i, 0, 0, 0);
  } else {
    for (int i = 0; i < g_ledCount; i++) {
      strip.setPixelColor(i, src[i * 3], src[i * 3 + 1], src[i * 3 + 2]);
    }
  }
  strip.show();
}

// ---------------------------------------------------------------------------
// Base64 + JSON scanning (self contained: no ArduinoJson dependency)
// ---------------------------------------------------------------------------
static int b64Val(char c) {
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= 'a' && c <= 'z') return c - 'a' + 26;
  if (c >= '0' && c <= '9') return c - '0' + 52;
  if (c == '+') return 62;
  if (c == '/') return 63;
  return -1;
}

static size_t b64Decode(const char *in, size_t inLen, uint8_t *out, size_t outCap) {
  size_t o = 0;
  int acc = 0, bits = 0;
  for (size_t i = 0; i < inLen; i++) {
    char c = in[i];
    if (c == '=') break;
    int v = b64Val(c);
    if (v < 0) continue;
    acc = (acc << 6) | v;
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      if (o >= outCap) return o;
      out[o++] = (uint8_t)((acc >> bits) & 0xFF);
    }
  }
  return o;
}

// Extract the value of "key" from a JSON document.
// The id is the FIRST field and the key is the SECOND, so searching from the
// start of the document is deterministic and can never latch onto an
// occurrence inside the base64 payload that follows. (The payload alphabet is
// [A-Za-z0-9+/=], so a quoted match inside it is impossible anyway - but we
// do not rely on that.)
static const char *jsonField(const char *doc, const char *key) {
  char pat[8];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  return strstr(doc, pat);
}

static bool jsonString(const char *doc, const char *key, char *out, size_t outCap) {
  const char *p = jsonField(doc, key);
  if (!p) return false;
  p += strlen(key) + 2;
  while (*p && (*p == ' ' || *p == ':')) p++;
  if (*p != '"') return false;
  p++;
  size_t o = 0;
  while (*p && *p != '"' && o + 1 < outCap) out[o++] = *p++;
  out[o] = 0;
  return true;
}

static bool jsonInt(const char *doc, const char *key, long *out) {
  const char *p = jsonField(doc, key);
  if (!p) return false;
  p += strlen(key) + 2;
  while (*p && (*p == ' ' || *p == ':')) p++;
  if (*p != '-' && (*p < '0' || *p > '9')) return false;
  *out = strtol(p, NULL, 10);
  return true;
}

// ---------------------------------------------------------------------------
// Apply incoming configuration blobs
// ---------------------------------------------------------------------------
// Recompute the effective brightness from the user's request and the supply
// headroom, then push it to the strip. Call this after ANY change to
// g_userBrightness or LED_SUPPLY_MV.
static void applyBrightness() {
  g_brightness = supplyCap(g_userBrightness);
  strip.setBrightness(g_brightness);
}

static void applyGlobals(uint8_t *b, size_t len) {
  if (len > 0) g_userBrightness = b[0];
  if (len > 1) g_globalSpdQ = b[1] < 1 ? 1 : (b[1] > 100 ? 100 : b[1]);
  if (len > 2) {
    // b[2] is the frame rate in fps. (An older UI sent frame-ms/10 here, so
    // 25 fps arrived as 0 and clamped to a silent 100 fps.)
    uint16_t fps = b[2];
    if (fps < 4)   fps = 4;               // 4 fps  == MAX_FRAME_MS
    if (fps > 100) fps = 100;             // 100 fps == MIN_FRAME_MS
    g_frameMs = (uint16_t)((1000 + fps / 2) / fps);
  }
  if (len > 3) g_masterOn = b[3] != 0;
  applyBrightness();
  strip.show();
}

static void applyLayers(uint8_t *b, size_t len) {
  if (len < 1) return;
  uint8_t count = b[0];
  if (count > MAX_LAYERS) count = MAX_LAYERS;
  size_t need = 1 + (size_t)count * 11;
  if (len < need) { ws.broadcastTXT("{\"ev\":\"nack\",\"e\":\"layer blob short\"}"); return; }

  g_layerCount = 0;
  for (uint8_t i = 0; i < count; i++) {
    uint8_t *p = b + 1 + i * 11;
    Layer L;
    memset(&L, 0, sizeof(L));
    L.type   = p[0] > LT_PIXELS ? LT_SOLID : p[0];
    L.effect = p[1] < FX_COUNT ? p[1] : FX_SOLID;
    L.c1.r = p[2]; L.c1.g = p[3]; L.c1.b = p[4];
    L.c2.r = p[5]; L.c2.g = p[6]; L.c2.b = p[7];
    L.speedQ = p[8] < 1 ? 1 : p[8];
    L.bpmQ   = p[9];
    L.start  = p[10] & 0x0F;
    L.end    = (p[10] >> 4) & 0x0F;
    if (L.end >= g_ledCount) L.end = g_ledCount - 1;
    if (L.start > L.end) L.start = L.end;
    L.blend  = BM_OVER;             // blend travels in the trailing byte
    L.hasPixels = false;
    g_layers[g_layerCount++] = L;
  }

  // Optional per-pixel payloads: [0]=layerCount repeated, then per layer
  //   [layerIndex][pixelCount][r,g,b ...]
  size_t off = need;
  if (len > off && b[off] == 0xFE) {
    off++;
    while (off + 2 <= len) {
      uint8_t li = b[off++];
      uint8_t pc = b[off++];
      if (off + (size_t)pc * 3 > len) break;
      if (li < g_layerCount) {
        uint8_t s = (pc > g_ledCount) ? g_ledCount : pc;
        memset(g_layers[li].pixels, 0, sizeof(g_layers[li].pixels));
        memcpy(g_layers[li].pixels, b + off, (size_t)s * 3);
        g_layers[li].hasPixels = true;
      }
      off += (size_t)pc * 3;
    }
  }

  // blend modes: packed in a trailing section [0xFD][blend per layer]
  if (len >= off + 1 && b[off] == 0xFD) {
    off++;
    for (uint8_t i = 0; i < g_layerCount && off + i < len; i++) {
      uint8_t m = b[off + i];
      g_layers[i].blend = m > BM_MUL ? BM_OVER : m;
    }
  }

  if (g_layerCount == 0) {
    // an empty push means "clear the stack": black and idle, like the preview
    g_mode = 0;
    g_playing = false;
    for (int i = 0; i < g_ledCount * 3; i++) g_base[i] = 0;
    showPixels(g_base);
  } else {
    // a pushed stack is a request to SEE it live: leave idle/show mode (and
    // resume if paused). Stream mode is left alone - frames keep arriving.
    if (g_mode != 3) g_mode = 1;
    g_playing = true;
    g_lastFxTick = millis();
  }

  pushLayers();
}

static void applyTimeline(uint8_t *b, size_t len) {
  // a header-only timeline (2 bytes, stepCount = 0) is a valid "clear the
  // stored show" request from the UI; anything shorter is noise
  if (len < 2) return;
  uint16_t count = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
  if (count > MAX_STEPS) count = MAX_STEPS;
  size_t need = 2 + (size_t)count * 4;
  if (len < need) { ws.broadcastTXT("{\"ev\":\"nack\",\"e\":\"timeline header short\"}"); return; }

  // free previous runs
  for (uint16_t i = 0; i < g_show.stepCount; i++) {
    if (g_show.steps[i].runs) { free(g_show.steps[i].runs); g_show.steps[i].runs = NULL; }
  }
  g_show.stepCount = 0;
  g_show.usedBytes = 0;

  size_t off = 2;
  uint16_t accepted = 0;
  for (uint16_t i = 0; i < count; i++) {
    Step &S = g_show.steps[accepted];
    S.duration   = (uint16_t)b[off] | ((uint16_t)b[off + 1] << 8);
    S.pixelCount = b[off + 2];
    S.blend      = b[off + 3] > BM_MUL ? BM_OVER : b[off + 3];
    S.runs = NULL; S.runsLen = 0;
    if (S.duration < 10) S.duration = 10;
    accepted++;
    off += 4;
  }

  for (uint16_t i = 0; i < accepted; i++) {
    Step &S = g_show.steps[i];
    uint16_t needBytes = (uint16_t)S.pixelCount * 3;
    if (off + needBytes > len) { accepted = i; break; }
    if (g_show.usedBytes + needBytes > MAX_SHOW_BYTES) { accepted = i; break; }
    S.runs = (uint8_t *)malloc(needBytes);
    if (!S.runs) { accepted = i; break; }
    memcpy(S.runs, b + off, needBytes);
    S.runsLen = needBytes;
    g_show.usedBytes += needBytes;
    off += needBytes;
  }

  g_show.stepCount = accepted;
  pushShow();
}

// Expand one RLE step into the 16-pixel buffer
static void expandStep(const Step &S, uint8_t *dst) {
  for (int i = 0; i < g_ledCount * 3; i++) dst[i] = 0;
  int px = 0;
  for (uint16_t o = 0; o + 3 < S.runsLen && px < g_ledCount; o += 4) {
    int run = S.runs[o] + 1;
    for (int j = 0; j < run && px < g_ledCount; j++, px++) {
      dst[px * 3 + 0] = S.runs[o + 1];
      dst[px * 3 + 1] = S.runs[o + 2];
      dst[px * 3 + 2] = S.runs[o + 3];
    }
  }
}

// ---------------------------------------------------------------------------
// WebSocket protocol
// ---------------------------------------------------------------------------
static void sendHello(uint8_t num) {
  char buf[340];
  // "brightness" is the EFFECTIVE value after the supply cap, so the UI shows
  // what the strip is really being driven at; "wantBrightness" is the user's
  // request, and "supplyMv" lets the UI explain any difference.
  snprintf(buf, sizeof(buf),
           "{\"ev\":\"hello\",\"fw\":\"1.0.0\",\"leds\":%u,\"maxLeds\":%u,"
           "\"pin\":%u,\"brightness\":%u,\"wantBrightness\":%u,\"supplyMv\":%u,"
           "\"gspd\":%u,\"fps\":%u,\"on\":%u,"
           "\"layers\":%u,\"steps\":%u,\"heap\":%u,\"ip\":\"%s\",\"showBytes\":%u}",
           g_ledCount, MAX_LEDS, g_dataPin, g_brightness, g_userBrightness,
           (unsigned)LED_SUPPLY_MV, g_globalSpdQ,
           (unsigned)(1000 / g_frameMs), g_masterOn ? 1 : 0, g_layerCount,
           g_show.stepCount, ESP.getFreeHeap(), WiFi.localIP().toString().c_str(),
           g_show.usedBytes);
  if (num == 0xFF) ws.broadcastTXT(buf); else ws.sendTXT(num, buf);
}

static void sendStats() {
  char buf[180];
  snprintf(buf, sizeof(buf),
           "{\"ev\":\"stats\",\"mode\":%u,\"play\":%u,\"step\":%u,\"heap\":%u,\"fps\":%u}",
           g_mode, g_playing ? 1 : 0, g_stepIdx, ESP.getFreeHeap(),
           (unsigned)(1000 / g_frameMs));
  ws.broadcastTXT(buf);
}

static void pushLayers() {
  // fixed buffer, not String: this fires on every slider drag and the String
  // version malloced twice per push, nibbling at the small ESP8266 heap
  char buf[48];
  snprintf(buf, sizeof(buf), "{\"ev\":\"layers\",\"n\":%u}", g_layerCount);
  ws.broadcastTXT(buf);
  sendStats();
}

static void pushShow() {
  char buf[96];
  snprintf(buf, sizeof(buf), "{\"ev\":\"show\",\"steps\":%u,\"bytes\":%u}",
           g_show.stepCount, g_show.usedBytes);
  ws.broadcastTXT(buf);
}

static void setPixelDirect(uint8_t *b, size_t len) {
  if (len < 2) return;
  uint8_t start = b[0];
  uint8_t count = b[1];
  if (start >= g_ledCount) return;
  if ((size_t)count * 3 + 2 > len) count = (len - 2) / 3;
  if (start + count > g_ledCount) count = g_ledCount - start;
  for (uint8_t i = 0; i < count; i++) {
    g_base[(start + i) * 3 + 0] = b[2 + i * 3 + 0];
    g_base[(start + i) * 3 + 1] = b[2 + i * 3 + 1];
    g_base[(start + i) * 3 + 2] = b[2 + i * 3 + 2];
    if (g_masterOn) strip.setPixelColor(start + i, b[2 + i * 3], b[2 + i * 3 + 1], b[2 + i * 3 + 2]);
  }
  g_mode = 0;
  g_playing = false;
  strip.show();
}

static void fillAll(uint8_t *b, size_t len) {
  if (len < 3) return;
  for (int i = 0; i < g_ledCount; i++) {
    g_base[i * 3 + 0] = b[0];
    g_base[i * 3 + 1] = b[1];
    g_base[i * 3 + 2] = b[2];
  }
  g_mode = 0;
  g_playing = false;
  showPixels(g_base);
}

static void applyStreamFrame(uint8_t *b, size_t len) {
  // [0] = start index, [1] = pixel count, then RGB triplets
  if (len < 2) return;
  uint8_t start = b[0];
  uint8_t count = b[1];
  if (start >= g_ledCount) return;
  if ((size_t)count * 3 + 2 > len) count = (len - 2) / 3;
  if (start + count > g_ledCount) count = g_ledCount - start;

  for (uint8_t i = 0; i < count; i++) {
    uint8_t r = b[2 + i * 3], g = b[3 + i * 3], bl = b[4 + i * 3];
    int o = (start + i) * 3;
    if (g_streamBlend == BM_OVER) {
      g_stream[o] = r; g_stream[o + 1] = g; g_stream[o + 2] = bl;
    } else {
      blendPixel(&g_stream[o], r, g, bl, g_streamBlend);
    }
  }
  g_mode = 3;
  showPixels(g_stream);
}

static void setPlayback(uint8_t *b, size_t len) {
  if (len < 1) return;
  uint8_t action = b[0];
  switch (action) {
    case 0: // stop - black, and it STAYS black (loop() no longer re-renders)
      g_playing = false;
      g_mode = 0;
      for (int i = 0; i < g_ledCount * 3; i++) g_base[i] = 0;
      showPixels(g_base);
      break;
    case 1: // play: the show if one is loaded, otherwise resume the live stack
      if (g_show.stepCount > 0) {
        g_mode = 2;
        g_playing = true;
        g_stepIdx = 0;
        g_stepStart = millis();
      } else {
        g_mode = 1;
        g_playing = true;
        g_lastFxTick = millis();
      }
      break;
    case 2: // pause - the animation clock freezes with it
      g_playing = false;
      break;
    case 3: // next step (only meaningful inside a show)
      if (g_mode == 2 && g_show.stepCount > 1) {
        g_stepIdx = (g_stepIdx + 1) % g_show.stepCount;
        g_stepStart = millis();
      }
      break;
    case 4: // restart the live effect clock
      g_fxClock = 0;
      g_lastFxTick = millis();
      g_mode = 1;
      g_playing = true;
      break;
  }
  sendStats();
}

// ---------------------------------------------------------------------------
// Flash persistence
// ---------------------------------------------------------------------------
static void saveConfig() {
  File f = LittleFS.open("/cfg.bin", "w");
  if (!f) { ws.broadcastTXT("{\"ev\":\"nack\",\"e\":\"open cfg failed\"}"); return; }
  f.write((uint8_t)1);                 // version
  f.write(g_ledCount);
  f.write(g_dataPin);
  f.write(g_userBrightness);           // store the intent, not the capped value
  f.write(g_globalSpdQ);
  f.write((uint8_t)(g_frameMs & 0xFF));
  f.write((uint8_t)(g_frameMs >> 8));
  f.write((uint8_t)(g_masterOn ? 1 : 0));
  f.write(g_layerCount);
  for (uint8_t i = 0; i < g_layerCount; i++) {
    Layer &L = g_layers[i];
    uint8_t rec[11] = { L.type, L.effect, L.c1.r, L.c1.g, L.c1.b,
                        L.c2.r, L.c2.g, L.c2.b, L.speedQ, L.bpmQ,
                        (uint8_t)((L.start & 0x0F) | ((L.end & 0x0F) << 4)) };
    f.write(rec, 11);
    f.write(L.blend);
    f.write((uint8_t)(L.hasPixels ? 1 : 0));
    if (L.hasPixels) f.write(L.pixels, g_ledCount * 3);
  }
  f.close();
  ws.broadcastTXT("{\"ev\":\"ack\",\"msg\":\"saved\"}");
}

static void loadConfig() {
  if (!LittleFS.exists("/cfg.bin")) return;
  File f = LittleFS.open("/cfg.bin", "r");
  if (!f) return;
  if (f.read() != 1) { f.close(); return; }
  g_ledCount   = f.read();
  g_dataPin    = f.read();
  g_userBrightness = f.read();         // user intent; capped when applied below
  g_globalSpdQ = f.read();
  uint8_t lo = f.read(), hi = f.read();
  g_frameMs = (uint16_t)lo | ((uint16_t)hi << 8);
  g_masterOn = f.read() != 0;
  uint8_t lc = f.read();
  if (lc > MAX_LAYERS) lc = MAX_LAYERS;
  g_layerCount = 0;
  for (uint8_t i = 0; i < lc && f.available(); i++) {
    Layer L; memset(&L, 0, sizeof(L));
    uint8_t rec[11];
    if (f.read(rec, 11) != 11) break;
    L.type = rec[0]; L.effect = rec[1];
    L.c1.r = rec[2]; L.c1.g = rec[3]; L.c1.b = rec[4];
    L.c2.r = rec[5]; L.c2.g = rec[6]; L.c2.b = rec[7];
    L.speedQ = rec[8]; L.bpmQ = rec[9];
    L.start = rec[10] & 0x0F; L.end = (rec[10] >> 4) & 0x0F;
    L.blend = f.read();
    uint8_t hp = f.read();
    if (hp) {
      if (f.read(L.pixels, g_ledCount * 3) == (int)(g_ledCount * 3)) L.hasPixels = true;
    }
    if (L.end >= g_ledCount) L.end = g_ledCount - 1;
    if (L.start > L.end) L.start = L.end;
    g_layers[g_layerCount++] = L;
  }
  f.close();
  strip.updateLength(g_ledCount);
  strip.setPin(g_dataPin);
  strip.begin();
  applyBrightness();                   // re-cap for the current supply setting
}

static void eraseConfig() {
  LittleFS.remove("/cfg.bin");
  ws.broadcastTXT("{\"ev\":\"ack\",\"msg\":\"erased\"}");
}

// [0]=ledCount [1]=dataPin - rebuilds the strip in place
static void applyHardware(uint8_t *b, size_t len) {
  if (len < 2) return;
  uint8_t n = b[0];
  uint8_t p = b[1];
  if (n < 1) n = 1;
  if (n > MAX_LEDS) n = MAX_LEDS;
  if (p > 16) p = PIN_NEOPIXEL;

  bool rebuild = (n != g_ledCount) || (p != g_dataPin);
  g_ledCount = n;
  g_dataPin = p;

  // clamp every stored range / pixel payload to the new geometry
  for (uint8_t i = 0; i < g_layerCount; i++) {
    if (g_layers[i].end >= n) g_layers[i].end = n - 1;
    if (g_layers[i].start > g_layers[i].end) g_layers[i].start = g_layers[i].end;
    if (!g_layers[i].hasPixels) continue;
    if (g_layers[i].end >= n) memset(&g_layers[i].pixels[n * 3], 0, sizeof(g_layers[i].pixels) - n * 3);
  }

  if (rebuild) {
    strip.updateLength(g_ledCount);
    strip.setPin(g_dataPin);
    strip.begin();
    applyBrightness();
    strip.show();
  }
  sendHello(0xFF);
}

// ---------------------------------------------------------------------------
// WebSocket event
// ---------------------------------------------------------------------------
uint16_t g_fragLen = 0;              // base64 accumulated for the current message

void onWsEvent(uint8_t num, uint8_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WS_EV_CONNECTED:
      Serial.printf("[ws] client %u connected from %s\n", num, ws.remoteIP(num).toString().c_str());
      g_fragLen = 0;
      g_showLen = 0;                     // never inherit a stale upload
      g_showActive = false;              // from a previous socket
      sendHello(num);
      break;

    case WS_EV_DISCONNECTED:
      Serial.printf("[ws] client %u disconnected\n", num);
      g_fragLen = 0;
      g_showLen = 0;
      g_showActive = false;
      // no clients: fall back to autonomous live rendering so the strip
      // keeps animating on its own
      if (ws.connectedClients() == 0 && g_mode == 3) {
        g_mode = 1; g_playing = true; g_lastFxTick = millis();
      }
      break;

    case WS_EV_TEXT: {
      if (length >= MAX_MSG_BYTES) {
        ws.sendTXT(num, "{\"ev\":\"nack\",\"e\":\"message too large\"}");
        return;
      }
      // trace every incoming message so a silent no-op is visible on serial.
      // No flush(): waiting for the UART to drain blocked loop() for ~5 ms per
      // message, which stacks up badly while sliders push on every input event.
      {
        char peek[56];
        size_t pn = length < sizeof(peek) - 1 ? length : sizeof(peek) - 1;
        memcpy(peek, payload, pn); peek[pn] = 0;
        Serial.printf("[ws] recv %u bytes: %s\n", (unsigned)length, peek);
      }
      memcpy(g_rx, payload, length);
      g_rx[length] = 0;
      const char *doc = (const char *)g_rx;

      char key[4] = { 0 };
      long id = 0;
      jsonInt(doc, "i", &id);
      if (!jsonString(doc, "k", key, sizeof(key))) {
        ws.sendTXT(num, "{\"ev\":\"nack\",\"e\":\"no key\"}");
        return;
      }

      // locate the base64 value in place (never copy it onto the stack)
      const char *vp = strstr(doc, "\"v\":\"");
      const char *vEnd = NULL;
      if (vp) { vp += 5; vEnd = strchr(vp, '"'); }

      const char k = key[0];

      // Two levels of reassembly:
      //   "f":1  starts a logical message, "m":1 means more fragments follow
      //   "p":1  starts a multi-message payload, "M":1 means more messages follow
      // The first level reassembles base64 in g_b64; the second appends DECODED
      // bytes into g_showbuf, which is how a show larger than one message
      // (MAX_MSG_BYTES is ~1 KB, a show can be 2.5 KB) gets through.
      long f = 0, m = 0, p = 0, more = 0;
      if (strstr(doc, "\"f\"")) jsonInt(doc, "f", &f);
      if (strstr(doc, "\"m\"")) jsonInt(doc, "m", &m);
      const bool isFrag = (f != 0) || (m != 0) || (strstr(doc, "\"f\"") != NULL);
      if (strstr(doc, "\"p\"")) jsonInt(doc, "p", &p);
      if (strstr(doc, "\"M\"")) jsonInt(doc, "M", &more);

      if (isFrag) {
        if (f == 1) g_fragLen = 0;
        if (vp && vEnd) {
          size_t n = (size_t)(vEnd - vp);
          if (g_fragLen + n >= sizeof(g_b64)) {
            g_fragLen = 0;
            ws.sendTXT(num, "{\"ev\":\"nack\",\"e\":\"payload overflow\"}");
            return;
          }
          memcpy(g_b64 + g_fragLen, vp, n);
          g_fragLen += (uint16_t)n;
        }
        if (m != 0) return;                     // wait for the remaining fragments
      }

      // decode this message's base64 into g_b64
      size_t dlen = 0;
      if (isFrag) {
        // In-place (dst == src == g_b64) is safe: the decoder advances its
        // output strictly slower than its input, so it never reads a byte it
        // has already overwritten.
        dlen = b64Decode((const char *)g_b64, g_fragLen, g_b64, sizeof(g_b64));
        g_fragLen = 0;
      } else if (vp && vEnd) {
        dlen = b64Decode(vp, (size_t)(vEnd - vp), g_b64, sizeof(g_b64));
      }

      // second level: accumulate decoded bytes across messages
      uint8_t *blob;
      if (p == 1) {
        g_showLen = 0;
        g_showActive = true;
        g_showKey = k;
      }
      // A different key while a multi-message upload is pending means that
      // upload was abandoned mid-way. Drop the stale bytes instead of
      // prefixing every later message with them - one interrupted upload used
      // to poison each request that followed until a page reload.
      if (g_showActive && k != g_showKey) {
        g_showLen = 0;
        g_showActive = false;
      }
      if (p != 0 || more != 0 || g_showActive) {
        if (g_showLen + dlen > sizeof(g_showbuf)) {
          g_showLen = 0; g_showActive = false;
          ws.sendTXT(num, "{\"ev\":\"nack\",\"e\":\"show payload overflow\"}");
          return;
        }
        memcpy(g_showbuf + g_showLen, g_b64, dlen);
        g_showLen += (uint16_t)dlen;
        if (more != 0) return;                  // wait for the remaining messages
        blob = g_showbuf;
        dlen = g_showLen;
        g_showLen = 0;
        g_showActive = false;
      } else {
        blob = g_b64;
      }

      if      (k == 'h') sendHello(num);
      else if (k == 'b') { applyGlobals(blob, dlen); }
      else if (k == 'B') { if (dlen >= 1) { g_userBrightness = blob[0]; applyBrightness(); strip.show(); } }
      else if (k == 'l') { applyLayers(blob, dlen); }
      else if (k == 't') { applyTimeline(blob, dlen); }
      else if (k == 'p') { setPixelDirect(blob, dlen); }
      else if (k == 'A') { fillAll(blob, dlen); }
      else if (k == 'c') { applyHardware(blob, dlen); }
      else if (k == 's') { setPlayback(blob, dlen); }
      else if (k == 'S') {                  // stream mode: [0]=on, [1]=blend
        if (dlen >= 1) {
          bool on = blob[0] != 0;
          if (dlen >= 2) g_streamBlend = blob[1] > BM_MUL ? BM_OVER : blob[1];
          if (on) {
            g_mode = 3; g_playing = false;
            for (int i = 0; i < g_ledCount * 3; i++) g_stream[i] = 0;
          } else if (g_mode == 3) {
            g_mode = 1; g_playing = true; g_lastFxTick = millis();
          }
        }
      }
      else if (k == 'x') { saveConfig(); return; }
      else if (k == 'F') { eraseConfig(); return; }
      else if (k == 'd') { printDiag(); ws.sendTXT(num, "{\"ev\":\"ack\",\"msg\":\"diag on serial\"}"); return; }
      else { ws.sendTXT(num, "{\"ev\":\"nack\",\"e\":\"unknown key\"}"); return; }

      char ack[48];
      snprintf(ack, sizeof(ack), "{\"ev\":\"ack\",\"i\":%ld,\"k\":\"%c\"}", id, k);
      ws.sendTXT(num, ack);
      break;
    }

    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Diagnostics - everything you need to debug a connection, on the serial line
// ---------------------------------------------------------------------------
static void printDiag() {
  Serial.println();
  Serial.println(F("---------------- diagnostics ----------------"));
  Serial.printf("  http        : http://%s:%u/\n",
                WiFi.localIP().toString().c_str(), (unsigned)HTTP_PORT);
  Serial.printf("  websocket   : ws://%s:%u/   <- the UI must use this\n",
                WiFi.localIP().toString().c_str(), (unsigned)WS_PORT);
  Serial.printf("  wifi        : %s  rssi %d dBm\n",
                WiFi.status() == WL_CONNECTED ? "connected" : "DISCONNECTED",
                (int)WiFi.RSSI());
  Serial.printf("  leds        : %u on GPIO%u, brightness %u\n",
                g_ledCount, g_dataPin, g_brightness);
  Serial.printf("  layers      : %u   steps %u   mode %u   playing %s\n",
                g_layerCount, g_show.stepCount, g_mode, g_playing ? "yes" : "no");
  Serial.printf("  ws clients  : %u\n", ws.connectedClients());
  Serial.printf("  free heap   : %u bytes\n", ESP.getFreeHeap());
  Serial.println(F("--------------------------------------------"));
  Serial.println();
}

// ---------------------------------------------------------------------------
// HTTP
// ---------------------------------------------------------------------------
// HTTP content
//
// The UI is available three ways, most preferred first:
//   1. /index.html.gz in LittleFS  - gzip, smallest transfer (optional)
//   2. /index.html    in LittleFS  - lets you iterate without reflashing
//   3. embedded in flash (PROGMEM) - always present, survives anything
//
// Level 3 is why a plain sketch upload is enough: there is no LittleFS data
// upload step to forget, and the board can never serve a 404 for its own UI.
// ---------------------------------------------------------------------------

// Stream the embedded copy. Served raw rather than gzip: hand-rolling a gzip
// stream with no CRC/ISIZE footer works in most browsers but is not something
// to bet the page on, and the HTTP layer here is too small to compress it.
//
// BACKPRESSURE MATTERS HERE. ESP8266WebServer::sendContent() ends in
// Stream::sendSize(), which gives up on a timeout and returns a SHORT count:
//     ssize_t sent = content->sendSize(&_currentClient, content_length);
//     if (sent != content_length) DBGWS("short send after timeout");
// It logs that and carries on. Pushing this ~80 KB page that way means the tail
// is silently dropped, the browser sees fewer bytes than Content-Length
// promised, and the load fails with ERR_CONTENT_LENGTH_MISMATCH. The truncated
// HTML then half-executes its <script>, which is why the WebSocket was "closed
// before the connection is established" and nothing ever transmitted.
//
// So we do our own flow control: only write what the socket will accept right
// now, and wait for space instead of overrunning it.
static void serveEmbeddedUI() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-UI-Source", "embedded");
  server.setContentLength((size_t)UI_EMBEDDED_LEN);
  server.send(200, "text/html; charset=utf-8", "");

  WiFiClient &client = server.client();
  char buf[320];                       // buffer is RAM; we write via client.write
  size_t off = 0;
  const size_t total = (size_t)UI_EMBEDDED_LEN;
  uint32_t lastProgress = millis();

  while (off < total) {
    if (!client.connected()) {
      Serial.printf("[http] client went away at %u/%u bytes\n", (unsigned)off, (unsigned)total);
      return;
    }

    // ask the socket how much room it has; never write more than that
    size_t room = client.availableForWrite();
    if (room == 0) {
      // socket buffer full: let the WiFi stack drain it and try again
      delay(1);
      yield();
      if (millis() - lastProgress > 15000) {
        Serial.printf("[http] stalled at %u/%u bytes, aborting\n", (unsigned)off, (unsigned)total);
        return;
      }
      continue;
    }

    size_t n = total - off;
    if (n > room)     n = room;
    if (n > sizeof(buf)) n = sizeof(buf);

    for (size_t i = 0; i < n; i++) buf[i] = (char)pgm_read_byte(UI_EMBEDDED + off + i);

    size_t wrote = client.write((const uint8_t *)buf, n);
    if (wrote == 0) { delay(1); yield(); continue; }
    off += wrote;
    lastProgress = millis();
    yield();
  }
  client.flush();
  Serial.printf("[http] served %u bytes\n", (unsigned)off);
}

static bool serveFile(String path) {
  if (path.endsWith("/")) path += "index.html";

  // gzip variant first - a 20 KB transfer instead of 74 KB
  String gz = path + ".gz";
  if (path == "/index.html" && LittleFS.exists(gz)) {
    File f = LittleFS.open(gz, "r");
    if (f) {
      server.sendHeader("Content-Encoding", "gzip");
      server.sendHeader("Cache-Control", "no-store");
      server.streamFile(f, "text/html; charset=utf-8");
      f.close();
      return true;
    }
  }

  if (LittleFS.exists(path)) {
    File f = LittleFS.open(path, "r");
    if (f) { server.streamFile(f, "text/html; charset=utf-8"); f.close(); return true; }
  }

  if (path == "/index.html") { serveEmbeddedUI(); return true; }
  return false;
}

void setupHttp() {
  server.on("/", HTTP_GET, []() { serveFile("/index.html"); });
  server.on("/api/state", HTTP_GET, []() {
    char buf[320];
    snprintf(buf, sizeof(buf),
             "{\"leds\":%u,\"pin\":%u,\"brightness\":%u,\"gspd\":%u,\"frameMs\":%u,"
             "\"on\":%u,\"layers\":%u,\"steps\":%u,\"mode\":%u,\"play\":%u,"
             "\"heap\":%u,\"showBytes\":%u}",
             g_ledCount, g_dataPin, g_brightness, g_globalSpdQ, g_frameMs,
             g_masterOn ? 1 : 0, g_layerCount, g_show.stepCount, g_mode,
             g_playing ? 1 : 0, ESP.getFreeHeap(), g_show.usedBytes);
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", buf);
  });
  server.on("/api/fill", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      String body = server.arg("plain");
      long r = 0, g = 0, b = 0;
      // very small parser for {"r":n,"g":n,"b":n}
      const char *c = body.c_str();
      const char *p;
      if ((p = strstr(c, "\"r\""))) r = strtol(p + 3, NULL, 10);
      if ((p = strstr(c, "\"g\""))) g = strtol(p + 3, NULL, 10);
      if ((p = strstr(c, "\"b\""))) b = strtol(p + 3, NULL, 10);
      g_base[0] = clamp8(r); g_base[1] = clamp8(g); g_base[2] = clamp8(b);
      uint8_t blob[3] = { clamp8(r), clamp8(g), clamp8(b) };
      fillAll(blob, 3);
      server.send(200, "application/json", "{\"ok\":true}");
    } else {
      server.send(400, "application/json", "{\"ok\":false}");
    }
  });
  server.onNotFound([]() {
    String path = server.uri();
    if (path == "/" ) path = "/index.html";
    if (!serveFile(path)) server.send(404, "text/plain", "not found");
  });
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
// Set to 1 to print a marker before every boot step. Left on deliberately: the
// board can only report progress over serial, and a silent crash between two
// markers is otherwise impossible to localise. Each marker is flushed, so the
// last one printed names the step that failed.
#define BOOT_TRACE 1
static void bootStep(const char *what) {
#if BOOT_TRACE
  Serial.print(F("[boot] "));
  Serial.println(what);
  Serial.flush();
#endif
}

void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println();
  Serial.println(F("=== NeoPixel Studio (ESP8266) ==="));
  Serial.printf("[boot] reset reason: %s\n", ESP.getResetReason().c_str());
  Serial.printf("[boot] chip id %08x  flash %u KB  sketch %u KB\n",
                ESP.getChipId(), ESP.getFlashChipRealSize() / 1024,
                ESP.getSketchSize() / 1024);
  Serial.flush();

  bootStep("clear pixel buffers");
  for (int i = 0; i < g_ledCount * 3; i++) { g_base[i] = 0; g_stream[i] = 0; }

  bootStep("strip.begin()");
  strip.begin();

  bootStep("strip brightness + show");
  applyBrightness();                   // caps brightness for the supply setting
  strip.show();                        // all off while booting
  Serial.printf("[pwr] supply %u mV -> brightness %u (requested %u)\n",
                (unsigned)LED_SUPPLY_MV, g_brightness, g_userBrightness);
  if (LED_SUPPLY_MV < 4500) {
    Serial.println(F("[pwr] 3.3 V supply: current is capped on purpose."));
    Serial.println(F("[pwr] A WS2812B wants 5 V - expect weak/blue-starved colour,"));
    Serial.println(F("[pwr] and consider a separate 5 V supply. Common ground is still required."));
  }
  Serial.flush();

  bootStep("LittleFS mount");
  if (!LittleFS.begin()) {
    Serial.println(F("[fs] LittleFS mount failed - formatting (this takes a few seconds)"));
    Serial.flush();
    LittleFS.format();
    LittleFS.begin();
  }
  // Note: fs::FS in ESP8266 core 3.x has no usedBytes()/totalBytes(); those live
  // on FSConfig/FSInfo. Do not call them on the FS object.
  FSInfo fsinfo;
  if (LittleFS.info(fsinfo)) {
    Serial.printf("[fs] mounted, %u KB total, %u KB used\n",
                  (unsigned)(fsinfo.totalBytes / 1024), (unsigned)(fsinfo.usedBytes / 1024));
  } else {
    Serial.println(F("[fs] mounted (no size info)"));
  }
  Serial.flush();

  bootStep("load saved config");
  loadConfig();

  bootStep("wifi mode + hostname");
  WiFi.mode(WIFI_STA);
  WiFi.hostname(OTA_HOSTNAME);

  bootStep("wifi begin");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[wifi] connecting to %s", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(400);
    Serial.print('.');
    yield();                           // never starve the WDT while waiting
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[wifi] connected: http://%s/\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println(F("[wifi] failed - starting soft AP 'NeoPixel-Studio'"));
    WiFi.mode(WIFI_AP);
    WiFi.softAP("NeoPixel-Studio", "neopixels");
    Serial.printf("[wifi] AP ip: http://%s/\n", WiFi.softAPIP().toString().c_str());
  }
  Serial.flush();

  bootStep("http routes");
  setupHttp();
  bootStep("http server begin");
  server.begin();
  bootStep("websocket begin");
  ws.onEvent(onWsEvent);
  ws.begin();

  // Never boot into darkness. With no stored config there are no layers, so the
  // strip would sit black until a browser connected and pushed something - which
  // looks exactly like dead hardware. Install a visible default instead, so a
  // freshly flashed board proves its wiring immediately.
  if (g_layerCount == 0) {
    Layer L;
    memset(&L, 0, sizeof(L));
    L.type   = LT_EFFECT;
    L.effect = FX_RAINBOW_CYCLE;
    L.c1.r = 255; L.c1.g = 0;   L.c1.b = 0;
    L.c2.r = 0;   L.c2.g = 0;   L.c2.b = 0;
    L.speedQ = 6;
    L.bpmQ   = 60;
    L.start  = 0;
    L.end    = g_ledCount - 1;
    L.blend  = BM_OVER;
    L.hasPixels = false;
    g_layers[g_layerCount++] = L;
    Serial.println(F("[fx] no saved config - starting with a rainbow cycle"));
  }

  g_fxClock = 0;
  g_lastFxTick = millis();
  g_mode = 1;
  g_playing = true;
  renderFrame(g_base);                 // prove the strip works on the first boot
  showPixels(g_base);

  Serial.printf("[leds] %u pixels on GPIO%u, brightness %u\n",
                g_ledCount, g_dataPin, g_brightness);
  Serial.printf("[sys] free heap: %u bytes\n", ESP.getFreeHeap());
  printDiag();
}

void loop() {
  server.handleClient();
  ws.loop();

  const uint32_t now = millis();
  if (now - g_lastFrame < g_frameMs) return;
  g_lastFrame = now;

  if (g_mode == 3) return;             // stream mode: pixels arrive by ws
  if (g_mode == 0) return;             // idle: fills / direct pixel edits hold

  if (g_mode == 2) {                   // show timeline
    if (g_playing && g_show.stepCount) {
      if (now - g_stepStart >= g_show.steps[g_stepIdx].duration) {
        g_stepStart = now;
        g_stepIdx = (g_stepIdx + 1) % g_show.stepCount;
        static uint16_t lastReported = 0xFFFF;
        if (g_stepIdx != lastReported) {
          char buf[48];
          snprintf(buf, sizeof(buf), "{\"ev\":\"step\",\"i\":%u}", g_stepIdx);
          ws.broadcastTXT(buf);
          lastReported = g_stepIdx;
        }
      }
      expandStep(g_show.steps[g_stepIdx], g_base);
      showPixels(g_base);
    }
    return;                            // paused show: hold the current step
  }

  // mode 1: live engine. The old condition `g_playing || g_layerCount`
  // re-rendered the stack even when stopped or paused, which overwrote Stop,
  // fills and direct pixel writes within one frame and made Pause freeze
  // nothing - the "every control except the quick-effect buttons is dead"
  // symptom.
  if (!g_playing) return;              // paused: freeze the last frame
  if (!g_layerCount) return;
  g_fxClock += now - g_lastFxTick;     // animation time only advances while playing
  g_lastFxTick = now;
  renderFrame(g_base);
  showPixels(g_base);
}