/*******************************************************
 * ESP32-S3 Web UI (on-device) + X4M03 Control  (RF + BB)
 * - UI like desktop app (buttons + status)
 * - Endpoints:
 *   GET  /            -> Web UI
 *   GET  /api/status  -> JSON status (requires Authorization header)
 *   POST /api/cmd     -> command (Connect/Disconnect/RunRF/RunBB/Stop/Reboot/Record)
 *
 * Radar:
 * - UART to X4M03
 * - Manual mode + RF preset + BB preset
 * - STOP button => Stop + Auto reboot radar (as requested)
 *
 * Notes:
 * - This version updates meta (FrameCounter, NFloats, etc.) from A0/12 packets.
 * - FramesStored counts "frames" by detecting changes in frameCounter (not per packet).
 *******************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include "esp_timer.h"

/* ================= WiFi (SoftAP) ================= */
#define USE_SOFTAP 1
const char* AP_SSID = "ESP-Radar";
const char* AP_PASS = "radar1234";

/* ================= Web Server ================= */
WebServer server(80);

/* ================= Simple Auth (optional) =================
 * UI sends header: Authorization: <API_TOKEN>
 */
static const char* API_TOKEN = "XUxkK&4Oqj!zS5JXYUu5w!04piUO3&u8";
static bool isAuthorized() {
  if (!server.hasHeader("Authorization")) return false;
  return server.header("Authorization") == String(API_TOKEN);
}

/* ================= Pins / UART ================= */
#define RADAR_UART_NUM 1
#define RADAR_RX_PIN 15
#define RADAR_TX_PIN 16
#define RADAR_RESET_PIN 5

#ifndef LED_PIN
#define LED_PIN 48
#endif

HardwareSerial RadarSerial(RADAR_UART_NUM);

/* ================= NeoPixel ================= */
Adafruit_NeoPixel pixels(1, LED_PIN, NEO_GRB + NEO_KHZ800);
static inline void led_init() {
  pixels.begin();
  pixels.clear();
  pixels.show();
}
static inline void led_set(uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(0, g, b));
  pixels.show();
}

/* ================= MCP constants ================= */
const uint8_t XT_START = 0x7D, XT_STOP = 0x7E, XT_ESC = 0x7F;

const uint8_t XTS_SPC_MOD_SETMODE = 0x20;
const uint8_t XTS_SPC_X4DRIVER = 0x50;
const uint8_t XTS_SPCX_SET = 0x10;
const uint8_t XTS_SPR_SYSTEM = 0x30;
const uint8_t XTS_SPR_ACK = 0x10;

const uint8_t XTS_SM_STOP = 0x13;
const uint8_t XTS_SM_MANUAL = 0x12;

const uint8_t XTS_SPC_DIR_COMMAND = 0x90;
const uint8_t XTS_SDC_COMM_SETBAUDRATE = 0x80;

const uint32_t XTS_SPCXI_FPS = 0x00000010;
const uint32_t XTS_SPCXI_PULSES_PER_STEP = 0x00000011;
const uint32_t XTS_SPCXI_ITERATIONS = 0x00000012;
const uint32_t XTS_SPCXI_DOWNCONVERSION = 0x00000013;  // 0=RF, 1=BB
const uint32_t XTS_SPCXI_FRAME_AREA = 0x00000014;
const uint32_t XTS_SPCXI_DAC_STEP = 0x00000015;
const uint32_t XTS_SPCXI_DAC_MIN = 0x00000016;
const uint32_t XTS_SPCXI_DAC_MAX = 0x00000017;
const uint32_t XTS_SPCXI_FRAME_AREA_OFFSET = 0x00000018;
const uint32_t XTS_SPCXI_ENABLE = 0x00000019;
const uint32_t XTS_SPCXI_PRF_DIV = 0x00000020;

const uint32_t XTS_SPRS_READY = 0x00000011;

/* ================= App State =================
RunStatus:
0  Idle
1  RunBB
2  RunRF
10 RecordBB (later)
20 RecordRF (later)
*/
static volatile int RunStatus = 0;
static volatile bool RadarConnected = false;

static volatile uint32_t FrameCounter = 0;
static volatile uint32_t FramesStored = 0;
static volatile uint32_t LastPktLen = 0;
static volatile uint32_t LastNFloats = 0;
static volatile uint32_t LastBytesRem = 0;

static uint32_t bootMs = 0;

/* ================= Helpers ================= */
static inline bool waitAvail(Stream& s, uint32_t to_ms) {
  uint32_t t0 = millis();
  while (!s.available() && (millis() - t0) < to_ms) delay(1);
  return s.available();
}
static inline uint32_t u32le(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static size_t radarWriteEscaped(const uint8_t* buf, size_t len) {
  uint8_t crc = XT_START;
  for (size_t i = 0; i < len; i++) crc ^= buf[i];

  auto w = [&](uint8_t b) {
    if (b == XT_START || b == XT_STOP || b == XT_ESC) RadarSerial.write(XT_ESC);
    RadarSerial.write(b);
  };

  RadarSerial.write(XT_START);
  for (size_t i = 0; i < len; i++) w(buf[i]);
  w(crc);
  RadarSerial.write(XT_STOP);
  return len + 3;
}

static inline void sendSetMode(uint8_t mode) {
  uint8_t p[2] = { XTS_SPC_MOD_SETMODE, mode };
  radarWriteEscaped(p, sizeof(p));
}
static inline void sendSet_u8(uint32_t id, uint8_t v) {
  uint8_t p[1 + 1 + 4 + 1] = { XTS_SPC_X4DRIVER, XTS_SPCX_SET };
  memcpy(&p[2], &id, 4);
  p[6] = v;
  radarWriteEscaped(p, sizeof(p));
}
static inline void sendSet_u32(uint32_t id, uint32_t v) {
  uint8_t p[1 + 1 + 4 + 4] = { XTS_SPC_X4DRIVER, XTS_SPCX_SET };
  memcpy(&p[2], &id, 4);
  memcpy(&p[6], &v, 4);
  radarWriteEscaped(p, sizeof(p));
}
static inline void sendSet_f32(uint32_t id, float v) {
  uint8_t p[1 + 1 + 4 + 4] = { XTS_SPC_X4DRIVER, XTS_SPCX_SET };
  memcpy(&p[2], &id, 4);
  memcpy(&p[6], &v, 4);
  radarWriteEscaped(p, sizeof(p));
}
static inline void sendSet_frame_area(float s, float e) {
  uint8_t p[1 + 1 + 4 + 4 + 4] = { XTS_SPC_X4DRIVER, XTS_SPCX_SET };
  uint32_t id = XTS_SPCXI_FRAME_AREA;
  memcpy(&p[2], &id, 4);
  memcpy(&p[6], &s, 4);
  memcpy(&p[10], &e, 4);
  radarWriteEscaped(p, sizeof(p));
}
static inline void sendSetBaud(uint32_t baud) {
  uint8_t p[1 + 1 + 4] = { XTS_SPC_DIR_COMMAND, XTS_SDC_COMM_SETBAUDRATE };
  memcpy(&p[2], &baud, 4);
  radarWriteEscaped(p, sizeof(p));
}

static inline void radarResetHW() {
  pinMode(RADAR_TX_PIN, OUTPUT);
  digitalWrite(RADAR_TX_PIN, HIGH);
  pinMode(RADAR_RESET_PIN, OUTPUT);
  digitalWrite(RADAR_RESET_PIN, LOW);
  delay(120);
  digitalWrite(RADAR_RESET_PIN, HIGH);
  delay(200);
  pinMode(RADAR_TX_PIN, INPUT);
}

static inline bool expectAck(uint32_t to = 1500) {
  uint32_t t0 = millis();
  while (millis() - t0 < to) {
    if (!waitAvail(RadarSerial, 10)) continue;
    if (RadarSerial.read() != XT_START) continue;

    uint8_t first = 0;
    while (waitAvail(RadarSerial, 50)) {
      uint8_t b = RadarSerial.read();
      if (first == 0 && b != XT_START) first = b;
      if (b == XT_STOP) break;
    }
    if (first == XTS_SPR_ACK) return true;
  }
  return false;
}

static inline bool waitForReady(uint32_t to = 4000) {
  uint32_t t0 = millis();
  while (millis() - t0 < to) {
    if (!waitAvail(RadarSerial, 5)) continue;
    if (RadarSerial.read() != XT_START) continue;

    uint8_t b[12];
    size_t n = 0;
    while (waitAvail(RadarSerial, 50) && n < sizeof(b)) {
      b[n++] = RadarSerial.read();
      if (b[n - 1] == XT_STOP) break;
    }
    if (n >= 6 && b[0] == XTS_SPR_SYSTEM) {
      uint32_t code = (uint32_t)b[1] | ((uint32_t)b[2] << 8) | ((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 24);
      if (code == XTS_SPRS_READY) return true;
    }
  }
  return false;
}

/* ================= Presets ================= */
static bool apply_preset_manual_RF(float fps) {
  bool ok = true;

  sendSet_u8(XTS_SPCXI_ENABLE, 1);
  ok &= expectAck();
  sendSet_u32(XTS_SPCXI_PULSES_PER_STEP, 3);
  ok &= expectAck();
  sendSet_u32(XTS_SPCXI_ITERATIONS, 16);
  ok &= expectAck();
  sendSet_u8(XTS_SPCXI_DOWNCONVERSION, 0);
  ok &= expectAck();  // RF

  sendSet_u32(XTS_SPCXI_DAC_MIN, 900);
  ok &= expectAck();
  sendSet_u32(XTS_SPCXI_DAC_MAX, 1150);
  ok &= expectAck();
  sendSet_u32(XTS_SPCXI_DAC_STEP, 1);
  ok &= expectAck();
  sendSet_u32(XTS_SPCXI_PRF_DIV, 3);
  ok &= expectAck();

  sendSet_f32(XTS_SPCXI_FRAME_AREA_OFFSET, 0.0f);
  ok &= expectAck();
  sendSet_frame_area(0.30f, 1.00f);
  ok &= expectAck();

  sendSet_f32(XTS_SPCXI_FPS, 0.0f);
  ok &= expectAck();
  sendSet_f32(XTS_SPCXI_FPS, fps);
  ok &= expectAck();

  return ok;
}

static bool apply_preset_manual_BB(float fps) {
  bool ok = true;

  sendSet_u8(XTS_SPCXI_ENABLE, 1);
  ok &= expectAck();
  sendSet_u32(XTS_SPCXI_PULSES_PER_STEP, 3);
  ok &= expectAck();
  sendSet_u32(XTS_SPCXI_ITERATIONS, 16);
  ok &= expectAck();
  sendSet_u8(XTS_SPCXI_DOWNCONVERSION, 1);
  ok &= expectAck();  // BB

  sendSet_u32(XTS_SPCXI_DAC_MIN, 900);
  ok &= expectAck();
  sendSet_u32(XTS_SPCXI_DAC_MAX, 1150);
  ok &= expectAck();
  sendSet_u32(XTS_SPCXI_DAC_STEP, 1);
  ok &= expectAck();
  sendSet_u32(XTS_SPCXI_PRF_DIV, 3);
  ok &= expectAck();

  sendSet_f32(XTS_SPCXI_FRAME_AREA_OFFSET, 0.0f);
  ok &= expectAck();
  sendSet_frame_area(0.30f, 1.00f);
  ok &= expectAck();

  sendSet_f32(XTS_SPCXI_FPS, 0.0f);
  ok &= expectAck();
  sendSet_f32(XTS_SPCXI_FPS, fps);
  ok &= expectAck();

  return ok;
}

/* ================= NoEscape packet reader =================
 * Format:
 *   7C 7C 7C 7C
 *   len(u32le)  -> payload length
 *   00          -> reserved
 *   payload[len]
 */
static bool readNoEscapePacket(uint8_t* payload, uint32_t cap, uint32_t& pkt_len_out) {
  uint8_t h[4] = { 0, 0, 0, 0 };

  // sync
  while (true) {
    if (!RadarSerial.available()) return false;
    h[0] = h[1];
    h[1] = h[2];
    h[2] = h[3];
    h[3] = (uint8_t)RadarSerial.read();
    if (h[0] == 0x7C && h[1] == 0x7C && h[2] == 0x7C && h[3] == 0x7C) break;
  }

  while (RadarSerial.available() < 4) delay(1);
  uint8_t lb[4];
  RadarSerial.readBytes((char*)lb, 4);
  uint32_t pkt_len = u32le(lb);

  // sanity
  if (pkt_len < 8 || pkt_len > cap) {
    if (RadarSerial.available()) (void)RadarSerial.read();  // reserved
    pkt_len_out = 0;
    return true;
  }

  while (!RadarSerial.available()) delay(1);
  (void)RadarSerial.read();  // reserved

  uint32_t got = 0;
  while (got < pkt_len) {
    if (!RadarSerial.available()) {
      delay(1);
      continue;
    }
    got += RadarSerial.readBytes((char*)(&payload[got]), pkt_len - got);
  }

  pkt_len_out = pkt_len;
  return true;
}

/* ================= Radar Control ================= */
static void radarConnect() {
  if (RadarConnected) return;

  radarResetHW();
  RadarSerial.setRxBufferSize(32768);
  RadarSerial.begin(115200, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  waitForReady(3500);

  sendSetMode(XTS_SM_STOP);
  expectAck();
  sendSetMode(XTS_SM_MANUAL);
  expectAck();

  sendSetBaud(921600);
  if (expectAck()) {
    RadarSerial.updateBaudRate(921600);
    delay(80);
  }

  RadarConnected = true;
}

static void radarDisconnect() {
  if (!RadarConnected) return;
  sendSetMode(XTS_SM_STOP);
  expectAck();
  RadarSerial.end();
  RadarConnected = false;
  RunStatus = 0;
}

static void radarRunRF() {
  if (!RadarConnected) radarConnect();
  FramesStored = 0;
  FrameCounter = 0;
  apply_preset_manual_RF(5.0f);
  RunStatus = 2;
}

static void radarRunBB() {
  if (!RadarConnected) radarConnect();
  FramesStored = 0;
  FrameCounter = 0;
  apply_preset_manual_BB(5.0f);
  RunStatus = 1;
}

static void radarStop() {
  if (!RadarConnected) {
    RunStatus = 0;
    return;
  }
  sendSetMode(XTS_SM_STOP);
  expectAck();
  RunStatus = 0;
}

static void radarStopAndReboot() {
  radarDisconnect();  // end UART + connected=false
  delay(200);
  radarConnect();  // begin 115200 + ready + manual + set 921600
}
static void radarReboot() {
  radarStop();
  radarDisconnect();
  delay(200);
  radarConnect();
}

/* ================= Radar Reader Task ================= */
static TaskHandle_t radarTaskHandle = nullptr;

void radarTask(void*) {
  static uint8_t payload[16384];
  static uint32_t lastFrame = 0xFFFFFFFF;

  while (true) {
    if ((RunStatus == 1 || RunStatus == 2) && RadarConnected) {
      uint32_t pkt_len = 0;
      bool ok = readNoEscapePacket(payload, sizeof(payload), pkt_len);

      if (ok && pkt_len > 0) {
        // Parse only Generic Float A0/12 meta
        if (pkt_len >= 16 && payload[0] == 0xA0 && payload[1] == 0x12) {
          const uint8_t* p = payload + 2;
          uint32_t contentId = u32le(p);
          p += 4;
          (void)contentId;
          uint32_t frameCounter = u32le(p);
          p += 4;
          uint32_t flen = u32le(p);
          p += 4;

          uint32_t bytes_rem = (uint32_t)((payload + pkt_len) - p);
          uint32_t data_bytes = 0;
          if (bytes_rem >= flen * 4) data_bytes = flen * 4;
          else if ((flen % 4) == 0 && bytes_rem >= flen) data_bytes = flen;
          else data_bytes = bytes_rem - (bytes_rem % 4);

          uint32_t nFloats = data_bytes / 4;

          // BB: I-only => half
          if (RunStatus == 1) nFloats = nFloats / 2;

          FrameCounter = frameCounter;
          LastPktLen = pkt_len;
          LastBytesRem = bytes_rem;
          LastNFloats = nFloats;

          // Count frames by frameCounter changes
          if (lastFrame == 0xFFFFFFFF) {
            lastFrame = frameCounter;
          } else if (frameCounter != lastFrame) {
            FramesStored++;
            lastFrame = frameCounter;
          }
        }
      }

      // LED: BB -> cyan/green, RF -> blue
      if (RunStatus == 1) led_set(90, 90);
      else led_set(20, 120);

      vTaskDelay(1);
    } else {
      led_set(5, 5);
      lastFrame = 0xFFFFFFFF;
      vTaskDelay(20 / portTICK_PERIOD_MS);
    }
  }
}

/* ================= Web UI HTML ================= */
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="fa" dir="rtl">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>ESP Radar Control</title>
  <style>
    body{font-family:system-ui,-apple-system,Segoe UI,Roboto; margin:0; background:#0b0f14; color:#e8eef7;}
    .wrap{max-width:1100px; margin:auto; padding:18px;}
    .top{display:flex; gap:12px; flex-wrap:wrap; align-items:center; justify-content:space-between;}
    .card{background:#111826; border:1px solid #1f2a3a; border-radius:14px; padding:14px;}
    .grid{display:grid; grid-template-columns: 330px 1fr; gap:14px; margin-top:14px;}
    @media (max-width:900px){ .grid{grid-template-columns:1fr;} }
    button{border:0; padding:10px 12px; border-radius:12px; cursor:pointer; font-weight:600;}
    .btn{background:#1f2a3a; color:#e8eef7;}
    .btn:hover{filter:brightness(1.15);}
    .btn.primary{background:#2563eb;}
    .btn.danger{background:#dc2626;}
    .btn.good{background:#16a34a;}
    .row{display:flex; gap:10px; flex-wrap:wrap;}
    .kv{display:grid; grid-template-columns: 1fr 1fr; gap:8px; margin-top:10px;}
    .k{opacity:.75;}
    .v{font-family:ui-monospace, SFMono-Regular, Menlo, monospace;}
    .hdr{font-size:14px; opacity:.8}
    input{background:#0b0f14; color:#e8eef7; border:1px solid #1f2a3a; border-radius:10px; padding:10px; width:260px;}
    .note{opacity:.7; font-size:13px; margin-top:10px; line-height:1.6}
    .big{min-height:340px; display:flex; align-items:center; justify-content:center; border:1px dashed #2a3a52; border-radius:14px; color:#94a3b8;}
  </style>
</head>
<body>
<div class="wrap">
  <div class="top">
    <div>
      <div style="font-size:18px;font-weight:800">کنترل رادار روی ESP</div>
      <div class="hdr">UI برای Run/Stop/Reboot و نمایش وضعیت</div>
    </div>
    <div class="card">
      <div class="row" style="align-items:center">
        <span class="k">Token:</span>
        <input id="token" placeholder="Authorization Token" />
        <button class="btn" onclick="saveToken()">ذخیره</button>
      </div>
    </div>
  </div>

  <div class="grid">
    <div class="card">
      <div style="font-weight:800;margin-bottom:10px">کنترل</div>
      <div class="row">
        <button class="btn good" onclick="cmd('Connect')">Connect</button>
        <button class="btn" onclick="cmd('Disconnect')">Disconnect</button>
        <button class="btn primary" onclick="cmd('RunRF')">Run RF</button>
        <button class="btn primary" onclick="cmd('RunBB')">Run BB</button>
        <button class="btn danger" onclick="cmd('Stop')">Stop (Stop+Reboot)</button>
        <button class="btn" onclick="cmd('Record')">Record</button>
        <button class="btn danger" onclick="cmd('Reboot')">Reboot Radar</button>
      </div>

      <div class="note">
        Stop شامل Stop + ریبوت اتومات رادار است (طبق درخواست).
      </div>

      <div style="font-weight:800;margin-top:16px;margin-bottom:6px">وضعیت</div>
      <div class="kv">
        <div class="k">IP</div><div class="v" id="ip">-</div>
        <div class="k">Connected</div><div class="v" id="conn">-</div>
        <div class="k">RunStatus</div><div class="v" id="rs">-</div>
        <div class="k">FrameCounter</div><div class="v" id="fc">-</div>
        <div class="k">FramesStored</div><div class="v" id="fs">-</div>
        <div class="k">BytesInBuf</div><div class="v" id="bib">-</div>
        <div class="k">LastPktLen</div><div class="v" id="lpl">-</div>
        <div class="k">LastNFloats</div><div class="v" id="lnf">-</div>
        <div class="k">Uptime(ms)</div><div class="v" id="up">-</div>
      </div>
    </div>

    <div class="card">
      <div style="font-weight:800;margin-bottom:10px">نمایش</div>
      <div class="big">مرحله بعدی: استریم داده و رسم نمودار داخل مرورگر (WebSocket/SSE)</div>
    </div>
  </div>
</div>

<script>
function saveToken(){
  localStorage.setItem("api_token", document.getElementById("token").value || "");
}
function getToken(){
  return localStorage.getItem("api_token") || "";
}
document.getElementById("token").value = getToken();

async function cmd(c){
  const t = getToken();
  const r = await fetch("/api/cmd",{
    method:"POST",
    headers:{
      "Content-Type":"application/json",
      "Authorization": t
    },
    body: JSON.stringify({Command:c})
  });
  if(!r.ok){
    alert("HTTP "+r.status);
    return;
  }
  await refresh();
}

async function refresh(){
  const t = getToken();
  const r = await fetch("/api/status", {headers:{"Authorization": t}});
  if(!r.ok){
    return;
  }
  const j = await r.json();
  document.getElementById("ip").textContent = j.ip;
  document.getElementById("conn").textContent = j.connected;
  document.getElementById("rs").textContent = j.run_status;
  document.getElementById("fc").textContent = j.frame_counter;
  document.getElementById("fs").textContent = j.frames_stored;
  document.getElementById("bib").textContent = j.bytes_in_buf;
  document.getElementById("lpl").textContent = j.last_pkt_len;
  document.getElementById("lnf").textContent = j.last_n_floats;
  document.getElementById("up").textContent = j.uptime_ms;
}

setInterval(refresh, 600);
refresh();
</script>
</body>
</html>
)HTML";

/* ================= Handlers ================= */
void handleRoot() {
  server.send(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  if (!isAuthorized()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  String ip = WiFi.isConnected() ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  uint32_t uptime = millis() - bootMs;
  uint32_t bytesInBuf = RadarConnected ? RadarSerial.available() : 0;

  String json = "{";
  json += "\"ip\":\"" + ip + "\",";
  json += "\"connected\":" + String(RadarConnected ? "true" : "false") + ",";
  json += "\"run_status\":" + String(RunStatus) + ",";
  json += "\"frame_counter\":" + String((uint32_t)FrameCounter) + ",";
  json += "\"frames_stored\":" + String((uint32_t)FramesStored) + ",";
  json += "\"bytes_in_buf\":" + String(bytesInBuf) + ",";
  json += "\"last_pkt_len\":" + String((uint32_t)LastPktLen) + ",";
  json += "\"last_n_floats\":" + String((uint32_t)LastNFloats) + ",";
  json += "\"uptime_ms\":" + String(uptime);
  json += "}";
  server.send(200, "application/json", json);
}

void handleCmd() {
  if (!isAuthorized()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }

  String body = server.arg("plain");
  int i = body.indexOf("Command");
  if (i < 0) {
    server.send(400, "text/plain", "No Command");
    return;
  }

  int q1 = body.indexOf('"', i + 7);
  q1 = body.indexOf('"', q1 + 1);
  int q2 = body.indexOf('"', q1 + 1);
  String cmd = body.substring(q1 + 1, q2);

  if (cmd == "Connect") radarConnect();
  else if (cmd == "Disconnect") radarDisconnect();
  else if (cmd == "RunRF") radarRunRF();
  else if (cmd == "RunBB") radarRunBB();
  else if (cmd == "Stop") radarStopAndReboot();  // ✅ Stop + Auto Reboot
  else if (cmd == "Reboot") radarReboot();
  else if (cmd == "Record") { /* placeholder */ } else {
    server.send(400, "text/plain", "Unknown Command");
    return;
  }

  server.send(200, "application/json", "{\"ok\":true}");
}

/* ================= Setup / Loop ================= */
void setup() {
  Serial.begin(115200);
  delay(150);
  bootMs = millis();

  led_init();
  led_set(5, 5);

#if USE_SOFTAP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
#else
  WiFi.mode(WIFI_STA);
  WiFi.begin("YOUR_WIFI", "YOUR_PASS");
  while (WiFi.status() != WL_CONNECTED) delay(200);
#endif

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/cmd", HTTP_POST, handleCmd);

  server.collectHeaders(
    new const char* [1] {
      "Authorization"
    },
    1);
  server.begin();

  xTaskCreatePinnedToCore(radarTask, "radarTask", 8192, nullptr, 2, &radarTaskHandle, 1);

  Serial.print("Web UI ready at: http://");
#if USE_SOFTAP
  Serial.println(WiFi.softAPIP());
#else
  Serial.println(WiFi.localIP());
#endif
  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
}

void loop() {
  server.handleClient();
  delay(2);
}
