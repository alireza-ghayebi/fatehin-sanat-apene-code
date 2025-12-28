/*******************************************************
   ESP32-S3 Web UI (on-device) + X4M03 Control  (RF + BB)
   - UI like desktop app (buttons + status + settings)
   - Endpoints:
     GET  /              -> Web UI
     GET  /api/status    -> JSON status (requires Authorization header)
     POST /api/cmd       -> command (Connect/Disconnect/RunRF/RunBB/Stop/Reboot/Record)
     GET  /api/settings  -> get current settings
     POST /api/settings  -> apply new settings

   Radar:
   - UART to X4M03
   - Manual mode + RF/BB control
   - STOP button => Stop + reconnect (Disconnect+Connect)

   Notes:
   - Updates meta (FrameCounter, NFloats, etc.) from A0/12 packets.
   - FramesStored counts "frames" by detecting changes in frameCounter (not per packet).
 *******************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include "esp_timer.h"

struct RadarSettings;
static bool apply_settings(const RadarSettings& s);
/* ================= WiFi (SoftAP) ================= */
#define USE_SOFTAP 1
const char* AP_SSID = "ESP-Radar";
const char* AP_PASS = "radar1234";

/* ================= Web Server ================= */
WebServer server(80);

/* ================= Simple Auth (optional) =================
   UI sends header: Authorization: <API_TOKEN>
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
static volatile bool LastApplyOk = true;

/* ================= RF Snapshot for UI ================= */
static const uint16_t RF_UI_MAX = 256;
static float gRfData[RF_UI_MAX];
static volatile uint16_t gRfCount = 0;
static volatile uint32_t gRfFrame = 0;
static portMUX_TYPE gRfMux = portMUX_INITIALIZER_UNLOCKED;

static inline void clearRfSnapshot() {
  portENTER_CRITICAL(&gRfMux);
  gRfCount = 0;
  gRfFrame = 0;
  portEXIT_CRITICAL(&gRfMux);
}

/* ================= BB Snapshot for UI ================= */
static const uint16_t BB_UI_MAX = 1024;
static float gBbData[BB_UI_MAX];
static volatile uint16_t gBbCount = 0;
static volatile uint32_t gBbFrame = 0;
static portMUX_TYPE gBbMux = portMUX_INITIALIZER_UNLOCKED;

static inline void clearBbSnapshot() {
  portENTER_CRITICAL(&gBbMux);
  gBbCount = 0;
  gBbFrame = 0;
  portEXIT_CRITICAL(&gBbMux);
}

/* ================= RF Raw Snapshot for UI ================= */
static const uint16_t RF_RAW_MAX = 512;
static float gRfRawData[RF_RAW_MAX];
static volatile uint16_t gRfRawCount = 0;
static volatile uint32_t gRfRawFrame = 0;
static portMUX_TYPE gRfRawMux = portMUX_INITIALIZER_UNLOCKED;

static inline void clearRfRawSnapshot() {
  portENTER_CRITICAL(&gRfRawMux);
  gRfRawCount = 0;
  gRfRawFrame = 0;
  portEXIT_CRITICAL(&gRfRawMux);
}

/* ================= Settings ================= */
struct RadarSettings {
  float fps = 5.0f;
  uint8_t downconv = 0;  // 0=RF, 1=BB
  float fa_start = 0.30f;
  float fa_end = 1.00f;
  uint32_t pulses = 3;
  uint32_t iterations = 16;
  uint32_t dac_min = 900;
  uint32_t dac_max = 1150;
  uint32_t dac_step = 1;
  uint32_t prf_div = 3;
  uint8_t enable = 1;
};
static RadarSettings gSet;

/* ================= Helpers ================= */
static inline bool waitAvail(Stream& s, uint32_t to_ms) {
  uint32_t t0 = millis();
  while (!s.available() && (millis() - t0) < to_ms) delay(1);
  return s.available();
}
static inline uint32_t u32le(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* --- tiny JSON parse helpers (no ArduinoJson needed) --- */
static bool jsonGetString(const String& body, const char* key, String& out) {
  String k = String("\"") + key + "\"";
  int i = body.indexOf(k);
  if (i < 0) return false;
  int c = body.indexOf(':', i);
  if (c < 0) return false;
  int q1 = body.indexOf('"', c);
  if (q1 < 0) return false;
  int q2 = body.indexOf('"', q1 + 1);
  if (q2 < 0) return false;
  out = body.substring(q1 + 1, q2);
  return true;
}

static bool jsonGetNumber(const String& body, const char* key, double& out) {
  String k = String("\"") + key + "\"";
  int i = body.indexOf(k);
  if (i < 0) return false;
  int c = body.indexOf(':', i);
  if (c < 0) return false;

  int s = c + 1;
  while (s < (int)body.length() && (body[s] == ' ')) s++;

  int e = s;
  while (e < (int)body.length()) {
    char ch = body[e];
    if ((ch >= '0' && ch <= '9') || ch == '.' || ch == '-' || ch == '+') e++;
    else break;
  }
  if (e <= s) return false;

  out = body.substring(s, e).toDouble();
  return true;
}

/* ================= MCP writer ================= */
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

/* ================= ACK/READY ================= */
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

/* ================= Apply Settings ================= */
static bool apply_settings(const RadarSettings& s) {
  if (!RadarConnected) {
    LastApplyOk = false;
    return false;
  }

  bool ok = true;

  // stop -> manual
  sendSet_f32(XTS_SPCXI_FPS, 0.0f);
  ok &= expectAck();
  sendSetMode(XTS_SM_STOP);
  ok &= expectAck();
  sendSetMode(XTS_SM_MANUAL);
  ok &= expectAck();

  sendSet_u8(XTS_SPCXI_ENABLE, s.enable);
  ok &= expectAck();
  sendSet_u32(XTS_SPCXI_PULSES_PER_STEP, s.pulses);
  ok &= expectAck();
  sendSet_u32(XTS_SPCXI_ITERATIONS, s.iterations);
  ok &= expectAck();
  sendSet_u8(XTS_SPCXI_DOWNCONVERSION, s.downconv);
  ok &= expectAck();

  sendSet_u32(XTS_SPCXI_DAC_MIN, s.dac_min);
  ok &= expectAck();
  sendSet_u32(XTS_SPCXI_DAC_MAX, s.dac_max);
  ok &= expectAck();
  sendSet_u32(XTS_SPCXI_DAC_STEP, s.dac_step);
  ok &= expectAck();

  // اگر یک روز دیدی APPLY fail می‌شه، اول همین خط PRF_DIV رو موقتاً کامنت کن
  sendSet_u32(XTS_SPCXI_PRF_DIV, s.prf_div);
  ok &= expectAck();

  sendSet_f32(XTS_SPCXI_FRAME_AREA_OFFSET, 0.0f);
  ok &= expectAck();
  sendSet_frame_area(s.fa_start, s.fa_end);
  ok &= expectAck();

  // start streaming
  sendSet_f32(XTS_SPCXI_FPS, 0.0f);
  ok &= expectAck();
  sendSet_f32(XTS_SPCXI_FPS, s.fps);
  ok &= expectAck();

  LastApplyOk = ok;
  if (ok) RunStatus = (s.downconv == 1) ? 1 : 2;
  return ok;
}

/* ================= NoEscape packet reader =================
   Format:
     7C 7C 7C 7C
     len(u32le)  -> payload length
     00          -> reserved
     payload[len]
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
  clearRfSnapshot();
  clearRfRawSnapshot();
  RunStatus = 0;
  gSet.downconv = 0;
  bool ok = apply_settings(gSet);
  if (!ok) RunStatus = 2;
}

static void radarRunBB() {
  if (!RadarConnected) radarConnect();
  FramesStored = 0;
  FrameCounter = 0;
  clearRfSnapshot();
  clearBbSnapshot();
  RunStatus = 0;
  gSet.downconv = 1;
  bool ok = apply_settings(gSet);
  if (!ok) RunStatus = 1;
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
  static float rfRawLocal[RF_RAW_MAX];
  static uint32_t lastFrame = 0xFFFFFFFF;
  static uint32_t packetCount = 0;
  static uint32_t packetsInMinute = 0;
  static uint64_t totalBytes = 0;
  static uint64_t minuteBytes = 0;

  while (true) {
    if ((RunStatus == 1 || RunStatus == 2) && RadarConnected) {
      uint32_t pkt_len = 0;
      bool ok = readNoEscapePacket(payload, sizeof(payload), pkt_len);

      if (ok && pkt_len > 0) {
        packetCount++;
        packetsInMinute++;
        totalBytes += pkt_len;
        minuteBytes += pkt_len;

        uint32_t packetsPerMinute = (uint32_t)(gSet.fps * 60.0f + 0.5f);
        if (packetsPerMinute == 0) packetsPerMinute = 60;

        bool reportEveryTen = (packetCount % 10u) == 0u;
        bool reportMinute = packetsInMinute >= packetsPerMinute;

        if (reportEveryTen || reportMinute) {
          Serial.print("Packets: ");
          Serial.print(packetCount);
          Serial.print(" | Total bytes: ");
          Serial.print((unsigned long long)totalBytes);
          if (reportMinute) {
            Serial.print(" | Minute bytes: ");
            Serial.print((unsigned long long)minuteBytes);
            packetsInMinute = 0;
            minuteBytes = 0;
          }
          Serial.println();
        }

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
          const uint8_t* dataStart = p;

          uint32_t bytes_rem = (uint32_t)((payload + pkt_len) - p);
          uint32_t data_bytes = 0;
          if (bytes_rem >= flen * 4) data_bytes = flen * 4;
          else if ((flen % 4) == 0 && bytes_rem >= flen) data_bytes = flen;
          else data_bytes = bytes_rem - (bytes_rem % 4);

          uint32_t totalFloats = data_bytes / 4;
          uint32_t nSamples = totalFloats;

          if (RunStatus == 2 && totalFloats > 0) {
            uint32_t step = (totalFloats > RF_RAW_MAX) ? ((totalFloats + RF_RAW_MAX - 1) / RF_RAW_MAX) : 1;
            uint16_t outCount = 0;
            for (uint32_t i = 0; i < totalFloats && outCount < RF_RAW_MAX; i += step) {
              float v;
              memcpy(&v, dataStart + i * 4, sizeof(float));
              rfRawLocal[outCount++] = v;
            }
            portENTER_CRITICAL(&gRfRawMux);
            memcpy(gRfRawData, rfRawLocal, outCount * sizeof(float));
            gRfRawCount = outCount;
            gRfRawFrame = frameCounter;
            portEXIT_CRITICAL(&gRfRawMux);
          }

          if (RunStatus == 1 && totalFloats > 0) {
            uint16_t bbCount = (totalFloats > BB_UI_MAX) ? BB_UI_MAX : (uint16_t)totalFloats;
            portENTER_CRITICAL(&gBbMux);
            memcpy(gBbData, dataStart, bbCount * sizeof(float));
            gBbCount = bbCount;
            gBbFrame = frameCounter;
            portEXIT_CRITICAL(&gBbMux);
          }

          // BB: I/Q pairs => samples
          if (RunStatus == 1) nSamples = totalFloats / 2;

          if ((RunStatus == 2 || RunStatus == 1) && nSamples > 0) {
            const uint32_t binCount = RF_UI_MAX / 2;
            uint32_t step = (nSamples > binCount) ? ((nSamples + binCount - 1) / binCount) : 1;
            float local[RF_UI_MAX];
            uint16_t outCount = 0;
            for (uint32_t i = 0; i < nSamples && (outCount + 1) < RF_UI_MAX; i += step) {
              float vMin = 0.0f;
              float vMax = 0.0f;
              bool seeded = false;
              uint32_t end = i + step;
              if (end > nSamples) end = nSamples;
              for (uint32_t j = i; j < end; j++) {
                float v;
                if (RunStatus == 1) {
                  uint32_t iIdx = j * 2;
                  uint32_t qIdx = iIdx + 1;
                  if (qIdx >= totalFloats) break;
                  float iVal;
                  float qVal;
                  memcpy(&iVal, dataStart + iIdx * 4, sizeof(float));
                  memcpy(&qVal, dataStart + qIdx * 4, sizeof(float));
                  float mag = sqrtf(iVal * iVal + qVal * qVal);
                  v = log10f(mag + 1.0e-6f);
                } else {
                  if (j >= totalFloats) break;
                  memcpy(&v, dataStart + j * 4, sizeof(float));
                }
                if (!seeded) {
                  vMin = vMax = v;
                  seeded = true;
                } else {
                  if (v < vMin) vMin = v;
                  if (v > vMax) vMax = v;
                }
              }
              if (!seeded) break;
              local[outCount++] = vMin;
              local[outCount++] = vMax;
            }
            portENTER_CRITICAL(&gRfMux);
            memcpy(gRfData, local, outCount * sizeof(float));
            gRfCount = outCount;
            gRfFrame = frameCounter;
            portEXIT_CRITICAL(&gRfMux);
          }

          FrameCounter = frameCounter;
          LastPktLen = pkt_len;
          LastBytesRem = bytes_rem;
          LastNFloats = nSamples;

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
    :root{
      --bg:#0b0f14;
      --panel:#111826;
      --panel-2:#0d1522;
      --border:#1f2a3a;
      --muted:#94a3b8;
      --text:#e8eef7;
      --accent:#38bdf8;
      --accent-2:#22c55e;
      --danger:#ef4444;
      --shadow:0 12px 30px rgba(2,6,23,0.45);
    }
    *{box-sizing:border-box;}
    body{
      font-family:"Vazirmatn","IRANSansX","Shabnam","Space Grotesk","Segoe UI",Tahoma,sans-serif;
      margin:0;
      color:var(--text);
      background:
        radial-gradient(900px 480px at 85% -10%, rgba(56,189,248,0.12), transparent 60%),
        radial-gradient(700px 400px at 10% 10%, rgba(34,197,94,0.10), transparent 55%),
        var(--bg);
      min-height:100vh;
      letter-spacing:0.2px;
    }
    .wrap{max-width:1180px; margin:auto; padding:20px 18px 32px;}
    .top{display:flex; gap:14px; flex-wrap:wrap; align-items:center; justify-content:space-between;}
    .card{
      background:linear-gradient(180deg, rgba(17,24,38,0.98), rgba(12,18,30,0.98));
      border:1px solid var(--border);
      border-radius:18px;
      padding:16px;
      box-shadow:var(--shadow);
    }
    .grid{display:grid; grid-template-columns: 380px 1fr; gap:16px; margin-top:16px;}
    @media (max-width:900px){ .grid{grid-template-columns:1fr;} .wrap{padding:16px;} }
    button{
      border:1px solid rgba(148,163,184,0.2);
      padding:10px 14px;
      border-radius:12px;
      cursor:pointer;
      font-weight:700;
      color:var(--text);
      background:rgba(31,42,58,0.9);
      transition:transform .15s ease, box-shadow .15s ease, filter .15s ease;
    }
    button:active{transform:translateY(1px) scale(0.99);}
    button:focus-visible{outline:2px solid rgba(56,189,248,0.6); outline-offset:2px;}
    .btn{
      background:linear-gradient(180deg, #1f2a3a, #17202f);
      box-shadow:inset 0 1px 0 rgba(255,255,255,0.04);
    }
    .btn:hover{filter:brightness(1.12);}
    .btn.primary{background:linear-gradient(180deg, #2e6cf2, #1e4bd1);}
    .btn.danger{background:linear-gradient(180deg, #ef4444, #b91c1c);}
    .btn.good{background:linear-gradient(180deg, #22c55e, #15803d);}
    .row{display:flex; gap:10px; flex-wrap:wrap;}
    .kv{
      display:grid;
      grid-template-columns: 1fr 1fr;
      gap:8px;
      margin-top:10px;
      background:rgba(15,23,42,0.35);
      padding:10px;
      border-radius:12px;
      border:1px solid rgba(148,163,184,0.12);
    }
    .k{opacity:.7;}
    .v{font-family:"JetBrains Mono","Fira Code","SFMono-Regular",Menlo,monospace;}
    .hdr{font-size:14px; opacity:.8}
    input, select{
      background:#0b111b;
      color:var(--text);
      border:1px solid rgba(148,163,184,0.18);
      border-radius:12px;
      padding:10px 12px;
      box-shadow:inset 0 1px 0 rgba(255,255,255,0.03);
    }
    input:focus, select:focus{
      outline:2px solid rgba(56,189,248,0.35);
      outline-offset:1px;
      border-color:rgba(56,189,248,0.6);
    }
    .note{opacity:.75; font-size:13px; margin-top:10px; line-height:1.7; color:var(--muted);}
    .big{
      min-height:340px;
      display:flex;
      flex-direction:column;
      align-items:stretch;
      justify-content:flex-start;
      gap:12px;
      border:1px dashed rgba(148,163,184,0.35);
      border-radius:16px;
      color:var(--muted);
      text-align:right;
      padding:18px;
      background:linear-gradient(180deg, rgba(15,23,42,0.55), rgba(10,16,26,0.55));
    }
    .bb-chart-header{
      display:flex;
      align-items:center;
      justify-content:space-between;
      gap:12px;
      flex-wrap:wrap;
    }
    .bb-chart-title{font-weight:800;}
    .bb-chart-meta{
      display:flex;
      gap:12px;
      flex-wrap:wrap;
      font-size:12px;
      color:var(--muted);
    }
    .bb-chart-body{width:100%;}
    #bbChart{width:100%; height:280px; display:block;}
    .sep{height:1px; background:rgba(148,163,184,0.12); margin:14px 0;}
    @keyframes rise{from{opacity:0; transform:translateY(10px);} to{opacity:1; transform:translateY(0);}}
    .card{animation:rise .5s ease both;}
    .grid .card:nth-child(2){animation-delay:.06s;}
    .top .card{animation-delay:.02s;}
    @media (prefers-reduced-motion: reduce){
      .card{animation:none;}
      button{transition:none;}
    }
  </style>
</head>
<body>
<div class="wrap">
  <div class="top">
    <div>
      <div style="font-size:18px;font-weight:800">کنترل رادار روی ESP</div>
      <div class="hdr">Run/Stop/Reboot + تنظیمات رادار</div>
    </div>
    <div class="card">
      <div class="row" style="align-items:center">
        <span class="k">Token:</span>
        <input id="token" placeholder="Authorization Token" style="width:260px"/>
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
        <button class="btn danger" onclick="cmd('Reboot')">Reboot Radar</button>
      </div>

      <div class="note">
        Stop شامل Disconnect + Connect است تا بعد ریست/قطع، باود و هندشیک دوباره درست شود.
      </div>

      <div class="sep"></div>

      <div style="font-weight:800;margin-bottom:8px">تنظیمات رادار</div>

      <div class="row" style="align-items:center">
        <span class="k">Mode</span>
        <select id="mode">
          <option value="RF">RF</option>
          <option value="BB">BB</option>
        </select>

        <span class="k">FPS</span>
        <input id="fps" type="number" step="0.1" value="5" style="width:110px"/>
      </div>

      <div class="row" style="align-items:center;margin-top:10px">
        <span class="k">FrameArea</span>
        <input id="fa_start" type="number" step="0.01" value="0.30" style="width:110px"/>
        <input id="fa_end" type="number" step="0.01" value="1.00" style="width:110px"/>
      </div>

      <div class="row" style="align-items:center;margin-top:10px">
        <span class="k">Pulses</span><input id="pulses" type="number" step="1" value="3" style="width:90px"/>
        <span class="k">Iter</span><input id="iterations" type="number" step="1" value="16" style="width:90px"/>
      </div>

      <div class="row" style="align-items:center;margin-top:10px">
        <span class="k">DAC</span>
        <input id="dac_min" type="number" step="1" value="900" style="width:90px"/>
        <input id="dac_max" type="number" step="1" value="1150" style="width:90px"/>
        <input id="dac_step" type="number" step="1" value="1" style="width:70px"/>
      </div>

      <div class="row" style="align-items:center;margin-top:10px">
        <span class="k">PRF_DIV</span>
        <input id="prf_div" type="number" step="1" value="3" style="width:90px"/>
        <button class="btn good" onclick="applySettings()">Apply</button>
        <button class="btn" onclick="loadSettings()">Load</button>
      </div>

      <div class="sep"></div>

      <div style="font-weight:800;margin-top:2px;margin-bottom:6px">وضعیت</div>
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
      <div class="big" id="bbChartPanel">
        <div class="bb-chart-header">
          <div class="bb-chart-title">Baseband (BB) Plot</div>
          <div class="bb-chart-meta">
            <span>Bytes in buffer: <b id="bb-bytes">-</b></span>
            <span>Frame Counter: <b id="bb-frame">-</b></span>
            <span>Frames Stored: <b id="bb-stored">-</b></span>
            <span>Len of Data: <b id="bb-len">-</b></span>
          </div>
        </div>
        <div class="bb-chart-body">
          <canvas id="bbChart"></canvas>
        </div>
      </div>
    </div>
  </div>
</div>

<script>
const tokenEl = document.getElementById("token");
function saveToken(){
  if (!tokenEl) return;
  localStorage.setItem("api_token", tokenEl.value || "");
}
function getToken(){
  if (tokenEl) {
    const v = tokenEl.value.trim();
    if (v) return v;
  }
  return localStorage.getItem("api_token") || "";
}
if (tokenEl) tokenEl.value = localStorage.getItem("api_token") || "";

const bbCanvas = document.getElementById("bbChart");
const bbCtx = bbCanvas ? bbCanvas.getContext("2d") : null;
const bbBytesEl = document.getElementById("bb-bytes");
const bbFrameEl = document.getElementById("bb-frame");
const bbStoredEl = document.getElementById("bb-stored");
const bbLenEl = document.getElementById("bb-len");
const bbTitleEl = document.querySelector(".bb-chart-title");
let bbLastFrame = -1;
let rfLastFrame = -1;
let bbTimer = null;
let plotMode = 0; // 0=idle, 1=BB, 2=RF

function updateBbMetaFromStatus(j){
  if (bbBytesEl) bbBytesEl.textContent = j.bytes_in_buf ?? "-";
  if (bbFrameEl) bbFrameEl.textContent = j.frame_counter ?? "-";
  if (bbStoredEl) bbStoredEl.textContent = j.frames_stored ?? "-";
}

function resizeBbChart(){
  if(!bbCanvas || !bbCtx) return;
  const rect = bbCanvas.getBoundingClientRect();
  if(!rect.width || !rect.height) return;
  const dpr = window.devicePixelRatio || 1;
  bbCanvas.width = Math.floor(rect.width * dpr);
  bbCanvas.height = Math.floor(rect.height * dpr);
  bbCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function drawBbChart(data){
  if(!bbCanvas || !bbCtx || !data || !data.length) return;
  const w = bbCanvas.clientWidth;
  const h = bbCanvas.clientHeight;
  if(!w || !h) return;

  let min = data[0];
  let max = data[0];
  for(let i = 1; i < data.length; i++){
    const v = data[i];
    if(v < min) min = v;
    if(v > max) max = v;
  }
  if(min === max){
    min -= 1;
    max += 1;
  }

  const pad = 10;
  const innerW = w - pad * 2;
  const innerH = h - pad * 2;

  // Matplotlib-like clear and draw
  bbCtx.clearRect(0, 0, w, h);

  // Optional grid
  bbCtx.strokeStyle = "rgba(148,163,184,0.12)";
  bbCtx.lineWidth = 1;
  bbCtx.beginPath();
  for(let i = 1; i <= 4; i++){
    const y = pad + (innerH * i / 4);
    bbCtx.moveTo(pad, y);
    bbCtx.lineTo(w - pad, y);
  }
  for(let i = 1; i <= 6; i++){
    const x = pad + (innerW * i / 6);
    bbCtx.moveTo(x, pad);
    bbCtx.lineTo(x, h - pad);
  }
  bbCtx.stroke();

  // Line chart: X = sample index, Y = value
  const span = Math.max(1, data.length - 1);
  const xStep = innerW / span;
  bbCtx.strokeStyle = "#38bdf8";
  bbCtx.lineWidth = 2;
  bbCtx.beginPath();
  for(let i = 0; i < data.length; i++){
    const v = data[i];
    const x = pad + xStep * i;
    const y = pad + (1 - (v - min) / (max - min)) * innerH;
    if(i === 0) bbCtx.moveTo(x, y);
    else bbCtx.lineTo(x, y);
  }
  bbCtx.stroke();
}

async function fetchPlotData(){
  if(!bbCtx) return;
  if(plotMode !== 1 && plotMode !== 2) return;
  const t = getToken();
  const url = (plotMode === 2) ? "/api/rf_raw" : "/api/bb";
  const r = await fetch(url, {headers:{"Authorization": t}});
  if(!r.ok) return;
  const j = await r.json();
  if(!j || !j.data || !j.data.length) return;

  const lastFrame = (plotMode === 2) ? rfLastFrame : bbLastFrame;
  if(j.frame === lastFrame) return;
  if(plotMode === 2) rfLastFrame = j.frame;
  else bbLastFrame = j.frame;

  if (bbFrameEl) bbFrameEl.textContent = j.frame;
  if (bbLenEl) bbLenEl.textContent = j.count ?? j.data.length;
  drawBbChart(j.data);
}

function startBbChart(){
  if(!bbCtx) return;
  resizeBbChart();
  window.addEventListener("resize", resizeBbChart);
  fetchPlotData();
  if(!bbTimer) bbTimer = setInterval(fetchPlotData, 250);
}

startBbChart();

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
  const newMode = Number(j.run_status);
  if (newMode !== plotMode) {
    plotMode = newMode;
    if (plotMode === 1) bbLastFrame = -1;
    else if (plotMode === 2) rfLastFrame = -1;
  }
  if (bbTitleEl) {
    if (plotMode === 2) bbTitleEl.textContent = "RF Plot";
    else if (plotMode === 1) bbTitleEl.textContent = "Baseband (BB) Plot";
    else bbTitleEl.textContent = "Radar Plot";
  }
  updateBbMetaFromStatus(j);
}

async function loadSettings(){
  const t = getToken();
  const r = await fetch("/api/settings", {headers:{"Authorization": t}});
  if(!r.ok) return;
  const s = await r.json();
  document.getElementById("mode").value = s.mode || "RF";
  document.getElementById("fps").value = s.fps ?? 5;
  document.getElementById("fa_start").value = s.fa_start ?? 0.30;
  document.getElementById("fa_end").value = s.fa_end ?? 1.00;
  document.getElementById("pulses").value = s.pulses ?? 3;
  document.getElementById("iterations").value = s.iterations ?? 16;
  document.getElementById("dac_min").value = s.dac_min ?? 900;
  document.getElementById("dac_max").value = s.dac_max ?? 1150;
  document.getElementById("dac_step").value = s.dac_step ?? 1;
  document.getElementById("prf_div").value = s.prf_div ?? 3;
}

async function applySettings(){
  const t = getToken();
  const payload = {
    mode: document.getElementById("mode").value,
    fps: parseFloat(document.getElementById("fps").value),
    fa_start: parseFloat(document.getElementById("fa_start").value),
    fa_end: parseFloat(document.getElementById("fa_end").value),
    pulses: parseInt(document.getElementById("pulses").value),
    iterations: parseInt(document.getElementById("iterations").value),
    dac_min: parseInt(document.getElementById("dac_min").value),
    dac_max: parseInt(document.getElementById("dac_max").value),
    dac_step: parseInt(document.getElementById("dac_step").value),
    prf_div: parseInt(document.getElementById("prf_div").value),
  };

  const r = await fetch("/api/settings",{
    method:"POST",
    headers:{
      "Content-Type":"application/json",
      "Authorization": t
    },
    body: JSON.stringify(payload)
  });

  if(!r.ok){
    alert("Apply failed: HTTP " + r.status);
    return;
  }
  await refresh();
}

setInterval(refresh, 600);
refresh();
loadSettings();
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
  json += "\"last_apply_ok\":" + String(LastApplyOk ? "true" : "false") + ",";
  json += "\"uptime_ms\":" + String(uptime);
  json += "}";
  server.send(200, "application/json", json);
}

void handleRfData() {
  if (!isAuthorized()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  float local[RF_UI_MAX];
  uint16_t count = 0;
  uint32_t frame = 0;

  portENTER_CRITICAL(&gRfMux);
  count = gRfCount;
  frame = gRfFrame;
  if (count > 0) memcpy(local, gRfData, count * sizeof(float));
  portEXIT_CRITICAL(&gRfMux);

  String json;
  json.reserve(64 + count * 8);
  json += "{";
  json += "\"frame\":" + String(frame) + ",";
  json += "\"count\":" + String(count) + ",";
  json += "\"data\":[";
  for (uint16_t i = 0; i < count; i++) {
    if (i) json += ",";
    json += String(local[i], 4);
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleRfRawData() {
  if (!isAuthorized()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  float local[RF_RAW_MAX];
  uint16_t count = 0;
  uint32_t frame = 0;

  portENTER_CRITICAL(&gRfRawMux);
  count = gRfRawCount;
  frame = gRfRawFrame;
  if (count > 0) memcpy(local, gRfRawData, count * sizeof(float));
  portEXIT_CRITICAL(&gRfRawMux);

  String json;
  json.reserve(64 + count * 8);
  json += "{";
  json += "\"frame\":" + String(frame) + ",";
  json += "\"count\":" + String(count) + ",";
  json += "\"data\":[";
  for (uint16_t i = 0; i < count; i++) {
    if (i) json += ",";
    json += String(local[i], 4);
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleBbData() {
  if (!isAuthorized()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  float local[BB_UI_MAX];
  uint16_t count = 0;
  uint32_t frame = 0;

  portENTER_CRITICAL(&gBbMux);
  count = gBbCount;
  frame = gBbFrame;
  if (count > 0) memcpy(local, gBbData, count * sizeof(float));
  portEXIT_CRITICAL(&gBbMux);

  String json;
  json.reserve(64 + count * 8);
  json += "{";
  json += "\"frame\":" + String(frame) + ",";
  json += "\"count\":" + String(count) + ",";
  json += "\"data\":[";
  for (uint16_t i = 0; i < count; i++) {
    if (i) json += ",";
    json += String(local[i], 4);
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleGetSettings() {
  if (!isAuthorized()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  String json = "{";
  json += "\"fps\":" + String(gSet.fps, 2) + ",";
  json += "\"mode\":\"" + String(gSet.downconv ? "BB" : "RF") + "\",";
  json += "\"fa_start\":" + String(gSet.fa_start, 2) + ",";
  json += "\"fa_end\":" + String(gSet.fa_end, 2) + ",";
  json += "\"pulses\":" + String(gSet.pulses) + ",";
  json += "\"iterations\":" + String(gSet.iterations) + ",";
  json += "\"dac_min\":" + String(gSet.dac_min) + ",";
  json += "\"dac_max\":" + String(gSet.dac_max) + ",";
  json += "\"dac_step\":" + String(gSet.dac_step) + ",";
  json += "\"prf_div\":" + String(gSet.prf_div);
  json += "}";
  server.send(200, "application/json", json);
}

void handlePostSettings() {
  if (!isAuthorized()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }

  String body = server.arg("plain");

  RadarSettings s = gSet;

  // mode
  String mode;
  if (jsonGetString(body, "mode", mode)) {
    mode.toUpperCase();
    s.downconv = (mode == "BB") ? 1 : 0;
  }

  // numbers
  double v;
  if (jsonGetNumber(body, "fps", v)) s.fps = (float)v;
  if (jsonGetNumber(body, "fa_start", v)) s.fa_start = (float)v;
  if (jsonGetNumber(body, "fa_end", v)) s.fa_end = (float)v;
  if (jsonGetNumber(body, "pulses", v)) s.pulses = (uint32_t)v;
  if (jsonGetNumber(body, "iterations", v)) s.iterations = (uint32_t)v;
  if (jsonGetNumber(body, "dac_min", v)) s.dac_min = (uint32_t)v;
  if (jsonGetNumber(body, "dac_max", v)) s.dac_max = (uint32_t)v;
  if (jsonGetNumber(body, "dac_step", v)) s.dac_step = (uint32_t)v;
  if (jsonGetNumber(body, "prf_div", v)) s.prf_div = (uint32_t)v;

  // very light sanity
  if (s.fa_start < 0.0f) s.fa_start = 0.0f;
  if (s.fa_end <= s.fa_start) s.fa_end = s.fa_start + 0.1f;
  if (s.fps < 0.0f) s.fps = 0.0f;
  if (s.fps > 50.0f) s.fps = 50.0f;

  bool ok = apply_settings(s);
  if (!ok) {
    server.send(500, "text/plain", "Apply failed");
    return;
  }

  gSet = s;
  server.send(200, "application/json", "{\"ok\":true}");
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
  else if (cmd == "Stop") radarStopAndReboot();  // ✅ Stop + Auto Reboot (Disconnect+Connect)
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
  server.on("/api/rf", HTTP_GET, handleRfData);
  server.on("/api/rf_raw", HTTP_GET, handleRfRawData);
  server.on("/api/bb", HTTP_GET, handleBbData);
  server.on("/api/cmd", HTTP_POST, handleCmd);

  server.on("/api/settings", HTTP_GET, handleGetSettings);
  server.on("/api/settings", HTTP_POST, handlePostSettings);

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
