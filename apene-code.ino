/*******************************************************
 *  ESP32-S3  <->  XeThru X4M03  (Manual Mode, Baseband)
 *  - خواندن استریم Generic Float (A0/12)
 *  - تجمیع I/Q برای هر فریم و استخراج |mag| روی یک bin انتخابی
 *  - بازخورد بصری با NeoPixel (RGB روی GPIO48)
 *  - بهینه‌سازی‌های ارتباطی: تغییر Baud به 921600
 *
 *  نکته: در حالت Manual، X4M03 داده‌ی Baseband را اغلب به‌صورت
 *  "Generic Float A0/12" می‌فرستد (نه AppData=0x50).
 *******************************************************/

#include <Arduino.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

/* ===================== پین‌ها ===================== */
#define RADAR_UART_NUM 1   // پورت UART1 روی ESP32-S3
#define RADAR_RX_PIN 15    // RX1 ← خروجی TX رادار
#define RADAR_TX_PIN 16    // TX1 → ورودی RX رادار
#define RADAR_RESET_PIN 5  // پین ریست ماژول رادار (nRESET)

#ifndef LED_PIN
#define LED_PIN 48  // NeoPixel آدرس‌پذیر روی GPIO48
#endif

/* ================== NeoPixel (RGB) ================= */
Adafruit_NeoPixel pixels(1, LED_PIN, NEO_GRB + NEO_KHZ800);

/* مقدار LED را مقداردهی می‌کند */
static inline void led_init() {
  pixels.begin();
  pixels.clear();
  pixels.show();
}

/* شدت LED را بر اساس a01 (بین 0..1) تنظیم می‌کند */
static inline void led_set(float a01) {
  if (a01 < 0) a01 = 0;
  if (a01 > 1) a01 = 1;

  const uint8_t maxLevel = 160;               // حداکثر شدت (ملایم)
  uint8_t level = (uint8_t)(a01 * maxLevel);  // نگاشت 0..1 → 0..160

  // رنگ آبی با کمی سبز برای لطافت
  pixels.setPixelColor(0, pixels.Color(
                            0,           // R
                            level >> 2,  // G (¼ آبی)
                            level        // B
                            ));
  pixels.show();
}

/* ============== ثوابت پروتکل MCP (XeThru) ============== */
const uint8_t XT_START = 0x7D, XT_STOP = 0x7E, XT_ESC = 0x7F;

const uint8_t XTS_SPC_MOD_SETMODE = 0x20;       // ست‌مود
const uint8_t XTS_SPC_X4DRIVER = 0x50;          // فضای دستورات X4
const uint8_t XTS_SPCX_SET = 0x10;              // SET
const uint8_t XTS_SPR_SYSTEM = 0x30;            // پیام‌های سیستمی (READY/BOOTING)
const uint8_t XTS_SPR_ACK = 0x10;               // ACK فریم
const uint8_t XTS_SM_STOP = 0x13;               // حالت Stop
const uint8_t XTS_SM_MANUAL = 0x12;             // حالت Manual (پایین‌سطحی)
const uint8_t XTS_SPC_DIR_COMMAND = 0x90;       // دستورات مستقیم
const uint8_t XTS_SDC_COMM_SETBAUDRATE = 0x80;  // تغییر Baud

// شناسه‌های پارامتر SET برای X4
const uint32_t XTS_SPCXI_FPS = 0x00000010;
const uint32_t XTS_SPCXI_PULSES_PER_STEP = 0x00000011;
const uint32_t XTS_SPCXI_ITERATIONS = 0x00000012;
const uint32_t XTS_SPCXI_DOWNCONVERSION = 0x00000013;
const uint32_t XTS_SPCXI_FRAME_AREA = 0x00000014;  // f32 start, f32 end
const uint32_t XTS_SPCXI_DAC_STEP = 0x00000015;    // اندازه گام DAC
const uint32_t XTS_SPCXI_DAC_MIN = 0x00000016;
const uint32_t XTS_SPCXI_DAC_MAX = 0x00000017;
const uint32_t XTS_SPCXI_FRAME_AREA_OFFSET = 0x00000018;
const uint32_t XTS_SPCXI_ENABLE = 0x00000019;   // فعال‌سازی پین‌های داخلی
const uint32_t XTS_SPCXI_PRF_DIV = 0x00000020;  // تقسیم‌کننده PRF (در دسترس برخی FW)

const uint32_t XTS_SPRS_READY = 0x00000011;  // کد READY

/* ================== شیء UART برای رادار ================== */
HardwareSerial RadarSerial(RADAR_UART_NUM);

/* ================== ابزارهای عمومی ================== */

/* منتظر موجود بودن داده در Stream با تایم‌اوت میلی‌ثانیه */
static inline bool waitAvail(Stream& s, uint32_t to_ms) {
  uint32_t t = millis();
  while (!s.available() && (millis() - t) < to_ms) delay(1);
  return s.available();
}

/* ارسال فریم به‌صورت Escape’d (7D ... CRC 7E) */
static size_t radarWriteEscaped(const uint8_t* buf, size_t len) {
  // CRC = XOR از Start و تمام payload
  uint8_t crc = XT_START;
  for (size_t i = 0; i < len; i++) crc ^= buf[i];

  // کمک‌تابع نوشتن با Escaping
  auto w = [&](uint8_t b) {
    if (b == XT_START || b == XT_STOP || b == XT_ESC) RadarSerial.write(XT_ESC);
    RadarSerial.write(b);
  };

  // Start → Payload (با escaping) → CRC → Stop
  RadarSerial.write(XT_START);
  for (size_t i = 0; i < len; i++) w(buf[i]);
  w(crc);
  RadarSerial.write(XT_STOP);

  return len + 3;  // تخمین (Start+CRC+Stop)
}

/* ارسال SetMode(Manual/Stop/...) */
static inline void sendSetMode(uint8_t mode) {
  uint8_t p[2] = { XTS_SPC_MOD_SETMODE, mode };
  radarWriteEscaped(p, sizeof(p));
}

/* SET مقدار u8 برای شناسه id */
static inline void sendSet_u8(uint32_t id, uint8_t v) {
  uint8_t p[1 + 1 + 4 + 1] = { XTS_SPC_X4DRIVER, XTS_SPCX_SET };
  memcpy(&p[2], &id, 4);
  p[6] = v;
  radarWriteEscaped(p, sizeof(p));
}

/* SET مقدار u32 برای شناسه id */
static inline void sendSet_u32(uint32_t id, uint32_t v) {
  uint8_t p[1 + 1 + 4 + 4] = { XTS_SPC_X4DRIVER, XTS_SPCX_SET };
  memcpy(&p[2], &id, 4);
  memcpy(&p[6], &v, 4);
  radarWriteEscaped(p, sizeof(p));
}

/* SET مقدار f32 برای شناسه id */
static inline void sendSet_f32(uint32_t id, float v) {
  uint8_t p[1 + 1 + 4 + 4] = { XTS_SPC_X4DRIVER, XTS_SPCX_SET };
  memcpy(&p[2], &id, 4);
  memcpy(&p[6], &v, 4);
  radarWriteEscaped(p, sizeof(p));
}

/* SET محدوده فاصله (frame_area: start..end به متر) */
static inline void sendSet_frame_area(float s, float e) {
  uint8_t p[1 + 1 + 4 + 4 + 4] = { XTS_SPC_X4DRIVER, XTS_SPCX_SET };
  uint32_t id = XTS_SPCXI_FRAME_AREA;
  memcpy(&p[2], &id, 4);
  memcpy(&p[6], &s, 4);
  memcpy(&p[10], &e, 4);
  radarWriteEscaped(p, sizeof(p));
}

/* دستور تغییر Baud سمت رادار */
static inline void sendSetBaud(uint32_t baud) {
  // فرم: 0x90 0x80 + (baud u32 little-endian)
  uint8_t p[1 + 1 + 4] = { XTS_SPC_DIR_COMMAND, XTS_SDC_COMM_SETBAUDRATE };
  memcpy(&p[2], &baud, 4);
  radarWriteEscaped(p, sizeof(p));
}

/* ریست امن رادار (TX را High نگه می‌داریم تا وارد بوت‌لودر نشود) */
static inline void radarReset() {
  pinMode(RADAR_TX_PIN, OUTPUT);
  digitalWrite(RADAR_TX_PIN, HIGH);

  pinMode(RADAR_RESET_PIN, OUTPUT);
  digitalWrite(RADAR_RESET_PIN, LOW);
  delay(100);
  digitalWrite(RADAR_RESET_PIN, HIGH);
  delay(120);

  pinMode(RADAR_TX_PIN, INPUT);
}

/* انتظار برای دریافت ACK (فریم نوع 0x10) */
static inline bool expectAck(uint32_t to = 1500) {
  uint32_t t0 = millis();
  while (millis() - t0 < to) {
    if (!waitAvail(RadarSerial, 10)) continue;
    if (RadarSerial.read() != XT_START) continue;

    uint8_t type = 0, b;
    // خواندن تا پایان فریم
    while (waitAvail(RadarSerial, 50)) {
      b = RadarSerial.read();
      if (!type && b != XT_START) type = b;  // بلافاصله بعد از Start نوع می‌آید
      if (b == XT_STOP) break;
    }
    if (type == XTS_SPR_ACK) return true;
  }
  return false;
}

/* انتظار برای پیام READY (System 0x30 + code=0x00000011) */
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
      uint32_t code = ((uint32_t)b[1]) | ((uint32_t)b[2] << 8) | ((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 24);
      if (code == XTS_SPRS_READY) return true;
    }
  }
  return false;
}

/* =============== پیش‌تنظیم حالت Manual + Baseband =============== */
static bool apply_preset_manual_BB(float fps) {
  bool ok = true;

  // فعال‌سازی بلوک‌ها
  sendSet_u8(XTS_SPCXI_ENABLE, 1);
  ok &= expectAck();
  Serial.println(F("ACK pin_enable=1"));

  // کیفیت نمونه‌برداری (SNR vs سرعت)
  sendSet_u32(XTS_SPCXI_PULSES_PER_STEP, 3);
  ok &= expectAck();
  Serial.println(F("ACK pulses_per_step=3"));
  sendSet_u32(XTS_SPCXI_ITERATIONS, 16);
  ok &= expectAck();
  Serial.println(F("ACK iterations=16"));

  // Baseband (I/Q)
  sendSet_u8(XTS_SPCXI_DOWNCONVERSION, 1);
  ok &= expectAck();
  Serial.println(F("ACK downconversion=1 (BB)"));

  // پنجره‌ی نمونه‌برداری آنالوگ (بازه‌ی عمق)
  sendSet_u32(XTS_SPCXI_DAC_MIN, 900);
  ok &= expectAck();
  Serial.println(F("ACK dac_min=900"));
  sendSet_u32(XTS_SPCXI_DAC_MAX, 1150);
  ok &= expectAck();
  Serial.println(F("ACK dac_max=1150"));
  sendSet_u32(XTS_SPCXI_DAC_STEP, 1);
  ok &= expectAck();
  Serial.println(F("ACK dac_step=1"));

  // PRF (در برخی FW پشتیبانی می‌شود)
  sendSet_u32(XTS_SPCXI_PRF_DIV, 3);
  ok &= expectAck();
  Serial.println(F("ACK prf_div=3"));

  // نگاشت فاصله
  sendSet_f32(XTS_SPCXI_FRAME_AREA_OFFSET, 0.0f);
  ok &= expectAck();
  Serial.println(F("ACK frame_area_offset=0.0"));
  sendSet_frame_area(0.30f, 1.00f);
  ok &= expectAck();
  Serial.println(F("ACK frame_area=[0.30..1.00] m"));

  // ری‌استارت استریم با fps مطلوب
  sendSet_f32(XTS_SPCXI_FPS, 0.0f);
  ok &= expectAck();
  Serial.println(F("ACK fps=0"));
  sendSet_f32(XTS_SPCXI_FPS, fps);
  ok &= expectAck();
  Serial.println(F("ACK fps>0 (stream ON)"));

  return ok;
}

/* =============== بافر و تجمیع فریم‌های Generic (A0/12) =============== */
static const uint32_t MAX_FLOATS = 8192;   // حد امن برای I+Q
static float g_accum[MAX_FLOATS];          // بافر تجمیع فریم جاری
static uint32_t g_accumCount = 0;          // تعداد floatهای ذخیره‌شده
static uint32_t g_currFrame = 0xFFFFFFFF;  // شماره فریم جاری (برای شناسایی مرز فریم)

/* پس از کامل شدن داده‌ی یک فریم، I/Q را پردازش و گزارش کن */
static void process_full_generic_frame() {
  if (g_accumCount < 4) return;   // حداقل I/Q یک جفت
  uint32_t N = g_accumCount / 2;  // تعداد binها
  const float* I = g_accum;       // نیمه اول I
  const float* Q = g_accum + N;   // نیمه دوم Q

  // انتخاب یک bin مرکزی + همسایه‌ها برای میانگین‌گیری
  uint16_t centerBin = 64;  // قابل تغییر: بسته به فاصله‌ی هدف
  uint8_t avgw = 5;         // پهنای پنجره‌ی میانگین‌گیری (تعداد bin)
  if (centerBin >= N) centerBin = (N ? N - 1 : 0);

  uint16_t half = (avgw ? (avgw - 1) / 2 : 0);
  uint16_t b0 = (centerBin > half ? centerBin - half : 0);
  uint16_t b1 = min<uint32_t>(N, centerBin + half + 1);

  // |mag| = sqrt(I^2 + Q^2) سپس میانگین روی پنجره
  float sum = 0;
  uint16_t cnt = 0;
  for (uint16_t b = b0; b < b1; b++) {
    float mag = sqrtf(I[b] * I[b] + Q[b] * Q[b]);
    sum += mag;
    cnt++;
  }
  float val = (cnt ? sum / cnt : 0.0f);

  // نرمال‌سازی نمایی برای LED و گزارش
  static float ema = 0, emavar = 0;
  if (ema == 0) ema = val;
  ema = 0.98f * ema + 0.02f * val;
  float d = fabsf(val - ema);
  emavar = 0.95f * emavar + 0.05f * d;

  float a01 = (emavar > 1e-6f ? (val - ema) / (3.0f * emavar) + 0.5f : 0.5f);
  if (a01 < 0) a01 = 0;
  if (a01 > 1) a01 = 1;
  led_set(a01);  // شدت LED متناسب با فعالیت تنفسی

  // هر 10 فریم یک بار گزارش متنی بده
  static uint32_t printc = 0;
  if ((++printc) % 10 == 0) {
    Serial.printf("[BB(Generic,acc)] frame=%lu N=%lu selBin=%u |mag|=%.5f\n",
                  (unsigned long)g_currFrame, (unsigned long)N, (unsigned)centerBin, val);
  }
}

/* =============== خواندن یک پکت استریم از رادار =============== */
static bool readPacketAndReport() {
  /* 1) Sync روی هدر 7C 7C 7C 7C */
  uint8_t h[4] = { 0 };
  while (true) {
    if (!waitAvail(RadarSerial, 100)) return false;
    uint8_t c = RadarSerial.read();
    h[0] = h[1];
    h[1] = h[2];
    h[2] = h[3];
    h[3] = c;
    if (h[0] == 0x7C && h[1] == 0x7C && h[2] == 0x7C && h[3] == 0x7C) break;
  }

  /* 2) خواندن طول پکت (4 بایت) */
  while (RadarSerial.available() < 4) delay(1);
  uint8_t lb[4];
  RadarSerial.readBytes((char*)lb, 4);
  uint32_t pkt_len = (uint32_t)lb[0] | ((uint32_t)lb[1] << 8)
                     | ((uint32_t)lb[2] << 16) | ((uint32_t)lb[3] << 24);

  /* 3) خواندن payload */
  static uint8_t payload[16384];  // بافر امن
  if (pkt_len > sizeof(payload)) {
    // اگر طول غیرواقعی بود، پکت را تخلیه کن و برگرد
    for (uint32_t i = 0; i < pkt_len; i++) {
      while (!RadarSerial.available()) {}
      RadarSerial.read();
    }
    return false;
  }
  uint32_t got = 0;
  while (got < pkt_len) {
    if (!waitAvail(RadarSerial, 50)) return false;
    got += RadarSerial.readBytes((char*)(&payload[got]), pkt_len - got);
  }
  if (pkt_len < 3) return true;  // پکت خیلی کوتاه

  /* 4) reserved: برخی FW یک 0x00 یا 4×0x00 در ابتدای payload می‌فرستند */
  const uint8_t* p = payload;
  if (pkt_len >= 2 && payload[0] == 0x00 && payload[1] == 0xA0) p += 1;  // الگوی دیده‌شده: 00 A0 12 ...
  else if (pkt_len >= 5 && payload[0] == 0x00 && payload[1] == 0x00
           && payload[2] == 0x00 && payload[3] == 0x00) p += 4;  // 00 00 00 00 A0 12 ...
  else p += 1;                                                   // محافظه‌کارانه

  if (p >= payload + pkt_len) return true;

  /* 5) برچسب نوع محتوا: A0=Generic, 50=AppData (در Manual معمولاً A0) */
  uint8_t tag = *p++;

  /* ----------- مسیر A0/12: Generic Float (I,Q فلت) ----------- */
  if (tag == 0xA0) {
    if (p >= payload + pkt_len) return true;

    uint8_t subtype = *p++;            // باید 0x12 باشد
    if (subtype != 0x12) return true;  // سایر ساب‌تایپ‌ها را فعلاً نادیده بگیر

    if (p + 12 > payload + pkt_len) return true;
    uint32_t contentId, frameCounter, flen;
    memcpy(&contentId, p, 4);
    p += 4;
    memcpy(&frameCounter, p, 4);
    p += 4;
    memcpy(&flen, p, 4);
    p += 4;

    // flen بعضی FWها را "تعداد floats" و بعضی "بایت" می‌فرستند. هر دو را پوشش بده:
    uint32_t bytes_rem = (uint32_t)(payload + pkt_len - p);
    uint32_t data_bytes = 0;
    if ((flen % 4) == 0 && bytes_rem >= flen) data_bytes = flen;  // flen = bytes
    else if (bytes_rem >= flen * 4) data_bytes = flen * 4;        // flen = floats
    else return true;                                             // ناکافی

    uint32_t nFloats = data_bytes / 4;
    if (nFloats == 0) return true;

    // اگر فریم عوض شد → فریم قبلی را پردازش کن و بافر را صفر کن
    if (g_currFrame != 0xFFFFFFFF && frameCounter != g_currFrame) {
      process_full_generic_frame();
      g_accumCount = 0;
    }
    g_currFrame = frameCounter;

    // هر چه در این پکت آمده به بافرِ فریم جاری اضافه کن (تا سقف)
    uint32_t add = min(nFloats, MAX_FLOATS - g_accumCount);
    memcpy(&g_accum[g_accumCount], (const float*)p, add * 4);
    g_accumCount += add;

    // گزارش سبک هر 10 فریم (برای آگاهی از وضعیت تجمع داده)
    if ((frameCounter % 10) == 0) {
      Serial.printf("[GENERIC] frame=%lu nFloats=%lu (acc=%lu)\n",
                    (unsigned long)frameCounter, (unsigned long)nFloats, (unsigned long)g_accumCount);
    }
    return true;
  }

  /* ----------- مسیر 0x50: AppData Baseband IQ (در Manual کم‌تر رخ می‌دهد) ----------- */
  // اگر لازم شد، مشابه قبل I/Q را از ساختار AppData بخوان و پردازش کن.
  return true;  // سایر تگ‌ها فعلاً نادیده گرفته می‌شوند
}

/* ========================= setup ========================= */
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[Bring-Up] ESP32-S3 <-> X4M03 (UART1)");

  // ریست امن رادار (جلوگیری از ورود به بوت‌لودر)
  radarReset();

  // شروع UART با نرخ پیش‌فرض
  RadarSerial.begin(115200, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  // راه‌اندازی LED تست
  led_init();
  led_set(0.05f);

  // انتظار برای پیام READY از رادار
  Serial.println("Waiting for READY...");
  bool okReady = waitForReady(4000);
  Serial.println(okReady ? "READY received ✅" : "READY timeout!");

  // ورود به حالت Manual
  sendSetMode(XTS_SM_STOP);
  expectAck();
  Serial.println("Mode=STOP ACK");
  sendSetMode(XTS_SM_MANUAL);
  expectAck();
  Serial.println("Mode=MANUAL ACK");

  // افزایش Baud سمت رادار و سپس سمت ESP
  sendSetBaud(921600);
  if (expectAck()) {
    Serial.println("ACK set_baud=921600");
    RadarSerial.updateBaudRate(921600);  // نرخ سمت ESP32
    delay(50);
  }

  // پیش‌تنظیمات Baseband و استارت استریم
  bool okPreset = apply_preset_manual_BB(5.0f);  // FPS=5 برای پایداری
  Serial.println(okPreset ? "Preset applied OK" : "Preset failed (some ACKs missed)");

  Serial.println("Streaming... (summary every ~10 frames)");
}

/* ========================= loop ========================= */
void loop() {
  // هر بار یک پکت استریم بخوان و در صورت تکمیل یک فریم، پردازش کن
  if (!readPacketAndReport()) {
    delay(2);  // در صورت نبود داده، کمی صبر کن تا CPU آزاد بماند
  }
}
