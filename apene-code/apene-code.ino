/*******************************************************
 *  ESP32-S3  <->  XeThru X4M03  (Manual Mode, Baseband, I-only output like Python BB)
 *  - NoEscape: [7C7C7C7C][len:U32LE][00][payload]
 *  - Generic Float (A0/12): I/Q contiguous → چاپ فقط I (نیمهٔ اول)
 *  - Log سازگار: [HDR] + TitleLine + CSV I
 *  - NeoPixel اختیاری (روشن می‌ماند)
 *  - Baud=921600 + RX buffer بزرگ
 *******************************************************/

#include <Arduino.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

/* ---------------- پیکربندی خروجی ---------------- */
#define PRINT_I_CSV_PER_FRAME  1   // 1: هر فریم یک خط CSV از I چاپ شود
#define PRINT_TITLE_PER_FRAME  1   // 1: یک خط Title (مثل GUI پایتون)
#define PRINT_HDR_PER_FRAME    1   // 1: [HDR] برای دیباگ باقی بماند

/* ---------------- پین‌ها ---------------- */
#define RADAR_UART_NUM 1
#define RADAR_RX_PIN   15
#define RADAR_TX_PIN   16
#define RADAR_RESET_PIN 5

#ifndef LED_PIN
#define LED_PIN 48
#endif

/* ---------------- NeoPixel ---------------- */
Adafruit_NeoPixel pixels(1, LED_PIN, NEO_GRB + NEO_KHZ800);
static inline void led_init(){ pixels.begin(); pixels.clear(); pixels.show(); }
static inline void led_set(float a01){
  if(a01<0)a01=0; if(a01>1)a01=1;
  const uint8_t maxLevel=180;
  uint8_t lvl=(uint8_t)(a01*maxLevel);
  pixels.setPixelColor(0, pixels.Color(0, lvl>>3, lvl));
  pixels.show();
}

/* ---------------- MCP (TX Normal+Escape) ---------------- */
const uint8_t XT_START=0x7D, XT_STOP=0x7E, XT_ESC=0x7F;

const uint8_t XTS_SPC_MOD_SETMODE=0x20;
const uint8_t XTS_SPC_X4DRIVER=0x50;
const uint8_t XTS_SPCX_SET=0x10;
const uint8_t XTS_SPR_SYSTEM=0x30;
const uint8_t XTS_SPR_ACK=0x10;
const uint8_t XTS_SM_STOP=0x13;
const uint8_t XTS_SM_MANUAL=0x12;
const uint8_t XTS_SPC_DIR_COMMAND=0x90;
const uint8_t XTS_SDC_COMM_SETBAUDRATE=0x80;

const uint32_t XTS_SPCXI_FPS             =0x00000010;
const uint32_t XTS_SPCXI_PULSES_PER_STEP =0x00000011;
const uint32_t XTS_SPCXI_ITERATIONS      =0x00000012;
const uint32_t XTS_SPCXI_DOWNCONVERSION  =0x00000013;
const uint32_t XTS_SPCXI_FRAME_AREA      =0x00000014;
const uint32_t XTS_SPCXI_DAC_STEP        =0x00000015;
const uint32_t XTS_SPCXI_DAC_MIN         =0x00000016;
const uint32_t XTS_SPCXI_DAC_MAX         =0x00000017;
const uint32_t XTS_SPCXI_FRAME_AREA_OFFSET=0x00000018;
const uint32_t XTS_SPCXI_ENABLE          =0x00000019;
const uint32_t XTS_SPCXI_PRF_DIV         =0x00000020;

const uint32_t XTS_SPRS_READY=0x00000011;

/* ---------------- UART ---------------- */
HardwareSerial RadarSerial(RADAR_UART_NUM);

/* ---------------- ابزار عمومی ---------------- */
static inline bool waitAvail(Stream& s, uint32_t to_ms){
  uint32_t t0=millis(); while(!s.available() && (millis()-t0)<to_ms) delay(1); return s.available();
}
static size_t radarWriteEscaped(const uint8_t* buf, size_t len){
  uint8_t crc=XT_START; for(size_t i=0;i<len;i++) crc^=buf[i];
  auto w=[&](uint8_t b){ if(b==XT_START||b==XT_STOP||b==XT_ESC) RadarSerial.write(XT_ESC); RadarSerial.write(b); };
  RadarSerial.write(XT_START); for(size_t i=0;i<len;i++) w(buf[i]); w(crc); RadarSerial.write(XT_STOP);
  return len+3;
}
static inline void sendSetMode(uint8_t mode){ uint8_t p[2]={XTS_SPC_MOD_SETMODE,mode}; radarWriteEscaped(p,sizeof(p)); }
static inline void sendSet_u8(uint32_t id,uint8_t v){ uint8_t p[1+1+4+1]={XTS_SPC_X4DRIVER,XTS_SPCX_SET}; memcpy(&p[2],&id,4); p[6]=v; radarWriteEscaped(p,sizeof(p)); }
static inline void sendSet_u32(uint32_t id,uint32_t v){ uint8_t p[1+1+4+4]={XTS_SPC_X4DRIVER,XTS_SPCX_SET}; memcpy(&p[2],&id,4); memcpy(&p[6],&v,4); radarWriteEscaped(p,sizeof(p)); }
static inline void sendSet_f32(uint32_t id,float v){ uint8_t p[1+1+4+4]={XTS_SPC_X4DRIVER,XTS_SPCX_SET}; memcpy(&p[2],&id,4); memcpy(&p[6],&v,4); radarWriteEscaped(p,sizeof(p)); }
static inline void sendSet_frame_area(float s,float e){ uint8_t p[1+1+4+4+4]={XTS_SPC_X4DRIVER,XTS_SPCX_SET}; uint32_t id=XTS_SPCXI_FRAME_AREA; memcpy(&p[2],&id,4); memcpy(&p[6],&s,4); memcpy(&p[10],&e,4); radarWriteEscaped(p,sizeof(p)); }
static inline void sendSetBaud(uint32_t baud){ uint8_t p[1+1+4]={XTS_SPC_DIR_COMMAND,XTS_SDC_COMM_SETBAUDRATE}; memcpy(&p[2],&baud,4); radarWriteEscaped(p,sizeof(p)); }
static inline void radarReset(){
  pinMode(RADAR_TX_PIN,OUTPUT); digitalWrite(RADAR_TX_PIN,HIGH);
  pinMode(RADAR_RESET_PIN,OUTPUT); digitalWrite(RADAR_RESET_PIN,LOW); delay(100);
  digitalWrite(RADAR_RESET_PIN,HIGH); delay(120);
  pinMode(RADAR_TX_PIN,INPUT);
}
static inline bool expectAck(uint32_t to=1500){
  uint32_t t0=millis();
  while(millis()-t0<to){
    if(!waitAvail(RadarSerial,10)) continue;
    if(RadarSerial.read()!=XT_START) continue;
    uint8_t first=0;
    while(waitAvail(RadarSerial,50)){
      uint8_t b=RadarSerial.read();
      if(first==0 && b!=XT_START) first=b;
      if(b==XT_STOP) break;
    }
    if(first==XTS_SPR_ACK) return true;
  }
  return false;
}
static inline bool waitForReady(uint32_t to=4000){
  uint32_t t0=millis();
  while(millis()-t0<to){
    if(!waitAvail(RadarSerial,5)) continue;
    if(RadarSerial.read()!=XT_START) continue;
    uint8_t b[12]; size_t n=0;
    while(waitAvail(RadarSerial,50) && n<sizeof(b)){ b[n++]=RadarSerial.read(); if(b[n-1]==XT_STOP) break; }
    if(n>=6 && b[0]==XTS_SPR_SYSTEM){
      uint32_t code=(uint32_t)b[1] | ((uint32_t)b[2]<<8) | ((uint32_t)b[3]<<16) | ((uint32_t)b[4]<<24);
      if(code==XTS_SPRS_READY) return true;
    }
  }
  return false;
}

/* ---------------- وضعیت/پارامترها ---------------- */
static float g_frame_start_m = 0.30f;
static float g_frame_end_m   = 1.00f;
static const uint32_t MAX_FLOATS=8192;
static float    g_accum[MAX_FLOATS];   // I و Q پشت‌سرهم
static uint32_t g_accumCount=0;        // تعداد floatهای انباشته
static uint32_t g_currFrame=0xFFFFFFFF;
static uint32_t g_lastChunkMs=0;
static uint32_t g_framesStored=0;      // برای چاپ Title

/* ---------------- preset: Manual + BB ---------------- */
static bool apply_preset_manual_BB(float fps){
  bool ok=true;
  sendSet_u8 (XTS_SPCXI_ENABLE,1);             ok&=expectAck();
  sendSet_u32(XTS_SPCXI_PULSES_PER_STEP,3);    ok&=expectAck();
  sendSet_u32(XTS_SPCXI_ITERATIONS,16);        ok&=expectAck();
  sendSet_u8 (XTS_SPCXI_DOWNCONVERSION,1);     ok&=expectAck();  // BB (I/Q)

  sendSet_u32(XTS_SPCXI_DAC_MIN,900);          ok&=expectAck();
  sendSet_u32(XTS_SPCXI_DAC_MAX,1150);         ok&=expectAck();
  sendSet_u32(XTS_SPCXI_DAC_STEP,1);           ok&=expectAck();
  sendSet_u32(XTS_SPCXI_PRF_DIV,3);            ok&=expectAck();

  sendSet_f32(XTS_SPCXI_FRAME_AREA_OFFSET,0.0f); ok&=expectAck();
  g_frame_start_m=0.30f; g_frame_end_m=1.00f;
  sendSet_frame_area(g_frame_start_m, g_frame_end_m); ok&=expectAck();

  sendSet_f32(XTS_SPCXI_FPS,0.0f);             ok&=expectAck();
  sendSet_f32(XTS_SPCXI_FPS,fps);              ok&=expectAck();
  return ok;
}

/* ---------------- چاپ سازگار با پایتون ---------------- */
static void print_title_line(uint32_t frameCounter, uint32_t lenI){
  // شبیه عنوان GUI: Bytes in buffer, Frame Counter, Frame Stored, LenOfDatas
  size_t bytesInBuf = RadarSerial.available();
  Serial.printf("Bytes in buffer: %u, Frame Counter: %lu, Frame Stored: %lu, LenOfDatas: %lu\n",
                (unsigned)bytesInBuf, (unsigned long)frameCounter,
                (unsigned long)g_framesStored, (unsigned long)g_framesStored);
}
static void print_I_csv(uint32_t frameCounter, const float* I, uint16_t N){
  // یک خط CSV: "I,frame=<n>,len=<N>,v1,v2,...,vN"
  Serial.printf("I,frame=%lu,len=%u", (unsigned long)frameCounter, (unsigned)N);
  for(uint16_t i=0;i<N;i++){
    Serial.print(',');
    Serial.print(I[i], 6);
  }
  Serial.print('\n');
}

/* ---------------- پردازش کامل فریم (I-only چاپ) ---------------- */
static void process_full_generic_frame(){
  if(g_accumCount<4) return;
  uint32_t N = g_accumCount/2;       // I و Q نصف-نصف
  const float* I = g_accum;          // نیمهٔ اول
  // Q = g_accum + N  (نیازی به چاپ Q نیست)

  if(PRINT_TITLE_PER_FRAME) print_title_line(g_currFrame, (uint32_t)N);
  if(PRINT_I_CSV_PER_FRAME) print_I_csv(g_currFrame, I, (uint16_t)N);

  // بازخورد LED (اختیاری)
  float s=0; for(uint16_t i=0;i<N;i++) s += fabsf(I[i]);
  float mean = (N? s/N : 0.0f);
  static float ema=0; static bool init=false;
  if(!init){ ema=mean; init=true; }
  ema = 0.98f*ema + 0.02f*mean;
  float a01 = (ema>1e-6f? mean/(2.0f*ema) : 0.5f);
  if(a01<0)a01=0; if(a01>1)a01=1;
  led_set(a01);

  g_framesStored++;
}

/* ---------------- Reader: NoEscape + Generic Float ---------------- */
static bool readPacketAndAccumulate(){
  // 1) Sync 7C*4
  uint8_t h[4]={0};
  while(true){
    if(!waitAvail(RadarSerial,100)) return false;
    h[0]=h[1]; h[1]=h[2]; h[2]=h[3]; h[3]=(uint8_t)RadarSerial.read();
    if(h[0]==0x7C && h[1]==0x7C && h[2]==0x7C && h[3]==0x7C) break;
  }
  // 2) len
  while(RadarSerial.available()<4) delay(1);
  uint8_t lb[4]; RadarSerial.readBytes((char*)lb,4);
  uint32_t pkt_len=(uint32_t)lb[0] | ((uint32_t)lb[1]<<8) | ((uint32_t)lb[2]<<16) | ((uint32_t)lb[3]<<24);
  // 3) reserved=0x00
  while(!waitAvail(RadarSerial,20)){}
  (void)RadarSerial.read();
  // 4) payload
  static uint8_t payload[16384];
  if(pkt_len>sizeof(payload)){ for(uint32_t i=0;i<pkt_len;i++){ while(!RadarSerial.available()){} (void)RadarSerial.read(); } return false; }
  uint32_t got=0;
  while(got<pkt_len){
    if(!waitAvail(RadarSerial,50)) return false;
    got+=RadarSerial.readBytes((char*)(&payload[got]), pkt_len-got);
  }
  if(pkt_len<3) return true;

  // 5) A0/12
  const uint8_t* p=payload; uint8_t tag=*p++; if(tag!=0xA0) return true;
  if(p>=payload+pkt_len) return true;
  uint8_t subtype=*p++; if(subtype!=0x12) return true;
  if(p+12>payload+pkt_len) return true;

  uint32_t contentId,frameCounter,flen;
  memcpy(&contentId,   p,4); p+=4;
  memcpy(&frameCounter,p,4); p+=4;
  memcpy(&flen,        p,4); p+=4;

  uint32_t bytes_rem=(uint32_t)(payload+pkt_len-p);
  // flen = تعداد float (پیش‌فرض)؛ اگر نشد، bytes
  uint32_t data_bytes = (bytes_rem>=flen*4) ? flen*4
                        : (((flen%4)==0 && bytes_rem>=flen) ? flen : 0);
  if(!data_bytes) return true;
  uint32_t nFloats=data_bytes/4;

  if(PRINT_HDR_PER_FRAME){
    Serial.printf("[HDR] frame=%lu flen=%lu bytes_rem=%lu data_bytes=%lu nFloats=%lu pkt_len=%lu\n",
      (unsigned long)frameCounter,(unsigned long)flen,(unsigned long)bytes_rem,
      (unsigned long)data_bytes,(unsigned long)nFloats,(unsigned long)pkt_len);
  }

  // مرز فریم
  if(g_currFrame!=0xFFFFFFFF && frameCounter!=g_currFrame){
    process_full_generic_frame();    // فریم قبلی را مثل پایتون «ثبت» کن
    g_accumCount=0;
  }
  g_currFrame=frameCounter;

  // تجمیع (I+Q پشت‌سر هم)
  uint32_t add=min(nFloats, MAX_FLOATS-g_accumCount);
  memcpy(&g_accum[g_accumCount], (const float*)p, add*4);
  g_accumCount+=add;
  g_lastChunkMs=millis();

  return true;
}

/* ---------------- setup/loop ---------------- */
void setup(){
  Serial.begin(115200); delay(200);
  Serial.println("\n[ESP32-S3] X4M03 → BB (I-only log like Python)");

  radarReset();

  RadarSerial.setRxBufferSize(32768);
  RadarSerial.begin(115200, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  led_init(); led_set(0.05f);

  Serial.println("Waiting for READY...");
  bool okReady=waitForReady(4000);
  Serial.println(okReady? "READY received ✅":"READY timeout!");

  sendSetMode(XTS_SM_STOP);   expectAck();
  sendSetMode(XTS_SM_MANUAL); expectAck();

  sendSetBaud(921600);
  if(expectAck()){
    Serial.println("ACK set_baud=921600");
    RadarSerial.updateBaudRate(921600);
    delay(80);
  }

  bool okPreset=apply_preset_manual_BB(5.0f); // FPS=5
  Serial.println(okPreset? "Preset applied OK":"Preset failed (ACK miss)");
  Serial.println("Streaming (I-only CSV per frame)...");
}

void loop(){
  if(!readPacketAndAccumulate()) delay(2);

  // بستن فریم با تایم‌اوت (در صورت چندپکتی بودن)
  static const uint32_t FRAME_CLOSE_TIMEOUT_MS = 20;
  if(g_currFrame!=0xFFFFFFFF && (millis()-g_lastChunkMs)>FRAME_CLOSE_TIMEOUT_MS){
    process_full_generic_frame();     // چاپ Title + CSV (I)
    g_accumCount=0;
    g_currFrame=0xFFFFFFFF;
  }
}
