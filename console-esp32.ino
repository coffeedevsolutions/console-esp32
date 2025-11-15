#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include "secrets.h"

#ifndef PIN_NEOPIXEL
  #define PIN_NEOPIXEL NEOPIXEL_PIN   // defined by QT Py S3 board package
#endif
Adafruit_NeoPixel px(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ---- Wi-Fi (STA so emulator/phone/PC all reach it) ----
const char* SSID = WIFI_SSID;
const char* PASS = WIFI_PASSWORD;

// ---- HTTP server ----
WebServer server(80);

// ---- NVS storage ----
Preferences prefs;
const char* NVS_NS = "whir";

// ---- Helpers ----
static inline float  clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
static inline uint8_t clampu8(int v, int lo, int hi){ return (uint8_t)clampf((float)v,(float)lo,(float)hi); }
void ack(int r,int g,int b,int ms=80){
  px.setPixelColor(0, px.Color(r,g,b));
  px.show();
  delay(ms);
  px.setPixelColor(0, 0);   // off (2-arg form: n, packedColor)
  px.show();
}

// Scan for SSID and connect to the strongest BSSID (AP)
bool connectToBestSSID(){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  Serial.print("Scanning for ");
  Serial.println(SSID);

  int n = WiFi.scanNetworks();
  if(n <= 0){
    Serial.println("No networks found.");
    return false;
  }

  int bestIndex = -1;
  int bestRSSI  = -1000;

  for(int i=0; i<n; i++){
    String thisSSID = WiFi.SSID(i);
    if(thisSSID == SSID){
      int rssi = WiFi.RSSI(i);
      uint8_t* bssid = WiFi.BSSID(i);
      int channel = WiFi.channel(i);

      Serial.print("Found ");
      Serial.print(SSID);
      Serial.print(" at index ");
      Serial.print(i);
      Serial.print(" | RSSI ");
      Serial.print(rssi);
      Serial.print(" dBm | channel ");
      Serial.print(channel);
      Serial.print(" | BSSID ");
      Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                    bssid[0], bssid[1], bssid[2],
                    bssid[3], bssid[4], bssid[5]);
      Serial.println();

      if(rssi > bestRSSI){
        bestRSSI  = rssi;
        bestIndex = i;
      }
    }
  }

  if(bestIndex == -1){
    Serial.print("No networks matching SSID ");
    Serial.print(SSID);
    Serial.println(" found in scan.");
    return false;
  }

  int channel = WiFi.channel(bestIndex);
  uint8_t* bssid = WiFi.BSSID(bestIndex);

  Serial.println();
  Serial.print("Connecting to best ");
  Serial.print(SSID);
  Serial.print(" (RSSI ");
  Serial.print(bestRSSI);
  Serial.print(" dBm) on channel ");
  Serial.println(channel);

  WiFi.begin(SSID, PASS, channel, bssid, true);

  int attempts = 0;
  const int maxAttempts = 20; // ~10s
  while(WiFi.status() != WL_CONNECTED && attempts < maxAttempts){
    Serial.print(".");
    attempts++;
    delay(500);
  }
  Serial.println();

  if(WiFi.status() == WL_CONNECTED){
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("Failed to connect to best AP.");
    return false;
  }
}

// =========================
//          STATE
// =========================

// Filter type enumeration
enum FilterType : uint8_t {
  FILTER_BW = 0,  // Butterworth
  FILTER_LR = 1   // Linkwitz-Riley
};

// Filter slope enumeration
enum FilterSlope : uint8_t {
  SLOPE_12 = 12,
  SLOPE_18 = 18,
  SLOPE_24 = 24,
  SLOPE_36 = 36
};

// Physical input channel (A or B)
struct Input {
  float gainDb = 0.0f;      // dB  (-45..+15)
  bool  mute = false;
  bool  active = true;      // display only: signal present
};

struct Peq {
  float f=1000.0f;   // Hz
  float g=0.0f;      // dB  (-12..+12)
  float q=1.0f;      // 0.4..10
};

struct Xover {
  FilterType type = FILTER_LR;    // BW or LR
  FilterSlope slope = SLOPE_24;   // 12/18/24/36 dB/oct
  float freq = 100.0f;            // Hz (10..22000)
  bool enabled = false;
};

struct Limiter {
  float threshold=-6.0f; // dB  (-24..0)
  float attack=10.0f;    // ms  (0.1..100)
  float release=100.0f;  // ms  (1..1600) ignored if autoRelease
  bool  autoRelease=true;
  bool  enabled=false;
  bool  active=false;    // display only
};

struct Output {
  String route = "A";          // "A" | "B" | "A+B"
  Xover hpf;
  Xover lpf;
  Peq peq;
  float delayMs = 0.0f;        // ms  (0..8 ≈ 275cm acoustic)
  float phaseDeg = 0.0f;       // degrees (0..180)
  Limiter lim;
  float gainDb = 0.0f;         // dB  (-45..+15)
  bool mute = false;
};

struct Generators {
  bool  sineEn=false;   float sineHz=1000.0f;    float sineLevelDb=-20.0f;
  bool  sweepEn=false;  float sweepStart=20.0f;  float sweepEnd=20000.0f; float sweepLevelDb=-20.0f;
  bool  pinkEn=false;   float pinkLevelDb=-20.0f;
};

struct Sequencer {
  bool s1=false, s2=false, s3=false;
  uint32_t intervalMs=500;
  uint32_t lastTick=0;
  bool running=false;
};

struct Battery {
  float v=12.6f, vmin=12.6f, vmax=12.6f; // display only
};

// =========================
//   CROSSOVER TEMPLATES
// =========================

struct XoTemplate {
  const char* name;
  struct FilterConfig {
    FilterType type;
    FilterSlope slope;
    float freq;
    bool enabled;
  } hpf, lpf;
};

const XoTemplate XO_TEMPLATES[11] PROGMEM = {
  {"FullRange", {FILTER_LR, SLOPE_24, 20.0f, false},    {FILTER_LR, SLOPE_24, 20000.0f, false}},
  {"Sub",       {FILTER_LR, SLOPE_24, 20.0f, false},    {FILTER_LR, SLOPE_24, 80.0f, true}},
  {"Mid",       {FILTER_LR, SLOPE_24, 80.0f, true},     {FILTER_LR, SLOPE_24, 5000.0f, true}},
  {"High",      {FILTER_LR, SLOPE_24, 5000.0f, true},   {FILTER_LR, SLOPE_24, 20000.0f, false}},
  {"2wayLow",   {FILTER_LR, SLOPE_24, 20.0f, false},    {FILTER_LR, SLOPE_24, 3000.0f, true}},
  {"2wayHigh",  {FILTER_LR, SLOPE_24, 3000.0f, true},   {FILTER_LR, SLOPE_24, 20000.0f, false}},
  {"3wayLow",   {FILTER_LR, SLOPE_24, 20.0f, false},    {FILTER_LR, SLOPE_24, 250.0f, true}},
  {"3wayMid",   {FILTER_LR, SLOPE_24, 250.0f, true},    {FILTER_LR, SLOPE_24, 3000.0f, true}},
  {"3wayHigh",  {FILTER_LR, SLOPE_24, 3000.0f, true},   {FILTER_LR, SLOPE_24, 20000.0f, false}},
  {"Bandpass",  {FILTER_LR, SLOPE_24, 80.0f, true},     {FILTER_LR, SLOPE_24, 5000.0f, true}},
  {"Custom",    {FILTER_LR, SLOPE_24, 100.0f, false},   {FILTER_LR, SLOPE_24, 10000.0f, false}}
};

struct Device {
  // Global master level
  float master = 0.50f;        // current master (0..1)

  // Physical inputs A & B
  Input in[2];                 // in[0]=A, in[1]=B

  // Input signal path
  float geq[15] = {0};         // 15-band GEQ, ±12 dB
  uint8_t geqPreset = 0;       // GEQ preset index (0..11)
  Peq inPeq;                   // Input parametric EQ

  // Output channels
  Output out[4];               // 4 output channels

  // Crossover preset index
  uint8_t xoPreset = 0;        // XO template index (0..10)

  // Test tools
  Generators gen;
  Sequencer seq;
  Battery bat;

  // System state
  String lastCmd = "none";
  bool locked = false;
  uint32_t lockCode = 0;       // 000000..999999
} dev;

// Smooth ramp target for master (so UI can be choppy, device stays smooth)
float masterTarget = 0.50f;         // 0..1
uint32_t masterLastStepMs = 0;
const float MASTER_RATE_PER_SEC = 1.0f; // 1.0 => 0..100% in ~1s

// ---- Preset names (labels only; curves are user-provided) ----
const char* GEQ_PRESET_NAMES[12] = {
  "Flat","BassBoost","TrebleBoost","VSmile","Loudness","Vocal","Acoustic","Rock","Pop","Jazz","Classical","Custom"
};
const char* XO_PRESET_NAMES[11] = {
  "FullRange","Sub","Mid","High","2wayLow","2wayHigh","3wayLow","3wayMid","3wayHigh","Bandpass","Custom"
};

// =========================
//     JSON (de)serialize
// =========================

void stateToJson(JsonDocument& doc){
  doc["device"]       = "QT Py ESP32-S3";
  doc["uptime"]       = (uint32_t)millis();
  doc["master"]       = dev.master;
  doc["masterTarget"] = masterTarget;
  doc["lastCmd"]      = dev.lastCmd;
  doc["locked"]       = dev.locked;

  // battery
  auto jb = doc.createNestedObject("battery");
  jb["v"]=dev.bat.v; jb["min"]=dev.bat.vmin; jb["max"]=dev.bat.vmax;

  // physical inputs A & B
  auto jins = doc.createNestedArray("inputs");
  for(int i=0; i<2; i++){
    auto jin = jins.createNestedObject();
    jin["ch"] = (i==0) ? "A" : "B";
    jin["gainDb"] = dev.in[i].gainDb;
    jin["mute"] = dev.in[i].mute;
    jin["active"] = dev.in[i].active;
  }

  // input signal path
  auto jin = doc.createNestedObject("input");
  auto jgeq = jin.createNestedArray("geq");
  for(int i=0;i<15;i++) jgeq.add(dev.geq[i]);
  jin["geqPreset"]=dev.geqPreset;
  auto jpeq = jin.createNestedObject("peq");
  jpeq["f"]=dev.inPeq.f; jpeq["g"]=dev.inPeq.g; jpeq["q"]=dev.inPeq.q;

  // outputs
  auto jouts = doc.createNestedArray("outputs");
  for(int ch=0; ch<4; ch++){
    auto jo = jouts.createNestedObject();
    jo["ch"]=ch+1;
    jo["route"]=dev.out[ch].route;
    // HPF
    auto jh = jo.createNestedObject("hpf");
    jh["type"]=(dev.out[ch].hpf.type == FILTER_BW) ? "BW" : "LR";
    jh["slope"]=(uint8_t)dev.out[ch].hpf.slope;
    jh["freq"]=dev.out[ch].hpf.freq;
    jh["enabled"]=dev.out[ch].hpf.enabled;
    // LPF
    auto jl = jo.createNestedObject("lpf");
    jl["type"]=(dev.out[ch].lpf.type == FILTER_BW) ? "BW" : "LR";
    jl["slope"]=(uint8_t)dev.out[ch].lpf.slope;
    jl["freq"]=dev.out[ch].lpf.freq;
    jl["enabled"]=dev.out[ch].lpf.enabled;
    // PEQ
    auto jpo = jo.createNestedObject("peq");
    jpo["f"]=dev.out[ch].peq.f; jpo["g"]=dev.out[ch].peq.g; jpo["q"]=dev.out[ch].peq.q;
    // Time alignment & phase
    jo["delayMs"]=dev.out[ch].delayMs;
    jo["phaseDeg"]=dev.out[ch].phaseDeg;
    jo["invert"]=(dev.out[ch].phaseDeg >= 179.0f);  // backward compat
    // Limiter
    auto jlim = jo.createNestedObject("limiter");
    jlim["thr"]=dev.out[ch].lim.threshold; jlim["atk"]=dev.out[ch].lim.attack;
    jlim["rel"]=dev.out[ch].lim.release; jlim["auto"]=dev.out[ch].lim.autoRelease;
    jlim["en"]=dev.out[ch].lim.enabled;  jlim["act"]=dev.out[ch].lim.active;
    // Gain / mute
    jo["gainDb"]=dev.out[ch].gainDb; jo["mute"]=dev.out[ch].mute;
  }

  // crossover preset index
  doc["xoPreset"]=dev.xoPreset;

  // generators
  auto jg = doc.createNestedObject("gen");
  jg["sineEn"]=dev.gen.sineEn;  jg["sineHz"]=dev.gen.sineHz;  jg["sineDb"]=dev.gen.sineLevelDb;
  jg["sweepEn"]=dev.gen.sweepEn; jg["sweepStart"]=dev.gen.sweepStart; jg["sweepEnd"]=dev.gen.sweepEnd; jg["sweepDb"]=dev.gen.sweepLevelDb;
  jg["pinkEn"]=dev.gen.pinkEn;  jg["pinkDb"]=dev.gen.pinkLevelDb;

  // sequencer
  auto js = doc.createNestedObject("seq");
  js["s1"]=dev.seq.s1; js["s2"]=dev.seq.s2; js["s3"]=dev.seq.s3;
  js["intervalMs"]=dev.seq.intervalMs;
}

// save/load working state
void saveWorking(){
  StaticJsonDocument<4096> doc;
  stateToJson(doc);
  String s; serializeJson(doc, s);
  prefs.putString("working", s);
}

bool loadWorking(){
  String s = prefs.getString("working", "");
  if(s.length()==0) return false;
  StaticJsonDocument<4096> doc;
  auto err = deserializeJson(doc, s);
  if(err) return false;

  // globals
  dev.master      = doc["master"]       | dev.master;
  masterTarget    = doc["masterTarget"] | dev.master; // if missing, target = current
  dev.bat.v       = doc["battery"]["v"]   | dev.bat.v;
  dev.bat.vmin    = doc["battery"]["min"] | dev.bat.vmin;
  dev.bat.vmax    = doc["battery"]["max"] | dev.bat.vmax;

  // physical inputs A & B
  JsonArray ins = doc["inputs"].as<JsonArray>();
  if(!ins.isNull() && ins.size()>=2){
    for(int i=0; i<2; i++){
      auto jin = ins[i];
      dev.in[i].gainDb = clampf((float)(jin["gainDb"] | dev.in[i].gainDb), -45, +15);
      dev.in[i].mute = (bool)(jin["mute"] | dev.in[i].mute);
      dev.in[i].active = (bool)(jin["active"] | dev.in[i].active);
    }
  }

  // input signal path
  JsonArray g = doc["input"]["geq"].as<JsonArray>();
  if(!g.isNull() && g.size()==15) for(int i=0;i<15;i++) dev.geq[i] = clampf(g[i].as<float>(), -12, +12);
  dev.geqPreset = doc["input"]["geqPreset"] | dev.geqPreset;
  dev.inPeq.f = clampf((float)(doc["input"]["peq"]["f"] | dev.inPeq.f), 10, 22000);
  dev.inPeq.g = clampf((float)(doc["input"]["peq"]["g"] | dev.inPeq.g), -12, +12);
  dev.inPeq.q = clampf((float)(doc["input"]["peq"]["q"] | dev.inPeq.q), 0.4, 10.0);

  // outputs
  JsonArray outs = doc["outputs"].as<JsonArray>();
  if(!outs.isNull() && outs.size()>=4){
    for(int ch=0; ch<4; ch++){
      auto jo = outs[ch];
      dev.out[ch].route    = (const char*)(jo["route"] | dev.out[ch].route.c_str());

      auto jh = jo["hpf"];
      if(jh.containsKey("type")){
        const char* t = jh["type"];
        dev.out[ch].hpf.type = (strcmp(t, "BW") == 0) ? FILTER_BW : FILTER_LR;
      }
      uint8_t slope_h = clampu8(jh["slope"] | (uint8_t)dev.out[ch].hpf.slope, 12, 36);
      switch(slope_h){ case 12: dev.out[ch].hpf.slope=SLOPE_12; break; case 18: dev.out[ch].hpf.slope=SLOPE_18; break; case 24: dev.out[ch].hpf.slope=SLOPE_24; break; case 36: dev.out[ch].hpf.slope=SLOPE_36; break; default: dev.out[ch].hpf.slope=SLOPE_24; }
      dev.out[ch].hpf.freq    = clampf((float)(jh["freq"] | dev.out[ch].hpf.freq), 10, 22000);
      dev.out[ch].hpf.enabled = (bool)(jh["enabled"] | dev.out[ch].hpf.enabled);

      auto jl = jo["lpf"];
      if(jl.containsKey("type")){
        const char* t = jl["type"];
        dev.out[ch].lpf.type = (strcmp(t, "BW") == 0) ? FILTER_BW : FILTER_LR;
      }
      uint8_t slope_l = clampu8(jl["slope"] | (uint8_t)dev.out[ch].lpf.slope, 12, 36);
      switch(slope_l){ case 12: dev.out[ch].lpf.slope=SLOPE_12; break; case 18: dev.out[ch].lpf.slope=SLOPE_18; break; case 24: dev.out[ch].lpf.slope=SLOPE_24; break; case 36: dev.out[ch].lpf.slope=SLOPE_36; break; default: dev.out[ch].lpf.slope=SLOPE_24; }
      dev.out[ch].lpf.freq    = clampf((float)(jl["freq"] | dev.out[ch].lpf.freq), 10, 22000);
      dev.out[ch].lpf.enabled = (bool)(jl["enabled"] | dev.out[ch].lpf.enabled);

      auto jpo = jo["peq"];
      dev.out[ch].peq.f = clampf((float)(jpo["f"] | dev.out[ch].peq.f), 10, 22000);
      dev.out[ch].peq.g = clampf((float)(jpo["g"] | dev.out[ch].peq.g), -12, +12);
      dev.out[ch].peq.q = clampf((float)(jpo["q"] | dev.out[ch].peq.q), 0.4, 10.0);

      dev.out[ch].delayMs = clampf((float)(jo["delayMs"] | dev.out[ch].delayMs), 0, 8.0);
      // Phase: support legacy "invert" or new "phaseDeg"
      if(jo.containsKey("invert"))
        dev.out[ch].phaseDeg = (bool)jo["invert"] ? 180.0f : 0.0f;
      dev.out[ch].phaseDeg = clampf((float)(jo["phaseDeg"] | dev.out[ch].phaseDeg), 0, 180);

      auto jlim = jo["limiter"];
      dev.out[ch].lim.threshold   = clampf((float)(jlim["thr"] | dev.out[ch].lim.threshold), -24, 0);
      dev.out[ch].lim.attack      = clampf((float)(jlim["atk"] | dev.out[ch].lim.attack), 0.1, 100);
      dev.out[ch].lim.release     = clampf((float)(jlim["rel"] | dev.out[ch].lim.release), 1, 1600);
      dev.out[ch].lim.autoRelease = (bool)(jlim["auto"] | dev.out[ch].lim.autoRelease);
      dev.out[ch].lim.enabled     = (bool)(jlim["en"]   | dev.out[ch].lim.enabled);

      dev.out[ch].gainDb = clampf((float)(jo["gainDb"] | dev.out[ch].gainDb), -45, +15);
      dev.out[ch].mute   = (bool)(jo["mute"]   | dev.out[ch].mute);
    }
  }
  dev.xoPreset = doc["xoPreset"] | dev.xoPreset;
  // generators
  auto jg = doc["gen"];
  dev.gen.sineEn      = (bool)(jg["sineEn"]  | dev.gen.sineEn);
  dev.gen.sineHz      = clampf((float)(jg["sineHz"] | dev.gen.sineHz), 10, 22000);
  dev.gen.sineLevelDb = clampf((float)(jg["sineDb"] | dev.gen.sineLevelDb), -60, 0);
  dev.gen.sweepEn     = (bool)(jg["sweepEn"] | dev.gen.sweepEn);
  dev.gen.sweepStart  = clampf((float)(jg["sweepStart"] | dev.gen.sweepStart), 10, 22000);
  dev.gen.sweepEnd    = clampf((float)(jg["sweepEnd"]   | dev.gen.sweepEnd), 10, 22000);
  dev.gen.sweepLevelDb= clampf((float)(jg["sweepDb"]    | dev.gen.sweepLevelDb), -60, 0);
  dev.gen.pinkEn      = (bool)(jg["pinkEn"]  | dev.gen.pinkEn);
  dev.gen.pinkLevelDb = clampf((float)(jg["pinkDb"]     | dev.gen.pinkLevelDb), -60, 0);

  // sequencer
  auto js = doc["seq"];
  dev.seq.s1 = (bool)(js["s1"] | dev.seq.s1);
  dev.seq.s2 = (bool)(js["s2"] | dev.seq.s2);
  dev.seq.s3 = (bool)(js["s3"] | dev.seq.s3);
  dev.seq.intervalMs = (uint32_t)(js["intervalMs"] | dev.seq.intervalMs);

  return true;
}

// ---- Preset slots in NVS: "preset_0".."preset_15" ----
const int MAX_PRESETS = 16;

bool savePresetSlot(int slot, const String& name){
  if(slot<0 || slot>=MAX_PRESETS) return false;
  StaticJsonDocument<4096> doc;
  stateToJson(doc);
  doc["meta"]["name"]=name;
  String s; serializeJson(doc, s);
  String key = "preset_" + String(slot);
  prefs.putString(key.c_str(), s);
  return true;
}
bool loadPresetSlot(int slot){
  if(slot<0 || slot>=MAX_PRESETS) return false;
  String key = "preset_" + String(slot);
  String s = prefs.getString(key.c_str(), "");
  if(s.length()==0) return false;
  StaticJsonDocument<4096> doc;
  if(deserializeJson(doc, s)) return false;

  // restore similar to loadWorking()
  dev.master      = doc["master"]       | dev.master;
  masterTarget    = doc["masterTarget"] | dev.master;

  // physical inputs A & B
  JsonArray ins = doc["inputs"].as<JsonArray>();
  if(!ins.isNull() && ins.size()>=2){
    for(int i=0; i<2; i++){
      auto jin = ins[i];
      dev.in[i].gainDb = (float)(jin["gainDb"] | dev.in[i].gainDb);
      dev.in[i].mute = (bool)(jin["mute"] | dev.in[i].mute);
      dev.in[i].active = (bool)(jin["active"] | dev.in[i].active);
    }
  }

  JsonArray g = doc["input"]["geq"].as<JsonArray>();
  if(!g.isNull() && g.size()==15) for(int i=0;i<15;i++) dev.geq[i] = g[i].as<float>();
  dev.inPeq.f = doc["input"]["peq"]["f"] | dev.inPeq.f;
  dev.inPeq.g = doc["input"]["peq"]["g"] | dev.inPeq.g;
  dev.inPeq.q = doc["input"]["peq"]["q"] | dev.inPeq.q;

  JsonArray outs = doc["outputs"].as<JsonArray>();
  if(!outs.isNull() && outs.size()>=4){
    for(int ch=0; ch<4; ch++){
      auto jo = outs[ch];
      dev.out[ch].route = (const char*)(jo["route"] | dev.out[ch].route.c_str());
      auto jh = jo["hpf"];
      if(jh.containsKey("type")){
        const char* t = jh["type"];
        dev.out[ch].hpf.type = (strcmp(t, "BW") == 0) ? FILTER_BW : FILTER_LR;
      }
      uint8_t slope_h2 = (uint8_t)(jh["slope"] | (uint8_t)dev.out[ch].hpf.slope);
      switch(slope_h2){ case 12: dev.out[ch].hpf.slope=SLOPE_12; break; case 18: dev.out[ch].hpf.slope=SLOPE_18; break; case 24: dev.out[ch].hpf.slope=SLOPE_24; break; case 36: dev.out[ch].hpf.slope=SLOPE_36; break; default: dev.out[ch].hpf.slope=SLOPE_24; }
      dev.out[ch].hpf.freq    = jh["freq"]  | dev.out[ch].hpf.freq;
      dev.out[ch].hpf.enabled = jh["enabled"] | dev.out[ch].hpf.enabled;

      auto jl = jo["lpf"];
      if(jl.containsKey("type")){
        const char* t = jl["type"];
        dev.out[ch].lpf.type = (strcmp(t, "BW") == 0) ? FILTER_BW : FILTER_LR;
      }
      uint8_t slope_l2 = (uint8_t)(jl["slope"] | (uint8_t)dev.out[ch].lpf.slope);
      switch(slope_l2){ case 12: dev.out[ch].lpf.slope=SLOPE_12; break; case 18: dev.out[ch].lpf.slope=SLOPE_18; break; case 24: dev.out[ch].lpf.slope=SLOPE_24; break; case 36: dev.out[ch].lpf.slope=SLOPE_36; break; default: dev.out[ch].lpf.slope=SLOPE_24; }
      dev.out[ch].lpf.freq    = jl["freq"]  | dev.out[ch].lpf.freq;
      dev.out[ch].lpf.enabled = jl["enabled"] | dev.out[ch].lpf.enabled;

      auto jpo = jo["peq"];
      dev.out[ch].peq.f = jpo["f"] | dev.out[ch].peq.f;
      dev.out[ch].peq.g = jpo["g"] | dev.out[ch].peq.g;
      dev.out[ch].peq.q = jpo["q"] | dev.out[ch].peq.q;

      dev.out[ch].delayMs = jo["delayMs"] | dev.out[ch].delayMs;
      // Phase: support legacy "invert" or new "phaseDeg"
      if(jo.containsKey("invert"))
        dev.out[ch].phaseDeg = (bool)jo["invert"] ? 180.0f : 0.0f;
      dev.out[ch].phaseDeg = (float)(jo["phaseDeg"] | dev.out[ch].phaseDeg);

      auto jlim = jo["limiter"];
      dev.out[ch].lim.threshold   = jlim["thr"] | dev.out[ch].lim.threshold;
      dev.out[ch].lim.attack      = jlim["atk"] | dev.out[ch].lim.attack;
      dev.out[ch].lim.release     = jlim["rel"] | dev.out[ch].lim.release;
      dev.out[ch].lim.autoRelease = jlim["auto"]| dev.out[ch].lim.autoRelease;
      dev.out[ch].lim.enabled     = jlim["en"]  | dev.out[ch].lim.enabled;

      dev.out[ch].gainDb = jo["gainDb"] | dev.out[ch].gainDb;
      dev.out[ch].mute   = jo["mute"]   | dev.out[ch].mute;
    }
  }
  return true;
}

// =========================
//    AUTH (6-digit pin)
// =========================
bool requireCodeIfLocked(){
  if(!dev.locked) return true;
  if(!server.hasArg("code")) { server.send(403, "application/json", "{\"ok\":false,\"err\":\"locked\"}"); return false; }
  String code = server.arg("code");
  if(code.length()!=6 || (uint32_t)code.toInt()!=dev.lockCode){
    server.send(403, "application/json", "{\"ok\":false,\"err\":\"bad code\"}");
    return false;
  }
  return true;
}

// =========================
//       HTTP HANDLERS
// =========================

void sendJson(const String& body, int code=200){ server.send(code, "application/json", body); }

void handleStatus(){
  StaticJsonDocument<1024> doc;
  doc["device"]="QT Py ESP32-S3";
  doc["uptime"]=(uint32_t)millis();
  doc["master"]=dev.master;
  doc["masterTarget"]=masterTarget;
  doc["locked"]=dev.locked;
  doc["lastCmd"]=dev.lastCmd;
  auto b = doc.createNestedObject("battery");
  b["v"]=dev.bat.v; b["min"]=dev.bat.vmin; b["max"]=dev.bat.vmax;
  String s; serializeJson(doc, s); sendJson(s);
}

void handleStateGet(){
  StaticJsonDocument<4096> doc; stateToJson(doc); String s; serializeJson(doc, s); sendJson(s);
}

void handleMaster(){
  if(!requireCodeIfLocked()) return;
  if(!server.hasArg("plain")) { sendJson("{\"ok\":false,\"err\":\"no body\"}", 400); return; }
  StaticJsonDocument<256> d;
  if(deserializeJson(d, server.arg("plain"))) { sendJson("{\"ok\":false,\"err\":\"bad json\"}", 400); return; }
  float pct = d["levelPct"] | -1.0f;
  if(pct<0) { sendJson("{\"ok\":false,\"err\":\"missing levelPct\"}", 400); return; }
  masterTarget = clampf(pct/100.0f, 0, 1);
  Serial.printf("[API] master target -> %.0f%%\n", masterTarget*100);
  ack(0,32,0);
  saveWorking(); // persists masterTarget via /state JSON
  sendJson("{\"ok\":true}");
}

void handleInputGeq(){
  if(!requireCodeIfLocked()) return;
  if(!server.hasArg("plain")) { sendJson("{\"ok\":false}", 400); return; }
  StaticJsonDocument<1024> d;
  if(deserializeJson(d, server.arg("plain"))) { sendJson("{\"ok\":false}", 400); return; }
  if(d.containsKey("bands")){
    JsonArray a = d["bands"].as<JsonArray>();
    if(a.size()!=15){ sendJson("{\"ok\":false,\"err\":\"need 15 bands\"}", 400); return; }
    for(int i=0;i<15;i++) dev.geq[i] = clampf(a[i].as<float>(), -12, +12);
  }
  if(d.containsKey("preset")) dev.geqPreset = clampu8((int)d["preset"], 0, 11);
  Serial.println("[API] input GEQ updated");
  ack(0,32,0);
  saveWorking();
  sendJson("{\"ok\":true}");
}

void handleInputPeq(){
  if(!requireCodeIfLocked()) return;
  if(!server.hasArg("plain")){ sendJson("{\"ok\":false}", 400); return; }
  StaticJsonDocument<256> d; if(deserializeJson(d, server.arg("plain"))){ sendJson("{\"ok\":false}",400); return; }
  if(d.containsKey("f")) dev.inPeq.f = clampf((float)d["f"], 10, 22000);
  if(d.containsKey("g")) dev.inPeq.g = clampf((float)d["g"], -12, +12);
  if(d.containsKey("q")) dev.inPeq.q = clampf((float)d["q"], 0.4, 10.0);
  Serial.printf("[API] input PEQ f=%.1f g=%.1f q=%.2f\n", dev.inPeq.f, dev.inPeq.g, dev.inPeq.q);
  ack(0,32,0);
  saveWorking();
  sendJson("{\"ok\":true}");
}

void handleInputChannel(){
  if(!requireCodeIfLocked()) return;
  if(!server.hasArg("plain")){ sendJson("{\"ok\":false}", 400); return; }
  StaticJsonDocument<256> d; if(deserializeJson(d, server.arg("plain"))){ sendJson("{\"ok\":false}",400); return; }
  if(!d.containsKey("ch")){ sendJson("{\"ok\":false,\"err\":\"missing ch\"}",400); return; }
  String ch = (const char*)d["ch"];
  int idx = (ch == "A") ? 0 : (ch == "B") ? 1 : -1;
  if(idx < 0){ sendJson("{\"ok\":false,\"err\":\"ch must be A or B\"}",400); return; }

  if(d.containsKey("gainDb")) dev.in[idx].gainDb = clampf((float)d["gainDb"], -45, +15);
  if(d.containsKey("mute")) dev.in[idx].mute = (bool)d["mute"];

  Serial.printf("[API] input %s gainDb=%.1f mute=%d\n", ch.c_str(), dev.in[idx].gainDb, dev.in[idx].mute);
  ack(0,32,0);
  saveWorking();
  sendJson("{\"ok\":true}");
}

void handleOutput(){
  if(!requireCodeIfLocked()) return;
  if(!server.hasArg("plain")){ sendJson("{\"ok\":false}", 400); return; }
  StaticJsonDocument<1024> d; if(deserializeJson(d, server.arg("plain"))){ sendJson("{\"ok\":false}",400); return; }
  int ch = (int)d["ch"] - 1; if(ch<0 || ch>3){ sendJson("{\"ok\":false,\"err\":\"ch 1..4\"}",400); return; }
  auto &o = dev.out[ch];

  if(d.containsKey("route")){ String r = (const char*)d["route"]; if(r=="A"||r=="B"||r=="A+B") o.route=r; }
  if(d.containsKey("hpf")){
    auto j=d["hpf"];
    if(j.containsKey("type")){
      String t=(const char*)j["type"];
      if(t=="BW"||t=="LR") o.hpf.type = (strcmp(t.c_str(), "BW") == 0) ? FILTER_BW : FILTER_LR;
    }
    if(j.containsKey("slope")){
      uint8_t s = clampu8((int)j["slope"], 12, 36);
      switch(s){ case 12: o.hpf.slope=SLOPE_12; break; case 18: o.hpf.slope=SLOPE_18; break; case 24: o.hpf.slope=SLOPE_24; break; case 36: o.hpf.slope=SLOPE_36; break; default: o.hpf.slope=SLOPE_24; }
    }
    if(j.containsKey("freq"))  o.hpf.freq  = clampf((float)j["freq"], 10, 22000);
    if(j.containsKey("enabled")) o.hpf.enabled = (bool)j["enabled"];
  }
  if(d.containsKey("lpf")){
    auto j=d["lpf"];
    if(j.containsKey("type")){
      String t=(const char*)j["type"];
      if(t=="BW"||t=="LR") o.lpf.type = (strcmp(t.c_str(), "BW") == 0) ? FILTER_BW : FILTER_LR;
    }
    if(j.containsKey("slope")){
      uint8_t s = clampu8((int)j["slope"], 12, 36);
      switch(s){ case 12: o.lpf.slope=SLOPE_12; break; case 18: o.lpf.slope=SLOPE_18; break; case 24: o.lpf.slope=SLOPE_24; break; case 36: o.lpf.slope=SLOPE_36; break; default: o.lpf.slope=SLOPE_24; }
    }
    if(j.containsKey("freq"))  o.lpf.freq  = clampf((float)j["freq"], 10, 22000);
    if(j.containsKey("enabled")) o.lpf.enabled = (bool)j["enabled"];
  }
  if(d.containsKey("peq")){
    auto j=d["peq"];
    if(j.containsKey("f")) o.peq.f = clampf((float)j["f"], 10,22000);
    if(j.containsKey("g")) o.peq.g = clampf((float)j["g"], -12,+12);
    if(j.containsKey("q")) o.peq.q = clampf((float)j["q"], 0.4,10.0);
  }
  if(d.containsKey("delayMs")) o.delayMs = clampf((float)d["delayMs"], 0, 8.0);
  // Phase: support legacy "invert" or new "phaseDeg"
  if(d.containsKey("invert")) o.phaseDeg = (bool)d["invert"] ? 180.0f : 0.0f;
  if(d.containsKey("phaseDeg")) o.phaseDeg = clampf((float)d["phaseDeg"], 0, 180);
  if(d.containsKey("limiter")){
    auto j=d["limiter"];
    if(j.containsKey("thr"))  o.lim.threshold = clampf((float)j["thr"], -24, 0);
    if(j.containsKey("atk"))  o.lim.attack    = clampf((float)j["atk"], 0.1, 100);
    if(j.containsKey("rel"))  o.lim.release   = clampf((float)j["rel"], 1, 1600);
    if(j.containsKey("auto")) o.lim.autoRelease = (bool)j["auto"];
    if(j.containsKey("en"))   o.lim.enabled   = (bool)j["en"];
  }
  if(d.containsKey("gainDb")) o.gainDb = clampf((float)d["gainDb"], -45, +15);
  if(d.containsKey("mute"))   o.mute   = (bool)d["mute"];

  Serial.printf("[API] out%d updated\n", ch+1);
  ack(0,32,0);
  saveWorking();
  sendJson("{\"ok\":true}");
}

void handleGenerators(){
  if(!requireCodeIfLocked()) return;
  if(!server.hasArg("plain")){ sendJson("{\"ok\":false}", 400); return; }
  StaticJsonDocument<512> d; if(deserializeJson(d, server.arg("plain"))){ sendJson("{\"ok\":false}",400); return; }
  if(d.containsKey("sineEn")) dev.gen.sineEn = (bool)d["sineEn"];
  if(d.containsKey("sineHz")) dev.gen.sineHz = clampf((float)d["sineHz"], 10, 22000);
  if(d.containsKey("sineDb")) dev.gen.sineLevelDb = clampf((float)d["sineDb"], -60, 0);
  if(d.containsKey("sweepEn")) dev.gen.sweepEn = (bool)d["sweepEn"];
  if(d.containsKey("sweepStart")) dev.gen.sweepStart = clampf((float)d["sweepStart"],10,22000);
  if(d.containsKey("sweepEnd"))   dev.gen.sweepEnd   = clampf((float)d["sweepEnd"],10,22000);
  if(d.containsKey("sweepDb"))    dev.gen.sweepLevelDb = clampf((float)d["sweepDb"], -60, 0);
  if(d.containsKey("pinkEn")) dev.gen.pinkEn = (bool)d["pinkEn"];
  if(d.containsKey("pinkDb")) dev.gen.pinkLevelDb = clampf((float)d["pinkDb"], -60, 0);

  Serial.println("[API] generators updated");
  ack(0,32,0);
  saveWorking();
  sendJson("{\"ok\":true}");
}

void handleSequencer(){
  if(!requireCodeIfLocked()) return;
  if(!server.hasArg("plain")){ sendJson("{\"ok\":false}", 400); return; }
  StaticJsonDocument<256> d; if(deserializeJson(d, server.arg("plain"))){ sendJson("{\"ok\":false}",400); return; }
  if(d.containsKey("s1")) dev.seq.s1 = (bool)d["s1"];
  if(d.containsKey("s2")) dev.seq.s2 = (bool)d["s2"];
  if(d.containsKey("s3")) dev.seq.s3 = (bool)d["s3"];
  if(d.containsKey("intervalMs")) dev.seq.intervalMs = (uint32_t)d["intervalMs"];
  Serial.printf("[API] sequencer S1=%d S2=%d S3=%d int=%ums\n", dev.seq.s1, dev.seq.s2, dev.seq.s3, dev.seq.intervalMs);
  ack(0,32,0);
  saveWorking();
  sendJson("{\"ok\":true}");
}

void handleXoApply(){
  if(!requireCodeIfLocked()) return;
  if(!server.hasArg("plain")){ sendJson("{\"ok\":false}", 400); return; }
  StaticJsonDocument<512> d; if(deserializeJson(d, server.arg("plain"))){ sendJson("{\"ok\":false}",400); return; }

  int preset = (int)d["preset"];
  if(preset < 0 || preset >= 11){ sendJson("{\"ok\":false,\"err\":\"preset 0..10\"}",400); return; }

  JsonArray channels = d["channels"].as<JsonArray>();
  if(channels.isNull() || channels.size()==0){ sendJson("{\"ok\":false,\"err\":\"missing channels\"}",400); return; }

  const XoTemplate& tpl = XO_TEMPLATES[preset];

  for(JsonVariant v : channels){
    int ch = (int)v - 1;
    if(ch >= 0 && ch < 4){
      dev.out[ch].hpf.type = tpl.hpf.type;
      dev.out[ch].hpf.slope = tpl.hpf.slope;
      dev.out[ch].hpf.freq = tpl.hpf.freq;
      dev.out[ch].hpf.enabled = tpl.hpf.enabled;

      dev.out[ch].lpf.type = tpl.lpf.type;
      dev.out[ch].lpf.slope = tpl.lpf.slope;
      dev.out[ch].lpf.freq = tpl.lpf.freq;
      dev.out[ch].lpf.enabled = tpl.lpf.enabled;
    }
  }

  dev.xoPreset = preset;

  Serial.printf("[API] XO preset %d (%s) applied\n", preset, tpl.name);
  ack(0,32,0);
  saveWorking();

  // Response with applied template details
  StaticJsonDocument<512> resp;
  resp["ok"] = true;
  resp["applied"]["name"] = tpl.name;
  auto jhpf = resp["applied"].createNestedObject("hpf");
  jhpf["type"] = (tpl.hpf.type == FILTER_BW) ? "BW" : "LR";
  jhpf["slope"] = (uint8_t)tpl.hpf.slope;
  jhpf["freq"] = tpl.hpf.freq;
  jhpf["enabled"] = tpl.hpf.enabled;
  auto jlpf = resp["applied"].createNestedObject("lpf");
  jlpf["type"] = (tpl.lpf.type == FILTER_BW) ? "BW" : "LR";
  jlpf["slope"] = (uint8_t)tpl.lpf.slope;
  jlpf["freq"] = tpl.lpf.freq;
  jlpf["enabled"] = tpl.lpf.enabled;
  auto jchs = resp.createNestedArray("channels");
  for(JsonVariant v : channels) jchs.add((int)v);

  String s; serializeJson(resp, s);
  sendJson(s);
}

void handleBattery(){
  // display-only updates (can be posted by app or ADC task)
  if(!server.hasArg("plain")){ sendJson("{\"ok\":false}", 400); return; }
  StaticJsonDocument<128> d; if(deserializeJson(d, server.arg("plain"))){ sendJson("{\"ok\":false}",400); return; }
  if(d.containsKey("v")){
    dev.bat.v = (float)d["v"];
    dev.bat.vmin = min(dev.bat.vmin, dev.bat.v);
    dev.bat.vmax = max(dev.bat.vmax, dev.bat.v);
  }
  ack(0,32,0);
  sendJson("{\"ok\":true}");
}

void handleLock(){
  if(!server.hasArg("plain")){ sendJson("{\"ok\":false}", 400); return; }
  StaticJsonDocument<128> d; if(deserializeJson(d, server.arg("plain"))){ sendJson("{\"ok\":false}",400); return; }
  if(d.containsKey("setCode")){
    uint32_t code = (uint32_t)d["setCode"];
    if(code>999999){ sendJson("{\"ok\":false,\"err\":\"6 digits\"}",400); return; }
    dev.lockCode = code; dev.locked = true;
    prefs.putUInt("lockCode", dev.lockCode);
    Serial.printf("[API] lock code set: %06u\n", dev.lockCode);
  }
  if(d.containsKey("lock"))   dev.locked = (bool)d["lock"];
  if(d.containsKey("unlock")) {
    uint32_t code = (uint32_t)d["unlock"];
    if(code==dev.lockCode){ dev.locked=false; Serial.println("[API] unlocked"); }
    else { sendJson("{\"ok\":false,\"err\":\"bad code\"}",403); return; }
  }
  ack(32,32,0);
  saveWorking();
  sendJson("{\"ok\":true}");
}

void handlePresetSave(){
  if(!requireCodeIfLocked()) return;
  if(!server.hasArg("plain")){ sendJson("{\"ok\":false}", 400); return; }
  StaticJsonDocument<256> d; if(deserializeJson(d, server.arg("plain"))){ sendJson("{\"ok\":false}",400); return; }
  int slot = (int)d["slot"]; String name = (const char*)(d["name"] | "Preset");
  if(!savePresetSlot(slot, name)){ sendJson("{\"ok\":false}",400); return; }
  Serial.printf("[API] preset saved slot=%d name=%s\n", slot, name.c_str());
  ack(0,32,0);
  sendJson("{\"ok\":true}");
}

void handlePresetLoad(){
  if(!requireCodeIfLocked()) return;
  if(!server.hasArg("plain")){ sendJson("{\"ok\":false}", 400); return; }
  StaticJsonDocument<128> d; if(deserializeJson(d, server.arg("plain"))){ sendJson("{\"ok\":false}",400); return; }
  int slot = (int)d["slot"];
  if(!loadPresetSlot(slot)){ sendJson("{\"ok\":false}",400); return; }
  Serial.printf("[API] preset loaded slot=%d\n", slot);
  ack(0,32,0);
  saveWorking();
  sendJson("{\"ok\":true}");
}

void handlePresetCopy(){
  if(!requireCodeIfLocked()) return;
  if(!server.hasArg("plain")){ sendJson("{\"ok\":false}", 400); return; }
  StaticJsonDocument<128> d; if(deserializeJson(d, server.arg("plain"))){ sendJson("{\"ok\":false}",400); return; }
  int from = (int)d["from"]; int to = (int)d["to"];
  String key = "preset_" + String(from);
  String s = prefs.getString(key.c_str(), "");
  if(s.length()==0){ sendJson("{\"ok\":false,\"err\":\"empty src\"}",400); return; }
  String key2 = "preset_" + String(to);
  prefs.putString(key2.c_str(), s);
  Serial.printf("[API] preset copied %d -> %d\n", from, to);
  ack(0,32,0);
  sendJson("{\"ok\":true}");
}

// Basic CORS
void addCors(){
  server.sendHeader("Access-Control-Allow-Origin","*");
  server.sendHeader("Access-Control-Allow-Headers","Content-Type, code");
}

// =========================
//        SETUP/LOOP
// =========================

void setup(){
  Serial.begin(115200); delay(150);
  Serial.println("\n\n=== Console ESP32 Starting ===");

  px.begin(); px.setBrightness(24); px.show();
  Serial.println("NeoPixel initialized");

  prefs.begin(NVS_NS, false);
  Serial.println("NVS initialized");

  dev.lockCode = prefs.getUInt("lockCode", 0);
  Serial.println("Loading working state...");
  if(loadWorking()){
    Serial.println("Working state loaded from NVS");
  } else {
    Serial.println("No working state found, using defaults");
  }

  // Connect to strongest WiFi network
  Serial.println("Connecting to WiFi...");
  if(!connectToBestSSID()){
    Serial.println("WARNING: WiFi connection failed; HTTP API will not be reachable over network.");
  }

  // Routes
  server.on("/api/status", HTTP_GET, [](){ addCors(); handleStatus(); });
  server.on("/api/state",  HTTP_GET, [](){ addCors(); handleStateGet(); });

  server.on("/api/master", HTTP_POST, [](){ addCors(); handleMaster(); });
  server.on("/api/input/geq", HTTP_POST, [](){ addCors(); handleInputGeq(); });
  server.on("/api/input/peq", HTTP_POST, [](){ addCors(); handleInputPeq(); });
  server.on("/api/input/channel", HTTP_POST, [](){ addCors(); handleInputChannel(); });

  server.on("/api/output", HTTP_POST, [](){ addCors(); handleOutput(); });

  server.on("/api/gen", HTTP_POST, [](){ addCors(); handleGenerators(); });
  server.on("/api/seq", HTTP_POST, [](){ addCors(); handleSequencer(); });
  server.on("/api/xo/apply", HTTP_POST, [](){ addCors(); handleXoApply(); });
  server.on("/api/battery", HTTP_POST, [](){ addCors(); handleBattery(); });

  server.on("/api/lock", HTTP_POST, [](){ addCors(); handleLock(); });

  server.on("/api/preset/save", HTTP_POST, [](){ addCors(); handlePresetSave(); });
  server.on("/api/preset/load", HTTP_POST, [](){ addCors(); handlePresetLoad(); });
  server.on("/api/preset/copy", HTTP_POST, [](){ addCors(); handlePresetCopy(); });

  // Preflight
  auto opt204 = [](){ addCors(); server.send(204); };
  server.on("/api/master", HTTP_OPTIONS, opt204);
  server.on("/api/input/geq", HTTP_OPTIONS, opt204);
  server.on("/api/input/peq", HTTP_OPTIONS, opt204);
  server.on("/api/input/channel", HTTP_OPTIONS, opt204);
  server.on("/api/output", HTTP_OPTIONS, opt204);
  server.on("/api/gen", HTTP_OPTIONS, opt204);
  server.on("/api/seq", HTTP_OPTIONS, opt204);
  server.on("/api/xo/apply", HTTP_OPTIONS, opt204);
  server.on("/api/battery", HTTP_OPTIONS, opt204);
  server.on("/api/lock", HTTP_OPTIONS, opt204);
  server.on("/api/preset/save", HTTP_OPTIONS, opt204);
  server.on("/api/preset/load", HTTP_OPTIONS, opt204);
  server.on("/api/preset/copy", HTTP_OPTIONS, opt204);

  Serial.println("Starting HTTP server...");
  server.begin();
  Serial.println("=================================");
  Serial.println("HTTP server ready!");
  if(WiFi.status() == WL_CONNECTED){
    Serial.print("Server IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi not connected - server only accessible locally");
  }
  Serial.println("=================================");
}

void loop(){
  server.handleClient();

  uint32_t now = millis();

  // Smooth ramp master toward masterTarget
  if (masterTarget != dev.master) {
    float dt = (masterLastStepMs == 0) ? 0.0f : (now - masterLastStepMs) / 1000.0f;
    float step = MASTER_RATE_PER_SEC * dt;
    float diff = masterTarget - dev.master;
    float move = (fabsf(diff) <= step) ? diff : (diff > 0 ? step : -step);
    dev.master = clampf(dev.master + move, 0.0f, 1.0f);
    // Optional: debug the ramp every 200ms
    // static uint32_t lastLog=0; if(now - lastLog > 200){ Serial.printf("[RAMP] master=%.0f%% -> target=%.0f%%\n", dev.master*100, masterTarget*100); lastLog = now; }
  }
  masterLastStepMs = now;

  // Simple sequencer tick logger (no GPIO yet)
  if(dev.seq.s1||dev.seq.s2||dev.seq.s3){
    uint32_t interval = (dev.seq.intervalMs < 50) ? 50u : dev.seq.intervalMs;
    if(now - dev.seq.lastTick >= interval){
      dev.seq.lastTick = now;
      Serial.printf("[SEQ] tick S1=%d S2=%d S3=%d @%ums\n", dev.seq.s1, dev.seq.s2, dev.seq.s3, dev.seq.intervalMs);
    }
  }
}
