#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Preferences.h>

// ---------- MPU6050 ----------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------- A7670C / TinyGSM ----------
#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
//#define TINY_GSM_DEBUG Serial  // uncomment for very verbose AT logs

// ----------------- EXISTING PINS (from your program) -----------------
#define RAIN_AO_PIN 34
#define RAIN_DO_PIN 26
#define WATER_PIN   36

// ----------------- NEW pins (unused in your program) -----------------
#define WATER_LEVEL_PIN 39  // Resistive probe for bucket level (ADC1)
static const int MODEM_RX = 16; // ESP32 RX2  -> Modem TX
static const int MODEM_TX = 17; // ESP32 TX2  -> Modem RX

// Optional modem control pins (set -1 if not connected)
static const int MODEM_PWRKEY = -1;
static const int MODEM_RST    = -1;

// ----------------- Sampling params -----------------
static const uint8_t  BURST_SAMPLES  = 10;
static const uint16_t READ_PERIOD_MS = 500;

// ----------------- TCP upload settings -----------------
// IMPORTANT: Put your server IP/domain and TCP port here.
static const char* TCP_HOST = "example.com"; // e.g. "1.2.3.4" or "mydomain.com"
static const uint16_t TCP_PORT = 9000;       // your TCP listener port
static const uint32_t TCP_SEND_PERIOD_MS = 5000;
static uint32_t lastSendMs = 0;

// ----------------- Wi-Fi AP / Captive Portal -----------------
const char* AP_SSID = "rajput_dev";
const char* AP_PASS = "rajput111";
IPAddress apIP(192,168,4,1);
IPAddress netMsk(255,255,255,0);

WebServer server(80);
DNSServer dnsServer;

// ----------------- Preferences (persist manual calibration) -----------------
Preferences prefs;

// ----------------- Calibration -----------------
// HW-038 (water/leak)
uint16_t waterCalDry = 600;
uint16_t waterCalWet = 3600;

// MH-RD (raindrops panel)
uint16_t rainCalDry  = 800;
uint16_t rainCalWet  = 3500;

// Bucket level probe (resistive) calibration
uint16_t levelCalEmpty = 800;   // loaded from prefs if saved
uint16_t levelCalFull  = 3500;  // loaded from prefs if saved

// ----------------- Live values -----------------
volatile uint16_t g_water_raw = 0;
volatile uint8_t  g_water_pct = 0;
String g_water_status = "DRY";

volatile uint16_t g_rain_raw  = 0;
volatile uint8_t  g_rain_pct  = 0;
String g_rain_status  = "DRY";
volatile int      g_rain_do   = 0; // active flag (wet)
static int g_rain_do_prev = 0;
volatile uint32_t g_rain_events = 0;

volatile uint16_t g_level_raw = 0;
volatile uint8_t  g_level_pct = 0;
String g_level_status = "EMPTY";

// ----------------- MPU6050 -----------------
Adafruit_MPU6050 mpu;
bool mpuOk = false;

volatile float g_pitch_deg = 0.0f;
volatile float g_roll_deg  = 0.0f;
volatile float g_tilt_deg  = 0.0f;

// Manual tilt zero offsets (user calibration)
float tiltPitchZero = 0.0f;
float tiltRollZero  = 0.0f;

enum TiltState : uint8_t { TILT_IDLE=0, TILT_IN_PROGRESS=1 };
volatile TiltState g_tilt_state = TILT_IDLE;
volatile uint32_t g_tilt_count = 0;

static const float TILT_START_DEG = 25.0f;
static const float TILT_END_DEG   = 10.0f;
static const uint32_t TILT_MIN_HOLD_MS = 300;
static uint32_t tiltAboveStartSince = 0;
static bool wasAboveStart = false;

// ----------------- Cellular -----------------
HardwareSerial SerialAT(2);
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
static bool cellularReady = false;
static uint32_t lastCellStatusMs = 0;

// ----------------- Timestamp -----------------
uint32_t lastReadMs = 0;

// ----------------- Helpers -----------------
static uint8_t rawToPercent(uint16_t raw, uint16_t dry, uint16_t wet) {
  if (wet <= dry) return 0;
  long pct = (long)(raw - dry) * 100L / (long)(wet - dry);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (uint8_t)pct;
}

static const char* statusFromPct(uint8_t pct) {
  if (pct < 5)    return "DRY";
  if (pct < 20)   return "TRACE";
  if (pct < 50)   return "DAMP";
  if (pct < 80)   return "WET";
  return "FULL";
}

static const char* levelStatusFromPct(uint8_t pct) {
  if (pct < 5)    return "EMPTY";
  if (pct < 25)   return "LOW";
  if (pct < 75)   return "MID";
  if (pct < 95)   return "HIGH";
  return "FULL";
}

static uint16_t readRawBurst(uint8_t pin) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < BURST_SAMPLES; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  return (uint16_t)(sum / BURST_SAMPLES);
}

// ----------------- EXISTING WEB PAGE (UNCHANGED) -----------------
const char PAGE_INDEX[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width,initial-scale=1" />
<title>Water & Rain Dashboard</title>
<style>
  :root { --fg:#111; --muted:#666; --bg:#f7f7f7; }
  html,body{margin:0;padding:0;background:var(--bg);color:var(--fg);font-family:system-ui,Segoe UI,Roboto,Arial,sans-serif;}
  .wrap{max-width:980px;margin:24px auto;padding:16px;}
  h1{font-size:1.6rem;margin:0 0 8px;}
  h2{font-size:1.2rem;margin:0 0 10px;}
  .cards{display:grid;grid-template-columns:1fr 1fr;gap:16px;}
  .card{background:#fff;border-radius:14px;box-shadow:0 4px 14px rgba(0,0,0,.08);padding:16px 18px;}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;}
  .k{color:var(--muted);font-size:.9rem;}
  .v{font-weight:600;}
  .barWrap{margin-top:10px;background:#eee;border-radius:10px;overflow:hidden;height:18px}
  .bar{height:18px;background:linear-gradient(90deg,#35a,#49a,#2aa);width:0%;}
  .mono{font-family:ui-monospace,Consolas,Monaco,Menlo,monospace}
  .footer{color:#888;font-size:.85rem;margin-top:10px}
  @media (max-width:900px){.cards{grid-template-columns:1fr}}
</style>
</head>
<body>
  <div class="wrap">
    <h1>HW-038 (Water/Leak) & MH-RD (Rain) — Live Dashboard</h1>

    <div class="cards">
      <div class="card">
        <h2>HW-038 Water Sensor (GPIO36)</h2>
        <div class="grid">
          <div><div class="k">Raw</div><div id="w_raw" class="v mono">—</div></div>
          <div><div class="k">Percent</div><div id="w_pct" class="v">— %</div></div>
          <div><div class="k">Status</div><div id="w_status" class="v">—</div></div>
          <div><div class="k">Cal (DRY / WET)</div><div id="w_cal" class="v mono">— / —</div></div>
        </div>
        <div class="barWrap" aria-label="water-level">
          <div id="w_bar" class="bar"></div>
        </div>
        <div class="footer">Updates every 500 ms</div>
      </div>

      <div class="card">
        <h2>MH-RD Raindrops (AO: GPIO34, DO: GPIO26)</h2>
        <div class="grid">
          <div><div class="k">Raw (AO)</div><div id="r_raw" class="v mono">—</div></div>
          <div><div class="k">Percent</div><div id="r_pct" class="v">— %</div></div>
          <div><div class="k">Status</div><div id="r_status" class="v">—</div></div>
          <div><div class="k">Cal (DRY / WET)</div><div id="r_cal" class="v mono">— / —</div></div>
          <div><div class="k">Digital (DO)</div><div id="r_do" class="v mono">—</div></div>
        </div>
        <div class="barWrap" aria-label="rain-level">
          <div id="r_bar" class="bar"></div>
        </div>
        <div class="footer">DO is usually ACTIVE LOW on wet (LM393)</div>
      </div>
    </div>

    <div class="card" style="margin-top:16px">
      <div class="k">Last update</div>
      <div id="ts" class="v mono">—</div>
    </div>
  </div>

<script>
async function refresh() {
  try {
    const r = await fetch('/data', {cache:'no-store'});
    if (!r.ok) return;
    const j = await r.json();

    document.getElementById('w_raw').textContent = j.water.raw;
    document.getElementById('w_pct').textContent = j.water.pct + ' %';
    document.getElementById('w_status').textContent = j.water.status;
    document.getElementById('w_cal').textContent = j.water.calDry + ' / ' + j.water.calWet;
    document.getElementById('w_bar').style.width = j.water.pct + '%';

    document.getElementById('r_raw').textContent = j.rain.raw;
    document.getElementById('r_pct').textContent = j.rain.pct + ' %';
    document.getElementById('r_status').textContent = j.rain.status;
    document.getElementById('r_cal').textContent = j.rain.calDry + ' / ' + j.rain.calWet;
    document.getElementById('r_bar').style.width = j.rain.pct + '%';
    document.getElementById('r_do').textContent = (j.rain.doActive) ? 'RAIN (DO active)' : 'NO RAIN (DO inactive)';

    document.getElementById('ts').textContent = new Date(j.ts).toLocaleString();
  } catch(e) {}
}
setInterval(refresh, 500);
refresh();
</script>
</body>
</html>
)HTML";

// ----------------- JSON endpoints -----------------
void handleRoot() {
  server.send(200, "text/html", PAGE_INDEX);
}

void handleData() {
  String j;
  j.reserve(900);

  j += F("{\"water\":{");
  j += F("\"raw\":");    j += g_water_raw;
  j += F(",\"pct\":");   j += g_water_pct;
  j += F(",\"status\":\""); j += g_water_status; j += F("\"");
  j += F(",\"calDry\":"); j += waterCalDry;
  j += F(",\"calWet\":"); j += waterCalWet;
  j += F("},\"rain\":{");
  j += F("\"raw\":");    j += g_rain_raw;
  j += F(",\"pct\":");   j += g_rain_pct;
  j += F(",\"status\":\""); j += g_rain_status; j += F("\"");
  j += F(",\"calDry\":"); j += rainCalDry;
  j += F(",\"calWet\":"); j += rainCalWet;
  j += F(",\"doActive\":"); j += (g_rain_do ? "true" : "false");
  j += F("}");

  // extra fields (main dashboard ignores them)
  j += F(",\"bucket\":{");
  j += F("\"levelRaw\":"); j += g_level_raw;
  j += F(",\"levelPct\":"); j += g_level_pct;
  j += F(",\"levelStatus\":\""); j += g_level_status; j += F("\"");
  j += F(",\"calEmpty\":"); j += levelCalEmpty;
  j += F(",\"calFull\":");  j += levelCalFull;
  j += F(",\"tiltDeg\":"); j += String(g_tilt_deg, 1);
  j += F(",\"pitchDeg\":"); j += String(g_pitch_deg, 1);
  j += F(",\"rollDeg\":");  j += String(g_roll_deg, 1);
  j += F(",\"tiltCount\":"); j += g_tilt_count;
  j += F("}");

  j += F(",\"events\":{");
  j += F("\"rainFlow\":"); j += g_rain_events;
  j += F("}");

  j += F(",\"ts\":"); j += (uint32_t)millis();
  j += F("}");

  server.send(200, "application/json", j);
}

void handleStatus() {
  String j;
  j.reserve(700);

  j += F("{\"wifi\":{");
  j += F("\"apSsid\":\""); j += AP_SSID; j += F("\"");
  j += F(",\"apIp\":\""); j += apIP.toString(); j += F("\"");
  j += F(",\"clients\":"); j += WiFi.softAPgetStationNum();
  j += F("},");

  j += F("\"cell\":{");
  j += F("\"ready\":"); j += (cellularReady ? "true":"false");
  j += F(",\"net\":"); j += (modem.isNetworkConnected() ? "true":"false");
  j += F(",\"gprs\":"); j += (modem.isGprsConnected() ? "true":"false");
  j += F(",\"rssi\":"); j += modem.getSignalQuality();
  j += F(",\"ip\":\""); j += modem.localIP().toString(); j += F("\"");
  j += F("},");

  j += F("\"cal\":{");
  j += F("\"lvlEmpty\":"); j += levelCalEmpty;
  j += F(",\"lvlFull\":");  j += levelCalFull;
  j += F(",\"pitchZero\":"); j += String(tiltPitchZero, 2);
  j += F(",\"rollZero\":");  j += String(tiltRollZero, 2);
  j += F("},");

  j += F("\"uptimeMs\":"); j += (uint32_t)millis();
  j += F("}");

  server.send(200, "application/json", j);
}

// ----------------- Live Calibration page (auto refresh) -----------------
void handleCalPage() {
  const char CAL_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>Recharge Pod Calibration</title>
  <style>
    body{font-family:system-ui,Arial;margin:16px;background:#f7f7f7;color:#111}
    .row{display:grid;grid-template-columns:1fr;gap:12px;max-width:900px}
    .card{background:#fff;padding:14px;border-radius:12px;box-shadow:0 4px 14px rgba(0,0,0,.08)}
    .mono{font-family:ui-monospace,Consolas,monospace}
    button,a.btn{display:inline-block;padding:10px 14px;border:0;border-radius:10px;background:#111;color:#fff;font-weight:700;text-decoration:none}
    .muted{color:#666}
    .grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
    @media(min-width:900px){.row{grid-template-columns:1fr 1fr}}
  </style>
</head>
<body>
  <h2>Recharge Pod — Manual Calibration (Live)</h2>
  <p class="muted">This page does not change your main dashboard. It only helps calibration. Auto-updates every 500 ms.</p>

  <div class="row">
    <div class="card">
      <h3>Bucket Level Probe (GPIO39)</h3>
      <div class="grid">
        <div><div class="muted">Raw (live)</div><div id="lvl_raw" class="mono">—</div></div>
        <div><div class="muted">Percent (live)</div><div id="lvl_pct" class="mono">—</div></div>
        <div><div class="muted">Status</div><div id="lvl_st" class="mono">—</div></div>
        <div><div class="muted">Cal Empty / Full</div><div id="lvl_cal" class="mono">—</div></div>
      </div>
      <p><a class="btn" href="/cal/level/empty">Set EMPTY = current</a></p>
      <p><a class="btn" href="/cal/level/full">Set FULL = current</a></p>
    </div>

    <div class="card">
      <h3>Bucket Tilt (MPU6050)</h3>
      <div class="grid">
        <div><div class="muted">Pitch (live)</div><div id="p" class="mono">—</div></div>
        <div><div class="muted">Roll (live)</div><div id="r" class="mono">—</div></div>
        <div><div class="muted">Tilt (live)</div><div id="t" class="mono">—</div></div>
        <div><div class="muted">Tilt Count</div><div id="tc" class="mono">—</div></div>
        <div><div class="muted">Zero Pitch/Roll</div><div id="z" class="mono">—</div></div>
      </div>
      <p><a class="btn" href="/cal/tilt/zero">Set ZERO = current</a></p>
    </div>
  </div>

  <div class="card" style="max-width:900px;margin-top:12px">
    <h3>Save / Reset</h3>
    <p><a class="btn" href="/cal/save">Save calibration to ESP32</a></p>
    <p><a class="btn" href="/cal/reset">Reset saved calibration</a></p>
    <p class="mono">Quick links: <a href="/status">/status</a> | <a href="/data">/data</a> | <a href="/">dashboard</a></p>
    <p class="mono muted" id="net">—</p>
  </div>

<script>
async function tick(){
  try{
    const [d, s] = await Promise.all([
      fetch('/data',{cache:'no-store'}).then(r=>r.json()),
      fetch('/status',{cache:'no-store'}).then(r=>r.json())
    ]);

    const b = d.bucket;

    document.getElementById('lvl_raw').textContent = b.levelRaw;
    document.getElementById('lvl_pct').textContent = b.levelPct + ' %';
    document.getElementById('lvl_st').textContent  = b.levelStatus;
    document.getElementById('lvl_cal').textContent = b.calEmpty + ' / ' + b.calFull;

    document.getElementById('p').textContent  = b.pitchDeg.toFixed ? b.pitchDeg.toFixed(1) : b.pitchDeg;
    document.getElementById('r').textContent  = b.rollDeg.toFixed ? b.rollDeg.toFixed(1) : b.rollDeg;
    document.getElementById('t').textContent  = b.tiltDeg.toFixed ? b.tiltDeg.toFixed(1) : b.tiltDeg;
    document.getElementById('tc').textContent = b.tiltCount;

    document.getElementById('z').textContent = s.cal.pitchZero + ' / ' + s.cal.rollZero;

    const cell = s.cell;
    document.getElementById('net').textContent =
      `CELL ready=${cell.ready} net=${cell.net} gprs=${cell.gprs} rssi=${cell.rssi} ip=${cell.ip}`;
  }catch(e){}
}
setInterval(tick, 500);
tick();
</script>
</body>
</html>
)HTML";

  server.send(200, "text/html", CAL_PAGE);
}

// ----------------- Calibration actions -----------------
void handleCalLevelEmpty() {
  levelCalEmpty = g_level_raw;
  Serial.print(F("[CAL] Set level EMPTY = ")); Serial.println(levelCalEmpty);
  server.sendHeader("Location", "/cal");
  server.send(302, "text/plain", "");
}

void handleCalLevelFull() {
  levelCalFull = g_level_raw;
  Serial.print(F("[CAL] Set level FULL = ")); Serial.println(levelCalFull);
  server.sendHeader("Location", "/cal");
  server.send(302, "text/plain", "");
}

void handleCalTiltZero() {
  // current g_pitch_deg/g_roll_deg already have existing zero applied,
  // so we compute new zero by adding them back (simpler method: store current *raw* angles).
  // For simplicity, we treat displayed angles as "current offset needed to zero".
  tiltPitchZero += g_pitch_deg;
  tiltRollZero  += g_roll_deg;

  Serial.print(F("[CAL] Set tilt ZERO updated => pitchZero="));
  Serial.print(tiltPitchZero, 2);
  Serial.print(F(" rollZero="));
  Serial.println(tiltRollZero, 2);

  server.sendHeader("Location", "/cal");
  server.send(302, "text/plain", "");
}

void handleCalSave() {
  prefs.begin("rechargepod", false);
  prefs.putUShort("lvlEmpty", levelCalEmpty);
  prefs.putUShort("lvlFull",  levelCalFull);
  prefs.putFloat("pZero", tiltPitchZero);
  prefs.putFloat("rZero", tiltRollZero);
  prefs.end();

  Serial.println(F("[CAL] Saved calibration to NVS"));
  server.sendHeader("Location", "/cal");
  server.send(302, "text/plain", "");
}

void handleCalReset() {
  prefs.begin("rechargepod", false);
  prefs.clear();
  prefs.end();

  // revert to defaults in RAM too (optional)
  levelCalEmpty = 800;
  levelCalFull  = 3500;
  tiltPitchZero = 0.0f;
  tiltRollZero  = 0.0f;

  Serial.println(F("[CAL] Cleared saved calibration (NVS) and reset RAM defaults"));
  server.sendHeader("Location", "/cal");
  server.send(302, "text/plain", "");
}

// Captive portal redirect
void handleNotFound() {
  server.sendHeader("Location", String("http://") + apIP.toString() + "/");
  server.send(302, "text/plain", "");
}

// ----------------- Modem helpers -----------------
static void modemPowerCycleIfWired() {
  if (MODEM_RST >= 0) {
    pinMode(MODEM_RST, OUTPUT);
    digitalWrite(MODEM_RST, LOW);
    delay(200);
    digitalWrite(MODEM_RST, HIGH);
    delay(800);
  }
  if (MODEM_PWRKEY >= 0) {
    pinMode(MODEM_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(1100);
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(2000);
  }
}

static bool setupCellular() {
  Serial.println(F("\n=== CELLULAR SETUP (A7670C / Jio SIM) ==="));

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(300);

  modemPowerCycleIfWired();

  Serial.println(F("[CELL] Restarting modem..."));
  if (!modem.restart()) {
    Serial.println(F("[CELL] modem.restart() failed (trying init)"));
    if (!modem.init()) {
      Serial.println(F("[CELL] modem.init() failed"));
      return false;
    }
  }

  Serial.print(F("[CELL] Modem Info: "));
  Serial.println(modem.getModemInfo());

  Serial.println(F("[CELL] Waiting for network..."));
  if (!modem.waitForNetwork(60000L)) {
    Serial.println(F("[CELL] Network not found"));
    return false;
  }
  Serial.println(F("[CELL] Network connected ✅"));

  // Jio APN fallback approach
  const char* apn  = "jionet";
  const char* user = "";
  const char* pass = "";

  Serial.print(F("[CELL] Connecting GPRS with APN: "));
  Serial.println(apn);

  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println(F("[CELL] GPRS failed on 'jionet'. Trying 'internet'..."));
    apn = "internet";
    if (!modem.gprsConnect(apn, user, pass)) {
      Serial.println(F("[CELL] GPRS connect failed (both APNs)."));
      return false;
    }
  }

  Serial.println(F("[CELL] GPRS connected ✅"));
  Serial.print(F("[CELL] IP: "));
  Serial.println(modem.localIP());

  return true;
}

static void printCellularStatusOccasionally() {
  if (millis() - lastCellStatusMs < 10000) return;
  lastCellStatusMs = millis();

  int16_t rssi = modem.getSignalQuality();
  Serial.print(F("[CELL] RSSI: ")); Serial.println(rssi);
  Serial.print(F("[CELL] Network: ")); Serial.println(modem.isNetworkConnected() ? "OK" : "DOWN");
  Serial.print(F("[CELL] GPRS: ")); Serial.println(modem.isGprsConnected() ? "OK" : "DOWN");
}

// ----------------- MPU tilt logic -----------------
static void updateMpuTilt() {
  if (!mpuOk) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float pitchRaw = atan2f(ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
  float rollRaw  = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / PI;

  // Apply manual zero offsets
  float pitch = pitchRaw - tiltPitchZero;
  float roll  = rollRaw  - tiltRollZero;

  g_pitch_deg = pitch;
  g_roll_deg  = roll;

  float tilt = max(fabsf(pitch), fabsf(roll));
  g_tilt_deg = tilt;

  // Tilt event state machine
  if (tilt >= TILT_START_DEG) {
    if (!wasAboveStart) {
      wasAboveStart = true;
      tiltAboveStartSince = millis();
    } else {
      if (g_tilt_state == TILT_IDLE && (millis() - tiltAboveStartSince >= TILT_MIN_HOLD_MS)) {
        g_tilt_state = TILT_IN_PROGRESS;
        Serial.println(F("[TILT] Bucket tilt started"));
      }
    }
  } else {
    wasAboveStart = false;
    tiltAboveStartSince = 0;
  }

  if (g_tilt_state == TILT_IN_PROGRESS && tilt <= TILT_END_DEG) {
    g_tilt_count++;
    g_tilt_state = TILT_IDLE;
    Serial.print(F("[TILT] Bucket tilt COMPLETED. Count="));
    Serial.println(g_tilt_count);
  }
}

// ----------------- TCP JSON send (RAW TCP PACKET) -----------------
static void sendJsonOverTcpIfReady() {
  if (!cellularReady) return;
  if (!modem.isGprsConnected()) return;
  if (millis() - lastSendMs < TCP_SEND_PERIOD_MS) return;
  lastSendMs = millis();

  // Build one JSON line (newline-delimited framing)
  String payload;
  payload.reserve(520);

  payload += F("{\"ts\":"); payload += (uint32_t)millis();

  payload += F(",\"water\":{\"raw\":"); payload += g_water_raw;
  payload += F(",\"pct\":"); payload += g_water_pct;
  payload += F(",\"status\":\""); payload += g_water_status; payload += F("\"}");

  payload += F(",\"rain\":{\"raw\":"); payload += g_rain_raw;
  payload += F(",\"pct\":"); payload += g_rain_pct;
  payload += F(",\"doActive\":"); payload += (g_rain_do ? "true":"false");
  payload += F(",\"events\":"); payload += g_rain_events;
  payload += F("}");

  payload += F(",\"bucket\":{\"levelRaw\":"); payload += g_level_raw;
  payload += F(",\"levelPct\":"); payload += g_level_pct;
  payload += F(",\"tiltDeg\":"); payload += String(g_tilt_deg, 1);
  payload += F(",\"pitchDeg\":"); payload += String(g_pitch_deg, 1);
  payload += F(",\"rollDeg\":");  payload += String(g_roll_deg, 1);
  payload += F(",\"tiltCount\":"); payload += g_tilt_count;
  payload += F("}");

  payload += F("}\n"); // newline terminator

  Serial.print(F("[TCP] Connecting "));
  Serial.print(TCP_HOST);
  Serial.print(F(":"));
  Serial.println(TCP_PORT);

  gsmClient.setTimeout(8000);

  if (!gsmClient.connect(TCP_HOST, TCP_PORT)) {
    Serial.println(F("[TCP] Connect failed"));
    return;
  }

  size_t written = gsmClient.print(payload);
  gsmClient.flush();
  delay(50);

  Serial.print(F("[TCP] Sent bytes="));
  Serial.print(written);
  Serial.print(F(" | jsonLen="));
  Serial.println(payload.length());

  // If your server responds (optional), read a little for debug
  uint32_t t0 = millis();
  while (gsmClient.connected() && millis() - t0 < 800) {
    while (gsmClient.available()) {
      String line = gsmClient.readStringUntil('\n');
      if (line.length()) {
        Serial.print(F("[TCP] RX: "));
        Serial.println(line);
      }
    }
    delay(10);
  }

  gsmClient.stop();
  Serial.println(F("[TCP] Disconnected"));
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(WATER_PIN,       INPUT);
  pinMode(RAIN_AO_PIN,     INPUT);
  pinMode(WATER_LEVEL_PIN, INPUT);
  pinMode(RAIN_DO_PIN,     INPUT_PULLUP);
  analogReadResolution(12);

  // Load saved calibration (if any)
  prefs.begin("rechargepod", true);
  levelCalEmpty = prefs.getUShort("lvlEmpty", levelCalEmpty);
  levelCalFull  = prefs.getUShort("lvlFull",  levelCalFull);
  tiltPitchZero = prefs.getFloat("pZero", tiltPitchZero);
  tiltRollZero  = prefs.getFloat("rZero", tiltRollZero);
  prefs.end();

  Serial.println(F("\n=== Loaded Calibration ==="));
  Serial.print(F("[CAL] levelEmpty=")); Serial.print(levelCalEmpty);
  Serial.print(F(" levelFull=")); Serial.println(levelCalFull);
  Serial.print(F("[CAL] pitchZero=")); Serial.print(tiltPitchZero,2);
  Serial.print(F(" rollZero=")); Serial.println(tiltRollZero,2);

  // MPU init
  Wire.begin(); // SDA 21, SCL 22
  Serial.println(F("\n=== MPU6050 init ==="));
  mpuOk = mpu.begin();
  if (!mpuOk) {
    Serial.println(F("[MPU] MPU6050 not found. Check SDA/SCL & power."));
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println(F("[MPU] MPU6050 OK ✅"));
  }

  // AP + captive portal
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP(AP_SSID, AP_PASS);

  dnsServer.start(53, "*", apIP);

  // Routes: keep / and /data unchanged, add /status + /cal + calibration actions
  server.on("/", HTTP_GET, handleRoot);
  server.on("/index.html", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);

  server.on("/status", HTTP_GET, handleStatus);
  server.on("/cal", HTTP_GET, handleCalPage);

  server.on("/cal/level/empty", HTTP_GET, handleCalLevelEmpty);
  server.on("/cal/level/full",  HTTP_GET, handleCalLevelFull);
  server.on("/cal/tilt/zero",   HTTP_GET, handleCalTiltZero);
  server.on("/cal/save",        HTTP_GET, handleCalSave);
  server.on("/cal/reset",       HTTP_GET, handleCalReset);

  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println(F("\n=== ESP32 Recharge Pod Dashboard (AP) ==="));
  Serial.print(F("Dashboard:   http://")); Serial.print(apIP); Serial.println(F("/"));
  Serial.print(F("Calibration: http://")); Serial.print(apIP); Serial.println(F("/cal"));
  Serial.print(F("Status JSON: http://")); Serial.print(apIP); Serial.println(F("/status"));

  // Cellular setup
  cellularReady = setupCellular();
  if (!cellularReady) {
    Serial.println(F("[CELL] Cellular not ready. AP dashboard will still work."));
  }
}

// ----------------- Loop -----------------
void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  if (millis() - lastReadMs >= READ_PERIOD_MS) {
    lastReadMs = millis();

    // HW-038
    uint16_t w_raw = readRawBurst(WATER_PIN);
    uint8_t  w_pct = rawToPercent(w_raw, waterCalDry, waterCalWet);
    g_water_raw = w_raw;
    g_water_pct = w_pct;
    g_water_status = statusFromPct(w_pct);

    // Rain AO
    uint16_t r_raw = readRawBurst(RAIN_AO_PIN);
    uint8_t  r_pct = rawToPercent(r_raw, rainCalDry, rainCalWet);
    g_rain_raw = r_raw;
    g_rain_pct = r_pct;
    g_rain_status = statusFromPct(r_pct);

    // Rain DO active low
    int doVal = digitalRead(RAIN_DO_PIN);
    int doActive = (doVal == LOW) ? 1 : 0;
    g_rain_do = doActive;

    // Flow event on rising edge
    if (doActive == 1 && g_rain_do_prev == 0) {
      g_rain_events++;
      Serial.print(F("[RAIN] Flow event detected. Count="));
      Serial.println(g_rain_events);
    }
    g_rain_do_prev = doActive;

    // Bucket level probe
    uint16_t lvl_raw = readRawBurst(WATER_LEVEL_PIN);
    uint8_t  lvl_pct = rawToPercent(lvl_raw, levelCalEmpty, levelCalFull);
    g_level_raw = lvl_raw;
    g_level_pct = lvl_pct;
    g_level_status = levelStatusFromPct(lvl_pct);

    // MPU tilt
    updateMpuTilt();

    // Console debug (requested)
    Serial.print(F("WATER Raw=")); Serial.print(g_water_raw);
    Serial.print(F(" Pct=")); Serial.print(g_water_pct);
    Serial.print(F("% ")); Serial.print(g_water_status);

    Serial.print(F(" | RAIN Raw=")); Serial.print(g_rain_raw);
    Serial.print(F(" Pct=")); Serial.print(g_rain_pct);
    Serial.print(F("% DO=")); Serial.print(g_rain_do ? "ACTIVE" : "INACTIVE");
    Serial.print(F(" Events=")); Serial.print(g_rain_events);

    Serial.print(F(" | LEVEL Raw=")); Serial.print(g_level_raw);
    Serial.print(F(" Pct=")); Serial.print(g_level_pct);
    Serial.print(F("% ")); Serial.print(g_level_status);

    Serial.print(F(" | TILT=")); Serial.print(g_tilt_deg, 1);
    Serial.print(F("deg (P=")); Serial.print(g_pitch_deg, 1);
    Serial.print(F(" R=")); Serial.print(g_roll_deg, 1);
    Serial.print(F(") Count=")); Serial.println(g_tilt_count);
  }

  if (cellularReady) {
    printCellularStatusOccasionally();
    sendJsonOverTcpIfReady();
  } else {
    // auto-retry every 30s
    static uint32_t lastRetry = 0;
    if (millis() - lastRetry > 30000) {
      lastRetry = millis();
      Serial.println(F("[CELL] Retrying cellular setup..."));
      cellularReady = setupCellular();
    }
  }
}