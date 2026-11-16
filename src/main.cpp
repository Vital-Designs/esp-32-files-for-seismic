/*
  ESP32 + MPU6050 -> dumb sensor for Quake Resource Planner (with setup page)

  Wiring: MPU6050 VCC->3V3, GND->GND, SDA->GPIO21 (D21), SCL->GPIO22 (D22)
  Optional active buzzer: + to GPIO25, - to GND. Onboard LED (GPIO2) also flashes on alert.

  SETUP WITHOUT RE-FLASHING
  - First boot (or when the saved WiFi isn't found) the board makes its own WiFi: "QuakeSensor-Setup".
    Join it from your phone; a page opens (or go to 192.168.4.1). Pick WiFi + password, and fill in
    server host, port, SSL and device token. It saves everything and reboots.
  - To change settings later: power the board, and while the LED blinks fast (first 4 seconds)
    press and hold the BOOT button until the LED stays solid. The setup page opens again.
    (Do NOT hold BOOT while pressing reset/plugging in - that starts flashing mode instead.)
*/
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <WebSocketsClient.h>
#include <WiFiManager.h>
#include <Preferences.h>

// ---- defaults shown on the setup page (change them there; no need to edit here) ----
#define DEF_HOST  "192.168.1.23"   // laptop IP on the same network, or e.g. my-app.onrender.com
#define DEF_PORT  "3000"           // 3000 for the laptop server, 443 for a hosted https site
#define DEF_SSL   "0"              // 0 = ws:// (laptop), 1 = wss:// (hosted https)
#define DEF_TOKEN "change-me"      // must match DEVICE_TOKEN on the server

const int BUZZER_PIN = 25;   // set to -1 if you have no buzzer
const int LED_PIN    = 2;
const int BOOT_PIN   = 0;    // the BOOT button
// ------------------------------------------------------------------------------------

const uint8_t MPU_ADDR = 0x68;           // try 0x69 if the sensor isn't found
const float   LSB_PER_G = 16384.0f;      // +/-2 g range
const uint32_t SAMPLE_US = 10000;        // 100 Hz
const int BATCH = 100;                   // ~1 second per message

WebSocketsClient ws;
Preferences prefs;
String gHost, gPort, gSsl, gToken, gPath;
bool shouldSave = false;

float bx[BATCH], by[BATCH], bz[BATCH];
int n = 0;
uint32_t nextSample = 0;
uint32_t alertUntil = 0;
bool connected = false;
char out[BATCH * 28 + 16];

bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuInit() {
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  mpuWrite(0x6B, 0x00);   // wake up
  delay(100);
  mpuWrite(0x1C, 0x00);   // +/-2 g
  mpuWrite(0x1A, 0x03);   // low-pass ~44 Hz
  return true;
}

bool readAccel(float &x, float &y, float &z) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 6) != 6) return false;
  int16_t rx = (Wire.read() << 8) | Wire.read();
  int16_t ry = (Wire.read() << 8) | Wire.read();
  int16_t rz = (Wire.read() << 8) | Wire.read();
  x = rx / LSB_PER_G; y = ry / LSB_PER_G; z = rz / LSB_PER_G;
  return true;
}

void startAlert(uint32_t ms) { alertUntil = millis() + ms; }

int failCount = 0;          // consecutive failed sensor reads
bool sensorOk = true;       // false = sensor lost, trying to recover
uint32_t lastRetry = 0;

void alertTick() {
  if (!sensorOk) {          // slow blink (1 s on / 1 s off) = sensor lost
    digitalWrite(LED_PIN, ((millis() / 1000) % 2) ? HIGH : LOW);
    if (BUZZER_PIN >= 0) digitalWrite(BUZZER_PIN, LOW);
    return;
  }
  bool on = (int32_t)(alertUntil - millis()) > 0;
  bool blink = on && ((millis() / 150) % 2 == 0);
  digitalWrite(LED_PIN, blink ? HIGH : LOW);
  if (BUZZER_PIN >= 0) digitalWrite(BUZZER_PIN, blink ? HIGH : LOW);
}

void onEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      connected = true; n = 0;
      Serial.println("Server connected");
      break;
    case WStype_DISCONNECTED:
      connected = false;
      Serial.println("Server disconnected (will retry)");
      break;
    case WStype_TEXT: {
      String s = String((char*)payload).substring(0, length);   // {"cmd":"alert","ms":5000}
      if (s.indexOf("\"alert\"") >= 0) {
        int i = s.indexOf("\"ms\"");
        uint32_t ms = 3000;
        if (i >= 0) { int c = s.indexOf(':', i); if (c >= 0) ms = (uint32_t)s.substring(c + 1).toInt(); }
        if (ms < 200 || ms > 30000) ms = 3000;
        Serial.printf("ALERT for %u ms\n", ms);
        startAlert(ms);
      }
      break;
    }
    default: break;
  }
}

void sendBatch() {
  int len = 0;
  len += snprintf(out + len, sizeof(out) - len, "{\"s\":[");
  for (int i = 0; i < n; i++) {
    len += snprintf(out + len, sizeof(out) - len, "%s[%.4f,%.4f,%.4f]", i ? "," : "", bx[i], by[i], bz[i]);
  }
  len += snprintf(out + len, sizeof(out) - len, "]}");
  ws.sendTXT((uint8_t*)out, len);
  n = 0;
}

void loadConfig() {
  prefs.begin("quake", true);
  gHost  = prefs.getString("host",  DEF_HOST);
  gPort  = prefs.getString("port",  DEF_PORT);
  gSsl   = prefs.getString("ssl",   DEF_SSL);
  gToken = prefs.getString("token", DEF_TOKEN);
  prefs.end();
}

void saveConfig() {
  prefs.begin("quake", false);
  prefs.putString("host",  gHost);
  prefs.putString("port",  gPort);
  prefs.putString("ssl",   gSsl);
  prefs.putString("token", gToken);
  prefs.end();
}

// Wait ~4 s with a fast-blinking LED. Returns true if BOOT is held (user wants the setup page).
bool wantSetupPage() {
  pinMode(BOOT_PIN, INPUT_PULLUP);
  uint32_t t0 = millis();
  while (millis() - t0 < 4000) {
    digitalWrite(LED_PIN, ((millis() / 100) % 2) ? HIGH : LOW);
    if (digitalRead(BOOT_PIN) == LOW) {
      delay(300);
      if (digitalRead(BOOT_PIN) == LOW) { digitalWrite(LED_PIN, HIGH); return true; }
    }
    delay(10);
  }
  digitalWrite(LED_PIN, LOW);
  return false;
}

void connectWifi() {
  loadConfig();

  WiFiManagerParameter pHost ("host",  "Server host or IP",              gHost.c_str(),  60);
  WiFiManagerParameter pPort ("port",  "Server port (3000 laptop, 443 hosted)", gPort.c_str(), 6);
  WiFiManagerParameter pSsl  ("ssl",   "Use SSL? 1 = yes (hosted https), 0 = no (laptop)", gSsl.c_str(), 2);
  WiFiManagerParameter pToken("token", "Device token (same as on the server)", gToken.c_str(), 60);

  WiFiManager wm;
  wm.addParameter(&pHost);
  wm.addParameter(&pPort);
  wm.addParameter(&pSsl);
  wm.addParameter(&pToken);
  wm.setSaveConfigCallback([]() { shouldSave = true; });
  wm.setAPCallback([](WiFiManager*) { digitalWrite(LED_PIN, HIGH); });   // LED solid = setup page is open
  wm.setConfigPortalTimeout(180);
  wm.setConnectTimeout(20);
  wm.setBreakAfterConfig(true);

  bool forced = wantSetupPage();
  bool ok;
  if (forced) {
    Serial.println("Opening setup page: join WiFi 'QuakeSensor-Setup', then open 192.168.4.1");
    ok = wm.startConfigPortal("QuakeSensor-Setup");
  } else {
    Serial.println("Connecting to saved WiFi (or opening setup page if none)...");
    ok = wm.autoConnect("QuakeSensor-Setup");
  }
  if (!ok) { Serial.println("No WiFi. Restarting..."); delay(500); ESP.restart(); }

  if (shouldSave) {
    gHost  = pHost.getValue();
    gPort  = pPort.getValue();
    gSsl   = pSsl.getValue();
    gToken = pToken.getValue();
    gHost.trim(); gPort.trim(); gSsl.trim(); gToken.trim();
    saveConfig();
    Serial.println("Settings saved");
  }
  digitalWrite(LED_PIN, LOW);
  Serial.print("WiFi OK, IP "); Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(LED_PIN, OUTPUT);
  if (BUZZER_PIN >= 0) pinMode(BUZZER_PIN, OUTPUT);

  if (!mpuInit()) {
    Serial.println("MPU6050 NOT FOUND. Check wiring: SDA->D21, SCL->D22, VCC->3V3, GND->GND. Try address 0x69.");
    while (true) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(120); }
  }
  Serial.println("MPU6050 OK");

  connectWifi();
  WiFi.setSleep(false);

  gPath = String("/esp?token=") + gToken;
  int port = gPort.toInt(); if (port <= 0) port = 3000;
  Serial.printf("Server: %s:%d  ssl=%s\n", gHost.c_str(), port, gSsl.c_str());
  if (gSsl == "1") ws.beginSSL(gHost.c_str(), port, gPath.c_str());
  else             ws.begin(gHost.c_str(), port, gPath.c_str());
  ws.onEvent(onEvent);
  ws.setReconnectInterval(3000);
  ws.enableHeartbeat(15000, 5000, 2);

  nextSample = micros();
}

void loop() {
  ws.loop();
  alertTick();

  if ((int32_t)(micros() - nextSample) >= 0) {
    nextSample += SAMPLE_US;
    if ((int32_t)(micros() - nextSample) > 100000) nextSample = micros();
    if (!sensorOk) {
      // sensor lost: once a second, restart the I2C bus and try to wake the sensor again
      if (millis() - lastRetry > 1000) {
        lastRetry = millis();
        Wire.end();
        if (mpuInit()) {
          float tx, ty, tz;
          if (readAccel(tx, ty, tz)) { sensorOk = true; failCount = 0; n = 0; Serial.println("MPU6050 recovered"); }
        }
      }
      return;
    }
    float x, y, z;
    if (readAccel(x, y, z)) {
      failCount = 0;
      if (connected) {
        bx[n] = x; by[n] = y; bz[n] = z;
        if (++n >= BATCH) sendBatch();
      }
    } else if (++failCount >= 30) {   // ~0.3 s of failures in a row
      sensorOk = false;
      n = 0;
      Serial.println("MPU6050 LOST - check SDA/SCL wires and solder joints. Retrying every second...");
    }
  }
}
