// Node C: Gateway and monitor
// Sensor:  DHT11 temperature and humidity (GPIO 4, 3.3 V)
// Outputs: buzzer (GPIO 23) and the serial dashboard over USB
// Input:   mute button (GPIO 27, to GND)
// Mesh:    ESP-NOW flood with TTL, JSON heartbeat every 2 s
// Libraries: ArduinoJson (v7), "DHT sensor library" and "Adafruit Unified Sensor" (Adafruit)
//
// Fail-safe: the buzzer alarms by itself when Node A or B is lost or reports SAFE.
// It does not wait for any command. The mute button silences it until the
// system returns to NORMAL.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <DHT.h>

const char MY_ID = 'C';
const uint8_t WIFI_CHANNEL = 1;        // placeholder, must match every node
const uint8_t TTL_START = 2;
const uint32_t HEARTBEAT_MS = 2000;
const uint32_t LATE_MS = 4000;
const uint32_t LOST_MS = 6000;
const uint32_t STARTUP_GRACE_MS = 10000;
const uint32_t SENSE_MS = 2000;        // DHT11 needs at least 1 s between reads
const uint32_t PRINT_MS = 1000;

// Pins
const int PIN_DHT = 4;                 // DHT11 data pin (bare 4-pin part: 10k pull-up to 3.3 V)
const int PIN_BUZZER = 23;
const int PIN_MUTE = 27;

const bool BUZZER_ACTIVE = true;       // false for a passive buzzer
DHT dht(PIN_DHT, DHT11);

// true: A and C ignore each other's direct frames, so A <-> C goes through B.
// Set the same on Node A. Needed to demonstrate multi-hop on one desk.
const bool FORCE_LINE = true;

// TEST MODE: Node C also pretends to be Node A, so B and C can be tested without
// the real Node A. Set to false when the real Node A is on the mesh.
// Serial monitor command: type k to stop or resume the fake Node A heartbeats.
const bool SIM_NODE_A = true;

const char* ZONES[3] = {"Source", "Distribution", "Gateway"};
const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

enum State { NORMAL, DEGRADED, SAFE };
const char* STATE_NAMES[] = {"NORMAL", "DEGRADED", "SAFE"};

struct Peer {
  bool everHeard = false;
  bool lost = false;
  bool leak = false;
  uint32_t lastHeard = 0;
  uint32_t lastSeq = 0;
  uint32_t lastUp = 0;
  int level = -1;
  char via = '?';
  char st[10] = "?";
  char pump[4] = "-";
  char valve[7] = "-";
};
Peer peers[3];  // 0 = A, 1 = B, 2 = C (self)

struct RxMsg {
  uint8_t data[200];
  int len;
};
QueueHandle_t rxQueue;

const int LOG_N = 6;
char logBuf[LOG_N][64];
int logCount = 0;

State sysState = NORMAL;
bool zoneSource = false, zoneDist = false;
bool muted = false;
float tempC = NAN, humPct = NAN;
int dhtFails = 0;
uint32_t mySeq = 0;
uint32_t lastBeat = 0, lastPrint = 0, lastSense = 0;

int idx(char c) { return c - 'A'; }

void copyStr(char* dst, size_t n, const char* src) {
  strncpy(dst, src, n - 1);
  dst[n - 1] = 0;
}

// Event log is a ring buffer shown on every dashboard refresh.
void logEvent(const char* text) {
  uint32_t s = millis() / 1000;
  snprintf(logBuf[logCount % LOG_N], sizeof(logBuf[0]), "[%02u:%02u] %s",
           (unsigned)(s / 60), (unsigned)(s % 60), text);
  logCount++;
}

// ---------- Sensor ----------
void readSensor() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    if (dhtFails < 100) dhtFails++;
    if (dhtFails == 3) logEvent("DHT11 read failing, check wiring");
    return;  // keep the last good values
  }
  if (dhtFails >= 3) logEvent("DHT11 reading again");
  dhtFails = 0;
  tempC = t;
  humPct = h;
}

// ---------- Mesh ----------
void sendDoc(JsonDocument& doc) {
  char buf[200];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  esp_now_send(BCAST, (uint8_t*)buf, n);
}

void sendHeartbeat() {
  char idStr[2] = {MY_ID, 0};
  JsonDocument doc;
  doc["id"] = idStr;
  doc["zone"] = ZONES[idx(MY_ID)];
  doc["seq"] = ++mySeq;
  doc["up"] = millis() / 1000;
  doc["ttl"] = TTL_START;
  doc["via"] = idStr;
  if (!isnan(tempC)) {
    doc["temp"] = (int)round(tempC);
    doc["hum"] = (int)round(humPct);
  }
  doc["st"] = STATE_NAMES[sysState];
  sendDoc(doc);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
#else
void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  if (len <= 0 || len > 200) return;
  RxMsg m;
  memcpy(m.data, data, len);
  m.len = len;
  xQueueSend(rxQueue, &m, 0);
}

bool peerAlive(int i) {
  return peers[i].everHeard && (millis() - peers[i].lastHeard) < LOST_MS;
}

bool peerLate(int i) {
  return peerAlive(i) && (millis() - peers[i].lastHeard) > LATE_MS;
}

void handleMsg(const RxMsg& m) {
  JsonDocument doc;
  if (deserializeJson(doc, (const char*)m.data, m.len)) return;

  const char* idc = doc["id"] | "?";
  char origin = idc[0];
  if (origin < 'A' || origin > 'C' || origin == MY_ID) return;

  const char* viac = doc["via"] | "?";
  char via = viac[0];
  if (FORCE_LINE && origin == 'A' && via == 'A') return;  // pretend A is out of range

  uint32_t seq = doc["seq"] | 0;
  uint32_t up = doc["up"] | 0;
  Peer& p = peers[idx(origin)];

  bool firstTime = !p.everHeard;
  bool rebooted = !firstTime && up < p.lastUp;
  if (!firstTime && !rebooted && seq <= p.lastSeq) return;  // duplicate or old

  const char* st = doc["st"] | "?";
  bool leakNow = (doc["leak"] | 0) != 0;
  char msg[64];

  if (firstTime) {
    snprintf(msg, sizeof(msg), "Node %c joined, reached via %c", origin, via);
    logEvent(msg);
  } else {
    if (p.lost) {
      snprintf(msg, sizeof(msg), "Node %c back, zone %s", origin, ZONES[idx(origin)]);
      logEvent(msg);
    }
    if (strcmp(p.st, st) != 0) {
      snprintf(msg, sizeof(msg), "Node %c reports %s", origin, st);
      logEvent(msg);
    }
    if (leakNow != p.leak) {
      snprintf(msg, sizeof(msg), leakNow ? "Leak reported, zone %s flagged" : "Leak cleared in zone %s",
               ZONES[idx(origin)]);
      logEvent(msg);
    }
  }

  p.everHeard = true;
  p.lost = false;
  p.leak = leakNow;
  p.lastHeard = millis();
  p.lastSeq = seq;
  p.lastUp = up;
  p.level = doc["lvl"] | -1;
  p.via = via;
  copyStr(p.st, sizeof(p.st), st);
  copyStr(p.pump, sizeof(p.pump), doc["pump"] | "-");
  copyStr(p.valve, sizeof(p.valve), doc["valve"] | "-");

  int ttl = doc["ttl"] | 0;
  if (ttl > 0) {
    char meStr[2] = {MY_ID, 0};
    doc["ttl"] = ttl - 1;
    doc["via"] = meStr;
    sendDoc(doc);
  }
}

void checkTimeouts() {
  for (int i = 0; i < 2; i++) {  // only A and B
    Peer& p = peers[i];
    if (p.everHeard && !p.lost && millis() - p.lastHeard > LOST_MS) {
      p.lost = true;
      char msg[56];
      snprintf(msg, sizeof(msg), "Node %c LOST, zone %s flagged", 'A' + i, ZONES[i]);
      logEvent(msg);
    }
  }
}

// ---------- Test mode: fake Node A ----------
bool simAStopped = false;
bool simPump = false;
uint32_t simSeqA = 0;
uint32_t lastSimBeat = 0;

// Sends a real JSON heartbeat as Node A. Node B hears it and forwards it, so C
// receives it back through B, which also shows the multi-hop path.
void simNodeA() {
  if (!SIM_NODE_A || simAStopped) return;
  if (millis() - lastSimBeat < HEARTBEAT_MS) return;
  lastSimBeat = millis();

  // Mirrors Node A's pump rule: follow Node B's tank level with hysteresis.
  int bl = peers[1].level;
  if (!peerAlive(1)) simPump = false;
  else if (bl >= 80) simPump = false;
  else if (bl >= 0 && bl < 30) simPump = true;

  JsonDocument doc;
  doc["id"] = "A";
  doc["zone"] = ZONES[0];
  doc["seq"] = ++simSeqA;
  doc["up"] = millis() / 1000;
  doc["ttl"] = TTL_START;
  doc["via"] = "A";
  doc["lvl"] = 70;
  doc["pump"] = simPump ? "ON" : "OFF";
  doc["st"] = peerAlive(1) ? "NORMAL" : "SAFE";
  sendDoc(doc);
}

void checkSimCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (SIM_NODE_A && (c == 'k' || c == 'K')) {
      simAStopped = !simAStopped;
      logEvent(simAStopped ? "[SIM] Node A heartbeats STOPPED" : "[SIM] Node A heartbeats RESUMED");
    }
  }
}

// ---------- System state ----------
bool missing(int i) {
  return !peerAlive(i) && (peers[i].everHeard || millis() > STARTUP_GRACE_MS);
}

bool reports(int i, const char* s) {
  return peers[i].everHeard && strcmp(peers[i].st, s) == 0;
}

void updateSystem() {
  zoneSource = missing(0) || reports(0, "SAFE");
  zoneDist = missing(1) || reports(1, "SAFE");

  State next = NORMAL;
  if (peerLate(0) || peerLate(1) || reports(0, "DEGRADED") || reports(1, "DEGRADED")) next = DEGRADED;
  if (zoneSource || zoneDist) next = SAFE;

  if (next != sysState) {
    char msg[56];
    snprintf(msg, sizeof(msg), "System %s -> %s", STATE_NAMES[sysState], STATE_NAMES[next]);
    sysState = next;
    logEvent(msg);
  }
  if (sysState != SAFE) muted = false;  // re-arm for the next fault
}

// ---------- Buzzer and mute ----------
void checkMute() {
  static bool last = false;
  static uint32_t changed = 0;
  bool down = digitalRead(PIN_MUTE) == LOW;
  if (down != last && millis() - changed > 40) {
    changed = millis();
    last = down;
    if (down && sysState == SAFE && !muted) {
      muted = true;
      logEvent("Operator muted the buzzer");
    }
  }
}

void buzzerTick() {
  bool alarm = (sysState == SAFE) && !muted;
  bool beepPhase = (millis() / 300) % 2 == 0;
  if (!alarm || !beepPhase) {
    digitalWrite(PIN_BUZZER, LOW);
    return;
  }
  if (BUZZER_ACTIVE) {
    digitalWrite(PIN_BUZZER, HIGH);
    return;
  }
  static uint32_t t = 0;
  static bool lvl = false;
  if (micros() - t >= 500) {  // about 1 kHz square wave
    t = micros();
    lvl = !lvl;
    digitalWrite(PIN_BUZZER, lvl);
  }
}

// ---------- Dashboard (serial monitor) ----------
void printPeerLine(int i) {
  Peer& p = peers[i];
  if (!p.everHeard) {
    Serial.printf(" %c %-13s WAITING\n", 'A' + i, ZONES[i]);
    return;
  }
  Serial.printf(" %c %-13s %s last %4.1fs  level %3d%%  state %-8s", 'A' + i, ZONES[i],
                peerAlive(i) ? "ALIVE" : "LOST ", (millis() - p.lastHeard) / 1000.0, p.level, p.st);
  if (i == 0) Serial.printf("  pump %s", p.pump);
  if (i == 1) Serial.printf("  valve %s  leak %s", p.valve, p.leak ? "YES" : "NO");
  Serial.printf("  via %c\n", p.via);
}

void printDashboard() {
  uint32_t s = millis() / 1000;
  Serial.printf("\n==== [%02u:%02u] GATEWAY DASHBOARD ====\n", (unsigned)(s / 60), (unsigned)(s % 60));
  Serial.printf("SYSTEM STATE: %s   buzzer: %s\n", STATE_NAMES[sysState],
                sysState != SAFE ? "OFF" : (muted ? "MUTED" : "ON"));
  if (zoneSource || zoneDist) {
    Serial.printf("AFFECTED ZONE: %s%s%s\n", zoneSource ? "Source" : "",
                  (zoneSource && zoneDist) ? " and " : "", zoneDist ? "Distribution" : "");
  }
  printPeerLine(0);
  printPeerLine(1);
  if (isnan(tempC)) Serial.printf(" C %-13s ALIVE (self)  DHT11 no reading yet\n", ZONES[2]);
  else Serial.printf(" C %-13s ALIVE (self)  temp %.1f C  humidity %.0f%%%s\n", ZONES[2], tempC, humPct,
                     dhtFails >= 3 ? "  (stale)" : "");

  Serial.print("MESH PATH: ");
  if (peerAlive(0)) Serial.print(peers[0].via == 'B' ? "A -> B -> C" : "A -> C (direct)");
  else Serial.print("A -x-");
  Serial.print("   ");
  if (peerAlive(1)) Serial.print(peers[1].via == 'A' ? "B -> A -> C" : "B -> C (direct)");
  else Serial.print("B -x-");
  Serial.println();

  Serial.println("EVENT LOG:");
  int n = min(logCount, LOG_N);
  for (int k = 0; k < n; k++) {
    Serial.printf("  %s\n", logBuf[(logCount - n + k) % LOG_N]);
  }
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_MUTE, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  rxQueue = xQueueCreate(8, sizeof(RxMsg));
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;  // add a PMK/LMK later, keep them as placeholders in shared code
  esp_now_add_peer(&peer);

  Serial.printf("Node %c up, zone %s\n", MY_ID, ZONES[idx(MY_ID)]);
  if (SIM_NODE_A) Serial.println("TEST MODE: this board also sends fake Node A heartbeats. Type k to stop or resume them.");
}

void loop() {
  RxMsg m;
  while (xQueueReceive(rxQueue, &m, 0) == pdTRUE) handleMsg(m);

  uint32_t now = millis();
  if (now - lastSense >= SENSE_MS) {
    lastSense = now;
    readSensor();
  }
  simNodeA();
  checkSimCommands();
  checkTimeouts();
  updateSystem();
  checkMute();
  buzzerTick();

  if (now - lastBeat >= HEARTBEAT_MS) {
    lastBeat = now;
    sendHeartbeat();
  }
  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    printDashboard();
  }
}
