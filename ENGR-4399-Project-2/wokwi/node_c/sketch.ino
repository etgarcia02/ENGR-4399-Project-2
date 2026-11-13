// WOKWI SIMULATION: Node C (gateway and monitor)
// Same logic as the hardware sketch node_c.ino. Only the radio layer differs:
// Wokwi cannot run ESP-NOW between boards, so Nodes A and B are emulated by a timed JSON
// generator and buttons (see the Wokwi emulation layer above setup()).
// Libraries to add in Wokwi: ArduinoJson, DHT sensor library, Adafruit Unified Sensor
// Wokwi only has the DHT22 part, so this sim uses the DHT22 type. On hardware use DHT11.

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

#include <ArduinoJson.h>
#include <DHT.h>

// Declared at the top so the Arduino auto-prototype step can see it.
struct Btn {
  int pin;
  bool down;
  uint32_t changed;
};

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
DHT dht(PIN_DHT, DHT22);  // Wokwi only has the DHT22 part. On hardware use DHT11.

// true: A and C ignore each other's direct frames, so A <-> C goes through B.
// Set the same on Node A. Needed to demonstrate multi-hop on one desk.
const bool FORCE_LINE = true;

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
  (void)n;
  Serial.printf("TX %s\n", buf);  // Wokwi: show the message instead of sending it by radio
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
// ---------------- Wokwi emulation layer ----------------
// Real JSON heartbeats are built here and passed to handleMsg(), the same function the radio
// calls on hardware, so parsing, timeouts and the dashboard are the real code.
//   Red button    GPIO 26  stop / resume Node A heartbeats
//   Yellow button GPIO 25  stop / resume Node B heartbeats
//   Blue button   GPIO 33  Node B reports a leak and goes SAFE
//   Green button  GPIO 27  mute the buzzer (real input)
//   Pot GPIO 34            Node B tank level, 0 to 100 percent
// With FORCE_LINE, A is reached only through B, so A also goes silent when B is stopped.
const int PIN_KILL_A = 26;
const int PIN_KILL_B = 25;
const int PIN_B_LEAK = 33;
const int PIN_B_TANK = 34;

bool pressed(Btn& b) {
  bool d = digitalRead(b.pin) == LOW;
  if (d != b.down && millis() - b.changed > 40) {
    b.changed = millis();
    b.down = d;
    return d;
  }
  return false;
}

// Builds the message exactly as a radio frame would arrive and passes it to handleMsg().
void feed(JsonDocument& doc) {
  RxMsg m;
  m.len = (int)serializeJson(doc, (char*)m.data, sizeof(m.data));
  handleMsg(m);
}

Btn btnKillA = {PIN_KILL_A, false, 0};
Btn btnKillB = {PIN_KILL_B, false, 0};
Btn btnBLeak = {PIN_B_LEAK, false, 0};
bool simStopA = false, simStopB = false, simBLeak = false, simPump = false, simFirst = true;
uint32_t simSeqA = 0, simSeqB = 0, simLast = 0, simAStopT = 0;

void simPeers() {
  if (pressed(btnKillA)) {
    simStopA = !simStopA;
    simAStopT = millis();
    logEvent(simStopA ? "[SIM] Node A heartbeats STOPPED" : "[SIM] Node A heartbeats RESUMED");
  }
  if (pressed(btnKillB)) {
    simStopB = !simStopB;
    logEvent(simStopB ? "[SIM] Node B heartbeats STOPPED" : "[SIM] Node B heartbeats RESUMED");
  }
  if (pressed(btnBLeak)) {
    simBLeak = !simBLeak;
    logEvent(simBLeak ? "[SIM] Node B reports a leak (SAFE)" : "[SIM] Node B leak cleared");
  }

  if (!simFirst && millis() - simLast < HEARTBEAT_MS) return;
  simFirst = false;
  simLast = millis();

  int bTank = map(analogRead(PIN_B_TANK), 0, 4095, 0, 100);
  bool aSilent = simStopA && millis() - simAStopT > LOST_MS;
  bool bSafe = simBLeak || aSilent;  // B goes SAFE on a leak, or when A has been silent for 6 s

  // Mirrors Node A's pump rule: follow Node B's tank level with hysteresis.
  if (bTank >= 80) simPump = false;
  else if (bTank < 30) simPump = true;

  if (!simStopA && !simStopB) {
    JsonDocument doc;
    doc["id"] = "A";
    doc["zone"] = ZONES[0];
    doc["seq"] = ++simSeqA;
    doc["up"] = millis() / 1000;
    doc["ttl"] = 0;
    doc["via"] = "B";  // A is reached through B, so the dashboard shows A -> B -> C
    doc["lvl"] = 70;
    doc["wet"] = 1;
    doc["pump"] = (bSafe || bTank >= 80) ? "OFF" : (simPump ? "ON" : "OFF");
    doc["st"] = "NORMAL";
    feed(doc);
  }
  if (!simStopB) {
    JsonDocument doc;
    doc["id"] = "B";
    doc["zone"] = ZONES[1];
    doc["seq"] = ++simSeqB;
    doc["up"] = millis() / 1000;
    doc["ttl"] = 0;
    doc["via"] = "B";
    doc["lvl"] = bTank;
    doc["leak"] = simBLeak ? 1 : 0;
    doc["valve"] = (bSafe || bTank >= 90) ? "CLOSED" : "OPEN";
    doc["st"] = bSafe ? "SAFE" : "NORMAL";
    doc["why"] = bSafe ? (simBLeak ? "LEAK" : "A_LOST") : "";
    feed(doc);
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_MUTE, INPUT_PULLUP);

  Serial.printf("Node %c up, zone %s\n", MY_ID, ZONES[idx(MY_ID)]);
  pinMode(PIN_KILL_A, INPUT_PULLUP);
  pinMode(PIN_KILL_B, INPUT_PULLUP);
  pinMode(PIN_B_LEAK, INPUT_PULLUP);
  Serial.println("WOKWI SIM, Node C. Red: stop/resume A | Yellow: stop/resume B | Blue: B leak | Green: mute");
  Serial.println("Tank pot = Node B tank level. DHT22 = temperature and humidity.");
}

void loop() {
  simPeers();  // emulated peers: Wokwi cannot run ESP-NOW between boards

  uint32_t now = millis();
  if (now - lastSense >= SENSE_MS) {
    lastSense = now;
    readSensor();
  }
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
