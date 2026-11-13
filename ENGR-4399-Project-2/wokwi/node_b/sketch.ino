// WOKWI SIMULATION: Node B (distribution zone)
// Same logic as the hardware sketch node_b.ino. Only the radio layer differs:
// Wokwi cannot run ESP-NOW between boards, so Nodes A and C are emulated by a timed JSON
// generator and buttons (see the Wokwi emulation layer above setup()).
// Libraries to add in Wokwi: ArduinoJson, ESP32Servo

// Node B: Distribution zone (greenhouse tank)
// Sensors: tank level (GPIO 34), leak / water presence (GPIO 35)
// Output:  servo valve (GPIO 18)
// Mesh:    ESP-NOW flood with TTL, JSON heartbeat every 2 s
// Libraries: ArduinoJson (v7), ESP32Servo
//
// Fail-safe: if Node A is silent for 6 s, or a leak is seen, this node closes
// its own valve and goes SAFE. It does not wait for the dashboard.
// Leaving SAFE: automatic once the cause has been gone for HOLD_MS.
// A closed valve is the safe direction, so no operator ACK is needed here.

#include <ArduinoJson.h>
#include <ESP32Servo.h>

// Declared at the top so the Arduino auto-prototype step can see it.
struct Btn {
  int pin;
  bool down;
  uint32_t changed;
};

const char MY_ID = 'B';
const uint8_t WIFI_CHANNEL = 1;        // placeholder, must match every node
const uint8_t TTL_START = 2;           // extra hops after the first send
const uint32_t HEARTBEAT_MS = 2000;
const uint32_t LATE_MS = 4000;         // 2 missed heartbeats: DEGRADED
const uint32_t LOST_MS = 6000;         // 3 missed heartbeats: LOST
const uint32_t HOLD_MS = 5000;         // cause must stay gone this long to leave SAFE
const uint32_t STARTUP_GRACE_MS = 10000;
const uint32_t SENSE_MS = 250;
const uint32_t PRINT_MS = 2000;

// Pins (analog on ADC1 only, GPIO 32 to 39)
const int PIN_TANK = 34;
const int PIN_LEAK = 35;
const int PIN_VALVE = 18;

// Calibration: read the raw values from the serial monitor and adjust.
// Power the analog sensors from 3.3 V so the signal stays within range.
const int TANK_EMPTY_RAW = 0;
const int TANK_FULL_RAW = 2000;
const int LEAK_RAW = 800;              // above this means water is present
const int FULL_CLOSE = 90;             // close valve at this tank level (overflow protection)
const int FULL_REOPEN = 80;            // reopen at this level (hysteresis)
const int VALVE_OPEN_DEG = 90;
const int VALVE_CLOSED_DEG = 0;

const char* ZONES[3] = {"Source", "Distribution", "Gateway"};
const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

enum State { NORMAL, DEGRADED, SAFE };
const char* STATE_NAMES[] = {"NORMAL", "DEGRADED", "SAFE"};

struct Peer {
  bool everHeard = false;
  bool lost = false;
  uint32_t lastHeard = 0;
  uint32_t lastSeq = 0;
  uint32_t lastUp = 0;
  int level = -1;
  char via = '?';
};
Peer peers[3];  // 0 = A, 1 = B (self), 2 = C

struct RxMsg {
  uint8_t data[200];
  int len;
};

Servo valve;
State state = NORMAL;
char why[10] = "";
bool leak = false;
bool fullLock = false;
bool valveOpen = false;
int leakHits = 0;
int tankPct = 0;
int tankRaw = 0;
int leakRaw = 0;
uint32_t mySeq = 0;
uint32_t lastBeat = 0, lastPrint = 0, lastSense = 0, lastUnsafe = 0;

int idx(char c) { return c - 'A'; }

void logEvent(const char* text) {
  uint32_t s = millis() / 1000;
  Serial.printf("[%02u:%02u] EVENT: %s\n", (unsigned)(s / 60), (unsigned)(s % 60), text);
}

// ---------- Sensors ----------
int readRaw(int pin) {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(pin);
  return (int)(sum / 8);
}

int rawToPct(int raw, int lo, int hi) {
  return constrain(map(raw, lo, hi, 0, 100), 0, 100);
}

void readSensors() {
  tankRaw = readRaw(PIN_TANK);
  tankPct = rawToPct(tankRaw, TANK_EMPTY_RAW, TANK_FULL_RAW);

  leakRaw = readRaw(PIN_LEAK);
  if (leakRaw > LEAK_RAW) {
    if (leakHits < 5) leakHits++;
  } else {
    leakHits = 0;
  }
  bool nowLeak = leakHits >= 3;  // 3 reads in a row, about 0.75 s
  if (nowLeak != leak) {
    leak = nowLeak;
    logEvent(leak ? "Leak detected, zone Distribution flagged" : "Leak cleared");
  }
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
  doc["lvl"] = tankPct;
  doc["leak"] = leak ? 1 : 0;
  doc["valve"] = valveOpen ? "OPEN" : "CLOSED";
  doc["st"] = STATE_NAMES[state];
  doc["why"] = why;
  sendDoc(doc);
}

void handleMsg(const RxMsg& m) {
  JsonDocument doc;
  if (deserializeJson(doc, (const char*)m.data, m.len)) return;  // bad JSON, drop

  const char* idc = doc["id"] | "?";
  char origin = idc[0];
  if (origin < 'A' || origin > 'C' || origin == MY_ID) return;

  const char* viac = doc["via"] | "?";
  char via = viac[0];
  uint32_t seq = doc["seq"] | 0;
  uint32_t up = doc["up"] | 0;
  Peer& p = peers[idx(origin)];

  bool rebooted = p.everHeard && up < p.lastUp;
  if (p.everHeard && !rebooted && seq <= p.lastSeq) return;  // duplicate or old

  if (p.lost) {
    char msg[48];
    snprintf(msg, sizeof(msg), "Node %c back, zone %s", origin, ZONES[idx(origin)]);
    logEvent(msg);
  }
  p.everHeard = true;
  p.lost = false;
  p.lastHeard = millis();
  p.lastSeq = seq;
  p.lastUp = up;
  p.level = doc["lvl"] | -1;
  p.via = via;

  // Forward so nodes out of direct range still hear it.
  int ttl = doc["ttl"] | 0;
  if (ttl > 0) {
    char meStr[2] = {MY_ID, 0};
    doc["ttl"] = ttl - 1;
    doc["via"] = meStr;
    sendDoc(doc);
  }
}

bool peerAlive(int i) {
  return peers[i].everHeard && (millis() - peers[i].lastHeard) < LOST_MS;
}

bool peerLate(int i) {
  return peerAlive(i) && (millis() - peers[i].lastHeard) > LATE_MS;
}

void checkTimeouts() {
  for (int i = 0; i < 3; i++) {
    if (i == idx(MY_ID)) continue;
    Peer& p = peers[i];
    if (p.everHeard && !p.lost && millis() - p.lastHeard > LOST_MS) {
      p.lost = true;
      char msg[56];
      snprintf(msg, sizeof(msg), "Node %c LOST, zone %s flagged", 'A' + i, ZONES[i]);
      logEvent(msg);
    }
  }
}

// ---------- State machine ----------
void setState(State s) {
  if (s == state) return;
  char msg[56];
  if (s == SAFE) snprintf(msg, sizeof(msg), "State %s -> SAFE (%s)", STATE_NAMES[state], why);
  else snprintf(msg, sizeof(msg), "State %s -> %s", STATE_NAMES[state], STATE_NAMES[s]);
  state = s;
  logEvent(msg);
}

void updateState() {
  uint32_t now = millis();
  bool graceOver = now > STARTUP_GRACE_MS;
  bool aMissing = !peerAlive(0) && (peers[0].everHeard || graceOver);
  bool cMissing = !peerAlive(2) && (peers[2].everHeard || graceOver);
  bool unsafe = aMissing || leak;

  if (unsafe) {
    lastUnsafe = now;
    strcpy(why, leak ? "LEAK" : "A_LOST");
    setState(SAFE);
    return;
  }
  if (state == SAFE) {
    if (now - lastUnsafe < HOLD_MS) return;  // still holding
    why[0] = 0;
  }
  bool late = peerLate(0) || peerLate(2);
  setState((cMissing || late) ? DEGRADED : NORMAL);
}

// ---------- Valve ----------
void updateValve() {
  if (tankPct >= FULL_CLOSE) fullLock = true;
  else if (tankPct <= FULL_REOPEN) fullLock = false;

  // Closed by default: only open after A has been heard and nothing is wrong.
  bool want = (state != SAFE) && peers[0].everHeard && !fullLock;
  if (want != valveOpen) {
    valveOpen = want;
    valve.write(valveOpen ? VALVE_OPEN_DEG : VALVE_CLOSED_DEG);
    if (valveOpen) logEvent("Valve OPEN");
    else logEvent(fullLock ? "Valve CLOSED, tank full" : "Valve CLOSED");
  }
}

// ---------- Status line ----------
const char* peerText(int i) {
  if (!peers[i].everHeard) return "WAIT ";
  return peerAlive(i) ? "ALIVE" : "LOST ";
}

float peerAge(int i) {
  return peers[i].everHeard ? (millis() - peers[i].lastHeard) / 1000.0 : 0.0;
}

void printStatus() {
  uint32_t s = millis() / 1000;
  Serial.printf("[%02u:%02u] B %-8s | tank %3d%% (raw %d) | leak %s (raw %d) | valve %s | A %s %.1fs | C %s %.1fs\n",
                (unsigned)(s / 60), (unsigned)(s % 60), STATE_NAMES[state], tankPct, tankRaw,
                leak ? "YES" : "NO", leakRaw, valveOpen ? "OPEN" : "CLOSED",
                peerText(0), peerAge(0), peerText(2), peerAge(2));
}

// ---------- Arduino ----------
// ---------------- Wokwi emulation layer ----------------
// Real JSON heartbeats are built here and passed to handleMsg(), the same function the radio
// calls on hardware, so parsing, timeouts and the fail-safe are the real code.
//   Red button    GPIO 26  stop / resume Node A heartbeats
//   Yellow button GPIO 25  stop / resume Node C heartbeats
//   Pot GPIO 34            tank level at Node B (real sensor input)
//   Pot GPIO 35            leak sensor at Node B (above the LEAK_RAW value counts as a leak)
const int PIN_KILL_A = 26;
const int PIN_KILL_C = 25;

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
Btn btnKillC = {PIN_KILL_C, false, 0};
bool simStopA = false, simStopC = false, simFirst = true;
uint32_t simSeqA = 0, simSeqC = 0, simLast = 0;

void simPeers() {
  if (pressed(btnKillA)) {
    simStopA = !simStopA;
    logEvent(simStopA ? "[SIM] Node A heartbeats STOPPED" : "[SIM] Node A heartbeats RESUMED");
  }
  if (pressed(btnKillC)) {
    simStopC = !simStopC;
    logEvent(simStopC ? "[SIM] Node C heartbeats STOPPED" : "[SIM] Node C heartbeats RESUMED");
  }

  if (!simFirst && millis() - simLast < HEARTBEAT_MS) return;
  simFirst = false;
  simLast = millis();

  if (!simStopA) {
    JsonDocument doc;
    doc["id"] = "A";
    doc["zone"] = ZONES[0];
    doc["seq"] = ++simSeqA;
    doc["up"] = millis() / 1000;
    doc["ttl"] = 0;
    doc["via"] = "A";
    doc["lvl"] = 70;
    doc["wet"] = 1;
    doc["pump"] = "ON";
    doc["st"] = "NORMAL";
    feed(doc);
  }
  if (!simStopC) {
    JsonDocument doc;
    doc["id"] = "C";
    doc["zone"] = ZONES[2];
    doc["seq"] = ++simSeqC;
    doc["up"] = millis() / 1000;
    doc["ttl"] = 0;
    doc["via"] = "C";
    doc["lvl"] = 40;
    doc["st"] = "NORMAL";
    feed(doc);
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  valve.attach(PIN_VALVE);
  valve.write(VALVE_CLOSED_DEG);  // closed until A is heard

  Serial.printf("Node %c up, zone %s\n", MY_ID, ZONES[idx(MY_ID)]);
  pinMode(PIN_KILL_A, INPUT_PULLUP);
  pinMode(PIN_KILL_C, INPUT_PULLUP);
  Serial.println("WOKWI SIM, Node B. Red: stop/resume A | Yellow: stop/resume C");
  Serial.println("Tank pot = tank level, leak pot = leak sensor. Servo = valve.");
}

void loop() {
  simPeers();  // emulated peers: Wokwi cannot run ESP-NOW between boards

  uint32_t now = millis();
  if (now - lastSense >= SENSE_MS) {
    lastSense = now;
    readSensors();
  }
  checkTimeouts();
  updateState();
  updateValve();

  if (now - lastBeat >= HEARTBEAT_MS) {
    lastBeat = now;
    sendHeartbeat();
  }
  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    printStatus();
  }
}
