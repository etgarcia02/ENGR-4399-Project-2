// Node A (partner's official sketch) with the fail-safe additions the project sheet requires.
// Original: ESP-NOW mesh + water sensor + L293D pump, same JSON heartbeat as Node B and Node C.
//
// Lines marked ADDED or CHANGED are new. The water sensor, the L293D pump drive and the mesh
// code are the partner's. What was added:
//   - local fail-safe: pump OFF when Node B is silent for 6 s (SAFE state)
//   - cooperation: pump OFF when Node B reports its tank full, or when B is in SAFE
//   - leaving SAFE: Node B stable for 5 s, then operator ACK (pushbutton GPIO 27, or type 'a')
//   - heartbeat now carries pump, wet and the real system state for the dashboard
// Needs the ArduinoJson library (version 7). Serial Monitor: 115200 baud.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

const char MY_ID = 'A';              // this sketch is for Node A
const uint8_t WIFI_CHANNEL = 1;      // must match on all nodes
const uint8_t TTL_START = 2;         // max extra hops after the first send
const uint32_t HEARTBEAT_MS = 2000;  // send rate
const uint32_t TIMEOUT_MS = 6000;    // 3 missed heartbeats = LOST
const uint32_t PRINT_MS = 1000;      // dashboard refresh
const uint32_t SENSE_MS = 1000;      // how often to read the water sensor

// Hardware
const int SIGNAL_PIN = 35;           // water sensor signal (ADC1, input-only)
const int IN1_PIN = 13;              // L293D IN1
const int IN2_PIN = 14;              // L293D IN2
const int THRESHOLD = 1000;          // raw ADC value (0-4095) that counts as "wet"

// ADDED: fail-safe settings
const int ACK_PIN = 27;              // pushbutton between GPIO 27 and GND
const uint32_t LATE_MS = 4000;       // 2 missed heartbeats = DEGRADED
const uint32_t HOLD_MS = 5000;       // Node B must be stable this long before ACK works
const uint32_t STARTUP_GRACE_MS = 10000;
const int TANK_STOP_ABOVE = 80;      // Node B tank level that stops the pump
const int TANK_START_BELOW = 30;     // pump may run again below this level (hysteresis)
const bool REQUIRE_ACK = true;       // false: leave SAFE automatically after the hold time

// true: A and C ignore each other's direct frames, so A <-> C must go through B.
// Set to false for real use once the boards are placed apart.
const bool FORCE_LINE = true;

const char* ZONES[3] = {"Source", "Distribution", "Gateway"};
const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ADDED: state machine
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
  char st[10] = "?";               // CHANGED: state reported by the peer (replaces the wet flag)
};
Peer peers[3];  // 0 = A (self), 1 = B, 2 = C

struct RxMsg {
  uint8_t data[200];
  int len;
};
QueueHandle_t rxQueue;
uint32_t mySeq = 0;
uint32_t lastBeat = 0;
uint32_t lastPrint = 0;
uint32_t lastSense = 0;

int myRaw = 0;
bool myWet = false;

// ADDED
State state = NORMAL;
char why[10] = "";
bool pumpOn = false;
bool tankFullLock = false;
bool ackReceived = false;
uint32_t lastUnsafe = 0;

int idx(char c) { return c - 'A'; }

void logEvent(const char* text) {
  uint32_t s = millis() / 1000;
  Serial.printf("[%02u:%02u] EVENT: %s\n", (unsigned)(s / 60), (unsigned)(s % 60), text);
}

// Read the water sensor (partner's logic)
void readSensor() {
  myRaw = analogRead(SIGNAL_PIN);
  bool wet = myRaw > THRESHOLD;

  if (wet != myWet) {
    logEvent(wet ? "Water detected at source" : "Water gone at source");  // CHANGED: pump is logged in drivePump()
  }
  myWet = wet;
}

// Drive the pump through the L293D (partner's logic)
void setPump(bool on) {
  if (on) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }
}

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
  doc["lvl"] = map(myRaw, 0, 4095, 0, 100);
  doc["wet"] = myWet ? 1 : 0;                 // ADDED
  doc["pump"] = pumpOn ? "ON" : "OFF";        // ADDED
  doc["st"] = STATE_NAMES[state];             // CHANGED: was "WET" or "NORMAL"
  doc["why"] = why;                           // ADDED
  sendDoc(doc);
}

// Runs in the WiFi task, so keep it tiny: copy the frame and leave.
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

void handleMsg(const RxMsg& m) {
  JsonDocument doc;
  if (deserializeJson(doc, (const char*)m.data, m.len)) return;  // bad JSON, drop it

  const char* idc = doc["id"] | "?";
  char origin = idc[0];
  if (origin < 'A' || origin > 'C' || origin == MY_ID) return;

  const char* viac = doc["via"] | "?";
  char via = viac[0];
  if (FORCE_LINE && via == origin &&
      ((MY_ID == 'A' && origin == 'C') || (MY_ID == 'C' && origin == 'A'))) {
    return;  // pretend A and C are out of radio range
  }

  uint32_t seq = doc["seq"] | 0;
  uint32_t up = doc["up"] | 0;
  Peer& p = peers[idx(origin)];

  bool rebooted = p.everHeard && up < p.lastUp;
  if (p.everHeard && !rebooted && seq <= p.lastSeq) return;  // duplicate or old

  p.everHeard = true;
  p.lastHeard = millis();
  p.lastSeq = seq;
  p.lastUp = up;
  p.level = doc["lvl"] | -1;
  p.via = via;
  const char* st = doc["st"] | "NORMAL";
  strncpy(p.st, st, sizeof(p.st) - 1);        // CHANGED: keep the reported state
  p.st[sizeof(p.st) - 1] = 0;

  if (p.lost) {
    p.lost = false;
    char msg[48];
    snprintf(msg, sizeof(msg), "Node %c back, zone %s", origin, ZONES[idx(origin)]);
    logEvent(msg);
  }

  // Forward it on so nodes out of direct range still hear it.
  int ttl = doc["ttl"] | 0;
  if (ttl > 0) {
    char meStr[2] = {MY_ID, 0};
    doc["ttl"] = ttl - 1;
    doc["via"] = meStr;
    sendDoc(doc);
  }
}

bool peerAlive(int i) {
  return peers[i].everHeard && (millis() - peers[i].lastHeard) < TIMEOUT_MS;
}

bool peerLate(int i) {
  return peerAlive(i) && (millis() - peers[i].lastHeard) > LATE_MS;
}

void checkTimeouts() {
  for (int i = 0; i < 3; i++) {
    if (i == idx(MY_ID)) continue;
    Peer& p = peers[i];
    if (p.everHeard && !p.lost && millis() - p.lastHeard > TIMEOUT_MS) {
      p.lost = true;
      char msg[48];
      snprintf(msg, sizeof(msg), "Node %c LOST, zone %s flagged", 'A' + i, ZONES[i]);
      logEvent(msg);
    }
  }
}

// ---------- ADDED: state machine ----------
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
  bool bMissing = !peerAlive(1) && (peers[1].everHeard || graceOver);
  bool cMissing = !peerAlive(2) && (peers[2].everHeard || graceOver);

  if (bMissing) {
    // The pump depends on Node B. Go safe on our own, do not wait for the dashboard.
    lastUnsafe = now;
    ackReceived = false;
    strcpy(why, "B_LOST");
    setState(SAFE);
    return;
  }
  if (state == SAFE) {
    if (now - lastUnsafe < HOLD_MS) return;       // B must stay stable first
    if (REQUIRE_ACK && !ackReceived) return;      // then the operator confirms
    ackReceived = false;
    why[0] = 0;
  }
  bool late = peerLate(1) || peerLate(2);
  setState((cMissing || late) ? DEGRADED : NORMAL);
}

void handleAck() {
  if (state != SAFE) {
    logEvent("ACK ignored, system not in SAFE");
    return;
  }
  if (peerAlive(1) && millis() - lastUnsafe >= HOLD_MS) {
    ackReceived = true;
    logEvent("Operator ACK accepted");
  } else {
    logEvent("ACK ignored, Node B not stable for 5 s yet");
  }
}

void checkAck() {
  static bool last = false;
  static uint32_t changed = 0;
  bool down = digitalRead(ACK_PIN) == LOW;
  if (down != last && millis() - changed > 40) {
    changed = millis();
    last = down;
    if (down) handleAck();
  }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'a' || c == 'A') handleAck();
  }
}

// ---------- ADDED: pump decision, runs every loop so SAFE stops the pump at once ----------
void drivePump() {
  // Cooperation: Node B's tank level decides when the pump must stop (with hysteresis).
  if (peerAlive(1)) {
    int lvl = peers[1].level;
    if (lvl >= TANK_STOP_ABOVE) tankFullLock = true;
    else if (lvl >= 0 && lvl < TANK_START_BELOW) tankFullLock = false;
  }
  bool bSafe = peers[1].everHeard && strcmp(peers[1].st, "SAFE") == 0;

  // Partner's rule: the pump runs only while water is detected at the source.
  bool want = myWet;
  // Everything below can only turn the pump OFF, never ON.
  if (state == SAFE) want = false;               // local fail-safe
  if (bSafe) want = false;                       // B closed its valve, do not push water into it
  if (tankFullLock) want = false;                // B's tank is full
  if (!peers[1].everHeard) want = false;         // never run before B has been heard

  if (want != pumpOn) {
    pumpOn = want;
    setPump(pumpOn);
    if (pumpOn) logEvent("Pump ON");
    else if (state == SAFE) logEvent("Pump OFF, SAFE state");
    else if (tankFullLock) logEvent("Pump OFF, destination tank full");
    else if (bSafe) logEvent("Pump OFF, Node B in SAFE");
    else logEvent("Pump OFF");
  }
}

void printDashboard() {
  uint32_t s = millis() / 1000;
  Serial.printf("---- [%02u:%02u] Node %c  state %s  pump %s ----\n", (unsigned)(s / 60),
                (unsigned)(s % 60), MY_ID, STATE_NAMES[state], pumpOn ? "ON" : "OFF");
  for (int i = 0; i < 3; i++) {
    Peer& p = peers[i];
    if (i == idx(MY_ID)) {
      Serial.printf("%c %-12s ALIVE (self)  raw %d  %s\n", 'A' + i, ZONES[i],
                    myRaw, myWet ? "WET" : "dry");
    } else if (!p.everHeard) {
      Serial.printf("%c %-12s WAITING\n", 'A' + i, ZONES[i]);
    } else {
      Serial.printf("%c %-12s %s  last %.1fs  level %d%%  state %s  via %c\n", 'A' + i,
                    ZONES[i], p.lost ? "LOST " : "ALIVE",
                    (millis() - p.lastHeard) / 1000.0, p.level,
                    p.st, p.via);
    }
  }
  if (state == SAFE && peerAlive(1) && millis() - lastUnsafe >= HOLD_MS && REQUIRE_ACK) {
    Serial.println("Node B is stable. Press the ACK button (or type 'a') to leave SAFE.");
  }
}

void setup() {
  Serial.begin(115200);              // set the Serial Monitor to 115200 baud
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);        // pump OFF at boot
  digitalWrite(IN2_PIN, LOW);
  pinMode(ACK_PIN, INPUT_PULLUP);    // ADDED

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
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.printf("Node %c up, zone %s\n", MY_ID, ZONES[idx(MY_ID)]);
}

void loop() {
  RxMsg m;
  while (xQueueReceive(rxQueue, &m, 0) == pdTRUE) handleMsg(m);

  uint32_t now = millis();
  if (now - lastSense >= SENSE_MS) {
    lastSense = now;
    readSensor();
  }
  if (now - lastBeat >= HEARTBEAT_MS) {
    lastBeat = now;
    sendHeartbeat();
  }
  checkTimeouts();
  updateState();                     // ADDED
  drivePump();                       // ADDED
  checkAck();                        // ADDED
  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    printDashboard();
  }
}
