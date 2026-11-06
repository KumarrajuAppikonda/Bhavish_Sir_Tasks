
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <FastLED.h>
#include <time.h>
#include <sys/time.h>

// ============================================================================
//  Configuration
// ============================================================================
constexpr uint8_t  LED_PIN        = 5;
constexpr uint8_t  NUM_LEDS       = 8;
constexpr uint8_t  BUTTON_PIN     = 4;
constexpr uint8_t  LED_BRIGHTNESS = 60;            // 0-255

constexpr uint32_t RETRY_WINDOW_MS    = 300000UL;  // RED: keep checking saved network: 5 minutes
constexpr uint32_t RETRY_INTERVAL_MS  = 10000UL;   // one new attempt every 10 s
constexpr uint32_t INDICATE_MS        = 300000UL;  // GREEN indication: 5 minutes
constexpr uint32_t AP_WAIT_MS         = 300000UL;  // AP waiting for a TCP client: 5 minutes
constexpr uint32_t CONFIG_WINDOW_MS   = 300000UL;  // configuration window after a client connects
constexpr uint32_t CONNECT_TIMEOUT_MS = 20000UL;   // test of a newly entered router
constexpr uint32_t PROGRESS_MS        = 2000UL;    // PROGRESS line interval while connecting
constexpr uint32_t LINK_LOST_MS       = 3000UL;    // router gone this long = link lost
constexpr uint32_t CLIENT_LOST_MS     = 2000UL;    // grace time before "client left" is accepted
constexpr uint32_t AP_ANNOUNCE_MS     = 2000UL;    // VIOLET shown this long when AP starts while connected
constexpr uint32_t STATUS_FLASH_MS    = 2000UL;    // short-press status LED
constexpr uint32_t RED_BLINK_MS       = 1200UL;    // RED once after a failed new network
constexpr uint32_t FORGET_FLASH_MS    = 1500UL;    // GREEN (ok) / RED (failed) after a forget
constexpr uint32_t LONG_PRESS_MS      = 3000UL;    // AP mode = STRICTLY MORE than 3 s
constexpr uint32_t FORGET_PRESS_MS    = 10000UL;   // forget saved Wi-Fi = 10 s
constexpr uint32_t DEBOUNCE_MS        = 30UL;
constexpr uint32_t RESTART_DELAY_MS   = 500UL;     // let the RESTART reply leave first

constexpr uint32_t TIME_RETRY_MS      = 15000UL;   // retry until first sync
constexpr uint32_t TIME_RESYNC_MS     = 3600000UL; // then every 1 hour

constexpr uint16_t TCP_PORT           = 5000;
constexpr size_t   RX_MAX             = 160;       // max characters in one command line
constexpr uint8_t  RX_BYTES_PER_LOOP  = 128;       // keeps loop() responsive

const char* const AP_SSID = "ESP32-WiFi-Setup";
const char* const AP_PASS = "esp32setup";          // >= 8 chars; "" for an open AP

constexpr long GMT_OFFSET_SEC = 19800;             // IST (UTC+05:30)

const char* const NVS_NAMESPACE = "wifi";
const char* const NVS_KEY_SSID  = "ssid";
const char* const NVS_KEY_PASS  = "pass";

// ============================================================================
//  Types
// ============================================================================
enum State : uint8_t {
  ST_BOOT,
  ST_CHECK_NVS,
  ST_WIFI_CONNECTED,     // GREEN 5 minutes
  ST_WIFI_FAILED,        // NOT CONNECTED: RED, checking the saved network for 5 minutes
  ST_AP_WAITING,         // AP + TCP on, VIOLET, waiting for a TCP client
  ST_CLIENT_CONNECTED,   // TCP client connected, BLUE
  ST_TCP_CONFIGURATION,  // client is sending configuration commands
  ST_WIFI_CONNECTING,    // testing the newly entered router
  ST_CONFIG_SUCCESS,     // new network connected: GREEN (AP stays, more networks can be added)
  ST_NORMAL_IDLE
};

enum LedColor : uint8_t { LED_OFF, LED_VIOLET, LED_BLUE, LED_GREEN, LED_RED };

// ============================================================================
//  Globals
// ============================================================================
CRGB        leds[NUM_LEDS];
Preferences prefs;
WiFiServer  tcpServer(TCP_PORT);
WiFiClient  tcpClient;

State state = ST_BOOT;

// credentials
String savedSsid, savedPass;                // what is in NVS
String pendingSsid, pendingPass;            // new network from SET_WIFI (RAM only)
bool   pendingValid     = false;            // SET_WIFI accepted
bool   pendingConnected = false;            // pending network connected, not yet saved
String connectFailReason;

// timers
uint32_t retryStart    = 0;
uint32_t lastAttemptMs = 0;
uint32_t apStart       = 0;
uint32_t cfgStart      = 0;
uint32_t connectStart  = 0;
uint32_t lastProgress  = 0;
uint32_t indicateStart = 0;
uint32_t linkLostSince = 0;
bool     monitorLink   = false;     // supervise the router link (connected / idle only)

// AP / TCP
bool     apActive          = false;
bool     tcpActive         = false;
bool     manualAp          = false;
bool     bootSearch        = false;       // boot AP: saved network is tried in the background
uint8_t  apClients         = 0;           // Wi-Fi stations on the AP (informational)
uint32_t clientAbsentSince = 0;
bool     tcpWasConnected   = false;
String   rxBuf;
bool     rxOverflow        = false;
bool     pendingRestart    = false;
uint32_t pendingRestartAt  = 0;

// LED
LedColor ledColor    = LED_OFF;     // colour requested by the state machine
LedColor ledApplied  = LED_OFF;     // colour currently on the strip
bool     flashActive = false;       // short flash (status / red blink / forget result)
LedColor flashColor  = LED_OFF;
uint32_t flashStart  = 0;
uint32_t flashDur    = STATUS_FLASH_MS;

// button
bool     btnRaw        = false;
bool     btnPressed    = false;
bool     apHandled     = false;     // 3 s action done for this press
bool     forgetHandled = false;     // 10 s action done for this press
uint32_t btnChangeMs   = 0;
uint32_t pressStartMs  = 0;
uint32_t holdMs        = 0;

// Google HTTP time sync
bool          timeSynced     = false;
uint32_t      lastSyncOk     = 0;
uint32_t      lastSyncTry    = 0;
bool          syncFailLogged = false;
volatile bool syncBusy       = false;
volatile bool syncDone       = false;
volatile bool syncResult     = false;

// serial status line / commands
uint32_t lastStatusMs = 0;
bool     statusForce  = true;
size_t   lastLineLen  = 0;
String   cmdBuf;

// ============================================================================
//  Forward declarations (loop services)
// ============================================================================
void handleButton();
void checkWiFiConnection();
void handleAPClients();
void handleAPTimeout();
void updateLEDStatus();
void handleTcpServer();
void printSerialStatus();
void handleTimeSync();
void handleSerialCommands();
void handlePendingRestart();

// ============================================================================
//  Helpers
// ============================================================================
static uint32_t remainingMs(uint32_t start, uint32_t duration) {
  uint32_t elapsed = millis() - start;
  return (elapsed >= duration) ? 0 : (duration - elapsed);
}

static String mmss(uint32_t ms) {
  uint32_t s = (ms + 999) / 1000;
  char b[8];
  snprintf(b, sizeof(b), "%02lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
  return String(b);
}

// Escape a value for the TCP protocol: '|' -> "\|", '\' -> "\\", control chars -> space.
static String escField(const String& in) {
  String o;
  o.reserve(in.length() + 4);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '|' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c < 0x20) { o += ' '; }
    else o += c;
  }
  return o;
}

static const char* stateName() {
  switch (state) {
    case ST_BOOT:              return "BOOT";
    case ST_CHECK_NVS:         return "CHECK_NVS";
    case ST_WIFI_CONNECTED:    return "WIFI_CONNECTED";
    case ST_WIFI_FAILED:       return "NOT_CONNECTED";
    case ST_AP_WAITING:        return manualAp ? "MANUAL_AP" : "AP_WAITING";
    case ST_CLIENT_CONNECTED:  return "CLIENT_CONNECTED";
    case ST_TCP_CONFIGURATION: return "TCP_CONFIGURATION";
    case ST_WIFI_CONNECTING:   return "WIFI_CONNECTING";
    case ST_CONFIG_SUCCESS:    return "CONFIG_SUCCESS";
    case ST_NORMAL_IDLE:       return "NORMAL_IDLE";
  }
  return "?";
}

// ---- Date / time (system clock is set from the Google HTTP "Date" header) ---
static String timeString() {
  if (timeSynced) {
    time_t t = time(nullptr) + GMT_OFFSET_SEC;     // local time = UTC + offset
    struct tm ti;
    gmtime_r(&t, &ti);
    char b[24];
    strftime(b, sizeof(b), "%d-%b-%Y %H:%M:%S", &ti);
    return String(b);
  }
  return String("DATE/TIME NOT SYNCED");
}

// ---- Event log (own line; the live status line is wiped first) --------------
static void logEvent(const String& msg) {
  Serial.print('\r');
  for (size_t i = 0; i < lastLineLen; i++) Serial.print(' ');
  Serial.print('\r');
  Serial.print('[');
  Serial.print(timeString());
  Serial.print("] EVENT: ");
  Serial.println(msg);
  lastLineLen = 0;
  statusForce = true;
}

static void setState(State s) {
  if (s == state) return;
  state = s;
  logEvent(String("State -> ") + stateName());
}

// ============================================================================
//  TCP I/O primitives
// ============================================================================
static bool tcpHasClient() {
  return tcpActive && tcpClient && tcpClient.connected();
}

// One protocol line, written in a single TCP segment.
static void tcpSend(const String& line) {
  if (!tcpHasClient()) return;
  String s = line;
  s += '\n';
  tcpClient.write((const uint8_t*)s.c_str(), s.length());
}

// ---- LED --------------------------------------------------------------------
static void setLed(LedColor c) { ledColor = c; }

static void startIndication(LedColor c) {          // GREEN / RED timing is kept in indicateStart
  ledColor      = c;
  indicateStart = millis();
}

static void startFlash(LedColor c, uint32_t ms) {  // short overlay colour
  flashColor  = c;
  flashStart  = millis();
  flashDur    = ms;
  flashActive = true;
}

// Colour actually shown on the strip.
static LedColor effectiveLed() {
  if (flashActive) return flashColor;              // short flash overrides everything
  if (state == ST_AP_WAITING && ledColor == LED_VIOLET &&
      WiFi.status() == WL_CONNECTED && (millis() - apStart) >= AP_ANNOUNCE_MS)
    return LED_GREEN;                              // AP on + router connected: VIOLET 2 s, then GREEN
  if (state == ST_CONFIG_SUCCESS && WiFi.status() != WL_CONNECTED)
    return tcpHasClient() ? LED_BLUE : LED_VIOLET; // router gone: back to the AP colours
  return ledColor;
}

static const char* ledName() {
  switch (effectiveLed()) {
    case LED_VIOLET: return "VIOLET";
    case LED_BLUE:   return "BLUE";
    case LED_GREEN:  return "GREEN";
    case LED_RED:    return "RED";
    default:         return "OFF";
  }
}

// Short press: GREEN once if connected, RED once if not.
static void showStatusFlash() {
  bool up = (WiFi.status() == WL_CONNECTED);
  startFlash(up ? LED_GREEN : LED_RED, STATUS_FLASH_MS);
  logEvent(up ? String("Status check: connected to ") + WiFi.SSID()
              : String("Status check: NOT connected"));
}

// ============================================================================
//  NVS credentials
// ============================================================================
static bool credentialsValid() {
  return savedSsid.length() > 0;                   // password may be empty only for open networks
}

static void loadCredentials() {
  prefs.begin(NVS_NAMESPACE, false);               // read/write open creates a missing namespace
  savedSsid = prefs.getString(NVS_KEY_SSID, "");
  savedPass = prefs.getString(NVS_KEY_PASS, "");
  prefs.end();
}

// Called ONLY after a router connection succeeded, so a failed attempt never overwrites NVS.
static bool saveCredentials(const String& ssid, const String& pass) {
  prefs.begin(NVS_NAMESPACE, false);
  size_t w1 = prefs.putString(NVS_KEY_SSID, ssid);
  size_t w2 = prefs.putString(NVS_KEY_PASS, pass);
  prefs.end();
  if (w1 == 0 && ssid.length() > 0) return false;
  if (w2 == 0 && pass.length() > 0) return false;

  prefs.begin(NVS_NAMESPACE, true);                // read back and verify
  String rs = prefs.getString(NVS_KEY_SSID, "");
  String rp = prefs.getString(NVS_KEY_PASS, "");
  prefs.end();
  return (rs == ssid) && (rp == pass);
}

static bool eraseCredentials() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.clear();                                   // erase every key of the "wifi" namespace
  prefs.end();
  savedSsid = "";
  savedPass = "";

  prefs.begin(NVS_NAMESPACE, false);               // verify
  bool empty = (prefs.getString(NVS_KEY_SSID, "").length() == 0) &&
               (prefs.getString(NVS_KEY_PASS, "").length() == 0);
  prefs.end();
  return empty;
}

// ============================================================================
//  Wi-Fi / AP control
// ============================================================================

// One non-blocking connection attempt to the saved network.
static void beginSavedAttempt() {
  WiFi.disconnect(false);
  WiFi.begin(savedSsid.c_str(), savedPass.c_str());
  lastAttemptMs = millis();
}

// Keep trying the saved network (own timer, independent of driver auto-reconnect).
static void retryTick() {
  if (WiFi.status() != WL_CONNECTED && (millis() - lastAttemptMs) >= RETRY_INTERVAL_MS)
    beginSavedAttempt();
}

// NOT CONNECTED mode: RED while the saved network is tried for 5 minutes.
static void startRetry() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  retryStart    = millis();
  lastAttemptMs = millis() - RETRY_INTERVAL_MS;    // first attempt right away
  monitorLink   = false;
  setLed(LED_RED);
  logEvent("Not connected: trying the saved network for 5 minutes");
  setState(ST_WIFI_FAILED);
}

// manual    : true when started by a long press
// searchSaved: true at boot -> saved network is tried in the background while the AP waits
static void startAP(bool manual, bool searchSaved) {
  // An EXISTING router link stays up, so the old network remains connected until the
  // user successfully switches to a new one.
  monitorLink = false;
  if (WiFi.status() != WL_CONNECTED) WiFi.disconnect(false);

  if (!apActive) {
    WiFi.mode(WIFI_AP_STA);                        // TCP stays reachable while a router is tested
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                      IPAddress(255, 255, 255, 0));
    bool ok = (strlen(AP_PASS) >= 8) ? WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4)
                                     : WiFi.softAP(AP_SSID);
    if (!ok) logEvent("softAP start FAILED");

    tcpServer.begin();                             // TCP server on port 5000
    tcpActive = true;
    apActive  = true;
  }

  manualAp          = manual;
  apClients         = 0;
  clientAbsentSince = 0;
  apStart           = millis();
  setLed(LED_VIOLET);

  bootSearch = false;
  if (searchSaved && credentialsValid()) {         // boot: look for the saved network meanwhile
    WiFi.setAutoReconnect(false);
    beginSavedAttempt();
    bootSearch = true;
  }

  logEvent(String(manual ? "Manual" : "Boot") + " AP mode ON, SSID: " + AP_SSID +
           ", IP: " + WiFi.softAPIP().toString() + ", TCP port: " + String(TCP_PORT) +
           (bootSearch ? " (saved network is also being tried)" : ""));
  setState(ST_AP_WAITING);
}

static void stopAP() {
  bool routerUp = (WiFi.status() == WL_CONNECTED);
  if (tcpClient) tcpClient.stop();
  tcpServer.stop();
  tcpActive       = false;
  tcpWasConnected = false;
  rxBuf           = "";
  rxOverflow      = false;
  WiFi.softAPdisconnect(true);
  WiFi.mode(routerUp ? WIFI_STA : WIFI_OFF);
  apActive   = false;
  bootSearch = false;
  apClients  = 0;
  logEvent("AP mode OFF, TCP server OFF");
}

// AP session ended without switching to a new network.
static void endApSession() {
  setLed(LED_OFF);
  stopAP();
  if (WiFi.status() == WL_CONNECTED) {             // previous network still connected
    monitorLink   = true;
    linkLostSince = 0;
    setState(ST_NORMAL_IDLE);
  } else if (credentialsValid()) {                 // NOT CONNECTED mode: RED, keep trying saved network
    startRetry();
  } else {
    setState(ST_NORMAL_IDLE);
  }
}

// Back to AP mode with the right colour, AP and TCP stay ON.
// BLUE if a TCP client is connected, VIOLET if not.
static void returnToApMode() {
  apClients         = WiFi.softAPgetStationNum();
  clientAbsentSince = 0;
  bootSearch        = false;
  if (tcpHasClient()) {
    cfgStart = millis();                           // fresh 5 minute configuration window
    setLed(LED_BLUE);
    setState(ST_CLIENT_CONNECTED);
  } else {
    apStart = millis();                            // fresh 5 minute wait for a client
    setLed(LED_VIOLET);
    setState(ST_AP_WAITING);
  }
}

// Newly entered router failed. NVS is untouched.
static void failConfig(const String& reason) {
  connectFailReason = reason;
  pendingConnected  = false;
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false);
  logEvent("New Wi-Fi failed: " + reason + " (previous NVS credentials preserved)");
  tcpSend("WIFI_FAILED|" + reason);
  startFlash(LED_RED, RED_BLINK_MS);               // RED once ...
  returnToApMode();                                // ... then BLUE / VIOLET
}

// Forget the saved network (TCP FORGET_WIFI, 10 s button, serial CLEARNVS).
// Returns true when NVS was erased AND verified empty.
static bool forgetSavedNetwork() {
  bool ok = eraseCredentials();
  logEvent(ok ? "NVS cleared and verified: saved Wi-Fi credentials erased"
              : "NVS clear FAILED");

  pendingValid     = false;
  pendingConnected = false;
  pendingSsid      = "";
  pendingPass      = "";
  monitorLink      = false;
  linkLostSince    = 0;
  bootSearch       = false;
  flashActive      = false;
  WiFi.setAutoReconnect(false);
  if ((WiFi.getMode() & WIFI_MODE_STA) != 0) WiFi.disconnect(false);   // drop the old router

  if (apActive) returnToApMode();                  // keep the AP, TCP server and client
  else          startAP(false, false);             // AP was off: open it so a new network can be added

  startFlash(ok ? LED_GREEN : LED_RED, FORGET_FLASH_MS);   // GREEN = erased, RED = failed
  return ok;
}

// ============================================================================
//  TCP command parser
// ============================================================================

// Split on unescaped '|'. Escapes: "\|" -> '|', "\\" -> '\'. Other "\x" stays as typed.
// Returns the number of fields, or -1 if there are more than maxF.
static int splitFields(const String& line, String* out, int maxF) {
  int n = 0;
  String cur;
  bool esc = false;
  for (size_t i = 0; i < line.length(); i++) {
    char c = line[i];
    if (esc) {
      if (c == '|' || c == '\\') cur += c;
      else { cur += '\\'; cur += c; }
      esc = false;
    } else if (c == '\\') {
      esc = true;
    } else if (c == '|') {
      if (n >= maxF) return -1;
      out[n++] = cur;
      cur = "";
    } else {
      cur += c;
    }
  }
  if (esc) cur += '\\';                            // trailing backslash is literal
  if (n >= maxF) return -1;
  out[n++] = cur;
  return n;
}

static bool hasControlChars(const String& s) {
  for (size_t i = 0; i < s.length(); i++) {
    uint8_t c = (uint8_t)s[i];
    if (c < 0x20 || c == 0x7F) return true;
  }
  return false;
}

static bool linkToSaved() {
  return credentialsValid() && WiFi.status() == WL_CONNECTED && WiFi.SSID() == savedSsid;
}

static String buildStatus() {
  bool up = (WiFi.status() == WL_CONNECTED);
  String s = "STATUS|STATE=";  s += stateName();
  s += "|AP=";                 s += apActive ? "ON" : "OFF";
  s += "|AP_SSID=";            s += escField(String(AP_SSID));
  s += "|AP_IP=";              s += apActive ? WiFi.softAPIP().toString() : String("-");
  s += "|AP_CLIENTS=";         s += String(apActive ? WiFi.softAPgetStationNum() : 0);
  s += "|ROUTER=";             s += up ? "CONNECTED" : (state == ST_WIFI_CONNECTING ? "CONNECTING" : "DISCONNECTED");
  s += "|SSID=";               s += up ? escField(WiFi.SSID()) : String("-");
  s += "|ROUTER_IP=";          s += up ? WiFi.localIP().toString() : String("-");
  s += "|TCP=";                s += tcpActive ? "ON" : "OFF";
  s += "|TCP_PORT=";           s += String(TCP_PORT);
  s += "|LED=";                s += ledName();
  return s;
}

static String buildWifiInfo() {
  String s = "WIFI_INFO|STORED=";  s += credentialsValid() ? "YES" : "NO";
  s += "|SSID=";                   s += credentialsValid() ? escField(savedSsid) : String("-");
  s += "|CONNECTED=";              s += linkToSaved() ? "YES" : "NO";
  s += "|PASSWORD=HIDDEN";
  return s;
}

static void sendHelp() {
  tcpSend("HELP|GET_STATUS                  - state, AP, router, TCP info");
  tcpSend("HELP|GET_WIFI_INFO               - saved SSID, stored?, connected to it? (password never shown)");
  tcpSend("HELP|SET_WIFI|ssid|password      - set new network in RAM (SSID 1-32, password 8-63 or empty)");
  tcpSend("HELP|CONNECT_WIFI                - test the new network (20 s timeout, progress is reported)");
  tcpSend("HELP|SAVE_CONFIG                 - after WIFI_CONNECTED: write to NVS and verify");
  tcpSend("HELP|FORGET_WIFI                 - erase saved SSID/password, keep AP + TCP");
  tcpSend("HELP|RESTART                     - controlled restart (maintenance only)");
  tcpSend("HELP|HELP                        - this list");
  tcpSend("HELP|Use \\| for a literal pipe and \\\\ for a literal backslash inside SSID/password");
  tcpSend("HELP|END");
}

static void cmdSetWifi(String* f, int n) {
  if (state == ST_WIFI_CONNECTING) { tcpSend("ERROR|BUSY"); return; }
  if (n < 2 || n > 3)              { tcpSend("ERROR|INVALID_ARGUMENTS"); return; }

  const String& ssid = f[1];
  String pass = (n == 3) ? f[2] : String("");

  if (ssid.length() < 1 || ssid.length() > 32)  { tcpSend("ERROR|INVALID_SSID_LENGTH"); return; }
  if (pass.length() > 63 || (pass.length() > 0 && pass.length() < 8)) {
    tcpSend("ERROR|INVALID_PASSWORD_LENGTH");
    return;
  }
  if (hasControlChars(ssid) || hasControlChars(pass)) { tcpSend("ERROR|INVALID_CHARACTERS"); return; }

  pendingSsid       = ssid;
  pendingPass       = pass;
  pendingValid      = true;
  pendingConnected  = false;
  connectFailReason = "";
  logEvent("New Wi-Fi received over TCP: " + pendingSsid + " (password hidden)");
  tcpSend("OK|NEW_WIFI_RECEIVED");
}

static void cmdConnectWifi() {
  if (state == ST_WIFI_CONNECTING) { tcpSend("ERROR|BUSY"); return; }
  if (!pendingValid)               { tcpSend("ERROR|NO_PENDING_WIFI"); return; }
  if (!apActive)                   { tcpSend("ERROR|BUSY"); return; }

  connectFailReason = "";
  pendingConnected  = false;
  flashActive       = false;
  bootSearch        = false;
  monitorLink       = false;

  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false);                          // leave the old router, test the new one
  WiFi.begin(pendingSsid.c_str(), pendingPass.c_str());
  connectStart = millis();
  lastProgress = connectStart;
  setLed(LED_BLUE);                                // BLUE while connecting
  logEvent("Connecting to new router: " + pendingSsid);
  setState(ST_WIFI_CONNECTING);
  tcpSend("WIFI_CONNECTING");
}

static void cmdSaveConfig() {
  if (state == ST_WIFI_CONNECTING) { tcpSend("ERROR|BUSY"); return; }
  if (!pendingValid)               { tcpSend("ERROR|NO_PENDING_WIFI"); return; }
  if (!pendingConnected || WiFi.status() != WL_CONNECTED || WiFi.SSID() != pendingSsid) {
    tcpSend("ERROR|NOT_CONNECTED_TO_NEW_WIFI");
    return;
  }
  if (credentialsValid() && savedSsid == pendingSsid && savedPass == pendingPass) {
    pendingConnected = false;                      // already stored: no repeated NVS write
    tcpSend("SAVE_SUCCESS|ALREADY_SAVED");
    return;
  }
  if (saveCredentials(pendingSsid, pendingPass)) {
    savedSsid        = pendingSsid;                // RAM copy updated only after verification
    savedPass        = pendingPass;
    pendingConnected = false;
    logEvent("Credentials saved to NVS and verified: " + savedSsid);
    tcpSend("SAVE_SUCCESS");
  } else {
    logEvent("NVS save/verify FAILED (previous credentials kept in RAM)");
    loadCredentials();                             // RAM copy mirrors whatever NVS really holds
    tcpSend("ERROR|SAVE_FAILED");
  }
}

static void cmdForgetWifi() {
  if (state == ST_WIFI_CONNECTING) { tcpSend("ERROR|BUSY"); return; }
  bool ok = forgetSavedNetwork();
  tcpSend(ok ? "FORGET_SUCCESS" : "ERROR|FORGET_FAILED");
}

static void processCommand(const String& line) {
  String f[3];
  int n = splitFields(line, f, 3);
  if (n < 0) { tcpSend("ERROR|TOO_MANY_FIELDS"); return; }

  String cmd = f[0];
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) { tcpSend("ERROR|INVALID_COMMAND"); return; }

  logEvent("TCP command: " + cmd);                 // never log arguments (password)

  // any valid line keeps the configuration window alive
  if (state == ST_CLIENT_CONNECTED || state == ST_TCP_CONFIGURATION) cfgStart = millis();
  if (state == ST_CLIENT_CONNECTED) setState(ST_TCP_CONFIGURATION);

  if (cmd == "GET_STATUS") {
    if (n != 1) { tcpSend("ERROR|INVALID_ARGUMENTS"); return; }
    tcpSend(buildStatus());
  } else if (cmd == "GET_WIFI_INFO") {
    if (n != 1) { tcpSend("ERROR|INVALID_ARGUMENTS"); return; }
    tcpSend(buildWifiInfo());
  } else if (cmd == "SET_WIFI") {
    cmdSetWifi(f, n);
  } else if (cmd == "CONNECT_WIFI") {
    if (n != 1) { tcpSend("ERROR|INVALID_ARGUMENTS"); return; }
    cmdConnectWifi();
  } else if (cmd == "SAVE_CONFIG") {
    if (n != 1) { tcpSend("ERROR|INVALID_ARGUMENTS"); return; }
    cmdSaveConfig();
  } else if (cmd == "FORGET_WIFI") {
    if (n != 1) { tcpSend("ERROR|INVALID_ARGUMENTS"); return; }
    cmdForgetWifi();
  } else if (cmd == "RESTART") {
    if (n != 1) { tcpSend("ERROR|INVALID_ARGUMENTS"); return; }
    tcpSend("OK|RESTARTING");
    pendingRestart   = true;
    pendingRestartAt = millis();
    logEvent("Restart requested over TCP");
  } else if (cmd == "HELP") {
    sendHelp();
  } else {
    tcpSend("ERROR|INVALID_COMMAND");
  }
}

// ---- TCP receive: byte-wise line assembly (a command may span or share packets) ----
static void tcpReceive() {
  if (!tcpHasClient()) return;
  uint8_t budget = RX_BYTES_PER_LOOP;
  while (budget-- > 0 && tcpClient.available() > 0) {
    int ci = tcpClient.read();
    if (ci < 0) break;
    char ch = (char)ci;

    if (ch == '\n') {
      if (rxOverflow) {
        rxOverflow = false;
        rxBuf = "";
        tcpSend("ERROR|LINE_TOO_LONG");
      } else if (rxBuf.length() > 0) {
        String l = rxBuf;
        rxBuf = "";
        processCommand(l);
        if (!tcpHasClient()) break;
      }
    } else if (ch == '\r' || rxOverflow) {
      // ignore CR; while overflowed, discard until the newline
    } else if (rxBuf.length() >= RX_MAX) {
      rxOverflow = true;
      rxBuf = "";
    } else {
      rxBuf += ch;
    }
  }
}

// ---- TCP accept: a new client replaces the old one (handles reconnects / dead sockets) ----
static void tcpAccept() {
  WiFiClient nc = tcpServer.available();
  if (!nc) return;

  if (tcpClient && tcpClient.connected()) {
    String bye = "BYE|REPLACED_BY_NEW_CLIENT\n";
    tcpClient.write((const uint8_t*)bye.c_str(), bye.length());
    tcpClient.stop();
  }
  tcpClient = nc;
  tcpClient.setNoDelay(true);
  tcpWasConnected = true;
  rxBuf           = "";
  rxOverflow      = false;
  statusForce     = true;
  logEvent("TCP client connected: " + tcpClient.remoteIP().toString());
  tcpSend(String("HELLO|") + AP_SSID + "|TCP_PORT=" + String(TCP_PORT) + "|SEND_HELP");
}

void handleTcpServer() {
  if (!tcpActive) return;

  tcpAccept();

  bool conn = tcpClient && tcpClient.connected();
  if (tcpWasConnected && !conn) {                  // client dropped: clean up, ready for a new one
    tcpClient.stop();
    tcpWasConnected = false;
    rxBuf           = "";
    rxOverflow      = false;
    statusForce     = true;
    logEvent("TCP client disconnected");
  }

  tcpReceive();
}

void handlePendingRestart() {
  if (!pendingRestart || (millis() - pendingRestartAt) < RESTART_DELAY_MS) return;
  pendingRestart = false;
  if (tcpClient) { tcpClient.flush(); tcpClient.stop(); }
  delay(50);                                       // one-time, right before the restart
  ESP.restart();
}

// ============================================================================
//  Serial command: CLEARNVS  (same as TCP FORGET_WIFI)
// ============================================================================
static void runCommand(String c) {
  c.trim();
  c.toUpperCase();
  if (c.length() == 0) return;
  if (c == "CLEARNVS") {
    if (state == ST_WIFI_CONNECTING) logEvent("Busy testing a new network, try again in a moment");
    else                             forgetSavedNetwork();
  } else if (c == "HELP") {
    logEvent("Commands: CLEARNVS (erase saved Wi-Fi) | HELP");
  } else {
    logEvent("Unknown command: " + c + "  (type HELP)");
  }
}

void handleSerialCommands() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdBuf.length()) { String c = cmdBuf; cmdBuf = ""; runCommand(c); }
    } else if (cmdBuf.length() < 32) {
      cmdBuf += ch;
    }
  }
}

// ============================================================================
//  Time from Google over plain HTTP ("Date:" response header)
// ============================================================================
static int monthFromName(const char* m) {
  static const char* names[12] = {"Jan","Feb","Mar","Apr","May","Jun",
                                  "Jul","Aug","Sep","Oct","Nov","Dec"};
  for (int i = 0; i < 12; i++) if (strncmp(m, names[i], 3) == 0) return i + 1;
  return 0;
}

static int64_t daysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

// Parses e.g. "Sat, 31 Oct 2026 05:18:00 GMT" -> UTC epoch seconds
static bool parseHttpDate(const String& s, time_t& out) {
  int d, y, hh, mm, ss;
  char mon[4] = {0};
  if (sscanf(s.c_str(), "%*3s, %d %3s %d %d:%d:%d", &d, mon, &y, &hh, &mm, &ss) != 6) return false;
  int m = monthFromName(mon);
  if (m == 0 || y < 2024 || d < 1 || d > 31) return false;
  out = (time_t)(daysFromCivil(y, (unsigned)m, (unsigned)d) * 86400LL + hh * 3600 + mm * 60 + ss);
  return true;
}

static bool syncTimeFromGoogleHttp() {
  static const char* urls[] = {
    "http://clients3.google.com/generate_204",
    "http://www.google.com"
  };
  const char* keys[] = {"Date"};

  for (int i = 0; i < 2; i++) {
    if (WiFi.status() != WL_CONNECTED) return false;
    HTTPClient http;
    http.setConnectTimeout(4000);
    http.setTimeout(4000);
    http.setReuse(false);
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    if (!http.begin(urls[i])) continue;
    http.collectHeaders(keys, 1);
    int code = http.GET();
    if (code > 0 && http.hasHeader("Date")) {
      time_t t;
      if (parseHttpDate(http.header("Date"), t)) {
        struct timeval tv = { t, 0 };
        settimeofday(&tv, nullptr);
        http.end();
        return true;
      }
    }
    http.end();
  }
  return false;
}

// Runs once in its own task so the main loop (Wi-Fi state machine, LEDs, button, TCP) never blocks.
static void timeSyncTask(void*) {
  syncResult = syncTimeFromGoogleHttp();
  syncDone   = true;
  syncBusy   = false;
  vTaskDelete(nullptr);
}

void handleTimeSync() {
  if (syncDone) {                                  // collect the result (log only from loop())
    syncDone = false;
    if (syncResult) {
      timeSynced     = true;
      lastSyncOk     = millis();
      syncFailLogged = false;
      logEvent("Time synchronized (Google HTTP)");
    } else if (!syncFailLogged) {
      syncFailLogged = true;
      logEvent("Time sync failed, will retry");
    }
  }

  if (syncBusy || WiFi.status() != WL_CONNECTED) return;

  uint32_t now = millis();
  bool due = timeSynced ? (now - lastSyncOk >= TIME_RESYNC_MS)
                        : (lastSyncTry == 0 || now - lastSyncTry >= TIME_RETRY_MS);
  if (!due) return;

  lastSyncTry = now ? now : 1;
  syncBusy = true;
  if (xTaskCreate(timeSyncTask, "timeSync", 8192, nullptr, 1, nullptr) != pdPASS) {
    syncBusy = false;
  }
}

// ============================================================================
//  Loop services
// ============================================================================

// ---- Button: short = status | hold > 3 s = AP mode | hold >= 10 s = forget Wi-Fi ----
static bool manualApAllowed() {
  // every state except: AP already waiting, client configuring, new router being tested
  return state == ST_WIFI_CONNECTED || state == ST_WIFI_FAILED ||
         state == ST_NORMAL_IDLE || state == ST_CONFIG_SUCCESS;
}

void handleButton() {
  uint32_t now = millis();
  bool raw = (digitalRead(BUTTON_PIN) == LOW);

  if (raw != btnRaw) { btnRaw = raw; btnChangeMs = now; }

  if ((now - btnChangeMs) >= DEBOUNCE_MS && raw != btnPressed) {
    btnPressed  = raw;
    statusForce = true;
    if (btnPressed) {
      pressStartMs  = now;
      holdMs        = 0;
      apHandled     = false;
      forgetHandled = false;
    } else {
      // short press only: a long hold already did its job, never add a status flash
      if (!apHandled && !forgetHandled && state != ST_WIFI_CONNECTING) showStatusFlash();
      holdMs = 0;
    }
  }

  if (btnPressed) {
    holdMs = now - pressStartMs;

    // 10 s: forget. Takes priority and runs exactly once per press.
    if (!forgetHandled && holdMs >= FORGET_PRESS_MS) {
      forgetHandled = true;
      apHandled     = true;                        // the 3 s action can never run after this
      if (state == ST_WIFI_CONNECTING) {
        logEvent("10 s hold ignored: busy testing a new network");
      } else {
        logEvent("10 s hold detected: forgetting saved Wi-Fi");
        forgetSavedNetwork();
      }
    }
    // 3 s: AP mode. Runs exactly once per press.
    else if (!apHandled && holdMs > LONG_PRESS_MS) {
      apHandled = true;
      if (manualApAllowed()) {
        logEvent("Long press detected (> 3 s): starting manual AP + TCP server");
        startAP(true, false);
      } else {
        logEvent("Long press ignored in this state");
      }
    }
  }
}

// ---- Router connection handling ------------------------------------------------
void checkWiFiConnection() {
  uint32_t now = millis();

  // Link supervision: connected (green or idle) but the router disappeared (hotspot off ...)
  if (monitorLink && (state == ST_WIFI_CONNECTED || state == ST_NORMAL_IDLE)) {
    if (WiFi.status() == WL_CONNECTED) {
      linkLostSince = 0;
    } else {
      if (linkLostSince == 0) linkLostSince = now;
      if (now - linkLostSince >= LINK_LOST_MS) {
        linkLostSince = 0;
        logEvent("Router link lost");
        startRetry();
        return;
      }
    }
  }

  switch (state) {

    case ST_AP_WAITING:                            // AP on, no client: saved network tried in the background
      // AP stays ON for its full 5 minutes so a client can still join.
      // LED: GREEN only while the router is connected, VIOLET when it is not (see effectiveLed).
      if (WiFi.status() == WL_CONNECTED) {
        if (bootSearch) {
          logEvent("Connected to saved network: " + WiFi.SSID() + " (AP stays on until timeout)");
          bootSearch = false;
        }
      } else if (credentialsValid()) {
        retryTick();                               // keep trying; GREEN returns when it connects
      }
      break;

    case ST_WIFI_FAILED:                           // NOT CONNECTED: RED, trying the saved network
      if (WiFi.status() == WL_CONNECTED) {
        logEvent("Connected to saved network: " + WiFi.SSID());
        monitorLink   = true;
        linkLostSince = 0;
        startIndication(LED_GREEN);                // RED off, GREEN 5 min, then LED OFF
        setState(ST_WIFI_CONNECTED);
      } else if (now - retryStart >= RETRY_WINDOW_MS) {
        WiFi.disconnect(false);
        WiFi.mode(WIFI_OFF);                       // stop checking; long press opens AP mode
        setLed(LED_OFF);
        logEvent("Saved network not available within 5 minutes: checking stopped");
        setState(ST_NORMAL_IDLE);
      } else {
        retryTick();
      }
      break;

    case ST_WIFI_CONNECTED:                        // GREEN 5 minutes, then LED OFF
      if (now - indicateStart >= INDICATE_MS) {
        setLed(LED_OFF);
        setState(ST_NORMAL_IDLE);
      }
      break;

    case ST_WIFI_CONNECTING: {                     // testing the router the user just entered
      wl_status_t st = WiFi.status();
      uint32_t elapsed = now - connectStart;
      if (st == WL_CONNECTED) {
        logEvent("Router connected: " + WiFi.SSID() + " (send SAVE_CONFIG to store it)");
        pendingConnected = true;                   // NVS is written only by SAVE_CONFIG
        flashActive      = false;
        startIndication(LED_GREEN);                // GREEN
        setState(ST_CONFIG_SUCCESS);               // AP stays: more networks can still be added
        tcpSend("WIFI_CONNECTED|" + escField(pendingSsid));
        tcpSend("INFO|SEND_SAVE_CONFIG_TO_STORE_THESE_CREDENTIALS");
      } else if (elapsed > 1500 && st == WL_CONNECT_FAILED) {
        failConfig("WRONG_PASSWORD_OR_AUTH_FAILED");
      } else if (elapsed >= CONNECT_TIMEOUT_MS) {
        failConfig(st == WL_NO_SSID_AVAIL ? "NETWORK_NOT_FOUND" : "CONNECTION_TIMED_OUT");
      } else if ((now - lastProgress) >= PROGRESS_MS) {
        lastProgress = now;
        tcpSend("PROGRESS|CONNECTING|" + String(elapsed / 1000) + "s_of_" +
                String(CONNECT_TIMEOUT_MS / 1000) + "s");
      }
      break;
    }

    case ST_CONFIG_SUCCESS:                        // GREEN while connected (max 5 min), then LED OFF, AP OFF
      // router disconnected: LED falls back to VIOLET (no client) / BLUE (client) and the
      // AP keeps running; a new network can be added at any time
      if (WiFi.status() != WL_CONNECTED && !tcpHasClient()) retryTick();
      if (now - indicateStart >= INDICATE_MS) {
        setLed(LED_OFF);
        if (pendingConnected) logEvent("GREEN period over: new network was never saved (SAVE_CONFIG not sent)");
        pendingConnected = false;
        stopAP();
        if (WiFi.status() == WL_CONNECTED) {
          monitorLink   = true;
          linkLostSince = 0;
          setState(ST_NORMAL_IDLE);
        } else {
          startRetry();
        }
      }
      break;

    default:
      break;
  }
}

// ---- AP station count (log) + TCP client presence -> VIOLET / BLUE -----------------
void handleAPClients() {
  if (!apActive) { apClients = 0; return; }

  uint32_t now = millis();
  uint8_t n = WiFi.softAPgetStationNum();
  if (n != apClients) {
    apClients   = n;
    statusForce = true;
    logEvent("AP stations: " + String(n));
  }

  bool client = tcpHasClient();

  switch (state) {
    case ST_AP_WAITING:
      if (client) {
        if (bootSearch) {                          // client has priority: stop background attempts
          bootSearch = false;
          if (WiFi.status() != WL_CONNECTED) WiFi.disconnect(false);
        }
        cfgStart          = now;                   // 5 minute configuration window starts here
        clientAbsentSince = 0;
        setLed(LED_BLUE);
        setState(ST_CLIENT_CONNECTED);
      }
      break;

    case ST_CLIENT_CONNECTED:
    case ST_TCP_CONFIGURATION:
      if (!client) {
        if (clientAbsentSince == 0) clientAbsentSince = now;
        if (now - clientAbsentSince >= CLIENT_LOST_MS) {
          logEvent("TCP client left, waiting again");
          setLed(LED_VIOLET);
          clientAbsentSince = 0;
          apStart = now;
          setState(ST_AP_WAITING);
        }
      } else {
        clientAbsentSince = 0;
      }
      break;

    default:
      break;
  }
}

// ---- AP timeouts ----------------------------------------------------------------
void handleAPTimeout() {
  uint32_t now = millis();

  if (state == ST_AP_WAITING && (now - apStart) >= AP_WAIT_MS) {
    logEvent("No TCP client connected within 5 minutes");
    endApSession();                                // AP OFF -> RED, keep trying saved network
  } else if ((state == ST_CLIENT_CONNECTED || state == ST_TCP_CONFIGURATION) &&
             (now - cfgStart) >= CONFIG_WINDOW_MS) {
    logEvent("Configuration window (5 minutes) expired");
    endApSession();
  }
}

// ---- LED rendering --------------------------------------------------------------
void updateLEDStatus() {
  if (flashActive && (millis() - flashStart) >= flashDur) flashActive = false;

  LedColor target = effectiveLed();
  if (target == ledApplied) return;

  CRGB c = CRGB::Black;
  switch (target) {
    case LED_VIOLET: c = CRGB(148, 0, 211); break;
    case LED_BLUE:   c = CRGB(0, 0, 255);   break;
    case LED_GREEN:  c = CRGB(0, 255, 0);   break;
    case LED_RED:    c = CRGB(255, 0, 0);   break;
    default:         c = CRGB::Black;       break;
  }
  fill_solid(leds, NUM_LEDS, c);
  FastLED.show();
  ledApplied = target;
}

// ---- Live status line (one line, redrawn with '\r') -----------------------------
void printSerialStatus() {
  uint32_t now = millis();
  uint32_t interval = btnPressed ? 200 : 1000;
  if (!statusForce && (now - lastStatusMs) < interval) return;
  statusForce  = false;
  lastStatusMs = now;

  String router;
  if (WiFi.status() == WL_CONNECTED)    router = WiFi.SSID();
  else if (state == ST_WIFI_CONNECTING) router = "CONNECTING";
  else if (bootSearch)                  router = "SEARCHING:" + savedSsid;
  else if (state == ST_WIFI_FAILED)     router = "RETRYING:" + savedSsid;
  else                                  router = "DISCONNECTED";

  String s;
  s.reserve(200);
  s += "[";  s += timeString();  s += "] | STATE: ";  s += stateName();
  s += " | Router: ";   s += router;
  s += " | AP: ";       s += apActive ? "ON" : "OFF";
  s += " | Stations: "; s += String(apClients);
  s += " | TCP: ";      s += tcpActive ? (tcpHasClient() ? "CLIENT" : "LISTEN") : "OFF";

  if (state == ST_WIFI_FAILED) {
    s += " | Retry: ";   s += mmss(remainingMs(retryStart, RETRY_WINDOW_MS));
  }

  if (apActive) {
    uint32_t rem = 0;
    switch (state) {
      case ST_AP_WAITING:          rem = remainingMs(apStart, AP_WAIT_MS);          break;
      case ST_CLIENT_CONNECTED:
      case ST_TCP_CONFIGURATION:
      case ST_WIFI_CONNECTING:     rem = remainingMs(cfgStart, CONFIG_WINDOW_MS);   break;
      case ST_CONFIG_SUCCESS:      rem = remainingMs(indicateStart, INDICATE_MS);   break;
      default: break;
    }
    s += " | AP Remaining: ";  s += mmss(rem);
  }

  s += " | Button: ";  s += btnPressed ? "ON" : "OFF";
  if (btnPressed) {
    char b[32];
    snprintf(b, sizeof(b), " | Hold: %.1fs (3=AP 10=FORGET)", holdMs / 1000.0);
    s += b;
  }
  s += " | LED: ";  s += ledName();

  Serial.print('\r');
  Serial.print(s);
  for (size_t i = s.length(); i < lastLineLen; i++) Serial.print(' ');   // erase leftovers
  lastLineLen = s.length();
}

// ============================================================================
//  setup() / loop()
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);                                      // one-time: let the serial port settle
  Serial.println();
  Serial.println("==== ESP32 Wi-Fi Configuration Manager (TCP) ====");
  Serial.println("Serial command: CLEARNVS (erase saved Wi-Fi) | HELP");

  // LED strip + button
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.clear(true);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  btnRaw = btnPressed = (digitalRead(BUTTON_PIN) == LOW);
  pressStartMs = millis();

  // Wi-Fi (our NVS is the single source of truth: the SDK must not store credentials)
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.disconnect();

  // NVS -> always boot into AP mode (VIOLET, 5 min); saved network is tried meanwhile
  setState(ST_CHECK_NVS);
  loadCredentials();

  if (credentialsValid()) logEvent("Saved credentials found for: " + savedSsid);
  else                    logEvent("No saved credentials");

  startAP(false, true);
}

void loop() {
  handleSerialCommands();
  handleButton();
  checkWiFiConnection();
  handleAPClients();
  handleAPTimeout();
  handleTcpServer();
  handlePendingRestart();
  handleTimeSync();
  updateLEDStatus();
  printSerialStatus();
  delay(1);                                        // yield to the Wi-Fi task
}