/*
 * ESP32 Wi-Fi Configuration and Connection Management  +  MQ-5 ADC (Task 4)
 * PlatformIO | Arduino framework | ESP32 DevKit (ESP32-WROOM-32)
 *
 * Pins : WS2812B data = GPIO 5 (8 LEDs), push button = GPIO 4 (INPUT_PULLUP, active LOW)
 *        MQ-5 analog out = GPIO 34 = ADC1_CHANNEL_6 (input only, 12-bit, 0-4095)   [TASK 4]
 *
 * BOOT  (always starts in AP mode)
 *   AP mode ON, VIOLET for up to 5 min, waiting for a phone
 *     while waiting, a saved network (if any) is tried in the background
 *       saved network connects -> AP OFF, GREEN 5 min -> LED OFF
 *     phone joins the AP       -> BLUE, captive HTML portal, 5 min configuration window
 *     5 min over, no phone, no connection:
 *       AP OFF -> NOT CONNECTED mode: RED, keep trying the saved network for 5 min
 *           connects within 5 min -> RED OFF, GREEN 5 min -> LED OFF
 *           not connected         -> RED OFF (checking stops)
 *
 * LINK LOST while connected   -> RED, keep trying the saved network 5 min, GREEN on reconnect
 *
 * BUTTON
 *   short press (<= 3 s)      -> status: GREEN once if connected, RED once if not connected
 *   long press  (>  3 s)      -> AP mode again (VIOLET 5 min waiting for a phone)
 *
 * WEB PAGE (whenever the AP is ON: connected to a router or not)   [CHANGE-1, CHANGE-2]
 *   - shows the saved network and whether it is connected right now
 *   - CONNECT: add / change the network at ANY time (also while connected or disconnected,
 *     also after an earlier network was added). Previous NVS data is kept until the new
 *     network really connects.
 *         new network connects  -> saved to NVS, GREEN
 *         new network fails     -> RED blink once, back to AP mode:
 *                                   BLUE if a phone is on the AP, VIOLET if not
 *   - "Forget saved network" button (tap twice to confirm): erases the saved SSID/password
 *     from NVS, drops the router, stays in AP mode (BLUE / VIOLET)
 *
 * SERIAL (optional): type CLEARNVS + Enter = same as the web "Forget saved network" button
 *   platformio.ini: monitor_eol = LF , monitor_filters = send_on_enter
 *
 * TIME (no NTP)                                                    [CHANGE-3]
 *   Date/time is read from the "Date:" header of Google over plain HTTP
 *   (clients3.google.com/generate_204, fallback www.google.com). Works in any country,
 *   needs only TCP port 80. Runs in a small FreeRTOS task, never blocks the state machine.
 *
 * MQ-5 GAS SENSOR (raw ADC only)                                   [TASK 4]
 *   ADC1 channel 6 (GPIO 34) is initialised at boot and a 180 s millis() warm-up timer
 *   starts once. During warm-up the ADC is sampled for monitoring only and is shown as
 *   "ADC_RAW: n (IGNORED)" with an "ADC_TIMER: mm:ss" countdown. After 180 s the sensor is
 *   WARMUP_COMPLETE and the status line shows "ADC: n". No ppm conversion, no calibration.
 *   Everything is inside the existing single-line printSerialStatus().
 *
 * Fully non-blocking (millis()). No restarts to change state.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
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
constexpr uint32_t AP_WAIT_MS         = 300000UL;  // AP waiting for a phone: 5 minutes
constexpr uint32_t CONFIG_WINDOW_MS   = 300000UL;  // configuration window after phone joins: 5 minutes
constexpr uint32_t CONNECT_TIMEOUT_MS = 20000UL;   // test of a newly entered router
constexpr uint32_t LINK_LOST_MS       = 3000UL;    // router gone this long = link lost
constexpr uint32_t PHONE_LOST_MS      = 2000UL;    // grace time before "phone left" is accepted
constexpr uint32_t AP_ANNOUNCE_MS     = 2000UL;    // VIOLET shown this long when AP starts while connected
constexpr uint32_t STATUS_FLASH_MS    = 2000UL;    // short-press status LED
constexpr uint32_t RED_BLINK_MS       = 1200UL;    // [CHANGE-2] RED once after a failed new network
constexpr uint32_t FORGET_DELAY_MS    = 400UL;     // let the web reply leave before the AP state changes
constexpr uint32_t LONG_PRESS_MS      = 3000UL;    // long press = STRICTLY MORE than 3 s
constexpr uint32_t DEBOUNCE_MS        = 30UL;

constexpr uint32_t TIME_RETRY_MS      = 15000UL;   // [CHANGE-3] retry until first sync
constexpr uint32_t TIME_RESYNC_MS     = 3600000UL; // [CHANGE-3] then every 1 hour

// [TASK 4] MQ-5 gas sensor on ADC1 channel 6
constexpr uint8_t  MQ5_ADC_PIN        = 34;        // GPIO34 = ADC1_CHANNEL_6 (classic ESP32)
constexpr uint32_t MQ5_WARMUP_MS      = 180000UL;  // 3-minute sensor warm-up
constexpr uint32_t MQ5_SAMPLE_MS      = 200UL;     // one ADC reading every 200 ms
constexpr uint8_t  MQ5_OVERSAMPLE     = 8;         // raw samples averaged per reading

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
  ST_WIFI_SEARCH,        // (not used at boot any more)
  ST_WIFI_CONNECTED,     // GREEN 5 minutes
  ST_WIFI_FAILED,        // NOT CONNECTED: RED, checking the saved network for 5 minutes
  ST_AP_WAITING,         // AP on, VIOLET, waiting for a phone
  ST_PHONE_CONNECTED,    // phone joined, BLUE
  ST_WEB_CONFIGURATION,  // portal page served
  ST_WIFI_CONNECTING,    // testing the newly entered router
  ST_CONFIG_SUCCESS,     // new network connected: GREEN (AP stays, more networks can be added)
  ST_CONFIG_FAILED,      // (not used any more: a failure returns straight to AP mode)
  ST_NORMAL_IDLE
};

enum LedColor : uint8_t { LED_OFF, LED_VIOLET, LED_BLUE, LED_GREEN, LED_RED };

// last result shown on the web page
enum Outcome : uint8_t { OUT_NONE = 0, OUT_SUCCESS = 1, OUT_FAILED = 2, OUT_FORGOTTEN = 3 };

// [TASK 4] MQ-5 sensor state
enum Mq5State : uint8_t { MQ5_WARMUP = 0, MQ5_WARMUP_COMPLETE = 1 };

// ============================================================================
//  Globals
// ============================================================================
CRGB        leds[NUM_LEDS];
Preferences prefs;
WebServer   server(80);
DNSServer   dns;

State state = ST_BOOT;

// credentials
String savedSsid, savedPass;
String pendingSsid, pendingPass;
String connectFailReason;
Outcome outcome = OUT_NONE;

// timers
uint32_t retryStart    = 0;
uint32_t lastAttemptMs = 0;
uint32_t apStart       = 0;
uint32_t cfgStart      = 0;
uint32_t connectStart  = 0;
uint32_t indicateStart = 0;
uint32_t linkLostSince = 0;
bool     monitorLink   = false;     // supervise the router link (connected / idle only)

// AP / web
bool     apActive    = false;
bool     webActive   = false;
bool     manualAp    = false;
bool     bootSearch  = false;       // boot AP: saved network is tried in the background
uint8_t  apClients   = 0;
uint32_t apZeroSince = 0;
bool     pendingForget   = false;   // web "forget" waiting for its reply to be sent
uint32_t pendingForgetAt = 0;

// LED
LedColor ledColor    = LED_OFF;     // colour requested by the state machine
LedColor ledApplied  = LED_OFF;     // colour currently on the strip
bool     flashActive = false;       // short flash (status / red blink)
LedColor flashColor  = LED_OFF;
uint32_t flashStart  = 0;
uint32_t flashDur    = STATUS_FLASH_MS;

// button
bool     btnRaw       = false;
bool     btnPressed   = false;
bool     longHandled  = false;
uint32_t btnChangeMs  = 0;
uint32_t pressStartMs = 0;
uint32_t holdMs       = 0;

// [CHANGE-3] Google HTTP time sync
bool          timeSynced     = false;
uint32_t      lastSyncOk     = 0;
uint32_t      lastSyncTry    = 0;
bool          syncFailLogged = false;
volatile bool syncBusy       = false;
volatile bool syncDone       = false;
volatile bool syncResult     = false;

// [TASK 4] MQ-5 ADC
Mq5State mq5State     = MQ5_WARMUP;
uint32_t mq5WarmStart = 0;          // set ONCE at the end of setup()
uint32_t mq5LastRead  = 0;
uint16_t mq5Raw       = 0;          // latest raw 12-bit ADC value (0-4095)

// serial status line / commands
uint32_t lastStatusMs = 0;
bool     statusForce  = true;
size_t   lastLineLen  = 0;
String   cmdBuf;

// ============================================================================
//  Web page
// ============================================================================
const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Wi-Fi Setup</title>
<style>
*{box-sizing:border-box}
body{margin:0;min-height:100vh;display:flex;align-items:center;justify-content:center;background:#0f172a;font-family:system-ui,Arial,sans-serif;color:#0f172a}
.card{width:92%;max-width:380px;background:#fff;border-radius:16px;padding:24px;margin:16px 0;box-shadow:0 10px 30px rgba(0,0,0,.35)}
h1{margin:0 0 4px;font-size:22px}
p.sub{margin:0 0 14px;color:#475569;font-size:14px}
.saved{padding:10px 12px;border-radius:10px;background:#eef2ff;color:#3730a3;font-size:14px;margin-bottom:6px;word-break:break-word}
label{display:block;margin:12px 0 6px;font-size:14px;font-weight:600}
input{width:100%;padding:12px;border:1px solid #cbd5e1;border-radius:10px;font-size:16px}
input:focus{outline:2px solid #6366f1;border-color:#6366f1}
.pw{position:relative}
.pw input{padding-right:48px}
.pw button{position:absolute;right:6px;top:50%;transform:translateY(-50%);width:38px;height:38px;border:0;background:none;color:#475569;cursor:pointer}
svg{width:22px;height:22px;fill:none;stroke:currentColor;stroke-width:2;stroke-linecap:round;stroke-linejoin:round}
#go{width:100%;margin-top:18px;padding:13px;border:0;border-radius:10px;background:#4f46e5;color:#fff;font-size:16px;font-weight:700;letter-spacing:.5px;cursor:pointer}
#go:disabled{background:#94a3b8;cursor:default}
#forget{width:100%;margin-top:10px;padding:12px;border:0;border-radius:10px;background:#dc2626;color:#fff;font-size:15px;font-weight:700;cursor:pointer}
.st{margin-top:16px;padding:12px;border-radius:10px;background:#f1f5f9;font-size:14px;line-height:1.4}
.st.wait{background:#fef3c7;color:#92400e}
.st.ok{background:#dcfce7;color:#166534}
.st.err{background:#fee2e2;color:#991b1b}
</style></head><body>
<div class="card">
<h1>ESP32 Wi-Fi Setup</h1>
<p class="sub">Add or change the router the device should join.</p>
<div id="saved" class="saved">Checking saved network...</div>
<form id="f" autocomplete="off">
<label for="ssid">Wi-Fi name (SSID)</label>
<input id="ssid" name="ssid" maxlength="32" required autocapitalize="none" spellcheck="false">
<label for="pass">Password</label>
<div class="pw">
<input id="pass" name="pass" type="password" maxlength="63" autocapitalize="none" spellcheck="false">
<button type="button" id="eye" aria-label="Show or hide password">
<svg id="eyeOn" viewBox="0 0 24 24"><path d="M1 12s4-7 11-7 11 7 11 7-4 7-11 7S1 12 1 12z"/><circle cx="12" cy="12" r="3"/></svg>
<svg id="eyeOff" viewBox="0 0 24 24" style="display:none"><path d="M1 12s4-7 11-7 11 7 11 7-4 7-11 7S1 12 1 12z"/><circle cx="12" cy="12" r="3"/><line x1="3" y1="3" x2="21" y2="21"/></svg>
</button>
</div>
<button type="submit" id="go">CONNECT</button>
</form>
<button type="button" id="forget" style="display:none">Forget saved network</button>
<div id="st" class="st">Waiting for your details.</div>
</div>
<script>
const $=id=>document.getElementById(id);
let armed=null, sending=false;
function msg(t,c){const s=$('st');s.textContent=t;s.className='st '+(c||'');}
$('eye').onclick=()=>{
  const p=$('pass'),show=p.type==='password';
  p.type=show?'text':'password';
  $('eyeOn').style.display=show?'none':'block';
  $('eyeOff').style.display=show?'block':'none';
};
function render(j){
  const fg=$('forget');
  if(j.saved){
    $('saved').textContent='Saved network: '+j.saved+' ('+(j.up?'connected':'not connected')+')';
    fg.style.display='block';
  }else{
    $('saved').textContent='No saved network';
    fg.style.display='none';
  }
  if(j.state==='connecting'){$('go').disabled=true;msg('Connecting to the router...','wait');return;}
  if(!sending)$('go').disabled=false;
  if(j.outcome===1)msg('Connected to '+j.ssid+'. Saved. You can add another network.','ok');
  else if(j.outcome===2)msg('Connection failed: '+j.reason+'. Try again.','err');
  else if(j.outcome===3)msg('Saved network forgotten.','ok');
  else if(!sending)msg('Waiting for your details.','');
}
async function poll(){
  try{
    const r=await fetch('/status',{cache:'no-store'});
    render(await r.json());
  }catch(e){msg('Waiting for the device...','wait');}
}
$('f').onsubmit=async e=>{
  e.preventDefault();
  sending=true;$('go').disabled=true;msg('Sending...','wait');
  try{
    const r=await fetch('/connect',{method:'POST',body:new URLSearchParams(new FormData($('f')))});
    if(!r.ok){const j=await r.json();msg(j.error||'Request rejected','err');$('go').disabled=false;sending=false;return;}
    msg('Connecting to the router...','wait');
  }catch(err){msg('Could not reach the device','err');$('go').disabled=false;}
  sending=false;
};
$('forget').onclick=async()=>{
  const b=$('forget');
  if(!armed){
    b.textContent='Tap again to confirm';
    armed=setTimeout(()=>{armed=null;b.textContent='Forget saved network';},4000);
    return;
  }
  clearTimeout(armed);armed=null;b.textContent='Forget saved network';
  try{
    const r=await fetch('/forget',{method:'POST'});
    if(!r.ok){const j=await r.json();msg(j.error||'Could not forget','err');}
    else msg('Forgetting saved network...','wait');
  }catch(err){msg('Could not reach the device','err');}
};
setInterval(poll,1500);poll();
</script></body></html>)HTML";

// ============================================================================
//  Forward declarations (loop services)
// ============================================================================
void handleButton();
void checkWiFiConnection();
void handleAPClients();
void handleAPTimeout();
void updateLEDStatus();
void handleWebServer();
void printSerialStatus();
void handleTimeSync();
void handleSerialCommands();
void handleMq5Sensor();                            // [TASK 4]

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

static String jsonEscape(const String& in) {
  String o;
  o.reserve(in.length() + 4);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c < 0x20) { o += ' '; }
    else o += c;
  }
  return o;
}

static const char* stateName() {
  switch (state) {
    case ST_BOOT:              return "BOOT";
    case ST_CHECK_NVS:         return "CHECK_NVS";
    case ST_WIFI_SEARCH:       return "WIFI_SEARCH";
    case ST_WIFI_CONNECTED:    return "WIFI_CONNECTED";
    case ST_WIFI_FAILED:       return "NOT_CONNECTED";
    case ST_AP_WAITING:        return manualAp ? "MANUAL_AP" : "AP_WAITING";
    case ST_PHONE_CONNECTED:   return "PHONE_CONNECTED";
    case ST_WEB_CONFIGURATION: return "WEB_CONFIGURATION";
    case ST_WIFI_CONNECTING:   return "WIFI_CONNECTING";
    case ST_CONFIG_SUCCESS:    return "CONFIG_SUCCESS";
    case ST_CONFIG_FAILED:     return "CONFIG_FAILED";
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
  Serial.print("\r\x1b[2K");                       // return + erase the whole live status line
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
    return (apClients > 0) ? LED_BLUE : LED_VIOLET; // router gone: back to the AP colours
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
    WiFi.mode(WIFI_AP_STA);                        // portal stays reachable while a router is tested
    bool ok = (strlen(AP_PASS) >= 8) ? WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4)
                                     : WiFi.softAP(AP_SSID);
    if (!ok) logEvent("softAP start FAILED");

    dns.setErrorReplyCode(DNSReplyCode::NoError);
    dns.start(53, "*", WiFi.softAPIP());           // captive portal: every name -> 192.168.4.1
    server.begin();
    webActive = true;
    apActive  = true;
  }

  manualAp    = manual;
  apClients   = 0;
  apZeroSince = 0;
  apStart     = millis();
  outcome     = OUT_NONE;
  setLed(LED_VIOLET);

  bootSearch = false;
  if (searchSaved && credentialsValid()) {         // boot: look for the saved network meanwhile
    WiFi.setAutoReconnect(false);
    beginSavedAttempt();
    bootSearch = true;
  }

  logEvent(String(manual ? "Manual" : "Boot") + " AP mode ON, SSID: " + AP_SSID +
           ", IP: " + WiFi.softAPIP().toString() +
           (bootSearch ? " (saved network is also being tried)" : ""));
  setState(ST_AP_WAITING);
}

static void stopAP() {
  bool routerUp = (WiFi.status() == WL_CONNECTED);
  dns.stop();
  server.stop();
  webActive = false;
  WiFi.softAPdisconnect(true);
  WiFi.mode(routerUp ? WIFI_STA : WIFI_OFF);
  apActive   = false;
  bootSearch = false;
  apClients  = 0;
  logEvent("AP mode OFF");
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

// [CHANGE-2] Back to AP mode with the right colour, AP stays ON.
//            BLUE if a phone is on the AP, VIOLET if not.
static void returnToApMode() {
  uint8_t n   = WiFi.softAPgetStationNum();
  apClients   = n;
  apZeroSince = 0;
  bootSearch  = false;
  if (n > 0) {
    cfgStart = millis();                           // fresh 5 minute configuration window
    setLed(LED_BLUE);
    setState(ST_PHONE_CONNECTED);
  } else {
    apStart = millis();                            // fresh 5 minute wait for a phone
    setLed(LED_VIOLET);
    setState(ST_AP_WAITING);
  }
}

// Newly entered router failed. NVS is untouched.
static void failConfig(const String& reason) {
  connectFailReason = reason;
  outcome = OUT_FAILED;
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false);
  logEvent("New Wi-Fi failed: " + reason + " (previous NVS credentials preserved)");
  startFlash(LED_RED, RED_BLINK_MS);               // RED once ...
  returnToApMode();                                // ... then BLUE / VIOLET
}

// [CHANGE-1] Forget the saved network (web button or serial CLEARNVS)
static void forgetSavedNetwork() {
  bool ok = eraseCredentials();
  logEvent(ok ? "NVS cleared: saved Wi-Fi credentials erased" : "NVS clear FAILED");

  monitorLink   = false;
  linkLostSince = 0;
  bootSearch    = false;
  flashActive   = false;
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false);

  if (apActive) returnToApMode();                  // keep the AP and the phone connection
  else          startAP(false, false);             // AP was off: open it so a new network can be added
  outcome = OUT_FORGOTTEN;
}

// ============================================================================
//  Web server handlers
// ============================================================================
static void sendJson(int code, const String& body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

static void handleRoot() {
  if (state == ST_PHONE_CONNECTED) setState(ST_WEB_CONFIGURATION);
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE);
}

static void handleStatus() {
  bool up = (WiFi.status() == WL_CONNECTED) && credentialsValid() && (WiFi.SSID() == savedSsid);
  String j = "{\"state\":\"";
  j += (state == ST_WIFI_CONNECTING) ? "connecting" : "ready";
  j += "\",\"outcome\":";   j += String((int)outcome);
  j += ",\"ssid\":\"";      j += jsonEscape(pendingSsid);
  j += "\",\"reason\":\"";  j += jsonEscape(connectFailReason);
  j += "\",\"saved\":\"";   j += jsonEscape(savedSsid);
  j += "\",\"up\":";        j += up ? "true" : "false";
  j += "}";
  sendJson(200, j);
}

// [CHANGE-2] allowed at ANY time while the AP is on (connected to a router or not),
//            only refused while another new network is still being tested
static void handleConnect() {
  if (!apActive || state == ST_WIFI_CONNECTING) {
    sendJson(409, "{\"error\":\"Device is busy, try again in a moment.\"}");
    return;
  }
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  ssid.trim();
  if (ssid.length() == 0 || ssid.length() > 32) {
    sendJson(400, "{\"error\":\"SSID must be 1 to 32 characters.\"}");
    return;
  }
  if (pass.length() > 63 || (pass.length() > 0 && pass.length() < 8)) {
    sendJson(400, "{\"error\":\"Password must be 8 to 63 characters (or empty for an open network).\"}");
    return;
  }

  pendingSsid = ssid;
  pendingPass = pass;
  connectFailReason = "";
  outcome = OUT_NONE;
  flashActive = false;
  bootSearch  = false;
  monitorLink = false;

  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false);                          // leave the old router, test the new one
  WiFi.begin(pendingSsid.c_str(), pendingPass.c_str());
  connectStart = millis();
  setLed(LED_BLUE);                                // BLUE while connecting
  logEvent("Connecting to new router: " + pendingSsid);
  setState(ST_WIFI_CONNECTING);
  sendJson(200, "{\"ok\":true}");
}

// [CHANGE-1] web "Forget saved network"
static void handleForget() {
  if (!apActive || state == ST_WIFI_CONNECTING) {
    sendJson(409, "{\"error\":\"Device is busy, try again in a moment.\"}");
    return;
  }
  pendingForget   = true;                          // done from loop() after the reply is sent
  pendingForgetAt = millis();
  sendJson(200, "{\"ok\":true}");
}

static void handleNotFound() {
  if (apActive) {                                  // captive portal redirect
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    server.send(302, "text/plain", "");
  } else {
    server.send(404, "text/plain", "Not found");
  }
}

static void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/connect", HTTP_POST, handleConnect);
  server.on("/forget", HTTP_POST, handleForget);
  // OS connectivity probes open the portal
  server.on("/generate_204", HTTP_GET, handleRoot);
  server.on("/gen_204", HTTP_GET, handleRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleRoot);
  server.on("/library/test/success.html", HTTP_GET, handleRoot);
  server.on("/connecttest.txt", HTTP_GET, handleRoot);
  server.on("/ncsi.txt", HTTP_GET, handleRoot);
  server.on("/fwlink", HTTP_GET, handleRoot);
  server.onNotFound(handleNotFound);
}

// ============================================================================
//  Serial command: CLEARNVS  (same as the web "Forget saved network")
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
//  [CHANGE-3] Time from Google over plain HTTP ("Date:" response header)
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

// Runs once in its own task so the main loop (Wi-Fi state machine, LEDs, button) never blocks.
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
//  [TASK 4] MQ-5 gas sensor : raw ADC1 channel 6 (GPIO34) with 180 s warm-up
// ============================================================================

// ADC1 only (ADC2 would conflict with Wi-Fi). 12-bit, 11 dB attenuation (~0-3.3 V range).
static void initMq5Adc() {
  pinMode(MQ5_ADC_PIN, INPUT);                     // GPIO34 is input-only, no pull resistors
  analogReadResolution(12);                        // 0-4095
  analogSetPinAttenuation(MQ5_ADC_PIN, ADC_11db);
  mq5State     = MQ5_WARMUP;
  mq5Raw       = (uint16_t)analogRead(MQ5_ADC_PIN);
  mq5LastRead  = millis();
  mq5WarmStart = millis();                         // warm-up timer starts exactly once, here
}

// True only after the COMPLETE 180 s have elapsed (unsigned subtraction survives millis() wrap).
static bool mq5WarmupElapsed() {
  return (millis() - mq5WarmStart) >= MQ5_WARMUP_MS;
}

// Called every loop(): fast, non-blocking, independent of Wi-Fi.
void handleMq5Sensor() {
  uint32_t now = millis();

  // warm-up -> complete transition (once)
  if (mq5State == MQ5_WARMUP && mq5WarmupElapsed()) {
    mq5State    = MQ5_WARMUP_COMPLETE;
    statusForce = true;
    logEvent("MQ-5 warm-up complete (180 s): ADC readings are now valid");
  }

  // periodic sampling (readings during warm-up are monitoring only)
  if ((now - mq5LastRead) >= MQ5_SAMPLE_MS) {
    mq5LastRead = now;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < MQ5_OVERSAMPLE; i++) sum += (uint32_t)analogRead(MQ5_ADC_PIN);
    mq5Raw = (uint16_t)(sum / MQ5_OVERSAMPLE);
  }
}

// ============================================================================
//  Loop services
// ============================================================================

// ---- Button: short press = status, long press (> 3 s) = AP mode ---------------
static bool manualApAllowed() {
  // every state except: AP already waiting, phone configuring, new router being tested
  return state == ST_WIFI_SEARCH || state == ST_WIFI_CONNECTED || state == ST_WIFI_FAILED ||
         state == ST_NORMAL_IDLE || state == ST_CONFIG_SUCCESS || state == ST_CONFIG_FAILED;
}

void handleButton() {
  uint32_t now = millis();
  bool raw = (digitalRead(BUTTON_PIN) == LOW);

  if (raw != btnRaw) { btnRaw = raw; btnChangeMs = now; }

  if ((now - btnChangeMs) >= DEBOUNCE_MS && raw != btnPressed) {
    btnPressed  = raw;
    statusForce = true;
    if (btnPressed) {
      pressStartMs = now;
      holdMs       = 0;
      longHandled  = false;
    } else {
      if (!longHandled && state != ST_WIFI_CONNECTING) showStatusFlash();   // short press
      holdMs = 0;
    }
  }

  if (btnPressed) {
    holdMs = now - pressStartMs;
    if (!longHandled && holdMs > LONG_PRESS_MS) {                          // strictly more than 3 s
      longHandled = true;                                                  // one decision per press
      if (manualApAllowed()) {
        logEvent("Long press detected (> 3 s): starting manual AP");
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

    case ST_AP_WAITING:                            // AP on, no phone: saved network tried in the background
      // AP stays ON for its full 5 minutes so a phone can still join.
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
        logEvent("Router connected: " + WiFi.SSID());
        if (saveCredentials(pendingSsid, pendingPass)) {
          savedSsid = pendingSsid;
          savedPass = pendingPass;
          outcome   = OUT_SUCCESS;
          flashActive = false;
          logEvent("Credentials saved to NVS and verified");
          startIndication(LED_GREEN);              // GREEN
          setState(ST_CONFIG_SUCCESS);             // AP stays: more networks can still be added
        } else {
          failConfig("Could not save credentials to NVS");
        }
      } else if (elapsed > 1500 && st == WL_CONNECT_FAILED) {
        failConfig("Wrong password or authentication failed");
      } else if (elapsed >= CONNECT_TIMEOUT_MS) {
        failConfig(st == WL_NO_SSID_AVAIL ? "Network not found" : "Connection timed out");
      }
      break;
    }

    case ST_CONFIG_SUCCESS:                        // GREEN while connected (max 5 min), then LED OFF, AP OFF
      // router disconnected: LED falls back to VIOLET (no phone) / BLUE (phone) and the
      // AP keeps running; the page can add another network at any time
      if (WiFi.status() != WL_CONNECTED && apClients == 0) retryTick();
      if (now - indicateStart >= INDICATE_MS) {
        setLed(LED_OFF);
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

// ---- AP client detection --------------------------------------------------------
void handleAPClients() {
  if (!apActive) { apClients = 0; return; }

  uint32_t now = millis();
  uint8_t n = WiFi.softAPgetStationNum();
  if (n != apClients) {
    apClients   = n;
    statusForce = true;
    logEvent("AP clients: " + String(n));
  }

  switch (state) {
    case ST_AP_WAITING:
      if (n > 0) {
        if (bootSearch) {                          // phone has priority: stop background attempts
          bootSearch = false;
          if (WiFi.status() != WL_CONNECTED) WiFi.disconnect(false);
        }
        cfgStart    = now;                         // 5 minute configuration window starts here
        apZeroSince = 0;
        setLed(LED_BLUE);
        setState(ST_PHONE_CONNECTED);
      }
      break;

    case ST_PHONE_CONNECTED:
    case ST_WEB_CONFIGURATION:
      if (n == 0) {
        if (apZeroSince == 0) apZeroSince = now;
        if (now - apZeroSince >= PHONE_LOST_MS) {
          logEvent("Phone left the AP, waiting again");
          setLed(LED_VIOLET);
          apZeroSince = 0;
          apStart = now;
          setState(ST_AP_WAITING);
        }
      } else {
        apZeroSince = 0;
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
    logEvent("No phone connected within 5 minutes");
    endApSession();                                // AP OFF -> RED, keep trying saved network
  } else if ((state == ST_PHONE_CONNECTED || state == ST_WEB_CONFIGURATION) &&
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

// ---- Web / DNS service ----------------------------------------------------------
void handleWebServer() {
  if (!webActive) return;
  if (apActive) dns.processNextRequest();
  server.handleClient();

  // web "forget": run it after the reply has left
  if (pendingForget && (millis() - pendingForgetAt) >= FORGET_DELAY_MS) {
    pendingForget = false;
    if (state != ST_WIFI_CONNECTING) forgetSavedNetwork();
  }
}

// ---- Short names used ONLY by the live status line (event log keeps full names) ----
static const char* stateShort() {
  switch (state) {
    case ST_BOOT:              return "BOOT";
    case ST_CHECK_NVS:         return "CHECK_NVS";
    case ST_WIFI_SEARCH:       return "WIFI_SRCH";
    case ST_WIFI_CONNECTED:    return "WIFI_CONN";
    case ST_WIFI_FAILED:       return "NOT_CONN";
    case ST_AP_WAITING:        return manualAp ? "MANUAL_AP" : "AP_WAIT";
    case ST_PHONE_CONNECTED:   return "PHONE_CONN";
    case ST_WEB_CONFIGURATION: return "WEB_CFG";
    case ST_WIFI_CONNECTING:   return "WIFI_CNTNG";
    case ST_CONFIG_SUCCESS:    return "CFG_OK";
    case ST_CONFIG_FAILED:     return "CFG_FAIL";
    case ST_NORMAL_IDLE:       return "IDLE";
  }
  return "?";
}

static String statusTimeString() {
  if (!timeSynced) return String("D/T NOT SYNC");
  time_t t = time(nullptr) + GMT_OFFSET_SEC;
  struct tm ti;
  gmtime_r(&t, &ti);
  char b[24];
  strftime(b, sizeof(b), "%d-%b-%y %H:%M:%S", &ti);
  return String(b);
}

// ---- Live status line (one line, redrawn with '\r') -----------------------------
void printSerialStatus() {
  uint32_t now = millis();
  uint32_t interval = btnPressed ? 200 : 1000;
  if (!statusForce && (now - lastStatusMs) < interval) return;
  statusForce  = false;
  lastStatusMs = now;

  // Compact router text so the whole line stays shorter than the terminal width.
  String router;
  if (WiFi.status() == WL_CONNECTED)    router = WiFi.SSID();
  else if (state == ST_WIFI_CONNECTING) router = "CONNECTING";
  else if (bootSearch)                  router = "SEARCHING";
  else if (state == ST_WIFI_FAILED)     router = "RETRYING";
  else                                  router = "NONE";
  if (router.length() > 12) router = router.substring(0, 12);

  String s;
  s.reserve(200);
  s += "[";  s += statusTimeString();  s += "] | STATE: ";  s += stateShort();
  s += " | Router: ";   s += router;
  s += " | AP: ";       s += apActive ? "ON" : "OFF";
  s += " | Clients: ";  s += String(apClients);

  if (state == ST_WIFI_FAILED) {
    s += " | Retry: ";  s += mmss(remainingMs(retryStart, RETRY_WINDOW_MS));
  }

  if (apActive) {
    uint32_t rem = 0;
    switch (state) {
      case ST_AP_WAITING:          rem = remainingMs(apStart, AP_WAIT_MS);          break;
      case ST_PHONE_CONNECTED:
      case ST_WEB_CONFIGURATION:
      case ST_WIFI_CONNECTING:     rem = remainingMs(cfgStart, CONFIG_WINDOW_MS);   break;
      case ST_CONFIG_SUCCESS:      rem = remainingMs(indicateStart, INDICATE_MS);   break;
      default: break;
    }
    s += " | AP Remains: ";  s += mmss(rem);
  }

  // [TASK 4] MQ-5 ADC fields (same single status line)
  if (mq5State == MQ5_WARMUP) {
    s += " | ADC: ";          s += String((unsigned)mq5Raw);  s += "(IGN)";
    s += " | ADC_TIMER: ";    s += mmss(remainingMs(mq5WarmStart, MQ5_WARMUP_MS));
  } else {
    s += " | ADC: ";          s += String((unsigned)mq5Raw);
  }

  s += " | Button: ";  s += btnPressed ? "ON" : "OFF";
  if (btnPressed) {
    char b[24];
    snprintf(b, sizeof(b), " Hold: %.1f/3s", holdMs / 1000.0);
    s += b;
  }
  s += " | LED: ";  s += ledName();

  // \r  = back to column 0
  // ESC[2K = erase the whole line (no leftover characters, no padding needed)
  // ESC[?7l / ESC[?7h = line-wrap off while drawing, on again afterwards
  Serial.print("\r\x1b[2K\x1b[?7l");
  Serial.print(s);
  Serial.print("\x1b[?7h");
  lastLineLen = s.length();
}

// ============================================================================
//  setup() / loop()
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);                                      // one-time: let the serial port settle
  Serial.println();
  Serial.println("==== ESP32 Wi-Fi Configuration Manager ====");
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

  // web server routes (sockets open when the AP starts)
  setupWebServer();

  // NVS -> always boot into AP mode (VIOLET, 5 min); saved network is tried meanwhile
  setState(ST_CHECK_NVS);
  loadCredentials();

  if (credentialsValid()) logEvent("Saved credentials found for: " + savedSsid);
  else                    logEvent("No saved credentials");

  startAP(false, true);

  // [TASK 4] MQ-5 on ADC1_CH6 (GPIO34) + start the one-time 180 s non-blocking warm-up
  initMq5Adc();
  logEvent("MQ-5 ADC1_CH6 (GPIO34) initialised: 180 s warm-up started, readings ignored until it ends");
}

void loop() {
  handleSerialCommands();
  handleButton();
  checkWiFiConnection();
  handleAPClients();
  handleAPTimeout();
  handleWebServer();
  handleTimeSync();
  handleMq5Sensor();                               // [TASK 4]
  updateLEDStatus();
  printSerialStatus();
  delay(1);                                        // yield to the Wi-Fi task
}
