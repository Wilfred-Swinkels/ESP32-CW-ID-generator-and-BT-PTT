/*
  ================================================================
  ESP32 + HM-10 (BLE)  ->  Remote PTT switch + Callgever (CW-ID)
  ================================================================
  This is the original "ESP32 + HM-10 Remote PTT" sketch, extended with
  the callgever (morse-ID beacon). The callgever no longer has its own
  physical DI pin: it is triggered by the same combined PTT status
  (remote via HM-10 OR the PTT button on the web page) that also drives
  the PTT output.

  PART 1 - PTT via HM-10 (UART, 9600 baud, pins 16/17)
  -----------------------------------------------------
    Unchanged from the original sketch: continuously poll "AT+PIO3?",
    handle OK+PIO3:0/1, handle OK+LOST/OK+CONN as asynchronous connection
    notifications.

  PART 2 - Outputs
  -----------------------------------------------------
    D21 (PTT_OUTPUT_PIN) becomes active as soon as PTT should be active:
    via the remote, via the PTT button on the web page, OR because the
    callgever is currently transmitting its morse ID (see PART 4). The
    latter is needed so the transmitter actually stays keyed if the
    operator has already released the PTT button while the ID is still
    playing.
    D18/D19 show the BLE link status, as before.

  PART 3 - WiFi + web page (dark mode)
  -----------------------------------------------------
    Unchanged: AP setup mode for WiFi credentials, main page with PTT
    button, /setup for MQTT credentials. The main page now also shows a
    second status line ("CW-ID standby" / "CW-ID transmitting...") and a
    link to the new /callgever configuration page.

  PART 4 - Callgever (CW-ID beacon), NEW
  -----------------------------------------------------
    As soon as the combined PTT status (remotePtt || webPtt) goes from
    low to high (PTT is pressed) AND the callgever is enabled, the ESP32
    waits the configured delay (0-50000 ms) and then transmits the
    configured callsign in morse, as an analog sine tone on the built-in
    DAC (GPIO25, 8-bit, only on the classic ESP32 WROOM/WROVER - not on
    S2/S3/C3/C6).
      - If you release the PTT before the delay has elapsed: the ID is
        cancelled.
      - Once started, the ID is always finished (the PTT output stays
        active for this, even if the operator/remote releases in the
        meantime) - unless you disable the callgever via the web page in
        the meantime, in which case it stops immediately.
      - If the PTT is still pressed after the ID finishes, a new ID is
        only accepted once the PTT has actually been released (prevents
        continuous repeating for as long as someone holds the button
        down).
    Configurable via the new /callgever page: callsign, on/off, volume
    (0-100%), speed (5-30 WPM) and tone frequency (400-1500 Hz).
    Everything is stored in NVS flash just like the WiFi/MQTT settings
    (same "cfg" namespace, separate "cg_*" keys).

    Hardware note: GPIO25 (the DAC output) needs a series capacitor
    (DC blocking) and possibly a voltage divider to bring it down to mic
    level before connecting it alongside (or instead of) the microphone
    signal on the transmitter - see README.md.

  Required libraries: all already included in the ESP32 board package
  (WiFi, WebServer, Preferences, ESPmDNS) - nothing extra to install,
  except for the included morse.h/.cpp and audio_dac.h/.cpp files in this
  folder.
  ================================================================
*/
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>

#include "morse.h"
#include "audio_dac.h"

// ================= Settings: HM-10 / PTT =================
#define HM10_RX_PIN          16     // ESP32 RX2  <-- HM-10 TX
#define HM10_TX_PIN          17     // ESP32 TX2  --> HM-10 RX
#define HM10_BAUD            9600
#define PTT_OUTPUT_PIN       21     // to transistor -> relay -> PTT line of the radio/transmitter
#define PTT_ACTIVE_HIGH      true   // set to false for an active-low output
#define STATUS_LED_PIN       2      // onboard LED on most ESP32 dev boards

// DO that actually draws out the morse code of the ID: high during each
// dot/dash, low during the gaps in between (so NOT simply continuously
// high during the whole transmission, NOT for as long as the PTT output
// is active for other reasons, and also not during the manual test-tone
// button on /callgever) - handy for e.g. letting an external
// device/logger/CW decoder react directly to the morse code. Set to -1
// to disable.
#define ID_ACTIVE_PIN        4

// Two extra outputs that show the BLE link with the remote:
//   connected    : LINK_OK_PIN high, LINK_LOST_PIN low (both steady)
//   link lost    : LINK_OK_PIN low, LINK_LOST_PIN blinking (1 Hz)
#define LINK_OK_PIN          18     // D18 - high for as long as the remote is connected
#define LINK_LOST_PIN        19     // D19 - blinks (1s on/off) for as long as the link is down
#define LINK_LOST_BLINK_MS   1000

// AT+PIO3? is only sent while the BLE link is up: the HM-10 on the ESP32
// side forwards it transparently to the remote HM-10 (AT+MODE1), which
// reports the state of its own PIO3 (the PTT button) back as
// OK+PIO3:0/1. So every response is a BLE round trip and takes at least
// roughly one BLE connection interval (typically a few tens of ms).
// MIN_POLL_GAP_MS and RESPONSE_TIMEOUT_MS together determine how quickly
// a PTT press on the remote can at best be noticed (any press shorter
// than the effective poll cycle falls between two measurements and is
// unavoidably missed - that's inherent to polling). Both are therefore
// set as tight as possible; should this in practice actually lead to
// AT-communication errors with the HM-10 (e.g. "resync" behaviour or more
// frequent timeouts than before), set them a bit more relaxed again
// (e.g. back to 20 / 150).
#define MIN_POLL_GAP_MS      5      // minimum pause between two requests
#define RESPONSE_TIMEOUT_MS  50     // max. time to wait for a response before giving up
#define DEBOUNCE_COUNT       1      // 1 = react immediately; higher (2-3) for noise/glitches

// ================= Settings: WiFi / web =================
#define AP_SSID              "ESP32-PTT-Setup"   // our own access point during setup
#define WIFI_CONNECT_TIMEOUT_MS  15000            // how long to try connecting before falling back to AP
// Was 1500ms; with a flaky WiFi connection (weak signal, roaming between
// APs) several keepalive pings in a row (every 250ms, see MAIN_PAGE) can
// get lost, which made this fail-safe keep shutting the PTT off again
// while the button was still held ("chattering"). 3000ms gives roughly
// 10x as much margin (up to ~11 missed pings) before it kicks in, while
// still being fast enough for a genuinely closed browser/connection.
#define WEB_PTT_TIMEOUT_MS   3000                 // fail-safe: no keepalive -> web-PTT off
#define MDNS_HOSTNAME         "esp32-BT-PTT"       // reachable via http://esp32-BT-PTT.local/

// ================= Settings: Callgever (CW-ID) =================
// NO separate DI pin anymore - the trigger is the combined PTT status
// above (remotePtt || webPtt). See PART 4 above for the behaviour.
#define CG_DEFAULT_CALLSIGN  "ON4TEST"
#define CG_DEFAULT_ENABLED   true
#define CG_DEFAULT_VOLUME    70    // 0-100 %
#define CG_DEFAULT_WPM       15    // 5-30 WPM
#define CG_DEFAULT_FREQ_HZ   800   // 400-1500 Hz
#define CG_DEFAULT_DELAY_MS  25000 // 0-50000 ms, delay after PTT start before the ID begins
#define CG_DEFAULT_ATTACK_MS  1    // 1-20 ms, ramp-up time of the tone envelope
#define CG_DEFAULT_RELEASE_MS 10   // 1-50 ms, ramp-down time of the tone envelope
#define CG_DEFAULT_ABORT_ON_RELEASE false // true = releasing PTT aborts a running ID immediately

// Fixed (not configurable via the web page) tail time: how much silence
// is still transmitted (PTT output stays active, no tone) after the ID
// has finished playing, before the PTT output actually releases - only
// if the ID wasn't aborted partway through via the new "abort
// immediately" option below (see cgUpdate()/CG_TAIL).
static const unsigned long CG_TAIL_SILENCE_MS = 500;

Preferences prefs;
WebServer   server(80);
bool        apMode = false;

// ================= Global variables: HM-10 =================
HardwareSerial HM10(2);   // ESP32 UART2
String        rxBuffer;
bool          waitingForResponse = false;
unsigned long nextPollAt    = 0;
unsigned long requestSentAt = 0;

int  candidateState = -1;     // last received raw PIO3 state (0/1)
int  candidateCount = 0;

// Diagnostics: counts how often an AT+PIO3? request went without a
// (timely) answer -> if this keeps climbing during testing, that's a
// strong sign that missed PTT presses are caused by communication
// problems with the HM-10 (rather than by the ESP32's own poll speed).
unsigned long pollTimeoutCount = 0;

bool linkUp    = false;  // BLE link ESP-HM10 <-> remote-HM10 (OK+CONN / OK+LOST)
bool remotePtt = false;  // PTT status as reported by the remote (after debounce)

bool          linkLostBlinkState  = false;
unsigned long lastLinkBlinkToggle = 0;

// ================= Global variables: web PTT and combined output =================
bool          webPtt         = false;  // manual PTT from the web page
unsigned long lastWebPttPing = 0;
bool          outputActive   = false;  // last applied (combined) output state

// ================= Global variables: Callgever (CW-ID) =================
struct CgConfig {
  char callsign[16];
  bool enabled;
  uint8_t volume;    // 0-100 %
  uint8_t wpm;       // 5-30
  uint16_t freqHz;   // 400-1500
  uint16_t delayMs;  // 0-50000
  uint8_t attackMs;  // 1-20, ramp-up time of the tone envelope
  uint8_t releaseMs; // 1-50, ramp-down time of the tone envelope
  bool abortOnRelease; // true = releasing PTT aborts a running ID immediately
                        // (instead of always finishing it, as is the default)
};
CgConfig cg;
MorsePlayer morse;

// CG_TAIL: the ID has finished playing, but the PTT output stays active
// for CG_TAIL_SILENCE_MS (silence, no tone) before it releases - only
// reached if the ID wasn't aborted partway through via cg.abortOnRelease.
enum CgState { CG_IDLE, CG_ARMED, CG_TRANSMITTING, CG_TAIL, CG_WAIT_RELEASE };
CgState       cgState      = CG_IDLE;
unsigned long cgArmedAtMs  = 0;
unsigned long cgTailAtMs   = 0;
bool          cgPttPrev    = false; // previous (remotePtt||webPtt), for edge detection

// Mandatory periodic repeat of the ID for as long as the PTT stays
// continuously pressed: the first repeat 4:59 (299000 ms) after the
// moment the PTT went on (cgArmedAtMs - "from the start", so including
// the time of the first/normal ID), and every 5:00 (300000 ms) after
// that, for as long as the PTT remains continuously pressed. If you
// release the PTT, the cycle stops and starts counting from scratch
// again on the next press (cgRepeatCount is then also reset, via
// cgArmedAtMs below).
static const unsigned long CG_REPEAT_FIRST_MS = 299000UL;    // 4 min 59 sec
static const unsigned long CG_REPEAT_INTERVAL_MS = 300000UL; // 5 min
int cgRepeatCount = 0; // number of periodic repeats since cgArmedAtMs

// Manual tone (press-and-hold button on /callgever) to check
// frequency/volume by ear without having to wait through the whole call.
// Keeps a keepalive with a fail-safe timeout, just like webPtt.
bool          manualToneOn         = false;
unsigned long lastManualTonePingMs = 0;
static const unsigned long MANUAL_TONE_TIMEOUT_MS = 1200;

// ================================================================
//                          HTML pages
// ================================================================
const char AP_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WiFi setup</title>
<style>
:root{color-scheme:dark}
body{background:#111418;color:#e8e8e8;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;
     display:flex;flex-direction:column;align-items:center;padding:2.5rem 1.2rem;margin:0;min-height:100vh;box-sizing:border-box}
h1{font-size:1.3rem;font-weight:600;margin:0 0 .3rem}
p.hint{color:#9aa;font-size:.85rem;margin:0 0 1.8rem;text-align:center;max-width:320px}
form{width:100%;max-width:320px}
label{font-size:.85rem;color:#9aa;display:block;margin-bottom:.3rem}
input{background:#1c2026;border:1px solid #333;color:#eee;padding:.65rem;border-radius:8px;
      font-size:1rem;width:100%;box-sizing:border-box;margin-bottom:1.1rem}
button{width:100%;padding:.85rem;border:none;border-radius:8px;background:#3b82f6;color:#fff;
       font-size:1rem;font-weight:600;cursor:pointer}
button:active{background:#2563eb}
</style></head><body>
<h1>WiFi setup</h1>
<p class="hint">Enter your network details. The ESP32 will then restart and try to connect.</p>
<form method="POST" action="/save">
  <label>Network name (SSID)</label>
  <input name="ssid" required maxlength="32" autocomplete="off">
  <label>Password</label>
  <input name="pass" type="password" maxlength="64" autocomplete="off">
  <button type="submit">Save &amp; restart</button>
</form>
</body></html>
)rawliteral";

const char MAIN_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>PTT</title>
<style>
:root{color-scheme:dark}
html,body{overscroll-behavior:none}
body{background:#111418;color:#e8e8e8;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;
     display:flex;flex-direction:column;align-items:center;padding:2.5rem 1.2rem;margin:0;min-height:100vh;box-sizing:border-box}
h1{font-size:1.3rem;font-weight:600;margin:0 0 1.4rem}
.status-row{display:flex;align-items:center;gap:.5rem;margin-bottom:.5rem;font-size:.9rem;color:#9aa}
.status-row.last{margin-bottom:2.6rem}
.lamp{width:13px;height:13px;border-radius:50%;background:#444;transition:background .2s,box-shadow .2s}
.lamp.on{background:#22c55e;box-shadow:0 0 8px #22c55e}
.lamp.cg-on{background:#f59e0b;box-shadow:0 0 8px #f59e0b}
.ptt-btn{width:180px;height:180px;border-radius:50%;border:none;background:#262b32;
         box-shadow:0 0 0 5px #1a1e24,0 10px 24px rgba(0,0,0,.5);cursor:pointer;
         touch-action:none;user-select:none;-webkit-user-select:none;
         display:flex;align-items:center;justify-content:center;
         transition:background .12s,box-shadow .12s,transform .08s}
.ptt-btn svg{width:64px;height:64px;fill:#8a93a3;transition:fill .12s}
.ptt-btn.active{background:#dc2626;box-shadow:0 0 0 5px #1a1e24,0 0 34px rgba(220,38,38,.65)}
.ptt-btn.active svg{fill:#fff}
.ptt-btn:active{transform:scale(.96)}
a.setup-link{color:#7aa2ff;text-decoration:none;margin-top:1.4rem;font-size:.9rem;display:block}
.uptime{color:#444;font-size:.7rem;margin-top:1.2rem}
</style></head><body>
<h1>Remote PTT</h1>
<div class="status-row"><span class="lamp" id="lamp"></span><span id="linkText">remote connected</span></div>
<div class="status-row last"><span class="lamp" id="cglamp"></span><span id="cgText">CW-ID standby</span></div>
<button class="ptt-btn" id="pttBtn" aria-label="PTT">
  <svg viewBox="0 0 24 24"><path d="M12 14a3 3 0 0 0 3-3V6a3 3 0 0 0-6 0v5a3 3 0 0 0 3 3zm5-3a5 5 0 0 1-10 0H5a7 7 0 0 0 6 6.92V21h2v-3.08A7 7 0 0 0 19 11h-2z"/></svg>
</button>
<div style="margin-top:5rem;text-align:center">
  <a class="setup-link" href="/callgever">Callgever (CW-ID) settings &rarr;</a>
  <a class="setup-link" href="/setup">MQTT settings &rarr;</a>
</div>
<div class="uptime" id="uptime"></div>
<div class="uptime" id="wifiRssi"></div>
<script>
const lamp     = document.getElementById('lamp');
const linkText = document.getElementById('linkText');
const cglamp   = document.getElementById('cglamp');
const cgText   = document.getElementById('cgText');
const btn      = document.getElementById('pttBtn');
const uptimeEl = document.getElementById('uptime');
const rssiEl   = document.getElementById('wifiRssi');
let keepAlive = null;
let lastUptime = null;
let lastLocalAction = 0;   // timestamp of the last local press/release action
function fmtUptime(s){
  const m = Math.floor(s/60), sec = s%60;
  return m > 0 ? (m + 'm ' + sec + 's') : (sec + 's');
}
function fmtRssi(dbm){
  // Rule of thumb for WiFi signal strength: the closer to 0, the stronger.
  let label = 'excellent';
  if (dbm <= -80) label = 'very poor';
  else if (dbm <= -70) label = 'poor';
  else if (dbm <= -60) label = 'fair';
  else if (dbm <= -50) label = 'good';
  return 'WiFi: ' + dbm + ' dBm (' + label + ')';
}
async function poll(){
  try{
    const r = await fetch('/api/status', {cache:'no-store'});
    const j = await r.json();
    lamp.classList.toggle('on', j.link);
    linkText.textContent = j.link ? 'remote connected' : 'remote not connected';
    cglamp.classList.toggle('cg-on', j.cg);
    cgText.textContent = j.cg ? 'CW-ID transmitting...' : 'CW-ID standby';
    // Don't let the button color "snap back" right after our own
    // press/release, caused by a poll response that still predates that
    // action.
    if(Date.now() - lastLocalAction > 350){
      btn.classList.toggle('active', j.ptt);
    }
    if(lastUptime !== null && j.uptime < lastUptime){
      uptimeEl.textContent = 'ESP32 restarted! uptime: ' + fmtUptime(j.uptime);
    } else {
      uptimeEl.textContent = 'uptime: ' + fmtUptime(j.uptime);
    }
    lastUptime = j.uptime;
    rssiEl.textContent = fmtRssi(j.rssi);
  }catch(e){
    lamp.classList.remove('on');
    linkText.textContent = 'no connection to ESP32';
    rssiEl.textContent = '';
    cglamp.classList.remove('cg-on');
    cgText.textContent = '';
  }
}
setInterval(poll, 300);
poll();
function pttOn(){
  lastLocalAction = Date.now();
  btn.classList.add('active');           // immediate visual feedback, don't wait for the poll
  fetch('/api/ptt?on=1', {cache:'no-store'});
  if(!keepAlive) keepAlive = setInterval(() => { lastLocalAction = Date.now(); fetch('/api/ptt?on=1', {cache:'no-store'}); }, 250);
}
function pttOff(){
  if(keepAlive){ clearInterval(keepAlive); keepAlive = null; }
  lastLocalAction = Date.now();
  btn.classList.remove('active');        // immediate visual feedback
  fetch('/api/ptt?on=0', {cache:'no-store'});
}
btn.addEventListener('contextmenu', e => e.preventDefault());
btn.addEventListener('pointerdown', e => { e.preventDefault(); pttOn(); });
['pointerup','pointercancel','pointerleave'].forEach(ev => btn.addEventListener(ev, pttOff));
document.addEventListener('visibilitychange', () => { if(document.hidden) pttOff(); });
window.addEventListener('pagehide', pttOff);
</script>
</body></html>
)rawliteral";

// ================================================================
//                            setup()
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("=== ESP32 + HM-10 Remote PTT + Callgever ==="));

  pinMode(PTT_OUTPUT_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(LINK_OK_PIN, OUTPUT);
  pinMode(LINK_LOST_PIN, OUTPUT);
  if (ID_ACTIVE_PIN >= 0) {
    pinMode(ID_ACTIVE_PIN, OUTPUT);
    digitalWrite(ID_ACTIVE_PIN, LOW);
  }
  applyPttOutput(false);
  setLinkPinsLost();   // linkUp starts as false, so blink right away

  HM10.begin(HM10_BAUD, SERIAL_8N1, HM10_RX_PIN, HM10_TX_PIN);
  delay(100);
  Serial.println(F("HM-10 UART active at 9600 baud (RX=16, TX=17)"));

  // Make sure the HM-10 automatically reports OK+CONN/OK+LOST on
  // connect/disconnect (AT+NOTI0 would suppress these notifications).
  delay(50);
  HM10.print("AT+NOTI1");
  delay(150);
  {
    String notiResp;
    while (HM10.available()) notiResp += (char)HM10.read();
    if (notiResp.length() > 0) {
      Serial.print(F("HM-10 response to AT+NOTI1: \""));
      Serial.print(notiResp);
      Serial.println(F("\""));
    } else {
      Serial.println(F("No response to AT+NOTI1 (can be normal for this module, or it doesn't support this command)"));
    }
  }

  rxBuffer.reserve(64);

  prefs.begin("cfg", false);

  cgLoadConfig();

  String ssid = prefs.getString("wifi_ssid", "");
  String pass = prefs.getString("wifi_pass", "");

  bool connected = false;
  if (ssid.length() > 0) {
    Serial.print(F("Connecting to WiFi: "));
    Serial.println(ssid);
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();
    connected = (WiFi.status() == WL_CONNECTED);
  }

  if (connected) {
    startStationMode();
  } else {
    startApMode();
  }

  // IMPORTANT: only start audioInit() (and therefore the 40kHz DAC sample
  // interrupt) AFTER the WiFi connection process and mDNS.begin(), not
  // before. WiFi.begin() (and certainly the very first successful
  // connection ever after a fresh flash) can internally do flash write
  // operations (e.g. writing PHY calibration data), during which the
  // instruction cache is briefly disabled. If our ISR is already running
  // at that point and calls dacWrite() (an ordinary, non-IRAM function),
  // that can cause a Guru Meditation crash (LoadProhibited) - this is
  // exactly what happened before these lines were moved here. Our own
  // runtime NVS writes stay separately protected via
  // audioPause()/audioResume() (see cgSaveConfig()/handleSetupSave()).
  audioInit();
  morse.begin(audioToneOn, audioToneOff);
  cgApplyAudioSettings();
}

// ================================================================
//                             loop()
// ================================================================
void loop() {
  unsigned long now = millis();

  // ---- HM-10 handling (PTT detection) is deliberately placed FIRST,
  // before server.handleClient(). The latter can block for a few ms on a
  // TCP send when a web client is connected (e.g. the PTT page polling
  // /api/status every 300ms, or someone loading /callgever) - time that
  // would then no longer be spent reading/processing the HM-10 UART. By
  // doing this block first, every incoming byte from the HM-10 is
  // processed as quickly as possible after arrival, regardless of what
  // the web server happens to be doing at that moment. ----

  // ---- 1) Collect all incoming bytes from the HM-10 ----
  while (HM10.available()) {
    char c = (char)HM10.read();
    rxBuffer += c;
    if (rxBuffer.length() > 60) {
      rxBuffer.remove(0, rxBuffer.length() - 60);
    }
  }

  // ---- 2) Asynchronous connection notifications always take priority ----
  if (rxBuffer.length() > 0) {
    int idxLost = rxBuffer.indexOf("OK+LOST");
    int idxConn = rxBuffer.indexOf("OK+CONN");
    if (idxLost >= 0) {
      handleLinkLost();
      rxBuffer = "";
      waitingForResponse = false;
      nextPollAt = now + MIN_POLL_GAP_MS;
    } else if (idxConn >= 0) {
      handleLinkUp();
      rxBuffer = "";
      waitingForResponse = false;
      nextPollAt = now + MIN_POLL_GAP_MS;
    } else if (waitingForResponse) {
      int idxPio = rxBuffer.indexOf("OK+PIO3:");
      if (idxPio >= 0 && rxBuffer.length() >= (unsigned)(idxPio + 9)) {
        char stateChar = rxBuffer.charAt(idxPio + 8);
        if (linkUp) {
          handleNewReading(stateChar == '1' ? 1 : 0);
        }
        waitingForResponse = false;
        rxBuffer = "";
        nextPollAt = now + MIN_POLL_GAP_MS;
      }
    }
  }

  // ---- 3) Timeout: invalid/incomplete messages are ignored ----
  if (waitingForResponse && (now - requestSentAt > RESPONSE_TIMEOUT_MS)) {
    waitingForResponse = false;
    rxBuffer = "";
    nextPollAt = now;
    pollTimeoutCount++; // tracked silently; see possible debug extension to log this again
  }

  // ---- 4) Next AT+PIO3? once the minimum pause has elapsed ----
  if (!waitingForResponse && now >= nextPollAt) {
    if (linkUp) {
      requestSentAt = now;
      waitingForResponse = true;
      rxBuffer = "";
      HM10.print("AT+PIO3?");
    } else {
      nextPollAt = now + MIN_POLL_GAP_MS;
    }
  }

  // ---- Web server: may run after the time-critical HM-10 handling above ----
  server.handleClient();

  // Blink D19 (1s on/off) for as long as the link with the remote is down
  if (!linkUp && (now - lastLinkBlinkToggle >= LINK_LOST_BLINK_MS)) {
    lastLinkBlinkToggle = now;
    linkLostBlinkState = !linkLostBlinkState;
    digitalWrite(LINK_LOST_PIN, linkLostBlinkState ? HIGH : LOW);
  }

  // Fail-safe: no more keepalive received for the web-PTT button
  if (webPtt && (now - lastWebPttPing > WEB_PTT_TIMEOUT_MS)) {
    webPtt = false;
    updatePttOutput();
    Serial.println(F(">>> Web-PTT fail-safe: no keepalive received, disabled"));
  }

  // Fail-safe: no more keepalive received for the manual tone button
  if (manualToneOn && (now - lastManualTonePingMs > MANUAL_TONE_TIMEOUT_MS)) {
    manualToneOn = false;
    audioToneOff();
    updatePttOutput();
  }

  // ---- 5) Callgever: morse timing + trigger state machine ----
  morse.update();
  cgUpdate();

  // updatePttOutput() only actually switches the PTT output when
  // something changed (remotePtt, webPtt, or morse.isPlaying()) - safe to
  // call on every loop iteration.
  updatePttOutput();

  // ID_ACTIVE_PIN actually "draws" the morse code of the ID: high during
  // each dot/dash, low during the gaps in between (isKeyed()) - instead
  // of just staying continuously high for as long as the ID is
  // transmitting (isPlaying()). Applies both to an ID via the PTT trigger
  // and via the test button, but NOT during the manual (raw) test tone -
  // unlike STATUS_LED_PIN, which shows the broader PTT output status.
  if (ID_ACTIVE_PIN >= 0) {
    digitalWrite(ID_ACTIVE_PIN, morse.isKeyed() ? HIGH : LOW);
  }
}

// ================================================================
//                  WiFi: AP mode / station mode
// ================================================================
void startApMode() {
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);
  Serial.println(F("No (working) WiFi credentials - AP mode started."));
  Serial.print(F("Connect to network '"));
  Serial.print(AP_SSID);
  Serial.println(F("' and go to http://192.168.4.1/"));

  server.on("/", HTTP_GET, handleApRoot);
  server.on("/save", HTTP_POST, handleApSave);
  server.begin();
}

void startStationMode() {
  apMode = false;
  Serial.print(F("Connected! IP address: "));
  Serial.println(WiFi.localIP());
  if (MDNS.begin(MDNS_HOSTNAME)) {
    Serial.print(F("Also reachable via http://"));
    Serial.print(MDNS_HOSTNAME);
    Serial.println(F(".local/"));
  }

  server.on("/", HTTP_GET, handleStaRoot);
  server.on("/setup", HTTP_GET, handleSetupPage);
  server.on("/setup/save", HTTP_POST, handleSetupSave);
  server.on("/callgever", HTTP_GET, handleCallgeverPage);
  server.on("/callgever/save", HTTP_POST, handleCallgeverSave);
  server.on("/callgever/test", HTTP_POST, handleCallgeverTest);
  server.on("/callgever/tone", HTTP_GET, handleCallgeverTone);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/ptt", HTTP_GET, handleApiPtt);
  server.on("/api/wifi/reset", HTTP_POST, handleWifiReset);
  server.begin();
}

// ================================================================
//                       Web server handlers
// ================================================================
void handleApRoot() {
  server.send_P(200, "text/html", AP_PAGE);
}

void handleApSave() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  ssid.trim();
  if (ssid.length() == 0) {
    server.send(400, "text/plain", "SSID must not be empty");
    return;
  }
  audioPause();
  prefs.putString("wifi_ssid", ssid);
  prefs.putString("wifi_pass", pass);
  audioResume();

  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center;padding-top:3rem}</style>"
                "</head><body><h2>Saved!</h2><p>ESP32 is now restarting and will try to connect to <b>" + htmlEscape(ssid) + "</b>...</p></body></html>";
  server.send(200, "text/html", html);
  delay(1000);
  ESP.restart();
}

void handleStaRoot() {
  server.send_P(200, "text/html", MAIN_PAGE);
}

void handleSetupPage() {
  String host = prefs.getString("mqtt_host", "");
  String port = prefs.getString("mqtt_port", "1883");
  String user = prefs.getString("mqtt_user", "");
  String pass = prefs.getString("mqtt_pass", "");
  bool   saved = server.hasArg("saved");

  String html;
  html.reserve(2200);
  html += F("<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>MQTT settings</title><style>"
            ":root{color-scheme:dark}"
            "body{background:#111418;color:#e8e8e8;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;"
            "display:flex;flex-direction:column;align-items:center;padding:2.5rem 1.2rem;margin:0;min-height:100vh;box-sizing:border-box}"
            "h1{font-size:1.3rem;font-weight:600;margin:0 0 1.4rem}"
            "form{width:100%;max-width:320px}"
            "label{font-size:.85rem;color:#9aa;display:block;margin-bottom:.3rem}"
            "input{background:#1c2026;border:1px solid #333;color:#eee;padding:.65rem;border-radius:8px;"
            "font-size:1rem;width:100%;box-sizing:border-box;margin-bottom:1.1rem}"
            "button.primary{width:100%;padding:.85rem;border:none;border-radius:8px;background:#3b82f6;color:#fff;"
            "font-size:1rem;font-weight:600;cursor:pointer;margin-bottom:.6rem}"
            "a{color:#7aa2ff;text-decoration:none;margin-top:1.6rem;font-size:.9rem;display:inline-block}"
            "p.msg{color:#4ade80;font-size:.9rem}"
            ".danger{width:100%;max-width:320px;margin-top:2.2rem;padding-top:1.4rem;border-top:1px solid #262b32}"
            ".danger button{width:100%;padding:.7rem;border-radius:8px;background:#2a1a1a;color:#f88;"
            "border:1px solid #5c2626;font-size:.9rem;cursor:pointer}"
            "</style></head><body>");
  html += F("<h1>MQTT settings</h1>");
  if (saved) html += F("<p class='msg'>Saved!</p>");
  html += F("<form method='POST' action='/setup/save'>");
  html += F("<label>Broker (host or IP)</label><input name='host' value='"); html += htmlEscape(host); html += F("' required>");
  html += F("<label>Port</label><input name='port' inputmode='numeric' value='"); html += htmlEscape(port); html += F("'>");
  html += F("<label>Username</label><input name='user' value='"); html += htmlEscape(user); html += F("'>");
  html += F("<label>Password</label><input name='pass' type='password' value='"); html += htmlEscape(pass); html += F("'>");
  html += F("<button class='primary' type='submit'>Save</button>");
  html += F("</form>");
  html += F("<a href='/'>&larr; Back to PTT</a>");
  html += F("<div class='danger'><form method='POST' action='/api/wifi/reset' "
            "onsubmit=\"return confirm('Clear WiFi settings and restart the ESP32 in setup mode?')\">"
            "<button type='submit'>Reset WiFi settings</button></form></div>");
  html += F("</body></html>");
  server.send(200, "text/html", html);
}

void handleSetupSave() {
  audioPause();
  prefs.putString("mqtt_host", server.arg("host"));
  prefs.putString("mqtt_port", server.arg("port"));
  prefs.putString("mqtt_user", server.arg("user"));
  prefs.putString("mqtt_pass", server.arg("pass"));
  audioResume();
  // audioPause() hard-resets the envelope to silent; if the manual tone
  // button on /callgever was still being held down, the tone needs to be
  // started again.
  if (manualToneOn) audioToneOn();
  server.sendHeader("Location", "/setup?saved=1");
  server.send(303);
}

void handleApiStatus() {
  server.sendHeader("Cache-Control", "no-store");
  // Fixed buffer instead of String concatenation: this endpoint is
  // requested by the page every 300ms. "cg" indicates whether the
  // callgever is currently transmitting its morse ID. "rssi" is the WiFi
  // signal strength (dBm, negative, closer to 0 = stronger) - handy to
  // see in the browser whether a bad connection is the cause when the
  // web-PTT drops out/chatters (see WEB_PTT_TIMEOUT_MS above).
  char buf[96];
  snprintf(buf, sizeof(buf), "{\"link\":%s,\"ptt\":%s,\"cg\":%s,\"uptime\":%lu,\"rssi\":%d}",
           linkUp ? "true" : "false",
           outputActive ? "true" : "false",
           morse.isPlaying() ? "true" : "false",
           millis() / 1000UL,
           apMode ? 0 : WiFi.RSSI());
  server.send(200, "application/json", buf);
}

void handleApiPtt() {
  server.sendHeader("Cache-Control", "no-store");
  if (server.hasArg("on")) {
    webPtt = (server.arg("on") == "1");
    lastWebPttPing = millis();
    updatePttOutput();
  }
  server.send(200, "text/plain", "OK");
}

void handleWifiReset() {
  audioPause();
  prefs.remove("wifi_ssid");
  prefs.remove("wifi_pass");
  audioResume();

  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center;padding-top:3rem}</style>"
                "</head><body><h2>WiFi settings cleared</h2><p>ESP32 is now restarting in setup mode...</p></body></html>";
  server.send(200, "text/html", html);
  delay(1000);
  ESP.restart();
}

// ---- Callgever web page ----
static long cgClamp(long v, long lo, long hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Small helper to safely place text inside an HTML attribute.
String htmlEscape(const String &in) {
  String out;
  out.reserve(in.length());
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out += c; break;
    }
  }
  return out;
}

void handleCallgeverPage() {
  bool playing = morse.isPlaying();

  String html;
  html.reserve(6200);
  html += F("<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width, initial-scale=1'>"
            "<title>Callgever</title><style>"
            ":root{color-scheme:dark}"
            "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;background:#111418;color:#e2e8f0;"
            "max-width:480px;margin:0 auto;padding:20px 16px 60px}"
            "h1{font-size:22px;margin-bottom:4px}"
            ".sub{color:#94a3b8;font-size:13px;margin-bottom:20px}"
            ".card{background:#1c2026;border-radius:12px;padding:18px;margin-bottom:16px}"
            "label{display:block;font-size:14px;color:#cbd5e1;margin-bottom:6px;font-weight:600}"
            "input[type=text]{width:100%;box-sizing:border-box;background:#111418;"
            "border:1px solid #334155;color:#f1f5f9;border-radius:8px;padding:10px;font-size:16px;margin-bottom:14px}"
            "input[type=range]{width:100%;margin-bottom:6px}"
            ".rowval{display:flex;justify-content:space-between;font-size:13px;color:#94a3b8;margin-bottom:14px}"
            ".switchrow{display:flex;align-items:center;justify-content:space-between;margin-bottom:4px}"
            "button{width:100%;padding:13px;border:none;border-radius:8px;font-size:16px;font-weight:600;"
            "cursor:pointer;margin-top:6px}"
            ".save{background:#22c55e;color:#052e12}"
            ".test{background:#334155;color:#e2e8f0}"
            ".tone{background:#334155;color:#e2e8f0;touch-action:none;user-select:none;-webkit-user-select:none}"
            ".tone.active{background:#f59e0b;color:#1c1305}"
            ".status{font-size:13px;color:#94a3b8;line-height:1.6}"
            ".dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:6px}"
            ".on{background:#f59e0b}.off{background:#64748b}"
            "a{color:#7aa2ff;text-decoration:none;margin-top:1.2rem;font-size:.9rem;display:inline-block}"
            "</style></head><body>"
            "<h1>&#128225; Callgever</h1>"
            "<div class='sub'>CW-ID beacon &mdash; triggered by PTT (remote or web page)</div>");

  html += F("<div class='card status'>");
  html += "<div><span class='dot ";
  html += (playing ? "on" : "off");
  html += "'></span>";
  html += playing ? "Transmitting the ID now..." : "Standby (waiting for PTT)";
  html += "</div>";
  html += F("</div>");

  html += F("<form method='POST' action='/callgever/save'>");
  html += F("<div class='card'>");

  html += F("<label for='callsign'>Callsign</label>");
  html += "<input type='text' id='callsign' name='callsign' maxlength='15' value='" +
          htmlEscape(String(cg.callsign)) + "' required>";

  html += F("<div class='switchrow'>"
            "<label for='enabled' style='margin:0'>Callgever enabled</label>"
            "<input type='checkbox' id='enabled' name='enabled' style='width:22px;height:22px'");
  if (cg.enabled) html += F(" checked");
  html += F("></div>");

  html += F("<div class='switchrow' style='margin-top:14px'>"
            "<label for='abortrel' style='margin:0'>Releasing PTT aborts the ID immediately</label>"
            "<input type='checkbox' id='abortrel' name='abortrel' style='width:22px;height:22px'");
  if (cg.abortOnRelease) html += F(" checked");
  html += F("></div>");
  html += F("<p style='font-size:12px;color:#64748b;margin-top:6px;margin-bottom:0'>Off (default): once an "
            "ID has started, it always finishes, even if you release the PTT earlier. On: the ID stops "
            "immediately as soon as you release the PTT. If the ID finishes normally (not aborted), the "
            "transmitter then sends another 500 ms of silence before the PTT output releases.</p>");

  html += F("</div>"); // card

  html += F("<div class='card'>");

  html += F("<label for='volume'>Volume</label>");
  html += "<input type='range' id='volume' name='volume' min='0' max='100' step='1' value='" +
          String(cg.volume) +
          "' oninput=\"document.getElementById('volumeval').innerText=this.value+' %'\">";
  html += "<div class='rowval'><span>0%</span><span id='volumeval'>" + String(cg.volume) + " %</span><span>100%</span></div>";

  html += F("<label for='wpm'>Morse speed (WPM)</label>");
  html += "<input type='range' id='wpm' name='wpm' min='5' max='30' step='1' value='" +
          String(cg.wpm) +
          "' oninput=\"document.getElementById('wpmval').innerText=this.value+' wpm'\">";
  html += "<div class='rowval'><span>5 wpm</span><span id='wpmval'>" + String(cg.wpm) + " wpm</span><span>30 wpm</span></div>";

  html += F("<label for='freq'>Tone frequency</label>");
  html += "<input type='range' id='freq' name='freq' min='400' max='1500' step='10' value='" +
          String(cg.freqHz) +
          "' oninput=\"document.getElementById('freqval').innerText=this.value+' Hz'\">";
  html += "<div class='rowval'><span>400 Hz</span><span id='freqval'>" + String(cg.freqHz) + " Hz</span><span>1500 Hz</span></div>";

  html += F("<label for='didelay'>Delay after PTT start</label>");
  html += "<input type='range' id='didelay' name='didelay' min='0' max='50000' step='10' value='" +
          String(cg.delayMs) +
          "' oninput=\"document.getElementById('didelayval').innerText=this.value+' ms'\">";
  html += "<div class='rowval'><span>0 ms</span><span id='didelayval'>" + String(cg.delayMs) + " ms</span><span>50000 ms</span></div>";

  html += F("</div>"); // card

  html += F("<div class='card'>");
  html += F("<label for='attackms'>Attack (ramp-up time)</label>");
  html += "<input type='range' id='attackms' name='attackms' min='1' max='20' step='1' value='" +
          String(cg.attackMs) +
          "' oninput=\"document.getElementById('attackmsval').innerText=this.value+' ms'\">";
  html += "<div class='rowval'><span>1 ms</span><span id='attackmsval'>" + String(cg.attackMs) + " ms</span><span>20 ms</span></div>";

  html += F("<label for='releasems'>Release (ramp-down time)</label>");
  html += "<input type='range' id='releasems' name='releasems' min='1' max='50' step='1' value='" +
          String(cg.releaseMs) +
          "' oninput=\"document.getElementById('releasemsval').innerText=this.value+' ms'\">";
  html += "<div class='rowval'><span>1 ms</span><span id='releasemsval'>" + String(cg.releaseMs) + " ms</span><span>50 ms</span></div>";
  html += F("<p style='font-size:12px;color:#64748b;margin-top:-6px'>Attack/release determine how smoothly each "
            "dot/dash ramps up/down (prevents an audible \"click\"). Note: at higher WPM a dot becomes shorter "
            "than the release time can be (e.g. at 20 wpm, 1 dit = 60ms) - the tone then no longer fully dies "
            "away between two elements. So choose release time and speed together.</p>");
  html += F("</div>"); // card

  html += F("<button type='submit' class='save'>Save</button>");
  html += F("</form>");

  html += F("<form method='POST' action='/callgever/test'>"
            "<button type='submit' class='test'>Test transmission now</button>"
            "</form>");
  html += F("<p style='font-size:12px;color:#64748b'>Note: the test button also actually activates the PTT "
            "output (D21) for as long as the ID is playing, so if the transmitter is connected it will really "
            "transmit.</p>");

  html += F("<button type='button' class='tone' id='toneBtn'>Tone (press and hold)</button>"
            "<p style='font-size:12px;color:#64748b'>Plays the plain tone at the last saved "
            "volume/frequency for as long as you hold the button down (e.g. to adjust things) - like "
            "the test button, this also actually activates the PTT output. Doesn't work while an ID "
            "transmission is in progress.</p>");

  html += F("<a href='/'>&larr; Back to PTT</a>");

  html += F(
      "<script>"
      "const toneBtn=document.getElementById('toneBtn');"
      "let toneKeepAlive=null;"
      "function toneOn(){"
      "toneBtn.classList.add('active');"
      "fetch('/callgever/tone?on=1',{cache:'no-store'});"
      "if(!toneKeepAlive)toneKeepAlive=setInterval(()=>fetch('/callgever/tone?on=1',{cache:'no-store'}),300);"
      "}"
      "function toneOff(){"
      "if(toneKeepAlive){clearInterval(toneKeepAlive);toneKeepAlive=null;}"
      "toneBtn.classList.remove('active');"
      "fetch('/callgever/tone?on=0',{cache:'no-store'});"
      "}"
      "toneBtn.addEventListener('contextmenu',e=>e.preventDefault());"
      "toneBtn.addEventListener('pointerdown',e=>{e.preventDefault();toneOn();});"
      "['pointerup','pointercancel','pointerleave'].forEach(ev=>toneBtn.addEventListener(ev,toneOff));"
      "document.addEventListener('visibilitychange',()=>{if(document.hidden)toneOff();});"
      "window.addEventListener('pagehide',toneOff);"
      "</script>");

  html += F("</body></html>");

  server.send(200, "text/html", html);
}

void handleCallgeverSave() {
  if (server.hasArg("callsign")) {
    String cs = server.arg("callsign");
    cs.trim();
    cs.toUpperCase();
    if (cs.length() > 0) {
      strncpy(cg.callsign, cs.c_str(), sizeof(cg.callsign) - 1);
      cg.callsign[sizeof(cg.callsign) - 1] = '\0';
    }
  }

  cg.enabled = server.hasArg("enabled"); // checkbox: only present when checked
  cg.abortOnRelease = server.hasArg("abortrel"); // checkbox: only present when checked

  if (server.hasArg("volume"))
    cg.volume = (uint8_t)cgClamp(server.arg("volume").toInt(), 0, 100);
  if (server.hasArg("wpm"))
    cg.wpm = (uint8_t)cgClamp(server.arg("wpm").toInt(), 5, 30);
  if (server.hasArg("freq"))
    cg.freqHz = (uint16_t)cgClamp(server.arg("freq").toInt(), 400, 1500);
  if (server.hasArg("didelay"))
    cg.delayMs = (uint16_t)cgClamp(server.arg("didelay").toInt(), 0, 50000);
  if (server.hasArg("attackms"))
    cg.attackMs = (uint8_t)cgClamp(server.arg("attackms").toInt(), 1, 20);
  if (server.hasArg("releasems"))
    cg.releaseMs = (uint8_t)cgClamp(server.arg("releasems").toInt(), 1, 50);

  cgSaveConfig();
  cgApplyAudioSettings();

  server.sendHeader("Location", "/callgever");
  server.send(303);
}

void handleCallgeverTest() {
  if (!manualToneOn && !morse.isPlaying() && strlen(cg.callsign) > 0) {
    cgApplyAudioSettings();
    morse.play(cg.callsign);
  }
  server.sendHeader("Location", "/callgever");
  server.send(303);
}

// Manual tone on/off (press-and-hold button on /callgever). Like the test
// button, this also actually activates the PTT output (D21) for as long
// as the tone sounds - see updatePttOutput().
void handleCallgeverTone() {
  server.sendHeader("Cache-Control", "no-store");

  if (server.hasArg("on") && server.arg("on") == "1") {
    if (morse.isPlaying()) {
      // An automatic ID transmission is already running - ignore the manual tone.
      server.send(409, "text/plain", "BUSY");
      return;
    }
    if (!manualToneOn) {
      cgApplyAudioSettings();
      audioToneOn();
    }
    manualToneOn = true;
    lastManualTonePingMs = millis();
    updatePttOutput();
  } else {
    if (manualToneOn) {
      audioToneOff();
    }
    manualToneOn = false;
    updatePttOutput();
  }
  server.send(200, "text/plain", "OK");
}

// ================================================================
//         HM-10: debounce, connection notifications, output
// ================================================================
void handleNewReading(int newState) {
  if (newState == candidateState) {
    candidateCount++;
  } else {
    candidateState = newState;
    candidateCount = 1;
  }
  if (candidateCount >= DEBOUNCE_COUNT) {
    bool newRemotePtt = (candidateState == 1);
    if (newRemotePtt != remotePtt) {
      remotePtt = newRemotePtt;
      Serial.print(F(">>> Remote PTT "));
      Serial.println(remotePtt ? F("PRESSED") : F("RELEASED"));
      updatePttOutput();
    }
  }
}

void handleLinkLost() {
  if (linkUp) {
    Serial.println(F(">>> Remote LINK LOST (OK+LOST)"));
  }
  linkUp = false;
  candidateState = -1;
  candidateCount = 0;
  setLinkPinsLost();
  if (remotePtt) {
    remotePtt = false;
    Serial.println(F(">>> Remote PTT released (fail-safe, no connection to remote)"));
  }
  updatePttOutput();
}

void handleLinkUp() {
  if (!linkUp) {
    Serial.println(F(">>> Remote connected (OK+CONN)"));
  }
  linkUp = true;
  candidateState = -1;
  candidateCount = 0;
  setLinkPinsConnected();
}

void setLinkPinsConnected() {
  digitalWrite(LINK_OK_PIN, HIGH);
  digitalWrite(LINK_LOST_PIN, LOW);
}

void setLinkPinsLost() {
  digitalWrite(LINK_OK_PIN, LOW);
  linkLostBlinkState = true;
  digitalWrite(LINK_LOST_PIN, HIGH);
  lastLinkBlinkToggle = millis();
}

// Combines the three independent sources (remote via BLE, the web page,
// and the callgever transmitting on its own) into one PTT output state.
void updatePttOutput() {
  // cgState == CG_TAIL: the ID has finished playing (so morse.isPlaying()
  // is already false), but we want to keep the PTT output active a
  // little longer (CG_TAIL_SILENCE_MS) for the tail silence - hence this
  // is explicitly included here.
  bool want = remotePtt || webPtt || morse.isPlaying() || manualToneOn || (cgState == CG_TAIL);
  if (want != outputActive) {
    outputActive = want;
    applyPttOutput(outputActive);
    Serial.print(F(">>> PTT output "));
    Serial.println(outputActive ? F("ON") : F("OFF"));
  }
}

void applyPttOutput(bool active) {
  bool level = PTT_ACTIVE_HIGH ? active : !active;
  digitalWrite(PTT_OUTPUT_PIN, level ? HIGH : LOW);
  digitalWrite(STATUS_LED_PIN, active ? HIGH : LOW);
}

// ================================================================
//         Callgever: load/save settings + trigger logic
// ================================================================
void cgLoadConfig() {
  strncpy(cg.callsign, CG_DEFAULT_CALLSIGN, sizeof(cg.callsign) - 1);
  cg.callsign[sizeof(cg.callsign) - 1] = '\0';
  cg.enabled = CG_DEFAULT_ENABLED;
  cg.volume = CG_DEFAULT_VOLUME;
  cg.wpm = CG_DEFAULT_WPM;
  cg.freqHz = CG_DEFAULT_FREQ_HZ;
  cg.delayMs = CG_DEFAULT_DELAY_MS;
  cg.attackMs = CG_DEFAULT_ATTACK_MS;
  cg.releaseMs = CG_DEFAULT_RELEASE_MS;
  cg.abortOnRelease = CG_DEFAULT_ABORT_ON_RELEASE;

  String cs = prefs.getString("cg_call", cg.callsign);
  cs.trim();
  cs.toUpperCase();
  if (cs.length() > 0) {
    strncpy(cg.callsign, cs.c_str(), sizeof(cg.callsign) - 1);
    cg.callsign[sizeof(cg.callsign) - 1] = '\0';
  }
  cg.enabled = prefs.getBool("cg_en", cg.enabled);
  cg.volume = (uint8_t)prefs.getUInt("cg_vol", cg.volume);
  cg.wpm = (uint8_t)prefs.getUInt("cg_wpm", cg.wpm);
  cg.freqHz = (uint16_t)prefs.getUInt("cg_freq", cg.freqHz);
  cg.delayMs = (uint16_t)prefs.getUInt("cg_delay", cg.delayMs);
  cg.attackMs = (uint8_t)prefs.getUInt("cg_atk", cg.attackMs);
  cg.releaseMs = (uint8_t)prefs.getUInt("cg_rel", cg.releaseMs);
  cg.abortOnRelease = prefs.getBool("cg_abortrel", cg.abortOnRelease);
}

void cgSaveConfig() {
  audioPause(); // prevents the sample interrupt from firing during the flash write operation
  prefs.putString("cg_call", cg.callsign);
  prefs.putBool("cg_en", cg.enabled);
  prefs.putUInt("cg_vol", cg.volume);
  prefs.putUInt("cg_wpm", cg.wpm);
  prefs.putUInt("cg_freq", cg.freqHz);
  prefs.putUInt("cg_delay", cg.delayMs);
  prefs.putUInt("cg_atk", cg.attackMs);
  prefs.putUInt("cg_rel", cg.releaseMs);
  prefs.putBool("cg_abortrel", cg.abortOnRelease);
  audioResume();
  // audioPause() hard-resets the envelope to silent; if the manual tone
  // button was still being held down while saving, the tone needs to be
  // started again.
  if (manualToneOn) audioToneOn();
}

void cgApplyAudioSettings() {
  audioSetFrequency(cg.freqHz);
  audioSetVolume(cg.volume);
  morse.setWpm(cg.wpm);
  audioSetAttackMs(cg.attackMs);
  audioSetReleaseMs(cg.releaseMs);
}

// Non-blocking trigger state machine: uses the combined
// (remotePtt || webPtt) as the "DI signal" instead of a physical pin.
void cgUpdate() {
  bool pttNow = (remotePtt || webPtt);

  // Note: the "release -> back to IDLE" transitions below deliberately do
  // NOT check cg.enabled (unlike the "goes on -> arm" transition, which
  // does). If you did check it there too (as used to be the case, via a
  // separate if(cg.enabled){...}else{...} structure), the following could
  // go wrong: the callgever gets disabled while cgState is still
  // CG_WAIT_RELEASE (PTT still pressed); the PTT is released and pressed
  // again while the callgever is still off - cgState then stays "stuck"
  // on CG_WAIT_RELEASE with an old cgArmedAtMs/cgRepeatCount, and the new
  // press doesn't count as a fresh edge (cgPttPrev was already true). If
  // the callgever is then re-enabled while the PTT happens to still be
  // pressed, it incorrectly resumes the old cycle instead of simply
  // waiting for a genuinely new edge. By always letting the reset
  // transitions run (regardless of cg.enabled), cgState always goes
  // straight back to IDLE as soon as the PTT is released, with a clean
  // cgRepeatCount - just as was already the case with the standalone
  // callgever (DI pin).
  if (pttNow && !cgPttPrev && cgState == CG_IDLE && cg.enabled) {
    // PTT goes on -> "arm" the callgever, let the delay run. Also arm if
    // a manual tone happens to be playing right now: the edge must not be
    // lost (otherwise the mandatory ID would get skipped). The
    // "if (manualToneOn) break;" below in the ARMED case holds back the
    // actual start until the tone button is released again.
    cgState = CG_ARMED;
    cgArmedAtMs = millis();
    cgRepeatCount = 0;
  } else if (!pttNow && cgState == CG_ARMED) {
    // PTT released before the delay had elapsed -> cancel
    cgState = CG_IDLE;
    cgRepeatCount = 0;
  } else if (!pttNow && cg.abortOnRelease && (cgState == CG_TRANSMITTING || cgState == CG_TAIL)) {
    // New "abort immediately on PTT release" option: unlike the default
    // behaviour (an ID that has started always finishes, see
    // CG_TRANSMITTING below), the transmission now stops IMMEDIATELY as
    // soon as the PTT is released - both during the ID itself and during
    // the tail silence (CG_TAIL) afterward. No tail silence in this case:
    // this is a deliberate, immediate abort action, not a normal
    // completion.
    if (morse.isPlaying()) morse.stop();
    cgState = CG_IDLE;
    cgRepeatCount = 0;
  } else if (!pttNow && cgState == CG_WAIT_RELEASE) {
    cgState = CG_IDLE;
    cgRepeatCount = 0;
  }
  cgPttPrev = pttNow;

  switch (cgState) {
    case CG_ARMED:
      if (!cg.enabled) {
        cgState = CG_IDLE;
        break;
      }
      if (manualToneOn) break; // manual tone takes priority, wait
      if ((millis() - cgArmedAtMs) >= cg.delayMs) {
        // If a transmission happens to already be running via the test
        // button (handleCallgeverTest), wait for it to finish instead of
        // interrupting/restarting it.
        if (morse.isPlaying()) break;
        cgApplyAudioSettings();
        morse.play(cg.callsign);
        cgState = CG_TRANSMITTING;
      }
      break;

    case CG_TRANSMITTING:
      if (!cg.enabled) {
        // Callgever disabled in the meantime from the web page -> go silent immediately
        morse.stop();
        cgState = CG_IDLE;
        cgRepeatCount = 0;
        break;
      }
      if (!morse.isPlaying()) {
        // ID fully played out (if this had been an abort instead of a
        // normal completion, we would already have gone straight to
        // CG_IDLE above - so we only get here if the ID was allowed to
        // transmit normally). Go to CG_TAIL first: the PTT output stays
        // active for CG_TAIL_SILENCE_MS (silence) before it releases.
        cgTailAtMs = millis();
        cgState = CG_TAIL;
      }
      break;

    case CG_TAIL:
      if (!cg.enabled) {
        // Callgever disabled in the meantime -> stop the tail silence immediately too
        cgState = CG_IDLE;
        cgRepeatCount = 0;
        break;
      }
      if ((millis() - cgTailAtMs) >= CG_TAIL_SILENCE_MS) {
        // Tail silence over - same continuation as before, right after
        // the ID: wait for release (or, on a long uninterrupted PTT, the
        // mandatory periodic repeat below in CG_WAIT_RELEASE).
        cgState = pttNow ? CG_WAIT_RELEASE : CG_IDLE;
        if (cgState == CG_IDLE) cgRepeatCoun