/*
  ================================================================
  ESP32 + HM-10 (BLE)  ->  Remote PTT schakelaar + Callgever (CW-ID)
  ================================================================
  Dit is de oorspronkelijke "ESP32 + HM-10 Remote PTT" sketch, uitgebreid met
  de callgever (morse-ID beacon). De callgever heeft nu GEEN eigen fysieke
  DI-pin meer: hij wordt getriggerd door dezelfde gecombineerde PTT-status
  (remote via HM-10 OF de PTT-knop op de webpagina) die ook de PTT-uitgang
  aanstuurt.

  DEEL 1 - PTT via HM-10 (UART, 9600 baud, pins 16/17)
  -----------------------------------------------------
    Ongewijzigd t.o.v. de oorspronkelijke sketch: continu "AT+PIO3?" pollen,
    OK+PIO3:0/1 verwerken, OK+LOST/OK+CONN als asynchrone verbindingsmelding.

  DEEL 2 - Uitgangen
  -----------------------------------------------------
    D21 (PTT_OUTPUT_PIN) wordt actief zodra PTT actief moet zijn: via de
    remote, via de PTT-knop op de webpagina, OF omdat de callgever op dit
    moment zijn morse-ID aan het uitzenden is (zie DEEL 4). Dat laatste is
    nodig zodat de zender ook echt aan blijft staan als de operator de
    PTT-knop al losgelaten heeft terwijl de ID nog bezig is.
    D18/D19 tonen de BLE-linkstatus, zoals voorheen.

  DEEL 3 - WiFi + webpagina (dark mode)
  -----------------------------------------------------
    Ongewijzigd: AP-setup-modus voor WiFi-gegevens, hoofdpagina met PTT-knop,
    /setup voor MQTT-gegevens. De hoofdpagina toont nu ook een tweede
    statusregel ("CW-ID stand-by" / "CW-ID wordt uitgezonden...") en een link
    naar de nieuwe /callgever configuratiepagina.

  DEEL 4 - Callgever (CW-ID beacon), NIEUW
  -----------------------------------------------------
    Zodra de gecombineerde PTT-status (remotePtt || webPtt) van laag naar
    hoog gaat (PTT wordt ingedrukt) EN de callgever ingeschakeld is, wacht
    de ESP32 de ingestelde vertraging (0-25000 ms) en zendt daarna de
    ingestelde roepnaam in morse uit, als analoge sinustoon op de ingebouwde
    DAC (GPIO25, 8-bit, alleen op de klassieke ESP32 WROOM/WROVER - niet op
    S2/S3/C3/C6).
      - Laat je de PTT los vóórdat de vertraging voorbij is: de ID wordt
        geannuleerd.
      - Eenmaal gestart wordt de ID altijd afgemaakt (de PTT-uitgang blijft
        hiervoor actief, ook als de operator/remote intussen loslaat) -
        behalve als je de callgever tussentijds via de webpagina uitschakelt,
        dan stopt hij direct.
      - Blijft de PTT na afloop van de ID nog steeds ingedrukt, dan wordt er
        pas weer een nieuwe ID geaccepteerd nadat de PTT ook echt losgelaten
        is (voorkomt continu herhalen zolang iemand de knop ingedrukt houdt).
    Instelbaar via de nieuwe /callgever pagina: roepnaam, aan/uit, volume
    (0-100%), snelheid (5-30 WPM) en toonfrequentie (400-1500 Hz). Alles
    wordt net als de WiFi/MQTT-instellingen in NVS-flash bewaard (zelfde
    "cfg" namespace, aparte "cg_*" keys).

    Hardware-opmerking: GPIO25 (de DAC-uitgang) moet je met een
    serie-condensator (DC-blokkering) en eventueel een spanningsdeler naar
    mic-niveau brengen voordat je hem samen met (of i.p.v.) het microfoon-
    signaal op de zender aansluit - zie README.md.

  Benodigde libraries: allemaal al aanwezig in het ESP32 board-package
  (WiFi, WebServer, Preferences, ESPmDNS) - niets extra installeren, behalve
  de meegeleverde morse.h/.cpp en audio_dac.h/.cpp bestanden in deze map.
  ================================================================
*/
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>

#include "morse.h"
#include "audio_dac.h"

// ================= Instellingen: HM-10 / PTT =================
#define HM10_RX_PIN          16     // ESP32 RX2  <-- HM-10 TX
#define HM10_TX_PIN          17     // ESP32 TX2  --> HM-10 RX
#define HM10_BAUD            9600
#define PTT_OUTPUT_PIN       21     // naar transistor -> relais -> PTT-lijn portofoon/zender
#define PTT_ACTIVE_HIGH      true   // zet op false voor een actief-lage uitgang
#define STATUS_LED_PIN       2      // onboard LED op de meeste ESP32 devboards

// DO die de morse-code van de ID daadwerkelijk uittekent: hoog tijdens elke
// dot/dash, laag in de pauzes ertussen (dus NIET gewoon continu hoog tijdens
// de hele uitzending, NIET zolang de PTT-uitgang om andere redenen actief
// is, en ook niet tijdens de handmatige testtoon-knop op /callgever) -
// handig om bv. een extern apparaat/logger/CW-decoder rechtstreeks op de
// morse-code te laten reageren. Zet op -1 om uit te zetten.
#define ID_ACTIVE_PIN        4

// Twee extra uitgangen die de BLE-link met de remote tonen:
//   verbonden : LINK_OK_PIN hoog, LINK_LOST_PIN laag (allebei stabiel)
//   link weg  : LINK_OK_PIN laag, LINK_LOST_PIN knippert (1 Hz)
#define LINK_OK_PIN          18     // D18 - hoog zolang de remote verbonden is
#define LINK_LOST_PIN        19     // D19 - knippert (1s aan/uit) zolang de link weg is
#define LINK_LOST_BLINK_MS   1000

// AT+PIO3? is een lokale vraag aan het HM-10 (geen BLE round-trip), dus het
// antwoord komt bij een normale module vrijwel meteen terug. MIN_POLL_GAP_MS
// en RESPONSE_TIMEOUT_MS bepalen samen hoe snel een PTT-druk op de remote
// hoogstens gezien kan worden (elke druk die korter duurt dan de effectieve
// pollcyclus valt tussen twee metingen door en wordt onvermijdelijk gemist -
// dat is inherent aan polling). Beide zijn daarom zo krap mogelijk gezet;
// mocht dit in de praktijk juist tot AT-communicatiefouten met de HM-10
// leiden (bv. "resync"-gedrag of vaker een timeout dan voorheen), zet ze dan
// weer wat ruimer (bv. terug naar 20 / 150).
#define MIN_POLL_GAP_MS      5      // minimale pauze tussen twee aanvragen
#define RESPONSE_TIMEOUT_MS  50     // max. wachttijd op antwoord voordat we opgeven
#define DEBOUNCE_COUNT       1      // 1 = direct reageren; hoger (2-3) bij ruis/glitches

// ================= Instellingen: WiFi / web =================
#define AP_SSID              "ESP32-PTT-Setup"   // eigen access point tijdens setup
#define WIFI_CONNECT_TIMEOUT_MS  15000            // hoe lang proberen te verbinden voor terugval naar AP
#define WEB_PTT_TIMEOUT_MS   1500                 // fail-safe: geen keepalive -> web-PTT uit
#define MDNS_HOSTNAME         "esp32-ptt"          // bereikbaar via http://esp32-ptt.local/

// ================= Instellingen: Callgever (CW-ID) =================
// GEEN aparte DI-pin meer - de trigger is de gecombineerde PTT-status
// hierboven (remotePtt || webPtt). Zie DEEL 4 hierboven voor het gedrag.
#define CG_DEFAULT_CALLSIGN  "PD0TEST"
#define CG_DEFAULT_ENABLED   true
#define CG_DEFAULT_VOLUME    70    // 0-100 %
#define CG_DEFAULT_WPM       15    // 5-30 WPM
#define CG_DEFAULT_FREQ_HZ   800   // 400-1500 Hz
#define CG_DEFAULT_DELAY_MS  25000 // 0-25000 ms, vertraging na start-PTT voordat de ID begint
#define CG_DEFAULT_ATTACK_MS  1    // 1-20 ms, oprijtijd van de toon-envelop
#define CG_DEFAULT_RELEASE_MS 10   // 10-100 ms, uitrijtijd van de toon-envelop
#define CG_DEFAULT_ABORT_ON_RELEASE false // true = PTT loslaten breekt een lopende ID direct af

// Vaste (niet via de webpagina instelbare) nazendtijd: hoeveel stilte er nog
// uitgezonden wordt (PTT-uitgang blijft actief, geen toon) nadat de ID
// volledig is afgespeeld, voordat de PTT-uitgang echt loslaat - alleen als
// de ID niet halverwege is afgebroken via de nieuwe "direct afbreken"-optie
// hieronder (zie cgUpdate()/CG_TAIL).
static const unsigned long CG_TAIL_SILENCE_MS = 500;

Preferences prefs;
WebServer   server(80);
bool        apMode = false;

// ================= Globale variabelen: HM-10 =================
HardwareSerial HM10(2);   // UART2 van de ESP32
String        rxBuffer;
bool          waitingForResponse = false;
unsigned long nextPollAt    = 0;
unsigned long requestSentAt = 0;

int  candidateState = -1;     // laatst ontvangen ruwe PIO3-status (0/1)
int  candidateCount = 0;

// Diagnose: telt hoe vaak een AT+PIO3?-aanvraag zonder (tijdig) antwoord
// bleef -> als dit tijdens het testen vaak oploopt, is dat een sterke
// aanwijzing dat gemiste PTT-drukken door communicatieproblemen met de
// HM-10 komen (i.p.v. door de pollsnelheid van de ESP32 zelf).
unsigned long pollTimeoutCount = 0;

bool linkUp    = false;  // BLE-link ESP-HM10 <-> remote-HM10 (OK+CONN / OK+LOST)
bool remotePtt = false;  // PTT-status zoals gemeld door de remote (na debounce)

bool          linkLostBlinkState  = false;
unsigned long lastLinkBlinkToggle = 0;

// ================= Globale variabelen: web-PTT en gecombineerde uitgang =================
bool          webPtt         = false;  // handmatige PTT vanaf de webpagina
unsigned long lastWebPttPing = 0;
bool          outputActive   = false;  // laatst toegepaste (gecombineerde) uitgangsstatus

// ================= Globale variabelen: Callgever (CW-ID) =================
struct CgConfig {
  char callsign[16];
  bool enabled;
  uint8_t volume;    // 0-100 %
  uint8_t wpm;       // 5-30
  uint16_t freqHz;   // 400-1500
  uint16_t delayMs;  // 0-25000
  uint8_t attackMs;  // 1-20, oprijtijd van de toon-envelop
  uint8_t releaseMs; // 10-100, uitrijtijd van de toon-envelop
  bool abortOnRelease; // true = PTT loslaten breekt een lopende ID direct af
                        // (i.p.v. hem, zoals standaard, altijd af te maken)
};
CgConfig cg;
MorsePlayer morse;

// CG_TAIL: de ID is klaar met afspelen, maar de PTT-uitgang blijft nog
// CG_TAIL_SILENCE_MS actief (stilte, geen toon) voordat hij loslaat - alleen
// bereikt als de ID niet halverwege afgebroken is via cg.abortOnRelease.
enum CgState { CG_IDLE, CG_ARMED, CG_TRANSMITTING, CG_TAIL, CG_WAIT_RELEASE };
CgState       cgState      = CG_IDLE;
unsigned long cgArmedAtMs  = 0;
unsigned long cgTailAtMs   = 0;
bool          cgPttPrev    = false; // vorige (remotePtt||webPtt), voor flank-detectie

// Verplichte periodieke herhaling van de ID zolang de PTT ononderbroken
// ingedrukt blijft: de eerste herhaling na 4:59 (299000 ms) sinds het moment
// dat de PTT inging (cgArmedAtMs - "vanaf begin", dus inclusief de tijd van
// de eerste/normale ID), en daarna elke 5:00 (300000 ms) opnieuw, zolang de
// PTT onafgebroken ingedrukt blijft. Laat je de PTT los, dan stopt de cyclus
// en telt hij bij een volgende druk weer van voren af aan (cgRepeatCount
// wordt dan mee gereset via cgArmedAtMs hieronder).
static const unsigned long CG_REPEAT_FIRST_MS = 299000UL;    // 4 min 59 sec
static const unsigned long CG_REPEAT_INTERVAL_MS = 300000UL; // 5 min
int cgRepeatCount = 0; // aantal periodieke herhalingen sinds cgArmedAtMs

// Handmatige toon (ingedrukt-houden-knop op /callgever) om frequentie/volume
// op het gehoor te controleren zonder de hele call te moeten afwachten.
// Houdt, net als webPtt, een keepalive bij met fail-safe timeout.
bool          manualToneOn         = false;
unsigned long lastManualTonePingMs = 0;
static const unsigned long MANUAL_TONE_TIMEOUT_MS = 1200;

// ================================================================
//                          HTML-pagina's
// ================================================================
const char AP_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="nl"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WiFi instellen</title>
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
<h1>WiFi instellen</h1>
<p class="hint">Vul de gegevens van je eigen netwerk in. De ESP32 herstart daarna en probeert te verbinden.</p>
<form method="POST" action="/save">
  <label>Netwerknaam (SSID)</label>
  <input name="ssid" required maxlength="32" autocomplete="off">
  <label>Wachtwoord</label>
  <input name="pass" type="password" maxlength="64" autocomplete="off">
  <button type="submit">Opslaan &amp; herstart</button>
</form>
</body></html>
)rawliteral";

const char MAIN_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="nl"><head><meta charset="utf-8">
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
<div class="status-row"><span class="lamp" id="lamp"></span><span id="linkText">remote verbonden</span></div>
<div class="status-row last"><span class="lamp" id="cglamp"></span><span id="cgText">CW-ID stand-by</span></div>
<button class="ptt-btn" id="pttBtn" aria-label="PTT">
  <svg viewBox="0 0 24 24"><path d="M12 14a3 3 0 0 0 3-3V6a3 3 0 0 0-6 0v5a3 3 0 0 0 3 3zm5-3a5 5 0 0 1-10 0H5a7 7 0 0 0 6 6.92V21h2v-3.08A7 7 0 0 0 19 11h-2z"/></svg>
</button>
<div style="margin-top:5rem;text-align:center">
  <a class="setup-link" href="/callgever">Callgever (CW-ID) instellingen &rarr;</a>
  <a class="setup-link" href="/setup">MQTT instellingen &rarr;</a>
</div>
<div class="uptime" id="uptime"></div>
<script>
const lamp     = document.getElementById('lamp');
const linkText = document.getElementById('linkText');
const cglamp   = document.getElementById('cglamp');
const cgText   = document.getElementById('cgText');
const btn      = document.getElementById('pttBtn');
const uptimeEl = document.getElementById('uptime');
let keepAlive = null;
let lastUptime = null;
let lastLocalAction = 0;   // tijdstip van de laatste eigen druk/loslaat-actie
function fmtUptime(s){
  const m = Math.floor(s/60), sec = s%60;
  return m > 0 ? (m + 'm ' + sec + 's') : (sec + 's');
}
async function poll(){
  try{
    const r = await fetch('/api/status', {cache:'no-store'});
    const j = await r.json();
    lamp.classList.toggle('on', j.link);
    linkText.textContent = j.link ? 'remote verbonden' : 'remote niet verbonden';
    cglamp.classList.toggle('cg-on', j.cg);
    cgText.textContent = j.cg ? 'CW-ID wordt uitgezonden...' : 'CW-ID stand-by';
    // Vlak na een eigen druk/loslaat niet de knopkleur laten "terugklappen"
    // door een poll-antwoord dat nog van vóór die actie dateert.
    if(Date.now() - lastLocalAction > 350){
      btn.classList.toggle('active', j.ptt);
    }
    if(lastUptime !== null && j.uptime < lastUptime){
      uptimeEl.textContent = 'ESP32 is herstart! actief: ' + fmtUptime(j.uptime);
    } else {
      uptimeEl.textContent = 'actief: ' + fmtUptime(j.uptime);
    }
    lastUptime = j.uptime;
  }catch(e){
    lamp.classList.remove('on');
    linkText.textContent = 'geen verbinding met ESP32';
    cglamp.classList.remove('cg-on');
    cgText.textContent = '';
  }
}
setInterval(poll, 300);
poll();
function pttOn(){
  lastLocalAction = Date.now();
  btn.classList.add('active');           // meteen visuele feedback, niet wachten op de poll
  fetch('/api/ptt?on=1', {cache:'no-store'});
  if(!keepAlive) keepAlive = setInterval(() => { lastLocalAction = Date.now(); fetch('/api/ptt?on=1', {cache:'no-store'}); }, 400);
}
function pttOff(){
  if(keepAlive){ clearInterval(keepAlive); keepAlive = null; }
  lastLocalAction = Date.now();
  btn.classList.remove('active');        // meteen visuele feedback
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
  setLinkPinsLost();   // linkUp start op false, dus meteen laten knipperen

  HM10.begin(HM10_BAUD, SERIAL_8N1, HM10_RX_PIN, HM10_TX_PIN);
  delay(100);
  Serial.println(F("HM-10 UART actief op 9600 baud (RX=16, TX=17)"));

  // Zorg dat de HM-10 automatisch OK+CONN/OK+LOST meldt bij verbinden/
  // verbreken (AT+NOTI0 zou deze meldingen onderdrukken).
  delay(50);
  HM10.print("AT+NOTI1");
  delay(150);
  {
    String notiResp;
    while (HM10.available()) notiResp += (char)HM10.read();
    if (notiResp.length() > 0) {
      Serial.print(F("HM-10 antwoord op AT+NOTI1: \""));
      Serial.print(notiResp);
      Serial.println(F("\""));
    } else {
      Serial.println(F("Geen antwoord op AT+NOTI1 (kan normaal zijn voor deze module, of hij ondersteunt dit commando niet)"));
    }
  }

  rxBuffer.reserve(64);

  prefs.begin("cfg", false);

  cgLoadConfig();

  String ssid = prefs.getString("wifi_ssid", "");
  String pass = prefs.getString("wifi_pass", "");

  bool connected = false;
  if (ssid.length() > 0) {
    Serial.print(F("Verbinden met WiFi: "));
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

  // BELANGRIJK: audioInit() (en dus de 40kHz DAC-sample-interrupt) pas
  // starten NA het WiFi-verbindingsproces en mDNS.begin(), niet ervoor.
  // WiFi.begin() (en zeker de eerste succesvolle verbinding ooit na een
  // verse flash) kan intern flash-schrijfacties doen (bv. PHY-kalibratiedata
  // wegschrijven), waarbij de instructiecache even wordt uitgeschakeld.
  // Draait onze ISR dan al en roept hij dacWrite() aan (een gewone,
  // niet-IRAM functie), dan kan dat een Guru Meditation crash
  // (LoadProhibited) veroorzaken - dit is precies wat er gebeurde voordat
  // deze regels hierheen verplaatst werden. Runtime NVS-writes van onszelf
  // blijven apart beschermd via audioPause()/audioResume() (zie
  // cgSaveConfig()/handleSetupSave()).
  audioInit();
  morse.begin(audioToneOn, audioToneOff);
  cgApplyAudioSettings();
}

// ================================================================
//                             loop()
// ================================================================
void loop() {
  unsigned long now = millis();

  // ---- HM-10-afhandeling (PTT-detectie) staat bewust HELEMAAL VOORAAN,
  // vóór server.handleClient(). Dat laatste kan bij een verbonden
  // webclient (bv. de PTT-pagina die elke 300ms /api/status opvraagt, of
  // iemand die /callgever laadt) een paar ms blokkeren op een TCP-send -
  // tijd die dan niet meer aan het uitlezen/verwerken van de HM-10-UART
  // besteed wordt. Door dit blok als eerste te doen, wordt elke binnenkomende
  // byte van de HM-10 zo snel mogelijk na aankomst verwerkt, ongeacht wat de
  // webserver op dat moment aan het doen is. ----

  // ---- 1) Verzamel alle binnenkomende bytes van de HM-10 ----
  while (HM10.available()) {
    char c = (char)HM10.read();
    rxBuffer += c;
    if (rxBuffer.length() > 60) {
      rxBuffer.remove(0, rxBuffer.length() - 60);
    }
  }

  // ---- 2) Asynchrone verbindingsmeldingen hebben altijd voorrang ----
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

  // ---- 3) Timeout: ongeldige/onvolledige berichten worden genegeerd ----
  if (waitingForResponse && (now - requestSentAt > RESPONSE_TIMEOUT_MS)) {
    waitingForResponse = false;
    rxBuffer = "";
    nextPollAt = now;
    pollTimeoutCount++; // stil bijgehouden; zie evt. debug-uitbreiding om dit weer te loggen
  }

  // ---- 4) Volgende AT+PIO3? zodra de minimale pauze voorbij is ----
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

  // ---- Webserver: mag na de tijdkritische HM-10-afhandeling hierboven ----
  server.handleClient();

  // D19 laten knipperen (1s aan/uit) zolang de link met de remote weg is
  if (!linkUp && (now - lastLinkBlinkToggle >= LINK_LOST_BLINK_MS)) {
    lastLinkBlinkToggle = now;
    linkLostBlinkState = !linkLostBlinkState;
    digitalWrite(LINK_LOST_PIN, linkLostBlinkState ? HIGH : LOW);
  }

  // Fail-safe: geen keepalive meer ontvangen voor de web-PTT knop
  if (webPtt && (now - lastWebPttPing > WEB_PTT_TIMEOUT_MS)) {
    webPtt = false;
    updatePttOutput();
    Serial.println(F(">>> Web-PTT fail-safe: geen keepalive ontvangen, uitgeschakeld"));
  }

  // Fail-safe: geen keepalive meer ontvangen voor de handmatige toon-knop
  if (manualToneOn && (now - lastManualTonePingMs > MANUAL_TONE_TIMEOUT_MS)) {
    manualToneOn = false;
    audioToneOff();
    updatePttOutput();
  }

  // ---- 5) Callgever: morse-timing + trigger-statusmachine ----
  morse.update();
  cgUpdate();

  // updatePttOutput() zet de PTT-uitgang alleen echt om als er iets
  // veranderd is (remotePtt, webPtt, of morse.isPlaying()) - veilig om elke
  // loop-iteratie aan te roepen.
  updatePttOutput();

  // ID_ACTIVE_PIN "tekent" de morse-code van de ID daadwerkelijk uit: hoog
  // tijdens elke dot/dash, laag in de pauzes ertussen (isKeyed()) - i.p.v.
  // gewoon continu hoog te blijven zolang de ID uitgezonden wordt
  // (isPlaying()). Geldt zowel voor een ID via PTT-trigger als via de
  // testknop, maar NIET tijdens de handmatige (rauwe) testtoon - in
  // tegenstelling tot STATUS_LED_PIN, dat de bredere PTT-uitgangsstatus
  // toont.
  if (ID_ACTIVE_PIN >= 0) {
    digitalWrite(ID_ACTIVE_PIN, morse.isKeyed() ? HIGH : LOW);
  }
}

// ================================================================
//                  WiFi: AP-modus / station-modus
// ================================================================
void startApMode() {
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);
  Serial.println(F("Geen (werkende) WiFi-gegevens - AP-modus gestart."));
  Serial.print(F("Verbind met netwerk '"));
  Serial.print(AP_SSID);
  Serial.println(F("' en ga naar http://192.168.4.1/"));

  server.on("/", HTTP_GET, handleApRoot);
  server.on("/save", HTTP_POST, handleApSave);
  server.begin();
}

void startStationMode() {
  apMode = false;
  Serial.print(F("Verbonden! IP-adres: "));
  Serial.println(WiFi.localIP());
  if (MDNS.begin(MDNS_HOSTNAME)) {
    Serial.print(F("Ook bereikbaar via http://"));
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
//                       Web-server handlers
// ================================================================
void handleApRoot() {
  server.send_P(200, "text/html", AP_PAGE);
}

void handleApSave() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  ssid.trim();
  if (ssid.length() == 0) {
    server.send(400, "text/plain", "SSID mag niet leeg zijn");
    return;
  }
  audioPause();
  prefs.putString("wifi_ssid", ssid);
  prefs.putString("wifi_pass", pass);
  audioResume();

  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center;padding-top:3rem}</style>"
                "</head><body><h2>Opgeslagen!</h2><p>ESP32 herstart nu en probeert te verbinden met <b>" + htmlEscape(ssid) + "</b>...</p></body></html>";
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
  html += F("<!DOCTYPE html><html lang='nl'><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>MQTT instellingen</title><style>"
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
  html += F("<h1>MQTT instellingen</h1>");
  if (saved) html += F("<p class='msg'>Opgeslagen!</p>");
  html += F("<form method='POST' action='/setup/save'>");
  html += F("<label>Broker (host of IP)</label><input name='host' value='"); html += htmlEscape(host); html += F("' required>");
  html += F("<label>Poort</label><input name='port' inputmode='numeric' value='"); html += htmlEscape(port); html += F("'>");
  html += F("<label>Gebruikersnaam</label><input name='user' value='"); html += htmlEscape(user); html += F("'>");
  html += F("<label>Wachtwoord</label><input name='pass' type='password' value='"); html += htmlEscape(pass); html += F("'>");
  html += F("<button class='primary' type='submit'>Opslaan</button>");
  html += F("</form>");
  html += F("<a href='/'>&larr; Terug naar PTT</a>");
  html += F("<div class='danger'><form method='POST' action='/api/wifi/reset' "
            "onsubmit=\"return confirm('WiFi-instellingen wissen en ESP32 herstarten in setup-modus?')\">"
            "<button type='submit'>WiFi opnieuw instellen</button></form></div>");
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
  // audioPause() zet de envelop hard terug naar stil; als de handmatige
  // toon-knop op /callgever nog ingedrukt werd gehouden, moet de toon
  // opnieuw gestart worden.
  if (manualToneOn) audioToneOn();
  server.sendHeader("Location", "/setup?saved=1");
  server.send(303);
}

void handleApiStatus() {
  server.sendHeader("Cache-Control", "no-store");
  // Vaste buffer i.p.v. String-concatenatie: dit endpoint wordt door de
  // pagina elke 300ms opgevraagd. "cg" geeft aan of de callgever op dit
  // moment zijn morse-ID aan het uitzenden is.
  char buf[80];
  snprintf(buf, sizeof(buf), "{\"link\":%s,\"ptt\":%s,\"cg\":%s,\"uptime\":%lu}",
           linkUp ? "true" : "false",
           outputActive ? "true" : "false",
           morse.isPlaying() ? "true" : "false",
           millis() / 1000UL);
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
                "</head><body><h2>WiFi-instellingen gewist</h2><p>ESP32 herstart nu in setup-modus...</p></body></html>";
  server.send(200, "text/html", html);
  delay(1000);
  ESP.restart();
}

// ---- Callgever webpagina ----
static long cgClamp(long v, long lo, long hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Kleine helper om tekst veilig in een HTML-attribuut te zetten.
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
  html += F("<!DOCTYPE html><html lang='nl'><head><meta charset='utf-8'>"
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
            "<div class='sub'>CW-ID beacon &mdash; getriggerd door PTT (remote of webpagina)</div>");

  html += F("<div class='card status'>");
  html += "<div><span class='dot ";
  html += (playing ? "on" : "off");
  html += "'></span>";
  html += playing ? "Zendt nu de ID uit..." : "Stand-by (wacht op PTT)";
  html += "</div>";
  html += F("</div>");

  html += F("<form method='POST' action='/callgever/save'>");
  html += F("<div class='card'>");

  html += F("<label for='callsign'>Roepnaam (call)</label>");
  html += "<input type='text' id='callsign' name='callsign' maxlength='15' value='" +
          htmlEscape(String(cg.callsign)) + "' required>";

  html += F("<div class='switchrow'>"
            "<label for='enabled' style='margin:0'>Callgever ingeschakeld</label>"
            "<input type='checkbox' id='enabled' name='enabled' style='width:22px;height:22px'");
  if (cg.enabled) html += F(" checked");
  html += F("></div>");

  html += F("<div class='switchrow' style='margin-top:14px'>"
            "<label for='abortrel' style='margin:0'>PTT loslaten breekt ID direct af</label>"
            "<input type='checkbox' id='abortrel' name='abortrel' style='width:22px;height:22px'");
  if (cg.abortOnRelease) html += F(" checked");
  html += F("></div>");
  html += F("<p style='font-size:12px;color:#64748b;margin-top:6px;margin-bottom:0'>Uit (standaard): een "
            "eenmaal gestarte ID wordt altijd afgemaakt, ook als je de PTT eerder loslaat. Aan: de ID stopt "
            "meteen zodra je de PTT loslaat. Wordt de ID gewoon afgemaakt (niet afgebroken), dan zendt de "
            "zender daarna nog 500 ms stilte uit voordat de PTT-uitgang loslaat.</p>");

  html += F("</div>"); // card

  html += F("<div class='card'>");

  html += F("<label for='volume'>Volume</label>");
  html += "<input type='range' id='volume' name='volume' min='0' max='100' step='1' value='" +
          String(cg.volume) +
          "' oninput=\"document.getElementById('volumeval').innerText=this.value+' %'\">";
  html += "<div class='rowval'><span>0%</span><span id='volumeval'>" + String(cg.volume) + " %</span><span>100%</span></div>";

  html += F("<label for='wpm'>Morse-snelheid (WPM)</label>");
  html += "<input type='range' id='wpm' name='wpm' min='5' max='30' step='1' value='" +
          String(cg.wpm) +
          "' oninput=\"document.getElementById('wpmval').innerText=this.value+' wpm'\">";
  html += "<div class='rowval'><span>5 wpm</span><span id='wpmval'>" + String(cg.wpm) + " wpm</span><span>30 wpm</span></div>";

  html += F("<label for='freq'>Toonfrequentie</label>");
  html += "<input type='range' id='freq' name='freq' min='400' max='1500' step='10' value='" +
          String(cg.freqHz) +
          "' oninput=\"document.getElementById('freqval').innerText=this.value+' Hz'\">";
  html += "<div class='rowval'><span>400 Hz</span><span id='freqval'>" + String(cg.freqHz) + " Hz</span><span>1500 Hz</span></div>";

  html += F("<label for='didelay'>Vertraging na start-PTT</label>");
  html += "<input type='range' id='didelay' name='didelay' min='0' max='25000' step='10' value='" +
          String(cg.delayMs) +
          "' oninput=\"document.getElementById('didelayval').innerText=this.value+' ms'\">";
  html += "<div class='rowval'><span>0 ms</span><span id='didelayval'>" + String(cg.delayMs) + " ms</span><span>25000 ms</span></div>";

  html += F("</div>"); // card

  html += F("<div class='card'>");
  html += F("<label for='attackms'>Attack (oprijtijd)</label>");
  html += "<input type='range' id='attackms' name='attackms' min='1' max='20' step='1' value='" +
          String(cg.attackMs) +
          "' oninput=\"document.getElementById('attackmsval').innerText=this.value+' ms'\">";
  html += "<div class='rowval'><span>1 ms</span><span id='attackmsval'>" + String(cg.attackMs) + " ms</span><span>20 ms</span></div>";

  html += F("<label for='releasems'>Release (uitrijtijd)</label>");
  html += "<input type='range' id='releasems' name='releasems' min='10' max='100' step='1' value='" +
          String(cg.releaseMs) +
          "' oninput=\"document.getElementById('releasemsval').innerText=this.value+' ms'\">";
  html += "<div class='rowval'><span>10 ms</span><span id='releasemsval'>" + String(cg.releaseMs) + " ms</span><span>100 ms</span></div>";
  html += F("<p style='font-size:12px;color:#64748b;margin-top:-6px'>Attack/release bepalen hoe vloeiend elke "
            "dot/dash op-/afbouwt (voorkomt een hoorbare \"klik\"). Let op: bij hogere WPM wordt een dot korter "
            "dan de release-tijd kan zijn (bv. bij 20 wpm is 1 dit = 60ms) - de toon sterft dan niet meer helemaal "
            "weg tussen twee elementen. Kies release en snelheid dus in combinatie.</p>");
  html += F("</div>"); // card

  html += F("<button type='submit' class='save'>Opslaan</button>");
  html += F("</form>");

  html += F("<form method='POST' action='/callgever/test'>"
            "<button type='submit' class='test'>Test uitzending nu</button>"
            "</form>");
  html += F("<p style='font-size:12px;color:#64748b'>Let op: de testknop zet ook echt de PTT-uitgang (D21) "
            "aan zolang de ID speelt, dus als de zender aangesloten is gaat hij daadwerkelijk zenden.</p>");

  html += F("<button type='button' class='tone' id='toneBtn'>Toon (ingedrukt houden)</button>"
            "<p style='font-size:12px;color:#64748b'>Speelt de kale toon op het laatst opgeslagen "
            "volume/frequentie zolang je de knop ingedrukt houdt (bv. om af te stellen) - zet, net als "
            "de testknop, ook echt de PTT-uitgang aan. Werkt niet tijdens een lopende ID-uitzending.</p>");

  html += F("<a href='/'>&larr; Terug naar PTT</a>");

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

  cg.enabled = server.hasArg("enabled"); // checkbox: alleen aanwezig als aangevinkt
  cg.abortOnRelease = server.hasArg("abortrel"); // checkbox: alleen aanwezig als aangevinkt

  if (server.hasArg("volume"))
    cg.volume = (uint8_t)cgClamp(server.arg("volume").toInt(), 0, 100);
  if (server.hasArg("wpm"))
    cg.wpm = (uint8_t)cgClamp(server.arg("wpm").toInt(), 5, 30);
  if (server.hasArg("freq"))
    cg.freqHz = (uint16_t)cgClamp(server.arg("freq").toInt(), 400, 1500);
  if (server.hasArg("didelay"))
    cg.delayMs = (uint16_t)cgClamp(server.arg("didelay").toInt(), 0, 25000);
  if (server.hasArg("attackms"))
    cg.attackMs = (uint8_t)cgClamp(server.arg("attackms").toInt(), 1, 20);
  if (server.hasArg("releasems"))
    cg.releaseMs = (uint8_t)cgClamp(server.arg("releasems").toInt(), 10, 100);

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

// Handmatige toon aan/uit (ingedrukt-houden-knop op /callgever). Zet, net
// als de testknop, ook echt de PTT-uitgang (D21) actief zolang de toon
// klinkt - zie updatePttOutput().
void handleCallgeverTone() {
  server.sendHeader("Cache-Control", "no-store");

  if (server.hasArg("on") && server.arg("on") == "1") {
    if (morse.isPlaying()) {
      // Er loopt al een automatische ID-uitzending - handmatige toon negeren.
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
//         HM-10: debounce, verbindingsmeldingen, uitgang
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
      Serial.println(remotePtt ? F("INGEDRUKT") : F("LOSGELATEN"));
      updatePttOutput();
    }
  }
}

void handleLinkLost() {
  if (linkUp) {
    Serial.println(F(">>> Remote LINK VERLOREN (OK+LOST)"));
  }
  linkUp = false;
  candidateState = -1;
  candidateCount = 0;
  setLinkPinsLost();
  if (remotePtt) {
    remotePtt = false;
    Serial.println(F(">>> Remote-PTT losgelaten (fail-safe, geen verbinding met remote)"));
  }
  updatePttOutput();
}

void handleLinkUp() {
  if (!linkUp) {
    Serial.println(F(">>> Remote verbonden (OK+CONN)"));
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

// Combineert de drie onafhankelijke bronnen (remote via BLE, de webpagina,
// en de callgever die zelf zendt) tot één PTT-uitgangsstatus.
void updatePttOutput() {
  // cgState == CG_TAIL: de ID is klaar met afspelen (morse.isPlaying() is dus
  // al false), maar we willen de PTT-uitgang nog even (CG_TAIL_SILENCE_MS)
  // aan houden voor de nazendstilte - vandaar hier expliciet meegenomen.
  bool want = remotePtt || webPtt || morse.isPlaying() || manualToneOn || (cgState == CG_TAIL);
  if (want != outputActive) {
    outputActive = want;
    applyPttOutput(outputActive);
    Serial.print(F(">>> PTT-uitgang "));
    Serial.println(outputActive ? F("AAN") : F("UIT"));
  }
}

void applyPttOutput(bool active) {
  bool level = PTT_ACTIVE_HIGH ? active : !active;
  digitalWrite(PTT_OUTPUT_PIN, level ? HIGH : LOW);
  digitalWrite(STATUS_LED_PIN, active ? HIGH : LOW);
}

// ================================================================
//         Callgever: instellingen laden/opslaan + trigger-logica
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
  audioPause(); // voorkomt dat de sample-interrupt tijdens de flash-schrijfactie afgaat
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
  // audioPause() zet de envelop hard terug naar stil; als de handmatige
  // toon-knop nog ingedrukt werd gehouden tijdens het opslaan, moet de toon
  // opnieuw gestart worden.
  if (manualToneOn) audioToneOn();
}

void cgApplyAudioSettings() {
  audioSetFrequency(cg.freqHz);
  audioSetVolume(cg.volume);
  morse.setWpm(cg.wpm);
  audioSetAttackMs(cg.attackMs);
  audioSetReleaseMs(cg.releaseMs);
}

// Niet-blokkerende trigger-statusmachine: gebruikt de gecombineerde
// (remotePtt || webPtt) als "DI-signaal" i.p.v. een fysieke pin.
void cgUpdate() {
  bool pttNow = (remotePtt || webPtt);

  // Let op: de "loslaten -> terug naar IDLE"-overgangen hieronder controleren
  // bewust NIET cg.enabled (in tegenstelling tot de "aangaan -> armen"-
  // overgang, die dat wel doet). Zou je dat wel doen (zoals eerder het geval
  // was, via een aparte if(cg.enabled){...}else{...}-structuur), dan kan het
  // volgende misgaan: callgever wordt uitgeschakeld terwijl cgState nog
  // CG_WAIT_RELEASE is (PTT nog ingedrukt); PTT wordt losgelaten én weer
  // ingedrukt terwijl de callgever nog steeds uit staat - cgState blijft dan
  // "vast" op CG_WAIT_RELEASE met een oude cgArmedAtMs/cgRepeatCount, en de
  // nieuwe druk telt niet als verse flank (cgPttPrev stond al op true).
  // Wordt de callgever daarna weer ingeschakeld terwijl de PTT toevallig nog
  // ingedrukt is, dan hervat hij foutief de oude cyclus i.p.v. gewoon te
  // wachten op een echte nieuwe flank. Door de reset-overgangen altijd te
  // laten lopen (ongeacht cg.enabled), gaat cgState altijd meteen terug naar
  // IDLE zodra de PTT losgelaten wordt, met een schone cgRepeatCount - net
  // als bij de standalone callgever (DI-pin) al het geval was.
  if (pttNow && !cgPttPrev && cgState == CG_IDLE && cg.enabled) {
    // PTT gaat aan -> callgever "armen", vertraging laten lopen. Ook armen
    // als er net een handmatige toon loopt: de flank mag niet verloren
    // gaan (anders wordt de verplichte ID overgeslagen). De
    // "if (manualToneOn) break;" hieronder in de ARMED-case houdt het
    // daadwerkelijk starten tegen tot de toon-knop weer losgelaten is.
    cgState = CG_ARMED;
    cgArmedAtMs = millis();
    cgRepeatCount = 0;
  } else if (!pttNow && cgState == CG_ARMED) {
    // PTT losgelaten voordat de vertraging voorbij was -> annuleren
    cgState = CG_IDLE;
    cgRepeatCount = 0;
  } else if (!pttNow && cg.abortOnRelease && (cgState == CG_TRANSMITTING || cgState == CG_TAIL)) {
    // Nieuwe optie "direct afbreken bij loslaten PTT": in tegenstelling tot
    // het standaardgedrag (een eenmaal gestarte ID altijd afmaken, zie
    // CG_TRANSMITTING hieronder) stopt de uitzending nu METEEN zodra de PTT
    // losgelaten wordt - zowel tijdens de ID zelf als tijdens de
    // nazendstilte (CG_TAIL) erna. Geen nazendstilte in dit geval: dit is
    // een bewuste, onmiddellijke afbreek-actie, geen normale afronding.
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
      if (manualToneOn) break; // handmatige toon heeft voorrang, wachten
      if ((millis() - cgArmedAtMs) >= cg.delayMs) {
        // Als er via de testknop (handleCallgeverTest) toevallig al een
        // uitzending bezig is, wachten we tot die klaar is i.p.v. hem te
        // onderbreken/herstarten.
        if (morse.isPlaying()) break;
        cgApplyAudioSettings();
        morse.play(cg.callsign);
        cgState = CG_TRANSMITTING;
      }
      break;

    case CG_TRANSMITTING:
      if (!cg.enabled) {
        // Callgever tussentijds uitgezet vanaf de webpagina -> direct stil
        morse.stop();
        cgState = CG_IDLE;
        cgRepeatCount = 0;
        break;
      }
      if (!morse.isPlaying()) {
        // ID volledig afgespeeld (was dit een afbreek-actie i.p.v. een
        // normale afronding, dan waren we hierboven al direct naar CG_IDLE
        // gegaan - hier komen we dus alleen aan als de ID gewoon uit mocht
        // zenden). Ga eerst naar CG_TAIL: de PTT-uitgang blijft nog
        // CG_TAIL_SILENCE_MS actief (stilte) voordat hij loslaat.
        cgTailAtMs = millis();
        cgState = CG_TAIL;
      }
      break;

    case CG_TAIL:
      if (!cg.enabled) {
        // Callgever tussentijds uitgezet -> ook de nazendstilte direct stoppen
        cgState = CG_IDLE;
        cgRepeatCount = 0;
        break;
      }
      if ((millis() - cgTailAtMs) >= CG_TAIL_SILENCE_MS) {
        // Nazendstilte voorbij - zelfde vervolg als voorheen direct na de ID:
        // wachten op loslaten (of, bij lange ononderbroken PTT, de
        // verplichte periodieke herhaling hieronder in CG_WAIT_RELEASE).
        cgState = pttNow ? CG_WAIT_RELEASE : CG_IDLE;
        if (cgState == CG_IDLE) cgRepeatCount = 0;
      }
      break;

    case CG_WAIT_RELEASE:
      // PTT staat nog steeds aan (anders waren we hierboven al naar CG_IDLE
      // gegaan). Zolang dat zo blijft, moet de callgever elke
      // CG_REPEAT_INTERVAL_MS opnieuw identificeren, te beginnen
      // CG_REPEAT_FIRST_MS na het moment dat de PTT voor het eerst inging.
      if (!cg.enabled) break;
      if (!pttNow) break;              // zou hierboven al afgevangen zijn
      if (manualToneOn) break;         // handmatige toon heeft voorrang
      if (morse.isPlaying()) break;    // voor de zekerheid, zou niet moeten
      {
        unsigned long threshold = CG_REPEAT_FIRST_MS +
                                   (unsigned long)cgRepeatCount * CG_REPEAT_INTERVAL_MS;
        if ((millis() - cgArmedAtMs) >= threshold) {
          cgApplyAudioSettings();
          morse.play(cg.callsign);
          cgRepeatCount++;
          cgState = CG_TRANSMITTING;
        }
      }
      break;

    default:
      break;
  }
}
