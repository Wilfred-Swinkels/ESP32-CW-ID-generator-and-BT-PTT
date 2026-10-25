# ESP32 + HM-10 Remote PTT + Callgever (CW-ID)

Firmware voor een klassieke ESP32 (WROOM/WROVER) die twee dingen combineert:
een **remote PTT-schakelaar** (bediend via een HM-10 BLE-module, bv. een
knopje op afstand) en een **callgever**: een CW-ID-beacon die automatisch je
roepnaam in morse uitzendt via de ingebouwde DAC zodra je zendt. Bediening en
instellingen (callsign, volume, snelheid, toonfrequentie, timing) lopen via
een ingebouwde webpagina - geen app of externe libraries nodig.

Eén gecombineerde sketch: de bestaande "remote PTT via HM-10" functionaliteit
plus de callgever (morse-ID beacon), die nu getriggerd wordt door diezelfde
PTT-status in plaats van een losse fysieke ingang.

## Bugfix: crash direct na het starten van de toon-interrupt opgelost

De echte oorzaak (bevestigd door de ESP32-DAC-broncode erop na te slaan):
`dacWrite()` initialiseert de DAC-peripheral **lui** - pas bij de allereerste
aanroep voor een pin wordt er intern een "oneshot channel" aangemaakt
(`dac_oneshot_new_channel`), en dat doet een heap-allocatie en pakt interne
locks. Dat is prima in gewone code, maar **niet veilig binnen een
interrupt**. `audioInit()` startte de 40kHz-timer/interrupt echter vóórdat er
ooit ergens anders al een `dacWrite()`-aanroep had plaatsgevonden, waardoor
die allereerste, onveilige initialisatie binnen de interrupt zelf gebeurde -
en dat gaf exact de gerapporteerde "Guru Meditation Error (LoadProhibited)",
vrijwel meteen nadat de timer voor het eerst afging (in jouw log precies na
"Ook bereikbaar via http://esp32-ptt.local/", want dat is waar `audioInit()`
in de opstartvolgorde wordt aangeroepen). Fix: `audioInit()` roept
`dacWrite()` nu eerst één keer aan in gewone code, vóórdat de timer/interrupt
wordt gestart, zodat de DAC-peripheral al klaar staat tegen de tijd dat de
interrupt voor het eerst draait (elke latere `dacWrite()`-aanroep vanuit de
interrupt is daarna alleen nog een lichte, veilige registerschrijf-actie).

(Een eerdere aanpassing verplaatste `audioInit()`/`morse.begin()` ook al naar
ná het WiFi-verbindingsproces en `mDNS.begin()` - dat blijft een verstandige
extra voorzorg tegen een ander, zeldzamer risico [een flash-schrijfactie
tijdens WiFi's eerste verbinding], maar bleek niet de daadwerkelijke oorzaak
van deze crash te zijn: de crash trad nog steeds op, meteen na het starten
van de interrupt, ongeacht wanneer dat in de opstartvolgorde gebeurde - zoals
ook in het nieuwe crashlog te zien was.)

**Tweede, aparte crash - opgelost:** na deze fix startte de ESP32 wél goed op
(zelfs tot en met "PTT-uitgang AAN" in de log), maar crashte alsnog op het
moment dat de allereerste toon echt begon te spelen, weer met dezelfde "Guru
Meditation Error (LoadProhibited)". Oorzaak: de interrupt rekende met
`float`-getallen (het envelop-niveau en de op-/afloop-stappen).
Floating-point-rekenwerk *binnen een interrupt* is op de ESP32 (Xtensa) niet
veilig - de FPU-registers worden alleen lui bewaard/hersteld voor gewone
taken, niet voor interrupt-handlers, dus zodra de interrupt voor het eerst
een `float`-berekening deed (namelijk zodra de envelop voor het eerst van
"stil" af ging, dus bij de eerste PTT/trigger-actie), crashte dat. Zolang er
niets werd uitgezonden, bleef de ISR in een puur integer-pad hangen (vandaar
dat opstarten zelf niet meer crashte, maar de eerste échte toon nog wel).
Fix: alle rekenwerk in de interrupt zelf is omgezet naar gehele getallen
(vast-kommaformaat, schaal 2^15) - alleen de code die de op-/afloop-snelheid
*berekent* (buiten de interrupt, als je de instellingen opslaat) gebruikt nog
`float`, wat prima is zolang het maar niet in de interrupt zelf gebeurt.

## Wat is er veranderd t.o.v. de twee losse projecten

- De callgever heeft **geen eigen DI-pin (GPIO27) meer**. Zijn trigger is nu
  de gecombineerde PTT-status `remotePtt || webPtt` (dus de HM-10-remote-knop
  ÓF de PTT-knop op de webpagina).
- Zodra die PTT-status van laag naar hoog gaat en de callgever aan staat,
  wordt na de ingestelde vertraging (0-25000 ms) de call in morse uitgezonden
  op de DAC (GPIO25), **terwijl de PTT-uitgang (D21) actief blijft** zolang de
  ID speelt - ook als de operator/remote intussen al losgelaten heeft. Zo
  blijft de zender aan tot de ID écht klaar is.
- Laat je de PTT los vóórdat de vertraging voorbij is: de ID wordt
  geannuleerd (net als voorheen).
- Zet je de callgever tussentijds uit via de webpagina terwijl hij aan het
  zenden is: hij stopt direct.
- Blijft de PTT na afloop van de ID nog ingedrukt: er wordt pas een nieuwe ID
  geaccepteerd nadat de PTT ook echt losgelaten is.
- Nieuwe pagina **`/callgever`** (link vanaf de hoofdpagina) voor callsign,
  aan/uit, volume, WPM en toonfrequentie - inclusief een testknop. Instellingen
  worden net als WiFi/MQTT bewaard in NVS-flash (zelfde "cfg"-namespace,
  nieuwe "cg_*"-keys).
- De hoofdpagina toont nu een tweede statusregel ("CW-ID stand-by" /
  "CW-ID wordt uitgezonden...").
- Alle instellingen die tijdens het draaien naar flash geschreven worden
  (WiFi-opslaan, MQTT-opslaan, WiFi-reset, callgever-opslaan) pauzeren nu
  eerst de DAC-sample-interrupt (`audioPause()`/`audioResume()`) - dat
  voorkomt een zeldzame crash doordat de audio-ISR tijdens een
  flash-schrijfactie de instructiecache nodig heeft.
- De morse-toon wordt niet meer abrupt aan/uit geschakeld: elke dot/dash
  heeft nu een **instelbare** attack/release-envelop, zodat er geen hoorbare
  "klik" (en bijbehorende bandbreedte-spatten) bij het aan/uit schakelen
  ontstaat. **Attack** (oprijtijd, 1-20 ms) en **release** (uitrijtijd,
  10-100 ms) staan als sliders op `/callgever`, met 1ms resp. 10ms als
  standaardwaarde. Let op: bij hogere WPM wordt een dot korter dan de
  release-tijd kan zijn (bv. bij 20 wpm is 1 dit maar 60ms) - de toon sterft
  dan niet meer helemaal weg tussen twee elementen; kies release en
  snelheid dus in combinatie.
- Op `/callgever` staat nu ook een "Toon (ingedrukt houden)"-knop om de kale
  toon te testen zonder de hele call af te wachten.
- Nieuwe uitgang **GPIO4 (`ID_ACTIVE_PIN`)**: tekent de morse-code van de ID
  daadwerkelijk uit - hoog tijdens elke dot/dash, laag in de pauzes ertussen
  (dus geen constante hoog-periode voor de hele ID, maar een echt
  knipperend morse-signaal, alsof je met een sleutel meetikt). Niet actief
  zolang de PTT-uitgang om een andere reden actief is, en ook niet tijdens
  de handmatige testtoon - in tegenstelling tot GPIO2, dat de bredere
  PTT-uitgangsstatus toont.
- Morse-snelheid blijft **5-30 WPM**.

## Latere aanpassingen

- **PTT-detectie versneld:** de pollingcyclus naar de HM-10 (`AT+PIO3?`) is
  verkrapt (pollgap 20 -> 5 ms, response-timeout 150 -> 50 ms), en de
  HM-10-afhandeling in `loop()` krijgt nu altijd voorrang op
  `server.handleClient()` (een trage webclient kan de PTT-detectie dus niet
  meer vertragen). Blijft inherent een pollingsysteem: een druk die korter
  duurt dan de effectieve pollcyclus (nu enkele ms) kan in theorie nog steeds
  tussen twee metingen door vallen.
- **Nieuwe optie "PTT loslaten breekt ID direct af"** op `/callgever` (uit,
  standaard). Staat hij aan, dan stopt een lopende ID-uitzending meteen zodra
  je de PTT loslaat, i.p.v. hem altijd af te maken.
- **Nazendstilte:** wordt een ID gewoon afgemaakt (niet afgebroken via
  bovenstaande optie), dan blijft de PTT-uitgang daarna nog 500 ms actief
  zonder toon (stilte) voordat hij loslaat - vaste waarde (`CG_TAIL_SILENCE_MS`
  in de code), niet instelbaar via de webpagina.

## Pinnen (ongewijzigd + toegevoegd)

| Pin | Functie |
|---|---|
| GPIO16 / GPIO17 | HM-10 UART2 (RX/TX) |
| GPIO21 | PTT-uitgang -> transistor -> relais -> PTT-lijn zender |
| GPIO18 / GPIO19 | BLE-linkstatus LED's (verbonden / link weg, knipperend) |
| GPIO2 | status-LED, aan zolang de PTT-uitgang actief is (dus ook tijdens het uitzenden van de ID) |
| **GPIO25** | analoge morse-toon (ingebouwde DAC1) |
| **GPIO4** | **nieuw:** tekent de morse-code van de ID uit (hoog per dot/dash, niet de bredere PTT-status) |
| ~~GPIO27~~ | vervallen (was de oude DI-pin van de losse callgever, wordt nu niet meer gebruikt) |

**Let op:** de ingebouwde DAC bestaat alleen op de klassieke ESP32
(WROOM/WROVER) - niet op ESP32-S2/S3/C3/C6.

### Audio-uitgang naar de zender

GPIO25 staat in rust op ongeveer **1.65V** (midden van de DAC-range), niet op
0V. Zet er een **serie-condensator** (DC-blokkering, bv. 1-10 µF) en
eventueel een **spanningsdeler** (bv. 10 kΩ/1 kΩ) achter om het signaal
(tot 3.3Vpp) naar mic-niveau te brengen. Omdat de PTT-uitgang tijdens het
zenden van de ID toch al actief is, kun je dit DAC-signaal met een simpele
weerstand bijmengen op dezelfde mic-lijn als je normale audio, of - netter -
met een klein relais/analoge schakelaar de mic-ingang tijdens het zenden van
de ID omschakelen naar de DAC, zodat je stem en de morse-toon elkaar niet
overlappen.

## Installatie (Arduino IDE)

1. Kopieer de map `esp32_callgever_ptt` (met alle bestanden) naar je
   Arduino-sketchmap zodat het pad `Arduino/esp32_callgever_ptt/esp32_callgever_ptt.ino`
   wordt (mapnaam = bestandsnaam, verplicht voor de Arduino IDE).
2. Board: "ESP32 Dev Module" (of je specifieke klassieke ESP32-bord) via het
   esp32-bordpakket van Espressif.
3. Geen externe libraries nodig - alleen de standaard ESP32-Arduino-core
   (WiFi, WebServer, Preferences, ESPmDNS, HardwareSerial) plus de
   meegeleverde `morse.h/.cpp` en `audio_dac.h/.cpp`.
4. Upload, open de Seriële Monitor (115200 baud).
5. Er zijn nog geen WiFi-gegevens opgeslagen -> de ESP32 start het access
   point **"ESP32-PTT-Setup"**. Verbind daarmee, ga naar `http://192.168.4.1/`
   en vul je eigen netwerk in. De ESP32 herstart en verbindt.
6. Daarna bereikbaar via `http://esp32-ptt.local/` (mDNS) of het IP-adres uit
   de Seriële Monitor. Hoofdpagina = PTT-knop, met links naar
   **Callgever (CW-ID) instellingen** en **MQTT instellingen**.

## Gedrag samengevat

- **Callgever ingeschakeld** staat aan/uit op `/callgever`. Staat hij uit, dan
  doet PTT niets met de callgever (normale PTT-werking blijft gewoon actief).
- De trigger is **flank-gebaseerd** (laag->hoog): een PTT-signaal dat al bij
  het opstarten "aan" staat, triggert dus niet vanzelf.
- De testknop op `/callgever` zet ook echt de PTT-uitgang aan zolang de test-ID
  speelt - dus als de zender is aangesloten, gaat hij daadwerkelijk zenden.
- **Verplichte herhaling bij lang ononderbroken PTT:** blijft de PTT
  onafgebroken ingedrukt (dus zonder ooit los te laten) langer dan **4
  minuten 59 seconden** vanaf het moment dat hij voor het eerst inging, dan
  zendt de callgever automatisch opnieuw de call uit - ook zonder nieuwe
  flank. Daarna herhaalt dit zich elke **5 minuten**, zolang de PTT
  ononderbroken ingedrukt blijft. Laat je de PTT een keer los, dan stopt deze
  cyclus direct en telt hij bij een volgende druk weer vanaf nul. Is een
  herhaal-ID nog bezig op het moment dat de PTT loslaat, dan wordt die ID -
  net als de eerste - altijd afgemaakt.

## Nog niet gedaan / mogelijke vervolgstappen

- De MQTT-instellingen (`/setup`) worden al opgeslagen, maar er wordt nog
  geen daadwerkelijke MQTT-verbinding opgezet - dat was in het aangeleverde
  bestand al aangegeven als "volgende stap" en is in deze samenvoeging niet
  aangepakt.
- Geen authenticatie op de webpagina's; ga ervan uit dat iedereen die met het
  netwerk (of de AP) verbonden is, instellingen kan wijzigen en de zender kan
  aansturen.
- Zorgvuldig nagelezen en met een onafhankelijke code-review gecontroleerd
  (o.a. een ontbrekende HTML-escape en een randgeval waarbij de testknop en
  een PTT-trigger elkaar konden overlappen zijn hierbij gevonden en
  gefixt), maar **niet fysiek getest op echte hardware** (geen
  ESP32/HM-10-toolchain beschikbaar in de omgeving waarin dit gemaakt is).
  Meld het gerust terug als je bij het compileren of testen iets tegenkomt.

## Licentie

Zie [LICENSE](LICENSE) (MIT) - vrij te gebruiken, aan te passen en te delen.
