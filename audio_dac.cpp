#include "audio_dac.h"
#include <math.h>

#define DAC_PIN 25 // GPIO25 = DAC kanaal 1 op de klassieke ESP32

static const uint32_t SAMPLE_RATE_HZ = 40000; // 40 kHz samplerate voor de sinusgenerator
static const uint32_t TIMER_TICK_HZ = 1000000; // 1 MHz tikfrequentie voor de hw-timer

// Attack-/release-tijd van de envelop, in milliseconden - instelbaar via
// audioSetAttackMs()/audioSetReleaseMs() (en dus via de webpagina), geklemd
// op de hieronder genoemde bereiken. Let op: bij hogere WPM-snelheden wordt
// een dot vrij kort (bv. bij 30 wpm is 1 dit = 40ms) - een release die daar
// (ruim) overheen gaat, laat de toon niet meer volledig wegsterven voordat
// het volgende element begint. Dat is geen fout (de envelop-statusmachine
// hieronder loopt gewoon door en glijdt vloeiend verder), maar wel iets om
// rekening mee te houden bij het kiezen van WPM + release samen.
static const uint16_t ENV_ATTACK_MIN_MS = 1;
static const uint16_t ENV_ATTACK_MAX_MS = 20;
static const uint16_t ENV_RELEASE_MIN_MS = 10;
static const uint16_t ENV_RELEASE_MAX_MS = 100;
static volatile float s_attackMs = 1.0f;    // huidige attack-tijd (default: 1ms, ondergrens van 1-20ms)
static volatile float s_releaseMs = 10.0f;  // huidige release-tijd (default: 10ms, ondergrens van 10-100ms)

static hw_timer_t *audioTimer = nullptr;

// 256-punts sinus-lookuptabel, waarden -127..+127 (volle schaal)
static int8_t sineLUT[256];

static volatile uint32_t s_phaseAcc = 0;       // 32-bit fase-accumulator (DDS)
static volatile uint32_t s_phaseIncrement = 0; // afgeleid van de gewenste frequentie
static volatile uint8_t s_volume = 70; // 0-100 %

// Envelop-statusmachine: ATTACK (oplopend), SUSTAIN (vol volume, vast),
// RELEASE (aflopend), OFF (stil). Geen apart "decay"-niveau nodig omdat de
// sustain hier altijd het volle volume is (gewoon een aan/uit-geschakelde
// toon, geen instrument met een aparte decay-fase).
//
// BELANGRIJK: het envelop-niveau en de stapgroottes zijn met opzet
// vast-kommagetallen (int32_t), GEEN floats. `onAudioTimerISR()` hieronder
// draait als interrupt, en floating-point-rekenwerk binnen een interrupt is
// op de ESP32 (Xtensa) niet veilig: de FPU-context wordt alleen lui
// bewaard/hersteld voor gewone taken (via de coprocessor-exception-afhandeling
// die een taakcontext verwacht), niet voor interrupt-handlers. De eerste keer
// dat de ISR echt floating-point-instructies uitvoerde (zodra de envelop voor
// het eerst van ENV_OFF af ging, dus bij de allereerste toon-trigger) gaf dit
// exact de gerapporteerde Guru Meditation crash (LoadProhibited) - pas ná de
// eerdere dacWrite()-volgordefix, dus een tweede, aparte oorzaak. Alle
// rekenwerk in de ISR gebeurt daarom nu met gehele getallen; alleen de
// (niet-ISR) `recomputeEnvelopeSteps()` hieronder mag nog met floats rekenen
// om de stapgroottes precies om te zetten naar dit vast-kommaformaat.
enum EnvState { ENV_OFF, ENV_ATTACK, ENV_SUSTAIN, ENV_RELEASE };
static const int32_t ENV_SCALE = 32768; // envelop-niveau 0..ENV_SCALE staat voor 0.0-1.0
static volatile EnvState s_envState = ENV_OFF;
static volatile int32_t s_envLevel = 0;   // huidig envelop-niveau, 0..ENV_SCALE
static volatile int32_t s_attackStep = 0; // ophoging per sample tijdens attack
static volatile int32_t s_releaseStep = 0; // afname per sample tijdens release

static void buildSineLUT() {
  for (int i = 0; i < 256; i++) {
    sineLUT[i] = (int8_t)lroundf(127.0f * sinf(2.0f * PI * (float)i / 256.0f));
  }
}

// Herberekent de per-sample envelop-stapgrootte (in vast-kommaeenheden van
// ENV_SCALE) vanuit de huidige s_attackMs/s_releaseMs. Wordt aangeroepen
// vanuit audioInit() en telkens als audioSetAttackMs()/audioSetReleaseMs()
// een nieuwe waarde krijgt - draait altijd in gewone (niet-ISR) context, dus
// floats gebruiken hier is geen probleem.
static void recomputeEnvelopeSteps() {
  float attackSamples = SAMPLE_RATE_HZ * (s_attackMs / 1000.0f);
  float releaseSamples = SAMPLE_RATE_HZ * (s_releaseMs / 1000.0f);
  int32_t attackStep = (int32_t)lroundf((float)ENV_SCALE / attackSamples);
  int32_t releaseStep = (int32_t)lroundf((float)ENV_SCALE / releaseSamples);
  s_attackStep = (attackStep < 1) ? 1 : attackStep;
  s_releaseStep = (releaseStep < 1) ? 1 : releaseStep;
}

void IRAM_ATTR onAudioTimerISR() {
  // Envelop bijwerken (attack/release-ramp) - uitsluitend integer-rekenwerk.
  switch (s_envState) {
    case ENV_ATTACK:
      s_envLevel += s_attackStep;
      if (s_envLevel >= ENV_SCALE) {
        s_envLevel = ENV_SCALE;
        s_envState = ENV_SUSTAIN;
      }
      break;
    case ENV_RELEASE:
      s_envLevel -= s_releaseStep;
      if (s_envLevel <= 0) {
        s_envLevel = 0;
        s_envState = ENV_OFF;
      }
      break;
    case ENV_SUSTAIN:
    case ENV_OFF:
    default:
      break;
  }

  if (s_envState != ENV_OFF) {
    s_phaseAcc += s_phaseIncrement;
    uint8_t idx = (uint8_t)(s_phaseAcc >> 24); // bovenste 8 bits -> LUT-index 0..255
    int32_t sample = (int32_t)sineLUT[idx];              // -127..127
    int32_t withVolume = (sample * (int32_t)s_volume) / 100;    // integer, 0-100%
    int32_t withEnv = (withVolume * s_envLevel) >> 15;           // terugschalen (ENV_SCALE = 2^15)
    int val = 128 + (int)withEnv;
    if (val < 0) val = 0;
    if (val > 255) val = 255;
    dacWrite(DAC_PIN, (uint8_t)val);
  } else {
    dacWrite(DAC_PIN, 128); // stilte = midden van de DAC-range
  }
}

void audioInit() {
  buildSineLUT();
  pinMode(DAC_PIN, OUTPUT);

  recomputeEnvelopeSteps();

  // BELANGRIJK: dacWrite() hier al één keer aanroepen, in gewone (niet-ISR)
  // context, VOORDAT de 40kHz-timer/interrupt hieronder wordt gestart. De
  // arduino-esp32 DAC-driver initialiseert de DAC-peripheral lui: pas bij de
  // allereerste dacWrite()-aanroep voor een pin wordt intern een "oneshot
  // channel" aangemaakt (dac_oneshot_new_channel), en dat doet een
  // heap-allocatie en pakt interne locks - dingen die niet veilig zijn om
  // vanuit een interrupt te doen. Als de timer-ISR (die zelf ook dacWrite()
  // aanroept) toevallig de allereerste dacWrite()-aanroep ooit zou zijn, dan
  // gebeurt die onveilige eenmalige initialisatie binnen interrupt-context,
  // en dat gaf precies de gerapporteerde Guru Meditation crash
  // (LoadProhibited) - vrijwel direct nadat de timer voor het eerst afging.
  // Door dacWrite() hier alvast te doen, is de DAC-peripheral al klaar
  // voordat de ISR ooit draait, en is elke latere dacWrite()-aanroep vanuit
  // de ISR alleen nog een lichte registerschrijf-actie (wel IRAM/ISR-veilig).
  dacWrite(DAC_PIN, 128);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // arduino-esp32 core 3.x: timerBegin() neemt direct de tik-frequentie,
  // timerAlarm() zet de alarmwaarde (in ticks) + autoreload.
  audioTimer = timerBegin(TIMER_TICK_HZ);
  timerAttachInterrupt(audioTimer, &onAudioTimerISR);
  timerAlarm(audioTimer, TIMER_TICK_HZ / SAMPLE_RATE_HZ, true, 0);
#else
  // arduino-esp32 core 2.x: timerBegin(timernr, prescaler, countUp)
  // 80 MHz APB-klok / 80 = 1 MHz tik-frequentie.
  audioTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(audioTimer, &onAudioTimerISR, true);
  timerAlarmWrite(audioTimer, TIMER_TICK_HZ / SAMPLE_RATE_HZ, true);
  timerAlarmEnable(audioTimer);
#endif
}

void audioSetFrequency(uint16_t freqHz) {
  // DDS: increment = freq * 2^32 / samplerate
  double inc = (double)freqHz * 4294967296.0 / (double)SAMPLE_RATE_HZ;
  s_phaseIncrement = (uint32_t)inc;
}

void audioSetVolume(uint8_t volumePercent) {
  if (volumePercent > 100) volumePercent = 100;
  s_volume = volumePercent;
}

void audioSetAttackMs(uint16_t ms) {
  if (ms < ENV_ATTACK_MIN_MS) ms = ENV_ATTACK_MIN_MS;
  if (ms > ENV_ATTACK_MAX_MS) ms = ENV_ATTACK_MAX_MS;
  s_attackMs = (float)ms;
  recomputeEnvelopeSteps();
}

void audioSetReleaseMs(uint16_t ms) {
  if (ms < ENV_RELEASE_MIN_MS) ms = ENV_RELEASE_MIN_MS;
  if (ms > ENV_RELEASE_MAX_MS) ms = ENV_RELEASE_MAX_MS;
  s_releaseMs = (float)ms;
  recomputeEnvelopeSteps();
}

void audioToneOn() {
  // Start (of hervat) de attack-ramp. s_envLevel wordt bewust niet naar 0
  // teruggezet: bij een heel snelle re-trigger (zou bij deze morse-timing
  // niet moeten voorkomen, maar toch) klinkt een ramp vanaf het huidige
  // niveau natuurlijker dan een sprong.
  s_envState = ENV_ATTACK;
}

void audioToneOff() {
  s_envState = ENV_RELEASE;
}

void audioPause() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  timerStop(audioTimer);
#else
  timerAlarmDisable(audioTimer);
#endif
  s_envState = ENV_OFF;
  s_envLevel = 0;
  dacWrite(DAC_PIN, 128); // hier veilig: gewone (niet-ISR) context
}

void audioResume() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  timerStart(audioTimer);
#else
  timerAlarmEnable(audioTimer);
#endif
}
