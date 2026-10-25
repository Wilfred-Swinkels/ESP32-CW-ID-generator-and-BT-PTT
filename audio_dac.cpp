#include "audio_dac.h"
#include <math.h>

#define DAC_PIN 25 // GPIO25 = DAC channel 1 on the classic ESP32

static const uint32_t SAMPLE_RATE_HZ = 40000; // 40 kHz sample rate for the sine generator
static const uint32_t TIMER_TICK_HZ = 1000000; // 1 MHz tick frequency for the hw timer

// Attack/release time of the envelope, in milliseconds - adjustable via
// audioSetAttackMs()/audioSetReleaseMs() (and therefore via the web page),
// clamped to the ranges named below. Note: at higher WPM speeds a dot gets
// fairly short (e.g. at 30 wpm, 1 dit = 40ms) - a release that (significantly)
// exceeds that won't let the tone fully die away before the next element
// starts. That's not a bug (the envelope state machine below just keeps
// running and glides smoothly onward), but it's something to keep in mind
// when choosing WPM and release together.
static const uint16_t ENV_ATTACK_MIN_MS = 1;
static const uint16_t ENV_ATTACK_MAX_MS = 20;
static const uint16_t ENV_RELEASE_MIN_MS = 1;
static const uint16_t ENV_RELEASE_MAX_MS = 50;
static volatile float s_attackMs = 1.0f;    // current attack time (default: 1ms, lower bound of 1-20ms)
static volatile float s_releaseMs = 10.0f;  // current release time (default: 10ms, within the 1-50ms range)

static hw_timer_t *audioTimer = nullptr;

// 256-point sine lookup table, values -127..+127 (full scale)
static int8_t sineLUT[256];

static volatile uint32_t s_phaseAcc = 0;       // 32-bit phase accumulator (DDS)
static volatile uint32_t s_phaseIncrement = 0; // derived from the desired frequency
static volatile uint8_t s_volume = 70; // 0-100 %

// Envelope state machine: ATTACK (ramping up), SUSTAIN (full volume, held),
// RELEASE (ramping down), OFF (silent). No separate "decay" level is needed
// because sustain here is always full volume (just an on/off-switched tone,
// not an instrument with its own decay stage).
//
// IMPORTANT: the envelope level and step sizes are deliberately fixed-point
// integers (int32_t), NOT floats. `onAudioTimerISR()` below runs as an
// interrupt, and doing floating-point arithmetic inside an interrupt is not
// safe on the ESP32 (Xtensa): the FPU context is only lazily saved/restored
// for regular tasks (via the coprocessor-exception handling that expects a
// task context), not for interrupt handlers. The first time the ISR actually
// executed floating-point instructions (as soon as the envelope first moved
// away from ENV_OFF, i.e. on the very first tone trigger) this produced
// exactly the reported Guru Meditation crash (LoadProhibited) - this was
// AFTER the earlier dacWrite() ordering fix, so a second, separate cause.
// All arithmetic in the ISR is therefore now done with integers; only the
// (non-ISR) `recomputeEnvelopeSteps()` below is still allowed to use floats,
// to convert the step sizes precisely into this fixed-point format.
enum EnvState { ENV_OFF, ENV_ATTACK, ENV_SUSTAIN, ENV_RELEASE };
static const int32_t ENV_SCALE = 32768; // envelope level 0..ENV_SCALE represents 0.0-1.0
static volatile EnvState s_envState = ENV_OFF;
static volatile int32_t s_envLevel = 0;   // current envelope level, 0..ENV_SCALE
static volatile int32_t s_attackStep = 0; // increment per sample during attack
static volatile int32_t s_releaseStep = 0; // decrement per sample during release

static void buildSineLUT() {
  for (int i = 0; i < 256; i++) {
    sineLUT[i] = (int8_t)lroundf(127.0f * sinf(2.0f * PI * (float)i / 256.0f));
  }
}

// Recomputes the per-sample envelope step size (in fixed-point units of
// ENV_SCALE) from the current s_attackMs/s_releaseMs. Called from
// audioInit() and whenever audioSetAttackMs()/audioSetReleaseMs() gets a
// new value - always runs in regular (non-ISR) context, so using floats
// here is not a problem.
static void recomputeEnvelopeSteps() {
  float attackSamples = SAMPLE_RATE_HZ * (s_attackMs / 1000.0f);
  float releaseSamples = SAMPLE_RATE_HZ * (s_releaseMs / 1000.0f);
  int32_t attackStep = (int32_t)lroundf((float)ENV_SCALE / attackSamples);
  int32_t releaseStep = (int32_t)lroundf((float)ENV_SCALE / releaseSamples);
  s_attackStep = (attackStep < 1) ? 1 : attackStep;
  s_releaseStep = (releaseStep < 1) ? 1 : releaseStep;
}

void IRAM_ATTR onAudioTimerISR() {
  // Update the envelope (attack/release ramp) - integer arithmetic only.
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
    uint8_t idx = (uint8_t)(s_phaseAcc >> 24); // top 8 bits -> LUT index 0..255
    int32_t sample = (int32_t)sineLUT[idx];              // -127..127
    int32_t withVolume = (sample * (int32_t)s_volume) / 100;    // integer, 0-100%
    int32_t withEnv = (withVolume * s_envLevel) >> 15;           // scale back down (ENV_SCALE = 2^15)
    int val = 128 + (int)withEnv;
    if (val < 0) val = 0;
    if (val > 255) val = 255;
    dacWrite(DAC_PIN, (uint8_t)val);
  } else {
    dacWrite(DAC_PIN, 128); // silence = middle of the DAC range
  }
}

void audioInit() {
  buildSineLUT();
  pinMode(DAC_PIN, OUTPUT);

  recomputeEnvelopeSteps();

  // IMPORTANT: call dacWrite() here once already, in regular (non-ISR)
  // context, BEFORE the 40kHz timer/interrupt below is started. The
  // arduino-esp32 DAC driver initializes the DAC peripheral lazily: only on
  // the very first dacWrite() call for a pin is a "oneshot channel" created
  // internally (dac_oneshot_new_channel), and that does a heap allocation
  // and takes internal locks - things that aren't safe to do from an
  // interrupt. If the timer ISR (which itself also calls dacWrite()) happened
  // to be the very first dacWrite() call ever, then that unsafe one-time
  // initialization would run inside interrupt context, and that produced
  // exactly the reported Guru Meditation crash (LoadProhibited) - almost
  // immediately after the timer fired for the first time. By doing
  // dacWrite() here ahead of time, the DAC peripheral is already set up
  // before the ISR ever runs, and every later dacWrite() call from the ISR
  // is then only a lightweight register write (which is IRAM/ISR-safe).
  dacWrite(DAC_PIN, 128);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // arduino-esp32 core 3.x: timerBegin() takes the tick frequency directly,
  // timerAlarm() sets the alarm value (in ticks) + autoreload.
  audioTimer = timerBegin(TIMER_TICK_HZ);
  timerAttachInterrupt(audioTimer, &onAudioTimerISR);
  timerAlarm(audioTimer, TIMER_TICK_HZ / SAMPLE_RATE_HZ, true, 0);
#else
  // arduino-esp32 core 2.x: timerBegin(timerNum, prescaler, countUp)
  // 80 MHz APB clock / 80 = 1 MHz tick frequency.
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
  // Start (or resume) the attack ramp. s_envLevel is deliberately not reset
  // to 0: on a very fast re-trigger (shouldn't happen with this morse
  // timing, but just in case), a ramp starting from the current level
  // sounds more natural than a jump.
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
  dacWrite(DAC_PIN, 128); // safe here: regular (non-ISR) context
}

void audioResume() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  timerStart(audioTimer);
#else
  timerAlarmEnable(audioTimer);
#endif
}
                                                                                        