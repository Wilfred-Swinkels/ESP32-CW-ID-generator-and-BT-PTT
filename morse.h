#pragma once
#include <Arduino.h>

// Niet-blokkerende morse-speler. play() start de uitzending van een tekst;
// update() moet elke loop()-iteratie aangeroepen worden en stuurt de
// dot/dash/gap-timing aan via millis() (geen delay()!), zodat de webserver
// intussen gewoon blijft reageren.
//
// De speler roept zelf alleen een "toon aan"/"toon uit" functie aan
// (bijvoorbeeld audioToneOn/audioToneOff); de eigenlijke toonopwekking zit
// in audio_dac.h/.cpp.

class MorsePlayer {
public:
  typedef void (*ToneFn)();

  void begin(ToneFn toneOnFn, ToneFn toneOffFn);
  void setWpm(uint8_t wpm);          // 5-30 WPM, herberekent de dot-lengte
  void play(const char *text);       // start met uitzenden (niet-blokkerend)
  void update();                     // elke loop() aanroepen
  void stop();                       // direct afbreken, toon uit
  bool isPlaying() const { return _playing; }
  // true tijdens een daadwerkelijke dot/dash (de toon staat aan), false
  // tijdens de pauzes ertussen (symbol/char/word-gap) en als er niets
  // speelt. Handig om een digitale uitgang de morse-code zelf te laten
  // "tekenen" (bv. voor een externe keying-uitgang), i.p.v. gewoon hoog te
  // blijven zolang isPlaying() true is.
  bool isKeyed() const { return _playing && _phase == PH_SYMBOL_ON; }

private:
  enum Phase { PH_IDLE, PH_SYMBOL_ON, PH_SYMBOL_GAP, PH_CHAR_GAP, PH_WORD_GAP };

  static const size_t BUF_LEN = 32;

  ToneFn _toneOn = nullptr;
  ToneFn _toneOff = nullptr;

  char _buffer[BUF_LEN];
  const char *_text = _buffer;
  int _charIndex = 0;
  int _pendingNextIndex = 0;

  const char *_currentPattern = nullptr;
  int _symbolIndex = 0;

  Phase _phase = PH_IDLE;
  unsigned long _phaseStartMs = 0;
  unsigned long _phaseDurationMs = 0;

  uint16_t _ditMs = 80; // 1200 / wpm

  bool _playing = false;

  void beginCharAt(int idx);
  void startSymbol();
  void finish();
};
