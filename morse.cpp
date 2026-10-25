#include "morse.h"

// Internationale morse-tabel: letters, cijfers en '/' (veel voorkomend in
// portable/mobiele roepnamen zoals "PA0ABC/P").
struct MorseEntry {
  char ch;
  const char *code;
};

static const MorseEntry MORSE_TABLE[] = {
    {'A', ".-"},    {'B', "-..."},  {'C', "-.-."},  {'D', "-.."},
    {'E', "."},     {'F', "..-."},  {'G', "--."},   {'H', "...."},
    {'I', ".."},    {'J', ".---"},  {'K', "-.-"},   {'L', ".-.."},
    {'M', "--"},    {'N', "-."},    {'O', "---"},   {'P', ".--."},
    {'Q', "--.-"},  {'R', ".-."},   {'S', "..."},   {'T', "-"},
    {'U', "..-"},   {'V', "...-"},  {'W', ".--"},   {'X', "-..-"},
    {'Y', "-.--"},  {'Z', "--.."},  {'0', "-----"}, {'1', ".----"},
    {'2', "..---"}, {'3', "...--"}, {'4', "....-"}, {'5', "....."},
    {'6', "-...."}, {'7', "--..."}, {'8', "---.."}, {'9', "----."},
    {'/', "-..-."},
};
static const size_t MORSE_TABLE_LEN = sizeof(MORSE_TABLE) / sizeof(MORSE_TABLE[0]);

static const char *lookupMorse(char c) {
  c = toupper(c);
  for (size_t i = 0; i < MORSE_TABLE_LEN; i++) {
    if (MORSE_TABLE[i].ch == c) return MORSE_TABLE[i].code;
  }
  return nullptr; // onbekend teken -> wordt overgeslagen
}

void MorsePlayer::begin(ToneFn toneOnFn, ToneFn toneOffFn) {
  _toneOn = toneOnFn;
  _toneOff = toneOffFn;
}

void MorsePlayer::setWpm(uint8_t wpm) {
  if (wpm < 5) wpm = 5;
  if (wpm > 30) wpm = 30;
  // PARIS-standaard: dot-lengte (T) in ms = 1200 / WPM
  _ditMs = 1200 / wpm;
}

void MorsePlayer::play(const char *text) {
  strncpy(_buffer, text, BUF_LEN - 1);
  _buffer[BUF_LEN - 1] = '\0';
  _text = _buffer;
  _playing = true;
  beginCharAt(0);
}

void MorsePlayer::stop() {
  _phase = PH_IDLE;
  _playing = false;
  if (_toneOff) _toneOff();
}

void MorsePlayer::finish() {
  _phase = PH_IDLE;
  _playing = false;
  if (_toneOff) _toneOff();
}

void MorsePlayer::beginCharAt(int idx) {
  _charIndex = idx;
  // eventuele spaties overslaan (zou hier normaal niet moeten voorkomen)
  while (_text[_charIndex] == ' ') _charIndex++;

  if (_text[_charIndex] == '\0') {
    finish();
    return;
  }

  _currentPattern = lookupMorse(_text[_charIndex]);
  _symbolIndex = 0;

  if (_currentPattern == nullptr || _currentPattern[0] == '\0') {
    // onbekend teken: direct doorschuiven naar het volgende teken
    beginCharAt(_charIndex + 1);
    return;
  }

  startSymbol();
}

void MorsePlayer::startSymbol() {
  char c = _currentPattern[_symbolIndex];
  _phaseDurationMs = (c == '-') ? (3UL * _ditMs) : (unsigned long)_ditMs;
  _phaseStartMs = millis();
  _phase = PH_SYMBOL_ON;
  if (_toneOn) _toneOn();
}

void MorsePlayer::update() {
  if (_phase == PH_IDLE) return;
  if ((millis() - _phaseStartMs) < _phaseDurationMs) return;

  switch (_phase) {
    case PH_SYMBOL_ON: {
      if (_toneOff) _toneOff();
      _symbolIndex++;
      if (_currentPattern[_symbolIndex] != '\0') {
        // volgende dot/dash van hetzelfde teken: 1 dit-pauze ertussen
        _phase = PH_SYMBOL_GAP;
        _phaseStartMs = millis();
        _phaseDurationMs = _ditMs;
      } else {
        // teken is klaar -> bepalen hoe lang de pauze tot het volgende teken is
        int nextIdx = _charIndex + 1;
        bool sawSpace = false;
        while (_text[nextIdx] == ' ') {
          nextIdx++;
          sawSpace = true;
        }
        if (_text[nextIdx] == '\0') {
          finish();
          return;
        }
        _pendingNextIndex = nextIdx;
        _phase = sawSpace ? PH_WORD_GAP : PH_CHAR_GAP;
        _phaseStartMs = millis();
        _phaseDurationMs = sawSpace ? (7UL * _ditMs) : (3UL * _ditMs);
      }
      break;
    }
    case PH_SYMBOL_GAP:
      startSymbol();
      break;
    case PH_CHAR_GAP:
    case PH_WORD_GAP:
      beginCharAt(_pendingNextIndex);
      break;
    default:
      break;
  }
}
