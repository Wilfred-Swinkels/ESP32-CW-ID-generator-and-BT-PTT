#pragma once
#include <Arduino.h>

// Genereert een analoge sinustoon op de ingebouwde 8-bit DAC van de ESP32
// (GPIO25 = DAC1). Werkt alleen op de "klassieke" ESP32 (WROOM/WROVER) -
// de ESP32-S2/S3/C3/C6 hebben geen DAC aan boord.
//
// Techniek: een hardware-timer geeft elke sample een interrupt; daarin wordt
// een fase-accumulator (DDS) opgehoogd en via een sinus-lookup-table een
// sample naar de DAC geschreven. Zo blijft de toonopwekking volledig los
// lopen van de webserver/morse-logica in loop().

void audioInit();
void audioSetFrequency(uint16_t freqHz); // 400-1500 Hz (grenzen niet afgedwongen hier)
void audioSetVolume(uint8_t volumePercent); // 0-100 %, lineaire amplitude

// audioToneOn()/audioToneOff() schakelen niet meer abrupt aan/uit, maar
// starten een kleine attack/release-envelop (vloeiende op-/afbouw van het
// volume). Dat voorkomt de "klik" (key click) die je hoort/hoort-op-de-band
// bij hard aan/uit schakelen van een toon. Een echte ADSR heeft ook een
// decay/sustain-fase, maar die voegen voor een simpel aan/uit-geschakelde
// toon niets toe (de "sustain" is gewoon het volle volume zolang de toon
// aanstaat) - vandaar alleen attack + release.
void audioToneOn();  // start de toon (morse "key down"): vloeiende attack
void audioToneOff(); // stopt de toon (morse "key up"): vloeiende release, dan stil

// Attack-/release-tijd instelbaar maken (in ms), zodat dit via de webpagina
// aangepast kan worden i.p.v. vaste compile-time waardes. Wordt intern
// geklemd op resp. 1-20ms en 10-100ms (zie audio_dac.cpp) - dat zijn ook de
// grenzen die de webpagina/sliders gebruiken.
void audioSetAttackMs(uint16_t ms);
void audioSetReleaseMs(uint16_t ms);

// Pauzeert/hervat de sample-interrupt. Nodig rond flash/NVS-acties (bv. het
// opslaan van instellingen): tijdens een flash-schrijfactie wordt de
// instructiecache even uitgeschakeld, en de interrupt roept dacWrite() aan
// (een gewone, niet-IRAM functie) - zonder deze pauze zou dat in zeldzame
// gevallen tot een crash kunnen leiden. Opslaan gebeurt maar af en toe, dus
// de korte onderbreking van de toon is niet hoorbaar.
void audioPause();
void audioResume();
