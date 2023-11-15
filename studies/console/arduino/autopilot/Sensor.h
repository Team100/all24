#ifndef SENSOR_H
#define SENSOR_H
#include "Data.h"
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_NeoKey_1x4.h"
// see https://github.com/sparkfun/SparkFun_TPA2016D2_Arduino_Library
#include <SparkFun_TPA2016D2_Arduino_Library.h>

class Sensor {
public:
  void initialize() {
    pinMode(RXLED, OUTPUT);
    pinMode(SOUNDPIN, OUTPUT);
    Wire.begin();
    delay(100);
    while (!neokey0.begin()) {
      delay(1000);
      Serial.println("neokey begin failed");
    }
    while (!amp.begin()) {
      delay(1000);
      Serial.println("amp begin failed");
    }
    amp.disableShutdown();
    amp.enableSpeakers();
    amp.disableLimiter();
    amp.disableNoiseGate();
    amp.writeAttack(0);
    amp.writeHold(0);
    amp.writeRelease(0);
    amp.writeFixedGain(0);
    amp.writeMaxGain(0);
    stop();
  }

  /**
   * Unpacks the uint32_t from digitalReadBulk without bit twiddling.  :-) 
   */
  struct Keys {
    uint8_t : 4;
    bool a : 1;
    bool b : 1;
    bool c : 1;
    bool d : 1;
    uint32_t : 24;
  };

  void sense(ReportTx& reportTx) {
    uint32_t buttons0 = ~neokey0.digitalReadBulk(NEOKEY_1X4_BUTTONMASK);
    Keys k0 = *(Keys*)&buttons0;
    reportTx.b1 = k0.a;
    reportTx.b2 = k0.b;
    reportTx.b3 = k0.c;
    reportTx.b4 = k0.d;
  }

  void indicate(ReportRx rpt) {
    if (rpt == prev) {  // speed up the common case
      return;
    }
    lite(neokey0, 0, 0x00ffff, rpt.i1, prev.i1);
    lite(neokey0, 1, 0x00ffff, rpt.i2, prev.i2);
    lite(neokey0, 2, 0x00ffff, rpt.i3, prev.i3);
    lite(neokey0, 3, 0x00ffff, rpt.i4, prev.i4);
    neokey0.pixels.show();

    // These pitches are from AIM-9, see https://youtu.be/QV4GStRN5UU
    if (rpt.i1) {  // looking for the target but can't see it
      stop();
      beep(500, 100);
    } else if (rpt.i2) {  // sees the target but not ok to shoot
      stop();
      steady(750);
    } else if (rpt.i3) {  // shoot now
      stop();
      steady(1500);
    } else { // silence
      stop();
    }
    prev = rpt;
  }

  void splash() {
    for (int i = 0; i < 4; i++) {
      neokey0.pixels.setPixelColor(i, 0xffffff);
      neokey0.pixels.show();
      delay(25);
    }
    delay(750);
    for (int i = 0; i < 4; i++) {
      neokey0.pixels.setPixelColor(i, 0x000000);
    }
    neokey0.pixels.show();
  }

  // needs to be called periodically to maintain the beeping
  void loop() {
    if (beep_duration_ms > 0) {
      uint32_t now_ms = millis();
      if (now_ms > beep_timeout_ms) {  // flip state
        if (beeping) {
          digitalWrite(RXLED, LOW);
          noTone(SOUNDPIN);
        } else {
          digitalWrite(RXLED, HIGH);
          tone(SOUNDPIN, beep_frequency_hz);
        }
        beep_timeout_ms = now_ms + beep_duration_ms;
        beeping ^= 1;
      }
    }
  }

private:
  static const uint8_t RXLED = 17;
  static const uint8_t SOUNDPIN = 9;
  TPA2016D2 amp;
  Adafruit_NeoKey_1x4 neokey0;
  uint32_t beep_timeout_ms;
  bool beeping;
  ReportRx prev;
  uint16_t beep_duration_ms;
  uint16_t beep_frequency_hz;

  void beep(uint16_t frequency_hz, uint16_t duration_ms) {
    Serial.println("beep");
    beep_duration_ms = duration_ms;
    beep_frequency_hz = frequency_hz;
  }

  void steady(uint16_t frequency_hz) {
    beep_timeout_ms = 0;
    beeping = false;
    tone(SOUNDPIN, frequency_hz);
  }

  void stop() {
    beep_duration_ms = 0;
    digitalWrite(RXLED, HIGH);
    noTone(SOUNDPIN);
    beep_timeout_ms = 0;
    beeping = false;
  }

  /**
   * Changes the state of one key, if the new state is different.
   */
  void lite(Adafruit_NeoKey_1x4& key, int i, uint32_t color, bool state, bool previousState) {
    if (state == previousState) {
      return;
    }
    key.pixels.setPixelColor(i, state ? color : 0x000000);
  }
};
#endif  // SENSOR_H