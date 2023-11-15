#ifndef INDICATOR_H
#define INDICATOR_H
#include "Data.h"
#include <Arduino.h>

#define LED_1 13
#define LED_2 12
#define LED_3 11

/**
 * Expresses physical state: lights, sounds, etc.
 * TODO: more outputs
 */
class Indicator {
public:
  Data& data_;
  Indicator(Data& data)
    : data_(data) {
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);
  }

  /** 
   * Indicating reasserts everything.
   * TODO: only some of them
   */
  void indicate() {
    digitalWrite(LED_1, data_.reportRx_.i1);
    digitalWrite(LED_2, data_.reportRx_.i2);
    digitalWrite(LED_3, data_.reportRx_.i3);
  }
};

#endif  // INDICATOR_H