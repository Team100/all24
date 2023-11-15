#ifndef INDICATOR_H
#define INDICATOR_H
#include "Data.h"
#include <Arduino.h>

// TODO: these are really neopixels
#define AT_GOAL_LED 11
#define STOP_LED 12
#define SLOW_LED 13
#define MED_LED 14
#define FAST_LED 15


/**
 * Expresses physical state: lights, sounds, etc.
 */
class Indicator {
public:
  /**
   * Interpretation of the RoboRIO outputs for this console.
   */
  typedef struct {
    bool atGoal : 1;
    uint8_t speed : 3;
    // extra bits below
    uint16_t extra: 12;
    // bool i5 : 1;
    // bool i6 : 1;
    // bool i7 : 1;
    // bool i8 : 1;
    // bool i9 : 1;
    // bool i10 : 1;
    // bool i11 : 1;
    // bool i12 : 1;
    // bool i13 : 1;
    // bool i14 : 1;
    // bool i15 : 1;
    // bool i16 : 1;
  } ArmOutputs;

  typedef union {
    Data::ReportRx report;
    ArmOutputs outputs;
  } ReportOutput;

  //Data& data_;
  ArmOutputs& armOutputs_;
  //  ArmOutputs previousArmOutputs_ = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  ArmOutputs previousArmOutputs_ = { 0, 0 };
  Indicator(Data::ReportRx& reportRx)
    //    : data_(data) {
    : armOutputs_(((ReportOutput&)reportRx).outputs) {
    pinMode(AT_GOAL_LED, OUTPUT);
    pinMode(STOP_LED, OUTPUT);
    pinMode(SLOW_LED, OUTPUT);
    pinMode(MED_LED, OUTPUT);
    pinMode(FAST_LED, OUTPUT);
  }

  /** 
   * Send newly changed outputs to the output devices.
   * TODO: actually do the I2C way.
   */
  void indicate() {
    if (armOutputs_.atGoal != previousArmOutputs_.atGoal) {
      previousArmOutputs_.atGoal = armOutputs_.atGoal;
      digitalWrite(AT_GOAL_LED, armOutputs_.atGoal);
    }
    if (armOutputs_.speed != previousArmOutputs_.speed) {
      previousArmOutputs_.speed = armOutputs_.speed;
      switch (armOutputs_.speed) {
        case 0:
          digitalWrite(STOP_LED, LOW);
          digitalWrite(SLOW_LED, LOW);
          digitalWrite(MED_LED, LOW);
          digitalWrite(FAST_LED, LOW);
          break;
        case 1:
          digitalWrite(STOP_LED, HIGH);
          digitalWrite(SLOW_LED, LOW);
          digitalWrite(MED_LED, LOW);
          digitalWrite(FAST_LED, LOW);
          break;
        case 2:
          digitalWrite(STOP_LED, LOW);
          digitalWrite(SLOW_LED, HIGH);
          digitalWrite(MED_LED, LOW);
          digitalWrite(FAST_LED, LOW);
          break;
        case 3:
          digitalWrite(STOP_LED, LOW);
          digitalWrite(SLOW_LED, LOW);
          digitalWrite(MED_LED, HIGH);
          digitalWrite(FAST_LED, LOW);
          break;
        case 4:
          digitalWrite(STOP_LED, LOW);
          digitalWrite(SLOW_LED, LOW);
          digitalWrite(MED_LED, LOW);
          digitalWrite(FAST_LED, HIGH);
          break;
        default:
          digitalWrite(STOP_LED, LOW);
          digitalWrite(SLOW_LED, LOW);
          digitalWrite(MED_LED, LOW);
          digitalWrite(FAST_LED, LOW);
          break;
      }
    }
  }
};

#endif  // INDICATOR_H