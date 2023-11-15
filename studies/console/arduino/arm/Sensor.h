#ifndef SENSOR_H
#define SENSOR_H
#include "Data.h"
#include <Arduino.h>
#include <Wire.h>

#define BUTTON_1 0
#define BUTTON_2 1

/**
 * Reads physical state: buttons, joysticks, etc. 
 * TODO: more channels
 */
class Sensor {
public:
  Data& data_;
  Sensor(Data& data)
    : data_(data) {
    pinMode(BUTTON_1, INPUT_PULLUP);
    pinMode(BUTTON_2, INPUT_PULLUP);
  }
  /** 
   * Updates state with inputs, return true if anything changed.
   */
  bool sense() {
    bool updated = false;

    bool readValue = !(bool)digitalRead(BUTTON_1);
    if (readValue != data_.reportTx_.b1) {
      data_.reportTx_.b1 = readValue;
      // fake an axis
      data_.reportTx_.x = -25000;
      data_.reportTx_.y = -20000;
      data_.reportTx_.z = -15000;
      data_.reportTx_.rx = -10000;
      data_.reportTx_.ry = -5000;
      data_.reportTx_.rz = 5000;
      data_.reportTx_.slider = 10000;
      data_.reportTx_.dial = 15000;
      updated = true;
    }

    readValue = !(bool)digitalRead(BUTTON_2);
    if (readValue != data_.reportTx_.b2) {
      data_.reportTx_.b2 = readValue;
      data_.reportTx_.x = 0;
      data_.reportTx_.y = 0;
      data_.reportTx_.z = 0;
      data_.reportTx_.rx = 0;
      data_.reportTx_.ry = 0;
      data_.reportTx_.rz = 0;
      data_.reportTx_.slider = 0;
      data_.reportTx_.dial = 0;
      updated = true;
    }
    return updated;
  }
};
#endif  // SENSOR_H