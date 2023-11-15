#ifndef SENSOR_H
#define SENSOR_H
#include "Data.h"
#include <Arduino.h>
//#include <Wire.h>
#include "Adafruit_seesaw.h"

#define SS_SWITCH 24

class Sensor {
public:
  void initialize() {
    if (!initOne(encoder0, 0x36)) {
      return;
    }
    if (!initOne(encoder1, 0x37)) {
      return;
    }
    initialized = true;
  }

  bool initOne(Adafruit_seesaw& encoder, uint8_t addr) {
    if (!encoder.begin(addr)) {
      return false;
    }
    encoder.pinMode(SS_SWITCH, INPUT_PULLUP);
    encoder.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
    encoder.enableEncoderInterrupt();
    return true;
  }

  void sense(ReportTx& reportTx) {
    reportTx.b5 = not encoder0.digitalRead(SS_SWITCH);
    reportTx.b6 = not encoder1.digitalRead(SS_SWITCH);
    reportTx.rz = encoder0.getEncoderPosition() << 10;      // reduce resolution so we can see it
    reportTx.slider = encoder1.getEncoderPosition() << 10;  // reduce resolution so we can see it
  }

  bool initialized{};
private:
  Adafruit_seesaw encoder0;
  Adafruit_seesaw encoder1;
};
#endif  // SENSOR_H