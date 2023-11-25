#ifndef SENSOR_H
#define SENSOR_H
#include "Data.h"

#include <Arduino.h>

#include "Adafruit_seesaw.h"

#define SS_SWITCH 24

/**
 * Reads physical state
 *
 * See https://github.com/adafruit/Adafruit_Seesaw
 */
class Sensor {
public:
  /**
   * Start all the neokeys.
   *
   * TODO: do something in the case where this fails.
   */
  void initialize() {
    if (!initOne(encoder0, 0x36)) {
      return;
    }
    if (!initOne(encoder1, 0x37)) {
      return;
    }
    if (!initOne(encoder2, 0x38)) {
      return;
    }
    if (!initOne(encoder3, 0x39)) {
      return;
    }
    if (!initOne(encoder4, 0x3a)) {
      return;
    }
    initialized = true;
  }

  // TODO: make this a library function
  bool initOne(Adafruit_seesaw& encoder, uint8_t addr) {
    if (!encoder.begin(addr)) {
      return false;
    }
    encoder.pinMode(SS_SWITCH, INPUT_PULLUP);
    encoder.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
    encoder.enableEncoderInterrupt();
    return true;
  }

  /**
   * Reads all the encoders and writes the values into reportTx.
   */
  void sense(ReportTx& reportTx) {
    // this should be the first five buttons.
    reportTx.b1 = not encoder0.digitalRead(SS_SWITCH);
    reportTx.b2 = not encoder1.digitalRead(SS_SWITCH);
    reportTx.b3 = not encoder2.digitalRead(SS_SWITCH);
    reportTx.b4 = not encoder3.digitalRead(SS_SWITCH);
    reportTx.b5 = not encoder4.digitalRead(SS_SWITCH);

    // Note the axes are int16_t, the encoder is int32_t,
    // but overflow would require >1000 revolutions, will
    // never happen.
    // For now, expand (<< 8) these to make them easier to see.
    // this should be the first five axes.
    reportTx.x = encoder0.getEncoderPosition() << 8;
    reportTx.y = encoder1.getEncoderPosition() << 8;
    reportTx.z = encoder2.getEncoderPosition() << 8;
    reportTx.rx = encoder3.getEncoderPosition() << 8;
    reportTx.ry = encoder4.getEncoderPosition() << 8;
  }


private:
  Adafruit_seesaw encoder0;
  Adafruit_seesaw encoder1;
  Adafruit_seesaw encoder2;
  Adafruit_seesaw encoder3;
  Adafruit_seesaw encoder4;
  // TODO: do something with initialized (e.g. report an error if it's false)
  bool initialized{};
};
#endif  // SENSOR_H