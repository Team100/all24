#ifndef SENSOR_H
#define SENSOR_H
#include "Data.h"

#include <Arduino.h>
//#include <Wire.h>

#include "Adafruit_NeoKey_1x4.h"
#include "Adafruit_seesaw.h"
#include "seesaw_neopixel.h"

#include <SparkFun_ADS1015_Arduino_Library.h>

#include <ams_as5048b.h>

#define SS_SWITCH 24

// Joystick Offsets
//
// Using the supplied 3.3v vref, the signal from the joystick ranges from
// about 0.6v to about 2.7v, with the center around 1.65v.  To see the upper
// end we need to use gain setting PGA_1 which is +/-4.096 full range
// or 2mV LSB so the center should be about 825.  The offsets below are
// the observed values.  The range is set so that we get pretty close
// to full output range, even though the sensor doesn't come close
// to full voltage range.
static const int16_t xOffset = 815;
static const int16_t yOffset = 884;
static const int16_t range = 59;

// Rotary sensor offset
//
// It's just randomly oriented; I eyeballed it.
static const int16_t angleOffset = 25828;

/**
 * Reads physical state: buttons, joysticks, etc. 
 *
 * For the keys:
 * See https://www.adafruit.com/product/4980
 * See https://learn.adafruit.com/neokey-1x4-qt-i2c
 *
 * For the encoders:
 * See https://www.adafruit.com/product/4991
 * See https://learn.adafruit.com/adafruit-i2c-qt-rotary-encoder
 *
 * Both keys and encoders use this library:
 * See https://github.com/adafruit/Adafruit_Seesaw
 *
 * For the ADC:
 * See https://www.sparkfun.com/products/15334
 * See https://github.com/sparkfun/SparkFun_ADS1015_Arduino_Library
 *
 * For the magnetometer:
 * See https://ams.com/as5048badapterboard
 * See https://github.com/sosandroid/AMS_AS5048B
 */
class Sensor {
public:

  /**
   * Start all the i2c devices.
   *
   * TODO: do something in the case where this fails.
   */
  void initialize() {
    if (!neokey0.begin(0x30)) {
      return;
    }
    if (!initOne(encoder0, 0x36)) {
      return;
    }
    if (!initOne(encoder1, 0x37)) {
      return;
    }
    if (!initOne(encoder2, 0x38)) {
      return;
    }

    if (!adcSensor.begin(0x48)) {
      return;
    }
    // PGA_1 == +/-4.096 volts, which will contain the 0-3.3v signal.
    adcSensor.setGain(ADS1015_CONFIG_PGA_1);
    mysensor.begin();
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

  /**
   * Reads all the sensors and writes the values into reportTx.
   */
  void sense(ReportTx& reportTx) {
    uint32_t buttons0 = ~neokey0.digitalReadBulk(NEOKEY_1X4_BUTTONMASK);
    Keys k0 = *(Keys*)&buttons0;
    reportTx.b1 = k0.a;
    reportTx.b2 = k0.b;
    reportTx.b3 = k0.c;
    reportTx.b4 = k0.d;

    reportTx.b5 = not encoder0.digitalRead(SS_SWITCH);
    reportTx.b6 = not encoder1.digitalRead(SS_SWITCH);
    reportTx.b7 = not encoder2.digitalRead(SS_SWITCH);

    // Use "spare" axes for the encoders.
    // Note the axes are int16_t, the encoder is int32_t,
    // but overflow would require >1000 revolutions, will
    // never happen.
    // For now, expand (<< 8) these to make them easier to see.
    reportTx.ry = encoder0.getEncoderPosition() << 8;
    reportTx.slider = encoder1.getEncoderPosition() << 8;
    reportTx.dial = encoder2.getEncoderPosition() << 8;

    reportTx.x = adcSensor.getSingleEndedSigned(0);
    reportTx.y = adcSensor.getSingleEndedSigned(1);
    // Use offsets because the center isn't exactly the center
    reportTx.x -= xOffset;
    reportTx.y -= yOffset;
    // Multiply instead of shifting because the full range is mechanically limited
    reportTx.x *= range;
    reportTx.y *= range;

    // Shift 14b to 16b
    // this is ENU counterclockwise-positive.
    reportTx.rz = (mysensor.angleRegR() << 2) - angleOffset;
  }

  /**
   * Lights the keys as specified in ReportRx.
   *
   * The key mapping here matches the one in sense(), as a demo.
   * In general, these outputs might be more useful if they represented
   * more than just a keypress.
   */
  void indicate(ReportRx rpt) {
    if (rpt == prev) {  // speed up the common case
      return;
    }
    lite(neokey0, 0, 0x00ffff, rpt.i1, prev.i1);  // a
    lite(neokey0, 1, 0x00ffff, rpt.i2, prev.i2);  // b
    lite(neokey0, 2, 0x00ffff, rpt.i3, prev.i3);  // c
    lite(neokey0, 3, 0x00ffff, rpt.i4, prev.i4);  // d
    neokey0.pixels.show();
    prev = rpt;
  }

  /**
   * A little light show to show it's working.
   */
  void splash() {
    for (int i = 0; i < 4; i++) {
      neokey0.pixels.setPixelColor(i, 0xffffff);
      neokey0.pixels.show();
      delay(25);
    }
    for (int i = 0; i < 4; i++) {
      neokey0.pixels.setPixelColor(i, 0x000000);
      neokey0.pixels.show();
      delay(25);
    }
    for (int i = 0; i < 256; i += 16) {
      for (int k = 0; k < 4; ++k) {
        neokey0.pixels.setPixelColor(k, i, 0, 0);
      }
      neokey0.pixels.show();
      delay(25);
    }
    delay(750);
    for (int i = 0; i < 4; i++) {
      neokey0.pixels.setPixelColor(i, 0x000000);
    }
    neokey0.pixels.show();
  }
  bool initialized{};
private:
  Adafruit_NeoKey_1x4 neokey0;
  ReportRx prev;
  Adafruit_seesaw encoder0;
  Adafruit_seesaw encoder1;
  Adafruit_seesaw encoder2;
  ADS1015 adcSensor;
  AMS_AS5048B mysensor{ 0x40 };
  // TODO: do something with initialized (e.g. report an error if it's false)


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