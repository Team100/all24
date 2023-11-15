#ifndef DATA_H
#define DATA_H
#include <stdint.h>

/** 
 * Contains state suitable for USB HID input reports.
 *
 * Represents up to 8 16-bit axes (e.g. joysticks, dials), and 32 booleans (e.g. buttons).
 *
 * This layout must match the HID Report Descriptor in Transceiver.
 */
struct ReportTx {
  // signed axes, center is zero
  // see www.usb.org/sites/default/files/hut1_3_0.pdf section 4
  // DS gets confused if the last axis ("wheel") is added, so skip it.
  int16_t x : 16;
  int16_t y : 16;
  int16_t z : 16;
  int16_t rx : 16;
  int16_t ry : 16;
  int16_t rz : 16;
  int16_t slider : 16;
  int16_t dial : 16;
  bool b1 : 1;  // TODO start with b0
  bool b2 : 1;
  bool b3 : 1;
  bool b4 : 1;
  bool b5 : 1;
  bool b6 : 1;
  bool b7 : 1;
  bool b8 : 1;
  bool b9 : 1;
  bool b10 : 1;
  bool b11 : 1;
  bool b12 : 1;
  bool b13 : 1;
  bool b14 : 1;
  bool b15 : 1;
  bool b16 : 1;
  bool b17 : 1;
  bool b18 : 1;
  bool b19 : 1;
  bool b20 : 1;
  bool b21 : 1;
  bool b22 : 1;
  bool b23 : 1;
  bool b24 : 1;
  bool b25 : 1;
  bool b26 : 1;
  bool b27 : 1;
  bool b28 : 1;
  bool b29 : 1;
  bool b30 : 1;
  bool b31 : 1;
  bool b32 : 1;
  ReportTx() {
    memset(this, 0, sizeof(ReportTx));
  };
  /**
   * True if every bit is the same.
   *
   * Strict equality results in many extraneous reports.
   */
  bool operator==(const ReportTx& other) const {
    return memcmp(this, &other, sizeof(ReportTx)) == 0;
  }
  /**
   * True if buttons are the same and if axes are within the tolerance.
   */
  bool approxEquals(const ReportTx& other, int16_t tolerance) const {
    if (abs(this->x - other.x) > tolerance) return false;
    if (abs(this->y - other.y) > tolerance) return false;
    if (abs(this->z - other.z) > tolerance) return false;
    if (abs(this->rx - other.rx) > tolerance) return false;
    if (abs(this->ry - other.ry) > tolerance) return false;
    if (abs(this->rz - other.rz) > tolerance) return false;
    if (abs(this->slider - other.slider) > tolerance) return false;
    if (abs(this->dial - other.dial) > tolerance) return false;
    return memcmp(((char*)this) + 16, ((char*)&other) + 16, 4) == 0;
  }
};

/**
 * Contains state suitable for USB HID output reports.
 *
 * Represents the RoboRIO's 16 bits of output.  The RIO API implies that there are 32 bits, but
 * if you use a 32 bit HID report, the RIO produces nothing.  16 bits seems to work.
 */
struct ReportRx {
  bool i1 : 1;  // TODO: start with i0
  bool i2 : 1;
  bool i3 : 1;
  bool i4 : 1;
  bool i5 : 1;
  bool i6 : 1;
  bool i7 : 1;
  bool i8 : 1;
  bool i9 : 1;
  bool i10 : 1;
  bool i11 : 1;
  bool i12 : 1;
  bool i13 : 1;
  bool i14 : 1;
  bool i15 : 1;
  bool i16 : 1;
  ReportRx() {
    memset(this, 0, sizeof(ReportRx));
  };
  bool operator==(const ReportRx& other) const {
    return memcmp(this, &other, sizeof(ReportRx)) == 0;
  }
};

#endif  // DATA_H