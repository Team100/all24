#include "Data.h"
#include "Transceiver.h"
#include <MIDI.h>

ReportRx reportRx_; // received data, could be used for MIDI OUT
ReportTx reportTx_; // data to transmit
Transceiver transceiver_(Transceiver::SubConsole::MIDI, reportRx_);

MIDI_CREATE_DEFAULT_INSTANCE();

// all the white keys from C2 to F6
void handleNote(byte pitch, bool on) {
  switch (pitch) {
    // pads
    case 36: // left-most pad
      reportTx_.b16 = on;
      break;
    case 37:
      reportTx_.b17 = on;
      break;
    case 38:
      reportTx_.b18 = on;
      break;
    case 39:
      reportTx_.b19 = on;
      break;
    case 40:
      reportTx_.b20 = on;
      break;
    case 41:
      reportTx_.b21 = on;
      break;
    case 42:
      reportTx_.b22 = on;
      break;
    case 43: // right-most pad
      reportTx_.b23 = on;
      break;

    // white keys
    case 48: // C3
      reportTx_.b1 = on;
      break;
    case 50: // D3
      reportTx_.b2 = on;
      break;
    case 52: // E3
      reportTx_.b3 = on;
      break;
    case 53: // F3
      reportTx_.b4 = on;
      break;
    case 55: // G3
      reportTx_.b5 = on;
      break;
    case 57: // A3
      reportTx_.b6 = on;
      break;
    case 59: // B3
      reportTx_.b7 = on;
      break;

    case 60: // C4
      reportTx_.b8 = on;
      break;
    case 62: // D4
      reportTx_.b9 = on;
      break;
    case 64: // E4
      reportTx_.b10 = on;
      break;
    case 65: // F4
      reportTx_.b11 = on;
      break;
    case 67: // G4
      reportTx_.b12 = on;
      break;
    case 69: // A4
      reportTx_.b13 = on;
      break;
    case 71: // B4
      reportTx_.b14 = on;
      break;

    case 72: // C5
      reportTx_.b15 = on;
      break;
  }
}

void handleNoteOn(byte channel, byte pitch, byte velocity) {
  // Serial.println(channel, DEC);
  // Serial.println(pitch, DEC);
  handleNote(pitch, true);
}

void handleNoteOff(byte channel, byte pitch, byte velocity) {
  handleNote(pitch, false);
}

void handleControlChange(byte channel, byte b1, byte b2) {
  // Serial.println(channel, DEC);
  // Serial.println(b1, DEC);
  // Serial.println(b2, DEC);
  switch (b1) {
    case 74: // knob 1
      reportTx_.x = (b2 << 9) - 32767;
      break;
    case 71: // knob 2
      reportTx_.y = (b2 << 9) - 32767;
      break;
    case 76: // knob 3
      reportTx_.z = (b2 << 9) - 32767;
      break;
    case 77: // knob 4
      reportTx_.slider = (b2 << 9) - 32767;
      break;
    case 82: // fader 1
      reportTx_.rx = (b2 << 9) - 32767;
      break;
    case 83: // fader 2
      reportTx_.ry = (b2 << 9) - 32767;
      break;
    case 85: // fader 1
      reportTx_.rz = (b2 << 9) - 32767;
      break;
    case 17: // fader 1
      reportTx_.dial = (b2 << 9) - 32767;
      break;
  }
}

void setup() {
  // Serial.println("setup");
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.setHandleControlChange(handleControlChange);
  MIDI.begin(MIDI_CHANNEL_OMNI);
}

void loop() {
  MIDI.read();
  transceiver_.send(reportTx_);
}
